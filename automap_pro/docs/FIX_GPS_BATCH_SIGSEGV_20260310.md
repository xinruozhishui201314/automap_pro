# GPS 对齐后 SIGSEGV 崩溃修复说明（2026-03-10）

## Executive Summary

- **现象**：GPS 对齐成功后，`automap_system` 在 `IncrementalOptimizer::commitAndUpdate()` 内崩溃，GDB 显示 `free()` / `tbb::...::internal_clear()` / `gtsam::ISAM2::addVariables()`。
- **根因**：① `commitAndUpdate()` 中连续两次空 `isam2_.update()` 与 GTSAM 内部 TBB 并发清理叠加，易触发 double-free/use-after-free（与 borglab/gtsam#1189 同类）；② 异步模式下 GPS 批量为每个因子入队一个任务，导致短时间内大量 `update()` 调用，加剧竞态；③ 回调线程在 `waitForPendingTasks()` 后再调 `forceUpdate()`，与 opt 线程先后触碰 `isam2_`，增加时序风险。
- **修复**：去掉冗余空 `update()`、异步 GPS 批量改为单次 BATCH_UPDATE（一次 `commitAndUpdate()`）、回调侧不再在异步模式下调用 `forceUpdate()`。

## 1. 背景与日志

- 日志：`logs/run_20260310_173601/full.log`
- 触发时机：`[GPS_ALIGN] try_align` 成功 → `[GPS_BATCH] enter` → `waitForPendingTasks` 返回后，Thread 13（opt 线程）在 `commitAndUpdate()` 内 SIGSEGV。
- 调用栈要点：`free` ← `tbb::...::concurrent_unordered_base::internal_clear` ← `gtsam::ISAM2::addVariables` ← `gtsam::ISAM2::update` ← `IncrementalOptimizer::commitAndUpdate` ← `IncrementalOptimizer::optLoop`。

## 2. 根因简述

1. **三重 update**：原逻辑为 `isam2_.update(pending_graph_, pending_values_); isam2_.update(); isam2_.update();`。后两次空 `update()` 会触发 GTSAM 内部重线性化/缓存清理，与 TBB 并行任务叠加，存在已知的 double-free 类问题（见 GTSAM issue #1189）。
2. **单因子单任务**：异步时每个 GPS 因子一个 `GPS_FACTOR` 任务，opt 线程对每个任务执行一次 `commitAndUpdate()`，即大量连续 `update()`，放大上述竞态。
3. **回调再调 forceUpdate**：`waitForPendingTasks()` 只保证队列空，不保证 opt 线程已离开 `commitAndUpdate()`。回调线程随后调 `forceUpdate()`（内部再次 `commitAndUpdate()`），两线程先后操作 `isam2_`，增加不稳定因素。

## 3. 变更清单

| 文件 | 变更 |
|------|------|
| `automap_pro/src/backend/incremental_optimizer.cpp` | 去掉两次空 `isam2_.update()`；新增 `addGPSFactorsBatch()`，批量加因子后只调用一次 `commitAndUpdate()`。 |
| `automap_pro/include/automap_pro/backend/incremental_optimizer.h` | 新增 `GPSFactorItem` 与 `addGPSFactorsBatch()` 声明。 |
| `automap_pro/src/system/automap_system.cpp` | 异步 GPS 批量改为入队单次 `BATCH_UPDATE`（调用 `addGPSFactorsBatch()`），并移除该路径下 `waitForPendingTasks()` 之后的 `forceUpdate()`；增加 `#include <memory>`。 |

## 4. 编译与验证

- 在 automap_ws 中：`colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 使用与崩溃时相同的离线命令回放（如 M2DGR street_03），等待 GPS 对齐与批量因子添加，确认不再出现 SIGSEGV。
- 建议：同一 bag 多跑几次，或适当提高回放倍率，以复现原先负载。

## 5. 风险与回滚

- **风险**：去掉两次空 `update()` 可能略微改变重线性化节奏，若依赖其行为需再评估；批量单次提交后，单次优化时间可能略增。
- **回滚**：若需回滚，恢复 `incremental_optimizer.cpp` 中两次 `isam2_.update()`、恢复 `automap_system.cpp` 中异步路径的逐因子入队与 `forceUpdate()` 调用即可。

---

## 6. 补充修复：double free in NoiseModelFactor::linearize（2026-03-11）

- **现象**：GPS 对齐后 `commitAndUpdate` 内报 `double free or corruption (out)`，栈在 `free` ← `gtsam::NoiseModelFactor::linearize` ← `NonlinearFactorGraph::linearize` ← `ISAM2::update`（无 TBB 栈帧时仍可发生）。
- **根因**：① `ISAM2Params::cacheLinearizedFactors = true` 时线性化缓存路径可能触发 double free；② `Gaussian::Covariance(Matrix33)` 在 linearize 路径上部分 GTSAM 构建存在内存问题；③ 若 ISAM2 内部持有对传入 graph/values 的引用，随后对 pending_ 的 clear 可能加剧问题。
- **修复**：
  1. 构造中设置 `params.cacheLinearizedFactors = false`。
  2. **GPS 噪声改为 Diagonal**：所有 GPS 因子（`addGPSFactor`、`addGPSFactorsBatch`、optLoop 中 `GPS_FACTOR`）统一使用 `noiseModel::Diagonal::Variances(cov 对角线)`，不再使用 `Gaussian::Covariance(Matrix33)`。
  3. **commitAndUpdate**：先复制 `graph_copy`/`values_copy`，再 `isam2_.update(graph_copy, values_copy)`，避免 GTSAM 持有对 pending_ 的引用；update 前后打 `[ISAM2_DIAG]` 日志。
  4. **强化日志**：update 前打 `[ISAM2_DIAG] pre_update value_keys=[...]` 与 `factor_i keys=[...]`，崩溃时可根据最后一条 pre_update 对应到具体因子。
- **代价**：Diagonal 仅用对角线方差，略损失非对角信息；关闭缓存与传副本带来少量性能开销。

---

## 7. 根因修复：同一 key 的 Prior + GPS 同批 update 导致 double free（2026-03-11）

- **现象**：强化日志显示 `pending_factors=2 pending_values=1`，且 `factor_0 keys=[s1]`、`factor_1 keys=[s1]`（同一 key），崩溃仍在 `NoiseModelFactor::linearize`。
- **根因**：子图冻结时 `addSubMapNode` 向 pending 加入 **value + Prior**，未立即 commit；GPS 对齐回调中 `addBatchGPSFactors` 再向同一 pending 加入 **GPS(s1)**，导致同一次 `isam2_.update()` 内同时存在 Prior(s1) 与 GPS(s1)，GTSAM 在 linearize 该批时触发 double free（与 LIO-SAM 等做法一致：新节点/先验与 GPS 分步 update）。
- **修复**：
  1. **addBatchGPSFactors 前先 flush**：在添加任意 GPS 因子前，若 `hasPendingFactorsOrValues()` 为 true 则先 `forceUpdate()`，再添加 GPS 因子并 `forceUpdate()`，保证 never 同批 update 中混合 (value+prior) 与 GPS。
  2. 新增 `IncrementalOptimizer::hasPendingFactorsOrValues()`（读锁下判断 pending 非空）。
  3. **historical 阶段** 同样在添加历史 GPS 前检查并 flush pending。
- **日志**：`commitAndUpdate` 的 pre_update 中为每个因子打 `type=Prior|GPS|Between`，便于确认不再出现「同一批中 Prior+GPS 同 key」。

---

## 7.1. HBA GTSAM fallback：共享 between_noise 导致 double free（2026-03-11）

- **现象**：使用 GTSAM fallback 时，`LevenbergMarquardtOptimizer` 构造阶段报 `double free or corruption (out)`，栈在 `free` ← `gtsam::NoiseModelFactor::error` ← `NonlinearFactorGraph::error` ← `LevenbergMarquardtOptimizer` 构造。
- **根因**：HBA fallback 中所有 Between 因子共用同一个 `between_noise` shared_ptr，与「单节点 Prior + 共享 prior_noise_」同类（borglab/gtsam#1189）；LM 构造时对 graph 求 error 触发因子求值/析构路径上的 double free。
- **修复**（`automap_pro/src/backend/hba_optimizer.cpp`）：
  1. **Prior/Between 每因子独立噪声**：在循环内每次 `graph.add(PriorFactor/BetweenFactor(...))` 时新建 `noiseModel::Diagonal::Variances(...)`，不再在循环外创建单一 prior_noise/between_noise 复用。
  2. **GPS 安全**：对 `position_enu` 与 `covariance` 做 `allFinite()` 校验；方差对角线 clamp 到 `[1e-6, 1e6]`，避免异常协方差导致 GTSAM 内部异常。
  3. **日志**：图构建完成后打 `[HBA][GTSAM] graph built: factors=... values=... gps_factors=...`，失败时用 ALOG_ERROR 记录异常信息。

## 7.2. HBA GTSAM fallback 与后端 ISAM2 双路 GTSAM 崩溃（2026-03-11）

- **现象**：即使每因子独立噪声，LM 构造阶段仍报 `double free or corruption (out)`，栈同上（NoiseModelFactor::error → free）。
- **可能根因**：① 同一进程内 **IncrementalOptimizer（ISAM2）** 与 **HBA GTSAM fallback（LevenbergMarquardtOptimizer）** 共用 libgtsam，ISAM2 与 LM 交替使用可能触发 GTSAM 内部/静态状态问题（与 borglab/gtsam#1189 同类）；② 某类因子（Prior/Between/GPS）在 LM 的 `graph.error(initial)` 路径存在库内 bug。
- **增强日志与诊断**（`hba_optimizer.cpp`）：
  1. 建图时记录每个因子的类型：`[HBA][GTSAM] factor[idx] type=Prior(k0)|Between(k0-k1)|GPS(k2)`，崩溃时可根据 factor 数量对应到类型。
  2. LM 构造前打 `[HBA][GTSAM] LevenbergMarquardtOptimizer constructor enter (若崩溃在此后...)`，构造后打 `constructor exit`，便于确认崩溃在 LM 构造内。
  3. **可选逐因子 error 诊断**：设置环境变量 `AUTOMAP_HBA_FACTOR_ERROR_DIAG=1` 后，在构造 LM 前对每个因子调用 `graph[i]->error(initial)` 并打 `factor_error idx=N type=...`；若崩溃发生在该循环，最后一条 `idx=N` 即肇事因子索引。
- **隔离验证**：临时关闭 HBA 触发或关闭 HBA 的 GPS，仅用 ISAM2+GPS，或反之仅用 HBA fallback，观察崩溃是否消失，以区分「双路共用导致」与「单路 LM 内某因子导致」。详见 `GTSAM_MULTI_USE_AND_LOGGING.md` 第 5 节。

## 7.3. HBA GTSAM fallback：Eigen 临时量 + 未传副本导致 LM 构造 double free（2026-03-11）

- **现象**：与 7.2 相同，LM 构造阶段 `double free or corruption (out)`，栈在 `free` ← `NoiseModelFactor::error` ← `NonlinearFactorGraph::error` ← `LevenbergMarquardtOptimizer` 构造。
- **根因**：① **Eigen 临时量**：Prior/Between 使用 `(gtsam::Vector(6) << ...).finished()` 作为临时传入 `noiseModel::Diagonal::Variances()`，若 GTSAM 内部某路径保存对传入向量的引用而非拷贝，则临时析构后悬垂引用，在 LM 构造内 `graph.error(initial)` 时触发未定义行为（表现为 double free）；② **LM 持引用**：直接传 `graph`/`initial` 给 LM 构造，与 ISAM2 双路共用 libgtsam 时，LM 内部若持有对栈上 graph/initial 的引用，生命周期与析构顺序易触发问题。
- **修复**（`automap_pro/src/backend/hba_optimizer.cpp`）：
  1. **命名方差向量**：用 `gtsam::Vector6 prior_var6` / `between_var6` 在栈上构造并填入数值，再 `Variances(prior_var6)` / `Variances(between_var6)`，避免将 Eigen 临时量传入 GTSAM。
  2. **传副本给 LM**：构造 `graph_copy(graph)`、`initial_copy(initial)`，用 `LevenbergMarquardtOptimizer opt(graph_copy, initial_copy)`，与 `commitAndUpdate` 中 ISAM2 的 graph_copy/values_copy 策略一致，避免 LM 内部持对原对象的引用。
  3. **key 一致性检查**：LM 构造前检查所有因子的 key 均在 `initial_copy` 中，若缺失则跳过优化并打 WARN，避免 key 缺失导致未定义行为。
  4. **日志增强**：pre-LM 打 `graph_copy.size`、`initial_copy.size`、`initial_keys_sample=[...]`；unknown 异常时提示查阅 7.1/7.2 与 pre-LM 日志。

---

## 8. 根因：单节点 Prior-only 与共享 prior_noise_ 导致 linearize double free（2026-03-11）

- **现象**：flush 时崩溃，日志为 `pending_factors=1 pending_values=1`、`factor_0 type=Prior`，即 **仅 1 个 Prior 因子 + 1 个 value** 时在 `NoiseModelFactor::linearize` 内 double free。
- **根因**：① GTSAM 在「首次 update 仅 1 value + 1 Prior」路径存在已知 double free（见 borglab/gtsam#1189）；② 所有 Prior 共用同一 `prior_noise_` shared_ptr，linearize 时可能触发共享噪声模型相关内存问题。
- **修复**：
  1. **Prior 噪声独立**：`addSubMapNode` 中每次新建 `noiseModel::Diagonal::Variances(prior_var6_)` 作为 Prior 因子噪声，不再共用 `prior_noise_`；保留 `prior_noise_` 仅做 shutdown 检查。
  2. **推迟单节点 Prior 提交**：`commitAndUpdate` 中若检测到「仅 1 value + 1 Prior」，则 **不调用** `isam2_.update()`，直接返回并保留 pending，待第二节点 + odom 加入后一起提交，避免触发 GTSAM 单节点首帧 update 的 bug 路径。
  3. **日志**：打 `[ISAM2_DIAG] defer single-node Prior update` 与 `[ISAM2_DIAG] first isam2 update ... [FIRST_UPDATE]` 便于确认首帧与推迟行为。
