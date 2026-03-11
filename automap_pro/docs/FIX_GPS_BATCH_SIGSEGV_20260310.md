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

## 8. 根因：单节点 Prior-only 与共享 prior_noise_ 导致 linearize double free（2026-03-11）

- **现象**：flush 时崩溃，日志为 `pending_factors=1 pending_values=1`、`factor_0 type=Prior`，即 **仅 1 个 Prior 因子 + 1 个 value** 时在 `NoiseModelFactor::linearize` 内 double free。
- **根因**：① GTSAM 在「首次 update 仅 1 value + 1 Prior」路径存在已知 double free（见 borglab/gtsam#1189）；② 所有 Prior 共用同一 `prior_noise_` shared_ptr，linearize 时可能触发共享噪声模型相关内存问题。
- **修复**：
  1. **Prior 噪声独立**：`addSubMapNode` 中每次新建 `noiseModel::Diagonal::Variances(prior_var6_)` 作为 Prior 因子噪声，不再共用 `prior_noise_`；保留 `prior_noise_` 仅做 shutdown 检查。
  2. **推迟单节点 Prior 提交**：`commitAndUpdate` 中若检测到「仅 1 value + 1 Prior」，则 **不调用** `isam2_.update()`，直接返回并保留 pending，待第二节点 + odom 加入后一起提交，避免触发 GTSAM 单节点首帧 update 的 bug 路径。
  3. **日志**：打 `[ISAM2_DIAG] defer single-node Prior update` 与 `[ISAM2_DIAG] first isam2 update ... [FIRST_UPDATE]` 便于确认首帧与推迟行为。
