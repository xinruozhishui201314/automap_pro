# GTSAM Double Free 崩溃报告与修复方案

## Executive Summary

| 项目 | 说明 |
|------|------|
| **崩溃类型** | `double free or corruption (out)` → `SIGABRT` |
| **崩溃位置** | `gtsam::NoiseModelFactor::linearize()` → `free()` |
| **根本原因** | GTSAM TBB 并行竞态 (borglab/gtsam#1189) |
| **触发条件** | ISAM2 首次 update（11 factors + 5 values） |
| **修复策略** | 重新编译 GTSAM 禁用 TBB 并行化 |

---

## 1. 崩溃详情

### 1.1 崩溃调用栈

```
double free or corruption (out)

Thread 1 "automap_system_" received signal SIGABRT, Aborted.
#0  0x00007ffff798f9fc in pthread_kill () from /lib/x86_64-linux-gnu/libc.so.6
#1  0x00007ffff793b476 in raise () from /lib/x86_64-linux-gnu/libc.so.6
#2  0x00007ffff79217f3 in abort () from /lib/x86_64-linux-gnu/libc.so.6
...
#6  0x00007ffff799e453 in free () from /lib/x86_64-linux-gnu/libc.so.6
#7  0x00007ffff4e31eb0 in gtsam::NoiseModelFactor::linearize(gtsam::Values const&) const
#8  0x00007ffff4e36b4b in gtsam::NonlinearFactorGraph::linearize(gtsam::Values const&) const
#9  0x00007ffff4e028dd in gtsam::ISAM2::update(...)
#10 0x00007ffff4dfb711 in gtsam::ISAM2::update(...)
#11 automap_pro::IncrementalOptimizer::commitAndUpdate()
#12 automap_pro::IncrementalOptimizer::forceUpdate()
#13 automap_pro::AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA()
```

### 1.2 崩溃前日志状态

- `[ISAM2_DIAG] first isam2 update (current_estimate was empty) factors=11 values=5`
- `[ISAM2_DIAG] update call enter (graph_copy.size=11 values_copy.size=5) [FIRST_UPDATE]`
- 随后出现 `double free or corruption (out)`

触发场景多为：**finish_mapping** 时调用 `ensureBackendCompletedAndFlushBeforeHBA()` → `forceUpdate()` → `commitAndUpdate()`，此时若 ISAM2 为首次 update（或批量因子首次提交），GTSAM 内部并行 linearize 会触发该 bug。

---

## 2. 根因分析

### 2.1 问题本质

- **TBB 并行竞态**：GTSAM 使用 Intel TBB 进行因子图并行线性化。
- **内存管理缺陷**：在 `NoiseModelFactor::linearize()` 路径下，TBB 并行任务导致同一块内存被重复释放。
- **触发条件**：首次 `ISAM2::update()` 或单次 update 中因子数较多时的批量线性化。

详见上游 issue: [borglab/gtsam#1189](https://github.com/borglab/gtsam/issues/1189)。

### 2.2 现有防护措施（已部署但不足）

| 措施 | 代码位置 | 效果 |
|------|----------|------|
| `cacheLinearizedFactors = false` | incremental_optimizer.cpp L38 | 部分有效 |
| `evaluateNonlinearError = false` | incremental_optimizer.cpp L40 | 部分有效 |
| 全局互斥锁 `GtsamCallScope` | gtsam_guard.cpp | 无效（TBB 内部不受控） |
| `TBB_NUM_THREADS=1` | gtsam_guard.cpp / 运行脚本 | 无效（竞态仍存在） |
| `Eigen::setNbThreads(1)` | gtsam_guard.cpp | 无效（不控制 TBB） |
| 单节点延迟更新 | commitAndUpdate 内 DEFER 逻辑 | 部分有效 |

**结论**：仅靠运行时限制无法完全消除 GTSAM 内部 TBB 竞态，需在**编译 GTSAM 时禁用 TBB**。

---

## 3. 修复方案

### 方案 A：重新编译 GTSAM 禁用 TBB（推荐）

- **Docker 镜像**：已修改 [docker/dockerfile](../docker/dockerfile)，GTSAM 构建使用 `-DGTSAM_WITH_TBB=OFF`。重新构建镜像后即可生效。
- **已有容器**：若镜像内 GTSAM 仍带 TBB，可在容器内重新编译 GTSAM 并关闭 TBB，然后重新编译 automap_pro。参见 `scripts/build_gtsam_no_tbb.sh`，或按下列步骤手动执行：

  ```bash
  # 容器内示例（GTSAM 源码与安装路径按实际调整）
  cd /tmp/gtsam && mkdir -p build_notbb && cd build_notbb
  cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF
  make -j$(nproc) && make install && ldconfig
  # 然后重新编译 automap_pro
  cd /root/automap_ws && colcon build --packages-select automap_pro --cmake-clean-cache
  ```

### 方案 B：代码侧加固（GTSAM 已关 TBB 时仍可做）

在 **GTSAM 已关闭 TBB** 的前提下，若仍出现 double free，可在应用侧做两处加固：

1. **Between 因子改用 Diagonal 噪声**  
   `Gaussian::Covariance` 在部分 GTSAM 版本的 `NoiseModelFactor::linearize()` 路径存在释放问题。  
   已做：`addOdomFactor` / `addLoopFactor` / 异步回环任务中，Between 因子统一使用 `infoToNoiseDiagonal()`（由信息矩阵对角线构造 `Diagonal::Variances`），不再使用 `infoToNoise()` 的满协方差。

2. **首次 update 拆成两段**  
   当 `current_estimate_.empty()` 且 `pending_values_.size() > 2` 时，将第一次 ISAM2 update 拆成两次：  
   先只提交前 2 个 key 及其相关因子并 `update`，再提交剩余 key 与因子并再次 `update`。  
   这样首更只涉及 2 个节点，降低首更路径上的复杂度，减轻 linearize 内 double free 风险。

参见：`IncrementalOptimizer::commitAndUpdate()`、`infoToNoiseDiagonal()`、`addOdomFactor` / `addLoopFactor`。

### 方案 C：临时规避（配置 + 环境变量）

- **配置**：[automap_pro/config/system_config.yaml](../config/system_config.yaml) 中 `backend.isam2` 已做临时规避：
  - `relinearize_skip: 10`
  - `enable_relinearization: false`
- **环境变量**：运行前设置（`run_automap.sh` 已自动注入，方案 C）：
  - `TBB_NUM_THREADS=1`
  - `OMP_NUM_THREADS=1`
  - `AUTOMAP_GTSAM_SERIAL=1`

上述规避可降低触发概率，但无法从根本消除问题，长期仍需采用方案 A。

---

## 4. 验证计划

1. **编译验证**：确认 GTSAM 未链接 TBB：  
   `ldd /usr/local/lib/libgtsam.so | grep tbb` 应无输出；或运行 `scripts/verify_gtsam_no_tbb.sh`。
2. **运行验证**：重新跑离线/在线建图至 finish_mapping，确认无 `double free` 及 SIGABRT。
3. **回归**：对比建图轨迹与点云，确认精度与性能无明显下降。

## 5. 精准定位：TRACE / CRASH_TRACE 日志

崩溃或异常时，用以下方式精确定位到步骤：

1. **看最后一条 TRACE 步骤**  
   日志中带 `[ISAM2_DIAG][TRACE] step=` 或 `[AutoMapSystem][HBA][TRACE] step=` 的行表示已执行到的步骤；**崩溃发生在“最后一条 step= 与下一条 step= 之间”**（若没有下一条，则崩溃在该 step 之后的 GTSAM 调用内）。

2. **常用 step 含义**  
   - `commitAndUpdate_enter`：进入 commitAndUpdate，后续会进入 GTSAM 调用。
   - `isam2_update_invoke`：即将调用 `isam2_.update()`；若此后无 `isam2_update_returned`，则崩溃在 **isam2_.update / linearize** 内。
   - `split_batch1_update_invoke` / `split_batch2_update_invoke`：首次拆分时的两段 update；同样用对应的 `_returned` 判断是否崩溃在 update 内。
   - `forceUpdate_enter` / `forceUpdate_exit`：由 ensureBackendCompletedAndFlushBeforeHBA 或 finish_mapping 触发的 forceUpdate；若只有 enter 无 exit，则崩溃在 **forceUpdate → commitAndUpdate** 内。
   - `ensureBackendCompletedAndFlushBeforeHBA_enter` → `waitForPendingTasks_done` → （若有 pending）`forceUpdate_...` → `forceUpdate_after_flush_done`：可判断崩溃是在等待阶段还是 forceUpdate 阶段。

3. **检索命令示例**  
   ```bash
   grep -E "CRASH_TRACE|step=" full.log | tail -30
   grep "ISAM2_DIAG" full.log | tail -50
   ```

---

## 6. 相关文档

- [HBA_GTSAM_FALLBACK_DOUBLE_FREE_FIX.md](HBA_GTSAM_FALLBACK_DOUBLE_FREE_FIX.md) — HBA 使用 GTSAM fallback 时的 double free 与隔离策略
- [BACKEND_BEFORE_HBA_AND_DOUBLE_FREE.md](BACKEND_BEFORE_HBA_AND_DOUBLE_FREE.md) — 后端与 HBA 调用顺序
- [GTSAM_MULTI_USE_AND_LOGGING.md](GTSAM_MULTI_USE_AND_LOGGING.md) — GTSAM 多路使用与日志
