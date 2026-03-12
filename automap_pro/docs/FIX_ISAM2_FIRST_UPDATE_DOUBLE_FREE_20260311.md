# ISAM2 首次 Update Double Free 修复文档

## 0) Executive Summary

| 项目 | 内容 |
|------|------|
| **崩溃类型** | `double free or corruption (out)` → SIGABRT |
| **触发场景** | `finish_mapping` 服务调用时，HBA 前需要 `forceUpdate()` flush pending factors |
| **调用栈** | `finish_mapping` → `ensureBackendCompletedAndFlushBeforeHBA` → `forceUpdate` → `commitAndUpdate` → `LevenbergMarquardtOptimizer` 构造函数 → `NoiseModelFactor::error()` → `free()` |
| **根本原因** | GTSAM 的 `NoiseModelFactor::error()` 在特定条件下存在 double free bug（borglab/gtsam#1189）。**不仅是 ISAM2，LM 优化器也会触发**，因为 LM 构造函数会调用 `NonlinearFactorGraph::error()` 计算初始误差。 |
| **修复方案** | **V5 完全跳过优化**：首次 update 仅注入 values，保留 factors 到下次增量 update 处理。不调用任何 GTSAM 优化器。 |
| **影响范围** | 仅影响 ISAM2 首次 update 逻辑，对已有优化结果无影响 |
| **风险等级** | 低 - 仅改变首次提交逻辑，最终优化结果一致 |

| 版本 | 日期 | 说明 |
|------|------|------|
| V1-V3 | 2026-03-11 | 三阶段拆分，仍在 phase1 linearize 崩溃 |
| V4 | 2026-03-12 | LM_then_ISAM2 方案，在 LM 构造函数崩溃 |
| **V5** | 2026-03-12 | 完全跳过首次优化，仅注入 values，保留 factors 到下次处理 |
| **V5.1** | 2026-03-12 | 移除 V5 路径中 FACTOR_DETAIL/VALUE_DETAIL/MEM_TRACE：访问 GPSFactor 的 measurementIn/noiseModel/sigmas 会触发 SIGSEGV in free()，故首次 update 不再访问因子内部 |

---

## 1) 崩溃现象

### 1.1 触发条件

```bash
# 触发命令
ros2 service call /automap/finish_mapping std_srvs/srv/Trigger '{}'
```

### 1.2 V4 崩溃日志（LM_then_ISAM2）

```text
09:49:58 [ISAM2_DIAG] first isam2 update (current_estimate was empty) factors=11 values=5
09:49:58 [ISAM2_DIAG] first update path=LM_then_ISAM2 (bypass ISAM2 first-update double free)
09:49:58 [CRASH_CONTEXT] step=first_update_lm_enter factors=11 values=5
09:49:58 [ISAM2_DIAG][TRACE] step=first_update_lm_constructor_enter
09:49:58 [CRASH_CONTEXT] step=first_update_lm_pre_optimize (若崩溃则发生在 LM 构造或 optimize 内)
09:49:58 double free or corruption (out)  ← 崩溃点！

Thread 1 "automap_system_" received signal SIGABRT, Aborted.
#7  gtsam::NoiseModelFactor::error(gtsam::Values const&) const
#8  gtsam::NonlinearFactorGraph::error(gtsam::Values const&) const
#9  gtsam::LevenbergMarquardtOptimizer::LevenbergMarquardtOptimizer(...)
#10 automap_pro::IncrementalOptimizer::commitAndUpdate()
#11 automap_pro::IncrementalOptimizer::forceUpdate()
#12 automap_pro::AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA()
```

**关键发现**：崩溃发生在 `LevenbergMarquardtOptimizer` 构造函数内部，而非 ISAM2！
原因是 LM 构造函数会调用 `NonlinearFactorGraph::error()` 计算初始误差，
这也会触发 `NoiseModelFactor::error()` 的 double free bug。

---

## 2) 根本原因分析

### 2.1 GTSAM 已知 Bug

这是 GTSAM 的已知问题，与 [borglab/gtsam#1189](https://github.com/borglab/gtsam/issues/1189) 相关：

- **触发条件**：首次对空 estimate 调用 `NoiseModelFactor::error()` 或 `linearize()` 时会崩溃
- **崩溃点**：`NoiseModelFactor::linearize()` 或 `NoiseModelFactor::error()` 内部
- **内存问题**：多次释放同一块内存

### 2.2 V4 方案的缺陷

V4 使用 LM_then_ISAM2 三阶段方案试图绕过 ISAM2 的 double free：

```cpp
// Phase1: LM 批量优化
gtsam::LevenbergMarquardtOptimizer lm_opt(graph_copy, values_copy, lm_params);
gtsam::Values lm_result = lm_opt.optimize();  // 崩溃在构造函数！
```

**但问题是**：LM 构造函数内部会调用 `NonlinearFactorGraph::error()` 计算初始误差，这也会触发 `NoiseModelFactor::error()` 的 double free。

---

## 3) 修复方案 (V5)

### 3.1 策略：完全跳过首次优化

**首次 update 不调用任何 GTSAM 优化器**：

| Step | 调用 | 说明 |
|------|------|------|
| Step1 | `isam2_.update(EmptyGraph, values_copy)` | 仅注入节点，无因子 → **不调用任何 linearize/error** |
| Step2 | 保留 factors 在 pending_graph_ | 不清空，等第二次增量 update 处理 |
| Step3 | `current_estimate_ = values_copy` | 直接用初始值作为当前估计 |

### 3.2 代码修改（V5）

```cpp
// 文件：src/backend/incremental_optimizer.cpp
// 函数：IncrementalOptimizer::commitAndUpdate()

if (is_first_update) {
    // V5: 完全跳过优化，仅注入 values
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[ISAM2_DIAG][V5] first update path=SKIP_OPTIMIZATION");

    // Step1: 仅注入 values，空 graph（不触发 linearize）
    gtsam::NonlinearFactorGraph empty_graph;
    isam2_.update(empty_graph, values_copy);

    // Step2: 不清空 pending_graph_，保留所有 factors 到下次 update
    // （清空逻辑在函数末尾根据 used_first_update_three_phase 跳过）

    // Step3: 直接用初始值作为当前估计（不做优化）
    current_estimate_ = values_copy;
}
```

### 3.3 增强日志

V5 增加了详细的诊断日志：

```cpp
// 因子详细信息
[ISAM2_DIAG][FACTOR_DETAIL] idx=0 type=Prior keys=[...] prior_xyz=[...]
[ISAM2_DIAG][FACTOR_DETAIL] idx=1 type=GPS keys=[...] gps_xyz=[...] noise_sigmas=[...]

// Values 详细信息
[ISAM2_DIAG][VALUE_DETAIL] key=... xyz=[...] rpy=[...]

// 内存地址追踪
[ISAM2_DIAG][MEM_TRACE] graph_ptr=0x... values_ptr=0x... factors_size=...
```

### 3.4 V5.1：移除首次 update 的因子/值详细日志（避免 SIGSEGV in free）

V5 上线后仍出现崩溃：崩溃发生在 **FACTOR_DETAIL 循环**内，在打印完 idx=1 (GPS) 后触发 `SIGSEGV in free()`。根因是对 GPSFactor 调用 `measurementIn()`、`noiseModel()`、`noise->sigmas()` 时，会走 GTSAM 内部与 NoiseModel 相关的路径，在部分环境下导致非法/二次 free。修复：在 V5 首次 update 分支中**删除** FACTOR_DETAIL、VALUE_DETAIL、MEM_TRACE 整块，不访问 `prior()`、`measurementIn()`、`noiseModel()`、`measured()` 等因子内部数据；仅保留 CRASH_CONTEXT/TRACE 与 Step1–Step3 逻辑。

### 3.5 修复后的预期日志（V5.1）

```text
[ISAM2_DIAG][V5] first update path=SKIP_OPTIMIZATION (bypass all GTSAM linearize)
[CRASH_CONTEXT] step=first_update_v5_enter factors=11 values=5
[ISAM2_DIAG][TRACE] step=first_update_v5_pre_update_values
[CRASH_CONTEXT] step=first_update_v5_pre_update_values nodes=5
[ISAM2_DIAG][TRACE] step=first_update_v5_post_update_values
[CRASH_CONTEXT] step=first_update_v5_post_update_values
[ISAM2_DIAG][V5] first update done: injected 5 values, deferred 11 factors to next update
[CRASH_CONTEXT] step=first_update_v5_done deferred_factors=11
[ISAM2_DIAG][V5] pending_graph_ NOT cleared, will be processed in next incremental update
```

---

## 4) 变更清单

| 文件 | 修改类型 | 说明 |
|------|----------|------|
| `src/backend/incremental_optimizer.cpp` | 逻辑修改 | V5：首次 update 跳过优化，仅注入 values；增强日志 |
| `docs/FIX_ISAM2_FIRST_UPDATE_DOUBLE_FREE_20260311.md` | 更新 | 本文档 |

---

## 5) 验证计划

### 5.1 回归测试

```bash
# 1. 编译
cd /home/wqs/Documents/github/automap_pro/automap_ws
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 2. 运行离线建图
ros2 launch automap_pro offline_mapping_launch.py config:=... bag:=...

# 3. 触发 finish_mapping
ros2 service call /automap/finish_mapping std_srvs/srv/Trigger '{}'

# 4. 检查日志
grep -E "V5|SKIP_OPTIMIZATION|double free|SIGABRT" logs/run_xxx/full.log
```

### 5.2 预期结果

1. **无崩溃**：不再出现 `double free or corruption` 或 `SIGABRT`
2. **V5 日志**：看到 `path=SKIP_OPTIMIZATION` → `first_update_v5_done deferred_factors=11`
3. **第二次 update**：看到增量 update 处理 deferred factors

---

## 6) 风险与回滚

### 6.1 风险评估

| 风险项 | 等级 | 说明 |
|--------|------|------|
| 优化精度 | 低 | 首次不做优化，第二次增量 update 会处理 deferred factors，最终结果一致 |
| 内存占用 | 低 | pending_graph_ 保留到下次 update，短时间内存增加 |
| 延迟 | 低 | 首次位姿为初始值，可能在下次 update 前有轻微偏差 |

### 6.2 回滚方案

若发现问题，可通过 Git 回滚：

```bash
git log --oneline -n 5  # 找到修复前的 commit
git revert <commit_hash>
```

---

## 7) 后续演进

### 7.1 短期 (MVP)

- V1-V3 (2026-03-11)：三阶段拆分，仍在 phase1 linearize 崩溃
- V4 (2026-03-12)：LM_then_ISAM2 方案，在 LM 构造函数崩溃（`NoiseModelFactor::error()` 也触发 bug）
- **V5 (2026-03-12)**：完全跳过首次优化，仅注入 values，保留 factors 到下次处理
- 增强：增加单元测试覆盖首次 update 场景

### 7.2 中期 (V1)

- 评估升级 GTSAM 到包含 borglab/gtsam#1189 修复的版本
- 增加更详细的因子提交诊断日志

### 7.3 长期 (V2)

- 考虑使用 Ceres-solver 替代 GTSAM
- 或进程级隔离 GTSAM 优化

---

## 8) 崩溃日志定位 (CRASH_CONTEXT / TRACE)

崩溃时用以下方式快速定位「最后成功步骤」：

```bash
# 1. 最后一条 CRASH_CONTEXT = 崩溃前最后一个成功步骤
grep "CRASH_CONTEXT" logs/run_xxx/full.log | tail -5

# 2. 按步骤名精确定位（step= 与代码内 TRACE 一一对应）
grep -E "CRASH_CONTEXT|ISAM2_DIAG.*TRACE|V5" logs/run_xxx/full.log | tail -30

# 3. finish_mapping 全链路
grep -E "finish_mapping|ensureBackend|forceUpdate|commitAndUpdate|V5" logs/run_xxx/full.log
```

**关键 step 含义**（若崩溃在该 step 之后、下一 step 之前，则崩溃发生在该 step 标注的调用内）：

| step | 含义 |
|------|------|
| `finish_mapping_before_ensureBackend` | 即将调用 ensureBackendCompletedAndFlushBeforeHBA |
| `ensureBackend_before_forceUpdate` | 即将调用 forceUpdate → commitAndUpdate |
| `forceUpdate_about_to_call_commitAndUpdate` | 即将调用 commitAndUpdate（持有锁，pending_factors/values 已打出） |
| `commitAndUpdate_enter` | 进入 commitAndUpdate |
| **V5 首次** | |
| `first_update_v5_enter` | 进入 V5 首次 update 路径 |
| `first_update_v5_pre_update_values` | 即将 update(empty_graph, values_copy)，仅注入节点 |
| `first_update_v5_post_update_values` | values 注入完成 |
| `first_update_v5_done` | V5 完成，deferred factors 数量 |
| `commitAndUpdate_v5_deferred_factors` | pending_graph_ 未清空，保留到下次 |
| `incremental_pre_update` | 常规增量路径，即将 isam2_.update(graph_copy, values_copy) |

V5 首次 update 会打出详细的因子/值/内存信息：

```text
[ISAM2_DIAG][V5] first update path=SKIP_OPTIMIZATION
[ISAM2_DIAG][FACTOR_DETAIL] idx=0 type=Prior keys=[...] prior_xyz=[...]
[ISAM2_DIAG][VALUE_DETAIL] key=... xyz=[...] rpy=[...]
[ISAM2_DIAG][MEM_TRACE] graph_ptr=0x... values_ptr=0x... factors_size=...
```

---

## 9) 相关文档

| 文档 | 说明 |
|------|------|
| `FIX_GPS_BATCH_SIGSEGV_20260310.md` | GPS 批量添加 SIGSEGV 修复 |
| `HBA_GTSAM_FALLBACK_DOUBLE_FREE_FIX.md` | HBA GTSAM fallback 修复 |
| `BACKEND_BEFORE_HBA_AND_DOUBLE_FREE.md` | 后端与 HBA 串行约束 |

---

## 快速定位崩溃点

```bash
grep -E "CRASH_CONTEXT|V5|SIGABRT|double free" logs/run_xxx/full.log | tail -20
```

**关键**：最后一条 `CRASH_CONTEXT` 就是崩溃前最后一个成功步骤。

```bash
# 查看 V5 相关日志
grep -E "\[V5\]|\[FACTOR_DETAIL\]|\[VALUE_DETAIL\]|\[MEM_TRACE\]" logs/run_xxx/full.log | tail -50
```

**文档版本历史**:

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | 2026-03-11 | 初始版本 (V1-V3 三阶段) |
| v2.0 | 2026-03-12 | V4 LM_then_ISAM2 方案 |
| v3.0 | 2026-03-12 | V5 跳过优化方案 |

---

**文档版本**：v3.0
**创建日期**：2026-03-11
**最后更新**：2026-03-12
**作者**：Automap Pro Team
