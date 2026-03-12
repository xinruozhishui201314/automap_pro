# GTSAM 崩溃日志增强与约束全量转储 (2026-03-12)

## 0) Executive Summary

| 项目 | 内容 |
|------|------|
| **问题** | 首次 update 使用 LM 路径后，崩溃仍发生在 `graph_copy.error(values_copy)` 或 LM 构造/optimize 内（double free in `NoiseModelFactor::error()`） |
| **根因** | GTSAM 内部在计算 factor error 时的内存管理缺陷，与 ISAM2 或 LM 调用路径均可能触发 |
| **本次改动** | 1) 优化前**全量约束转储**（因子类型、keys、key 是否在 values、value 是否有限）；2) **约束合理性校验**；3) **细粒度 TRACE** 精确定位崩溃步骤 |
| **使用方式** | 崩溃后 `grep GTSAM_CONSTRAINTS` 看约束列表，`grep "step="` 看最后到达的步骤 |

---

## 1) 崩溃为何难以根除

### 1.1 最新崩溃现场 (run_20260312_074129)

```text
[ISAM2_DIAG][TRACE] step=first_update_using_LM_optimizer factors=12 values=5
double free or corruption (out)
#7  gtsam::NoiseModelFactor::error(gtsam::Values const&)
#8  gtsam::NonlinearFactorGraph::error(gtsam::Values const&)
#9  automap_pro::IncrementalOptimizer::commitAndUpdate()
```

- 崩溃发生在 **LM 路径** 上，且在我们调用 `graph_copy.error(values_copy)` 时（或 LM 构造/optimize 内部同样会调用 error）。
- 即：问题不在 ISAM2 首次 update，而在 **任意** 对当前因子图做 `error(Values)` 的路径上，属于 GTSAM 库内 NoiseModelFactor 的内存管理缺陷（如共享 noise model 或内部状态被重复释放）。

### 1.2 为何难以在应用层彻底解决

- 根因在 **GTSAM 库内部**（NoiseModelFactor::error → 某处 free），应用层无法直接修。
- 可行方向：升级/打补丁 GTSAM、或换用其他优化库（如 Ceres）；短期只能 **规避**（如避免首次 error 调用）或 **精准定位** 以便上报/复现。

---

## 2) 本次日志增强内容

### 2.1 优化前全量约束转储（tag=first_update_before_LM / incremental_before_isam2）

每次执行 GTSAM 优化（首次 LM 或增量 ISAM2）**之前**，会输出：

- **汇总**：`[GTSAM_CONSTRAINTS] tag=... total_factors=N total_values=M`
- **每个因子**：`[GTSAM_CONSTRAINTS] factor i type=Prior|GPS|Between|Other keys=[k1,k2] keys_in_values=[yes|NO,...] value_finite=[yes|nan/inf|n/a] valid=yes|NO`
- **每个 value**：`[GTSAM_CONSTRAINTS] value key=k x= y= z= finite=0|1`
- **校验结果**：`[GTSAM_CONSTRAINTS] tag=... validation done keys_exist=0|1 values_finite=0|1`

便于确认：

- 是否有因子的 key 不在 values 中（keys_in_values=NO）。
- 是否有 NaN/Inf（value_finite=0 或 valid=NO）。
- 崩溃时是**哪一批约束**（看 tag 与 factor 列表）。

### 2.2 约束合理性校验

- **all_keys_exist**：所有因子的 keys 是否均存在于 values 中。
- **all_values_finite**：所有 value 的 Pose3 的 translation/rotation 是否均有限（无 NaN/Inf）。
- 若校验失败会打 WARN：`[GTSAM_CONSTRAINTS] constraint validation FAILED`，并继续尝试优化（便于区分“无效约束导致异常”与“有效约束下 GTSAM 内部 double free”）。

### 2.3 细粒度 TRACE 步骤（首次 LM 路径）

| 步骤 tag | 含义 |
|----------|------|
| `copy_graph_values_done` | graph/values 拷贝完成；崩溃在此后则与拷贝无关 |
| `first_update_using_LM_optimizer` | 进入首次 LM 分支 |
| `first_update_constraints_dump_done` | 约束转储与校验完成 |
| `first_update_initial_error_enter` | **即将**调用 `graph.error(values)` |
| `first_update_initial_error_done` | `graph.error(values)` 正常返回 |
| `first_update_initial_error_exception` | `graph.error(values)` 抛异常 |
| `first_update_LM_constructor_enter` | 即将构造 LevenbergMarquardtOptimizer |
| `first_update_LM_constructor_done` | LM 构造完成 |
| `first_update_LM_optimize_enter` | 即将调用 optimizer.optimize() |
| `first_update_LM_optimize_done` | optimize() 返回 |
| `LM_optimization_done` | LM 阶段收尾日志 |

**定位方式**：崩溃后查看最后一条 `[ISAM2_DIAG][TRACE] step=...` 或 `[GTSAM_CONSTRAINTS]`：

- 若最后是 `first_update_initial_error_enter` 且无 `first_update_initial_error_done` → 崩溃在 **graph.error(values)** 内。
- 若最后是 `first_update_LM_constructor_enter` 且无 `first_update_LM_constructor_done` → 崩溃在 **LM 构造**内。
- 若最后是 `first_update_LM_optimize_enter` 且无 `first_update_LM_optimize_done` → 崩溃在 **optimizer.optimize()** 内。

---

## 3) 如何用日志精准定位

### 3.1 崩溃后执行

```bash
# 约束与校验
grep "GTSAM_CONSTRAINTS" full.log

# 最后到达的 TRACE 步骤
grep "step=" full.log | tail -20
```

### 3.2 判断崩溃点

1. 看 **最后一个 step=** 是哪一个 → 确定是 error()、LM 构造还是 optimize()。
2. 看 **同一 tag 的 GTSAM_CONSTRAINTS**：是否有 keys_in_values=NO 或 valid=NO → 若存在，先修复约束一致性或数值。
3. 若约束均 valid 仍 double free → 可判定为 GTSAM 库内 bug，保留 full.log（含 GTSAM_CONSTRAINTS 与 step=）便于上报或复现。

---

## 4) 变更清单

| 文件 | 修改类型 | 说明 |
|------|----------|------|
| `src/backend/incremental_optimizer.cpp` | 新增 + 修改 | 匿名命名空间内 `logAllConstraintsAndValidate()`；首次 LM 与增量路径优化前调用；细粒度 TRACE |
| `docs/GTSAM_CRASH_LOGGING_AND_CONSTRAINTS_20260312.md` | 新增 | 本文档 |

---

## 5) 编译与验证

```bash
cd /root/automap_ws  # 或 automap_ws 路径
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
```

复现步骤同前（offline mapping + finish_mapping 触发）。确认：

- 优化前能看到 `[GTSAM_CONSTRAINTS] tag=first_update_before_LM` 及每条 factor/value。
- 若再次崩溃，用 3.1/3.2 的 grep 与表格定位崩溃步骤与约束是否合理。

---

## 6) 后续建议

- **短期**：若确认崩溃在 `graph.error(values)` 且约束均合理，可尝试 **去掉** 首次 LM 前的 `initial_error = graph_copy.error(values_copy)` 调用（仅保留 TRACE），看是否避免触发 double free（LM 内部仍会算 error，若仍崩则需库级修复或换库）。
- **中期**：升级 GTSAM 或提交 minimal repro 给上游。
- **长期**：评估 Ceres 等替代后端。

---

**文档版本**：v1.0  
**创建日期**：2026-03-12
