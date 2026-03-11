# GTSAM 多路使用与崩溃定位（防护 + 日志）

## 1. Executive Summary

- **问题**：同一进程内多处使用 GTSAM（iSAM2、Optimizer、HBA PGO）时，GTSAM 内部 TBB 并发容器易在 `internal_clear`/`free` 处触发 SIGSEGV，且崩溃后难以区分是哪一个调用点。
- **方案**：① 全局串行化 GTSAM 调用（互斥 + 可选 TBB 并行度=1）；② 统一 `[GTSAM_ENTRY]` / `[GTSAM_EXIT]` 日志，崩溃时**最后一条 ENTRY 无对应 EXIT** 即出事调用点。
- **涉及模块**：`IncrementalOptimizer`（ISAM2）、`Optimizer`（LevenbergMarquardt）、`HBAOptimizer`（hba_api PGO）。

---

## 2. 多路 GTSAM 使用策略

| 机制 | 说明 | 配置 |
|------|------|------|
| **全局互斥** | 任意时刻仅允许一处执行“进入 GTSAM 的代码”（commitAndUpdate / optimize / HBA PGO），由 `GtsamCallScope` 在构造时加锁、析构时解锁。 | 默认开启；`AUTOMAP_GTSAM_SERIAL=0` 关闭 |
| **TBB 并行度=1** | 进程内首次创建 `IncrementalOptimizer` 时调用 `ensureGtsamTbbSerialized()`，将 TBB `max_allowed_parallelism` 设为 1，避免 GTSAM 内部并发容器竞态。 | 自动执行一次；依赖 `tbb/global_control.h` 可用 |
| **单次 update** | iSAM2 仅一次 `isam2_.update(graph, values)`，不做空 `update()`；GPS 批量为单次 BATCH_UPDATE。 | 见 `FIX_GPS_BATCH_SIGSEGV_20260310.md` |

---

## 3. 统一日志约定

所有进入 GTSAM 的入口均通过 `GtsamCallScope` 打两条日志：

- **ENTRY**：`[GTSAM_ENTRY] caller=ISAM2|Optimizer|HBA op=... thread_id=... [params] (若崩溃在此后、无 GTSAM_EXIT→崩溃在该次 GTSAM 调用内)`
- **EXIT**：`[GTSAM_EXIT] caller=... op=... duration_ms=... success=0|1`

| caller | 含义 | op 典型值 |
|--------|------|-----------|
| ISAM2 | IncrementalOptimizer（iSAM2） | `commitAndUpdate`, `addGPSFactorsBatch` |
| Optimizer | Optimizer::optimizeGTSAM（位姿图批量） | `optimize` |
| HBA | HBA 位姿图优化（hba_api） | `PGO` |

---

## 4. 崩溃时如何精确定位

### 4.1 推荐 grep 命令

```bash
# 只看 GTSAM 入口/出口，最后 N 条
grep -E 'GTSAM_ENTRY|GTSAM_EXIT' full.log | tail -30

# 结合 ISAM2_DIAG / GPS_BATCH 看完整时间线
grep -E 'GTSAM_ENTRY|GTSAM_EXIT|\[ISAM2_DIAG\]|\[GPS_BATCH\]' full.log | tail -50
```

### 4.2 解读规则

| 日志现象 | 结论 |
|----------|------|
| 最后一条为 `[GTSAM_ENTRY] caller=ISAM2 op=commitAndUpdate ...`，且**无**后续 `[GTSAM_EXIT]` | 崩溃发生在 **iSAM2 的 commitAndUpdate** 内部（如 `isam2_.update()` → GTSAM addVariables / TBB internal_clear）。 |
| 最后一条为 `[GTSAM_ENTRY] caller=HBA op=PGO ...`，无 EXIT | 崩溃在 **HBA PGO**（hba_api 内 LevenbergMarquardtOptimizer）内部。 |
| 最后一条为 `[GTSAM_ENTRY] caller=Optimizer op=optimize ...`，无 EXIT | 崩溃在 **Optimizer::optimizeGTSAM** 内部。 |
| ENTRY 与 EXIT 成对出现，崩溃在之后 | 崩溃不在本次 GTSAM 调用内，需结合 GDB 栈或后续日志（如地图发布、其他节点）排查。 |

### 4.3 与 GDB 栈对应

- GDB 显示 `commitAndUpdate` / `addGPSFactorsBatch` → 日志中应有 `caller=ISAM2` 且 `op=commitAndUpdate` 或 `addGPSFactorsBatch`。
- GDB 显示 `optimizer.optimize()`（hba_api）→ 日志中应有 `caller=HBA op=PGO`。
- 结合 **thread_id** 可确认是否为 opt 线程或 HBA 工作线程。

---

## 5. 配置与环境变量

| 项 | 说明 |
|----|------|
| `AUTOMAP_GTSAM_SERIAL` | 设为 `1`、`true`、`yes` 时启用全局 GTSAM 互斥（默认启用）；设为 `0` 关闭互斥（仅保留日志，不串行化）。 |

---

## 6. 代码位置速查

| 功能 | 文件 | 说明 |
|------|------|------|
| GtsamCallScope / TBB 控制 | `include/automap_pro/backend/gtsam_guard.h`, `src/backend/gtsam_guard.cpp` | 全局互斥、ENTRY/EXIT 日志、`ensureGtsamTbbSerialized()` |
| ISAM2 使用处 | `src/backend/incremental_optimizer.cpp` | 构造时 `ensureGtsamTbbSerialized()`；`commitAndUpdate()`、`addGPSFactorsBatch()` 内 `GtsamCallScope` |
| Optimizer GTSAM | `src/backend/optimizer.cpp` | `optimizeGTSAM()` 内 `GtsamCallScope` |
| HBA GTSAM | `src/backend/hba_optimizer.cpp` | `runHBA()` 内 `optimizer.optimize()` 外包裹 `GtsamCallScope` |

---

## 7. 验证与回滚

- **验证**：同场景回放（如 M2DGR + GPS 对齐），确认日志中 `[GTSAM_ENTRY]` 与 `[GTSAM_EXIT]` 成对、无 SIGSEGV。
- **关闭互斥**：`AUTOMAP_GTSAM_SERIAL=0` 仅关闭全局锁，ENTRY/EXIT 日志仍保留，便于定位。
- **回滚**：移除 `GtsamCallScope` 与 `ensureGtsamTbbSerialized()` 调用，并从 CMake 去掉 `gtsam_guard.cpp` 即可恢复旧行为。
