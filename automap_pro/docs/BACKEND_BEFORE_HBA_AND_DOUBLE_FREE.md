# 后端先于 HBA 的时序保证与 double free 防护

## Executive Summary

- **设计原则**：必须先完成后端（ISAM2）优化并释放 GTSAM 相关状态，再进行 HBA（GTSAM fallback 的 LevenbergMarquardtOptimizer）；否则两路 GTSAM 同时运行易触发 `double free or corruption (out)`（崩溃栈常出现在 `LevenbergMarquardtOptimizer` 构造内 `graph.error(initial)` → `NoiseModelFactor::error`）。
- **当前实现**：所有 HBA 触发路径在调用 `hba_optimizer_.triggerAsync` 或 `hba_optimizer_.onGPSAligned` 前均调用 `ensureBackendCompletedAndFlushBeforeHBA()`；且 `waitForPendingTasks()` 现同时等待「队列空」与「无 commitAndUpdate 正在执行」；HBA 与 ISAM2 的 GTSAM 调用均通过 `GtsamCallScope(use_global_mutex=true)` 串行化。
- **本次加固**：在 `IncrementalOptimizer` 中增加 `optimization_in_progress_` 标志，在 `commitAndUpdate` 内设 true/false；`waitForPendingTasks()` 在等待队列空的同时等待该标志为 false，确保「后端真正空闲」后再允许 HBA 触发，进一步消除竞态窗口。

---

## 1. 需求：先后端，再 HBA

| 要求 | 说明 |
|------|------|
| 顺序 | 后端（ISAM2）完成当前优化并释放 GTSAM 相关变量后，才允许 HBA 使用 GTSAM（LM 优化） |
| 目的 | 避免同一进程内 ISAM2 与 LevenbergMarquardtOptimizer 同时操作 GTSAM 内部状态，导致 double free / 内存损坏 |

---

## 2. 代码是否按该逻辑实现？

### 2.1 触发点与 ensureBackendCompletedAndFlushBeforeHBA

以下所有会向 HBA 入队任务的路径，在触发前都调用了 `ensureBackendCompletedAndFlushBeforeHBA()`：

| 触发点 | 文件与位置 | 调用顺序 |
|--------|------------|----------|
| 子图冻结周期 HBA | `automap_system.cpp` 约 1455–1464 | `ensureBackendCompletedAndFlushBeforeHBA()` → `hba_optimizer_.triggerAsync(all, false)` |
| GPS 对齐后 HBA | `automap_system.cpp` 约 1752–1760 | `addBatchGPSFactors()`（内部含 wait/flush）→ `ensureBackendCompletedAndFlushBeforeHBA()` → `hba_optimizer_.onGPSAligned(result, all)`（内部 `triggerAsync`） |
| 传感器空闲最终 HBA | `automap_system.cpp` 约 878–881 | `ensureBackendCompletedAndFlushBeforeHBA()` → `hba_optimizer_.triggerAsync(all, true)` |
| 服务 TriggerHBA | `automap_system.cpp` 约 2169–2171 | `ensureBackendCompletedAndFlushBeforeHBA()` → `hba_optimizer_.triggerAsync(all, req->wait_for_result)` |
| 建图结束最终 HBA | `automap_system.cpp` 约 2288–2290 | `ensureBackendCompletedAndFlushBeforeHBA()` → `hba_optimizer_.triggerAsync(all, true)` |

结论：**所有 HBA 触发路径都先调用了 ensureBackendCompletedAndFlushBeforeHBA，再触发 HBA。**

### 2.2 ensureBackendCompletedAndFlushBeforeHBA 的实现

```cpp
void AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA() {
    isam2_optimizer_.waitForPendingTasks();   // 等待队列空且无 commitAndUpdate 执行中
    if (isam2_optimizer_.hasPendingFactorsOrValues())
        isam2_optimizer_.forceUpdate();       // 把当前 pending 提交并释放 GTSAM
}
```

- `waitForPendingTasks()`：等待 **opt 队列为空** 且 **optimization_in_progress_ == false**（本次加固后），即后端当前没有在执行 `commitAndUpdate`。
- `forceUpdate()`：在调用线程上执行一次 `commitAndUpdate()`（持 `rw_mutex_` 与全局 GTSAM 锁），把尚未提交的 pending 提交并释放 GTSAM，然后返回。

因此，从语义上保证了「先完成后端优化（并释放 GTSAM），再允许 HBA 被触发」。

### 2.3 GTSAM 串行化（GtsamCallScope）

- **ISAM2**：`commitAndUpdate()` 内使用 `GtsamCallScope(GtsamCaller::ISAM2, "commitAndUpdate", ..., true)`，即 **use_global_mutex = true**。
- **HBA GTSAM fallback**：`runGTSAMFallback()` 入口使用 `GtsamCallScope(GtsamCaller::HBA, "GTSAM_fallback", ..., true)`。

因此，任意时刻最多只有一个「ISAM2 的 commitAndUpdate」或「HBA 的 GTSAM」在执行，不会两路 GTSAM 同时跑。

### 2.4 潜在竞态（已通过 optimization_in_progress_ 收紧）

原先 `waitForPendingTasks()` 只等 **队列空**，不保证 opt 线程已**离开** `commitAndUpdate()`。因此理论上存在：

- opt 线程：正在执行 `commitAndUpdate()`（持全局 GTSAM 锁）；
- 调用线程：`waitForPendingTasks()` 因队列已空立即返回，随后 `forceUpdate()` 等 `rw_mutex_` 而阻塞。

此时虽不会出现「两路 GTSAM 同时执行」（因为 forceUpdate 会等锁），但为语义清晰、避免未来改动引入竞态，本次增加：

- **optimization_in_progress_**：在 `commitAndUpdate()` 入口（通过 pending 非空后）置 true，出口（RAII guard）置 false。
- **waitForPendingTasks()**：在等待「队列空」的同时，等待 `optimization_in_progress_ == false`（带超时），这样返回时表示「后端当前没有在执行 commitAndUpdate」，再执行 `forceUpdate()` 更符合「先完成后端，再 HBA」的语义。

---

## 3. 崩溃原因与防护对应关系

- **现象**：`double free or corruption (out)`，SIGABRT，栈在 `LevenbergMarquardtOptimizer` 构造函数内 `graph.error(initial)` → `NoiseModelFactor::error`（见终端日志 #7–#10）。
- **可能原因**：两路 GTSAM（ISAM2 与 HBA LM）在时间上重叠使用 GTSAM（或库内静态/共享状态），导致内存重复释放或损坏。
- **防护**：
  1. **顺序**：所有 HBA 触发前都调用 `ensureBackendCompletedAndFlushBeforeHBA()`，保证「先后端、再 HBA」。
  2. **空闲条件**：`waitForPendingTasks()` 同时等待队列空和 `optimization_in_progress_ == false`，确保后端「真正空闲」再继续。
  3. **互斥**：ISAM2 与 HBA 的 GTSAM 调用均通过 `GtsamCallScope(use_global_mutex=true)` 串行化。

---

## 4. 变更清单

| 文件 | 变更 |
|------|------|
| `include/automap_pro/backend/incremental_optimizer.h` | 新增成员 `std::atomic<bool> optimization_in_progress_`。 |
| `src/backend/incremental_optimizer.cpp` | 在 `commitAndUpdate()` 内对非空 pending 分支设置/清除 `optimization_in_progress_`（RAII）；`waitForPendingTasks()` 在等待队列空的同时等待 `!optimization_in_progress_`（带超时）。 |
| `src/system/automap_system.cpp` | `ensureBackendCompletedAndFlushBeforeHBA()` 增加说明性日志。 |

---

## 5. 编译与验证

- 编译：`colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 验证建议：
  - 使用曾出现 double free 的 bag/场景回放，确认不再出现 `LevenbergMarquardtOptimizer` 构造内崩溃。
  - 日志中确认：在每次 `[HBA][STATE] optimization start` 之前，有 `event=backend_flushed_before_hba` 以及（若发生过等待）`waitForPendingTasks done ... (queue empty and no commitAndUpdate in progress)`。

---

## 6. 风险与回滚

- **风险**：`waitForPendingTasks()` 现在多等一个条件，在极端负载下可能略增延迟（通常仍受现有 5s 超时限制）。
- **回滚**：若需回滚，去掉 `optimization_in_progress_` 的设值/等待逻辑，恢复 `waitForPendingTasks()` 仅按队列空判断即可；其余「先 ensure 再 trigger」逻辑保持不变即可保留主要防护效果。

---

## 7. 参考

- `HBA_CONSTRAINTS_AND_BACKEND_ORDER.md`：HBA 约束与「后端优化后再 HBA」的语义与初始值策略。
- `FIX_GPS_BATCH_SIGSEGV_20260310.md`：GPS 批量与 GTSAM 使用相关崩溃及 7.1/7.2 诊断。
- `GTSAM_MULTI_USE_AND_LOGGING.md`：多路 GTSAM 与 GtsamCallScope 使用约定。
