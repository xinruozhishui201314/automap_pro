# 卡住根因分析：run_20260318_162915 full.log

## 1. 现象摘要

- **表现**：建图在 **processed_no=56**（frame_no=276）处卡住；最后一条 BACKEND STEP 为 `createKeyFrame_gps_query_enter`，之后再无 `createKeyFrame_gps_query_exit` 或 `about_to_lock_keyframe_mutex`，backend 线程一直未前进。
- **HEARTBEAT**：`[HEARTBEAT] CRITICAL: threads stuck: map_pub(54s) loop_opt(54s)`。
- **与上次 run_20260318_160614 的差异**：上次卡在 `tryCreateKeyFrame_enter(56)` 之后、未出现 `createKeyFrame_enter`（疑为 merge/体素耗时）；本次**明确卡在 GPS 查询段**：有 `createKeyFrame_gps_query_enter`，无 `createKeyFrame_gps_query_exit`。

## 2. 日志定位结论

### 2.1 最后一条 BACKEND STEP

```text
16:33:20.939 [BACKEND][STEP] step=tryCreateKeyFrame_enter processed_no=56 ts=1628249905.211
16:33:20.939 [BACKEND][STEP] step=createKeyFrame_gps_query_enter ts=1628249905.211  ← 最后一条
```

之后**没有**出现：

- `step=createKeyFrame_gps_query_exit`
- `step=about_to_lock_keyframe_mutex`
- 任意后续 STEP

因此 **backend 线程卡在 `createKeyFrame` 内的「GPS 查询」代码块**（`automap_system.cpp` 1514–1554 行之间），即：

- `gps_manager_.queryByNearestPosition(cur_pose.translation())`，或
- `gps_manager_.queryByTimestampEnhanced(ts)`（当 position 查询无结果时），或
- 其后的 `if (gps_opt)` / `else` 分支中的 `getFirstGpsTimestamp()`、`getLastGpsTimestamp()`、`getGpsWindowSize()`、`RCLCPP_WARN_THROTTLE(...)` 等。

### 2.2 时间线（简要）

| 时间       | 事件 |
|------------|------|
| 16:33:20.739 | `runScheduledAlignment` → `try_align` → callbacks（GPS_BATCH、forceUpdate）→ `try_align callbacks_done`、`runScheduledAlignment exit` |
| 16:33:20.939 | Backend：`about_to_process frame_no=276 processed_no=56` → `tryCreateKeyFrame_enter(56)` → **createKeyFrame_gps_query_enter** |
| 16:33:21.138 | 仅见 onOdometry/onCloud（其他线程），无新 BACKEND STEP |
| 16:34:05    | fast_livo PUB #500（约 44s 后），backend 仍无新 STEP |

结论：自 16:33:20.939 起，backend 一直停在「GPS 查询」块内，至少 44 秒未返回。

## 3. 根因分析

### 3.1 卡住位置（代码）

卡点位于 `automap_system.cpp` 1514–1554 行之间：

```cpp
RCLCPP_INFO(..., "step=createKeyFrame_gps_query_enter ...");
auto gps_opt = gps_manager_.queryByNearestPosition(cur_pose.translation());
if (!gps_opt)
    gps_opt = gps_manager_.queryByTimestampEnhanced(ts);
if (gps_opt) { ... } else {
    double first_gps_ts = gps_manager_.getFirstGpsTimestamp();
    double last_gps_ts  = gps_manager_.getLastGpsTimestamp();
    // ... RCLCPP_WARN_THROTTLE(get_logger(), *clk, 30000, ...);
}
RCLCPP_INFO(..., "step=createKeyFrame_gps_query_exit ...");
```

以上所有 `gps_manager_` 的 query/get 接口内部都会对 **`GPSManager::mutex_`** 加锁（`std::lock_guard<std::mutex> lk(mutex_)`）。

### 3.2 最可能的根本原因：GPS mutex 锁竞争 / 长时间持锁

- **Backend**：在 `createKeyFrame_gps_query` 中调用 `queryByNearestPosition` 或 `queryByTimestampEnhanced` 或 `getFirstGpsTimestamp`/`getLastGpsTimestamp` 等，需要获取 **`gps_manager_.mutex_`**。
- **其他线程**：  
  - **addGPSMeasurement**（由 ROS 回调/worker 等调用）在 100、151 行等处持同一把 `mutex_`；  
  - 若某处存在**长时间持锁**（例如在持锁时做重计算、I/O 或误调用会阻塞的接口），则 backend 在等待该锁时就会长时间卡在「GPS 查询」块内，表现为有 `createKeyFrame_gps_query_enter` 而无 `createKeyFrame_gps_query_exit`。
- 当前实现中 `addGPSMeasurement` 的临界区仅做状态更新与回调列表拷贝，理论上不应持锁数十秒；但在**高负载或异常路径**下（如大量 GPS 回调、配置/日志在锁内、或第三方回调阻塞），仍可能出现长时间持锁，从而让 backend 一直等锁。

因此，**最符合日志的根因是：backend 在等待 `GPSManager::mutex_` 时被阻塞**，而该锁被其他线程（最可能是执行 `addGPSMeasurement` 或其它使用 `mutex_` 的路径）长时间占用。

### 3.3 次要可能

- **RCLCPP_WARN_THROTTLE**（30s 节流）及 `get_clock()`：若在「无 gps_opt」的 else 分支里，且日志/时钟实现异常导致阻塞，理论上可能拖慢该段，但通常不足以单独解释 44s 且无 exit 日志；更合理的仍是「等锁」导致整段无法完成。
- **同一线程重入 mutex**：当前 `queryByNearestPosition` 先持锁再释锁，再调 `queryByTimestamp`（内部再次持锁），无同一线程二次持锁，故重入死锁概率低。

### 3.4 HEARTBEAT 中 map_pub / loop_opt 的 54s

与 [run_20260318_160614](ROOT_CAUSE_STUCK_RUN_20260318_160614.md) 一致：因 **backend 卡在 56 帧的 GPS 查询**，不再产生新关键帧与地图更新，map_pub 与 loop_opt 线程无新工作可做，表现为「stuck 54s」；二者是**后果**，不是独立死锁。

## 4. 建议措施

### 4.1 短期（可立即做）

- **缩短 GPS 临界区**：  
  - 确保 `addGPSMeasurement` 及所有持 `mutex_` 的路径在锁内**只做最小必要**操作（拷贝、更新窗口与状态），不在锁内调用可能阻塞的 I/O、日志或复杂计算。  
  - 将「统计、判断、回调列表拷贝」以外的逻辑尽量移到锁外（已有部分如此，可再审查）。
- **createKeyFrame 内 GPS 查询**：  
  - 在 `queryByNearestPosition` / `queryByTimestampEnhanced` 调用处增加**超时或 try_lock**（例如 `std::timed_mutex` 或非阻塞尝试），若在约定时间内拿不到 `mutex_` 则跳过本帧 GPS 绑定并打 WARN，避免 backend 无限等锁。  
  - 这样即使出现长时间持锁，也不会导致整条 backend 卡死，便于从日志区分「等锁超时」与「逻辑死锁」。

### 4.2 中期（结构）——已实现：窗口快照

- **GPS 窗口快照（已实现）**：  
  - `GPSManager` 增加 `GpsWindowSnapshot` 与 `getSnapshot()`：持 `mutex_` 仅做一次拷贝（`gps_window_` + 对齐状态 R/t + 配置参数），然后释放锁。  
  - 所有只读 query（`queryByTimestamp`、`queryByNearestPosition`、`queryByTimestampEnhanced`、`queryByTimestampForLog`、`getFirstGpsTimestamp`、`getLastGpsTimestamp`、`getGpsWindowSize`、`getGpsWindowTimeRange`、`getGpsPositionsInMapFrame`、`currentQuality`、`getLatestPositionENU`、`getGoodSampleCount`）均改为：先 `getSnapshot()`，再在锁外基于快照计算，从而大幅缩短持锁时间，避免 backend 在 GPS 查询段长时间等锁。
- **可观测性**：  
  - 在 `queryByNearestPosition` / `queryByTimestampEnhanced` 入口处打 TRACE（或采样），并在**拿锁前后**打时间戳；若「拿锁前」到「拿锁后」间隔异常大，即可确认为等锁导致卡住。

### 4.3 再次卡住时的排查命令

```bash
# 最后几条 BACKEND/SubMapMgr STEP，确认是否仍是「GPS 查询」段
grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' logs/run_*/full.log | tail -20

# 是否出现 createKeyFrame_gps_query_enter 但无 exit
grep -E 'createKeyFrame_gps_query_enter|createKeyFrame_gps_query_exit' logs/run_*/full.log | tail -10

# HEARTBEAT 与 STUCK
grep -E 'HEARTBEAT|STUCK_DIAG' logs/run_*/full.log
```

若再次出现「有 createKeyFrame_gps_query_enter、无 createKeyFrame_gps_query_exit」：

- 优先考虑 **GPS mutex 被其他线程长时间占用**，用上述「拿锁前后时间戳」或 try_lock/超时 验证并缓解。
- 若增加 try_lock/超时后出现「GPS query timeout」类日志，则可确认根因并进一步优化持锁路径（addGPSMeasurement 及所有使用 `mutex_` 的调用点）。

## 5. 总结

| 项目     | 结论 |
|----------|------|
| **卡住位置** | Backend 停在 `createKeyFrame` 内 GPS 查询块（1514–1554 行），未执行到 `createKeyFrame_gps_query_exit`。 |
| **根因** | Backend 在调用 `queryByNearestPosition` / `queryByTimestampEnhanced` 或 else 分支中的 get* 时，**等待 `GPSManager::mutex_`** 被其他线程长时间占用而阻塞。 |
| **map_pub / loop_opt 54s** | 因 backend 卡住导致无新进度，属连锁反应，非独立死锁。 |
| **与 160614 的差异** | 160614：卡在 tryCreateKeyFrame 内、未到 createKeyFrame_enter（merge/体素）。本次：明确卡在 GPS 查询段，属**不同卡点**，需针对性缩短 GPS 持锁时间并为 query 增加超时/ try_lock。 |

**后续复现**：若再次卡在同一位置，建议在 `gps_manager` 的 query/get 入口加「等锁超时」或 try_lock + 诊断日志，以便直接确认是否为 mutex 竞争并定位持锁方。
