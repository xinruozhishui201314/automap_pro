# 后端 GPS 异步处理与建图精度影响评估

## 1. Executive Summary

- **结论**：当前采用的「同步写入窗口 + 异步执行重逻辑」方案**不牺牲建图精度**；仅将 SVD 对齐与因子回调移出 ROS 回调线程，数据可见性保持不变。
- **收益**：ROS 回调快速返回，避免 Executor 阻塞导致的 LivoBridge odom/cloud 断流。
- **风险**：无额外精度风险；对齐延迟最多为 backend 一轮等待周期（通常 &lt;500ms）。

---

## 2. 异步方案设计

### 2.1 原则：数据同步、计算异步

| 环节 | 同步 / 异步 | 说明 |
|------|-------------|------|
| **GPS 写入 gps_window_** | **同步** | `addGPSMeasurement` 内持锁写入 `gps_window_`、更新 `good_sample_count_`/`state_`，保证 `queryByTimestamp(ts)` 在关键帧创建时能读到最新数据。 |
| **try_align()（SVD 对齐）** | **异步** | 达到对齐条件时仅设置 `pending_align_` 并调用 `align_scheduler_()`；实际 `try_align()` 在 backend worker 中通过 `runScheduledAlignment()` 执行。 |
| **measurement_log_cbs_** | **锁外执行** | 已在先期修复中移出锁，在回调线程内执行但不持 GPSManager 锁。 |
| **gps_factor_cbs_** | **锁外执行** | 在锁内只复制 (ts, pos_map, cov) 与回调列表，锁外执行回调，避免长时间持锁。 |
| **ENU 原点 / map_frame.cfg** | **同步（call_once）** | 首帧 GPS 仅执行一次，开销可接受。 |

### 2.2 数据流（简化）

```
LivoBridge::onGPS (Executor)
  → AutoMapSystem::onGPS
  → GPSManager::addGPSMeasurement
       ├─ call_once(ENU + MapFrameConfig::write)
       ├─ [锁1] 写入 gps_window_、更新 good_sample/state_
       ├─ [锁外] measurement_log_cbs_
       ├─ [锁2] good_sample/state_ 更新、若达阈值 pending_align_=true, align_scheduler_()
       ├─ [锁2] 若 ALIGNED 且本帧需加因子：复制 (ts,pos,cov) 与 factor_cbs
       └─ [锁外] gps_factor_cbs_

Backend worker 每轮循环
  → gps_manager_.runScheduledAlignment()
       └─ 若 pending_align_：try_align() → on_aligned() → align_cbs_（如 onGPSAligned）
```

---

## 3. 建图精度影响评估

### 3.1 关键帧–GPS 绑定（queryByTimestamp）

- **调用点**：`tryCreateKeyFrame(ts)` 中 `gps_manager_.queryByTimestamp(ts)`。
- **数据来源**：`gps_window_`（在 `addGPSMeasurement` 中**同步**写入）。
- **结论**：**无精度损失**。关键帧创建时，当前已到达的 GPS 一定已进入 `gps_window_`，时间戳匹配与插值逻辑与改造前一致。

### 3.2 GPS–LiDAR 对齐（try_align）

- **输入**：`gps_window_`、`kf_window_`（后者由 `addKeyFramePose` 在 backend 线程同步写入）。
- **变化**：`try_align()` 由 ROS 回调线程挪到 backend 线程执行，可能延迟最多一个 backend 等待周期（例如 500ms）。
- **影响**：
  - 对齐结果（R, t）与**同一份** `gps_window_`、`kf_window_` 计算，仅执行时机略晚，**不改变数学结果**。
  - 对齐触发到实际执行之间可能多收到若干帧 GPS/KF，数据更多，对齐基于的样本只可能更多、不会更少。
- **结论**：**无精度损失**，仅对齐发生时刻略滞后。

### 3.3 实时 GPS 因子（gps_factor_cbs_）

- **行为**：仍在对齐后的每帧合格 GPS 上、按距离间隔触发；仅改为在锁外执行回调。
- **结论**：**无精度损失**；因子内容（ts, pos_map, cov）与之前一致。

### 3.4 轨迹日志 / 诊断

- **measurement_log_cbs_**、轨迹 CSV 等仅用于日志与对比，不参与优化。
- **结论**：**无精度影响**。

### 3.5 若改为「全异步 ingestion」会怎样（未采用）

若将 **整条 addGPSMeasurement 放入队列、由单独 worker 写入 gps_window_**：

- 关键帧创建时刻 `queryByTimestamp(ts)` 可能尚未看到「已到达但未入队」的 GPS。
- 会导致**部分关键帧漏绑 GPS** → 约束变少，**可能降低全局一致性精度**。
- 因此当前方案**不**采用全异步 ingestion，只做「重计算异步 + 回调锁外执行」。

---

## 4. 配置与扩展

- **关闭异步对齐**：不调用 `setAlignScheduler(...)` 或传入空 `std::function`，则达到条件时仍在 `addGPSMeasurement` 内同步执行 `try_align()`（与旧行为一致）。
- **自定义调度**：可传入自定义 `AlignScheduler`，例如将 `runScheduledAlignment()` 投递到专用线程或线程池，只要在适当时机调用 `gps_manager_.runScheduledAlignment()` 即可。

---

## 5. 验证建议

1. **回放同一 bag**：对比改造前后轨迹 CSV、最终地图与 HBA 输出，确认轨迹与地图一致。
2. **日志**：确认无 `[GPS_CALLBACK] ... exception`，且 `[GPS_STATE] NOT_ALIGNED→ALIGNING` 与 `GPS alignment SUCCESS` 仍按预期出现。
3. **延迟**：若需量化对齐延迟，可在 `runScheduledAlignment()` 内打时间戳，与 `pending_align_` 被置位时刻对比。

---

## 6. 变更清单（代码）

| 文件 | 变更概要 |
|------|-----------|
| `gps_manager.h` | 新增 `setAlignScheduler`、`runScheduledAlignment`；`align_scheduler_`、`pending_align_`。 |
| `gps_manager.cpp` | 达到对齐条件时改为设置 `pending_align_` + 调用 `align_scheduler_()`，否则同步 `try_align()`；实现 `runScheduledAlignment()`；`gps_factor_cbs_` 在锁外执行。 |
| `automap_system.cpp` | 注册 `setAlignScheduler`（no-op）；backend 循环开头调用 `runScheduledAlignment()`。 |
