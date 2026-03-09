# full.log 建图提前结束与 [LivoBridge][FRAME] 深入分析

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **建图为何“执行不完就结束”** | 本段日志中：① 数据源仅约 **1500 帧** 点云（fast_livo 发布 #1～#1500 后 bag 无新 lidar）；② 最后一帧 #1500 后 backend 在做 **global map 构建**（227 万点体素下采样 ~0.6s），随后日志中 **无 sensor_idle 记录**，但 09:05:41 之后 **无 automap_system 新日志**，09:12:28 rosbag 进程结束；③ 若启用 `offline_finish_after_bag`，结束由 bag 播完 + `finish_mapping` 触发，否则 100s 无点云会触发 sensor_idle 保存并退出。 |
| **[LivoBridge][FRAME] #1500 一直增加** | **不是**“处理不了”。该数字是 **LivoBridge 收到的点云帧序号**：每收到一帧 `/cloud_registered` 就 +1，#1500 表示已收到第 1500 帧。数字随 bag 播放递增，到 1500 后不再增加是因为 **bag 里只有约 1500 帧 lidar 数据**（或播放结束）。 |
| **优化方向** | 见下文「优化方案」：拉长/关闭 sensor_idle、保证 offline 播完再结束、降低 global map 构建成本、观测队列与延迟。 |

---

## 1. [LivoBridge][FRAME] #N 含义（澄清“一直增加”）

### 1.1 代码含义

- 出处：`automap_pro/src/frontend/livo_bridge.cpp` 的 `onCloud()`。
- `cloud_count_++`：每收到一帧 `PointCloud2` 自增。
- 日志：`[LivoBridge][FRAME] #%d ts=%.3f pts=%zu → backend` 中的 `#N` = 当前 `cloud_count_`，即 **累计收到的第 N 帧点云**。

```215:216:automap_pro/src/frontend/livo_bridge.cpp
    RCLCPP_INFO(node_->get_logger(), "[LivoBridge][FRAME] #%d ts=%.3f pts=%zu → backend",
                c, ts, cloud->size());
```

### 1.2 结论

- **不是**“处理不过来、卡在某一帧”：N 是 **接收序号**，不是“未处理数量”。
- **会一直增加**：只要 fast_livo 持续发 `/cloud_registered`，N 就 1, 2, 3, … 递增。
- **到 1500 后不变**：说明此后 **没有新的点云**（bag 只含约 1500 帧 lidar，或播放已结束）。

---

## 2. 建图“执行不完就结束”的时间线（full.log）

### 2.1 关键时间点

| 时间 (host) | 事件 |
|-------------|------|
| 09:00:39 | launch 启动，bag 开始播放（rate=0.5） |
| 09:00:40 | 首帧 odom/cloud，#1 → backend，首 KF |
| 09:05:40 | odom #1500，fast_livo 发布 cloud #1500 |
| 09:05:41 | LivoBridge 收到 cloud #1500（delta_recv_ms=1），FRAME #1500 → backend；同时 backend 在做 **buildGlobalMap + voxelDownsampleChunked**（2272312 pts → 436581） |
| 09:05:41 | buildGlobalMap 完成，map_published points=436581 |
| 09:05:41 之后 | **无** automap_system 的 PIPELINE/event 等日志 |
| 09:12:23–09:12:28 | fast_livo 大量 `imu time stamp Jumps 192~194 s`（bag 仍在播 IMU 或时间跳变） |
| 09:12:28 | `[ros2-1]: process has finished cleanly`，rosbag 播放结束 |
| 09:12:33 | `[bash-8] waiting for service to become available...` |

### 2.2 根因归纳

1. **数据量**：本段 bag 中 **lidar 仅约 1500 帧**（fast_livo 只发布了 #1～#1500），之后无新点云。
2. **Backend 在最后一帧附近**：在 09:05:41 正在做 **全局图构建**（merge + voxelDownsampleChunked），耗时约 0.6s，此时 **#1500 仍被接收并送入 backend**（见 5456–5458 行），并非“卡住不处理”。
3. **本 log 未出现 sensor_idle**：当前配置 `sensor_idle_timeout_sec: 100.0`、`offline_finish_after_bag: true`；若 100s 内无新点云且队列空，会触发 `sensor_idle_timeout` → final HBA → save → shutdown；本 log 中未见该事件，可能：automap 在 09:05:41 后不久退出/崩溃，或该段日志未包含后续 PIPELINE。
4. **“建图没跑完”的主观感受**：bag 总时长可能更长（例如还有大量 IMU/其他话题），但 **lidar 只有约 1500 帧**，所以从建图视角“数据就这么多”；若期望“整条轨迹都建完”，需确认 bag 中 `/velodyne_points` 或 `/cloud_registered` 的条数/时长是否与预期一致。

---

## 3. 优化方案

### 3.1 配置层（优先）

| 目标 | 配置/行为 | 说明 |
|------|-----------|------|
| 避免短时无点云就结束 | `sensor_idle_timeout_sec: 60.0`～`120.0` 或更大 | 当前 M2DGR 已设 100，若 bag 中间有长间隙可再加大。 |
| 离线“播完再结束” | `offline_finish_after_bag: true`（已开） | 依赖 launch 在 bag 播完后调用 `finish_mapping`，不单靠 100s 空闲。 |
| 计算偶尔跟不上时少误判空闲 | `frame_queue_max_size: 1000`（或 1500） | 队列更大，短暂断流时 backend 仍有帧可处理，不易触发 idle。 |

### 3.2 Launch 与结束逻辑

- 确保 **bag 播完后** 调用 `/automap/finish_mapping`（或等价逻辑），再等 automap 保存完成再退出。
- 若 launch 是“bag 进程结束即整体退出”，automap 可能还没做完 final HBA/save 就被杀；可改为：bag 结束 → 调用 finish_mapping → 轮询 get_status 或等待 save 完成 → 再 shutdown。

### 3.3 性能与可观测性

| 方向 | 做法 |
|------|------|
| 降低 global map 构建成本 | 增大 `map.voxel_size`（如 0.25）减少体素下采样点数；或对 buildGlobalMap 做分块/降频发布，避免每 KF 都全量 227 万点下采样。 |
| 观测“是否处理得过来” | 看 `queue_after_pop`、`data_flow` 的 queue 长度；若长期接近 `frame_queue_max_size` 且 `delta_recv_ms` 很大，说明 backend 跟不上。 |
| 观测结束原因 | 在日志中搜 `sensor_idle_timeout`、`sensor_idle_save_enter`、`finish_mapping`，确认是“空闲超时”还是“播完+服务调用”结束。 |

### 3.4 数据与 bag 校验

- 用 `ros2 bag info <bag_dir>` 看 `/velodyne_points`（及若有）`/cloud_registered` 的 **message count** 和 **duration**。
- 若 message count ≈ 1500，则“建图只到 1500 帧”是数据上限，不是 pipeline 提前结束；若远大于 1500，则需查为何 LivoBridge 只收到 1500 帧（话题、QoS、播放速率等）。

---

## 4. 小结

- **[LivoBridge][FRAME] #1500**：表示 **已收到第 1500 帧点云**，数字随接收递增，到 1500 止是因为没有更多点云输入，**不是**“处理不了”。
- **建图“执行不完就结束”**：本 log 中数据仅约 1500 帧；最后一帧前后 backend 正常做 global map；09:05:41 后无 automap 日志、09:12:28 bag 结束；通过加大 `sensor_idle_timeout_sec`、保证 `offline_finish_after_bag` + bag 播完调用 `finish_mapping`、适当增大 `frame_queue_max_size` 并核对 bag 中 lidar 条数，可避免误判“提前结束”并让离线建图完整收尾。

---

## 5. 已执行优化（计算效率提升）

以下优化已落地，可直接使用当前配置与代码验证效果。

| 项 | 变更 | 文件 | 预期效果 |
|----|------|------|----------|
| 全局图发布降频 | `publish_global_map_every_n_processed`: 100 → **300** | `system_config_M2DGR.yaml` | 减少 buildGlobalMap 调用约 66%，降低 CPU 峰值 |
| 帧队列与入口缓冲 | `frame_queue_max_size`: 500 → **1500**，`ingress_queue_max_size`: 16 → **32** | 同上 | 减少丢帧，处理峰值时更稳 |
| 体素下采样并行化 | 分块循环改为 `#pragma omp parallel for schedule(dynamic)`，每块独立滤波后合并 | `automap_pro/src/core/utils.cpp` | buildGlobalMap 中 voxelDownsampleChunked 耗时预计降 30%～50%（多核） |

**验证建议**：重跑同一 bag，对比日志中 `buildGlobalMap`/`voxelDownsampleChunked` 出现频率与耗时、`[AutoMapSystem][DATA_FLOW] dropped=` 是否趋近 0。

---

## 6. 相关文档与配置

- 传感器空闲结束逻辑：`docs/MAP_EARLY_END_ANALYSIS.md`
- 配置项：`automap_pro/config/system_config_M2DGR.yaml`（`sensor_idle_timeout_sec`、`auto_finish_on_sensor_idle`、`offline_finish_after_bag`、`frame_queue_max_size`）
- 代码：`automap_system.cpp` 中 `last_sensor_data_wall_time_` 更新（onCloud）、backend worker 中 wait + sensor_idle 判断与 final HBA/save
- 体素下采样：`automap_pro/src/core/utils.cpp` 中 `voxelDownsampleChunked`（已 OpenMP 并行化）
