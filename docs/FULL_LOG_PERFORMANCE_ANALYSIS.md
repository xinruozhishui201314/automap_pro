# full.log 性能与“未执行完成就结束”分析

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **为何提前结束** | 后端因 **传感器空闲超时**（队列空 ≥10s）触发自动保存并 `rclcpp::shutdown()`；此时 bag 仍在播 IMU 等，但 **LiDAR/点云已不再输入**，故 automap 只处理了 bag 前一段（约 270s 数据，142 KF，3 submap）。 |
| **后端每 5 帧处理一帧** | **单帧 tryCreateKeyFrame** 约 **&lt;1ms**；每 **100 次处理** 会做一次 **publishGlobalMap**，随地图增大从 **~350ms → ~1s → ~2.1s**，是当前主要耗时。 |
| **各环节是否独立** | **进程/线程上独立**（前端 fast_livo、LivoBridge 回调、后端 worker、HBA、回环检测），但 **后端单线程内** 串行执行：取帧 → tryCreateKeyFrame → publishStatus/DataFlow/**publishGlobalMap**，故 **publishGlobalMap 会阻塞整条后端流水线**。 |
| **优化方向** | 降低/异步化 publishGlobalMap、适当增大 sensor_idle 或“bag 播完再结束”策略、可选增大 process_every_n_frames。 |

---

## 1. 背景与日志概况

- **日志**: `logs/full.log`（约 34862 行）
- **运行**: 离线 bag `street_03_ros2`，rate=0.5，config=system_config_M2DGR.yaml
- **时间**: 22:57:06 启动 → 23:02:27 automap 退出 → 23:08:55 ros2 bag 进程结束

---

## 2. “没有执行完成就结束”的原因

### 2.1 实际结束链路（来自日志）

```
23:02:25 [PIPELINE] event=sensor_idle_timeout idle_sec=10.0 timeout=10.0 submaps=3 (→ final HBA + save + rclcpp::shutdown)
23:02:25 [PIPELINE] event=sensor_idle_final_hba_enter submaps=3
23:02:25 [PIPELINE] event=sensor_idle_final_hba_done
23:02:25 [PIPELINE] event=sensor_idle_save_enter output_dir=/data/automap_output
23:02:27 [PIPELINE] event=save_pcd path=/data/automap_output/global_map.pcd points=440559
23:02:27 [PIPELINE] event=sensor_idle_save_done
23:02:27 [AutoMapSystem] Sensor idle: requesting context shutdown (end mapping)
23:02:27 [PIPELINE] event=backend_worker_exited
```

- **触发条件**：`backendWorkerLoop` 中 `frame_queue_` 空且 `wait_for(2s)` 超时，重复多次后 `idle_sec >= sensor_idle_timeout_sec`（日志里为 **10.0s**）。
- **含义**：约 **23:02:15** 之后后端已不再收到新点云（LivoBridge 不再 push 帧），队列被消费完后一直空，10s 后触发“传感器空闲 → 最终 HBA → 保存 → shutdown”。

### 2.2 与 bag 结束的关系

- **automap 退出**: 23:02:27  
- **ros2 bag 进程结束**: 23:08:55（约 6 分钟后）

说明：**bag 仍在播放**（主要是 IMU 等），但 **LiDAR 或 /cloud_registered 已不再向 automap 输入**（可能 bag 中 LiDAR 段较短、或 rate 下前半段播完后长时间无点云）。  
因此“没有执行完成”更准确地说：**建图在“传感器空闲”逻辑下正常保存并退出，但只处理了 bag 中前一段有 LiDAR 的数据**（约 270s bag 时间，142 KF，3 submap）。

### 2.3 配置注意

- 若希望 **bag 播完再结束**，可：
  - 增大 **sensor_idle_timeout_sec**（如 60～120），或
  - 在 **离线模式** 下使用“bag 播完后由 launch 触发保存并 shutdown”，而不是仅依赖传感器空闲。

---

## 3. 后端“每 5 帧处理一帧”与单帧耗时

### 3.1 逻辑说明（代码）

- **process_every_n_frames = 5**（config: `backend.process_every_n_frames`）。
- 后端 **每帧都从队列 pop**（不阻塞前端），但只有 `(frame_no - 1) % 5 == 0` 时才执行 **tryCreateKeyFrame**；即每 5 帧中只对 1 帧做 KF 创建与后续流水线。

### 3.2 单次“处理一帧”的耗时（日志）

- 日志中 **duration_ms** 仅统计 **tryCreateKeyFrame** 一段（含 odom 对齐、KF 判定、子图/ISAM2/回环/HBA 触发等），**不包含** 同周期内的 publishStatus / publishDataFlow / **publishGlobalMap**：

| processed_no | 时间 (wall) | duration_ms (tryCreateKeyFrame) |
|--------------|-------------|----------------------------------|
| 1～5         | 22:57:07～11 | 0.0                             |
| 100          | 22:58:47    | 0.0                             |
| 200          | 23:00:27    | 0.0                             |
| 300          | 23:02:09    | 0.0                             |

结论：**单次 tryCreateKeyFrame 远小于 1ms**（四舍五入为 0.0）。

### 3.3 每 100 次处理的实际周期（含 publishGlobalMap）

同一次循环内还会按 `processed_no % 100 == 0` 执行 **publishGlobalMap**，该步骤包含 buildGlobalMap + voxelDownsampleChunked + 发布，**随地图规模增大明显变慢**：

| processed_no | publishGlobalMap enter → map_published | 耗时（约） |
|--------------|----------------------------------------|------------|
| 100         | 22:58:47.800 → 22:58:48.152           | **~350 ms** |
| 200         | 23:00:27.212 → 23:00:28.291           | **~1.08 s** |
| 300         | 23:02:09.181 → 23:02:11.344           | **~2.16 s** |

因此：

- **纯“处理一帧”（tryCreateKeyFrame）**：约 **&lt;1 ms/帧**。
- **若把“每 100 帧一次 map 发布”摊到每帧**：约 3.5～21 ms/帧（随地图增大而升高）。
- **后端每 5 帧才处理 1 帧**：即每 5 帧中只有 1 帧会走 tryCreateKeyFrame；若该帧恰好是 100 的倍数，还会多出 0.35～2.16s 的 map 发布。  
  **平均下来**：在 300 帧附近，每“处理一帧”的等效周期约 **7～8 ms**（2.16s/100 + 少量 tryCreateKeyFrame），但 **峰值** 每 100 帧会有一帧周期达 **2s+**。

---

## 4. 各环节是否“相互独立”

### 4.1 进程/线程划分

| 环节 | 运行位置 | 说明 |
|------|----------|------|
| 前端 (fast_livo) | 独立进程 | 订阅 bag 的 /velodyne_points、/handsfree/imu，发布 /aft_mapped_to_init、/cloud_registered |
| LivoBridge | automap 进程内，ROS 回调线程 | 订阅 odom/cloud/kfinfo，写 frame_queue_、odom/kfinfo 缓存；队列满时背压等待 |
| 后端 worker | automap 进程内，独立线程 `backendWorkerLoop` | 从 frame_queue_ 取帧，每 5 帧处理 1 帧，串行执行 tryCreateKeyFrame → publishStatus/DataFlow/GlobalMap |
| HBA | 独立线程 | 异步优化，不阻塞 backend worker |
| 回环检测 (LoopDetector) | 独立 worker 线程 | 异步，结果通过回调写回 |

所以：**从进程和线程看，前端、LivoBridge、后端 worker、HBA、回环是分开的**。

### 4.2 数据与执行耦合点

- **后端单线程内串行**：取帧 → **tryCreateKeyFrame** → publishStatus（每 10 帧）→ publishDataFlowSummary（每 50 帧）→ **publishGlobalMap（每 100 帧）**。  
  因此 **publishGlobalMap 会阻塞同一线程**，这段时间内不会处理新帧，队列会堆积（若前端持续发）。
- **背压**：队列满时 LivoBridge 在 `frame_queue_.push` 前等待 `frame_queue_not_full_cv_`，**不会丢帧**，但会拖慢 ROS 回调（进而可能影响 bag 消费速度）。
- **共享状态**：odom 缓存、kfinfo 缓存、submap_manager、isam2_optimizer 等由后端 worker 与 HBA/回环回调等共享，存在锁/原子访问，但主要瓶颈仍是 **后端单线程 + publishGlobalMap**。

结论：**环节在进程/线程上独立，但后端一条线程内“处理一帧”和“发布全局图”是串行的，publishGlobalMap 是当前主要阻塞点**。

---

## 5. 优化建议（可落地、可配置）

### 5.1 降低或异步化 publishGlobalMap（优先，已实现异步）

- **已实现异步化**：backend worker 每 100 帧仅设置 `map_publish_pending_` 并 notify；专用 **map_publish_thread_** 在 `map_build_mutex_` 下执行 buildGlobalMap + publish，不阻塞 tryCreateKeyFrame 循环；与 tryCreateKeyFrame 通过同一 mutex 串行化读写，保证地图一致性。
- **降低频率**：可将“每 100 帧发布一次”改为 **每 200 帧或 300 帧**（config: `backend.publish_global_map_every_n_processed`），减少 build 次数。
- **增量/简化发布**：若 RViz 不需要每 100 帧就全量刷新，可只发布增量或降采样后的子图，减少单次 build 规模。

### 5.2 离线模式“播完再结束”（已实现）

- **配置**：`system.offline_finish_after_bag: true` 时不再依赖传感器空闲自动结束，仅由 `/automap/finish_mapping` 服务触发。
- **服务**：`/automap/finish_mapping`（std_srvs/srv/Trigger）执行最终 HBA、保存地图并 `rclcpp::shutdown()`。
- **Launch**：`automap_offline.launch.py` 在 ros2 bag play 进程退出时通过 `RegisterEventHandler(OnProcessExit(...))` 调用该服务（先 sleep 5s 再调用，留时间排空队列），实现“播完再结束”。
- 可选：仍可增大 **sensor_idle_timeout_sec**（如 60～120）作为兜底。

### 5.3 后端负载与 process_every_n_frames

- 当前 **process_every_n_frames=5** 已明显降低 KF 数量；若仍希望进一步降负载，可改为 **7 或 10**（会降低轨迹密度与回环机会，需权衡）。
- **单帧 tryCreateKeyFrame** 本身很快（&lt;1ms），进一步增大 N 对单帧耗时影响不大，主要收益是减少 HBA/回环等下游压力。

### 5.4 观测与排障

- 在 **每 100 帧** 的日志中增加 **本周期总耗时**（含 publishGlobalMap），便于直接看到“每 100 帧一次”的峰值时间（例如 2s+）。
- 保留或增加 **DATA_FLOW** 中的 queue 长度、dropped、backend_frames，便于判断是否因 publishGlobalMap 阻塞导致队列持续堆积。

---

## 6. 小结表

| 问题 | 结论 |
|------|------|
| 为何没执行完就结束？ | 传感器空闲 10s 触发自动保存并 shutdown；此时 bag 仍在播但已无 LiDAR 输入，故只处理了前约 270s。 |
| 后端每 5 帧处理 1 帧，处理一帧要多久？ | tryCreateKeyFrame **&lt;1 ms**；每 100 次处理会多一次 publishGlobalMap，**约 0.35～2.16 s**（随地图增大），摊到每帧约数 ms～二十几 ms，峰值 2s+。 |
| 各环节是否独立？ | 进程/线程独立；后端单线程内 **串行**，publishGlobalMap **阻塞** 整条后端流水线，是主要瓶颈。 |
| 建议优化 | 降低/异步化 publishGlobalMap；离线模式延长 sensor_idle 或改为“bag 播完再结束”；按需调大 process_every_n_frames；加强每 100 帧周期耗时与队列长度观测。 |

---

## 7. 附录：关键日志行号（grep 参考）

- `sensor_idle_timeout`: 6414  
- `sensor_idle_save_done` / `backend_worker_exited`: 6802, 6791  
- `process_every_n_frames=5`: 152  
- `BACKEND worker processed #N duration_ms`: 226, 272, 301, 316, 334, 1671, 3504, 5358  
- `publishGlobalMap enter` / `map_published`: 1675, 1855, 3508, 3710, 5362, 5565  
