# 地图未建完就结束 — automap.log 分析报告

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **直接原因** | 流水线检测到 **传感器空闲超时**（10 秒内未收到新的点云），自动执行「最终 HBA → 保存 → 退出」。 |
| **是否崩溃** | 否。Automap 进程 **正常退出**（`exited normally`）；容器退出码 130 来自之后用户 **Ctrl+C**。 |
| **建议** | 若希望等 bag 全部播完再结束，可增大 `sensor_idle_timeout_sec` 或关闭 `auto_finish_on_sensor_idle`；并排查为何 10s 内无点云（bag 间隙 / LIVO 停发）。 |

---

## 1. 时间线与关键日志

### 1.1 结束触发时刻（22:02:02）

```text
[AutoMapSystem][PIPELINE] event=sensor_idle_timeout idle_sec=10.0 timeout=10.0 submaps=3 (→ final HBA + save + rclcpp::shutdown)
[AutoMapSystem][PIPELINE] event=sensor_idle_final_hba_enter submaps=3
[HBA][BACKEND] hba_api not available, skipping optimization
[AutoMapSystem][PIPELINE] event=sensor_idle_save_enter output_dir=/data/automap_output
… buildGlobalMap → 保存 global_map.pcd (484751 points)、trajectory_tum.txt (166 poses)、session …
[AutoMapSystem][PIPELINE] event=sensor_idle_save_done output_dir=/data/automap_output
[AutoMapSystem] Sensor idle: requesting context shutdown (end mapping)
```

含义：**连续 10 秒没有新的“传感器数据”（即进入 AutoMapSystem 的点云）**，且帧队列为空，于是按设计执行「结束建图」流程并正常退出。

### 1.2 退出过程（22:02:05–22:02:06）

- Backend worker 退出 → HealthMonitor/ErrorMonitor 停止 → 各线程依次退出。
- `[Inferior 1 (process 261) exited normally]` → **进程为正常退出**。
- 之后日志中大量 `[fastlivo_mapping-2] imu time stamp Jumps`：说明 **bag 仍在播**（至少 IMU 在播），但 automap 已关闭。

### 1.3 容器退出码 130（22:02:58）

- 日志中出现 `^C^C^C^C`，随后 `容器退出码: 130`。
- 130 = 128 + 2，即 **SIGINT（Ctrl+C）**。即：**地图结束并保存后，用户手动中断了容器**，不是 automap 崩溃。

---

## 2. 根因说明：“传感器空闲”判的是什么？

- **“传感器数据”** 在代码里指：**通过 `onCloud()` 进入 AutoMapSystem 的点云**（即 LivoBridge 转发给后端的那路点云）。
- **`last_sensor_data_wall_time_`** 仅在 **每收到一帧点云** 时更新（`automap_system.cpp` 约 401 行）。
- 逻辑（约 506–517 行）：backend  worker 用 `wait_for(2s)` 等帧；若 **超时且队列仍空** 且 **距上次点云 ≥ 10s**，则触发 `sensor_idle_timeout`。

因此：“地图没建完就结束” = **在某一时刻起，连续 10 秒没有新的点云进入 AutoMapSystem**，系统认为“数据流结束”并自动收尾。

可能原因包括：

1. **Bag 中该段时间没有 lidar 数据**（例如 10s+ 的间隙或 bag 已播完）。
2. **LIVO 在这 10s 内没有输出点云**（例如因 `imu time stamp Jumps` 等导致内部暂停/丢帧），即使 bag 仍在播 IMU。
3. **LivoBridge/话题连接** 异常，导致点云未进入 automap（相对少见，需结合 topic/频率排查）。

当前日志中，在触发 idle 前后都有大量 **fast_livo `imu time stamp Jumps`**，说明 bag 的 IMU 与 host 时间或 bag 内时间存在大跳变，**不能排除 LIVO 因时间跳变在一段时间内未输出点云**，从而触发 10s 空闲。

---

## 3. 配置与代码位置

| 配置项 | 文件 | 当前值 | 含义 |
|--------|------|--------|------|
| `sensor_idle_timeout_sec` | `automap_pro/config/system_config.yaml` | `10.0` | 超过该秒数无点云且队列空 → 结束建图 |
| `auto_finish_on_sensor_idle` | 同上 | `true` | 是否启用“传感器空闲时自动结束” |

相关代码：

- `automap_pro/src/system/automap_system.cpp`：`last_sensor_data_wall_time_` 更新（约 401 行）、idle 判断与 final HBA/save/shutdown（约 494–530 行）。

---

## 4. 建议措施

### 4.1 若希望“尽量建完整个 bag”再结束

- **加大空闲超时**：例如将 `sensor_idle_timeout_sec` 改为 `60.0` 或 `120.0`，避免 bag 中短间隙或 LIVO 短暂停发就结束。
- **或暂时关闭自动结束**：设 `auto_finish_on_sensor_idle: false`，建图不会因空闲自动退出，需手动触发保存/退出（或依赖其它结束条件）。

### 4.1b 若怀疑是“计算跟不上、卡住”导致被误判为空闲

- **增大帧队列缓冲**：在 `system_config.yaml` 中设置 `frame_queue_max_size: 1500` 或 `2000`（默认 500），允许后端“算慢一点”时在队列里多缓冲帧，减少因瞬时断流或处理延迟触发的空闲结束。
- **同时拉长空闲超时**：例如 `sensor_idle_timeout_sec: 30.0` 或 `60.0`，在计算较慢、队列偶尔排空时不易误判为“数据结束”。
- 启动日志中会打印 `frame_queue_max_size` 与 `sensor_idle_timeout_sec`，便于确认配置已生效。

### 4.2 若希望保持 10s 自动结束，但要避免“误判”

- 确认 bag 中 **lidar 话题** 在结束前 10s 内是否仍有数据：  
  `ros2 bag play <bag> --topics /os1_cloud_node1/points`，并用 `ros2 topic hz /os1_cloud_node1/points` 看是否有大于 10s 的断档。
- 若 lidar 连续有数据但 automap 仍 10s 收不到点云，则问题在 **LIVO 或 LivoBridge**（例如 IMU 时间跳变导致 LIVO 一段时间不发布 `/cloud_registered`），需从 LIVO 日志与话题频率排查。

### 4.3 本次运行的其他注意点

- **HBA 未真正执行**：`[HBA][BACKEND] hba_api not available, skipping optimization`，最终轨迹未做 HBA 优化，若需后端优化需保证 hba_api 可用。
- **IMU 时间跳变**：日志中大量 `imu time stamp Jumps`，建议检查 bag 的 IMU 时间戳与 playback 时钟（`--clock`）设置，减少跳变，避免 LIVO 异常停发点云。

---

## 5. 小结

- 地图**没有建完就结束**的直接原因是：**传感器空闲超时（10s 无点云）** 触发自动结束流程，而不是进程崩溃。
- 若 bag 或 LIVO 在某一阶段**超过 10s 没有点云**进入 AutoMapSystem，就会出现“地图提前结束”的现象。
- 通过增大 `sensor_idle_timeout_sec` 或关闭 `auto_finish_on_sensor_idle`，并配合 bag/LIVO 的排查，可以避免非预期的提前结束。
