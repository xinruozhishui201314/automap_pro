# 轨迹 CSV 中 GPS 全为 0/none 原因分析

## Executive Summary

| 结论 | 说明 |
|------|------|
| **现象** | `trajectory_odom_*.csv` 中 `gps_x,gps_y,gps_z` 全为 0，`gps_frame` 为 `none`，`gps_valid` 为 0。 |
| **根因** | 写入时 `gps_manager_.queryByTimestamp(ts)` 未返回有效 GPS，导致始终走“无 GPS”分支。可能原因：**① 没有任何 GPS 数据进入系统**（未订阅或消息被丢弃）；**② 时间戳匹配窗口过窄**（0.1s）导致 1Hz GPS 与 10Hz 里程计无法匹配。 |
| **关键证据** | 同次运行未生成 `trajectory_gps_*.csv` → 说明 `addGPSMeasurement` 从未被调用，或调用后未触发日志回调，即 **GPS 数据源未进入 GPSManager**。 |

---

## 1. 数据流与写入逻辑

### 1.1 轨迹 CSV 写入链

```
LivoBridge::onOdometry(odom_msg)
  → AutoMapSystem::onOdometry(ts, pose, cov)
    → writeTrajectoryOdom(ts, pose, cov)
      → gps_opt = gps_manager_.queryByTimestampForLog(ts, 0.5)  // 轨迹日志用 0.5s 窗口
      → 若 gps_opt && gps_opt->is_valid：写 gps_x/y/z, frame, gps_valid=1
      → 否则：写 0, 0, 0, "none", 0
```

- **ts 来源**：里程计消息 `msg->header.stamp`（与 fast_livo 发布的 `/aft_mapped_to_init` 一致，通常为 LiDAR/ bag 时间）。
- **GPS 来源**：`LivoBridge` 订阅 `sensor.gps.topic`（M2DGR 为 `/ublox/fix`），在 `onGPS(NavSatFix)` 中解析后调用 `gps_manager_.addGPSMeasurement(ts, lat, lon, alt, hdop, sats)`。

### 1.2 GPS 写入轨迹的两个条件

1. **有数据**：`queryByTimestamp(ts)` 返回 `std::optional` 有值。
2. **有效**：返回的 `GPSMeasurement::is_valid == true`（即 `quality >= GPSQuality::MEDIUM`，对应 `hdop <= 5.0`）。

任一不满足则 CSV 中该行 GPS 列为 0 和 none。

---

## 2. 根因分析

### 2.0 rosbag2 有 GPS 话题但“接收不到”的常见原因

若 **bag 列表里已有 `/ublox/fix`**（或 `ros2 bag info` 显示该话题存在），但轨迹 CSV 仍无 GPS，按下面顺序排查。

| 原因 | 日志特征 | 处理 |
|------|----------|------|
| **GPS 未启用（最常见）** | 出现 `[LivoBridge][GPS] GPS disabled (sensor.gps.enabled=false)` | 本次运行**未加载带 GPS 的配置**。必须用 **`--config system_config_M2DGR.yaml`** 启动，并确认日志中有 **`[AutoMapSystem][CONFIG] sensor.gps.enabled=true`**。 |
| config_file 未传入 | 出现 `[AutoMapSystem][CONFIG] config_file param=(empty)` 或 `No config file specified` | launch 未传 `config:=` 或路径错误。脚本启动时务必带 `--config system_config_M2DGR.yaml`（或等价 config 路径）。 |
| 部分 ublox 话题被忽略 | `[rosbag2_player] Ignoring a topic '/ublox/...' reason: package 'ublox_msgs' not found` | 仅影响 **ublox_msgs** 类型话题；**`/ublox/fix` 为 sensor_msgs/NavSatFix**，不会被忽略，会正常发布。可忽略该 WARN。 |
| 启动顺序 / QoS | 无 `First GPS message received` 且 enabled=true | bag 先于节点发布且 VOLATILE，或 QoS 不匹配。可尝试延迟 bag 播放或放宽订阅 QoS（见 4.2）。 |

**本次 full.log 根因**：日志第 167 行为 **`[LivoBridge][GPS] GPS disabled (sensor.gps.enabled=false)`**，说明**未启用 GPS**，不是 bag 没发。请确认启动命令包含 `--config system_config_M2DGR.yaml`，并查看同一段日志中是否有 `[AutoMapSystem][CONFIG] Config loaded from ...` 和 **sensor.gps.enabled=true**。

### 2.1 无 GPS 数据进入系统（最可能）

**证据**：同目录下**没有** `trajectory_gps_<session_id>.csv`。

- `trajectory_gps_*.csv` 在 `onGPSMeasurementForLog` 中写入，该回调在 **每次** `GPSManager::addGPSMeasurement` 内被调用。
- 若该文件不存在或为空，说明 **没有一次** GPS 测量被加入 `gps_window_`，即：
  - 要么 **GPS 订阅未建立**（如配置未生效 `sensor.gps.enabled=false`）；
  - 要么 **话题名/ bag 中无该话题**（如 bag 里没有 `/ublox/fix` 或名字不一致）；
  - 要么 **每条 NavSatFix 都被 LivoBridge 丢弃**。

**LivoBridge::onGPS 丢弃条件**（`livo_bridge.cpp`）：

```cpp
if (msg->status.status < 0) return;  // GPS fix 无效
```

- `sensor_msgs/msg/NavSatFix` 中 `status.status`：**-1 = STATUS_NO_FIX**，0/1/2 表示有 fix。
- 若 bag 中所有 GPS 消息的 `status.status == -1`（无定位），则 **所有消息都被丢弃**，不会调用 `addGPSMeasurement`。

### 2.2 时间戳匹配窗口过窄（有 GPS 时仍可能全 0）

`GPSManager::queryByTimestamp` 逻辑（`gps_manager.cpp`）：

```cpp
double best_dt = 0.1;   // 仅接受 0.1 秒内最近一条
for (const auto& r : gps_window_) {
    double dt = std::abs(r.timestamp - ts);
    if (dt < best_dt) { best_dt = dt; best = &r; }
}
```

- 里程计约 **10 Hz**，GPS 常见 **1 Hz**。
- 若 GPS 与 odom 使用**不同时间基准**（如 GPS 为接收机时间、odom 为 bag 回放时间），可能存在固定时间差，导致所有 odom 的 `ts` 与任意 GPS 的 `r.timestamp` 都 **> 0.1 s**，从而 `best` 始终为 `nullptr`，`queryByTimestamp` 始终返回 `std::nullopt`。

因此即使有 GPS 数据进入系统，**0.1 s 窗口**在 1 Hz GPS + 时间偏差下也可能导致轨迹 CSV 中 GPS 全为 0。

### 2.3 质量导致 is_valid 全为 false（次要）

- `is_valid = (best->quality >= GPSQuality::MEDIUM)`，`hdop <= 5.0` 为 MEDIUM。
- LivoBridge 中 `hdop = std::max(0.5, sigma_h / 0.3)`，若 `position_covariance` 未设置或很小，hdop 会偏小，质量通常至少 MEDIUM。因此 **质量导致全无效** 的概率低于“无数据”和“时间不匹配”。

---

## 3. 结论汇总

| 原因 | 可能性 | 说明 |
|------|--------|------|
| Bag 无 `/ublox/fix` 或话题名不一致 | 高 | 未订阅到任何 GPS → gps_window_ 始终为空。 |
| 所有 NavSatFix 的 `status.status < 0` | 高 | 无有效 fix，LivoBridge 全部 return，不调用 addGPSMeasurement。 |
| 配置未生效（gps.enabled=false 或未加载 M2DGR 配置） | 中 | 若未用 `--config system_config_M2DGR.yaml` 或路径错误，会使用默认 false。 |
| 时间戳匹配窗口 0.1 s 过窄 | 中 | 有 GPS 但 odom 与 GPS 时间差 > 0.1 s → queryByTimestamp 永远无匹配。 |
| 所有 GPS 质量 < MEDIUM | 低 | 会匹配到但 is_valid 为 0，仍写 0/none（需先有匹配）。 |

当前最符合现象的解释是：**没有任何 GPS 测量进入 GPSManager**（无 `trajectory_gps_*.csv`），即上述“无数据”类原因之一成立。

---

## 4. 建议措施

### 4.1 验证（必做）

1. **确认 bag 中是否有 GPS 话题及消息数**  
   ```bash
   ros2 bag info data/automap_input/M2DGR/street_03_ros2 --yaml
   # 或
   python3 automap_pro/scripts/diagnose_and_fix_bag.py data/automap_input/M2DGR/street_03_ros2 --list-topics --verbose
   ```
   - 确认存在 `/ublox/fix`（或与 `sensor.gps.topic` 一致的话题）。
   - 确认该话题消息数 > 0。

2. **确认运行使用的配置**  
   - 启动命令中是否包含 `--config system_config_M2DGR.yaml`。
   - 配置中 `sensor.gps.enabled: true` 且 `sensor.gps.topic: "/ublox/fix"`（与 bag 一致）。

3. **查看运行日志**  
   - 搜索 `[LivoBridge][TOPIC]`：应看到 `gps=/ublox/fix`，若为 `gps=disabled` 则 GPS 未启用。
   - 搜索 `[LivoBridge][DATA] gps count=`：若有打印说明有 NavSatFix 被收到；若无，说明未收到或全部在 `status.status < 0` 时 return。

### 4.2 代码侧改进（可选但推荐）

1. **放宽轨迹日志用的时间匹配窗口**  
   - 在 **仅用于写轨迹 CSV** 的逻辑中，对 `queryByTimestamp` 使用更大窗口（例如 0.5 s 或 1.0 s），或单独实现一个 `queryByTimestampForLog(ts, max_dt)`，避免 1 Hz GPS 因小时间差全部匹配失败。  
   - 关键帧/优化等仍可继续使用当前 0.1 s 逻辑。

2. **无匹配时的诊断日志**  
   - 在 `writeTrajectoryOdom` 中，若 `trajectory_log_enabled_` 且 `!gps_opt`，可每 N 帧打印一次：当前 `ts`、`gps_window_` 大小、最近一条 GPS 的 timestamp，便于确认是“无数据”还是“时间对不上”。

3. **NavSatFix 被丢弃时打日志**  
   - 在 `LivoBridge::onGPS` 中，当 `msg->status.status < 0` 时，可每 50 条打印一次，便于确认是否因 status 导致全部丢弃。

4. **可选：无有效 GPS 时仍写最近一条**  
   - 若希望 CSV 中至少能看到“最近 GPS”（便于排查时间对齐），可对轨迹日志单独支持：当 `gps_opt` 有值但 `!gps_opt->is_valid` 时，仍写入 gps_x/y/z 并设 `gps_valid=0`，便于绘图或脚本区分“无数据”和“有数据但无效”。

---

## 5. 验证清单

- [ ] `ros2 bag info` / `diagnose_and_fix_bag.py --list-topics` 显示存在 `/ublox/fix` 且消息数 > 0。
- [ ] 启动参数包含 `--config system_config_M2DGR.yaml`，且其中 `sensor.gps.enabled: true`、`sensor.gps.topic: "/ublox/fix"`。
- [ ] 运行日志中出现 `[LivoBridge][DATA] gps count=...`（说明有 GPS 消息进入回调）。
- [ ] 同次运行生成 `trajectory_gps_<session_id>.csv` 且非空（说明 addGPSMeasurement 被调用）。
- [ ] 若以上均正常仍全 0：检查 odom 与 GPS 时间戳差，并考虑放宽轨迹用 queryByTimestamp 窗口或使用 `queryByTimestampForLog(ts, 0.5)`。

完成上述验证与可选代码改动后，可复跑同一条 bag，再检查 `trajectory_odom_*.csv` 中 `gps_valid=1` 是否出现及 `trajectory_gps_*.csv` 是否生成。

---

## 6. 日志排查指南（强化日志后）

运行后若轨迹 CSV 仍无 GPS，按下列顺序查日志，可精确定位环节。

### 6.1 一键 grep 命令

```bash
# 所有 GPS 相关诊断（LivoBridge 订阅/首包/丢弃 + GPSManager 首条 + 轨迹无匹配）
grep -E 'LivoBridge\]\[GPS\]|GPS_DIAG|TRAJ_LOG' logs/automap.log

# 仅看“无 GPS”原因（轨迹写时为何没匹配到）
grep 'TRAJ_LOG no GPS' logs/automap.log
```

### 6.2 日志含义速查

| 日志 tag | 含义 | 若轨迹无 GPS 时说明 |
|----------|------|----------------------|
| `[LivoBridge][GPS] Subscription created: topic=...` | GPS 已启用并订阅该 topic | 若没有此行、却有 `GPS disabled` → 配置 `sensor.gps.enabled=false` 或未加载对应 config。 |
| `[LivoBridge][GPS] First GPS message received (any status): status=X ts=...` | **任意首条** GPS 消息（不论是否有效） | 有则说明订阅已收到至少一条；若连此行都没有 → bag 未发该 topic 或 automap 先结束。 |
| `[LivoBridge][GPS] First GPS message has no fix (status=-1)...` | 收到的第一条 GPS 无定位 | Bag 里该 topic 的 NavSatFix 全部 status&lt;0，全部被丢弃，不会写 trajectory_gps_*.csv。 |
| `[LivoBridge][GPS] First valid GPS received: ts=...` | 第一条有效 fix 已收到并转发 | 说明 LivoBridge→GPSManager 通路正常；若仍无 CSV GPS，看下游。 |
| `[LivoBridge][DATA] gps count=...` | 周期性有效 GPS 计数 | 有增长说明持续有有效 fix。 |
| `[GPS_DIAG] First GPS measurement added: ...` | GPSManager 首次调用 addGPSMeasurement | 若始终没有 → 要么 LivoBridge 未收到有效 fix，要么回调未注册。 |
| `[GPS_DIAG] GPS measurement #N ...` | 每 100 条 GPS 测量 | 有则说明数据在 gps_window_ 中积累。 |
| `[TRAJ_LOG] First trajectory row: odom_ts=...` | 首行轨迹写入时的 odom 时间 | 与 `gps_ts_range` 对比可判断 odom 与 GPS 时间是否在同一量级、是否重叠。 |
| `[TRAJ_LOG] no GPS for odom ts=... gps_window_size=N` | 写轨迹时未匹配到有效 GPS | **gps_window_size=0**：无任何 GPS 进系统（见上）；**gps_window_size>0** 且 reason=**no match within 0.5s**：时间戳偏差大，可尝试放宽到 1.0s 或检查 bag 时间源；**quality<MEDIUM**：匹配到但 hdop 差。 |
| `[TRAJ_LOG] no GPS ... gps_ts_range=[min, max]` | 无匹配且窗口非空时打印 GPS 时间范围 | 若 odom_ts 明显在 [min,max] 外 → 时间基准不一致；若在范围内仍 no match → 0.5s 窗口内无最近邻（可试 1.0s）。 |

### 6.3 典型组合判断

- **只有 `Subscription created`，没有 `First valid GPS` 也没有 `First GPS message has no fix`**  
  → Bag 可能未发布该 topic，或回放顺序/时间导致 automap 先跑完未收到任何一条。用 `ros2 bag info <bag>` 确认该 topic 存在且消息数>0。

- **有 `First GPS message has no fix (status=-1)`，没有 `First valid GPS`**  
  → Bag 中该 topic 所有消息 status 均为 -1，需换有有效 fix 的 bag 或检查数据集说明。

- **有 `First valid GPS` 和 `GPS_DIAG] First GPS measurement added`，但 `TRAJ_LOG no GPS` 且 gps_window_size>0、reason=no match within 0.5s**  
  → 时间对齐问题：看同条日志中的 **gps_ts_range=[min,max]**，若 odom_ts 不在该范围内说明时间基准不一致；若在范围内仍无匹配可把轨迹用 `queryByTimestampForLog(ts, 1.0)` 再试，或检查 bag 的 use_sim_time / 各 topic 的 header.stamp 来源。
