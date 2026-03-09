# 轨迹对比记录与绘图（建图精度分析）

## 1. 功能说明

系统在运行期间将**每帧位姿**（odom）与**每条 GPS 测量**写入带时间戳的 CSV 文件，便于用脚本绘制曲线对比分析，从而评估点云建图相对 GPS 的精度。

- **Odom 文件**：每收到一帧里程计写入一行（时间戳、位姿、位置标准差）。
- **GPS 文件**：每收到一条 GPS 测量写入一行（时间戳、位置；已对齐时为地图系，未对齐时为 ENU）。

## 2. 输出文件

| 文件 | 含义 | 表头 |
|------|------|------|
| `trajectory_odom_<session_id>.csv` | 建图轨迹（每帧）+ 对应GPS信息（便于对比分析） | timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid |
| `trajectory_gps_<session_id>.csv`  | GPS 轨迹 | timestamp,x,y,z,frame |

### 2.1 trajectory_odom_<session_id>.csv 字段说明

| 字段 | 类型 | 含义 | 单位/取值 |
|------|------|------|-----------|
| timestamp | double | 时间戳（秒） | - |
| x, y, z | double | 里程计位置（map frame） | 米 |
| qx, qy, qz, qw | double | 里程计姿态四元数 | - |
| pos_std_x, pos_std_y, pos_std_z | double | 位置标准差（从协方差计算） | 米 |
| gps_x, gps_y, gps_z | double | 对应GPS位置（map/enu frame） | 米（若无GPS则为0） |
| gps_frame | string | GPS坐标系 | "map"（已对齐）或 "enu"（未对齐）或 "none" |
| gps_valid | int | GPS是否有效 | 1（有效）或 0（无效/无数据） |

### 2.2 GPS与里程计对比分析

轨迹日志现已包含GPS信息，可直接在 `trajectory_odom_*.csv` 文件中对比里程计与GPS位置，无需额外匹配时间戳。

**使用方法**：
```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取里程计日志（包含GPS）
df = pd.read_csv('trajectory_odom_20260308_172644.csv')

# 提取有效GPS数据
gps_valid = df[df['gps_valid'] == 1]

# 绘制对比图
plt.figure(figsize=(12, 8))
plt.plot(df['x'], df['y'], 'b-', label='Odom trajectory', alpha=0.7)
plt.scatter(gps_valid['gps_x'], gps_valid['gps_y'], c='red', s=5, label='GPS points', alpha=0.5)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Odom vs GPS Trajectory Comparison')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('odom_vs_gps.png', dpi=150)
```

- `session_id` 为本次运行启动时间，格式 `YYYYMMDD_HHMMSS`，与 odom/gps 成对出现。
- 默认目录：`AUTOMAP_LOG_DIR` 环境变量或参数 `trajectory_log_dir`，未设置时为 `logs`。

## 3. 参数

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `trajectory_log_enable` | bool | true | 是否写入轨迹 CSV |
| `trajectory_log_dir`    | string | "" | 输出目录（空则用 AUTOMAP_LOG_DIR 或 `logs`） |

在 launch 或 config 中可覆盖，例如：

```yaml
automap_system:
  ros__parameters:
    trajectory_log_enable: true
    trajectory_log_dir: "/path/to/logs"
```

## 4. 绘图脚本

使用仓库自带脚本对比两条轨迹并出图：

```bash
# 指定 odom 与 gps 文件
python3 automap_pro/scripts/plot_trajectory_compare.py \
  --odom logs/trajectory_odom_20260307_204500.csv \
  --gps  logs/trajectory_gps_20260307_204500.csv \
  --out  trajectory_compare.png

# 或指定日志目录，自动找最新一对文件
python3 automap_pro/scripts/plot_trajectory_compare.py --dir logs --out compare.png
```

依赖：`pip install pandas matplotlib`

输出图为 2×2：XY 平面轨迹、X-t、Y-t、Z-t。蓝线为建图轨迹，红点为 GPS，可直观对比偏差与漂移。

## 5. 如何反映建图精度

- **XY 平面**：odom 与 GPS 点越贴合，说明建图轨迹与 GPS 一致性好（在 GPS 已对齐到地图系的前提下）。
- **X/Y/Z - 时间**：若建图存在漂移，会看到蓝线与红线随时间拉大；若存在尺度或平移偏差，会看到整体偏移。
- 结合 `[PRECISION][LIO]`、`[PRECISION][GPS]` 等日志中的 rmse、pos_std，可定量与曲线定性结合分析建图精度。

## 6. GPS 列「每 5 行无值」现象（原因与修复）

**现象**：`trajectory_odom_*.csv` 中 `gps_x/gps_y/gps_z` 与 `gps_frame` 呈规律交替——约每 5 行有 GPS 数据、接下来 5 行为 `none`/0。

**原因**：
- 轨迹按**每条 odom** 写行（约 10 Hz），GPS 通常 **1 Hz**。
- 原逻辑用 0.5s 时间窗匹配：仅当 `|odom_ts - gps_ts| < 0.5` 才写入 GPS；且边界 `dt == 0.5` 因判断为 `< best_dt`（初值 0.5）未被接受。
- 相邻两帧 GPS 间隔 1 s，中间约 0.5 s 内的 odom 与前一帧 GPS 的 dt 落在 (0.5, 1.0]，与下一帧 GPS 尚未到达，导致这批 odom 行无匹配 → 出现「5 行有 GPS、5 行 none」的交替。

**修复**（已落地）：
1. **queryByTimestampForLog**：在 `dt <= max_dt_s` 内选**最近**的 GPS（含边界），避免边界漏匹配。
2. **轨迹日志**：匹配窗由 0.5s 改为 **1.0s**，使 1 Hz GPS 下整秒内的 odom 都能匹配到前一/后一帧 GPS，消除交替空洞。

复跑同一条 bag 后，CSV 中有 GPS 的时段应连续有值，不再出现规律性每 5 行空缺。

## 7. 变更文件清单

- `automap_pro/include/automap_pro/frontend/gps_manager.h`：`registerMeasurementLogCallback`
- `automap_pro/src/frontend/gps_manager.cpp`：每条 GPS 测量后调用回调；`queryByTimestampForLog` 边界与时间窗逻辑
- `automap_pro/include/automap_pro/system/automap_system.h`：轨迹文件与写方法声明
- `automap_pro/src/system/automap_system.cpp`：`ensureTrajectoryLogDir`、`writeTrajectoryOdom`（GPS 匹配窗 1.0s）、`onGPSMeasurementForLog`，以及 onOdometry 中调用写 odom、参数声明与 GPS 回调注册
- `automap_pro/scripts/plot_trajectory_compare.py`：绘图脚本（新增）
