# 轨迹对比记录与绘图（建图精度分析）

## 1. 功能说明

系统在运行期间将**每帧位姿**（odom）与**每条 GPS 测量**写入带时间戳的 CSV 文件，便于用脚本绘制曲线对比分析，从而评估点云建图相对 GPS 的精度。

- **Odom 文件**：每收到一帧里程计写入一行（时间戳、位姿、位置标准差）。
- **GPS 文件**：每收到一条 GPS 测量写入一行（时间戳、位置；已对齐时为地图系，未对齐时为 ENU）。

## 2. 输出文件

| 文件 | 含义 | 表头 |
|------|------|------|
| `trajectory_odom_<session_id>.csv` | 建图轨迹（每帧） | timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z |
| `trajectory_gps_<session_id>.csv`  | GPS 轨迹 | timestamp,x,y,z,frame |

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

## 6. 变更文件清单

- `automap_pro/include/automap_pro/frontend/gps_manager.h`：`registerMeasurementLogCallback`
- `automap_pro/src/frontend/gps_manager.cpp`：每条 GPS 测量后调用回调
- `automap_pro/include/automap_pro/system/automap_system.h`：轨迹文件与写方法声明
- `automap_pro/src/system/automap_system.cpp`：`ensureTrajectoryLogDir`、`writeTrajectoryOdom`、`onGPSMeasurementForLog`，以及 onOdometry 中调用写 odom、参数声明与 GPS 回调注册
- `automap_pro/scripts/plot_trajectory_compare.py`：绘图脚本（新增）
