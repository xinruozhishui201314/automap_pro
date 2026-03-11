# 轨迹对比记录与绘图（建图精度分析）

## 1. 功能说明

系统在运行期间将**每帧位姿**（odom）与**每条 GPS 测量**写入带时间戳的 CSV 文件，便于用脚本绘制曲线对比分析，从而评估点云建图相对 GPS 的精度。

- **Odom 文件**：每收到一帧里程计写入一行（时间戳、位姿、位置标准差）。
- **GPS 文件**：每收到一条 GPS 测量写入一行（时间戳、位置；已对齐时为地图系，未对齐时为 ENU）。

## 2. 输出文件

| 文件 | 含义 | 表头 |
|------|------|------|
| `trajectory_odom_<session_id>.csv` | 建图轨迹（每帧）+ 对应GPS信息（便于对比分析） | timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid,gps_hdop,gps_quality |
| `trajectory_gps_<session_id>.csv`  | GPS 轨迹（含姿态估计） | timestamp,x,y,z,frame,pitch,roll,yaw,attitude_source,velocity,attitude_valid |
| `gps_positions_map.pcd` | **建图结束时**保存的地图坐标系下 GPS 位置点云（与 global_map.pcd 同目录） | PCD 格式：x,y,z 为地图系坐标（米），intensity 为点索引 |

**姿态估计**：由配置 `gps.has_attitude` 控制。为 **false**（默认）时使用估计姿态：IMU 估计 pitch/roll、GPS 航迹角或里程计估计 yaw；为 **true** 时若接收机通过 `addGPSAttitude` 注入了姿态（双天线/INS），则直接使用。`attitude_source`：0=NONE 1=IMU_ONLY 2=GPS_TRAJECTORY 3=GPS_DUAL_ANTENNA 4=ODOMETRY 5=FUSED。

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
| `trajectory_log_after_mapping_only` | bool | **true** | 为 true 时仅在建图完成（saveMapToFiles）时写 trajectory_odom CSV（关键帧+最终GPS），保证与 GPS 在同一地图系；为 false 时边建图边写（调试用） |
| `trajectory_log_dir`    | string | "" | 边建图边写时的输出目录。**建图完成后**：trajectory_odom 会同时写入 save_map 的 output_dir 与 trajectory_log_dir（若两者不同），便于在 logs 目录直接找到该 CSV |

在 launch 或 config 中可覆盖，例如：

```yaml
automap_system:
  ros__parameters:
    trajectory_log_enable: true
    trajectory_log_dir: "/path/to/logs"
```

## 4. 绘图脚本

使用仓库自带脚本对比两条轨迹并出图。**推荐在项目内使用 Python 虚拟环境（venv）**，避免与系统 Python 冲突且依赖可复现。

### 4.1 环境准备（推荐：venv 完整流程）

在**仓库根目录**（即包含 `automap_pro` 子目录的那一级，例如 `~/Documents/github/automap_pro`）执行：

**步骤 1：创建虚拟环境**

```bash
cd /path/to/automap_pro   # 替换为你的仓库根目录
python3 -m venv .venv
```

**步骤 2：激活虚拟环境（可选，若用下方“绝对路径”方式可跳过）**

```bash
source .venv/bin/activate   # Linux/macOS
# 激活后提示符前会显示 (.venv)
```

**步骤 3：配置 pip 镜像（可选，国内网络建议配置以加速安装）**

项目内已可为 `.venv` 配置清华源，若需自行写入：

```bash
# 写入 .venv/pip.conf（仅对该 venv 生效）
cat << 'EOF' > .venv/pip.conf
[global]
index-url = https://pypi.tuna.tsinghua.edu.cn/simple
trusted-host = pypi.tuna.tsinghua.edu.cn
EOF
```

**步骤 4：安装依赖**

```bash
# 已激活 venv 时
pip install pandas matplotlib numpy

# 未激活时用绝对路径
.venv/bin/pip install pandas matplotlib numpy
```

**步骤 5：验证**

```bash
.venv/bin/python -c "import pandas, matplotlib, numpy; print('OK')"
```

### 4.2 运行绘图

**方式 A：使用 venv 中的 Python（推荐）**

```bash
# 位置参数：直接传 odom CSV 路径
.venv/bin/python ./automap_pro/scripts/plot_trajectory_compare.py \
  automap_ws/logs/trajectory_odom_20260309_181008.csv \
  --out compare.png

# 或指定日志目录，自动找最新 trajectory_odom_*.csv
.venv/bin/python ./automap_pro/scripts/plot_trajectory_compare.py --dir logs --out compare.png
```

**方式 B：使用 --odom 显式指定文件**

```bash
.venv/bin/python ./automap_pro/scripts/plot_trajectory_compare.py \
  --odom automap_ws/logs/trajectory_odom_20260309_181008.csv \
  --out compare.png
```

**方式 C：未使用 venv 时（需已安装依赖）**

```bash
python3 ./automap_pro/scripts/plot_trajectory_compare.py \
  automap_ws/logs/trajectory_odom_20260309_181008.csv \
  --out compare.png
```

**常用参数**

| 参数 | 说明 |
|------|------|
| `odom_file`（位置参数） | trajectory_odom_*.csv 路径，与 `--odom` 二选一 |
| `--odom` | 同上，显式指定 odom CSV |
| `--dir` | 日志目录，脚本自动查找该目录下最新 `trajectory_odom_*.csv` |
| `--out` | 输出图片路径，默认 `trajectory_compare.png` |
| `--stats` | 打印偏差统计信息 |

**依赖**：`pandas`、`matplotlib`、`numpy`（见 4.1 安装步骤）。

输出图为 2×2：XY 平面轨迹、X-t、Y-t、Z-t。蓝线为建图轨迹，红点为 GPS，可直观对比偏差与漂移。若系统已安装中文字体（如 `fonts-noto-cjk`），图中中文标题可正常显示；否则脚本会抑制缺失字形警告。

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

## 7. 为何 GPS 轨迹与关键帧轨迹不重合（边建图边写的问题）

### 7.1 现象

在 `trajectory_odom_*.csv` 中对比「建图轨迹」(x,y,z) 与「GPS」(gps_x, gps_y, gps_z) 时，两条轨迹不重合：存在整体偏移、尺度差异或随时间拉大的漂移。

### 7.2 根本原因：边建图边写导致坐标系与位姿不一致

| 因素 | 说明 |
|------|------|
| **写入的位姿来源** | 当前实现是在 **每帧 odom 回调**（`onOdometry`）里写 CSV，写入的 (x,y,z) 来自 **前端实时位姿**（如 Livo 的 `/aft_mapped_to_init`），**不是**后端优化后的关键帧位姿。 |
| **建图过程中的优化** | 建图过程中会进行：GPS 对齐（SVD 求 enu→map）、回环、BA 等。这些会**更新**地图系、`enu_to_map` 以及关键帧的 `T_w_b_optimized`。 |
| **实时写入的“快照”** | 每一行写的是「当时那一刻」的前端位姿与当时可用的 GPS 匹配。随着后续 GPS 对齐/优化，地图系和位姿都会变，**已写入的旧行不会回溯更新**。 |
| **GPS 列的含义** | GPS 列在写入时若已对齐则用当时的 `enu_to_map` 转换；对齐发生前后，同一 ENU 点对应的 map 坐标会变，导致同一文件内前后段 GPS 与轨迹不在同一套「最终」地图系下。 |

因此：**边建图边写 = 把“中间状态”的位姿和“可能尚未稳定”的 GPS 对齐混在一起**，轨迹与 GPS 自然无法在最终地图系下重合。

### 7.3 正确做法：建图完成后再写轨迹

- **位姿**：应使用 **全部优化完成后的关键帧位姿** `T_w_b_optimized`（与 `trajectory_tum.txt` / 保存的地图一致）。
- **GPS**：在**同一次保存时**用**最终的** `enu_to_map` 将 GPS 转换到地图系，再按时间戳匹配到关键帧写入。
- **时机**：在 **saveMapToFiles**（或等价「建图完成/保存地图」）时，一次性生成 `trajectory_odom_*.csv`，保证：
  - 轨迹 = 关键帧轨迹（与 trajectory_tum.txt 一致）；
  - GPS = 同一地图系、同一套对齐结果；
  - 绘图对比时两条线才具有可比性。

### 7.4 行为与参数（默认：仅建图完成后写）

- **`trajectory_log_after_mapping_only`**（默认 **true**）  
  - **true**：不在 `onOdometry` 里写 CSV；仅在 **saveMapToFiles** 时写一份 `trajectory_odom_<session_id>.csv` 到**保存目录**（与 trajectory_tum.txt 同目录），内容为关键帧位姿 + 按时间匹配的 GPS（最终地图系）。  
  - **false**：保留旧行为，边建图边写（仅用于调试实时 odom，不保证与 GPS 在最终系下一致）。
- 建图完成后写的文件与现有 `plot_trajectory_compare.py` 使用的 CSV 格式兼容（表头与列一致），可直接用同一脚本绘图。

## 8. 变更文件清单

- `automap_pro/include/automap_pro/frontend/gps_manager.h`：`registerMeasurementLogCallback`
- `automap_pro/src/frontend/gps_manager.cpp`：每条 GPS 测量后调用回调；`queryByTimestampForLog` 边界与时间窗逻辑
- `automap_pro/include/automap_pro/system/automap_system.h`：轨迹文件与写方法声明；`trajectory_log_after_mapping_only_`、`writeTrajectoryOdomAfterMapping`
- `automap_pro/src/system/automap_system.cpp`：`ensureTrajectoryLogDir`、`writeTrajectoryOdom`（GPS 匹配窗 1.0s）、`onGPSMeasurementForLog`；**建图完成后写**：`trajectory_log_after_mapping_only` 参数、`writeTrajectoryOdomAfterMapping`（关键帧 + 最终 GPS），saveMapToFiles 内调用；onOdometry 中仅在 `!trajectory_log_after_mapping_only_` 时写 odom
- `automap_pro/scripts/plot_trajectory_compare.py`：绘图脚本（新增）
