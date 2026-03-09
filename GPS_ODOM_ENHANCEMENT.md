# 里程计日志增强GPS信息

## 0. Executive Summary

| 项目 | 内容 |
|------|------|
| **目标** | 在里程计日志文件中添加对应GPS信息，便于对比分析建图精度 |
| **修改文件** | `automap_pro/src/system/automap_system.cpp`, `automap_pro/docs/TRAJECTORY_LOG_AND_PLOT.md` |
| **新字段** | gps_x, gps_y, gps_z, gps_frame, gps_valid |
| **优势** | 避免跨文件匹配时间戳，直接在单文件中对比 |

---

## 1. 背景 & 目标

### 1.1 原有架构
- `trajectory_odom_<session_id>.csv`：里程计轨迹（每帧）
  - 字段：timestamp, x, y, z, qx, qy, qz, qw, pos_std_x, pos_std_y, pos_std_z
- `trajectory_gps_<session_id>.csv`：GPS轨迹
  - 字段：timestamp, x, y, z, frame

### 1.2 问题
- GPS与里程计是**独立文件**，需要通过时间戳匹配对比
- GPS采样频率低（约1Hz），里程计频率高（约10Hz），时间戳匹配复杂
- 无法直接在单文件中快速可视化对比

### 1.3 解决方案
在 `trajectory_odom_<session_id>.csv` 中添加对应GPS列，便于：
1. 直接在同一行对比里程计与GPS位置
2. 无需跨文件匹配时间戳
3. 更快地进行精度分析

---

## 2. 变更清单

| 文件路径 | 修改类型 | 说明 |
|----------|----------|------|
| `automap_pro/src/system/automap_system.cpp` | 修改 | `writeTrajectoryOdom()` 添加GPS查询和写入 |
| `automap_pro/docs/TRAJECTORY_LOG_AND_PLOT.md` | 更新 | 更新字段说明和使用方法 |

---

## 3. 关键代码变更

### 3.1 automap_system.cpp - writeTrajectoryOdom()

```cpp
// 查询当前时间戳对应的GPS数据（用于对比分析）
double gps_x = 0.0, gps_y = 0.0, gps_z = 0.0;
bool gps_valid = false;
const char* gps_frame = "none";

auto gps_opt = gps_manager_.queryByTimestamp(ts);
if (gps_opt && gps_opt->is_valid) {
    gps_valid = true;
    // 将GPS坐标转换到map frame（如果已对齐）或保持enu
    Eigen::Vector3d pos = gps_manager_.isAligned() ? 
        gps_manager_.enu_to_map(gps_opt->position_enu) : gps_opt->position_enu;
    gps_x = pos.x();
    gps_y = pos.y();
    gps_z = pos.z();
    gps_frame = gps_manager_.isAligned() ? "map" : "enu";
}

// 写入时包含GPS信息
trajectory_odom_file_ << std::fixed << std::setprecision(6)
    << ts << ","
    << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
    << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
    << px << "," << py << "," << pz << ","
    << gps_x << "," << gps_y << "," << gps_z << ","      // 新增
    << gps_frame << "," << (gps_valid ? "1" : "0") << "\n";  // 新增
```

### 3.2 日志表头更新

```csv
timestamp,x,y,z,qx,qy,qz,qw,pos_std_x,pos_std_y,pos_std_z,gps_x,gps_y,gps_z,gps_frame,gps_valid
```

---

## 4. 新字段说明

| 字段 | 类型 | 含义 | 单位/取值 |
|------|------|------|-----------|
| timestamp | double | 时间戳（秒） | - |
| x, y, z | double | 里程计位置（map frame） | 米 |
| qx, qy, qz, qw | double | 里程计姿态四元数 | - |
| pos_std_x, pos_std_y, pos_std_z | double | 位置标准差（从协方差计算） | 米 |
| **gps_x, gps_y, gps_z** | double | **对应GPS位置（map/enu frame）** | **米（若无GPS则为0）** |
| **gps_frame** | string | **GPS坐标系** | **"map"（已对齐）或 "enu"（未对齐）或 "none"** |
| **gps_valid** | int | **GPS是否有效** | **1（有效）或 0（无效/无数据）** |

---

## 5. 编译/部署/运行说明

### 5.1 编译

```bash
cd /home/wqs/Documents/github/automap_pro
bash run_automap.sh --build-only --clean
```

### 5.2 运行

```bash
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml
```

### 5.3 验证

运行后检查 `logs/trajectory_odom_*.csv`：
1. 表头应包含 `gps_x,gps_y,gps_z,gps_frame,gps_valid`
2. `gps_valid=1` 的行应填充GPS位置
3. `gps_valid=0` 的行GPS位置应为0

---

## 6. 使用示例

### 6.1 直接对比里程计与GPS

```python
import pandas as pd
import matplotlib.pyplot as plt

# 读取里程计日志（包含GPS）
df = pd.read_csv('logs/trajectory_odom_20260308_172644.csv')

# 提取有效GPS数据
gps_valid = df[df['gps_valid'] == 1]

# 绘制对比图
plt.figure(figsize=(12, 8))
plt.plot(df['x'], df['y'], 'b-', label='Odom trajectory', alpha=0.7)
plt.scatter(gps_valid['gps_x'], gps_valid['gps_y'], c='red', s=5, 
           label='GPS points', alpha=0.5)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Odom vs GPS Trajectory Comparison')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('odom_vs_gps.png', dpi=150)
```

### 6.2 计算偏差统计

```python
import pandas as pd
import numpy as np

df = pd.read_csv('logs/trajectory_odom_20260308_172644.csv')

# 计算有效点的距离偏差
gps_valid = df[df['gps_valid'] == 1].copy()
gps_valid['dx'] = gps_valid['x'] - gps_valid['gps_x']
gps_valid['dy'] = gps_valid['y'] - gps_valid['gps_y']
gps_valid['dz'] = gps_valid['z'] - gps_valid['gps_z']
gps_valid['dist'] = np.sqrt(gps_valid['dx']**2 + gps_valid['dy']**2 + gps_valid['dz']**2)

# 统计
print(f"平均偏差: {gps_valid['dist'].mean():.3f} m")
print(f"最大偏差: {gps_valid['dist'].max():.3f} m")
print(f"中位数偏差: {gps_valid['dist'].median():.3f} m")
```

---

## 7. 向后兼容性

| 兼容性 | 说明 |
|----------|------|
| **绘图脚本** | 旧脚本 `plot_trajectory_compare.py` 需要修改以使用新字段 |
| **现有分析工具** | 需要更新以处理新的GPS列 |
| **日志格式** | 新增字段在末尾，不影响现有字段解析 |

---

## 8. 风险与缓解

| 风险 | 级别 | 缓解措施 |
|------|------|----------|
| GPS查询增加开销 | 低 | GPS查询基于时间戳索引，O(log n)复杂度 |
| 文件大小增加 | 低 | 每行增加约30字节（5列×6字节） |
| 无GPS时数据冗余 | 无 | `gps_valid=0` 标记无效数据，不影响分析 |

---

## 9. 后续演进建议

| 阶段 | 措施 |
|------|------|
| **短期** | 1. 测试验证新日志格式<br>2. 更新 `plot_trajectory_compare.py` 脚本支持新字段 |
| **中期** | 1. 添加GPS质量指标（如HDOP）到日志<br>2. 添加自动化偏差分析脚本 |
| **长期** | 1. 考虑二进制日志格式（减少文件大小）<br>2. 添加实时偏差监控 |
