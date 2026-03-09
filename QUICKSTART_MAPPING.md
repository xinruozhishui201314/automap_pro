# AutoMap-Pro 建图快速开始

## 5分钟快速开始

### 步骤1：编译（第一次使用）

```bash
cd /home/wqs/Documents/github/automap_pro

# 设置工作空间
make setup

# 编译
make build-release
```

### 步骤2：启动建图

```bash
# 使用默认数据集启动
./start_mapping.sh

# 或使用 Makefile
make run-offline BAG_FILE=data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 步骤3：查看结果

```bash
# 建图完成后，查看输出目录
ls -lh /data/automap_output/

# 可视化地图
python3 automap_pro/scripts/visualize_results.py --output_dir /data/automap_output
```

---

## 使用自定义数据

```bash
# 方法1：使用启动脚本
./start_mapping.sh -b /path/to/your/data.bag -o /path/to/output

# 方法2：使用 Makefile
make run-offline BAG_FILE=/path/to/your/data.bag

# 方法3：直接使用 ros2 launch
source ~/automap_ws/install/setup.bash
ros2 launch automap_pro automap_offline.launch.py \
    bag_file:=/path/to/your/data.bag
```

---

## 常用命令

```bash
# 保存地图
make save-map

# 触发优化
make trigger-opt

# 查看状态
make status

# 评估轨迹
make eval-traj

# 评估地图
make eval-map

# 可视化结果
make visualize
```

---

## 数据集说明

### nya_02 数据集

- **位置**: `data/automap_input/nya_02_slam_imu_to_lidar/`
- **大小**: 9.4GB
- **传感器**: Livox Avia LiDAR + IMU + GPS
- **配置文件**:
  - `imu_v100.yaml` - IMU 参数
  - `lidar_horz.yaml` - 水平 LiDAR 参数
  - `lidar_vert.yaml` - 垂直 LiDAR 参数

### 数据格式要求

| 传感器 | 话题 | 格式 |
|--------|------|------|
| LiDAR | `/livox/lidar` | sensor_msgs/PointCloud2 |
| IMU | `/livox/imu` | sensor_msgs/Imu |
| GPS | `/gps/fix` | sensor_msgs/NavSatFix |

---

## 故障排查

### 问题1：找不到 bag 文件

```bash
# 检查文件是否存在
ls -lh data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 使用绝对路径
./start_mapping.sh -b /home/wqs/Documents/github/automap_pro/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 问题2：话题名称不匹配

```bash
# 检查 bag 文件中的话题
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 更新配置文件
vim automap_pro/config/system_config.yaml
```

### 问题3：GPU 内存不足

```bash
# 禁用GPU
vim automap_pro/config/system_config.yaml
# 修改: use_gpu: false
```

---

## 更多信息

- **详细文档**: `docs/MAPPING_WORKFLOW.md`
- **系统配置**: `automap_pro/config/system_config.yaml`
- **项目README**: `automap_pro/README.md`
- **启动脚本**: `./start_mapping.sh --help`

---

**维护者**: Automap Pro Team
**最后更新**: 2026-03-01
