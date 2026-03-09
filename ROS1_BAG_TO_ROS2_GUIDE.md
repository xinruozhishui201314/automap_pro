# ROS1 Bag 转 ROS2 快速指南

## 问题描述

**数据集**: `nya_02_slam_imu_to_lidar/nya_02.bag` (9.4GB)
**格式**: ROS1 (`.bag`)
**目标环境**: ROS2 Humble
**问题**: ROS2 的 rosbag2 无法直接读取 ROS1 格式的 bag 文件

---

## 快速解决方案（推荐）

### 方案A: 使用 Docker 转换（最简单）⭐⭐⭐⭐⭐

```bash
cd /home/wqs/Documents/github/automap_pro

# 1. 构建转换镜像
docker-compose -f docker/docker-compose.converter.yml build bag-converter

# 2. 运行转换（一次命令）
docker-compose -f docker/docker-compose.converter.yml up bag-converter-run

# 3. 交互式转换（需要手动控制）
docker-compose -f docker/docker-compose.converter.yml run --rm bag-converter
```

在容器中：
```bash
# 转换nya_02数据集
source /opt/ros/humble/setup.bash

python3 /workspace/scripts/convert_ros1_to_ros2.py \
    /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    /workspace/data/automap_input/nya_02_slam_imu_to_lidar/ros2 \
    --verbose
```

### 方案B: 使用 rosbag2_converter (无需ROS1) ⭐⭐⭐⭐

```bash
# 1. 安装转换工具
pip3 install rosbag2-converter rosbag2-storage-sqlite3

# 2. 转换
cd /home/wqs/Documents/github/automap_pro
source /opt/ros/humble/setup.bash

ros2 bag convert \
    data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    data/automap_input/nya_02_slam_imu_to_lidar/ros2
```

### 方案C: 安装 ROS1 Noetic + rosbag2_migration (完整方案) ⭐⭐⭐

```bash
# 1. 安装 ROS1 Noetic
sudo apt update
sudo apt install ros-noetic-desktop-full

# 2. 安装 rosbag2_migration
cd ~
git clone https://github.com/ros2/rosbag2_migration.git
cd rosbag2_migration
rosdep install --from-paths src --ignore-src -r -y
colcon build

# 3. 转换
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash

ros2 bag play \
    data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    --storage-id sqlite3 \
    --convert-to-sqlite3
```

---

## 详细步骤

### 步骤1: 检查bag格式

```bash
# 方法1: 使用file命令
file data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# ROS1 输出: data
# ROS2 输出: SQLite 3.x database

# 方法2: 尝试读取
rosbag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag  # ROS1
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag  # ROS2
```

### 步骤2: 转换bag文件

#### 使用 Docker（推荐）

```bash
cd /home/wqs/Documents/github/automap_pro

# 构建镜像
docker build -t ros1-to-ros2-converter -f docker/converter.Dockerfile .

# 运行转换
docker run --rm -it \
    -v $(pwd)/data:/workspace/data \
    ros1-to-ros2-converter \
    bash -c "
        source /opt/ros/humble/setup.bash &&
        python3 /workspace/scripts/convert_ros1_to_ros2.py \
            /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
            /workspace/data/automap_input/nya_02_slam_imu_to_lidar/ros2 \
            --verbose
    "
```

#### 使用 rosbag2_converter

```bash
# 安装
pip3 install rosbag2-converter rosbag2-storage-sqlite3

# 转换
cd /home/wqs/Documents/github/automap_pro
source /opt/ros/humble/setup.bash

ros2 bag convert \
    data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    data/automap_input/nya_02_slam_imu_to_lidar/ros2
```

### 步骤3: 验证转换结果

```bash
# 查看转换后的目录
ls -lh data/automap_input/nya_02_slam_imu_to_lidar/ros2

# 查看转换后的bag信息
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2

# 查看话题列表
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2 | grep "Topic:"
```

### 步骤4: 使用转换后的数据建图

```bash
# 使用转换后的数据
source ~/automap_ws/install/setup.bash

./start_mapping.sh \
    -b data/automap_input/nya_02_slam_imu_to_lidar/ros2 \
    -c automap_pro/config/system_config_nya02.yaml

# 或使用 Makefile
make run-offline \
    BAG_FILE=data/automap_input/nya_02_slam_imu_to_lidar/ros2
```

---

## 更新配置文件

由于话题名称可能不同，需要验证并更新配置：

```bash
# 1. 检查转换后的话题
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2 | grep "Topic:"

# 2. 更新配置文件
vim automap_pro/config/system_config_nya02_ros2.yaml

# 确认话题名称：
# sensor.lidar.topic
# sensor.imu.topic
```

---

## 输出目录结构

转换后的目录结构：

```
data/automap_input/nya_02_slam_imu_to_lidar/
├── nya_02.bag                    # 原始 ROS1 bag (9.4GB)
├── imu_v100.yaml                 # IMU 配置
├── lidar_horz.yaml              # LiDAR 配置
├── ros2/                        # 转换后的 ROS2 bag
│   ├── metadata.yaml
│   ├── metadata.yaml.db3
│   └── *.db3                   # ROS2 bag 文件
└── ...
```

---

## 常见问题

### Q1: 转换后找不到话题

**问题**: 转换后的话题名称改变

**解决**:
```bash
# 查看原bag话题
rosbag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag | grep topics

# 查看转换后话题
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2 | grep "Topic:"

# 更新配置文件
vim automap_pro/config/system_config_nya02_ros2.yaml
```

### Q2: 转换速度慢

**问题**: 大文件转换时间长

**解决**:
```bash
# 方法1: 只转换需要的部分
ros2 bag convert \
    input.bag output_dir \
    --topics /imu/imu /os1_cloud_node1/points

# 方法2: 使用并行处理
# (需要自定义脚本）
```

### Q3: 消息类型不兼容

**问题**: ROS1和ROS2消息类型不同

**解决**: 使用 rosbag2_ros1_bridge 运行时转换

### Q4: Docker 容器中找不到数据

**问题**: volume挂载路径错误

**解决**:
```bash
# 使用绝对路径
docker run --rm -it \
    -v /home/wqs/Documents/github/automap_pro/data:/workspace/data \
    ros1-to-ros2-converter \
    bash

# 在容器中验证
ls -lh /workspace/data/
```

---

## 性能优化

### 提高转换速度

```bash
# 方法1: 增加缓存大小
export ROS_BAG2_CACHE_SIZE=1048576  # 1GB

# 方法2: 使用SSD存储
# 将临时目录放在SSD上
export TMPDIR=/tmp/rosbag_cache

# 方法3: 并行转换（需要自定义脚本）
```

### 减少输出大小

```bash
# 只转换需要的部分
ros2 bag convert \
    input.bag output_dir \
    --topics /imu/imu /os1_cloud_node1/points \
    --exclude-topics /camera/*  # 排除相机话题
```

---

## 参考资源

- [ROS1 to ROS2 Migration Guide](docs/ROS1_TO_ROS2_MIGRATION.md)
- [rosbag2_migration](https://github.com/ros2/rosbag2_migration)
- [rosbag2_converter](https://github.com/ros2/rosbag2_converter)
- [Docker Hub - ROS Images](https://hub.docker.com/r/osrf/ros)

---

## 下一步

转换完成后：

1. 验证转换结果
2. 更新建图配置（如果话题名称变化）
3. 启动建图流程（见 START_MAPPING_GUIDE.md）
4. 查看建图结果

**维护者**: Automap Pro Team
**最后更新**: 2026-03-01
**版本**: 1.0
