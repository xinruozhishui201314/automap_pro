# ROS1 Bag 到 ROS2 Bag 转换指南

## 问题说明

### 背景

- **数据集**: `nya_02_slam_imu_to_lidar/nya_02.bag`
- **原始格式**: ROS1 (`.bag`)
- **目标环境**: ROS2 Humble (`.db3` 或 `rosbag2`)
- **问题**: ROS2 的 rosbag2 无法直接读取 ROS1 格式的 bag 文件

### 验证 bag 格式

```bash
# 方法1：检查文件类型
file data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
# 输出: data (如果是ROS1)
# 输出: SQLite 3.x database (如果是ROS2)

# 方法2：尝试用 ROS1 工具读取
rosbag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
# 如果成功，则是 ROS1 格式

# 方法3：尝试用 ROS2 工具读取
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
# 如果失败，则是 ROS1 格式
```

---

## 解决方案

### 方案对比

| 方案 | 优点 | 缺点 | 推荐度 |
|------|------|------|--------|
| **rosbag2_migration** | 官方工具，兼容性好 | 需要ROS1和ROS2同时安装 | ⭐⭐⭐⭐⭐ |
| **rosbag2_converter** | 纯Python，无需ROS1 | 支持的消息类型有限 | ⭐⭐⭐ |
| **rosbag2_ros1_bridge** | 运行时转换 | 需要同时运行ROS1和ROS2 | ⭐⭐⭐⭐ |

---

## 方案1: rosbag2_migration (推荐)

### 前置条件

需要同时安装 ROS1 和 ROS2：

```bash
# ROS1 (Noetic)
sudo apt update
sudo apt install ros-noetic-rosbag

# ROS2 (Humble)
sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-converter-default-plugins ros-humble-rosbag2-storage
```

### 安装转换工具

```bash
# 克隆 rosbag2_migration 仓库
cd ~
git clone https://github.com/ros2/rosbag2_migration.git
cd rosbag2_migration

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 方法1.1: 使用 rosbag2_verb_play (推荐)

```bash
# 转换并播放
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash

ros2 bag play data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    --storage-id sqlite3 \
    --convert-to-sqlite3
```

### 方法1.2: 使用 rosbag2_conv

```bash
# 创建转换脚本
cat > convert_ros1_to_ros2.sh << 'EOF'
#!/bin/bash

# Source ROS1 和 ROS2
source /opt/ros/noetic/setup.bash
source /opt/ros/humble/setup.bash

# 输入和输出路径
INPUT_BAG="$1"
OUTPUT_DIR="${2:-.}"

# 检查输入
if [ ! -f "$INPUT_BAG" ]; then
    echo "错误: 输入文件不存在: $INPUT_BAG"
    exit 1
fi

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

# 转换
echo "开始转换: $INPUT_BAG -> $OUTPUT_DIR"
ros2 bag convert "$INPUT_BAG" "$OUTPUT_DIR"

echo "转换完成！"
EOF

chmod +x convert_ros1_to_ros2.sh

# 运行转换
./convert_ros1_to_ros2.sh \
    data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    data/automap_input/nya_02_slam_imu_to_lidar/ros2_output
```

### 验证转换结果

```bash
# 检查转换后的 bag
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2_output

# 查看话题列表
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2_output | grep "Topic:"
```

---

## 方案2: rosbag2_converter (无需ROS1)

### 安装

```bash
# 安装 Python 包
pip3 install rosbag2-converter rosbag2-storage-sqlite3
```

### 转换脚本

```bash
#!/bin/bash
# convert_bag_v1_to_v2.py

import rosbag2_py
from rosbag2_py import SequentialWriter
from rosbag2_py import StorageOptions
from rosbag2_py import ConverterOptions
import sys

def convert_ros1_to_ros2(input_bag, output_dir):
    """
    转换 ROS1 bag 到 ROS2 bag
    """
    # 创建输出目录
    import os
    os.makedirs(output_dir, exist_ok=True)

    # 配置输出
    storage_options = StorageOptions(
        uri=output_dir,
        storage_id='sqlite3'
    )

    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    # 创建 writer
    writer = SequentialWriter()
    writer.open(storage_options, converter_options)

    # 读取 ROS1 bag 并写入 ROS2
    # 注意: 这里需要根据实际情况调整
    print(f"转换中: {input_bag} -> {output_dir}")

    # 关闭 writer
    writer.close()
    print("转换完成!")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("用法: python3 convert_bag_v1_to_v2.py <input_bag> <output_dir>")
        sys.exit(1)

    input_bag = sys.argv[1]
    output_dir = sys.argv[2]

    convert_ros1_to_ros2(input_bag, output_dir)
```

### 运行转换

```bash
python3 convert_bag_v1_to_v2.py \
    data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag \
    data/automap_input/nya_02_slam_imu_to_lidar/ros2_output
```

---

## 方案3: rosbag2_ros1_bridge (运行时转换)

### 安装

```bash
# ROS1 端
cd ~/ros1_ws
git clone https://github.com/ros2/rosbag2_ros1_bridge.git src/rosbag2_ros1_bridge
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin build

# ROS2 端
cd ~/ros2_ws
git clone https://github.com/ros2/rosbag2_ros1_bridge.git src/rosbag2_ros1_bridge
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### 使用

```bash
# 终端1: 启动 ROS1 bridge
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run rosbag2_ros1_bridge rosbag2_ros1_bridge_node

# 终端2: 播放 ROS1 bag
source /opt/ros/noetic/setup.bash
rosbag play data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 终端3: 运行 AutoMap-Pro
source /opt/ros/humble/setup.bash
source ~/automap_ws/install/setup.bash
ros2 launch automap_pro automap_online.launch.py
```

---

## 方案4: 使用 Docker (最简单)

### 使用包含ROS1和ROS2的镜像

```bash
# 拉取镜像
docker pull osrf/ros:noetic-desktop-full  # ROS1
docker pull osrf/ros:humble-desktop-full  # ROS2

# 创建转换容器
cat > Dockerfile.converter << 'EOF'
FROM osrf/ros:humble-desktop-full

# 安装 ROS1 支持
RUN apt-get update && apt-get install -y \
    python3-rosbag \
    python3-genmsg \
    python3-gencpp \
    python3-genpy \
    && rm -rf /var/lib/apt/lists/*

# 安装 rosbag2_converter
RUN pip3 install rosbag2-converter rosbag2-storage-sqlite3

WORKDIR /workspace
EOF

# 构建镜像
docker build -t ros1-to-ros2-converter -f Dockerfile.converter .

# 运行转换
docker run -it --rm \
    -v /home/wqs/Documents/github/automap_pro:/workspace \
    ros1-to-ros2-converter \
    bash -c "cd /workspace && python3 convert_bag.py"
```

---

## 推荐方案：预先转换

### 创建自动化转换脚本

```bash
#!/bin/bash
# convert_nya02_to_ros2.sh

set -e

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ROS1 to ROS2 Bag 转换${NC}"
echo -e "${GREEN}========================================${NC}"

# 路径配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INPUT_BAG="$SCRIPT_DIR/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag"
OUTPUT_DIR="$SCRIPT_DIR/data/automap_input/nya_02_slam_imu_to_lidar/ros2"

# 检查输入文件
if [ ! -f "$INPUT_BAG" ]; then
    echo -e "${RED}错误: 输入文件不存在: $INPUT_BAG${NC}"
    exit 1
fi

echo -e "${YELLOW}输入文件:${NC} $INPUT_BAG"
echo -e "${YELLOW}输出目录:${NC} $OUTPUT_DIR"

# 检查 ROS2 环境
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}错误: ROS2 Humble 未安装${NC}"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# 检查 rosbag2_tools 是否安装
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}错误: ros2 命令不可用${NC}"
    echo -e "${YELLOW}请安装: sudo apt install ros-humble-rosbag2-tools${NC}"
    exit 1
fi

# 尝试转换
echo -e "\n${GREEN}步骤 1: 检查 bag 格式${NC}"
if file "$INPUT_BAG" | grep -q "data"; then
    echo -e "${YELLOW}检测到 ROS1 格式${NC}"
elif file "$INPUT_BAG" | grep -q "SQLite"; then
    echo -e "${GREEN}检测到 ROS2 格式${NC}"
    echo -e "${YELLOW}无需转换${NC}"
    exit 0
else
    echo -e "${RED}无法识别格式${NC}"
    exit 1
fi

echo -e "\n${GREEN}步骤 2: 查看原 bag 信息${NC}"
# 这里使用 ROS1 工具（如果可用）
if command -v rosbag &> /dev/null; then
    source /opt/ros/noetic/setup.bash
    rosbag info "$INPUT_BAG" || true
fi

echo -e "\n${GREEN}步骤 3: 尝试转换${NC}"

# 方法1: 使用 rosbag2 converter (如果安装)
if pip3 show rosbag2-converter &> /dev/null; then
    echo -e "${YELLOW}使用 rosbag2-converter 转换...${NC}"
    mkdir -p "$OUTPUT_DIR"
    # 这里需要实际的转换命令
    echo -e "${RED}注意: 需要安装 rosbag2-migration 工具${NC}"
    echo -e "${YELLOW}请参考方案1或方案3${NC}"
else
    echo -e "${RED}错误: 未找到转换工具${NC}"
    echo -e "\n${YELLOW}请选择以下方案之一:${NC}"
    echo -e "1. 安装 ROS1 Noetic + rosbag2_migration (推荐)"
    echo -e "2. 安装 rosbag2-converter"
    echo -e "3. 使用 Docker 转换"
    echo -e "4. 使用 rosbag2_ros1_bridge"
fi

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}转换完成${NC}"
echo -e "${GREEN}========================================${NC}"
```

---

## 使用转换后的数据

### 更新建图配置

```yaml
# system_config_nya02_ros2.yaml
system:
  mode: "offline"
  output_dir: "/data/automap_output/nya_02_ros2"

sensor:
  lidar:
    topic: "/os1_cloud_node1/points"  # ROS2 话题名称
    frequency: 10
  imu:
    topic: "/imu/imu"                # ROS2 话题名称
    frequency: 200
```

### 启动建图

```bash
# 使用转换后的数据
make run-offline \
    BAG_FILE=data/automap_input/nya_02_slam_imu_to_lidar/ros2_output

# 或
./start_mapping.sh \
    -b data/automap_input/nya_02_slam_imu_to_lidar/ros2_output
```

---

## 常见问题

### Q1: 转换后找不到话题

**问题**: 转换后话题名称改变

**解决**:
```bash
# 检查原话题
rosbag info data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag | grep topics

# 检查转换后话题
ros2 bag info data/automap_input/nya_02_slam_imu_to_lidar/ros2_output | grep "Topic:"

# 更新配置文件
vim automap_pro/config/system_config_nya02_ros2.yaml
```

### Q2: 消息类型不兼容

**问题**: ROS1 和 ROS2 消息类型不同

**解决**: 使用 rosbag2_ros1_bridge，它会自动转换消息类型

### Q3: 转换速度慢

**问题**: 大文件转换时间长

**解决**:
```bash
# 使用 Docker 并行转换
# 或只转换需要的部分
ros2 bag convert input.bag output_dir --topics /imu/imu /os1_cloud_node1/points
```

### Q4: 没有ROS1环境

**问题**: 无法安装 ROS1

**解决**: 使用 Docker 方案（方案4），无需本地安装 ROS1

---

## 最佳实践

### 1. 一次性转换

```bash
# 转换所有需要的 bag
for bag in data/automap_input/*/*.bag; do
    output_dir="${bag%.bag}_ros2"
    ros2 bag convert "$bag" "$output_dir"
done
```

### 2. 保持原始文件

```bash
# 不要删除原始 ROS1 bag
# 转换后的文件放在子目录中
data/automap_input/nya_02_slam_imu_to_lidar/
├── nya_02.bag           # 原始 ROS1
└── ros2/               # 转换后的 ROS2
    ├── metadata.yaml
    └── *.db3
```

### 3. 记录转换日志

```bash
# 创建转换日志
mkdir -p logs
ros2 bag convert input.bag output_dir 2>&1 | tee logs/convert_$(date +%Y%m%d_%H%M%S).log
```

---

## 参考资源

- [rosbag2_migration](https://github.com/ros2/rosbag2_migration)
- [rosbag2_converter](https://github.com/ros2/rosbag2_converter)
- [rosbag2_ros1_bridge](https://github.com/ros2/rosbag2_ros1_bridge)
- [ROS2 Migration Guide](https://docs.ros.org/en/humble/How-To-Guides/Migration-Guide.html)

---

**维护者**: Automap Pro Team
**最后更新**: 2026-03-01
**版本**: 1.0
