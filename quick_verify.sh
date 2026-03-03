#!/bin/bash
# 快速验证脚本
# 检查关键项

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
SYMLINK="$PROJECT_ROOT/automap_ws/src/automap_pro"

echo "========================================"
echo "快速验证"
echo "========================================"
echo ""

# 1. 符号链接
echo -n "1. 符号链接: "
TARGET=$(readlink -f "$SYMLINK")
if [ "$TARGET" == "$PROJECT_ROOT/automap_pro" ]; then
    echo -e "${GREEN}✓ 正确${NC}"
else
    echo -e "${RED}✗ 错误: $TARGET${NC}"
fi

# 2. 配置文件话题
echo -n "2. 配置文件话题: "
CONFIG="$SYMLINK/config/system_config.yaml"
LIDAR=$(grep -A 2 "lidar:" "$CONFIG" | grep "topic:" | awk '{print $2}' | tr -d '"')
IMU=$(grep -A 2 "imu:" "$CONFIG" | grep "topic:" | awk '{print $2}' | tr -d '"')

if [ "$LIDAR" == "/os1_cloud_node1/points" ] && [ "$IMU" == "/imu/imu" ]; then
    echo -e "${GREEN}✓ 正确 (nya_02)${NC}"
else
    echo -e "${RED}✗ 错误: $LIDAR, $IMU${NC}"
fi

# 3. launch 文件
echo -n "3. launch 文件修改: "
if grep -q "移除固定 remappings" "$SYMLINK/launch/automap_offline.launch.py"; then
    echo -e "${GREEN}✓ 已生效${NC}"
else
    echo -e "${RED}✗ 未生效${NC}"
fi

# 4. bag 文件
echo -n "4. bag 文件: "
BAG="$PROJECT_ROOT/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2"
if [ -d "$BAG" ]; then
    echo -e "${GREEN}✓ 存在${NC}"
else
    echo -e "${RED}✗ 不存在${NC}"
fi

echo ""
echo "========================================"
echo "验证完成"
echo "========================================"
