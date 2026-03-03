#!/bin/bash
# 完整的验证和运行脚本
# 1. 验证符号链接
# 2. 验证配置文件
# 3. 验证 launch 文件修改
# 4. 运行建图

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
WORKSPACE_SRC="$PROJECT_ROOT/automap_ws/src"
SYMLINKS_DIR="$WORKSPACE_SRC"

log_info() {
    echo -e "${GREEN}[INFO]${NC} $@"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $@"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $@"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $@"
}

check_passed=0
check_failed=0

check_result() {
    if [ $? -eq 0 ]; then
        echo -e "  ${GREEN}✓ 通过${NC}"
        ((check_passed++))
    else
        echo -e "  ${RED}✗ 失败${NC}"
        ((check_failed++))
    fi
}

echo "========================================"
echo "AutoMap-Pro 环境验证"
echo "========================================"
echo ""

# ========================================
# 1. 验证符号链接
# ========================================
log_step "1. 验证符号链接"

# automap_pro 符号链接
echo -n "  automap_pro 符号链接: "
if [ -L "$SYMLINKS_DIR/automap_pro" ]; then
    TARGET=$(readlink -f "$SYMLINKS_DIR/automap_pro")
    if [ "$TARGET" == "$PROJECT_ROOT/automap_pro" ]; then
        check_result
    else
        echo -e "  ${RED}✗ 指向错误: $TARGET${NC}"
        ((check_failed++))
    fi
else
    echo -e "  ${RED}✗ 不存在${NC}"
    ((check_failed++))
fi

# fast_livo 符号链接（可选，因为容器内会通过挂载映射）
echo -n "  fast_livo 符号链接: "
if [ -L "$SYMLINKS_DIR/fast_livo" ]; then
    TARGET=$(readlink -f "$SYMLINKS_DIR/fast_livo")
    if [ "$TARGET" == "$PROJECT_ROOT/fast-livo2-humble" ]; then
        check_result
    else
        echo -e "  ${YELLOW}⚠ 指向其他路径: $TARGET (容器内会修正)${NC}"
        ((check_passed++))
    fi
else
    echo -e "  ${YELLOW}⚠ 不存在 (容器内会通过挂载映射)${NC}"
    ((check_passed++))
fi

echo ""

# ========================================
# 2. 验证配置文件
# ========================================
log_step "2. 验证配置文件"

CONFIG_FILE="$SYMLINKS_DIR/automap_pro/config/system_config.yaml"

echo -n "  配置文件存在: "
if [ -f "$CONFIG_FILE" ]; then
    check_result
else
    echo -e "  ${RED}✗ 不存在${NC}"
    ((check_failed++))
    exit 1
fi

echo -n "  配置文件行数 (>300): "
LINES=$(wc -l < "$CONFIG_FILE")
if [ "$LINES" -gt 300 ]; then
    echo -e "  ${GREEN}✓ $LINES 行${NC}"
    ((check_passed++))
else
    echo -e "  ${RED}✗ $LINES 行 (可能是旧版本)${NC}"
    ((check_failed++))
fi

# 话题配置
LIDAR_TOPIC=$(grep -A 2 "lidar:" "$CONFIG_FILE" | grep "topic:" | awk '{print $2}' | tr -d '"')
IMU_TOPIC=$(grep -A 2 "imu:" "$CONFIG_FILE" | grep "topic:" | awk '{print $2}' | tr -d '"')

echo -n "  LiDAR 话题 (预期: /os1_cloud_node1/points): "
if [ "$LIDAR_TOPIC" == "/os1_cloud_node1/points" ]; then
    check_result
elif [ "$LIDAR_TOPIC" == "/livox/lidar" ]; then
    echo -e "  ${YELLOW}⚠ Livox Avia 默认值${NC}"
    ((check_passed++))
else
    echo -e "  ${RED}✗ $LIDAR_TOPIC${NC}"
    ((check_failed++))
fi

echo -n "  IMU 话题 (预期: /imu/imu): "
if [ "$IMU_TOPIC" == "/imu/imu" ]; then
    check_result
elif [ "$IMU_TOPIC" == "/livox/imu" ]; then
    echo -e "  ${YELLOW}⚠ Livox Avia 默认值${NC}"
    ((check_passed++))
else
    echo -e "  ${RED}✗ $IMU_TOPIC${NC}"
    ((check_failed++))
fi

echo ""

# ========================================
# 3. 验证 launch 文件修改
# ========================================
log_step "3. 验证 launch 文件修改"

LAUNCH_FILE="$SYMLINKS_DIR/automap_pro/launch/automap_offline.launch.py"

echo -n "  launch 文件存在: "
if [ -f "$LAUNCH_FILE" ]; then
    check_result
else
    echo -e "  ${RED}✗ 不存在${NC}"
    ((check_failed++))
    exit 1
fi

echo -n "  已移除 camera_pinhole.yaml: "
if ! grep -q "camera_pinhole.yaml" "$LAUNCH_FILE"; then
    check_result
else
    echo -e "  ${RED}✗ 仍在使用 camera_pinhole.yaml${NC}"
    ((check_failed++))
fi

echo -n "  已移除固定 remappings: "
if ! grep -q '("/livox/lidar", "/livox/lidar")' "$LAUNCH_FILE"; then
    check_result
else
    echo -e "  ${RED}✗ 仍在使用固定 remappings${NC}"
    ((check_failed++))
fi

echo ""

# ========================================
# 4. 验证 Bag 文件
# ========================================
log_step "4. 验证 Bag 文件"

BAG_FILE="$PROJECT_ROOT/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2"

echo -n "  Bag 文件存在: "
if [ -d "$BAG_FILE" ]; then
    check_result
else
    echo -e "  ${RED}✗ 不存在${NC}"
    ((check_failed++))
fi

echo -n "  metadata.yaml 存在: "
if [ -f "$BAG_FILE/metadata.yaml" ]; then
    check_result
else
    echo -e "  ${YELLOW}⚠ 不存在 (但可能不影响运行)${NC}"
    ((check_passed++))
fi

# 检查 bag 中的话题
if command -v ros2 &> /dev/null; then
    echo -n "  Bag 话题检查 (需要 ROS2 环境): "
    echo -e "  ${YELLOW}⚠ 跳过 (容器内会检查)${NC}"
    ((check_passed++))
else
    echo -n "  Bag 话题检查: "
    echo -e "  ${YELLOW}⚠ 跳过 (ROS2 未安装)${NC}"
    ((check_passed++))
fi

echo ""

# ========================================
# 5. 总结
# ========================================
echo "========================================"
echo "验证总结"
echo "========================================"
echo -e "  ${GREEN}通过: $check_passed${NC}"
echo -e "  ${RED}失败: $check_failed${NC}"
echo ""

if [ $check_failed -eq 0 ]; then
    echo -e "${GREEN}✓✓✓ 所有检查通过！${NC}"
    echo ""
    
    # 询问是否运行建图
    read -p "是否立即运行建图? (y/n): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        log_step "运行 Docker 建图"
        cd "$PROJECT_ROOT"
        ./run_full_mapping_docker.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2
    fi
else
    echo -e "${RED}✗ 有 $check_failed 个检查失败，请先修复问题！${NC}"
    echo ""
    log_warn "可能的解决方案："
    echo "  1. 运行 ./fix_symlink.sh 修复符号链接"
    echo "  2. 检查配置文件是否正确"
    echo "  3. 确认 launch 文件修改已生效"
    exit 1
fi
