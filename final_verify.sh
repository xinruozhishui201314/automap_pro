#!/bin/bash
# 最终验证脚本
# 验证所有符号链接、配置文件和 launch 文件

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
WORKSPACE_SRC="$PROJECT_ROOT/automap_ws/src"

all_passed=true

print_header() {
    echo ""
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

print_ok() {
    echo -e "  ${GREEN}✓$1${NC}"
}

print_fail() {
    echo -e "  ${RED}✗$1${NC}"
    all_passed=false
}

print_warn() {
    echo -e "  ${YELLOW}⚠$1${NC}"
}

echo ""
echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     AutoMap-Pro 最终验证               ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"

# ========================================
# 1. 符号链接验证
# ========================================
print_header "1. 符号链接验证"

# automap_pro
AUTOMAP_SYMLINK="$WORKSPACE_SRC/automap_pro"
AUTOMAP_TARGET=$(readlink -f "$AUTOMAP_SYMLINK" 2>/dev/null || echo "")
if [ "$AUTOMAP_TARGET" == "$PROJECT_ROOT/automap_pro" ]; then
    print_ok "automap_pro 符号链接正确"
else
    print_fail "automap_pro 符号链接错误: $AUTOMAP_TARGET"
fi

# fast_livo
FAST_LIVO_SYMLINK="$WORKSPACE_SRC/fast_livo"
FAST_LIVO_TARGET=$(readlink -f "$FAST_LIVO_SYMLINK" 2>/dev/null || echo "")
if [ "$FAST_LIVO_TARGET" == "$PROJECT_ROOT/fast-livo2-humble" ]; then
    print_ok "fast_livo 符号链接正确"
else
    print_fail "fast_livo 符号链接错误: $FAST_LIVO_TARGET"
fi

# ========================================
# 2. 配置文件验证
# ========================================
print_header "2. 配置文件验证"

CONFIG="$WORKSPACE_SRC/automap_pro/config/system_config.yaml"

if [ -f "$CONFIG" ]; then
    print_ok "配置文件存在"
    
    # 检查行数（完整版本应该 >300 行）
    LINES=$(wc -l < "$CONFIG")
    if [ "$LINES" -gt 300 ]; then
        print_ok "配置文件完整 ($LINES 行)"
    else
        print_warn "配置文件可能不完整 ($LINES 行)"
    fi
    
    # 检查话题
    LIDAR_TOPIC=$(grep -A 2 "^  lidar:" "$CONFIG" | grep "topic:" | awk '{print $2}' | tr -d '"')
    IMU_TOPIC=$(grep -A 2 "^  imu:" "$CONFIG" | grep "topic:" | awk '{print $2}' | tr -d '"')
    
    if [ "$LIDAR_TOPIC" == "/os1_cloud_node1/points" ]; then
        print_ok "LiDAR 话题正确: $LIDAR_TOPIC"
    elif [ "$LIDAR_TOPIC" == "/livox/lidar" ]; then
        print_warn "LiDAR 话题是默认值 (Livox Avia): $LIDAR_TOPIC"
    else
        print_fail "LiDAR 话题不匹配: $LIDAR_TOPIC"
    fi
    
    if [ "$IMU_TOPIC" == "/imu/imu" ]; then
        print_ok "IMU 话题正确: $IMU_TOPIC"
    elif [ "$IMU_TOPIC" == "/livox/imu" ]; then
        print_warn "IMU 话题是默认值 (Livox Avia): $IMU_TOPIC"
    else
        print_fail "IMU 话题不匹配: $IMU_TOPIC"
    fi
else
    print_fail "配置文件不存在: $CONFIG"
fi

# ========================================
# 3. Launch 文件验证
# ========================================
print_header "3. Launch 文件验证"

LAUNCH="$WORKSPACE_SRC/automap_pro/launch/automap_offline.launch.py"

if [ -f "$LAUNCH" ]; then
    print_ok "launch 文件存在"
    
    # 检查是否移除了 camera_pinhole.yaml
    if grep -q "移除固定 remappings" "$LAUNCH"; then
        print_ok "已移除固定 remappings"
    else
        print_fail "未移除固定 remappings"
    fi
    
    # 检查是否移除了 camera_pinhole.yaml
    if ! grep -q "camera_pinhole.yaml" "$LAUNCH" || grep -q "# 注意：移除 camera_pinhole.yaml" "$LAUNCH"; then
        print_ok "已移除 camera_pinhole.yaml"
    else
        print_fail "仍在使用 camera_pinhole.yaml"
    fi
else
    print_fail "launch 文件不存在: $LAUNCH"
fi

# ========================================
# 4. Bag 文件验证
# ========================================
print_header "4. Bag 文件验证"

BAG="$PROJECT_ROOT/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2"

if [ -d "$BAG" ]; then
    print_ok "Bag 文件存在"
    
    if [ -f "$BAG/metadata.yaml" ]; then
        print_ok "metadata.yaml 存在"
    else
        print_warn "metadata.yaml 不存在"
    fi
else
    print_fail "Bag 文件不存在: $BAG"
fi

# ========================================
# 5. 源码目录验证
# ========================================
print_header "5. 源码目录验证"

if [ -d "$PROJECT_ROOT/automap_pro" ]; then
    print_ok "automap_pro 源码目录存在"
else
    print_fail "automap_pro 源码目录不存在"
fi

if [ -d "$PROJECT_ROOT/fast-livo2-humble" ]; then
    print_ok "fast-livo2-humble 源码目录存在"
else
    print_fail "fast-livo2-humble 源码目录不存在"
fi

# ========================================
# 6. 脚本验证
# ========================================
print_header "6. 脚本验证"

SCRIPTS=(
    "fix_symlink.sh"
    "fix_fast_livo_symlink.sh"
    "quick_verify.sh"
    "verify_and_run.sh"
    "run_full_mapping_docker.sh"
)

for script in "${SCRIPTS[@]}"; do
    if [ -f "$PROJECT_ROOT/$script" ]; then
        print_ok "脚本存在: $script"
    else
        print_warn "脚本不存在: $script"
    fi
done

# ========================================
# 总结
# ========================================
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}验证总结${NC}"
echo -e "${BLUE}========================================${NC}"

if [ "$all_passed" = true ]; then
    echo -e "${GREEN}✓✓✓ 所有检查通过！${NC}"
    echo ""
    echo -e "${CYAN}下一步：运行建图${NC}"
    echo -e "${CYAN}  ./run_full_mapping_docker.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2${NC}"
    exit 0
else
    echo -e "${RED}✗ 有检查失败，请先修复问题！${NC}"
    echo ""
    echo -e "${CYAN}修复建议：${NC}"
    echo -e "${CYAN}  1. 运行 ./fix_symlink.sh${NC}"
    echo -e "${CYAN}  2. 运行 ./fix_fast_livo_symlink.sh${NC}"
    echo -e "${CYAN}  3. 运行 ./final_verify.sh 重新验证${NC}"
    exit 1
fi
