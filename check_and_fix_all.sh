#!/bin/bash
# 全面检查和修复符号链接
# 检查所有符号链接，修复指向错误路径的问题

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
WORKSPACE_SRC="$PROJECT_ROOT/automap_ws/src"

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

print_header() {
    echo ""
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

total_issues=0
fixed_issues=0

echo -e "${CYAN}╔════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║    AutoMap-Pro 符号链接全面检查        ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════╝${NC}"

# ========================================
# 1. 检查 automap_ws/src/ 下的符号链接
# ========================================
print_header "1. 检查 automap_ws/src/ 符号链接"

symlinks=(
    "automap_pro|${PROJECT_ROOT}/automap_pro"
    "fast_livo|${PROJECT_ROOT}/fast-livo2-humble"
    "hba|${PROJECT_ROOT}/HBA-main/HBA_ROS2"
    "overlap_transformer_msgs|${PROJECT_ROOT}/overlap_transformer_msgs"
    "overlap_transformer_ros2|${PROJECT_ROOT}/overlap_transformer_ros2"
)

for item in "${symlinks[@]}"; do
    IFS='|' read -r name target <<< "$item"
    link="$WORKSPACE_SRC/$name"
    
    echo -n "  $name: "
    
    if [ ! -L "$link" ]; then
        echo -e "${YELLOW}不是符号链接或不存在${NC}"
        continue
    fi
    
    current_target=$(readlink -f "$link" 2>/dev/null || echo "")
    
    if [ -z "$current_target" ]; then
        echo -e "${RED}损坏${NC}"
        ((total_issues++))
        continue
    fi
    
    if [ ! -e "$current_target" ]; then
        echo -e "${RED}指向不存在: $current_target${NC}"
        ((total_issues++))
        
        # 尝试修复
        if [ -d "$target" ]; then
            log_warn "  尝试修复..."
            rm "$link" 2>/dev/null || true
            ln -s "$target" "$link"
            new_target=$(readlink -f "$link" 2>/dev/null || echo "")
            
            if [ "$new_target" == "$target" ]; then
                echo -e "  ${GREEN}✓ 已修复${NC}"
                ((fixed_issues++))
            else
                echo -e "  ${RED}✗ 修复失败${NC}"
            fi
        else
            echo -e "  ${RED}✗ 目标不存在: $target${NC}"
        fi
    else
        if [[ "$current_target" =~ ^/root/ ]]; then
            echo -e "${YELLOW}容器路径 (可能不影响): $current_target${NC}"
        else
            echo -e "${GREEN}✓ 正确${NC}"
        fi
    fi
done

echo ""

# ========================================
# 2. 检查实际目录是否存在
# ========================================
print_header "2. 检查实际目录"

directories=(
    "$PROJECT_ROOT/automap_pro"
    "$PROJECT_ROOT/fast-livo2-humble"
    "$PROJECT_ROOT/HBA-main"
    "$PROJECT_ROOT/OverlapTransformer-master"
    "$PROJECT_ROOT/overlap_transformer_msgs"
    "$PROJECT_ROOT/overlap_transformer_ros2"
)

for dir in "${directories[@]}"; do
    name=$(basename "$dir")
    echo -n "  $name: "
    
    if [ -d "$dir" ]; then
        if [ -f "$dir/package.xml" ] || [ -f "$dir/CMakeLists.txt" ] || [ -d "$dir/include" ]; then
            echo -e "${GREEN}✓ 存在 (有效)${NC}"
        else
            echo -e "${YELLOW}⚠ 存在 (可能不是有效包)${NC}"
        fi
    else
        echo -e "${RED}✗ 不存在${NC}"
        ((total_issues++))
    fi
done

echo ""

# ========================================
# 3. 检查配置文件
# ========================================
print_header "3. 检查配置文件"

CONFIG="$WORKSPACE_SRC/automap_pro/config/system_config.yaml"

if [ -f "$CONFIG" ]; then
    LINES=$(wc -l < "$CONFIG")
    echo "  配置文件存在 (${LINES} 行)"
    
    # 检查话题
    LIDAR_TOPIC=$(grep -A 2 "^  lidar:" "$CONFIG" | grep "topic:" | awk '{print $2}' | tr -d '"')
    IMU_TOPIC=$(grep -A 2 "^  imu:" "$CONFIG" | grep "topic:" | awk '{print $2}' | tr -d '"')
    
    echo "  LiDAR 话题: $LIDAR_TOPIC"
    echo "  IMU 话题: $IMU_TOPIC"
    
    if [ "$LIDAR_TOPIC" == "/os1_cloud_node1/points" ] && [ "$IMU_TOPIC" == "/imu/imu" ]; then
        echo -e "  ${GREEN}✓ 话题正确 (nya_02)${NC}"
    elif [ "$LIDAR_TOPIC" == "/livox/lidar" ] && [ "$IMU_TOPIC" == "/livox/imu" ]; then
        echo -e "  ${YELLOW}⚠ 话题是默认值 (Livox Avia)${NC}"
    else
        echo -e "  ${YELLOW}⚠ 话题不匹配${NC}"
    fi
else
    echo -e "  ${RED}✗ 配置文件不存在${NC}"
    ((total_issues++))
fi

echo ""

# ========================================
# 4. 检查 launch 文件
# ========================================
print_header "4. 检查 launch 文件"

LAUNCH="$WORKSPACE_SRC/automap_pro/launch/automap_offline.launch.py"

if [ -f "$LAUNCH" ]; then
    echo "  launch 文件存在"
    
    # 检查修改
    if grep -q "移除固定 remappings" "$LAUNCH"; then
        echo -e "  ${GREEN}✓ 已移除固定 remappings${NC}"
    else
        echo -e "  ${YELLOW}⚠ 未移除固定 remappings${NC}"
    fi
    
    if ! grep -q "camera_pinhole.yaml" "$LAUNCH" || grep -q "# 注意：移除 camera_pinhole.yaml" "$LAUNCH"; then
        echo -e "  ${GREEN}✓ 已移除 camera_pinhole.yaml${NC}"
    else
        echo -e "  ${RED}✗ 仍在使用 camera_pinhole.yaml${NC}"
        ((total_issues++))
    fi
else
    echo -e "  ${RED}✗ launch 文件不存在${NC}"
    ((total_issues++))
fi

echo ""

# ========================================
# 5. 总结
# ========================================
print_header "检查总结"

echo -e "  发现问题: ${RED}$total_issues${NC}"
echo -e "  已修复:   ${GREEN}$fixed_issues${NC}"
echo ""

if [ $total_issues -eq 0 ]; then
    echo -e "${GREEN}✓✓✓ 所有检查通过！${NC}"
    echo ""
    echo -e "${CYAN}下一步：运行建图${NC}"
    echo -e "${CYAN}  ./run_full_mapping_docker.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2${NC}"
    exit 0
else
    echo -e "${YELLOW}⚠ 发现 $total_issues 个问题，部分已自动修复${NC}"
    echo ""
    
    if [ $fixed_issues -gt 0 ]; then
        echo -e "${GREEN}已修复 $fixed_issues 个问题${NC}"
    fi
    
    remaining=$((total_issues - fixed_issues))
    if [ $remaining -gt 0 ]; then
        echo -e "${RED}还有 $remaining 个问题需要手动修复${NC}"
        echo ""
        echo -e "${CYAN}建议：${NC}"
        echo "  1. 查看上面的错误信息"
        echo "  2. 运行 ./final_verify.sh 详细验证"
        echo "  3. 检查相关目录是否存在"
    fi
    
    exit 1
fi
