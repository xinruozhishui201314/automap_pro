#!/bin/bash
# 修复 automap_ws 符号链接
# 确保指向正确的源码目录

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
WORKSPACE_SRC="$PROJECT_ROOT/automap_ws/src"
TARGET_DIR="$PROJECT_ROOT/automap_pro"
SYMLINK="$WORKSPACE_SRC/automap_pro"

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

echo "========================================"
echo "修复 automap_ws 符号链接"
echo "========================================"
echo ""

# 1. 检查当前符号链接
log_step "1. 检查当前符号链接状态"
if [ -L "$SYMLINK" ]; then
    CURRENT_TARGET=$(readlink -f "$SYMLINK")
    echo -e "  当前链接: ${YELLOW}$SYMLINK${NC}"
    echo -e "  指向目标: ${RED}$CURRENT_TARGET${NC}"
    echo ""
    
    if [ "$CURRENT_TARGET" != "$TARGET_DIR" ]; then
        log_warn "符号链接指向错误路径！"
        log_info "应该指向: $TARGET_DIR"
    else
        log_info "✓ 符号链接已正确，无需修复"
        exit 0
    fi
else
    log_error "符号链接不存在！"
    exit 1
fi

# 2. 确认目标目录存在
log_step "2. 确认目标目录存在"
if [ -d "$TARGET_DIR" ]; then
    log_info "✓ 目标目录存在: $TARGET_DIR"
else
    log_error "目标目录不存在: $TARGET_DIR"
    exit 1
fi
echo ""

# 3. 删除旧符号链接
log_step "3. 删除旧符号链接"
log_warn "删除: $SYMLINK"
rm "$SYMLINK"
log_info "✓ 已删除旧符号链接"
echo ""

# 4. 创建新符号链接
log_step "4. 创建新符号链接"
log_info "创建链接: $SYMLINK -> $TARGET_DIR"
ln -s "$TARGET_DIR" "$SYMLINK"
log_info "✓ 符号链接已更新"
echo ""

# 5. 验证新链接
log_step "5. 验证新符号链接"
NEW_TARGET=$(readlink -f "$SYMLINK")
echo -e "  新链接: ${GREEN}$SYMLINK${NC}"
echo -e "  指向目标: ${GREEN}$NEW_TARGET${NC}"
echo ""

if [ "$NEW_TARGET" == "$TARGET_DIR" ]; then
    log_info "✓✓✓ 符号链接修复成功！"
else
    log_error "✗ 符号链接修复失败！"
    exit 1
fi

# 6. 验证配置文件
log_step "6. 验证配置文件"
CONFIG_FILE="$SYMLINK/config/system_config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    LIDAR_TOPIC=$(grep -A 2 "lidar:" "$CONFIG_FILE" | grep "topic:" | awk '{print $2}')
    IMU_TOPIC=$(grep -A 2 "imu:" "$CONFIG_FILE" | grep "topic:" | awk '{print $2}')
    
    echo "  配置文件: $CONFIG_FILE"
    echo "  LiDAR 话题: ${GREEN}$LIDAR_TOPIC${NC}"
    echo "  IMU 话题: ${GREEN}$IMU_TOPIC${NC}"
    echo ""
    
    if [ "$LIDAR_TOPIC" == "/os1_cloud_node1/points" ] && [ "$IMU_TOPIC" == "/imu/imu" ]; then
        log_info "✓ 配置文件正确（nya_02 数据集）"
    elif [ "$LIDAR_TOPIC" == "/livox/lidar" ] && [ "$IMU_TOPIC" == "/livox/imu" ]; then
        log_warn "配置文件是默认值（Livox Avia），请检查是否使用了正确的配置"
    else
        log_warn "配置文件话题不匹配，请确认"
    fi
else
    log_warn "配置文件不存在: $CONFIG_FILE"
fi

echo ""
echo "========================================"
echo "修复完成！"
echo "========================================"
echo ""
log_info "下一步：运行 ./run_full_mapping_docker.sh"
echo ""
