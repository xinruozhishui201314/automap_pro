#!/bin/bash
# 修复 fast_livo 符号链接
# 确保指向正确的 fast-livo2-humble 源码

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
WORKSPACE_SRC="$PROJECT_ROOT/automap_ws/src"
TARGET_DIR="$PROJECT_ROOT/fast-livo2-humble"
SYMLINK="$WORKSPACE_SRC/fast_livo"

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
echo "修复 fast_livo 符号链接"
echo "========================================"
echo ""

# 1. 检查当前符号链接
log_step "1. 检查当前符号链接状态"
if [ -L "$SYMLINK" ]; then
    CURRENT_TARGET=$(readlink -f "$SYMLINK")
    echo -e "  当前链接: ${YELLOW}$SYMLINK${NC}"
    echo -e "  指向目标: ${RED}$CURRENT_TARGET${NC}"
    echo ""
    
    if [ "$CURRENT_TARGET" == "$TARGET_DIR" ]; then
        log_info "✓ 符号链接已正确，无需修复"
        exit 0
    else
        log_warn "符号链接指向错误路径！"
        log_info "应该指向: $TARGET_DIR"
    fi
else
    log_error "符号链接不存在！"
    exit 1
fi

# 2. 确认目标目录存在
log_step "2. 确认目标目录存在"
if [ -d "$TARGET_DIR" ]; then
    log_info "✓ 目标目录存在: $TARGET_DIR"
    
    # 检查是否有关键文件
    if [ -f "$TARGET_DIR/package.xml" ]; then
        log_info "✓ package.xml 存在（有效 ROS2 包）"
    else
        log_warn "package.xml 不存在，可能不是有效的 ROS2 包"
    fi
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

# 6. 验证包配置
log_step "6. 验证包配置"
if [ -f "$SYMLINK/package.xml" ]; then
    PACKAGE_NAME=$(grep -o '<name>[^<]*' "$SYMLINK/package.xml" | head -1 | sed 's/<name>//g')
    echo "  包名: ${GREEN}$PACKAGE_NAME${NC}"
    log_info "✓ 有效的 ROS2 包"
else
    log_warn "package.xml 不存在"
fi
echo ""

# 7. 对比 automap_pro 和 fast_livo 符号链接
log_step "7. 检查所有关键符号链接"
AUTOMAP_SYMLINK="$WORKSPACE_SRC/automap_pro"
AUTOMAP_TARGET=$(readlink -f "$AUTOMAP_SYMLINK" 2>/dev/null || echo "不存在")

echo "  automap_pro: $AUTOMAP_SYMLINK"
echo "    → $AUTOMAP_TARGET"
echo "  fast_livo: $SYMLINK"
echo "    → $NEW_TARGET"
echo ""

if [ "$AUTOMAP_TARGET" == "$PROJECT_ROOT/automap_pro" ]; then
    log_info "✓ automap_pro 符号链接正确"
else
    log_warn "automap_pro 符号链接可能需要修复"
fi

if [ "$NEW_TARGET" == "$TARGET_DIR" ]; then
    log_info "✓ fast_livo 符号链接正确"
fi

echo ""
echo "========================================"
echo "修复完成！"
echo "========================================"
echo ""
log_info "下一步：运行 ./run_full_mapping_docker.sh"
echo ""
