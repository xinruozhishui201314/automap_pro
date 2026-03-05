#!/bin/bash
#
# 验证编译错误修复
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "========================================="
echo "验证编译错误修复"
echo "========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ️  $1${NC}"
}

FIXES_APPLIED=0
FIXES_TOTAL=4

# 1. 验证 LoadSession.srv
echo "[1/$FIXES_TOTAL] 检查 LoadSession.srv..."
if grep -q "string session_dir" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    print_success "session_dir 字段已添加"
    ((FIXES_APPLIED++))
else
    print_error "session_dir 字段缺失"
fi

if grep -q "uint32 submaps_loaded" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    print_success "submaps_loaded 字段已添加"
    ((FIXES_APPLIED++))
else
    print_error "submaps_loaded 字段缺失"
fi

if grep -q "uint32 descriptors_loaded" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    print_success "descriptors_loaded 字段已添加"
    ((FIXES_APPLIED++))
else
    print_error "descriptors_loaded 字段缺失"
fi

# 2. 验证 computeOdomInfoMatrix 声明
echo ""
echo "[2/$FIXES_TOTAL] 检查 automap_system.h..."
if grep -q "computeOdomInfoMatrix" "$PROJECT_ROOT/automap_pro/include/automap_pro/system/automap_system.h"; then
    print_success "computeOdomInfoMatrix 声明已添加"
    ((FIXES_APPLIED++))
else
    print_error "computeOdomInfoMatrix 声明缺失"
fi

# 3. 验证成员变量访问修复
echo ""
echo "[3/$FIXES_TOTAL] 检查 automap_system.cpp 成员变量访问..."
if grep -q "shutdown_requested_\\.load" "$PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"; then
    if ! grep -q "shared_this->shutdown_requested_" "$PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"; then
        print_success "shutdown_requested_ 访问已修复（直接访问）"
        ((FIXES_APPLIED++))
    else
        print_error "shutdown_requested_ 仍有错误访问（通过 shared_this）"
    fi
else
    print_error "未找到相关代码"
fi

# 4. 验证格式化字符串修复
echo ""
echo "[4/$FIXES_TOTAL] 检查格式化字符串..."
if grep -q "kf_id=%lu" "$PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"; then
    print_success "格式化字符串已修复（%lu 用于 uint64_t）"
    ((FIXES_APPLIED++))
else
    print_error "格式化字符串未修复"
fi

# 总结
echo ""
echo "========================================="
echo "验证总结"
echo "========================================="
echo "修复应用: $FIXES_APPLIED / $FIXES_TOTAL"
echo ""

if [ $FIXES_APPLIED -eq $FIXES_TOTAL ]; then
    print_success "所有修复已正确应用！"
    echo ""
    echo "下一步:"
    echo "  1. 编译项目:"
    echo "     bash run_automap.sh --build-only --clean"
    echo ""
    echo "  2. 或运行系统:"
    echo "     bash run_automap.sh"
    echo ""
    echo "详细信息请查看: $PROJECT_ROOT/COMPILATION_FIX_REPORT.md"
    exit 0
else
    print_error "部分修复未正确应用，请检查"
    echo ""
    print_info "请检查以下文件:"
    echo "  - $PROJECT_ROOT/automap_pro/srv/LoadSession.srv"
    echo "  - $PROJECT_ROOT/automap_pro/include/automap_pro/system/automap_system.h"
    echo "  - $PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"
    echo ""
    exit 1
fi
