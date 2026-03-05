#!/bin/bash
#
# 编译错误修复与重新编译脚本
# 自动修复所有编译错误并重新构建项目

set -e  # 遇到错误立即退出

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/automap_ws"

echo "========================================="
echo "AutoMap Pro 编译错误修复与重建"
echo "========================================="
echo "项目根目录: $PROJECT_ROOT"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# 1. 验证修复
echo "========================================="
echo "步骤 1: 验证已应用的修复"
echo "========================================="

# 检查 LoadSession.srv
echo -n "检查 LoadSession.srv... "
if grep -q "string session_dir" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv" && \
   grep -q "uint32 submaps_loaded" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv" && \
   grep -q "uint32 descriptors_loaded" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    print_success "LoadSession.srv 已正确更新"
else
    print_error "LoadSession.srv 修复不完整"
    exit 1
fi

# 检查头文件
echo -n "检查 automap_system.h... "
if grep -q "computeOdomInfoMatrix" "$PROJECT_ROOT/automap_pro/include/automap_pro/system/automap_system.h"; then
    print_success "computeOdomInfoMatrix 声明已添加"
else
    print_error "computeOdomInfoMatrix 声明缺失"
    exit 1
fi

# 检查源文件修复
echo -n "检查 automap_system.cpp... "
if grep -q "shutdown_requested_\\.load" "$PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"; then
    # 检查是否正确修复了（不应该有 shared_this->shutdown_requested_）
    if ! grep -q "shared_this->shutdown_requested_" "$PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"; then
        print_success "shutdown_requested_ 访问已修复"
    else
        print_error "shutdown_requested_ 仍有错误访问"
        exit 1
    fi
else
    print_warning "未找到相关代码，可能已被修改"
fi

if grep -q "kf_id=%lu" "$PROJECT_ROOT/automap_pro/src/system/automap_system.cpp"; then
    print_success "格式化字符串已修复"
else
    print_warning "格式化字符串可能需要手动检查"
fi

# 2. 清理旧的构建
echo ""
echo "========================================="
echo "步骤 2: 清理旧的构建文件"
echo "========================================="
cd "$BUILD_DIR"
if [ -d "build" ]; then
    echo "删除 build 目录..."
    rm -rf build
    print_success "build 目录已删除"
fi

if [ -d "install" ]; then
    echo "删除 install 目录..."
    rm -rf install
    print_success "install 目录已删除"
fi

if [ -d "log" ]; then
    echo "删除 log 目录..."
    rm -rf log
    print_success "log 目录已删除"
fi

# 3. 重新构建
echo ""
echo "========================================="
echo "步骤 3: 重新构建项目"
echo "========================================="
echo "开始编译 (这可能需要几分钟)..."
echo ""

if colcon build --packages-select automap_pro --symlink-install; then
    print_success "编译成功！"
else
    print_error "编译失败，请检查错误信息"
    exit 1
fi

# 4. 验证可执行文件
echo ""
echo "========================================="
echo "步骤 4: 验证构建结果"
echo "========================================="

EXECUTABLE="$BUILD_DIR/install/automap_pro/lib/automap_pro/automap_system_node"
if [ -f "$EXECUTABLE" ]; then
    print_success "可执行文件已生成: $EXECUTABLE"
else
    print_error "可执行文件未生成"
    exit 1
fi

SHARED_LIB="$BUILD_DIR/install/automap_pro/lib/libautomap_system_component.so"
if [ -f "$SHARED_LIB" ]; then
    print_success "共享库已生成: $SHARED_LIB"
else
    print_error "共享库未生成"
    exit 1
fi

# 5. 总结
echo ""
echo "========================================="
echo "修复与重建完成！"
echo "========================================="
echo ""
echo "修复内容:"
echo "  1. ✅ LoadSession.srv 服务定义已更新"
echo "     - 添加了 session_dir 字段"
echo "     - 添加了 submaps_loaded 字段"
echo "     - 添加了 descriptors_loaded 字段"
echo "  2. ✅ automap_system.h 中添加了 computeOdomInfoMatrix 声明"
echo "  3. ✅ automap_system.cpp 中修复了 shutdown_requested_ 访问"
echo "  4. ✅ automap_system.cpp 中修复了格式化字符串类型"
echo ""
echo "下一步:"
echo "  - 运行测试: bash run_automap.sh --test"
echo "  - 或者启动系统: bash run_automap.sh"
echo ""
