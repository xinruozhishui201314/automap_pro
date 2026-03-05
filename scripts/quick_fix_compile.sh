#!/bin/bash
#
# 快速修复编译错误并重新编译 automap_pro
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "========================================="
echo "AutoMap Pro 编译错误快速修复"
echo "========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# 1. 验证修复已应用
echo "[1/3] 验证修复..."
if grep -q "string session_dir" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    print_success "LoadSession.srv 已更新"
else
    print_error "LoadSession.srv 未更新"
    exit 1
fi

# 2. 在Docker容器中重新编译 automap_pro
echo ""
echo "[2/3] 重新编译 automap_pro..."

docker run --rm \
    --gpus all \
    --net=host \
    -v "${PROJECT_ROOT}/automap_ws:/root/automap_ws:rw" \
    -v "${PROJECT_ROOT}/automap_pro:/root/automap_ws/src/automap_pro:ro" \
    automap-env:humble \
    /bin/bash -c "
        set -e
        source /opt/ros/humble/setup.bash
        cd /root/automap_ws

        # 删除旧的 automap_pro 编译产物
        rm -rf build/automap_pro install/automap_pro log/latest_build/automap_pro

        # 编译 automap_pro
        colcon build --packages-select automap_pro --symlink-install

        # 验证编译结果
        if [ -f install/automap_pro/lib/libautomap_system_component.so ]; then
            echo '✅ 共享库编译成功'
        else
            echo '❌ 共享库编译失败'
            exit 1
        fi

        if [ -f install/automap_pro/lib/automap_pro/automap_system_node ]; then
            echo '✅ 可执行文件编译成功'
        else
            echo '❌ 可执行文件编译失败'
            exit 1
        fi
    "

if [ $? -eq 0 ]; then
    print_success "编译成功！"
else
    print_error "编译失败"
    exit 1
fi

# 3. 验证编译产物
echo ""
echo "[3/3] 验证编译产物..."
if [ -f "$PROJECT_ROOT/automap_ws/install/automap_pro/lib/libautomap_system_component.so" ]; then
    print_success "共享库存在"
else
    print_error "共享库不存在"
    exit 1
fi

if [ -f "$PROJECT_ROOT/automap_ws/install/automap_pro/lib/automap_pro/automap_system_node" ]; then
    print_success "可执行文件存在"
else
    print_error "可执行文件不存在"
    exit 1
fi

echo ""
echo "========================================="
echo "🎉 所有修复已完成！"
echo "========================================="
echo ""
echo "修复内容:"
echo "  1. ✅ LoadSession.srv 服务定义已更新"
echo "  2. ✅ automap_system.h 添加了函数声明"
echo "  3. ✅ automap_system.cpp 修复了成员变量访问"
echo "  4. ✅ automap_system.cpp 修复了格式化字符串"
echo ""
echo "下一步:"
echo "  - 运行测试: bash run_automap.sh --build-only"
echo "  - 或者直接运行: bash run_automap.sh"
echo ""
