#!/bin/bash
#
# 编译错误修复验证脚本
# 用于验证 LoadSession.srv 和其他编译错误是否已修复

set -e  # 遇到错误立即退出

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/automap_ws/build/automap_pro"

echo "========================================="
echo "编译错误修复验证"
echo "========================================="
echo "项目根目录: $PROJECT_ROOT"
echo ""

# 1. 检查 LoadSession.srv 是否已更新
echo "[1/4] 检查 LoadSession.srv 文件..."
if grep -q "string session_dir" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    echo "✅ LoadSession.srv 已添加 session_dir 字段"
else
    echo "❌ LoadSession.srv 缺少 session_dir 字段"
    exit 1
fi

if grep -q "uint32 submaps_loaded" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    echo "✅ LoadSession.srv 已添加 submaps_loaded 字段"
else
    echo "❌ LoadSession.srv 缺少 submaps_loaded 字段"
    exit 1
fi

if grep -q "uint32 descriptors_loaded" "$PROJECT_ROOT/automap_pro/srv/LoadSession.srv"; then
    echo "✅ LoadSession.srv 已添加 descriptors_loaded 字段"
else
    echo "❌ LoadSession.srv 缺少 descriptors_loaded 字段"
    exit 1
fi

# 2. 检查头文件中是否有 computeOdomInfoMatrix 声明
echo ""
echo "[2/4] 检查 automap_system.h 文件..."
if grep -q "computeOdomInfoMatrix" "$PROJECT_ROOT/automap_pro/include/automap_pro/system/automap_system.h"; then
    echo "✅ automap_system.h 已添加 computeOdomInfoMatrix 声明"
else
    echo "❌ automap_system.h 缺少 computeOdomInfoMatrix 声明"
    exit 1
fi

# 3. 清理旧的构建文件
echo ""
echo "[3/4] 清理旧的构建文件..."
cd "$PROJECT_ROOT/automap_ws"
if [ -d "build" ]; then
    rm -rf build install log
    echo "✅ 已清理 build/install/log 目录"
else
    echo "⚠️  build 目录不存在，跳过清理"
fi

# 4. 重新编译
echo ""
echo "[4/4] 重新编译项目..."
colcon build --packages-select automap_pro --symlink-install

echo ""
echo "========================================="
echo "✅ 编译成功！所有修复已验证"
echo "========================================="
