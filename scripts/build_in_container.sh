#!/bin/bash
# ==========================================================================
# 在 Docker 容器内编译 AutoMap-Pro
#
# 使用方法:
#   bash scripts/build_in_container.sh [选项]
#
# 选项:
#   --clean         清理编译产物后重新编译
#   --release       Release 模式编译（默认）
#   --debug         Debug 模式编译
#   --package <name> 仅编译指定包
#
# 示例:
#   bash scripts/build_in_container.sh --clean
#   bash scripts/build_in_container.sh --package automap_pro
# ==========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="automap-env:humble"
WORKSPACE_DIR="${HOME}/automap_ws"

# 编译模式
BUILD_TYPE="Release"
CLEAN=false
PACKAGE=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean)
            CLEAN=true
            shift
            ;;
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --package)
            PACKAGE="$2"
            shift 2
            ;;
        --help|-h)
            echo "在 Docker 容器内编译 AutoMap-Pro"
            echo ""
            echo "使用方法: bash scripts/build_in_container.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --clean         清理编译产物后重新编译"
            echo "  --release       Release 模式编译（默认）"
            echo "  --debug         Debug 模式编译"
            echo "  --package <name> 仅编译指定包"
            echo "  --help          显示帮助信息"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            echo "使用 --help 查看帮助信息"
            exit 1
            ;;
    esac
done

echo "========================================="
echo "在容器内编译 AutoMap-Pro"
echo "========================================="
echo "镜像: ${IMAGE_NAME}"
echo "工作空间: ${WORKSPACE_DIR}"
echo "编译模式: ${BUILD_TYPE}"
echo "清理编译: ${CLEAN}"
if [ -n "$PACKAGE" ]; then
    echo "仅编译包: ${PACKAGE}"
fi
echo ""

# 清理编译产物
if [ "$CLEAN" = true ]; then
    echo "[INFO] 清理编译产物..."
    rm -rf "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"
    echo "✓ 清理完成"
fi

# 构建编译命令
BUILD_CMD="cd /root/automap_ws && source /opt/ros/humble/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

if [ -n "$PACKAGE" ]; then
    BUILD_CMD="${BUILD_CMD} --packages-select ${PACKAGE}"
fi

echo "[INFO] 开始编译..."
echo "[INFO] 编译命令: ${BUILD_CMD}"

# 在容器内编译
docker run --rm \
    --gpus all \
    --net=host \
    -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
    "${IMAGE_NAME}" \
    /bin/bash -c "${BUILD_CMD}"

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✓ 编译成功"
    echo "========================================="
else
    echo ""
    echo "========================================="
    echo "✗ 编译失败"
    echo "========================================="
    exit 1
fi
