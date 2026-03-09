#!/bin/bash
# ==========================================================================
# 清理 AutoMap-Pro 编译产物和临时文件
#
# 使用方法:
#   bash scripts/clean.sh [选项]
#
# 选项:
#   --workspace     清理工作空间（build、install、log）
#   --docker        清理 Docker 容器和镜像
#   --all           清理所有（工作空间 + Docker）
#   --help          显示帮助信息
#
# 示例:
#   bash scripts/clean.sh --workspace
#   bash scripts/clean.sh --all
# ==========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${HOME}/automap_ws"
IMAGE_NAME="automap-env:humble"

# 清理选项
CLEAN_WORKSPACE=false
CLEAN_DOCKER=false
CLEAN_ALL=false

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --workspace)
            CLEAN_WORKSPACE=true
            shift
            ;;
        --docker)
            CLEAN_DOCKER=true
            shift
            ;;
        --all)
            CLEAN_ALL=true
            shift
            ;;
        --help|-h)
            echo "清理 AutoMap-Pro 编译产物和临时文件"
            echo ""
            echo "使用方法: bash scripts/clean.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --workspace     清理工作空间（build、install、log）"
            echo "  --docker        清理 Docker 容器和镜像"
            echo "  --all           清理所有（工作空间 + Docker）"
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

# --all 意味着清理所有
if [ "$CLEAN_ALL" = true ]; then
    CLEAN_WORKSPACE=true
    CLEAN_DOCKER=true
fi

echo "========================================="
echo "清理 AutoMap-Pro"
echo "========================================="

# 清理工作空间
if [ "$CLEAN_WORKSPACE" = true ]; then
    echo "[INFO] 清理工作空间..."
    echo "  - ${WORKSPACE_DIR}/build"
    echo "  - ${WORKSPACE_DIR}/install"
    echo "  - ${WORKSPACE_DIR}/log"

    if [ -d "${WORKSPACE_DIR}/build" ]; then
        rm -rf "${WORKSPACE_DIR}/build"
        echo "  ✓ 已删除 build"
    fi

    if [ -d "${WORKSPACE_DIR}/install" ]; then
        rm -rf "${WORKSPACE_DIR}/install"
        echo "  ✓ 已删除 install"
    fi

    if [ -d "${WORKSPACE_DIR}/log" ]; then
        rm -rf "${WORKSPACE_DIR}/log"
        echo "  ✓ 已删除 log"
    fi

    echo "✓ 工作空间清理完成"
    echo ""
fi

# 清理 Docker
if [ "$CLEAN_DOCKER" = true ]; then
    echo "[INFO] 清理 Docker..."

    # 停止并删除相关容器
    RUNNING_CONTAINERS=$(docker ps -q --filter ancestor="${IMAGE_NAME}")
    if [ -n "$RUNNING_CONTAINERS" ]; then
        echo "[INFO] 停止运行中的容器..."
        docker stop $RUNNING_CONTAINERS
        docker rm $RUNNING_CONTAINERS
        echo "  ✓ 已停止并删除容器"
    fi

    # 删除镜像
    if docker image inspect "${IMAGE_NAME}" &> /dev/null; then
        echo "[INFO] 删除镜像: ${IMAGE_NAME}"
        docker rmi "${IMAGE_NAME}"
        echo "  ✓ 已删除镜像"
    fi

    # 删除镜像归档文件
    IMAGE_ARCHIVE="${PROJECT_DIR}/docker/automap-env_humble.tar"
    if [ -f "${IMAGE_ARCHIVE}" ]; then
        echo "[INFO] 删除镜像归档: ${IMAGE_ARCHIVE}"
        rm -f "${IMAGE_ARCHIVE}"
        echo "  ✓ 已删除镜像归档"
    fi

    echo "✓ Docker 清理完成"
    echo ""
fi

echo "========================================="
echo "✓ 清理完成"
echo "========================================="
