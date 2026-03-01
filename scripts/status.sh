#!/bin/bash
# ==========================================================================
# 检查 AutoMap-Pro 系统状态
#
# 使用方法:
#   bash scripts/status.sh
# ==========================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="automap-env:humble"
WORKSPACE_DIR="${HOME}/automap_ws"

echo "========================================="
echo "AutoMap-Pro 系统状态检查"
echo "========================================="
echo ""

# ==================== Docker 镜像 ====================
echo "[Docker 镜像]"
if docker image inspect "${IMAGE_NAME}" &> /dev/null; then
    echo "  ✓ 镜像存在: ${IMAGE_NAME}"
    docker image inspect "${IMAGE_NAME}" --format='    大小: {{.Size}}' | awk '{print "    " substr($1, 1, length($1)-9) " GB"}'
    docker image inspect "${IMAGE_NAME}" --format='    创建时间: {{.Created}}' | awk '{print "    " substr($0, 0, 19)}'
else
    echo "  ✗ 镜像不存在: ${IMAGE_NAME}"
fi
echo ""

# ==================== 工作空间 ====================
echo "[工作空间: ${WORKSPACE_DIR}]"

if [ -d "${WORKSPACE_DIR}" ]; then
    echo "  ✓ 工作空间目录存在"

    # 检查源码
    if [ -L "${WORKSPACE_DIR}/src/automap_pro" ]; then
        echo "  ✓ 符号链接: ${WORKSPACE_DIR}/src/automap_pro -> $(readlink ${WORKSPACE_DIR}/src/automap_pro)"
    else
        echo "  ✗ 符号链接不存在: ${WORKSPACE_DIR}/src/automap_pro"
    fi

    # 检查编译状态
    if [ -f "${WORKSPACE_DIR}/install/automap_pro/share/automap_pro/package.xml" ]; then
        echo "  ✓ automap_pro 已编译"
        PKG_VERSION=$(grep -o '<version>.*</version>' "${WORKSPACE_DIR}/install/automap_pro/share/automap_pro/package.xml" | sed 's/<version>\(.*\)<\/version>/\1/')
        echo "    版本: ${PKG_VERSION}"
    else
        echo "  ✗ automap_pro 未编译"
    fi

    if [ -f "${WORKSPACE_DIR}/install/livox_ros_driver2/share/livox_ros_driver2/package.xml" ]; then
        echo "  ✓ livox_ros_driver2 已编译"
    else
        echo "  ✗ livox_ros_driver2 未编译"
    fi

    # 编译产物大小
    if [ -d "${WORKSPACE_DIR}/build" ]; then
        BUILD_SIZE=$(du -sh "${WORKSPACE_DIR}/build" 2>/dev/null | cut -f1)
        echo "  编译产物 (build): ${BUILD_SIZE}"
    fi

    if [ -d "${WORKSPACE_DIR}/install" ]; then
        INSTALL_SIZE=$(du -sh "${WORKSPACE_DIR}/install" 2>/dev/null | cut -f1)
        echo "  编译产物 (install): ${INSTALL_SIZE}"
    fi

else
    echo "  ✗ 工作空间目录不存在"
fi
echo ""

# ==================== 数据目录 ====================
echo "[数据目录]"
DATA_DIR="${HOME}/data"
if [ -d "${DATA_DIR}" ]; then
    echo "  ✓ 数据目录存在: ${DATA_DIR}"

    # 列出 rosbag 文件
    BAG_FILES=$(find "${DATA_DIR}" -type f \( -name "*.db3" -o -name "*.mcap" -o -name "*.bag" \) 2>/dev/null)
    if [ -n "$BAG_FILES" ]; then
        echo "  发现 rosbag 文件:"
        echo "$BAG_FILES" | while read bag; do
            BAG_SIZE=$(du -sh "$bag" | cut -f1)
            echo "    - $(basename "$bag") (${BAG_SIZE})"
        done
    else
        echo "  未找到 rosbag 文件"
    fi

    # 输出目录
    OUTPUT_DIR="${DATA_DIR}/automap_output"
    if [ -d "${OUTPUT_DIR}" ]; then
        echo "  ✓ 输出目录存在: ${OUTPUT_DIR}"
    fi
else
    echo "  ✗ 数据目录不存在: ${DATA_DIR}"
fi
echo ""

# ==================== GPU 支持 ====================
echo "[GPU 支持]"
if command -v nvidia-smi &> /dev/null; then
    if nvidia-smi &> /dev/null; then
        echo "  ✓ NVIDIA 驱动正常"
        GPU_COUNT=$(nvidia-smi --list-gpus | wc -l)
        echo "    GPU 数量: ${GPU_COUNT}"
        nvidia-smi --query-gpu=index,name,memory.total --format=csv,noheader | while IFS=, read -r idx name mem; do
            echo "    GPU ${idx}: ${name} (${mem})"
        done
    else
        echo "  ✗ NVIDIA 驱动不可用"
    fi
else
    echo "  ✗ 未安装 NVIDIA 驱动"
fi
echo ""

# ==================== Docker GPU 支持 ====================
echo "[Docker GPU 支持]"
if docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
    echo "  ✓ Docker NVIDIA 运行时正常"
else
    echo "  ✗ Docker NVIDIA 运行时不可用"
fi
echo ""

echo "========================================="
echo "✓ 状态检查完成"
echo "========================================="
