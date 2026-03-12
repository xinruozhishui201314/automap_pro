#!/bin/bash
# ============================================================================
# GTSAM 编译脚本 - 禁用 TBB 以避免 SIGSEGV 崩溃
# ============================================================================
# 使用方法:
#   bash build_gtsam_no_tbb.sh
#
# 编译后 GTSAM 将安装到 /usr/local/
# automap_pro 会自动使用这个版本
# ============================================================================

set -e

GTSAM_SOURCE_DIR="/root/automap_ws/src/automap_pro/thrid_party/gtsam"
BUILD_DIR="/root/automap_ws/build_gtsam_no_tbb"
INSTALL_PREFIX="/usr/local"

echo "========================================"
echo "GTSAM 编译 (禁用 TBB)"
echo "========================================"
echo "源码目录: ${GTSAM_SOURCE_DIR}"
echo "安装目录: ${INSTALL_PREFIX}"
echo ""

# 检查源码目录
if [ ! -d "${GTSAM_SOURCE_DIR}" ]; then
    echo "错误: GTSAM 源码目录不存在: ${GTSAM_SOURCE_DIR}"
    exit 1
fi

# 创建构建目录
mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}

# 配置 CMake
echo "配置 GTSAM CMake..."
cmake ${GTSAM_SOURCE_DIR} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON=OFF

# 编译
echo ""
echo "编译 GTSAM (这可能需要几分钟)..."
make -j$(nproc)

# 安装
echo ""
echo "安装 GTSAM 到 ${INSTALL_PREFIX}..."
sudo make install

# 更新动态链接库缓存
echo ""
echo "更新动态链接库缓存..."
sudo ldconfig

echo ""
echo "========================================"
echo "GTSAM 编译完成!"
echo "========================================"
echo "已安装到: ${INSTALL_PREFIX}"
echo "已禁用 TBB 支持"
echo ""
echo "现在可以重新编译 automap_pro:"
echo "  cd /root/automap_ws"
echo "  colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo ""
