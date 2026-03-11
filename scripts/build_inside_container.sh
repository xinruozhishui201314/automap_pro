#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd /root/automap_ws

# 尽可能并行编译：包内多核编译 + 多包并行
PARALLEL_JOBS=$(nproc)
export CMAKE_BUILD_PARALLEL_LEVEL="${PARALLEL_JOBS}"
COLCON_PARALLEL="--parallel-workers ${PARALLEL_JOBS}"
echo "[INFO] 并行编译: 每包 ${PARALLEL_JOBS} 线程, colcon --parallel-workers ${PARALLEL_JOBS}"

# 第三方库安装到 install_deps，--clean 时不删除，编译一次后续跳过
INSTALL_DEPS="/root/automap_ws/install_deps"
mkdir -p "${INSTALL_DEPS}"

# 若 CMake 缓存为其他路径创建，清理（不删 install_deps）
if [ -d build ] && find build -name CMakeCache.txt -exec grep -l '/workspace/' {} \; 2>/dev/null | grep -q .; then
  echo '[WARN] 检测到 CMake 缓存路径不一致，清理 build/install/log'
  rm -rf build install log build_teaserpp build_sophus build_ceres
fi

# 链入 overlap_transformer_msgs / overlap_transformer_ros2 / hba（若存在）
[ -d /root/mapping/overlap_transformer_msgs ] && ln -sf /root/mapping/overlap_transformer_msgs src/ 2>/dev/null || true
[ -d /root/mapping/overlap_transformer_ros2 ] && ln -sf /root/mapping/overlap_transformer_ros2 src/ 2>/dev/null || true
[ -d /root/mapping/HBA-main/HBA_ROS2 ]        && ln -sf /root/mapping/HBA-main/HBA_ROS2 src/hba 2>/dev/null || true

if [ -d src/overlap_transformer_msgs ]; then
  echo '========================================'
  echo '编译 overlap_transformer_msgs'
  echo '========================================'
  colcon build ${COLCON_PARALLEL} --packages-select overlap_transformer_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [ -d src/overlap_transformer_ros2 ]; then
  echo '========================================'
  echo '编译 overlap_transformer_ros2'
  echo '========================================'
  colcon build ${COLCON_PARALLEL} --packages-select overlap_transformer_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [ -d src/hba ]; then
  echo '========================================'
  echo 'Compile GTSAM first (disable TBB to avoid SIGSEGV)'
  echo '========================================'
  GTSAM_INSTALL_DIR="${INSTALL_DEPS}/gtsam"
  NEED_GTSAM_BUILD=false
  if [ ! -f "${GTSAM_INSTALL_DIR}/lib/libgtsam.so" ]; then
    NEED_GTSAM_BUILD=true
  else
    # 已安装的 GTSAM 若与当前环境 Eigen 版本不一致会导致 hba 编译报错，需强制重编
    CURRENT_EIGEN_WORLD=$(grep -m1 '#define EIGEN_WORLD_VERSION' /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | sed 's/.*EIGEN_WORLD_VERSION[^0-9]*\([0-9]*\).*/\1/' || echo "")
    CURRENT_EIGEN_MAJOR=$(grep -m1 '#define EIGEN_MAJOR_VERSION' /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | sed 's/.*EIGEN_MAJOR_VERSION[^0-9]*\([0-9]*\).*/\1/' || echo "")
    GTSAM_CFG="${GTSAM_INSTALL_DIR}/include/gtsam/config.h"
    if [ -n "${CURRENT_EIGEN_WORLD}${CURRENT_EIGEN_MAJOR}" ] && [ -f "${GTSAM_CFG}" ]; then
      GTSAM_EIGEN_WORLD=$(grep -m1 '#define GTSAM_EIGEN_VERSION_WORLD' "${GTSAM_CFG}" 2>/dev/null | sed 's/.*GTSAM_EIGEN_VERSION_WORLD[^0-9]*\([0-9]*\).*/\1/' || echo "")
      GTSAM_EIGEN_MAJOR=$(grep -m1 '#define GTSAM_EIGEN_VERSION_MAJOR' "${GTSAM_CFG}" 2>/dev/null | sed 's/.*GTSAM_EIGEN_VERSION_MAJOR[^0-9]*\([0-9]*\).*/\1/' || echo "")
      if [ "${CURRENT_EIGEN_WORLD}" != "${GTSAM_EIGEN_WORLD}" ] || [ "${CURRENT_EIGEN_MAJOR}" != "${GTSAM_EIGEN_MAJOR}" ]; then
        echo "[INFO] Eigen 版本与已安装 GTSAM 不一致 (当前 ${CURRENT_EIGEN_WORLD}.${CURRENT_EIGEN_MAJOR} vs GTSAM 构建时 ${GTSAM_EIGEN_WORLD}.${GTSAM_EIGEN_MAJOR})，将重新编译 GTSAM"
        rm -rf "${GTSAM_INSTALL_DIR}" /root/automap_ws/build_gtsam_no_tbb
        NEED_GTSAM_BUILD=true
      fi
    fi
  fi
  if [ "$NEED_GTSAM_BUILD" = true ]; then
      echo "[INFO] Building GTSAM without TBB..."
      if [ -d /root/automap_ws/build_gtsam_no_tbb ]; then
          rm -rf /root/automap_ws/build_gtsam_no_tbb
      fi
      mkdir -p /root/automap_ws/build_gtsam_no_tbb
      cd /root/automap_ws/build_gtsam_no_tbb
      cmake /root/automap_ws/src/automap_pro/thrid_party/gtsam \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=${GTSAM_INSTALL_DIR} \
          -DGTSAM_WITH_TBB=OFF \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DBUILD_SHARED_LIBS=ON \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_PYTHON=OFF
      make -j$(nproc)
      make install
      mkdir -p ${GTSAM_INSTALL_DIR}/share/gtsam
      echo '<?xml version="1.0"?>' > ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '<package format="3">' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '  <name>gtsam</name>' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '  <version>4.2.0</version>' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '</package>' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      # colcon 依赖检查需要 package.sh（ament 风格）
      printf '# ament package env for gtsam\n' > ${GTSAM_INSTALL_DIR}/share/gtsam/package.sh
      printf 'export CMAKE_PREFIX_PATH="%s:$CMAKE_PREFIX_PATH"\n' "${GTSAM_INSTALL_DIR}" >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.sh
      printf 'export LD_LIBRARY_PATH="%s/lib:$LD_LIBRARY_PATH"\n' "${GTSAM_INSTALL_DIR}" >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.sh
      echo "[INFO] GTSAM compiled and installed"
      cd /root/automap_ws
  else
      echo "[INFO] GTSAM 已安装于 install_deps，跳过"
  fi

  # 若缺少 package.sh（旧安装或 colcon 依赖检查需要）则补写
  if [ ! -f "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh" ]; then
    mkdir -p "${GTSAM_INSTALL_DIR}/share/gtsam"
    printf '# ament package env for gtsam\n' > "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh"
    printf 'export CMAKE_PREFIX_PATH="%s:$CMAKE_PREFIX_PATH"\n' "${GTSAM_INSTALL_DIR}" >> "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh"
    printf 'export LD_LIBRARY_PATH="%s/lib:$LD_LIBRARY_PATH"\n' "${GTSAM_INSTALL_DIR}" >> "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh"
  fi
  export CMAKE_PREFIX_PATH="${GTSAM_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"

  # 让 colcon 在 workspace install 下找到 gtsam（hba 依赖）
  mkdir -p /root/automap_ws/install
  ln -sfn /root/automap_ws/install_deps/gtsam /root/automap_ws/install/gtsam

  echo '========================================'
  echo '编译 hba (HBA-main)'
  echo '========================================'
  colcon build ${COLCON_PARALLEL} --packages-select hba --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# TEASER++ 源码（安装到 install_deps，已安装则跳过）
TEASER_SRC=""
[ -d /root/mapping/TEASER-plusplus-master ] && TEASER_SRC=/root/mapping/TEASER-plusplus-master
[ -z "${TEASER_SRC}" ] && [ -d /root/mapping/automap_pro/src/modular/TEASER-plusplus-master ] && TEASER_SRC=/root/mapping/automap_pro/src/modular/TEASER-plusplus-master

TEASER_INSTALL_DIR="${INSTALL_DEPS}/teaserpp"
if [ -n "${TEASER_SRC}" ]; then
  if [ -f "${TEASER_INSTALL_DIR}/lib/cmake/teaserpp/teaserppTargets.cmake" ]; then
    echo "[INFO] TEASER++ 已安装于 install_deps，跳过"
  else
    echo '========================================'
    echo '编译 TEASER++'
    echo '========================================'
    TEASER_BUILD=/root/automap_ws/build_teaserpp
    mkdir -p "${TEASER_INSTALL_DIR}"
    mkdir -p "${TEASER_BUILD}" && cd "${TEASER_BUILD}"
    MAP_THRID=/root/mapping/automap_pro/thrid_party
    [ ! -d "${MAP_THRID}/pmc-master" ] && MAP_THRID=/root/mapping/thrid_party
    # 使用本地 thrid_party 的 pmc/tinyply/spectra，避免 FetchContent 访问 GitHub 超时/离线失败
    TEASER_CMAKE_EXTRA=""
    [ -d "${MAP_THRID}/spectra" ] && TEASER_CMAKE_EXTRA="-DFETCHCONTENT_SOURCE_DIR_SPECTRA=${MAP_THRID}/spectra"
    cmake "${TEASER_SRC}" \
        -DCMAKE_INSTALL_PREFIX="${TEASER_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DPMC_SOURCE_DIR="${MAP_THRID}/pmc-master" \
        -DTINYPLY_SOURCE_DIR="${MAP_THRID}/tinyply" \
        -DFETCHCONTENT_SOURCE_DIR_PMC="${MAP_THRID}/pmc-master" \
        -DFETCHCONTENT_SOURCE_DIR_TINYPLY="${MAP_THRID}/tinyply" \
        ${TEASER_CMAKE_EXTRA}
    make -j$(nproc)
    make install
    echo "[INFO] TEASER++ 已安装于 install_deps"
  fi
  cd /root/automap_ws
fi

# vikit_common / vikit_ros（安装到 install_deps，已安装则跳过）
if [ -d src/thrid_party/rpg_vikit_ros2 ]; then
  if [ -f "${INSTALL_DEPS}/share/vikit_common/package.xml" ] || [ -f "${INSTALL_DEPS}/lib/libvikit_common.so" ]; then
    echo "[INFO] vikit 已安装于 install_deps，跳过"
  else
    echo '========================================'
    echo '编译 vikit'
    echo '========================================'
    colcon build ${COLCON_PARALLEL} --install-base "${INSTALL_DEPS}" --paths src/thrid_party/rpg_vikit_ros2/vikit_common src/thrid_party/rpg_vikit_ros2/vikit_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] vikit 已安装于 install_deps"
  fi
fi

# 后续 fast_livo / automap_pro 需要找到 gtsam、teaserpp、vikit
[ -f "${INSTALL_DEPS}/setup.bash" ] && source "${INSTALL_DEPS}/setup.bash"
export CMAKE_PREFIX_PATH="${INSTALL_DEPS}/gtsam:${INSTALL_DEPS}/teaserpp:${INSTALL_DEPS}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${INSTALL_DEPS}/gtsam/lib:${INSTALL_DEPS}/teaserpp/lib:${LD_LIBRARY_PATH}"

# fast_livo：优先用 in-tree 直接路径，避免 colcon 对 src/fast_livo 符号链接解析后当成 automap_pro 子目录而忽略（导致 0 packages finished）
FAST_LIVO_PATH=""
[ -d src/automap_pro/src/modular/fast-livo2-humble ] && FAST_LIVO_PATH="src/automap_pro/src/modular/fast-livo2-humble"
[ -z "${FAST_LIVO_PATH}" ] && [ -d src/fast_livo ] && FAST_LIVO_PATH="src/fast_livo"
echo "[INFO] fast_livo 源码检查: in-tree=$([ -d src/automap_pro/src/modular/fast-livo2-humble ] && echo 存在 || echo 不存在), src/fast_livo=$([ -d src/fast_livo ] && echo 存在 || echo 不存在) → FAST_LIVO_PATH=${FAST_LIVO_PATH:-未设置}"
if [ -n "${FAST_LIVO_PATH}" ]; then
  echo '========================================'
  echo '编译 fast_livo'
  echo '========================================'
  echo "[INFO] 使用路径: ${FAST_LIVO_PATH}（避免 symlink 导致 colcon 0 packages）"
  colcon build ${COLCON_PARALLEL} --paths "${FAST_LIVO_PATH}" --cmake-args -DCMAKE_BUILD_TYPE=Release
  if [ ! -f install/fast_livo/share/fast_livo/package.xml ]; then
    echo "[ERROR] fast_livo 编译后未找到 install/fast_livo，请检查上方 colcon 输出是否有报错"
    exit 1
  fi
  echo "[INFO] fast_livo 已安装于 install/fast_livo"
else
  echo '[WARN] 未找到 fast_livo 源码（src/automap_pro/src/modular/fast-livo2-humble 或 src/fast_livo），跳过；运行时会报 package fast_livo not found'
fi

# automap_pro
echo '========================================'
echo '编译 automap_pro'
echo '========================================'
colcon build ${COLCON_PARALLEL} --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release -DNLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3 --event-handlers console_direct+

echo '========================================'
echo '验证编译产物'
echo '========================================'
if [ ! -f install/automap_pro/share/automap_pro/package.xml ]; then
  echo '✗ automap_pro 未找到安装产物'
  exit 1
fi
echo '✓ automap_pro 已安装'
