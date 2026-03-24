#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd /root/automap_ws

# 充分发挥宿主机性能：优先使用宿主机传入的核数（AUTOMAP_BUILD_JOBS 或 HOST_NPROC），否则容器内 nproc
PARALLEL_JOBS="${AUTOMAP_BUILD_JOBS:-${HOST_NPROC:-$(nproc)}}"
export CMAKE_BUILD_PARALLEL_LEVEL="${PARALLEL_JOBS}"
COLCON_PARALLEL="--parallel-workers ${PARALLEL_JOBS}"
echo "[INFO] 并行编译: 每包 ${PARALLEL_JOBS} 线程, colcon --parallel-workers ${PARALLEL_JOBS}"

# 使用 Ninja 生成器可显著加速编译（镜像已装 ninja-build）
NINJA_CMAKE_ARG=""
if command -v ninja >/dev/null 2>&1; then
  NINJA_CMAKE_ARG="-G Ninja"
  echo "[INFO] 使用 Ninja 生成器以加速编译"
fi

# 第三方库（GTSAM/TEASER++/vikit）安装到 install_deps，--clean 时不删除，编译一次后续跳过
INSTALL_DEPS="/root/automap_ws/install_deps"
mkdir -p "${INSTALL_DEPS}"

# 若 CMake 缓存为其他路径创建，清理（不删 install_deps）
if [ -d build ] && find build -name CMakeCache.txt -exec grep -l '/workspace/' {} \; 2>/dev/null | grep -q .; then
  echo '[WARN] 检测到 CMake 缓存路径不一致，清理 build/install/log'
  rm -rf build install log build_teaserpp build_sophus build_ceres
fi

# 检测 automap_pro/CMakeLists.txt 是否比 build 目录更新，如果是则清理缓存
if [ -d build/automap_pro ]; then
  CMAKE_CACHE_TIME=$(find build/automap_pro -name CMakeCache.txt -type f 2>/dev/null | head -1 | xargs stat -c %Y 2>/dev/null || echo 0)
  CMAKE_FILE_TIME=$(find automap_pro/CMakeLists.txt -type f 2>/dev/null | head -1 | xargs stat -c %Y 2>/dev/null || echo 0)
  if [ "${CMAKE_FILE_TIME}" -gt "${CMAKE_CACHE_TIME}" ]; then
    echo '[INFO] CMakeLists.txt 已更新，清理 automap_pro 编译缓存'
    rm -rf build/automap_pro
  fi
fi

# 链入 overlap_transformer_msgs / overlap_transformer_ros2 / hba（若存在）
[ -d /root/mapping/overlap_transformer_msgs ] && ln -sf /root/mapping/overlap_transformer_msgs src/ 2>/dev/null || true
[ -d /root/mapping/overlap_transformer_ros2 ] && ln -sf /root/mapping/overlap_transformer_ros2 src/ 2>/dev/null || true
[ -d /root/mapping/HBA-main/HBA_ROS2 ]        && ln -sf /root/mapping/HBA-main/HBA_ROS2 src/hba 2>/dev/null || true

# ScanContext 在 automap_pro 编译时通过 -DSCANCONTEXT_ROOT 参数传入

if [ -d src/overlap_transformer_msgs ]; then
  echo '========================================'
  echo '编译 overlap_transformer_msgs'
  echo '========================================'
  colcon build ${COLCON_PARALLEL} --packages-select overlap_transformer_msgs --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
fi

if [ -d src/overlap_transformer_ros2 ]; then
  echo '========================================'
  echo '编译 overlap_transformer_ros2'
  echo '========================================'
  colcon build ${COLCON_PARALLEL} --packages-select overlap_transformer_ros2 --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
fi

HBA_INSTALL_DIR="${INSTALL_DEPS}/hba"
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
          ${NINJA_CMAKE_ARG} \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=${GTSAM_INSTALL_DIR} \
          -DGTSAM_WITH_TBB=OFF \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DBUILD_SHARED_LIBS=ON \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_PYTHON=OFF
      if [ -n "${NINJA_CMAKE_ARG}" ]; then ninja -j${PARALLEL_JOBS}; ninja install; else make -j${PARALLEL_JOBS}; make install; fi
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

  # hba：已安装则跳过（colcon 产出 setup.bash，--clean 不删 install_deps）
  NEED_HBA_BUILD=true
  if [ -f "${HBA_INSTALL_DIR}/setup.bash" ] || [ -f "${HBA_INSTALL_DIR}/lib/libhba_core.so" ] || [ -d "${HBA_INSTALL_DIR}/lib" ]; then
    NEED_HBA_BUILD=false
  fi
  if [ "$NEED_HBA_BUILD" = true ]; then
    echo '========================================'
    echo '编译 hba (HBA-main)（安装到 install_deps/hba）'
    echo '========================================'
    mkdir -p "${HBA_INSTALL_DIR}"
    # colcon 在 --install-base 下查找依赖，需在 hba 目录内提供 gtsam 符号链接
    ln -sfn ../gtsam "${HBA_INSTALL_DIR}/gtsam" 2>/dev/null || true
    colcon build ${COLCON_PARALLEL} --install-base "${HBA_INSTALL_DIR}" --packages-select hba --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] hba 已安装于 install_deps/hba"
  else
    echo "[INFO] hba 已安装于 install_deps，跳过"
  fi
  [ -d "${HBA_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${HBA_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${HBA_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${HBA_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
  ln -sfn "${HBA_INSTALL_DIR}" /root/automap_ws/install/hba 2>/dev/null || true
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
        ${NINJA_CMAKE_ARG} \
        -DCMAKE_INSTALL_PREFIX="${TEASER_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DPMC_SOURCE_DIR="${MAP_THRID}/pmc-master" \
        -DTINYPLY_SOURCE_DIR="${MAP_THRID}/tinyply" \
        -DFETCHCONTENT_SOURCE_DIR_PMC="${MAP_THRID}/pmc-master" \
        -DFETCHCONTENT_SOURCE_DIR_TINYPLY="${MAP_THRID}/tinyply" \
        ${TEASER_CMAKE_EXTRA}
    if [ -n "${NINJA_CMAKE_ARG}" ]; then ninja -j${PARALLEL_JOBS}; ninja install; else make -j${PARALLEL_JOBS}; make install; fi
    echo "[INFO] TEASER++ 已安装于 install_deps"
  fi
  cd /root/automap_ws
fi

# Ceres Solver（与 GTSAM 一致：安装到 install_deps/ceres，首次编译后后续跳过）
CERES_INSTALL_DIR="${INSTALL_DEPS}/ceres"
CERES_SRC="/root/automap_ws/src/automap_pro/thrid_party/ceres-solver"
if [ -d "${CERES_SRC}" ] && [ -f "${CERES_SRC}/CMakeLists.txt" ]; then
  NEED_CERES_BUILD=false
  if [ ! -f "${CERES_INSTALL_DIR}/lib/libceres.so" ] && [ ! -f "${CERES_INSTALL_DIR}/lib/cmake/Ceres/CeresConfig.cmake" ]; then
    NEED_CERES_BUILD=true
  fi
  if [ "$NEED_CERES_BUILD" = true ]; then
    echo '========================================'
    echo '编译 Ceres Solver（安装到 install_deps/ceres）'
    echo '========================================'
    CERES_BUILD=/root/automap_ws/build_ceres
    rm -rf "${CERES_BUILD}"
    mkdir -p "${CERES_BUILD}" && cd "${CERES_BUILD}"
    cmake "${CERES_SRC}" \
        ${NINJA_CMAKE_ARG} \
        -DCMAKE_INSTALL_PREFIX="${CERES_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_DOCUMENTATION=OFF
    if [ -n "${NINJA_CMAKE_ARG}" ]; then ninja -j${PARALLEL_JOBS}; ninja install; else make -j${PARALLEL_JOBS}; make install; fi
    echo "[INFO] Ceres Solver 已安装于 install_deps/ceres"
    cd /root/automap_ws
  else
    echo "[INFO] Ceres Solver 已安装于 install_deps，跳过"
  fi
  [ -d "${CERES_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${CERES_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${CERES_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${CERES_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
fi

# vikit_common / vikit_ros（与 GTSAM 一致：安装到 install_deps/vikit 子目录，只编译一次，已安装则跳过）
VIKIT_INSTALL_DIR="${INSTALL_DEPS}/vikit"
VIKIT_SRC=""
[ -d src/thrid_party/rpg_vikit_ros2 ] && VIKIT_SRC="src/thrid_party/rpg_vikit_ros2"
[ -z "${VIKIT_SRC}" ] && [ -d src/automap_pro/thrid_party/rpg_vikit_ros2 ] && VIKIT_SRC="src/automap_pro/thrid_party/rpg_vikit_ros2"
if [ -n "${VIKIT_SRC}" ]; then
  NEED_VIKIT_BUILD=true
  # 已安装则跳过（colcon 产出 setup.bash，--clean 不删 install_deps）
  if [ -f "${VIKIT_INSTALL_DIR}/setup.bash" ] || [ -f "${VIKIT_INSTALL_DIR}/lib/libvikit_common.so" ] || [ -d "${VIKIT_INSTALL_DIR}/lib" ]; then
    NEED_VIKIT_BUILD=false
  elif [ -f "${INSTALL_DEPS}/lib/libvikit_common.so" ]; then
    NEED_VIKIT_BUILD=false
    VIKIT_INSTALL_DIR="${INSTALL_DEPS}"
  fi
  if [ "$NEED_VIKIT_BUILD" = true ]; then
    echo '========================================'
    echo '编译 vikit（安装到 install_deps/vikit，与 GTSAM 一致）'
    echo '========================================'
    mkdir -p "${VIKIT_INSTALL_DIR}"
    colcon build ${COLCON_PARALLEL} --install-base "${VIKIT_INSTALL_DIR}" --paths "${VIKIT_SRC}/vikit_common" "${VIKIT_SRC}/vikit_ros" --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] vikit 已安装于 install_deps/vikit"
  else
    echo "[INFO] vikit 已安装于 install_deps，跳过"
  fi
  # 让 colcon/ament 找到 vikit（与 GTSAM 一致）
  [ -d "${VIKIT_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${VIKIT_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${VIKIT_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${VIKIT_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
  # 独立 vikit 目录时创建符号链接，便于 overlay
  if [ "${VIKIT_INSTALL_DIR}" != "${INSTALL_DEPS}" ]; then
    mkdir -p /root/automap_ws/install
    ln -sfn "${VIKIT_INSTALL_DIR}" /root/automap_ws/install/vikit 2>/dev/null || true
  fi
fi

# LibTorch（OverlapTransformer 推理）：与 GTSAM 一致，安装到 install_deps，仅下载解压一次，不随工程重复编译
LIBTORCH_INSTALL_DIR="${INSTALL_DEPS}/libtorch"
NEED_LIBTORCH_DOWNLOAD=false
if [ ! -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] && [ ! -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; then
  NEED_LIBTORCH_DOWNLOAD=true
fi
if [ "${LIBTORCH_SKIP_DOWNLOAD:-0}" = "1" ]; then
  NEED_LIBTORCH_DOWNLOAD=false
  echo "[INFO] LIBTORCH_SKIP_DOWNLOAD=1，跳过 LibTorch 下载（请确保已手动放置到 ${LIBTORCH_INSTALL_DIR}）"
fi
if [ "$NEED_LIBTORCH_DOWNLOAD" = true ]; then
  echo '========================================'
  echo '安装 LibTorch 到 install_deps（下载预编译包，仅执行一次）'
  echo '========================================'
  LIBTORCH_URL="${LIBTORCH_URL:-https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.2%2Bcpu.zip}"
  LIBTORCH_ZIP="/tmp/libtorch-cpu.zip"
  mkdir -p "${INSTALL_DEPS}"
  if [ -d "${LIBTORCH_INSTALL_DIR}" ]; then
    rm -rf "${LIBTORCH_INSTALL_DIR}"
  fi
  echo "[INFO] 下载 LibTorch: ${LIBTORCH_URL}"
  if command -v wget >/dev/null 2>&1; then
    wget -q --show-progress -O "${LIBTORCH_ZIP}" "${LIBTORCH_URL}" || { echo "[ERROR] wget LibTorch 失败"; rm -f "${LIBTORCH_ZIP}"; exit 1; }
  elif command -v curl >/dev/null 2>&1; then
    curl -# -L -o "${LIBTORCH_ZIP}" "${LIBTORCH_URL}" || { echo "[ERROR] curl LibTorch 失败"; rm -f "${LIBTORCH_ZIP}"; exit 1; }
  else
    echo "[ERROR] 需要 wget 或 curl 下载 LibTorch"; exit 1
  fi
  echo "[INFO] 解压到 ${LIBTORCH_INSTALL_DIR}"
  unzip -q -o "${LIBTORCH_ZIP}" -d "${INSTALL_DEPS}"
  rm -f "${LIBTORCH_ZIP}"
  if [ -d "${INSTALL_DEPS}/libtorch" ]; then
    echo "[INFO] LibTorch 已安装于 install_deps/libtorch"
  else
    echo "[ERROR] 解压后未找到 ${INSTALL_DEPS}/libtorch"; exit 1
  fi
else
  if [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] || [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; then
    echo "[INFO] LibTorch 已安装于 install_deps，跳过"
  fi
fi
if [ -d "${LIBTORCH_INSTALL_DIR}/lib" ]; then
  export LIBTORCH_HOME="${LIBTORCH_INSTALL_DIR}"
  export CMAKE_PREFIX_PATH="${LIBTORCH_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${LIBTORCH_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
fi

# ONNX Runtime（SLOAM 语义分割）：与 GTSAM 一致，安装到 install_deps，首次编译后后续跳过
ONNXRUNTIME_INSTALL_DIR="${INSTALL_DEPS}/onnxruntime"
NEED_ONNXRUNTIME_BUILD=false
if [ ! -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so" ]; then
  NEED_ONNXRUNTIME_BUILD=true
fi
if [ "${ONNXRUNTIME_SKIP_BUILD:-0}" = "1" ]; then
  NEED_ONNXRUNTIME_BUILD=false
  echo "[INFO] ONNXRUNTIME_SKIP_BUILD=1，跳过 ONNX Runtime 编译（未安装时将使用 SLOAM stub 模式）"
fi
if [ "$NEED_ONNXRUNTIME_BUILD" = true ]; then
  echo '========================================'
  echo '编译 ONNX Runtime 到 install_deps（首次执行，后续跳过）'
  echo '========================================'
  ONNX_SRC_DIR="/root/automap_ws/src_onnxruntime"
  ONNX_DEPS_MOUNTED="/root/mapping/docker/deps/onnxruntime"
  mkdir -p "${INSTALL_DEPS}"
  if [ ! -d "${ONNX_SRC_DIR}" ] || [ ! -f "${ONNX_SRC_DIR}/build.sh" ]; then
    if [ -d "${ONNX_DEPS_MOUNTED}" ] && [ -f "${ONNX_DEPS_MOUNTED}/build.sh" ]; then
      echo "[INFO] 使用挂载的 ONNX Runtime 源码: ${ONNX_DEPS_MOUNTED}（无需 git clone）"
      rm -rf "${ONNX_SRC_DIR}"
      cp -a "${ONNX_DEPS_MOUNTED}" "${ONNX_SRC_DIR}"
    else
      echo "[INFO] 未找到 docker/deps/onnxruntime，克隆 ONNX Runtime v1.8.2（含 submodule）..."
      git clone --depth 1 --branch v1.8.2 --recursive https://github.com/Microsoft/onnxruntime "${ONNX_SRC_DIR}" || { echo "[ERROR] git clone onnxruntime 失败"; exit 1; }
    fi
  fi
  cd "${ONNX_SRC_DIR}"
  ./build.sh \
    --config RelWithDebInfo \
    --build_shared_lib \
    --skip_tests \
    --parallel "${PARALLEL_JOBS}" \
    --cmake_extra_defines "CMAKE_INSTALL_PREFIX=${ONNXRUNTIME_INSTALL_DIR}"
  cd build/Linux/RelWithDebInfo
  cmake --build . --target install
  cd /root/automap_ws
  echo "[INFO] ONNX Runtime 已安装于 install_deps/onnxruntime"
else
  if [ -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so" ]; then
    echo "[INFO] ONNX Runtime 已安装于 install_deps，跳过"
  fi
fi
if [ -d "${ONNXRUNTIME_INSTALL_DIR}/lib" ]; then
  export ONNXRUNTIME_HOME="${ONNXRUNTIME_INSTALL_DIR}"
  export LD_LIBRARY_PATH="${ONNXRUNTIME_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
fi

# 后续 fast_livo / automap_pro 需要找到 gtsam、ceres、teaserpp、vikit、hba、fast_livo、libtorch、onnxruntime
[ -f "${INSTALL_DEPS}/setup.bash" ] && source "${INSTALL_DEPS}/setup.bash"
# vikit / hba / fast_livo 安装到 install_deps 子目录（与 GTSAM 一致），需显式加入路径
_VIKIT_PREFIX=""
_HBA_PREFIX=""
_FAST_LIVO_PREFIX=""
_CERES_PREFIX=""
[ -d "${VIKIT_INSTALL_DIR}/lib" ] 2>/dev/null && _VIKIT_PREFIX="${VIKIT_INSTALL_DIR}"
[ -d "${HBA_INSTALL_DIR}/lib" ] 2>/dev/null && _HBA_PREFIX="${HBA_INSTALL_DIR}"
[ -d "${INSTALL_DEPS}/fast_livo/lib" ] 2>/dev/null && _FAST_LIVO_PREFIX="${INSTALL_DEPS}/fast_livo"
[ -d "${INSTALL_DEPS}/ceres/lib" ] 2>/dev/null && _CERES_PREFIX="${INSTALL_DEPS}/ceres"
export CMAKE_PREFIX_PATH="${INSTALL_DEPS}/gtsam:${_CERES_PREFIX:+${_CERES_PREFIX}:}${INSTALL_DEPS}/teaserpp:${_VIKIT_PREFIX:+${_VIKIT_PREFIX}:}${_HBA_PREFIX:+${_HBA_PREFIX}:}${_FAST_LIVO_PREFIX:+${_FAST_LIVO_PREFIX}:}${INSTALL_DEPS}/libtorch:${INSTALL_DEPS}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${INSTALL_DEPS}/gtsam/lib:${_CERES_PREFIX:+${_CERES_PREFIX}/lib:}${INSTALL_DEPS}/teaserpp/lib:${_VIKIT_PREFIX:+${_VIKIT_PREFIX}/lib:}${_HBA_PREFIX:+${_HBA_PREFIX}/lib:}${_FAST_LIVO_PREFIX:+${_FAST_LIVO_PREFIX}/lib:}${INSTALL_DEPS}/libtorch/lib:${INSTALL_DEPS}/onnxruntime/lib:${LD_LIBRARY_PATH}"

# fast_livo：与 GTSAM/vikit/hba 一致，安装到 install_deps/fast_livo，已安装则跳过
FAST_LIVO_INSTALL_DIR="${INSTALL_DEPS}/fast_livo"
FAST_LIVO_PATH=""
[ -d src/automap_pro/src/modular/fast-livo2-humble ] && FAST_LIVO_PATH="src/automap_pro/src/modular/fast-livo2-humble"
[ -z "${FAST_LIVO_PATH}" ] && [ -d src/fast_livo ] && FAST_LIVO_PATH="src/fast_livo"
echo "[INFO] fast_livo 源码检查: in-tree=$([ -d src/automap_pro/src/modular/fast-livo2-humble ] && echo 存在 || echo 不存在), src/fast_livo=$([ -d src/fast_livo ] && echo 存在 || echo 不存在) → FAST_LIVO_PATH=${FAST_LIVO_PATH:-未设置}"
if [ -n "${FAST_LIVO_PATH}" ]; then
  NEED_FAST_LIVO_BUILD=true
  # 已安装则跳过（colcon 产出 setup.bash，--clean 不删 install_deps）
  if [ -f "${FAST_LIVO_INSTALL_DIR}/setup.bash" ] || [ -d "${FAST_LIVO_INSTALL_DIR}/fast_livo" ] || [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] || [ -d "${FAST_LIVO_INSTALL_DIR}/share" ]; then
    NEED_FAST_LIVO_BUILD=false
  fi
  if [ "$NEED_FAST_LIVO_BUILD" = true ]; then
    echo '========================================'
    echo '编译 fast_livo（安装到 install_deps/fast_livo）'
    echo '========================================'
    echo "[INFO] 使用路径: ${FAST_LIVO_PATH}（避免 symlink 导致 colcon 0 packages）"
    mkdir -p "${FAST_LIVO_INSTALL_DIR}"
    colcon build ${COLCON_PARALLEL} --install-base "${FAST_LIVO_INSTALL_DIR}" --paths "${FAST_LIVO_PATH}" --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
    _fl_ok=false
    [ -f "${FAST_LIVO_INSTALL_DIR}/setup.bash" ] && _fl_ok=true
    [ -f "${FAST_LIVO_INSTALL_DIR}/share/fast_livo/package.xml" ] && _fl_ok=true
    [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] && _fl_ok=true
    [ -d "${FAST_LIVO_INSTALL_DIR}/share" ] && _fl_ok=true
    [ -d "${FAST_LIVO_INSTALL_DIR}/fast_livo" ] && _fl_ok=true
    if [ "$_fl_ok" = false ]; then
      echo "[WARN] fast_livo 编译完成但未在预期路径找到产物，检查: ${FAST_LIVO_INSTALL_DIR}"
      ls -la "${FAST_LIVO_INSTALL_DIR}" 2>/dev/null || true
    fi
    echo "[INFO] fast_livo 已安装于 install_deps/fast_livo"
  else
    echo "[INFO] fast_livo 已安装于 install_deps，跳过"
  fi
  [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${FAST_LIVO_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${FAST_LIVO_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
  mkdir -p /root/automap_ws/install
  ln -sfn "${FAST_LIVO_INSTALL_DIR}" /root/automap_ws/install/fast_livo 2>/dev/null || true
else
  echo '[WARN] 未找到 fast_livo 源码（src/automap_pro/src/modular/fast-livo2-humble 或 src/fast_livo），跳过；运行时会报 package fast_livo not found'
fi

# automap_pro

# ============================================================
# ScanContext 详细定位日志
# ============================================================
echo "========================================"
echo "ScanContext 定位调试"
echo "========================================"

# 检查可能的源码位置
echo "[DEBUG-1] 检查 /root/automap_ws/src/ 目录结构:"
ls -la /root/automap_ws/src/ 2>&1 || echo "[DEBUG-1] /root/automap_ws/src/ 不存在"

echo ""
echo "[DEBUG-2] 检查 /root/mapping/ 目录结构 (如果存在):"
ls -la /root/mapping/ 2>&1 || echo "[DEBUG-2] /root/mapping/ 不存在"

echo ""
echo "[DEBUG-3] 在 /root/mapping 下搜索 scancontext_tro:"
find /root/mapping -maxdepth 3 -type d -name "scancontext*" 2>/dev/null || echo "[DEBUG-3] 未找到"

echo ""
echo "[DEBUG-4] 在 /root/automap_ws 下搜索 scancontext_tro:"
find /root/automap_ws -maxdepth 4 -type d -name "scancontext*" 2>/dev/null || echo "[DEBUG-4] 未找到"

echo ""
echo "[DEBUG-5] 检查 automap_pro 源码目录:"
if [ -d "/root/automap_ws/src/automap_pro" ]; then
    echo "[DEBUG-5] /root/automap_ws/src/automap_pro 存在"
    ls -la /root/automap_ws/src/automap_pro/ | head -20
    echo ""
    echo "[DEBUG-5b] 在 automap_pro 下搜索 scancontext:"
    find /root/automap_ws/src/automap_pro -maxdepth 5 -type d -name "scancontext*" 2>/dev/null || echo "未找到"
else
    echo "[DEBUG-5] /root/automap_ws/src/automap_pro 不存在"
fi

# 修复 ScanContext 头文件路径：在可写目录创建符号链接
# 因为容器内 automap_pro 是只读挂载，需要在 /root/automap_ws 下创建链接
SC_SOURCE=""
SC_TARGET="/root/automap_ws/automap_pro_thrid_party_scancontext"

# 尝试多个可能的源码位置
echo ""
echo "[DEBUG-6] 尝试确定 SC_SOURCE:"
if [ -d "/root/automap_ws/src/automap_pro/thrid_party/scancontext_tro" ]; then
    SC_SOURCE="/root/automap_ws/src/automap_pro/thrid_party/scancontext_tro"
    echo "[DEBUG-6a] 使用 SC_SOURCE=/root/automap_ws/src/automap_pro/thrid_party/scancontext_tro"
elif [ -d "/root/automap_ws/src/thrid_party/scancontext_tro" ]; then
    SC_SOURCE="/root/automap_ws/src/thrid_party/scancontext_tro"
    echo "[DEBUG-6b] 使用 SC_SOURCE=/root/automap_ws/src/thrid_party/scancontext_tro"
elif [ -d "/root/mapping/scancontext_tro" ]; then
    SC_SOURCE="/root/mapping/scancontext_tro"
    echo "[DEBUG-6c] 使用 SC_SOURCE=/root/mapping/scancontext_tro"
else
    # 搜索所有可能的 scancontext 目录
    SC_SOURCE=$(find /root/automap_ws /root/mapping -maxdepth 4 -type d -name "scancontext_tro" 2>/dev/null | head -1)
    if [ -n "${SC_SOURCE}" ]; then
        echo "[DEBUG-6d] 使用自动搜索 SC_SOURCE=${SC_SOURCE}"
    else
        echo "[DEBUG-6e] 无法找到 scancontext_tro 目录!"
    fi
fi

echo ""
echo "[DEBUG-7] SC_SOURCE=${SC_SOURCE}, SC_TARGET=${SC_TARGET}"

if [ -n "${SC_SOURCE}" ] && [ -d "${SC_SOURCE}" ]; then
  echo "[DEBUG-7a] SC_SOURCE 目录存在，验证内部结构:"
  ls -la "${SC_SOURCE}/" 2>&1 | head -20
  echo ""
  echo "[DEBUG-7b] 搜索 Scancontext.h:"
  find "${SC_SOURCE}" -name "Scancontext.h" 2>/dev/null

  if [ ! -L "${SC_TARGET}" ] && [ ! -d "${SC_TARGET}" ]; then
    echo ""
    echo "[INFO] 创建 scancontext_tro 符号链接: ${SC_TARGET} -> ${SC_SOURCE}"
    ln -sfn "${SC_SOURCE}" "${SC_TARGET}"
  elif [ -L "${SC_TARGET}" ]; then
    echo "[INFO] scancontext_tro 符号链接已存在: ${SC_TARGET}"
  fi
  # 验证链接是否正确
  if [ -d "${SC_TARGET}/cpp/module/Scancontext" ]; then
    echo "[OK] ScanContext 路径验证成功: ${SC_TARGET}/cpp/module/Scancontext"
    echo "[OK] 列出 Scancontext 目录内容:"
    ls -la "${SC_TARGET}/cpp/module/Scancontext/" 2>&1
  else
    echo "[ERROR] ScanContext 路径验证失败!"
    echo "[ERROR] 检查 ${SC_TARGET} 内容:"
    ls -la "${SC_TARGET}/" 2>&1 || true
    echo ""
    echo "[ERROR] 搜索 Scancontext.h 位置:"
    find "${SC_TARGET}" -name "Scancontext.h" 2>/dev/null || echo "未找到"
  fi
else
  echo "[ERROR] ScanContext 源目录不存在: ${SC_SOURCE}"
fi

echo '========================================'
echo '编译 automap_pro'
echo '========================================'
echo "[INFO] SCANCONTEXT_ROOT=/root/automap_ws/automap_pro_thrid_party_scancontext"
echo "[INFO] NLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3"

# 清理 automap_pro 的 CMake 缓存以确保使用新的 SCANCONTEXT_ROOT 参数
if [ -d build/automap_pro ]; then
  echo "[INFO] 清理 automap_pro CMake 缓存"
  rm -rf build/automap_pro
fi

# 编译前再次确认 scancontext 链接和目录
echo ""
echo "[INFO] 编译前确认 ScanContext 状态:"
if [ -L "/root/automap_ws/automap_pro_thrid_party_scancontext" ]; then
    echo "[INFO] 符号链接存在:"
    ls -la /root/automap_ws/automap_pro_thrid_party_scancontext
    echo ""
    echo "[INFO] 链接目标内容:"
    ls -la /root/automap_ws/automap_pro_thrid_party_scancontext/cpp/module/Scancontext/ 2>&1 || echo "目录不存在"
elif [ -d "/root/automap_ws/automap_pro_thrid_party_scancontext" ]; then
    echo "[INFO] 目录存在:"
    ls -la /root/automap_ws/automap_pro_thrid_party_scancontext/cpp/module/Scancontext/ 2>&1 || echo "目录不存在"
else
    echo "[ERROR] scancontext 链接/目录不存在!"
fi

echo ""
echo "[INFO] 开始编译 automap_pro..."

colcon build ${COLCON_PARALLEL} --packages-select automap_pro --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release -DNLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3 -DSCANCONTEXT_ROOT=/root/automap_ws/automap_pro_thrid_party_scancontext --event-handlers console_direct+ 2>&1 | tee /tmp/automap_build.log

BUILD_EXIT_CODE=${PIPESTATUS[0]}
echo ""
echo "[INFO] 编译命令退出码: ${BUILD_EXIT_CODE}"

if [ ${BUILD_EXIT_CODE} -ne 0 ]; then
    echo ""
    echo "========================================"
    echo "编译失败，分析错误原因"
    echo "========================================"
    echo ""
    echo "[ERROR-1] 检查 CMake 配置阶段是否检测到 ScanContext:"
    grep -i "scancontext" /tmp/automap_build.log | head -20 || echo "未找到 ScanContext 相关日志"
    echo ""
    echo "[ERROR-2] 检查 CMake 生成的 include directories:"
    grep -i "include.*dir" /tmp/automap_build.log | head -20 || echo "未找到 include dir 日志"
    echo ""
    echo "[ERROR-3] 检查编译错误:"
    grep -i "error:" /tmp/automap_build.log | head -10 || echo "未找到错误日志"
    echo ""
    echo "[ERROR-4] 检查 loop_detector 编译详情:"
    grep -A5 "loop_detector" /tmp/automap_build.log | head -20 || echo "未找到 loop_detector 日志"
fi

echo '========================================'
echo '验证编译产物'
echo '========================================'
if [ ! -f install/automap_pro/share/automap_pro/package.xml ]; then
  echo '✗ automap_pro 未找到安装产物'
  exit 1
fi
echo '✓ automap_pro 已安装'
