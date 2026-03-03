#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# AutoMap-Pro v2.0 一键构建脚本
# ══════════════════════════════════════════════════════════════════════════════
set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${WS_DIR}/src"
THRID_DIR="${SRC_DIR}/thrid_party"

echo "====================================================="
echo " AutoMap-Pro Build System"
echo " Workspace: ${WS_DIR}"
echo "====================================================="

# ── 环境检查 ────────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash 2>/dev/null || { echo "[ERROR] ROS2 Humble not found"; exit 1; }

# ── 选项解析 ────────────────────────────────────────────────────────────────
JOBS=$(nproc)
BUILD_TYPE="Release"
CLEAN=false
SKIP_TEASER=false
WITH_CUDA=true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --clean)    CLEAN=true;        shift ;;
        --debug)    BUILD_TYPE="Debug"; shift ;;
        --jobs)     JOBS="$2";         shift 2 ;;
        --no-cuda)  WITH_CUDA=false;   shift ;;
        --no-teaser) SKIP_TEASER=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo "[Build] Type=${BUILD_TYPE}, Jobs=${JOBS}, CUDA=${WITH_CUDA}"

# ── 清理（可选）────────────────────────────────────────────────────────────
if [ "$CLEAN" = true ]; then
    echo "[Build] Cleaning build/install/log..."
    rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
fi

# ── 第三方库准备 ─────────────────────────────────────────────────────────────
mkdir -p "${THRID_DIR}"

# TEASER++（源码级编译，add_subdirectory 方式）
if [ ! -d "${THRID_DIR}/TEASER-plusplus/CMakeLists.txt" ] && [ "$SKIP_TEASER" = false ]; then
    TEASER_SRC="/home/wqs/Documents/github/mapping/TEASER-plusplus-master"
    if [ -d "${TEASER_SRC}" ]; then
        echo "[Build] Linking TEASER++ source..."
        ln -sfn "${TEASER_SRC}" "${THRID_DIR}/TEASER-plusplus"
    else
        echo "[Build] Downloading TEASER++..."
        git clone --depth=1 https://github.com/MIT-SPARK/TEASER-plusplus.git \
            "${THRID_DIR}/TEASER-plusplus"
    fi
fi

# nlohmann/json
if [ ! -f "${THRID_DIR}/nlohmann-json3/CMakeLists.txt" ]; then
    JSON_SRC="/usr/include/nlohmann"
    if [ ! -d "${JSON_SRC}" ]; then
        echo "[Build] Downloading nlohmann-json..."
        git clone --depth=1 --branch v3.11.2 \
            https://github.com/nlohmann/json.git "${THRID_DIR}/nlohmann-json3"
    fi
fi

# ── 符号链接验证 ─────────────────────────────────────────────────────────────
echo "[Build] Verifying source symlinks..."
declare -A LINKS=(
    ["automap_pro"]="/home/wqs/Documents/github/automap_pro/automap_pro"
    ["fast_livo"]="/home/wqs/Documents/github/mapping/fast-livo2-humble"
    ["hba"]="/home/wqs/Documents/github/mapping/HBA-main/HBA_ROS2"
    ["overlap_transformer_msgs"]="/home/wqs/Documents/github/automap_pro/overlap_transformer_msgs"
)

for name in "${!LINKS[@]}"; do
    target="${LINKS[$name]}"
    link="${SRC_DIR}/${name}"
    if [ ! -e "$link" ] && [ -e "$target" ]; then
        ln -sfn "$target" "$link"
        echo "  Created link: ${link} → ${target}"
    elif [ -e "$link" ]; then
        echo "  OK: ${link}"
    else
        echo "  WARNING: ${target} not found, skipping ${name}"
    fi
done

# hba_api（新包）
if [ ! -e "${SRC_DIR}/hba_api" ]; then
    ln -sfn "${WS_DIR}/src/hba_api" "${SRC_DIR}/hba_api" 2>/dev/null || true
fi

# ── CMAKE 参数 ────────────────────────────────────────────────────────────────
CMAKE_ARGS=(
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    "-DBUILD_TEASER_TESTS=OFF"
    "-DBUILD_TEASER_DEMOS=OFF"
    "-DBUILD_PYTHON_BINDINGS=OFF"
    "-DGTSAM_BUILD_TESTS=OFF"
    "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
    "-DGTSAM_USE_SYSTEM_EIGEN=ON"
    "-DCMAKE_CXX_FLAGS=-march=native"
)

if [ "$WITH_CUDA" = true ] && command -v nvcc &>/dev/null; then
    CMAKE_ARGS+=("-DUSE_CUDA=ON")
    echo "[Build] CUDA enabled"
fi

# ── LibTorch 配置（如果已安装）───────────────────────────────────────────────
TORCH_CANDIDATES=(
    "/opt/libtorch"
    "/usr/local/libtorch"
    "${HOME}/libtorch"
    "/workspace/libtorch"
)
for TORCH_DIR in "${TORCH_CANDIDATES[@]}"; do
    if [ -d "${TORCH_DIR}/share/cmake/Torch" ]; then
        CMAKE_ARGS+=("-DTorch_DIR=${TORCH_DIR}/share/cmake/Torch")
        echo "[Build] LibTorch found at ${TORCH_DIR}"
        break
    fi
done

# ── 构建 ─────────────────────────────────────────────────────────────────────
echo "[Build] Running colcon build..."
cd "${WS_DIR}"

colcon build \
    --symlink-install \
    --cmake-args "${CMAKE_ARGS[@]}" \
    --parallel-workers "${JOBS}" \
    --event-handlers console_cohesion+ \
    --packages-select \
        overlap_transformer_msgs \
        hba_api \
        hba \
        fast_livo \
        automap_pro \
    2>&1 | tee "${WS_DIR}/build.log"

BUILD_STATUS=${PIPESTATUS[0]}

echo "====================================================="
if [ $BUILD_STATUS -eq 0 ]; then
    echo " [SUCCESS] Build completed!"
    echo ""
    echo " Source environment with:"
    echo "   source ${WS_DIR}/install/setup.bash"
    echo ""
    echo " Quick start:"
    echo "   ros2 launch automap_pro automap_composable.launch.py"
    echo "   ros2 launch automap_pro automap_composable.launch.py composable:=false"
    echo ""
    echo " Model preparation (OverlapTransformer):"
    echo "   cd /path/to/OverlapTransformer-master/OT_libtorch"
    echo "   python gen_libtorch_model.py"
    echo "   # Then update config: loop_closure.overlap_transformer.model_path"
else
    echo " [FAILED] Build failed! See ${WS_DIR}/build.log"
fi
echo "====================================================="
exit $BUILD_STATUS
