#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# AutoMap-Pro v2.0  一键编译 & 运行脚本
#
# 特性：
#   - Docker 镜像：默认 NGC Isaac ROS（Jazzy，适配 RTX 50 等）；可设 AUTOMAP_DOCKER_IMAGE=automap-env:humble
#   - 工作空间内无符号链接（源码直接复制进容器）
#   - 全局 spdlog 日志（文件 + 彩色 stdout）
#   - 离线模式自动回放 nya_02_ros2
#   - 一键 build/clean/run 全流程
#
# 用法：
#   bash automap_start.sh [OPTIONS]
#
# OPTIONS:
#   --build        仅编译
#   --run          仅运行（须先编译）
#   --clean        清理编译产物后重新编译
#   --no-rviz      不启动 RViz2
#   --debug        Debug 模式编译（无优化）
#   --bag <path>   指定 rosbag 路径（默认 nya_02_ros2）
#   --help         帮助
#
# 示例：
#   bash automap_start.sh                  # 一键编译+运行
#   bash automap_start.sh --clean --build  # 清理后重新编译
#   bash automap_start.sh --run --no-rviz  # 仅运行
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

# ─────────────────────────────────────────────────────────────────────────────
# 路径与全局变量
# ─────────────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/automap_docker_defaults.sh
source "${SCRIPT_DIR}/scripts/automap_docker_defaults.sh"

# 容器内路径
CONTAINER_WS="/workspace/automap_ws"
CONTAINER_DATA="/workspace/data"
CONTAINER_LOGS="/workspace/logs"
CONTAINER_OUTPUT="/workspace/output"

# 宿主机路径
HOST_WS="${SCRIPT_DIR}/automap_ws"
HOST_DATA="${SCRIPT_DIR}/data"
HOST_LOGS="${SCRIPT_DIR}/logs/automap_$(date +%Y%m%d_%H%M%S)"
HOST_OUTPUT="${SCRIPT_DIR}/output"

# 源码路径（宿主机）
SRC_AUTOMAP_PRO="${SCRIPT_DIR}/automap_pro"
# fast_livo：工程中为 fast-livo2-humble（带连字符），兼容 fast_livo2_humble 命名
if [[ -d "${SCRIPT_DIR}/automap_pro/src/modular/fast-livo2-humble" ]]; then
    SRC_FAST_LIVO="${SCRIPT_DIR}/automap_pro/src/modular/fast-livo2-humble"
elif [[ -d "${SCRIPT_DIR}/automap_pro/src/modular/fast_livo2_humble" ]]; then
    SRC_FAST_LIVO="${SCRIPT_DIR}/automap_pro/src/modular/fast_livo2_humble"
else
    SRC_FAST_LIVO=""
fi
SRC_HBA="${SCRIPT_DIR}/automap_pro/src/modular/HBA-main"
SRC_TEASER="${SCRIPT_DIR}/automap_pro/src/modular/TEASER-plusplus-master"
SRC_OT_MSGS="${SCRIPT_DIR}/automap_pro/src/modular/overlap_transformer_msgs"
SRC_OT_ROS2="${SCRIPT_DIR}/automap_pro/src/modular/overlap_transformer_ros2"

# 数据路径
BAG_DIR="${SCRIPT_DIR}/data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2"
BAG_FILE="${BAG_DIR}/nya_02_ros2.db3"

# ─────────────────────────────────────────────────────────────────────────────
# 选项解析
# ─────────────────────────────────────────────────────────────────────────────
DO_BUILD=true
DO_RUN=true
DO_CLEAN=false
USE_RVIZ=true
BUILD_TYPE="Release"
CUSTOM_BAG=""

args=("$@")
i=0
while [[ $i -lt ${#args[@]} ]]; do
    case "${args[$i]}" in
        --build)   DO_RUN=false ;;
        --run)     DO_BUILD=false ;;
        --clean)   DO_CLEAN=true ;;
        --no-rviz) USE_RVIZ=false ;;
        --debug)   BUILD_TYPE="Debug" ;;
        --help|-h) sed -n '/^# ══/,/^# ══/p' "$0" | sed 's/^# \?//'; exit 0 ;;
        --bag)     i=$((i+1)); CUSTOM_BAG="${args[$i]}" ;;
    esac
    i=$((i+1))
done

[[ -n "$CUSTOM_BAG" ]] && BAG_FILE="$CUSTOM_BAG"

# ─────────────────────────────────────────────────────────────────────────────
# 彩色输出
# ─────────────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

log_info()    { echo -e "${BLUE}[INFO ]${NC} $*"; }
log_ok()      { echo -e "${GREEN}[OK   ]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN ]${NC} $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }
log_section() { echo -e "\n${CYAN}${BOLD}══════════════════════════════════════════════${NC}"; \
                echo -e "${CYAN}${BOLD}  $*${NC}"; \
                echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"; }

# ─────────────────────────────────────────────────────────────────────────────
# 预检
# ─────────────────────────────────────────────────────────────────────────────
preflight_check() {
    log_section "预检"

    command -v docker &>/dev/null || { log_error "Docker 未安装"; exit 1; }
    docker info &>/dev/null       || { log_error "Docker daemon 未运行"; exit 1; }
    log_ok "Docker 运行正常"

    if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
        if [[ -n "${IMAGE_ARCHIVE}" ]] && [[ -f "${IMAGE_ARCHIVE}" ]]; then
            log_info "加载镜像 ${IMAGE_ARCHIVE} ..."
            docker load -i "${IMAGE_ARCHIVE}"
            log_ok "镜像已加载: ${IMAGE_NAME}"
        elif [[ "${IMAGE_NAME}" == nvcr.io/* ]] || [[ "${IMAGE_NAME}" == ghcr.io/* ]]; then
            log_info "拉取镜像 ${IMAGE_NAME} ..."
            docker pull "${IMAGE_NAME}"
            log_ok "镜像已就绪: ${IMAGE_NAME}"
        else
            log_error "镜像 ${IMAGE_NAME} 不存在；未找到 tar ${IMAGE_ARCHIVE:-（未配置）}，且非可 pull 的 nvcr.io/ghcr.io 镜像"
            exit 1
        fi
    else
        log_ok "镜像 ${IMAGE_NAME} 就绪"
    fi

    # GPU 检查（可选，失败不退出）
    if docker run --rm --gpus all "${IMAGE_NAME}" nvidia-smi &>/dev/null 2>&1; then
        log_ok "NVIDIA GPU 可用"
        GPU_FLAG="--gpus all"
    else
        log_warn "NVIDIA GPU 不可用，将使用 CPU 模式"
        GPU_FLAG=""
    fi

    # 数据文件检查
    if [[ ! -f "${BAG_FILE}" ]]; then
        log_error "ROS2 bag 文件不存在: ${BAG_FILE}"
        log_error "请检查数据路径或使用 --bag <path> 指定"
        exit 1
    fi
    log_ok "ROS2 bag: ${BAG_FILE}"

    # 源码检查
    for dir in "${SRC_AUTOMAP_PRO}" "${SRC_HBA}"; do
        [[ -d "$dir" ]] || { log_error "源码目录不存在: $dir"; exit 1; }
    done
    if [[ -n "${SRC_FAST_LIVO}" && -d "${SRC_FAST_LIVO}" ]]; then
        log_ok "fast-livo2-humble: $(basename "${SRC_FAST_LIVO}")"
    else
        log_warn "fast-livo2-humble 未找到 (请检查 automap_pro/src/modular/fast-livo2-humble)"
    fi
    [[ -d "${SRC_TEASER}" ]]    || log_warn "TEASER++ 未找到: ${SRC_TEASER}"
    log_ok "源码路径检查完成"

    # 创建宿主机目录
    mkdir -p "${HOST_LOGS}" "${HOST_OUTPUT}" "${HOST_WS}/src"
}

# ─────────────────────────────────────────────────────────────────────────────
# 生成容器内构建脚本（内嵌，避免挂载脚本文件的权限问题）
# ─────────────────────────────────────────────────────────────────────────────
generate_build_script() {
    cat <<'INNERSCRIPT'
#!/usr/bin/env bash
# 此脚本在容器内执行，设置工作空间并编译
set -euo pipefail

CONTAINER_WS="${CONTAINER_WS:-/workspace/automap_ws}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
DO_CLEAN="${DO_CLEAN:-false}"
JOBS=$(nproc)

# ── 彩色输出 ──────────────────────────────────────────────────────────────────
log_info()  { echo -e "\033[0;34m[INFO ]\033[0m $*"; }
log_ok()    { echo -e "\033[0;32m[OK   ]\033[0m $*"; }
log_warn()  { echo -e "\033[1;33m[WARN ]\033[0m $*"; }
log_error() { echo -e "\033[0;31m[ERROR]\033[0m $*" >&2; }
log_sec()   { echo -e "\n\033[0;36m\033[1m══ $* ══\033[0m"; }

cd "${CONTAINER_WS}"
# ROS setup.bash 可能引用未设置变量（如 AMENT_TRACE_SETUP_FILES），临时关闭 -u
set +u
source /opt/ros/${AUTOMAP_ROS_DISTRO:-humble}/setup.bash
set -u

# ── 清理（可选） ──────────────────────────────────────────────────────────────
if [[ "${DO_CLEAN}" == "true" ]]; then
    log_sec "清理编译产物"
    rm -rf build install log build_teaserpp
    log_ok "清理完成"
fi

# ── 设置工作空间（无符号链接：源码已直接挂载到 src/ 子目录） ─────────────────
log_sec "工作空间结构验证"
for pkg in automap_pro fast_livo hba hba_api overlap_transformer_msgs; do
    if [[ -d "src/${pkg}" ]]; then
        log_ok "  src/${pkg}  ✓"
    else
        log_warn "  src/${pkg}  missing"
    fi
done

# ── thrid_party 目录检查 ─────────────────────────────────────────────────────
TEASER_SRC="${CONTAINER_WS}/src/thrid_party/TEASER-plusplus"
if [[ ! -d "${TEASER_SRC}" ]]; then
    log_warn "TEASER++ 源码未挂载到 ${TEASER_SRC}，将跳过内联编译"
fi

# ── Step 1: overlap_transformer_msgs ─────────────────────────────────────────
log_sec "Step 1/7: overlap_transformer_msgs"
colcon build \
    --packages-select overlap_transformer_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    --event-handlers console_cohesion+ \
    --parallel-workers "${JOBS}" 2>&1 | tee -a "${CONTAINER_WS}/build.log"
set +u
source install/setup.bash
set -u
log_ok "overlap_transformer_msgs built"

# ── Step 2: vikit (fast_livo 依赖，与 GTSAM 一致：只编译一次，已安装则跳过) ─────
log_sec "Step 2/6: vikit_common + vikit_ros"
VIKIT_DIR="${CONTAINER_WS}/src/thrid_party/rpg_vikit_ros2"
if [[ -d "${VIKIT_DIR}/vikit_common" ]]; then
    if [[ -f "install/share/vikit_common/package.xml" && -f "install/share/vikit_ros/package.xml" ]]; then
        log_ok "vikit 已安装，跳过"
    else
        colcon build \
            --paths "${VIKIT_DIR}/vikit_common" "${VIKIT_DIR}/vikit_ros" \
            --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
            --event-handlers console_cohesion+ \
            --parallel-workers "${JOBS}" 2>&1 | tee -a "${CONTAINER_WS}/build.log"
        set +u
        source install/setup.bash
        set -u
        log_ok "vikit built"
    fi
else
    log_warn "vikit not found at ${VIKIT_DIR}, skipping"
fi

# ── Step 3: Sophus ────────────────────────────────────────────────────────────
log_sec "Step 3/6: Sophus"
if colcon list --packages-select Sophus | grep -q Sophus; then
    colcon build \
        --packages-select Sophus \
        --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" -DBUILD_SOPHUS_TESTS=OFF \
        --event-handlers console_cohesion+ \
        --parallel-workers "${JOBS}" 2>&1 | tee -a "${CONTAINER_WS}/build.log"
    set +u
    source install/setup.bash
    set -u
    log_ok "Sophus built"
else
    log_warn "Sophus not found in workspace, assuming system version"
fi

# ── Step 4: fast_livo ─────────────────────────────────────────────────────────
log_sec "Step 4/6: fast_livo (FAST-LIVO2)"
if [[ -d "${CONTAINER_WS}/src/fast_livo" ]]; then
    colcon build \
        --packages-select fast_livo \
        --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
        --event-handlers console_cohesion+ \
        --parallel-workers "${JOBS}" 2>&1 | tee -a "${CONTAINER_WS}/build.log"
    set +u
    source install/setup.bash
    set -u
    log_ok "fast_livo built"
else
    log_warn "fast_livo source not found, skipping"
fi

# ── Step 5: hba + hba_api ─────────────────────────────────────────────────────
log_sec "Step 5/6: hba + hba_api"
colcon build \
    --packages-select hba hba_api \
    --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    --event-handlers console_cohesion+ \
    --parallel-workers "${JOBS}" 2>&1 | tee -a "${CONTAINER_WS}/build.log"
set +u
source install/setup.bash
set -u
log_ok "hba + hba_api built"

# ── Step 6: automap_pro (主包) ────────────────────────────────────────────────
log_sec "Step 6/6: automap_pro"
CMAKE_EXTRA_ARGS="-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
CMAKE_EXTRA_ARGS="${CMAKE_EXTRA_ARGS} -DCMAKE_CXX_FLAGS=-march=native"
if [[ -d "${TEASER_SRC}" ]]; then
    log_info "  TEASER++ source at ${TEASER_SRC}: using inline add_subdirectory"
fi

colcon build \
    --packages-select automap_pro \
    --cmake-args ${CMAKE_EXTRA_ARGS} \
    --event-handlers console_direct+ \
    --parallel-workers "${JOBS}" 2>&1 | tee -a "${CONTAINER_WS}/build.log"
set +u
source install/setup.bash
set -u
log_ok "automap_pro built"

# ── 验证 ─────────────────────────────────────────────────────────────────────
log_sec "编译验证"
BUILD_OK=true
for pkg in automap_pro hba hba_api; do
    if [[ -f "install/${pkg}/share/${pkg}/package.xml" ]]; then
        log_ok "  ${pkg} installed ✓"
    else
        log_warn "  ${pkg} install artifact missing"
        BUILD_OK=false
    fi
done

ros2 pkg list 2>/dev/null | grep -E "^(automap_pro|fast_livo|hba)$" | while read p; do
    log_ok "  ros2 pkg: ${p} ✓"
done

echo ""
if [[ "${BUILD_OK}" == "true" ]]; then
    echo -e "\033[0;32m╔══════════════════════════════╗\033[0m"
    echo -e "\033[0;32m║   BUILD SUCCESS ✓             ║\033[0m"
    echo -e "\033[0;32m╚══════════════════════════════╝\033[0m"
else
    echo -e "\033[0;33m╔══════════════════════════════╗\033[0m"
    echo -e "\033[0;33m║   BUILD PARTIAL - CHECK LOG  ║\033[0m"
    echo -e "\033[0;33m╚══════════════════════════════╝\033[0m"
fi
INNERSCRIPT
}

# ─────────────────────────────────────────────────────────────────────────────
# 构建 Docker volume 挂载参数
# ─────────────────────────────────────────────────────────────────────────────
build_volume_args() {
    local args=()

    # 工作空间（编译产物持久化）
    args+=(-v "${HOST_WS}:${CONTAINER_WS}:rw")

    # 日志和输出
    args+=(-v "${HOST_LOGS}:${CONTAINER_LOGS}:rw")
    args+=(-v "${HOST_OUTPUT}:${CONTAINER_OUTPUT}:rw")

    # 数据路径（ROS2 bag）
    args+=(-v "$(dirname "${BAG_FILE}"):${CONTAINER_DATA}/bag:ro")

    # ── 源码包（无符号链接：直接挂载到 src/ 子目录）─────────────────────────
    # automap_pro 主控包
    args+=(-v "${SRC_AUTOMAP_PRO}:${CONTAINER_WS}/src/automap_pro:rw")

    # fast_livo (FAST-LIVO2)
    if [[ -d "${SRC_FAST_LIVO}" ]]; then
        args+=(-v "${SRC_FAST_LIVO}:${CONTAINER_WS}/src/fast_livo:rw")
    fi

    # HBA (ROS2 节点包)
    if [[ -d "${SRC_HBA}/HBA_ROS2" ]]; then
        args+=(-v "${SRC_HBA}/HBA_ROS2:${CONTAINER_WS}/src/hba:rw")
    elif [[ -d "${SRC_HBA}" ]]; then
        args+=(-v "${SRC_HBA}:${CONTAINER_WS}/src/hba:rw")
    fi

    # hba_api (新包，在 automap_ws/src/hba_api/)
    if [[ -d "${HOST_WS}/src/hba_api" ]]; then
        args+=(-v "${HOST_WS}/src/hba_api:${CONTAINER_WS}/src/hba_api:rw")
    fi

    # overlap_transformer_msgs
    if [[ -d "${SRC_OT_MSGS}" ]]; then
        args+=(-v "${SRC_OT_MSGS}:${CONTAINER_WS}/src/overlap_transformer_msgs:rw")
    fi

    # overlap_transformer_ros2 (Python 服务，可选)
    if [[ -d "${SRC_OT_ROS2}" ]]; then
        args+=(-v "${SRC_OT_ROS2}:${CONTAINER_WS}/src/overlap_transformer_ros2:rw")
    fi

    # TEASER++ 源码（add_subdirectory 模式）
    if [[ -d "${SRC_TEASER}" ]]; then
        args+=(-v "${SRC_TEASER}:${CONTAINER_WS}/src/thrid_party/TEASER-plusplus:ro")
    fi

    # vikit_ros2 (fast_livo 依赖)
    local VIKIT_SRC="/home/wqs/Documents/github/mapping/thrid_party/rpg_vikit_ros2"
    [[ ! -d "${VIKIT_SRC}" ]] && VIKIT_SRC="${SCRIPT_DIR}/automap_ws/src/thrid_party/rpg_vikit_ros2"
    if [[ -d "${VIKIT_SRC}" ]]; then
        args+=(-v "${VIKIT_SRC}:${CONTAINER_WS}/src/thrid_party/rpg_vikit_ros2:ro")
    fi

    # X11 转发（RViz）
    args+=(-v "/tmp/.X11-unix:/tmp/.X11-unix:rw")
    args+=(-v "/dev:/dev")

    echo "${args[@]}"
}

# ─────────────────────────────────────────────────────────────────────────────
# 编译
# ─────────────────────────────────────────────────────────────────────────────
do_build() {
    log_section "Docker 编译 (${BUILD_TYPE})"
    log_info "镜像: ${IMAGE_NAME}"
    log_info "工作空间: ${HOST_WS} → ${CONTAINER_WS}"

    # 生成构建脚本到临时文件
    local build_script
    build_script=$(mktemp /tmp/automap_build_XXXXX.sh)
    generate_build_script > "${build_script}"
    chmod +x "${build_script}"

    xhost +local:docker &>/dev/null || true

    local vol_args
    read -ra vol_args <<< "$(build_volume_args)"

    docker run --rm \
        ${GPU_FLAG:-} \
        --net=host \
        "${vol_args[@]}" \
        -v "${build_script}:/tmp/automap_build.sh:ro" \
        -e CONTAINER_WS="${CONTAINER_WS}" \
        -e BUILD_TYPE="${BUILD_TYPE}" \
        -e DO_CLEAN="${DO_CLEAN}" \
        -e AUTOMAP_ROS_DISTRO="${AUTOMAP_ROS_DISTRO}" \
        -e AUTOMAP_LOG_LEVEL="debug" \
        -w "${CONTAINER_WS}" \
        "${IMAGE_NAME}" \
        bash /tmp/automap_build.sh

    local exit_code=$?
    rm -f "${build_script}"

    if [[ ${exit_code} -ne 0 ]]; then
        log_error "编译失败 (exit=${exit_code})"
        log_error "查看日志: ${HOST_WS}/build.log"
        exit 1
    fi
    log_ok "编译成功"
}

# ─────────────────────────────────────────────────────────────────────────────
# 运行
# ─────────────────────────────────────────────────────────────────────────────
do_run() {
    log_section "启动建图系统"

    # bag 文件在容器内的路径
    local container_bag="${CONTAINER_DATA}/bag/$(basename "${BAG_FILE}")"
    local rviz_arg="use_rviz:=$(echo ${USE_RVIZ} | tr '[:upper:]' '[:lower:]')"
    local config_path="${CONTAINER_WS}/src/automap_pro/config/system_config.yaml"
    local log_dir="${CONTAINER_LOGS}"

    log_info "ROS2 bag: ${BAG_FILE}"
    log_info "Container bag path: ${container_bag}"
    log_info "Log dir: ${HOST_LOGS}"

    xhost +local:docker &>/dev/null || true

    local vol_args
    read -ra vol_args <<< "$(build_volume_args)"

    # 生成运行脚本（容器内执行）
    # 约定：\$VAR / \$(...) 在容器内展开；${VAR} 在宿主机生成此字符串时展开
    local run_cmd="
set -uo pipefail
set +u
source /opt/ros/${AUTOMAP_ROS_DISTRO}/setup.bash
source ${CONTAINER_WS}/install/setup.bash
set -u

export AUTOMAP_LOG_LEVEL=info
export AUTOMAP_LOG_DIR=${log_dir}

# ── 带时间戳的彩色日志 ────────────────────────────────────────────────
_ts() { date '+%Y-%m-%d %H:%M:%S'; }
log_info()  { echo -e \"\$(_ts) \033[0;34m[INFO ]\033[0m \$*\"; }
log_ok()    { echo -e \"\$(_ts) \033[0;32m[OK   ]\033[0m \$*\"; }
log_warn()  { echo -e \"\$(_ts) \033[1;33m[WARN ]\033[0m \$*\"; }
log_error() { echo -e \"\$(_ts) \033[0;31m[ERROR]\033[0m \$*\" >&2; }
log_mon()   { echo -e \"\$(_ts) \033[0;36m[MON  ]\033[0m \$*\"; }

echo ''
echo '╔══════════════════════════════════════════════╗'
echo '║        AutoMap-Pro v2.0  STARTING             ║'
echo '╠══════════════════════════════════════════════╣'
echo \"║  ROS2 bag: ${container_bag}\"
echo \"║  Config:   ${config_path}\"
echo \"║  Log dir:  ${log_dir}\"
echo '╚══════════════════════════════════════════════╝'
echo ''

# ── 从 system_config.yaml 动态读取传感器话题（避免硬编码 /livox/*）────────
LID_TOPIC=\$(python3 -c \"import yaml; c=yaml.safe_load(open('${config_path}')); print(c['sensor']['lidar']['topic'])\" 2>/dev/null || echo '/os1_cloud_node1/points')
IMU_TOPIC=\$(python3 -c \"import yaml; c=yaml.safe_load(open('${config_path}')); print(c['sensor']['imu']['topic'])\"  2>/dev/null || echo '/os1_cloud_node1/imu')
log_info \"Input topics from config: LiDAR=\${LID_TOPIC}  IMU=\${IMU_TOPIC}\"

# ════════════════════════════════════════════════════════════════
# Step 1: 先启动建图系统（fast_livo + automap_system），暂不播放 bag
# ════════════════════════════════════════════════════════════════
log_info 'Step 1: Starting launch system (fast_livo + automap_system) using unified config: ${config_path}'
log_info '  fast_livo params will be extracted from fast_livo: section at runtime'
ros2 launch automap_pro automap_composable.launch.py \
    config:='${config_path}' \
    '${rviz_arg}' \
    &
LAUNCH_PID=\$!
log_info \"automap_pro launch started (PID=\${LAUNCH_PID})\"

# ════════════════════════════════════════════════════════════════
# Step 2: 等待 fast_livo 节点健康（最多 45s）+ 稳定性检查（防启动即崩溃）
# ════════════════════════════════════════════════════════════════
log_info 'Step 2: Waiting for /fast_livo node (max 45s). If it crashes you will see [ERROR] process has died below.'
MAX_WAIT=45
STABILITY_WAIT=5
ELAPSED=0
FAST_LIVO_OK=false
while [[ \$ELAPSED -lt \$MAX_WAIT ]]; do
    if ! kill -0 \$LAUNCH_PID 2>/dev/null; then
        log_error 'Launch process exited unexpectedly — fast_livo likely crashed!'
        break
    fi
    if ros2 node list 2>/dev/null | grep -qE '(^|/)fast_livo\$'; then
        NODE_LIST=\$(ros2 node list 2>/dev/null | grep -E '(^|/)fast_livo\$' || true)
        log_ok \"fast_livo node seen in ros2 node list (after \${ELAPSED}s): \$NODE_LIST\"
        log_info \"Stability check: waiting \${STABILITY_WAIT}s then re-checking node and publisher ...\"
        sleep \${STABILITY_WAIT}
        if ! kill -0 \$LAUNCH_PID 2>/dev/null; then
            log_error 'Launch process died during stability wait — fast_livo crashed.'
            break
        fi
        if ! ros2 node list 2>/dev/null | grep -qE '(^|/)fast_livo\$'; then
            log_error 'fast_livo node DISAPPEARED after \${STABILITY_WAIT}s (crashed). Search log for \"process has died\" or \"parameter \\\"\\\"\" or InvalidParameterTypeException.'
            break
        fi
        # 检查关键话题是否有发布者（有发布者才说明真的在跑）
        if ros2 topic info /aft_mapped_to_init 2>/dev/null | grep -q 'Publisher count: [1-9]'; then
            log_ok \"fast_livo node STABLE and publishing /aft_mapped_to_init (after \${ELAPSED}s + \${STABILITY_WAIT}s)\"
            FAST_LIVO_OK=true
            break
        fi
        log_ok \"fast_livo node still present after stability wait (topic may appear after first lidar frame)\"
        FAST_LIVO_OK=true
        break
    fi
    sleep 2
    ELAPSED=\$((ELAPSED + 2))
    log_info \"  Waiting for fast_livo... \${ELAPSED}s / \${MAX_WAIT}s\"
done

if [[ \"\$FAST_LIVO_OK\" != 'true' ]]; then
    log_error '======================================================'
    log_error ' fast_livo did NOT become active or STABLE within \${MAX_WAIT}s'
    log_error ' Possible causes:'
    log_error '   1. parameter \"\" crash in avia.yaml (check build; search log for InvalidParameterTypeException)'
    log_error '   2. fast_livo binary not compiled / not found'
    log_error '   3. Config path mismatch (check --params-file output above)'
    log_error ' Aborting bag playback. Fix fast_livo first.'
    log_error '======================================================'
    kill \$LAUNCH_PID 2>/dev/null || true
    exit 1
fi

# ════════════════════════════════════════════════════════════════
# Step 3: 确认 fast_livo 输出话题已注册（并打开发布者数量便于排障）
# ════════════════════════════════════════════════════════════════
log_info 'Step 3: Checking fast_livo output topic registration and publisher count ...'
EXPECTED_TOPICS='/aft_mapped_to_init /cloud_registered /fast_livo/keyframe_info'
for topic in \$EXPECTED_TOPICS; do
    if ros2 topic list 2>/dev/null | grep -q \"\$topic\"; then
        PUB_COUNT=\$(ros2 topic info \"\$topic\" 2>/dev/null | grep 'Publisher count:' | sed 's/.*: *//' || echo '?')
        log_ok \"  [REG] \$topic  (publishers: \$PUB_COUNT)\"
    else
        log_warn \"  [MISS] \$topic (will appear after first lidar frame)\"
    fi
done

# ════════════════════════════════════════════════════════════════
# Step 4: 启动 bag 播放（rate=0.5 正常频率，--clock 仿真时间，--loop 循环）
# ════════════════════════════════════════════════════════════════
log_info \"Step 4: Starting ros2 bag play (rate=0.5, --clock, --loop). Key input topics: \${LID_TOPIC}, \${IMU_TOPIC}\"
ros2 bag play '${container_bag}' \
    --rate 0.5 \
    --clock \
    --loop \
    &
BAG_PID=\$!
log_ok \"ros2 bag play started (PID=\${BAG_PID})\"

# ════════════════════════════════════════════════════════════════
# Step 5: 首次数据接收确认（15s 内检测关键话题是否有数据流入）
# ════════════════════════════════════════════════════════════════
(
    sleep 15
    log_info '--- [DATACHECK] First-data verification (15s after bag play) ---'
    ALL_OK=true
    # 输入话题（bag → fast_livo）
    for topic in \${LID_TOPIC} \${IMU_TOPIC}; do
        MSG=\$(timeout 4 ros2 topic echo \"\$topic\" --once 2>/dev/null | head -c 80 || true)
        if [[ -n \"\$MSG\" ]]; then
            log_ok \"  [INPUT ] \$topic → data received ✓\"
        else
            log_warn \"  [INPUT ] \$topic → NO data in 4s (bag not playing or topic mismatch)\"
            ALL_OK=false
        fi
    done
    # 输出话题（fast_livo → automap_system）
    for topic in /aft_mapped_to_init /cloud_registered; do
        MSG=\$(timeout 6 ros2 topic echo \"\$topic\" --once 2>/dev/null | head -c 80 || true)
        if [[ -n \"\$MSG\" ]]; then
            log_ok \"  [OUTPUT] \$topic → data received ✓\"
        else
            log_warn \"  [OUTPUT] \$topic → NO data in 6s (fast_livo not outputting)\"
            ALL_OK=false
        fi
    done
    if [[ \"\$ALL_OK\" == 'true' ]]; then
        log_ok '[DATACHECK] All key topics flowing normally ✓'
    else
        log_warn '[DATACHECK] Some OUTPUT topics missing.'
        log_warn '[DATACHECK] Likely cause: fast_livo crashed at startup — search log for \"process has died\" or \"parameter \\\"\\\"\" or InvalidParameterTypeException above.'
    fi
    log_info '--- [DATACHECK] End ---'
) &
DATACHECK_PID=\$!

# ════════════════════════════════════════════════════════════════
# Step 6: 后台数据流监控（每 60s 打印关键话题频率快照）
# ════════════════════════════════════════════════════════════════
(
    sleep 60
    CHECK_NUM=0
    while kill -0 \$BAG_PID 2>/dev/null && kill -0 \$LAUNCH_PID 2>/dev/null; do
        CHECK_NUM=\$((CHECK_NUM + 1))
        log_mon \"=== Periodic Data-Flow Check #\${CHECK_NUM} ========================\"
        # 输入话题
        for topic in \${LID_TOPIC} \${IMU_TOPIC}; do
            HAVE=\$(timeout 3 ros2 topic echo \"\$topic\" --once 2>/dev/null | head -c 10 && echo 'yes' || echo 'no')
            if [[ \"\$HAVE\" == *'yes'* ]]; then
                log_mon \"  INPUT  \$topic  → active\"
            else
                log_warn \"  INPUT  \$topic  → SILENT (bag may have stalled)\"
            fi
        done
        # fast_livo 输出话题
        for topic in /aft_mapped_to_init /cloud_registered /fast_livo/keyframe_info; do
            HAVE=\$(timeout 5 ros2 topic echo \"\$topic\" --once 2>/dev/null | head -c 10 && echo 'yes' || echo 'no')
            if [[ \"\$HAVE\" == *'yes'* ]]; then
                log_mon \"  OUTPUT \$topic  → active\"
            else
                log_warn \"  OUTPUT \$topic  → SILENT (fast_livo may have crashed; check for 'process has died' earlier in log)\"
            fi
        done
        log_mon \"================================================================\"
        sleep 60
    done
    log_mon 'Periodic monitor stopped (bag or launch exited).'
) &
MONITOR_PID=\$!

# ════════════════════════════════════════════════════════════════
# Step 7: 等待 bag 或 launch 任一退出
# ════════════════════════════════════════════════════════════════
log_info \"Step 7: Data flow: bag → \${LID_TOPIC}, \${IMU_TOPIC} → fast_livo → /aft_mapped_to_init, /cloud_registered → automap_system. Waiting for exit ...\"
wait -n \${BAG_PID} \${LAUNCH_PID}
EXIT_CODE=\$?
log_info \"Main process exited (code=\${EXIT_CODE}), shutting down all subprocesses ...\"
kill \${BAG_PID} \${LAUNCH_PID} \${DATACHECK_PID} \${MONITOR_PID} 2>/dev/null || true
exit \${EXIT_CODE}
"

    docker run -it --rm \
        ${GPU_FLAG:-} \
        --privileged \
        --net=host \
        --ipc=host \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e AUTOMAP_ROS_DISTRO="${AUTOMAP_ROS_DISTRO}" \
        -e AUTOMAP_LOG_LEVEL=info \
        "${vol_args[@]}" \
        -w "${CONTAINER_WS}" \
        "${IMAGE_NAME}" \
        bash -c "${run_cmd}"
}

# ─────────────────────────────────────────────────────────────────────────────
# 主流程
# ─────────────────────────────────────────────────────────────────────────────
main() {
    log_section "AutoMap-Pro v2.0  一键脚本"
    log_info "时间: $(date '+%Y-%m-%d %H:%M:%S')"
    log_info "选项: build=${DO_BUILD} run=${DO_RUN} clean=${DO_CLEAN} type=${BUILD_TYPE} rviz=${USE_RVIZ}"
    log_info "Bag:  ${BAG_FILE}"

    preflight_check

    if [[ "${DO_BUILD}" == "true" ]]; then
        do_build
    else
        log_info "--build 未指定，跳过编译"
    fi

    if [[ "${DO_RUN}" == "true" ]]; then
        do_run
    else
        log_info "--run 未指定，跳过运行"
    fi

    log_section "完成"
    log_ok "日志保存于: ${HOST_LOGS}"
    log_ok "输出保存于: ${HOST_OUTPUT}"
}

main "$@"
