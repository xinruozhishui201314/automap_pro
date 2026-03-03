#!/bin/bash
# ROS1 Bag → 转换 → Docker 建图 一键脚本
# 在宿主机将 ROS1 bag 转为 ROS2，再调用 run_full_mapping_docker.sh 建图（不依赖容器内 Docker）

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 默认 Humble 转换镜像（含 ROS1 读取 + ros2 bag convert）
CONVERTER_IMAGE="${CONVERTER_IMAGE:-ros1-to-ros2-converter}"
# 若设置则使用「纯 Python rosbags」小镜像做转换（不依赖 ROS1/ROS2，无需在 Noetic 里装 pip）
# 因 Noetic 为纯 ROS1 环境，无法直接输出 ROS2 包，转换由独立 rosbags 镜像完成
# 例: export ROS1_CONVERTER_IMAGE=1  或  export ROS1_CONVERTER_IMAGE=docker.1ms.run/library/ros:noetic-perception
ROS1_CONVERTER_IMAGE="${ROS1_CONVERTER_IMAGE:-}"
ROSBAGS_IMAGE_NAME="ros1-to-ros2-rosbags"

log_info()    { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()    { echo -e "${CYAN}[STEP]${NC} $*"; }
log_success() { echo -e "${GREEN}[OK]${NC} $*"; }

show_help() {
    cat << EOF
ROS1 Bag 转 ROS2 并建图

用法: $0 -b <ros1_bag> [run_full_mapping_docker.sh 的其它选项]

选项:
    -b, --bag FILE   ROS1 bag 文件路径（相对或绝对，如 data/automap_input/.../xxx.bag）
    其余选项会透传给 run_full_mapping_docker.sh（如 -c, -o, --build 等）

流程:
    1) 在宿主机用 Docker 将 ROS1 bag 转为 ROS2（输出到同目录下的 xxx_ros2/）
    2) 调用 run_full_mapping_docker.sh -b <xxx_ros2> 进行建图

数据流说明:
    - play_data.sh：仅用于「播放」bag（ROS1 用 rosbag play，ROS2 用 ros2 bag play），不产出新文件。
    - 建图需要 ROS2 格式：run_full_mapping_docker.sh 会回放 bag 进行建图，因此需先将 ROS1 转为 ROS2。
    - 本脚本 = 宿主机转换 + 建图一步完成，无需在容器内再跑 Docker 转换。

示例:
    $0 -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
    $0 -b data/automap_input/nya_02.bag -o data/automap_output/nya_02

宿主机仅有 ROS1(Noetic)、无 ROS2 时，用 rosbags 小镜像转换（不依赖 Noetic 内 pip）:
    export ROS1_CONVERTER_IMAGE=1
    $0 -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
  国内构建过慢可指定 PyPI 镜像（仅首次构建镜像时生效）:
    export PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
    export ROS1_CONVERTER_IMAGE=1
    $0 -b data/automap_input/.../xxx.bag

注意: Noetic 为纯 ROS1 环境，不能直接输出 ROS2 包；脚本会构建并运行纯 Python 的 rosbags 镜像完成转换。
若 bag 未建索引导致报错，可先在 Noetic 容器内: rosbag reindex xxx.bag

EOF
}

# 解析 -b/--bag，其余参数透传
BAG_FILE=""
PASSTHROUGH=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -b|--bag|--bag-file)
            BAG_FILE="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            PASSTHROUGH+=("$1")
            shift
            ;;
    esac
done

if [ -z "$BAG_FILE" ]; then
    log_error "请指定 ROS1 bag 文件: -b <path>"
    show_help
    exit 1
fi

# 解析为绝对路径
if [[ "$BAG_FILE" != /* ]]; then
    BAG_ABS="$SCRIPT_DIR/$BAG_FILE"
else
    BAG_ABS="$BAG_FILE"
fi

if [ ! -f "$BAG_ABS" ]; then
    log_error "Bag 文件不存在: $BAG_ABS"
    exit 1
fi

# 输出目录：与 bag 同目录，名为 <basename>_ros2（ROS2 为目录）
BAG_DIR=$(dirname "$BAG_FILE")
BAG_NAME=$(basename "$BAG_FILE" .bag)
ROS2_BAG_REL="$BAG_DIR/${BAG_NAME}_ros2"
ROS2_BAG_ABS="$SCRIPT_DIR/$ROS2_BAG_REL"

# 若已是 ROS2 目录则直接建图
if [ -d "$ROS2_BAG_ABS" ] && [ -f "$ROS2_BAG_ABS/metadata.yaml" ]; then
    log_info "已存在 ROS2 目录，跳过转换: $ROS2_BAG_REL"
    exec "$SCRIPT_DIR/run_full_mapping_docker.sh" -b "$ROS2_BAG_REL" "${PASSTHROUGH[@]}"
fi

# 检测是否为 ROS1（避免误转 ROS2）
detect_ros1() {
    if command -v file >/dev/null 2>&1; then
        local ft=$(file "$BAG_ABS" 2>/dev/null | cut -d: -f2)
        if echo "$ft" | grep -q "SQLite"; then
            return 1
        fi
        if echo "$ft" | grep -q "data"; then
            return 0
        fi
    fi
    return 0
}

if ! detect_ros1; then
    log_warn "文件可能已是 ROS2 格式，将直接用于建图"
    exec "$SCRIPT_DIR/run_full_mapping_docker.sh" -b "$BAG_FILE" "${PASSTHROUGH[@]}"
fi

# 容器内路径：挂载 SCRIPT_DIR/data -> /workspace/data
if [[ "$BAG_DIR" == data/* ]]; then
    BAG_REL_TO_DATA="${BAG_DIR#data/}"
else
    BAG_REL_TO_DATA="$BAG_DIR"
fi
CONTAINER_INPUT="/workspace/data/$BAG_REL_TO_DATA/$(basename "$BAG_FILE")"
CONTAINER_OUTPUT="/workspace/data/$BAG_REL_TO_DATA/${BAG_NAME}_ros2"

log_info "输入 (容器内): $CONTAINER_INPUT"
log_info "输出 (容器内): $CONTAINER_OUTPUT"

if [ -n "$ROS1_CONVERTER_IMAGE" ]; then
    # Noetic 为纯 ROS1，无法直接输出 ROS2 包 → 使用纯 Python rosbags 小镜像做转换（不依赖 ROS1/ROS2）
    log_step "使用 rosbags 镜像转换（无需 ROS1/ROS2 环境）"
    if ! docker images | grep -q "$ROSBAGS_IMAGE_NAME"; then
        log_step "构建 rosbags 转换镜像..."
        DOCKER_BUILDKIT=1 docker build -t "$ROSBAGS_IMAGE_NAME" \
            ${PIP_INDEX_URL:+--build-arg PIP_INDEX_URL="$PIP_INDEX_URL"} \
            -f "$SCRIPT_DIR/docker/converter.Dockerfile.rosbags" "$SCRIPT_DIR" || {
            log_error "rosbags 镜像构建失败"
            exit 1
        }
    fi
    docker run --rm \
        -v "$SCRIPT_DIR/data:/workspace/data" \
        "$ROSBAGS_IMAGE_NAME" \
        rosbags-convert --src "$CONTAINER_INPUT" --dst "$CONTAINER_OUTPUT" || {
        log_error "ROS1 → ROS2 转换失败（rosbags）"
        exit 1
    }
else
    # 使用项目 Humble 转换镜像（需含 ros2 bag convert）
    log_step "在宿主机将 ROS1 bag 转为 ROS2"
    if ! docker images | grep -q "$CONVERTER_IMAGE"; then
        log_step "构建转换镜像..."
        docker build -t "$CONVERTER_IMAGE" -f "$SCRIPT_DIR/docker/converter.Dockerfile" "$SCRIPT_DIR" || {
            log_error "转换镜像构建失败"
            exit 1
        }
    fi
    docker run --rm \
        -v "$SCRIPT_DIR/data:/workspace/data" \
        "$CONVERTER_IMAGE" \
        bash -c "
            source /opt/ros/humble/setup.bash &&
            ros2 bag convert '$CONTAINER_INPUT' '$CONTAINER_OUTPUT'
        " || {
        log_error "ROS1 → ROS2 转换失败"
        exit 1
    }
fi

if [ ! -d "$ROS2_BAG_ABS" ] || [ ! -f "$ROS2_BAG_ABS/metadata.yaml" ]; then
    log_error "转换后未找到有效 ROS2 目录: $ROS2_BAG_ABS"
    exit 1
fi

log_success "转换完成: $ROS2_BAG_REL"
log_step "启动 Docker 建图..."
exec "$SCRIPT_DIR/run_full_mapping_docker.sh" -b "$ROS2_BAG_REL" "${PASSTHROUGH[@]}"
