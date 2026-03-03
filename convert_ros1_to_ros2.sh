#!/bin/bash
# ROS1 Bag → ROS2 Bag 独立转换脚本
# 先转换，再使用 run_full_mapping_enhanced.sh -b <输出目录> --no-convert 建图
# 支持：Docker（ros1-to-ros2-converter / rosbags 镜像）或本机 rosbags-convert（pip install rosbags）

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERTER_IMAGE="${CONVERTER_IMAGE:-ros1-to-ros2-converter}"
ROSBAGS_IMAGE="${ROSBAGS_IMAGE:-ros1-to-ros2-rosbags}"

log_info()    { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $*"; }
log_step()    { echo -e "${CYAN}[STEP]${NC} $*"; }
log_ok()      { echo -e "${GREEN}[OK]${NC} $*"; }

show_help() {
    cat << EOF
用法: $0 -b <ros1_bag路径> [选项]

先将 ROS1 bag 转为 ROS2 格式，再执行建图时请指定转换后的目录并加 --no-convert。

选项:
  -b, --bag FILE   ROS1 bag 文件路径（必填，如 data/automap_input/.../nya_02.bag）
  --rosbags       使用 rosbags 镜像转换（不依赖 ROS1/ROS2，适合无 Humble 环境）
  -h, --help      显示此帮助

转换输出: 与 bag 同目录下的 <文件名>_ros2/ 目录（ROS2 为目录格式）

示例:
  # 1) 先转换
  $0 -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

  # 2) 再建图（指定转换后的目录并跳过转换步骤）
  ./run_full_mapping_enhanced.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2 --no-convert

无 Docker 时: 若本机已安装 rosbags（pip install rosbags），将自动使用 rosbags-convert 进行转换。
EOF
}

BAG_FILE=""
USE_ROSBAGS_IMAGE=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        -b|--bag|--bag-file)
            BAG_FILE="$2"
            shift 2
            ;;
        --rosbags)
            USE_ROSBAGS_IMAGE=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            log_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

if [ -z "$BAG_FILE" ]; then
    log_error "请指定 ROS1 bag 文件: -b <路径>"
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

BAG_DIR_ABS=$(dirname "$BAG_ABS")
BAG_NAME=$(basename "$BAG_FILE" .bag)
ROS2_DIR_ABS="$BAG_DIR_ABS/${BAG_NAME}_ros2"

# 若已是 ROS2 目录则跳过
if [ -d "$ROS2_DIR_ABS" ] && [ -f "$ROS2_DIR_ABS/metadata.yaml" ]; then
    log_ok "已存在 ROS2 目录，跳过转换: $ROS2_DIR_ABS"
    echo ""
    echo "建图请执行:"
    echo "  ./run_full_mapping_enhanced.sh -b \"$ROS2_DIR_ABS\" --no-convert"
    echo "或（相对路径）："
    if [[ "$ROS2_DIR_ABS" == "$SCRIPT_DIR"/* ]]; then
        REL="${ROS2_DIR_ABS#$SCRIPT_DIR/}"
        echo "  ./run_full_mapping_enhanced.sh -b $REL --no-convert"
    fi
    exit 0
fi

# 检测是否为 ROS1（避免误转 ROS2）
detect_ros1() {
    if command -v file >/dev/null 2>&1; then
        local ft=$(file "$BAG_ABS" 2>/dev/null | cut -d: -f2)
        if echo "$ft" | grep -q "SQLite"; then return 1; fi
        if echo "$ft" | grep -q "data"; then return 0; fi
    fi
    return 0
}

if ! detect_ros1; then
    log_warn "文件可能已是 ROS2 格式（SQLite），无需转换"
    exit 0
fi

log_step "ROS1 → ROS2 转换: $BAG_FILE -> ${BAG_NAME}_ros2/"

# 方法1: 本机 rosbags-convert（无需 Docker）
if command -v rosbags-convert &>/dev/null; then
    log_info "使用本机 rosbags-convert 转换"
    rm -rf "$ROS2_DIR_ABS"
    if rosbags-convert --src "$BAG_ABS" --dst "$ROS2_DIR_ABS"; then
        log_ok "转换完成: $ROS2_DIR_ABS"
    else
        log_error "rosbags-convert 执行失败"
        exit 1
    fi
# 方法2: Docker（rosbags 小镜像 或 Humble 转换镜像）
elif command -v docker &>/dev/null; then
    CONTAINER_SRC="/workspace/$(basename "$BAG_ABS")"
    CONTAINER_DST="/workspace/${BAG_NAME}_ros2"
    if [ "$USE_ROSBAGS_IMAGE" = true ]; then
        if ! docker images | grep -q "$ROSBAGS_IMAGE"; then
            log_step "构建 rosbags 转换镜像..."
            DOCKER_BUILDKIT=1 docker build -t "$ROSBAGS_IMAGE" \
                -f "$SCRIPT_DIR/docker/converter.Dockerfile.rosbags" "$SCRIPT_DIR" || {
                log_error "镜像构建失败"
                exit 1
            }
        fi
        docker run --rm \
            -v "$BAG_DIR_ABS:/workspace" \
            "$ROSBAGS_IMAGE" \
            rosbags-convert --src "$CONTAINER_SRC" --dst "$CONTAINER_DST" || {
            log_error "转换失败（rosbags 镜像）"
            exit 1
        }
    else
        # 使用 Humble 转换镜像（ros2 bag convert）
        log_info "使用 Docker 镜像 $CONVERTER_IMAGE 转换"
        if ! docker images | grep -q "$CONVERTER_IMAGE"; then
            log_step "构建转换镜像..."
            docker build -t "$CONVERTER_IMAGE" -f "$SCRIPT_DIR/docker/converter.Dockerfile" "$SCRIPT_DIR" || {
                log_error "镜像构建失败；可尝试: $0 -b \"$BAG_FILE\" --rosbags"
                exit 1
            }
        fi
        docker run --rm \
            -v "$BAG_DIR_ABS:/workspace" \
            "$CONVERTER_IMAGE" \
            bash -c "
                source /opt/ros/humble/setup.bash 2>/dev/null || true
                ros2 bag convert '/workspace/$(basename "$BAG_ABS")' '/workspace/${BAG_NAME}_ros2'
            " || {
            log_error "转换失败（Humble 镜像）。可尝试: $0 -b \"$BAG_FILE\" --rosbags"
            exit 1
        }
    fi
    log_ok "转换完成: $ROS2_DIR_ABS"
else
    log_error "未找到 Docker 且本机无 rosbags-convert。请任选其一："
    echo "  1) 安装 Docker 后重新运行本脚本"
    echo "  2) 本机安装: pip install rosbags  后重新运行"
    exit 1
fi

if [ ! -d "$ROS2_DIR_ABS" ] || [ ! -f "$ROS2_DIR_ABS/metadata.yaml" ]; then
    log_error "转换后未找到有效 ROS2 目录: $ROS2_DIR_ABS"
    exit 1
fi

# 修复 metadata.yaml：rosbags/ros2 bag convert 生成的多行 type_description_hash 会导致
# ros2 bag play 解析时 yaml-cpp "bad conversion"（第25行附近）。统一改为单行以兼容 Humble 播放。
if [ -f "$SCRIPT_DIR/scripts/fix_ros2_bag_metadata.py" ]; then
    if [ ! -w "$ROS2_DIR_ABS/metadata.yaml" ]; then
        chmod u+w "$ROS2_DIR_ABS/metadata.yaml" 2>/dev/null || true
    fi
    if python3 "$SCRIPT_DIR/scripts/fix_ros2_bag_metadata.py" "$ROS2_DIR_ABS"; then
        log_ok "已修复 metadata.yaml，兼容 ros2 bag play"
    elif [ ! -w "$ROS2_DIR_ABS/metadata.yaml" ]; then
        log_warn "metadata.yaml 无写权限（多为 Docker 生成），请执行："
        echo "  sudo chmod u+w $ROS2_DIR_ABS/metadata.yaml"
        echo "  python3 $SCRIPT_DIR/scripts/fix_ros2_bag_metadata.py $ROS2_DIR_ABS"
    fi
fi

echo ""
echo "建图请执行:"
echo "  ./run_full_mapping_enhanced.sh -b \"$ROS2_DIR_ABS\" --no-convert"
REL="${ROS2_DIR_ABS#$SCRIPT_DIR/}"
if [ "$REL" != "$ROS2_DIR_ABS" ]; then
    echo "或（相对路径）:"
    echo "  ./run_full_mapping_enhanced.sh -b $REL --no-convert"
fi
