#!/bin/bash
# AutoMap-Pro 建图启动脚本
# 用途：快速启动离线建图流程

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"
CONFIG="${CONFIG:-automap_pro/config/system_config.yaml}"
OUTPUT_DIR="${OUTPUT_DIR:-/data/automap_output}"
MODE="${MODE:-offline}"
RATE="${RATE:-1.0}"
USE_RVIZ="${USE_RVIZ:-true}"

# 显示帮助信息
show_help() {
    cat << EOF
AutoMap-Pro 建图启动脚本

用法: $0 [选项]

选项:
    -m, --mode MODE           建图模式: online, offline, incremental (默认: offline)
    -b, --bag-file FILE       ROS bag 文件路径 (默认: data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag)
    -c, --config FILE         配置文件路径 (默认: automap_pro/config/system_config.yaml)
    -o, --output-dir DIR      输出目录 (默认: /data/automap_output)
    -r, --rate RATE          回放速率 (默认: 1.0)
    --no-rviz                不启动 RViz
    --use-external-frontend  使用外部前端 (fast-livo2)
    --use-external-overlap   使用外部回环检测服务
    -h, --help               显示此帮助信息

示例:
    # 默认离线建图
    $0

    # 指定 bag 文件
    $0 -b /path/to/mapping.bag

    # 指定输出目录
    $0 -o /home/user/output

    # 在线建图
    $0 -m online

    # 不启动 RViz（后台运行）
    $0 --no-rviz

环境变量:
    BAG_FILE         ROS bag 文件路径
    CONFIG           配置文件路径
    OUTPUT_DIR       输出目录
    MODE             建图模式
    RATE             回放速率
    USE_RVIZ         是否启动 RViz (true/false)

EOF
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -m|--mode)
            MODE="$2"
            shift 2
            ;;
        -b|--bag-file)
            BAG_FILE="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG="$2"
            shift 2
            ;;
        -o|--output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -r|--rate)
            RATE="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ="false"
            shift
            ;;
        --use-external-frontend)
            USE_EXTERNAL_FRONTEND="true"
            shift
            ;;
        --use-external-overlap)
            USE_EXTERNAL_OVERLAP="true"
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}错误: 未知选项 $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 检查工作空间
WORKSPACE="${HOME}/automap_ws"
if [ ! -d "$WORKSPACE" ]; then
    echo -e "${RED}错误: 工作空间不存在: $WORKSPACE${NC}"
    echo -e "${YELLOW}请先运行: make setup${NC}"
    exit 1
fi

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}错误: ROS2 Humble 未安装${NC}"
    exit 1
fi

# Source 工作空间
if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
else
    echo -e "${YELLOW}警告: 工作空间未编译，尝试编译...${NC}"
    cd "$SCRIPT_DIR" && make build-release
    source "$WORKSPACE/install/setup.bash"
fi

# 检查配置文件
if [ ! -f "$SCRIPT_DIR/$CONFIG" ]; then
    echo -e "${RED}错误: 配置文件不存在: $SCRIPT_DIR/$CONFIG${NC}"
    exit 1
fi

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

# 显示配置信息
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}AutoMap-Pro 建图配置${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}模式:${NC}           $MODE"
echo -e "${GREEN}Bag 文件:${NC}      $BAG_FILE"
echo -e "${GREEN}配置文件:${NC}      $CONFIG"
echo -e "${GREEN}输出目录:${NC}      $OUTPUT_DIR"
echo -e "${GREEN}回放速率:${NC}      $RATE"
echo -e "${GREEN}RViz:${NC}          $USE_RVIZ"
echo -e "${BLUE}========================================${NC}"

# 验证 bag 文件（如果是离线模式）
if [ "$MODE" = "offline" ]; then
    # 转换相对路径为绝对路径
    if [[ "$BAG_FILE" != /* ]]; then
        BAG_FILE="$SCRIPT_DIR/$BAG_FILE"
    fi

    if [ ! -f "$BAG_FILE" ]; then
        echo -e "${RED}错误: Bag 文件不存在: $BAG_FILE${NC}"
        exit 1
    fi

    # 显示 bag 文件信息
    echo -e "\n${YELLOW}Bag 文件信息:${NC}"
    ros2 bag info "$BAG_FILE" | head -20
fi

# 确认启动
if [ "$USE_RVIZ" = "true" ]; then
    echo -e "\n${YELLOW}即将启动建图流程，按 Enter 继续，或 Ctrl+C 取消...${NC}"
    read -r
fi

# 启动建图
echo -e "\n${GREEN}启动建图...${NC}\n"

case $MODE in
    online)
        # 在线建图
        ros2 launch automap_pro automap_online.launch.py \
            config:="$SCRIPT_DIR/$CONFIG" \
            output_dir:="$OUTPUT_DIR" \
            use_rviz:="$USE_RVIZ" \
            use_external_frontend:="${USE_EXTERNAL_FRONTEND:-false}" \
            use_external_overlap:="${USE_EXTERNAL_OVERLAP:-false}"
        ;;
    offline)
        # 离线建图
        ros2 launch automap_pro automap_offline.launch.py \
            config:="$SCRIPT_DIR/$CONFIG" \
            bag_file:="$BAG_FILE" \
            rate:="$RATE" \
            use_rviz:="$USE_RVIZ" \
            use_external_frontend:="${USE_EXTERNAL_FRONTEND:-false}" \
            use_external_overlap:="${USE_EXTERNAL_OVERLAP:-false}"
        ;;
    incremental)
        # 增量式建图
        ros2 launch automap_pro automap_incremental.launch.py \
            config:="$SCRIPT_DIR/$CONFIG" \
            output_dir:="$OUTPUT_DIR" \
            session_id:="${SESSION_ID:-1}" \
            prev_session:="${PREV_SESSION:-}" \
            use_rviz:="$USE_RVIZ" \
            use_external_frontend:="${USE_EXTERNAL_FRONTEND:-false}" \
            use_external_overlap:="${USE_EXTERNAL_OVERLAP:-false}"
        ;;
    *)
        echo -e "${RED}错误: 不支持的模式: $MODE${NC}"
        echo -e "${YELLOW}支持的模式: online, offline, incremental${NC}"
        exit 1
        ;;
esac
