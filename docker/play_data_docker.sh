#!/bin/bash
# AutoMap-Pro Docker 数据播放脚本
# 在 Docker 容器中播放 bag 文件

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置参数
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"
RATE="${RATE:-1.0}"
USE_RVIZ="${USE_RVIZ:-true}"
CONTAINER_NAME="automap_player"
IMAGE_NAME="automap_pro:latest"

# 显示帮助
show_help() {
    cat << EOF
AutoMap-Pro Docker 数据播放脚本

用法: $0 [选项]

选项:
    -b, --bag FILE          ROS bag 文件路径
    -r, --rate RATE         回放速率 (默认: 1.0)
    --no-rviz               不启动 RViz
    --topics TOPICS          只播放指定的话题 (逗号分隔）
    -l, --loop              循环播放
    --pause                 暂停播放
    --clock                 发布 /clock
    --verbose               详细输出
    -h, --help               显示此帮助信息

示例:
    # 默认播放
    $0

    # 指定 bag 文件
    $0 -b /path/to/your/data.bag

    # 2倍速播放
    $0 -r 2.0

    # 不启动 RViz
    $0 --no-rviz

    # 循环播放
    $0 -l

环境变量:
    BAG_FILE         ROS bag 文件路径
    RATE             回放速率
    USE_RVIZ         是否启动 RViz (true/false)

注意:
    - 支持播放 ROS1 和 ROS2 格式的 bag
    - 使用 RViz 可视化
    - Ctrl+C 停止播放

EOF
}

# 解析命令行参数
USE_LOOP=false
USE_PAUSE=false
USE_CLOCK=false
VERBOSE=false
TOPICS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bag-file)
            BAG_FILE="$2"
            shift 2
            ;;
        -r|--rate)
            RATE="$2"
            shift 2
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        --topics)
            TOPICS="$2"
            shift 2
            ;;
        -l|--loop)
            USE_LOOP=true
            shift
            ;;
        --pause)
            USE_PAUSE=true
            shift
            ;;
        --clock)
            USE_CLOCK=true
            shift
            ;;
        --verbose)
            VERBOSE=true
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

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $@"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $@"
}

log_step() {
    echo -e "${CYAN}[STEP]${NC} $@"
}

# 打印标题
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# 检查 Docker
check_docker() {
    log_step "检查 Docker 环境"
    
    if ! command -v docker &> /dev/null; then
        log_error "Docker 未安装"
        log_error "请先安装 Docker: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    log_info "✓ Docker 已安装"
}

# 检查镜像
check_image() {
    log_step "检查 Docker 镜像"
    
    if docker images | grep -q "^$IMAGE_NAME "; then
        log_info "✓ 镜像已存在: $IMAGE_NAME"
    else
        log_warn "镜像不存在: $IMAGE_NAME"
        read -p "是否构建镜像? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "开始构建镜像..."
            docker build -t "$IMAGE_NAME" -f docker/Dockerfile . || {
                log_error "镜像构建失败"
                exit 1
            }
            log_info "✓ 镜像构建成功"
        else
            log_error "没有镜像，无法运行"
            exit 1
        fi
    fi
}

# 停止并删除旧容器
cleanup_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "停止并删除旧容器: $CONTAINER_NAME"
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
    fi
}

# 在容器中播放数据
play_data_in_container() {
    log_step "在 Docker 容器中播放数据"
    
    # 转换相对路径为绝对路径
    if [[ "$BAG_FILE" != /* ]]; then
        BAG_FILE_ABS="$SCRIPT_DIR/$BAG_FILE"
    else
        BAG_FILE_ABS="$BAG_FILE"
    fi
    
    # 检查 bag 文件是否存在
    if [ ! -f "$BAG_FILE_ABS" ]; then
        log_error "Bag 文件不存在: $BAG_FILE_ABS"
        log_info "搜索可用的 bag 文件..."
        local found_bags=$(find "$SCRIPT_DIR/data" -name "*.bag" -o -name "*.db3" 2>/dev/null)
        
        if [ -n "$found_bags" ]; then
            echo -e "${GREEN}找到以下 bag 文件:${NC}"
            echo "$found_bags" | head -10
        else
            log_error "未找到任何 bag 文件"
        fi
        
        exit 1
    fi
    
    # 转换为容器内路径
    BAG_FILE_CONTAINER="/workspace/data/$(basename "$BAG_FILE")"
    
    local bag_size=$(du -h "$BAG_FILE_ABS" | cut -f1)
    log_info "本地 Bag 文件: $BAG_FILE_ABS ($bag_size)"
    log_info "容器 Bag 文件: $BAG_FILE_CONTAINER"
    log_info "回放速率: $RATE"
    log_info "使用 RViz: $USE_RVIZ"
    
    # 构建挂载点
    local mounts=(
        "-v $SCRIPT_DIR:/workspace/automap_pro"
        "-v $SCRIPT_DIR/data:/workspace/data"
    )
    
    # 构建环境变量
    local envs=(
        "-e ROS_DOMAIN_ID=0"
        "-e BAG_FILE=$BAG_FILE_CONTAINER"
        "-e RATE=$RATE"
        "-e USE_RVIZ=$USE_RVIZ"
        "-e USE_LOOP=$USE_LOOP"
        "-e USE_PAUSE=$USE_PAUSE"
        "-e USE_CLOCK=$USE_CLOCK"
        "-e TOPICS=$TOPICS"
        "-e VERBOSE=$VERBOSE"
    )
    
    # 构建运行命令
    local play_cmd="docker run"
    
    # 添加容器名称
    play_cmd="$play_cmd --name $CONTAINER_NAME"
    
    # 添加交互式和TTY
    play_cmd="$play_cmd -it"
    
    # 添加挂载点
    play_cmd="$play_cmd ${mounts[@]}"
    
    # 添加环境变量
    play_cmd="$play_cmd ${envs[@]}"
    
    # 添加删除选项
    play_cmd="$play_cmd --rm"
    
    # 添加镜像
    play_cmd="$play_cmd $IMAGE_NAME"
    
    # 添加播放命令
    play_cmd="$play_cmd bash -c \"
        echo '========================================' &&
        echo 'AutoMap-Pro Docker 数据播放' &&
        echo '========================================' &&
        echo '' &&
        cd /workspace/automap_pro/docker &&
        source /opt/ros/humble/setup.bash &&
        echo '' &&
        echo '检查 bag 文件...' &&
        ls -lh \$BAG_FILE 2>/dev/null || echo 'ERROR: Bag file not found' &&
        echo '' &&
        echo '检查 bag 格式...' &&
        file \$BAG_FILE | head -1 &&
        echo '' &&
        echo '开始播放...' &&
        echo '' &&
        
        # 检查 bag 格式
        if file \$BAG_FILE | grep -q 'SQLite'; then
            echo '检测到 ROS2 格式'
            echo '========================================'
            ros2 bag play \$BAG_FILE --rate \$RATE \$USE_LOOP_ARG \$USE_PAUSE_ARG \$USE_CLOCK_ARG \$VERBOSE_ARG
        else
            echo '检测到 ROS1 格式'
            echo '========================================'
            rosbag play \$BAG_FILE --rate \$RATE \$USE_LOOP_ARG \$USE_PAUSE_ARG \$USE_CLOCK_ARG
        fi
    \""
    
    log_info "运行命令:"
    echo "$play_cmd" | sed 's/ -v /\n  -v /g' | sed 's/ -e /\n  -e /g'
    echo ""
    
    # 运行容器
    eval $play_cmd
}

# 主函数
main() {
    print_header "AutoMap-Pro Docker 数据播放脚本"
    
    # 显示配置
    echo -e "${GREEN}配置信息:${NC}"
    echo -e "${CYAN}----------------------------------------${NC}"
    echo -e "${GREEN}脚本目录:${NC}           $SCRIPT_DIR"
    echo -e "${GREEN}Bag 文件:${NC}            $BAG_FILE"
    echo -e "${GREEN}回放速率:${NC}            $RATE"
    echo -e "${GREEN}使用 RViz:${NC}           $USE_RVIZ"
    echo -e "${GREEN}循环播放:${NC}            $USE_LOOP"
    echo -e "${GREEN}暂停播放:${NC}            $USE_PAUSE"
    echo -e "${GREEN}指定话题:${NC}            ${TOPICS:-无}"
    echo -e "${GREEN}详细输出:${NC}            $VERBOSE"
    echo -e "${CYAN}----------------------------------------${NC}"
    
    # 检查 Docker
    check_docker
    
    # 检查镜像
    check_image
    
    # 清理旧容器
    cleanup_container
    
    # 确认开始
    if [ "$VERBOSE" = false ]; then
        read -p "是否开始播放? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "用户取消"
            exit 0
        fi
    fi
    
    # 运行播放
    play_data_in_container
    
    # 完成
    print_header "播放完成！"
    echo ""
}

# 运行主函数
main "$@"
