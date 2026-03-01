#!/bin/bash
# AutoMap-Pro 数据播放一键脚本
# 功能：播放 bag 文件并启动可视化

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
START_TIME="${START_TIME:-0}"
DURATION="${DURATION:-}"

# 显示帮助信息
show_help() {
    cat << EOF
AutoMap-Pro 数据播放一键脚本

用法: $0 [选项]

选项:
    -b, --bag FILE          ROS bag 文件路径
    -r, --rate RATE         回放速率 (默认: 1.0)
    --start TIME             开始时间 (秒，默认: 0)
    --duration SEC           播放时长 (秒，默认: 全部）
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
    $0 -b /path/to/data.bag

    # 2倍速播放
    $0 -r 2.0

    # 只播放 LiDAR 和 IMU
    $0 --topics /os1_cloud_node1/points,/imu/imu

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
    - 对于 ROS1 格式，会使用 rosbag 命令
    - 对于 ROS2 格式，会使用 ros2 bag play 命令
    - 播放 ROS1 格式需要 ROS1 环境支持

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
        --start)
            START_TIME="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
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

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $@"
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

# 检查环境
check_environment() {
    log_step "检查环境"
    
    # 检查 bag 文件
    if [ ! -f "$BAG_FILE" ]; then
        log_error "Bag 文件不存在: $BAG_FILE"
        exit 1
    fi
    local bag_size=$(du -h "$BAG_FILE" | cut -f1)
    log_info "✓ Bag 文件: $BAG_FILE ($bag_size)"
    
    # 检查 bag 格式
    local file_type=$(file "$BAG_FILE" | cut -d: -f2)
    if echo "$file_type" | grep -q "SQLite"; then
        BAG_FORMAT="ros2"
        log_info "✓ 检测到 ROS2 格式"
    elif echo "$file_type" | grep -q "data"; then
        BAG_FORMAT="ros1"
        log_info "✓ 检测到 ROS1 格式"
    else
        log_error "无法识别的 bag 格式: $file_type"
        exit 1
    fi
    
    # 检查 ROS2 环境
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_info "✓ ROS2 Humble 已安装"
        ROS2_AVAILABLE=true
    else
        log_warn "ROS2 Humble 未安装"
        ROS2_AVAILABLE=false
    fi
    
    # 检查 ROS1 环境
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        log_info "✓ ROS1 Noetic 已安装"
        ROS1_AVAILABLE=true
    else
        log_warn "ROS1 Noetic 未安装"
        ROS1_AVAILABLE=false
    fi
    
    # 检查 RViz
    if [ "$USE_RVIZ" = true ]; then
        if [ "$ROS2_AVAILABLE" = true ]; then
            if command -v rviz2 &> /dev/null; then
                log_info "✓ RViz2 可用"
            else
                log_warn "RViz2 不可用"
            fi
        fi
    fi
}

# 显示 bag 信息
show_bag_info() {
    print_header "Bag 信息"
    
    if [ "$BAG_FORMAT" = "ros2" ]; then
        if [ "$ROS2_AVAILABLE" = true ]; then
            log_info "读取 ROS2 bag 信息..."
            source /opt/ros/humble/setup.bash
            ros2 bag info "$BAG_FILE" || true
        fi
    elif [ "$BAG_FORMAT" = "ros1" ]; then
        if [ "$ROS1_AVAILABLE" = true ]; then
            log_info "读取 ROS1 bag 信息..."
            source /opt/ros/noetic/setup.bash
            rosbag info "$BAG_FILE" || true
        fi
    fi
}

# 播放 ROS2 bag
play_ros2_bag() {
    if [ "$ROS2_AVAILABLE" != true ]; then
        log_error "ROS2 环境不可用"
        return 1
    fi
    
    source /opt/ros/humble/setup.bash
    
    log_step "播放 ROS2 bag"
    log_info "Bag 文件: $BAG_FILE"
    log_info "回放速率: $RATE"
    
    # 构建播放命令
    local cmd="ros2 bag play $BAG_FILE --rate $RATE"
    
    # 添加选项
    if [ -n "$START_TIME" ] && [ "$START_TIME" != "0" ]; then
        cmd="$cmd --start-offset $START_TIME"
        log_info "开始时间: $START_TIME 秒"
    fi
    
    if [ -n "$DURATION" ]; then
        cmd="$cmd --duration $DURATION"
        log_info "播放时长: $DURATION 秒"
    fi
    
    if [ "$USE_LOOP" = true ]; then
        cmd="$cmd --loop"
        log_info "循环播放"
    fi
    
    if [ "$USE_PAUSE" = true ]; then
        cmd="$cmd --clock"
        log_info "暂停播放（使用 --clock）"
    fi
    
    if [ "$USE_CLOCK" = true ]; then
        cmd="$cmd --clock"
        log_info "发布 /clock"
    fi
    
    if [ -n "$TOPICS" ]; then
        cmd="$cmd --topics $TOPICS"
        log_info "指定话题: $TOPICS"
    fi
    
    if [ "$VERBOSE" = true ]; then
        cmd="$cmd --verbose"
    fi
    
    # 启动 RViz（如果需要）
    local rviz_pid=""
    if [ "$USE_RVIZ" = true ] && command -v rviz2 &> /dev/null; then
        log_info "启动 RViz2..."
        rviz2 &
        rviz_pid=$!
        log_info "RViz2 PID: $rviz_pid"
        sleep 2
    fi
    
    # 播放 bag
    log_info "开始播放..."
    echo -e "${CYAN}========================================${NC}"
    eval $cmd
    echo -e "${CYAN}========================================${NC}"
    
    # 等待 RViz（如果启动）
    if [ -n "$rviz_pid" ]; then
        wait $rviz_pid 2>/dev/null || true
    fi
}

# 播放 ROS1 bag
play_ros1_bag() {
    if [ "$ROS1_AVAILABLE" != true ]; then
        log_error "ROS1 环境不可用"
        return 1
    fi
    
    source /opt/ros/noetic/setup.bash
    
    log_step "播放 ROS1 bag"
    log_info "Bag 文件: $BAG_FILE"
    log_info "回放速率: $RATE"
    
    # 构建播放命令
    local cmd="rosbag play $BAG_FILE --rate $RATE"
    
    # 添加选项
    if [ -n "$START_TIME" ] && [ "$START_TIME" != "0" ]; then
        cmd="$cmd --start $START_TIME"
        log_info "开始时间: $START_TIME 秒"
    fi
    
    if [ -n "$DURATION" ]; then
        cmd="$cmd --duration $DURATION"
        log_info "播放时长: $DURATION 秒"
    fi
    
    if [ "$USE_LOOP" = true ]; then
        cmd="$cmd --loop"
        log_info "循环播放"
    fi
    
    if [ "$USE_PAUSE" = true ]; then
        cmd="$cmd --pause"
        log_info "暂停播放"
    fi
    
    if [ "$USE_CLOCK" = true ]; then
        cmd="$cmd --clock"
        log_info "发布 /clock"
    fi
    
    if [ -n "$TOPICS" ]; then
        cmd="$cmd --topics"
        log_info "指定话题: $TOPICS"
        # ROS1 使用不同的语法
        local IFS=','
        read -ra ADDR <<< "$TOPICS"
        for topic in "${ADDR[@]}"; do
            cmd="$cmd $topic"
        done
    fi
    
    if [ "$VERBOSE" = true ]; then
        cmd="$cmd --verbose"
    fi
    
    # 启动 RViz（如果需要）
    local rviz_pid=""
    if [ "$USE_RVIZ" = true ] && command -v rviz &> /dev/null; then
        log_info "启动 RViz..."
        rviz &
        rviz_pid=$!
        log_info "RViz PID: $rviz_pid"
        sleep 2
    fi
    
    # 播放 bag
    log_info "开始播放..."
    echo -e "${CYAN}========================================${NC}"
    eval $cmd
    echo -e "${CYAN}========================================${NC}"
    
    # 等待 RViz（如果启动）
    if [ -n "$rviz_pid" ]; then
        wait $rviz_pid 2>/dev/null || true
    fi
}

# 显示配置
show_config() {
    print_header "播放配置"
    echo -e "${GREEN}Bag 文件:${NC}            $BAG_FILE"
    echo -e "${GREEN}Bag 格式:${NC}            $BAG_FORMAT"
    echo -e "${GREEN}回放速率:${NC}            $RATE"
    echo -e "${GREEN}使用 RViz:${NC}           $USE_RVIZ"
    echo -e "${GREEN}开始时间:${NC}            $START_TIME"
    echo -e "${GREEN}播放时长:${NC}            ${DURATION:-全部}"
    echo -e "${GREEN}循环播放:${NC}            $USE_LOOP"
    echo -e "${GREEN}暂停播放:${NC}            $USE_PAUSE"
    echo -e "${GREEN}发布 /clock:${NC}         $USE_CLOCK"
    echo -e "${GREEN}指定话题:${NC}            ${TOPICS:-无}"
    echo -e "${GREEN}详细输出:${NC}            $VERBOSE"
}

# 控制信息
show_controls() {
    if [ "$BAG_FORMAT" = "ros2" ]; then
        cat << EOF
${YELLOW}控制信息:${NC}
  ${GREEN}Ctrl+C${NC}    - 停止播放
  ${GREEN}空格键${NC}    - 暂停/继续

EOF
    else
        cat << EOF
${YELLOW}控制信息:${NC}
  ${GREEN}Ctrl+C${NC}    - 停止播放
  ${GREEN}空格键${NC}    - 暂停/继续

EOF
    fi
}

# 主函数
main() {
    print_header "AutoMap-Pro 数据播放脚本"
    
    # 检查环境
    check_environment
    
    # 显示 bag 信息
    show_bag_info
    
    # 显示配置
    show_config
    
    # 确认开始
    if [ "$VERBOSE" = false ]; then
        read -p "是否开始播放? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "用户取消"
            exit 0
        fi
    fi
    
    # 显示控制信息
    show_controls
    
    # 播放 bag
    if [ "$BAG_FORMAT" = "ros2" ]; then
        play_ros2_bag
    elif [ "$BAG_FORMAT" = "ros1" ]; then
        play_ros1_bag
    fi
    
    # 完成
    echo -e "\n${GREEN}播放完成！${NC}\n"
}

# 运行主函数
main "$@"
