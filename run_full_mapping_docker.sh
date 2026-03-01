#!/bin/bash
# AutoMap-Pro Docker 完整建图一键脚本
# 功能：在 Docker 容器中运行完整建图流程

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

# 处理 BAG_FILE 参数（去掉可能的 @ 符号）
if [[ "$BAG_FILE" == @* ]]; then
    BAG_FILE="${BAG_FILE#@}"
fi

BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"
CONFIG="${CONFIG:-automap_pro/config/system_config_nya02.yaml}"
OUTPUT_DIR="${OUTPUT_DIR:-/data/automap_output/nya_02}"
CONTAINER_NAME="automap_mapping"
IMAGE_NAME="automap_pro:latest"

# 显示帮助信息
show_help() {
    cat << EOF
AutoMap-Pro Docker 完整建图一键脚本

用法: $0 [选项]

选项:
    -b, --bag FILE          ROS bag 文件路径（相对或绝对）
    -c, --config FILE        配置文件路径
    -o, --output-dir DIR     输出目录
    --build                 重新构建 Docker 镜像
    --no-build               不检查镜像是否存在
    --keep-container         保留容器（不自动删除）
    --detach                后台运行
    --shell                 启动容器 shell（不自动运行建图）
    -h, --help               显示此帮助信息

示例:
    # 默认配置运行（推荐）
    $0

    # 重新构建镜像并运行
    $0 --build

    # 后台运行
    $0 --detach

    # 启动容器 shell（调试用）
    $0 --shell

    # 指定 bag 文件
    $0 -b data/automap_input/your_data.bag

环境变量:
    BAG_FILE         ROS bag 文件路径
    CONFIG           配置文件路径
    OUTPUT_DIR       输出目录

注意:
    - 此脚本会在 Docker 容器中运行建图流程
    - 容器会自动挂载数据目录
    - 建图完成后，结果会保存到本地 /data 目录
    - 使用 --shell 选项可以进入容器进行调试

EOF
}

# 解析命令行参数
BUILD_IMAGE=false
NO_BUILD=false
KEEP_CONTAINER=false
DETACH=false
START_SHELL=false

while [[ $# -gt 0 ]]; do
    case $1 in
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
        --build)
            BUILD_IMAGE=true
            shift
            ;;
        --no-build)
            NO_BUILD=true
            shift
            ;;
        --keep-container)
            KEEP_CONTAINER=true
            shift
            ;;
        --detach)
            DETACH=true
            shift
            ;;
        --shell)
            START_SHELL=true
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

# 检查 Docker
check_docker() {
    log_step "检查 Docker 环境"
    
    if ! command -v docker &> /dev/null; then
        log_error "Docker 未安装"
        log_error "请先安装 Docker: https://docs.docker.com/get-docker/"
        exit 1
    fi
    
    log_info "✓ Docker 已安装: $(docker --version)"
    
    # 检查 Docker 是否运行
    if ! docker info &> /dev/null; then
        log_error "Docker 守护进程未运行"
        log_error "请启动 Docker: sudo systemctl start docker"
        exit 1
    fi
    
    log_info "✓ Docker 守护进程正在运行"
}

# 检查镜像
check_image() {
    if [ "$NO_BUILD" = true ]; then
        return 0
    fi
    
    log_step "检查 Docker 镜像"
    
    if docker images | grep -q "^$IMAGE_NAME "; then
        log_info "✓ 镜像已存在: $IMAGE_NAME"
    else
        log_warn "镜像不存在: $IMAGE_NAME"
        read -p "是否构建镜像? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            BUILD_IMAGE=true
        else
            log_error "没有镜像，无法运行"
            exit 1
        fi
    fi
}

# 构建 Docker 镜像
build_image() {
    if [ "$BUILD_IMAGE" = false ]; then
        return 0
    fi
    
    log_step "构建 Docker 镜像"
    
    # 检查 Dockerfile
    if [ ! -f "docker/Dockerfile" ]; then
        log_error "Dockerfile 不存在: docker/Dockerfile"
        exit 1
    fi
    
    log_info "开始构建镜像..."
    docker build -t "$IMAGE_NAME" -f docker/Dockerfile . || {
        log_error "镜像构建失败"
        exit 1
    }
    
    log_info "✓ 镜像构建成功: $IMAGE_NAME"
}

# 停止并删除旧容器
cleanup_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        log_info "停止并删除旧容器: $CONTAINER_NAME"
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
    fi
}

# 运行容器并执行建图
run_mapping_in_container() {
    log_step "在 Docker 容器中运行建图"
    
    # 转换相对路径为绝对路径
    if [[ "$BAG_FILE" != /* ]]; then
        BAG_FILE_ABS="$SCRIPT_DIR/$BAG_FILE"
    else
        BAG_FILE_ABS="$BAG_FILE"
    fi
    
    if [[ "$CONFIG" != /* ]]; then
        CONFIG_ABS="$SCRIPT_DIR/$CONFIG"
    else
        CONFIG_ABS="$CONFIG"
    fi
    
    # 输出目录（本地）
    OUTPUT_DIR_LOCAL="$OUTPUT_DIR"
    
    # 检查 bag 文件是否存在
    if [ ! -f "$BAG_FILE_ABS" ]; then
        log_error "Bag 文件不存在: $BAG_FILE_ABS"
        log_info "可用的 bag 文件:"
        find "$SCRIPT_DIR/data" -name "*.bag" -o -name "*.db3" 2>/dev/null || true
        exit 1
    fi
    
    # 转换为容器内路径
    # bag 文件直接挂载到 /workspace/data
    BAG_FILE_CONTAINER="/workspace/data/$(basename "$BAG_FILE")"
    # 配置文件在 /workspace/automap_pro
    CONFIG_CONTAINER="/workspace/automap_pro/$(basename "$CONFIG_ABS")"
    # 输出目录挂载到 /workspace/output
    OUTPUT_DIR_CONTAINER="/workspace/output"
    
    log_info "本地 Bag 文件: $BAG_FILE_ABS"
    log_info "容器 Bag 文件: $BAG_FILE_CONTAINER"
    log_info "本地配置文件: $CONFIG_ABS"
    log_info "容器配置文件: $CONFIG_CONTAINER"
    log_info "本地输出目录: $OUTPUT_DIR_LOCAL"
    log_info "容器输出目录: $OUTPUT_DIR_CONTAINER"
    
    # 创建本地输出目录
    mkdir -p "$OUTPUT_DIR_LOCAL"
    
    # 构建挂载点
    local mounts=(
        "-v $SCRIPT_DIR:/workspace/automap_pro"
        "-v $SCRIPT_DIR/data:/workspace/data"
        "-v $OUTPUT_DIR_LOCAL:/workspace/output"
    )
    
    # 构建环境变量
    local envs=(
        "-e ROS_DOMAIN_ID=0"
        "-e BAG_FILE=$BAG_FILE_CONTAINER"
        "-e CONFIG=$CONFIG_CONTAINER"
        "-e OUTPUT_DIR=$OUTPUT_DIR_CONTAINER"
    )
    
    # 构建运行命令
    local run_cmd="docker run"
    
    # 添加容器名称
    run_cmd="$run_cmd --name $CONTAINER_NAME"
    
    # 添加交互式和TTY
    if [ "$DETACH" = false ]; then
        run_cmd="$run_cmd -it"
    fi
    
    # 添加挂载点
    run_cmd="$run_cmd ${mounts[@]}"
    
    # 添加环境变量
    run_cmd="$run_cmd ${envs[@]}"
    
    # 添加删除选项
    if [ "$KEEP_CONTAINER" = false ]; then
        run_cmd="$run_cmd --rm"
    fi
    
    # 添加镜像
    run_cmd="$run_cmd $IMAGE_NAME"
    
    # 添加命令
    if [ "$START_SHELL" = true ]; then
        run_cmd="$run_cmd /bin/bash"
    else
        run_cmd="$run_cmd bash -c \"
            echo '========================================' &&
            echo 'AutoMap-Pro Docker 建图' &&
            echo '========================================' &&
            cd /workspace/automap_pro &&
            source /opt/ros/humble/setup.bash &&
            source /workspace/install/setup.bash &&
            echo '' &&
            echo '开始建图...' &&
            echo '' &&
            ./run_full_mapping.sh \\
                --bag-file $BAG_FILE_CONTAINER \\
                --config $CONFIG_CONTAINER \\
                --output-dir $OUTPUT_DIR_CONTAINER \\
                --verbose
        \""
    fi
    
    log_info "运行命令:"
    echo "$run_cmd" | sed 's/ -v /\n  -v /g' | sed 's/ -e /\n  -e /g'
    echo ""
    
    # 运行容器
    eval $run_cmd
}

# 显示结果
show_results() {
    if [ "$DETACH" = true ] || [ "$START_SHELL" = true ]; then
        return 0
    fi
    
    print_header "查看结果"
    
    if [ -d "$OUTPUT_DIR" ]; then
        log_info "输出目录: $OUTPUT_DIR"
        echo ""
        ls -lh "$OUTPUT_DIR"
    else
        log_warn "输出目录不存在或为空"
    fi
}

# 主函数
main() {
    print_header "AutoMap-Pro Docker 完整建图一键脚本"
    
    # 显示配置
    echo -e "${GREEN}配置信息:${NC}"
    echo -e "${CYAN}----------------------------------------${NC}"
    echo -e "${GREEN}脚本目录:${NC}           $SCRIPT_DIR"
    echo -e "${GREEN}Bag 文件:${NC}            $BAG_FILE"
    echo -e "${GREEN}配置文件:${NC}            $CONFIG"
    echo -e "${GREEN}输出目录:${NC}            $OUTPUT_DIR"
    echo -e "${GREEN}容器名称:${NC}            $CONTAINER_NAME"
    echo -e "${GREEN}镜像名称:${NC}            $IMAGE_NAME"
    echo -e "${CYAN}----------------------------------------${NC}"
    
    # 检查 Docker
    check_docker
    
    # 检查镜像
    check_image
    
    # 构建镜像（如果需要）
    build_image
    
    # 清理旧容器
    cleanup_container
    
    # 确认开始
    if [ "$START_SHELL" = false ]; then
        read -p "是否开始建图? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "用户取消"
            exit 0
        fi
    fi
    
    # 运行建图
    run_mapping_in_container
    
    # 显示结果
    show_results
    
    # 完成
    print_header "Docker 建图完成！"
    
    if [ "$START_SHELL" = true ]; then
        log_info "您现在在容器 shell 中"
        log_info "退出容器后，建图将停止"
    else
        log_info "结果已保存到: $OUTPUT_DIR"
    fi
    
    echo ""
}

# 运行主函数
main "$@"
