#!/bin/bash
# AutoMap-Pro 完整建图一键脚本
# 功能：自动完成环境检查、编译、bag转换、建图、保存地图等所有步骤

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
CONFIG="${CONFIG:-automap_pro/config/system_config_nya02.yaml}"
OUTPUT_DIR="${OUTPUT_DIR:-/data/automap_output/nya_02}"
WORKSPACE="${HOME}/automap_ws"
LOG_DIR="${SCRIPT_DIR}/logs"

# 显示帮助信息
show_help() {
    cat << EOF
AutoMap-Pro 完整建图一键脚本

用法: $0 [选项]

选项:
    -b, --bag FILE          ROS bag 文件路径
    -c, --config FILE        配置文件路径
    -o, --output-dir DIR     输出目录
    --no-compile            跳过编译步骤
    --no-convert            跳过bag转换步骤
    --no-mapping            跳过建图步骤
    --no-save               跳过保存地图步骤
    --clean                 清理之前的工作空间
    --verbose               详细输出
    -h, --help               显示此帮助信息

示例:
    # 默认配置运行
    $0

    # 指定 bag 文件
    $0 -b /path/to/data.bag

    # 跳过编译（已编译）
    $0 --no-compile

    # 跳过 bag 转换（已经是 ROS2）
    $0 --no-convert

    # 详细模式
    $0 --verbose

环境变量:
    BAG_FILE         ROS bag 文件路径
    CONFIG           配置文件路径
    OUTPUT_DIR       输出目录

步骤:
    1. 环境检查
    2. 编译项目（可选）
    3. 转换 bag（如果是 ROS1）
    4. 启动建图
    5. 保存地图
    6. 显示结果

EOF
}

# 解析命令行参数
SKIP_COMPILE=false
SKIP_CONVERT=false
SKIP_MAPPING=false
SKIP_SAVE=false
CLEAN_WORKSPACE=false
VERBOSE=false

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
        --no-compile)
            SKIP_COMPILE=true
            shift
            ;;
        --no-convert)
            SKIP_CONVERT=true
            shift
            ;;
        --no-mapping)
            SKIP_MAPPING=true
            shift
            ;;
        --no-save)
            SKIP_SAVE=true
            shift
            ;;
        --clean)
            CLEAN_WORKSPACE=true
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

# 创建日志目录
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/full_mapping_$(date +%Y%m%d_%H%M%S).log"

# 日志函数
log() {
    local level=$1
    shift
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo -e "[$timestamp] $level $message" | tee -a "$LOG_FILE"
}

log_info() {
    log "${GREEN}[INFO]${NC}" "$@"
}

log_warn() {
    log "${YELLOW}[WARN]${NC}" "$@"
}

log_error() {
    log "${RED}[ERROR]${NC}" "$@"
}

log_step() {
    local step=$1
    local total=$2
    local name=$3
    log "${CYAN}[STEP $step/$total]${NC}" "$name"
}

# 打印标题
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# 打印配置
print_config() {
    echo -e "${GREEN}AutoMap-Pro 完整建图配置${NC}"
    echo -e "${CYAN}----------------------------------------${NC}"
    echo -e "${GREEN}脚本目录:${NC}           $SCRIPT_DIR"
    echo -e "${GREEN}Bag 文件:${NC}            $BAG_FILE"
    echo -e "${GREEN}配置文件:${NC}            $CONFIG"
    echo -e "${GREEN}输出目录:${NC}            $OUTPUT_DIR"
    echo -e "${GREEN}工作空间:${NC}            $WORKSPACE"
    echo -e "${GREEN}日志文件:${NC}            $LOG_FILE"
    echo -e "${CYAN}----------------------------------------${NC}"
    
    # 选项状态
    echo -e "${GREEN}跳过编译:${NC}           $SKIP_COMPILE"
    echo -e "${GREEN}跳过转换:${NC}           $SKIP_CONVERT"
    echo -e "${GREEN}跳过建图:${NC}           $SKIP_MAPPING"
    echo -e "${GREEN}跳过保存:${NC}           $SKIP_SAVE"
    echo -e "${GREEN}清理工作空间:${NC}        $CLEAN_WORKSPACE"
    echo -e "${GREEN}详细输出:${NC}            $VERBOSE"
    echo -e "${CYAN}========================================${NC}\n"
}

# 步骤1: 环境检查
check_environment() {
    print_header "步骤 1/6: 环境检查"
    
    # 检查 ROS2
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        log_error "ROS2 Humble 未安装"
        log_error "请先安装 ROS2 Humble"
        exit 1
    fi
    log_info "✓ ROS2 Humble 已安装"
    
    # 检查 Docker（用于 bag 转换）
    if ! command -v docker &> /dev/null; then
        log_warn "Docker 未安装，bag 转换可能受限"
    else
        log_info "✓ Docker 已安装"
    fi
    
    # 检查 Python3
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 未安装"
        exit 1
    fi
    log_info "✓ Python3 已安装: $(python3 --version)"
    
    # 检查 colcon
    if ! command -v colcon &> /dev/null; then
        log_error "colcon 未安装"
        exit 1
    fi
    log_info "✓ colcon 已安装"
    
    # 检查 bag 文件
    if [ ! -f "$BAG_FILE" ]; then
        log_error "Bag 文件不存在: $BAG_FILE"
        exit 1
    fi
    local bag_size=$(du -h "$BAG_FILE" | cut -f1)
    log_info "✓ Bag 文件存在: $BAG_FILE ($bag_size)"
    
    # 检查配置文件
    if [ ! -f "$CONFIG" ]; then
        log_error "配置文件不存在: $CONFIG"
        exit 1
    fi
    log_info "✓ 配置文件存在: $CONFIG"
    
    # 检查工作空间
    if [ ! -d "$WORKSPACE" ]; then
        log_warn "工作空间不存在，将创建"
    else
        log_info "✓ 工作空间存在: $WORKSPACE"
    fi
    
    # 创建输出目录
    mkdir -p "$OUTPUT_DIR"
    log_info "✓ 输出目录已创建: $OUTPUT_DIR"
    
    log_info "环境检查完成"
}

# 步骤2: 编译项目
compile_project() {
    if [ "$SKIP_COMPILE" = true ]; then
        log_info "跳过编译步骤"
        return 0
    fi
    
    print_header "步骤 2/6: 编译项目"
    
    # 清理工作空间（如果需要）
    if [ "$CLEAN_WORKSPACE" = true ]; then
        log_info "清理工作空间..."
        cd "$SCRIPT_DIR"
        make clean 2>&1 | tee -a "$LOG_FILE" || true
    fi
    
    # 设置工作空间
    log_info "设置工作空间..."
    cd "$SCRIPT_DIR"
    make setup 2>&1 | tee -a "$LOG_FILE" || {
        log_error "工作空间设置失败"
        exit 1
    }
    
    # 编译项目
    log_info "编译项目（Release 模式）..."
    make build-release 2>&1 | tee -a "$LOG_FILE" || {
        log_error "项目编译失败"
        exit 1
    }
    
    log_info "✓ 项目编译完成"
}

# 步骤3: 转换 bag（如果是 ROS1）
convert_bag() {
    if [ "$SKIP_CONVERT" = true ]; then
        log_info "跳过 bag 转换步骤"
        return 0
    fi
    
    print_header "步骤 3/6: 检查并转换 Bag"
    
    # 检查 bag 格式
    log_info "检查 bag 格式..."
    local file_type=$(file "$BAG_FILE" | cut -d: -f2)
    
    if echo "$file_type" | grep -q "SQLite"; then
        log_info "✓ 检测到 ROS2 格式，无需转换"
        BAG_FILE_ROS2="$BAG_FILE"
    elif echo "$file_type" | grep -q "data"; then
        log_info "✓ 检测到 ROS1 格式，需要转换"
        
        # 确定输出目录
        local bag_dir=$(dirname "$BAG_FILE")
        local bag_name=$(basename "$BAG_FILE" .bag)
        BAG_FILE_ROS2="$bag_dir/${bag_name}_ros2"
        
        # 检查是否已经转换
        if [ -d "$BAG_FILE_ROS2" ]; then
            log_warn "转换后的 bag 已存在: $BAG_FILE_ROS2"
            read -p "是否重新转换? (y/n): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                log_info "使用已转换的 bag"
                return 0
            fi
            rm -rf "$BAG_FILE_ROS2"
        fi
        
        # 方法1: 使用 Docker 转换（推荐）
        if command -v docker &> /dev/null; then
            log_info "使用 Docker 转换 bag..."
            
            # 检查是否已构建转换镜像
            if ! docker images | grep -q "ros1-to-ros2-converter"; then
                log_info "构建转换镜像..."
                cd "$SCRIPT_DIR"
                docker build -t ros1-to-ros2-converter -f docker/converter.Dockerfile . \
                    2>&1 | tee -a "$LOG_FILE" || {
                    log_error "转换镜像构建失败"
                    exit 1
                }
            fi
            
            # 运行转换
            log_info "运行 bag 转换..."
            docker run --rm \
                -v "$SCRIPT_DIR/data:/workspace/data" \
                ros1-to-ros2-converter \
                bash -c "
                    source /opt/ros/humble/setup.bash &&
                    ros2 bag convert /workspace/$BAG_FILE /workspace/$BAG_FILE_ROS2
                " 2>&1 | tee -a "$LOG_FILE" || {
                log_error "Bag 转换失败"
                exit 1
            }
            
            log_info "✓ Bag 转换完成"
        else
            log_warn "Docker 不可用，跳过 bag 转换"
            log_warn "请手动转换 bag 或安装 Docker"
            read -p "是否继续使用原始 bag? (y/n): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
            BAG_FILE_ROS2="$BAG_FILE"
        fi
    else
        log_error "无法识别的 bag 格式: $file_type"
        exit 1
    fi
}

# 步骤4: 启动建图
run_mapping() {
    if [ "$SKIP_MAPPING" = true ]; then
        log_info "跳过建图步骤"
        return 0
    fi
    
    print_header "步骤 4/6: 启动建图"
    
    log_info "Source ROS2 和工作空间..."
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE/install/setup.bash"
    
    log_info "启动 AutoMap-Pro 建图..."
    log_info "Bag 文件: $BAG_FILE_ROS2"
    log_info "配置文件: $CONFIG"
    log_info "输出目录: $OUTPUT_DIR"
    
    # 启动建图（使用 launch 文件）
    local bag_arg=""
    if [ -f "$BAG_FILE_ROS2" ]; then
        bag_arg="bag_file:=$BAG_FILE_ROS2"
    elif [ -d "$BAG_FILE_ROS2" ]; then
        bag_arg="bag_file:=$BAG_FILE_ROS2"
    else
        bag_arg="bag_file:=$BAG_FILE"
    fi
    
    # 启动建图
    cd "$WORKSPACE"
    ros2 launch automap_pro automap_offline.launch.py \
        config:="$SCRIPT_DIR/$CONFIG" \
        $bag_arg \
        rate:=1.0 \
        use_rviz:=true \
        use_external_frontend:=true \
        2>&1 | tee -a "$LOG_FILE"
    
    log_info "✓ 建图完成"
}

# 步骤5: 保存地图
save_map() {
    if [ "$SKIP_SAVE" = true ]; then
        log_info "跳过保存地图步骤"
        return 0
    fi
    
    print_header "步骤 5/6: 保存地图"
    
    # Source ROS2 和工作空间
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE/install/setup.bash"
    
    log_info "保存地图到: $OUTPUT_DIR"
    
    # 调用保存服务
    ros2 service call /automap/save_map automap_pro/srv/SaveMap \
        "{output_dir: '$OUTPUT_DIR', save_pcd: true, save_ply: true, save_las: false, save_trajectory: true}" \
        2>&1 | tee -a "$LOG_FILE" || {
        log_warn "保存地图服务调用失败，可能需要手动保存"
        return 0
    }
    
    log_info "✓ 地图已保存"
}

# 步骤6: 显示结果
show_results() {
    print_header "步骤 6/6: 显示结果"
    
    log_info "输出目录: $OUTPUT_DIR"
    
    # 检查输出目录
    if [ -d "$OUTPUT_DIR" ]; then
        echo -e "\n${GREEN}输出目录内容:${NC}"
        ls -lh "$OUTPUT_DIR"
        
        # 检查地图文件
        if [ -f "$OUTPUT_DIR/map/global_map.pcd" ]; then
            local map_size=$(du -h "$OUTPUT_DIR/map/global_map.pcd" | cut -f1)
            log_info "✓ 地图文件: global_map.pcd ($map_size)"
        fi
        
        if [ -f "$OUTPUT_DIR/map/global_map.ply" ]; then
            local map_size=$(du -h "$OUTPUT_DIR/map/global_map.ply" | cut -f1)
            log_info "✓ 地图文件: global_map.ply ($map_size)"
        fi
        
        # 检查轨迹文件
        if [ -f "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt" ]; then
            local traj_lines=$(wc -l < "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt")
            log_info "✓ 轨迹文件: optimized_trajectory_tum.txt ($traj_lines 行)"
        fi
        
        # 检查子图
        if [ -d "$OUTPUT_DIR/submaps" ]; then
            local submap_count=$(find "$OUTPUT_DIR/submaps" -maxdepth 1 -type d | wc -l)
            log_info "✓ 子图数量: $submap_count"
        fi
    else
        log_warn "输出目录不存在或为空"
    fi
}

# 主函数
main() {
    print_header "AutoMap-Pro 完整建图一键脚本"
    
    # 显示配置
    print_config
    
    # 确认开始
    if [ "$VERBOSE" = true ]; then
        log_info "详细模式已启用"
    else
        read -p "是否开始建图? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "用户取消"
            exit 0
        fi
    fi
    
    # 执行步骤
    check_environment
    compile_project
    convert_bag
    run_mapping
    save_map
    show_results
    
    # 完成
    print_header "建图完成！"
    log_info "日志文件: $LOG_FILE"
    log_info "输出目录: $OUTPUT_DIR"
    
    # 询问是否可视化
    if command -v python3 &> /dev/null; then
        echo -e "\n${YELLOW}是否可视化结果? (y/n):${NC}"
        read -p "" -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "启动可视化..."
            python3 "$SCRIPT_DIR/automap_pro/scripts/visualize_results.py" \
                --output_dir "$OUTPUT_DIR" || {
                log_warn "可视化失败"
            }
        fi
    fi
    
    echo -e "${GREEN}所有步骤已完成！${NC}\n"
}

# 运行主函数
main "$@"
