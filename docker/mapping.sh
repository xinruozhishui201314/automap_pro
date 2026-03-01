#!/bin/bash
# AutoMap-Pro Docker 一键建图脚本
# 在 Docker 容器中执行建图流程

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置参数
BAG_FILE="${BAG_FILE:-/workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"
CONFIG="${CONFIG:-/workspace/automap_pro/config/system_config_nya02.yaml}"
OUTPUT_DIR="${OUTPUT_DIR:-/workspace/data/automap_output/nya_02}"

# 显示帮助
show_help() {
    cat << EOF
AutoMap-Pro Docker 一键建图脚本

用法: $0 [选项]

选项:
    -b, --bag FILE          ROS bag 文件路径
    -c, --config FILE        配置文件路径
    -o, --output-dir DIR     输出目录
    -n, --no-compile        跳过编译
    -v, --verbose           详细输出
    -h, --help              显示帮助信息

示例:
    # 默认配置
    $0

    # 指定 bag 文件
    $0 -b /workspace/data/your_bag.bag

    # 跳过编译（已编译）
    $0 -n

环境变量:
    BAG_FILE         bag 文件路径
    CONFIG           配置文件路径
    OUTPUT_DIR       输出目录
EOF
}

# 解析参数
NO_COMPILE=false
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
        -n|--no-compile)
            NO_COMPILE=true
            shift
            ;;
        -v|--verbose)
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

log_step() {
    echo -e "${CYAN}[STEP]${NC} $@"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $@"
}

# 打印配置
print_config() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}AutoMap-Pro Docker 建图配置${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}Bag 文件:${NC}  $BAG_FILE"
    echo -e "${GREEN}配置文件:${NC}  $CONFIG"
    echo -e "${GREEN}输出目录:${NC}  $OUTPUT_DIR"
    echo -e "${BLUE}========================================${NC}\n"
}

# 检查 bag 文件
check_bag_file() {
    log_step "检查 bag 文件"
    
    if [ ! -f "$BAG_FILE" ]; then
        log_error "Bag 文件不存在: $BAG_FILE"
        exit 1
    fi
    
    local bag_size=$(du -h "$BAG_FILE" | cut -f1)
    log_info "✓ Bag 文件存在: $BAG_FILE ($bag_size)"
}

# 检查配置文件
check_config_file() {
    log_step "检查配置文件"
    
    if [ ! -f "$CONFIG" ]; then
        log_error "配置文件不存在: $CONFIG"
        exit 1
    fi
    
    log_info "✓ 配置文件存在: $CONFIG"
}

# 编译项目
compile_project() {
    if [ "$NO_COMPILE" = true ]; then
        log_info "跳过编译（已编译）"
        return 0
    fi
    
    log_step "编译项目"
    
    cd /workspace/automap_pro
    make setup 2>&1 | grep -v "WARNING" || true
    make build-release 2>&1 | grep -v "WARNING" || true
    
    log_info "✓ 项目编译完成"
}

# 转换 bag（如果是 ROS1）
convert_bag_if_needed() {
    log_step "检查并转换 bag"
    
    local file_type=$(file "$BAG_FILE" | cut -d: -f2)
    
    if echo "$file_type" | grep -q "SQLite"; then
        log_info "✓ 检测到 ROS2 格式，无需转换"
        BAG_FILE_FINAL="$BAG_FILE"
    elif echo "$file_type" | grep -q "data"; then
        log_info "✓ 检测到 ROS1 格式，需要转换"
        
        local bag_dir=$(dirname "$BAG_FILE")
        local bag_name=$(basename "$BAG_FILE" .bag)
        BAG_FILE_FINAL="$bag_dir/${bag_name}_ros2"
        
        if [ -d "$BAG_FILE_FINAL" ]; then
            log_info "✓ 转换后的 bag 已存在: $BAG_FILE_FINAL"
        else
            log_info "开始转换 bag..."
            ros2 bag convert "$BAG_FILE" "$BAG_FILE_FINAL" 2>&1
            log_info "✓ Bag 转换完成"
        fi
    else
        log_error "无法识别的 bag 格式: $file_type"
        exit 1
    fi
}

# 启动建图
run_mapping() {
    log_step "启动建图"
    
    # Source 环境
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    
    log_info "开始建图..."
    log_info "Bag: $BAG_FILE_FINAL"
    log_info "Config: $CONFIG"
    log_info "Output: $OUTPUT_DIR"
    
    # 创建输出目录
    mkdir -p "$OUTPUT_DIR"
    
    # 运行建图
    cd /workspace
    ros2 launch automap_pro automap_offline.launch.py \
        config:="$CONFIG" \
        bag_file:="$BAG_FILE_FINAL" \
        rate:=1.0 \
        use_rviz:=false \
        use_external_frontend:=true \
        2>&1
    
    log_info "✓ 建图完成"
}

# 保存地图
save_map() {
    log_step "保存地图"
    
    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    
    ros2 service call /automap/save_map automap_pro/srv/SaveMap \
        "{output_dir: '$OUTPUT_DIR', save_pcd: true, save_ply: true, save_las: false, save_trajectory: true}" \
        2>&1 || {
        log_info "⚠ 保存地图失败，可能已自动保存"
        return 0
    }
    
    log_info "✓ 地图已保存"
}

# 显示结果
show_results() {
    log_step "显示结果"
    
    if [ -d "$OUTPUT_DIR" ]; then
        echo -e "\n${GREEN}输出目录内容:${NC}"
        ls -lh "$OUTPUT_DIR"
        
        # 检查地图文件
        if [ -f "$OUTPUT_DIR/map/global_map.pcd" ]; then
            local size=$(du -h "$OUTPUT_DIR/map/global_map.pcd" | cut -f1)
            log_info "✓ 地图: global_map.pcd ($size)"
        fi
        
        if [ -f "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt" ]; then
            local lines=$(wc -l < "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt")
            log_info "✓ 轨迹: optimized_trajectory_tum.txt ($lines 行)"
        fi
    else
        log_info "⚠ 输出目录不存在或为空"
    fi
}

# 主函数
main() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}AutoMap-Pro Docker 一键建图${NC}"
    echo -e "${BLUE}========================================${NC}\n"
    
    # 显示配置
    print_config
    
    # 确认开始
    if [ "$VERBOSE" = false ]; then
        read -p "是否开始建图? (y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            log_info "用户取消"
            exit 0
        fi
    fi
    
    # 执行步骤
    check_bag_file
    check_config_file
    compile_project
    convert_bag_if_needed
    run_mapping
    save_map
    show_results
    
    # 完成
    echo -e "\n${GREEN}========================================${NC}"
    echo -e "${GREEN}建图完成！${NC}"
    echo -e "${GREEN}========================================${NC}\n"
    
    # 可选：可视化
    if command -v python3 &> /dev/null; then
        read -p "是否可视化结果? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            log_info "启动可视化..."
            python3 /workspace/automap_pro/scripts/visualize_results.py \
                --output_dir "$OUTPUT_DIR" || {
                log_info "⚠ 可视化失败"
            }
        fi
    fi
}

# 运行主函数
main "$@"
