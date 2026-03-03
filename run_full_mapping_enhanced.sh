#!/bin/bash
# AutoMap-Pro 完整建图一键脚本（增强日志版）
# 功能：自动完成环境检查、编译、bag转换、建图、保存地图等所有步骤
# 增强：详细的各个环节日志记录、状态监控、进度追踪

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# 配置参数
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"
CONFIG="${CONFIG:-automap_pro/config/system_config.yaml}"
OUTPUT_DIR="${OUTPUT_DIR:-/data/automap_output/nya_02}"
# 容器内可由 run_full_mapping_docker.sh 传入 WORKSPACE=/workspace/automap_ws
WORKSPACE="${WORKSPACE:-${HOME}/automap_ws}"
LOG_DIR="${SCRIPT_DIR}/logs"
MONITOR_DIR="${LOG_DIR}/monitoring"

# 全局状态变量
BAG_FILE_ROS2=""
START_TIME=""
END_TIME=""
NODE_STATUS="unknown"

# 创建必要的目录
mkdir -p "$LOG_DIR"
mkdir -p "$MONITOR_DIR"
LOG_FILE="$LOG_DIR/full_mapping_$(date +%Y%m%d_%H%M%S).log"
NODE_MONITOR_FILE="$MONITOR_DIR/nodes_$(date +%Y%m%d_%H%M%S).log"
TOPIC_MONITOR_FILE="$MONITOR_DIR/topics_$(date +%Y%m%d_%H%M%S).log"
PROGRESS_FILE="$MONITOR_DIR/progress_$(date +%Y%m%d_%H%M%S).log"
# 本次运行的 launch 日志目录（ROS2 各节点日志写入此处，便于按进程排查）
LAUNCH_LOG_DIR="$LOG_DIR/launch_$(date +%Y%m%d_%H%M%S).d"
RUN_MANIFEST_FILE="$LOG_DIR/run_manifest_$(date +%Y%m%d_%H%M%S).txt"

# ========================================
# 增强日志系统
# ========================================

# 基础日志函数
log() {
    local level=$1
    shift
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
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

log_success() {
    log "${GREEN}[✓]${NC}" "$@"
}

log_debug() {
    if [ "$VERBOSE" = true ]; then
        log "${CYAN}[DEBUG]${NC}" "$@"
    fi
}

log_step() {
    local step=$1
    local total=$2
    local name=$3
    log "${CYAN}[STEP $step/$total]${NC}" "$name"
}

log_substep() {
    log "${MAGENTA}[  →]${NC}" "$@"
}

# 环节日志：精准定位，grep "LINK_2_CONTAINER" 可追踪容器内脚本全流程
log_link2() {
    local ts=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$ts] [LINK_2_CONTAINER] $@" | tee -a "$LOG_FILE"
}

log_progress() {
    local message="$@"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$timestamp] ${BLUE}[PROGRESS]${NC} $message" | tee -a "$PROGRESS_FILE"
}

log_section() {
    local title="$@"
    echo -e "\n${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  $title${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"
}

# 打印标题
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# ========================================
# 状态检查函数
# ========================================

# 检查并记录 ROS2 节点状态
check_ros2_nodes() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] ROS2 节点状态检查:" >> "$NODE_MONITOR_FILE"
    
    if ! command -v ros2 &> /dev/null; then
        echo "[ERROR] ROS2 命令不可用" >> "$NODE_MONITOR_FILE"
        return 1
    fi
    
    # 检查节点列表
    ros2 node list >> "$NODE_MONITOR_FILE" 2>&1 || true
    
    # 检查关键节点是否运行
    local nodes=("automap_system" "laserMapping" "rviz" "rosbag_player")
    echo "" >> "$NODE_MONITOR_FILE"
    echo "[INFO] 关键节点状态:" >> "$NODE_MONITOR_FILE"
    for node in "${nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            echo "  [✓] $node - 运行中" >> "$NODE_MONITOR_FILE"
        else
            echo "  [✗] $node - 未运行" >> "$NODE_MONITOR_FILE"
        fi
    done
    echo "" >> "$NODE_MONITOR_FILE"
}

# 检查并记录 ROS2 话题状态
check_ros2_topics() {
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] ROS2 话题状态检查:" >> "$TOPIC_MONITOR_FILE"
    
    if ! command -v ros2 &> /dev/null; then
        echo "[ERROR] ROS2 命令不可用" >> "$TOPIC_MONITOR_FILE"
        return 1
    fi
    
    # 列出所有话题
    ros2 topic list >> "$TOPIC_MONITOR_FILE" 2>&1 || true
    
    # 检查关键话题的频率和队列
    echo "" >> "$TOPIC_MONITOR_FILE"
    echo "[INFO] 关键话题状态:" >> "$TOPIC_MONITOR_FILE"
    local topics=(
        "/livox/lidar"
        "/livox/imu"
        "/optimized_pose"
        "/submap_map"
        "/global_map"
    )
    
    for topic in "${topics[@]}"; do
        if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
            local info=$(ros2 topic info "$topic" 2>/dev/null || echo "N/A")
            echo "  [✓] $topic" >> "$TOPIC_MONITOR_FILE"
            echo "      $info" >> "$TOPIC_MONITOR_FILE"
        else
            echo "  [✗] $topic - 未发布" >> "$TOPIC_MONITOR_FILE"
        fi
    done
    echo "" >> "$TOPIC_MONITOR_FILE"
}

# 检查并记录 ROS2 服务状态
check_ros2_services() {
    log_substep "检查 ROS2 服务..."
    
    if ! command -v ros2 &> /dev/null; then
        log_warn "ROS2 命令不可用"
        return 1
    fi
    
    local services=$(ros2 service list 2>/dev/null || echo "")
    if [ -n "$services" ]; then
        log_info "发现 $(echo "$services" | wc -l) 个服务"
        if echo "$services" | grep -q "/automap/save_map"; then
            log_success "✓ 保存地图服务可用"
        fi
    else
        log_warn "未发现任何服务"
    fi
}

# 检查文件系统状态
check_filesystem_status() {
    log_substep "检查文件系统状态..."
    
    # 检查磁盘空间
    local disk_usage=$(df -h "$OUTPUT_DIR" 2>/dev/null | tail -1 | awk '{print $5}' | sed 's/%//')
    if [ ! -z "$disk_usage" ]; then
        if [ "$disk_usage" -gt 90 ]; then
            log_warn "磁盘空间使用率过高: $disk_usage%"
        else
            log_info "磁盘空间使用率: $disk_usage%"
        fi
    fi
    
    # 检查输出目录
    if [ -d "$OUTPUT_DIR" ]; then
        local output_size=$(du -sh "$OUTPUT_DIR" 2>/dev/null | cut -f1)
        log_info "输出目录大小: $output_size"
    fi
}

# 检查系统资源
check_system_resources() {
    log_substep "检查系统资源..."
    
    # CPU 使用率
    local cpu_load=$(uptime | awk -F'load average:' '{print $2}' | cut -d, -f1 | xargs)
    log_info "CPU 负载: $cpu_load"
    
    # 内存使用
    local mem_info=$(free -h | grep "Mem:")
    local mem_total=$(echo $mem_info | awk '{print $2}')
    local mem_used=$(echo $mem_info | awk '{print $3}')
    local mem_percent=$(free | grep Mem | awk '{printf("%.1f"), $3/$2 * 100.0}')
    log_info "内存使用: $mem_used / $mem_total (${mem_percent}%)"
    
    # GPU 使用率（如果可用）
    if command -v nvidia-smi &> /dev/null; then
        local gpu_info=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader 2>/dev/null | head -1)
        log_info "GPU 使用: $gpu_info"
    fi
}

# 监控建图进度（后台运行）
monitor_mapping_progress() {
    local monitor_interval=10  # 秒
    
    while true; do
        # 检查 ROS2 节点
        check_ros2_nodes
        
        # 检查 ROS2 话题
        check_ros2_topics
        
        # 检查输出文件
        if [ -d "$OUTPUT_DIR" ]; then
            local submap_count=$(find "$OUTPUT_DIR" -name "*.pcd" 2>/dev/null | wc -l)
            log_progress "已生成 $submap_count 个子图文件"
            
            # 检查全局地图
            if [ -f "$OUTPUT_DIR/map/global_map.pcd" ]; then
                local map_size=$(du -h "$OUTPUT_DIR/map/global_map.pcd" | cut -f1)
                log_progress "全局地图大小: $map_size"
            fi
        fi
        
        sleep $monitor_interval
    done
}

# ========================================
# 主流程函数（增强日志）
# ========================================

# 显示帮助信息
show_help() {
    cat << EOF
AutoMap-Pro 完整建图一键脚本（增强日志版）

用法: $0 [选项]

选项:
    -b, --bag FILE          ROS bag 文件路径
    --bag-file FILE         ROS bag 文件路径（兼容格式）
    -c, --config FILE        配置文件路径
    -o, --output-dir DIR     输出目录
    --no-compile            跳过编译步骤
    --no-convert            跳过bag转换步骤
    --no-mapping            跳过建图步骤
    --no-save               跳过保存地图步骤
    --clean                 清理之前的工作空间
    --verbose               详细输出（包含调试信息）
    --monitor               启用实时监控（默认启用）
    --no-monitor            禁用实时监控
    --no-ui                 不启动 RViz/UI 界面（无头模式）
    -h, --help               显示此帮助信息

增强功能:
    - 详细的节点/话题/服务状态监控
    - 实时进度追踪（子图数量、地图大小）
    - 系统资源监控（CPU/内存/GPU）
    - 文件系统状态检查
    - 完整的日志记录（日志文件存储在 $LOG_DIR）

示例:
    # 默认配置运行
    $0

    # 详细模式
    $0 --verbose

    # 指定 bag 文件
    $0 -b /path/to/data.bag

环境变量:
    BAG_FILE         ROS bag 文件路径
    CONFIG           配置文件路径
    OUTPUT_DIR       输出目录

EOF
}

# 解析命令行参数
SKIP_COMPILE=false
SKIP_CONVERT=false
SKIP_MAPPING=false
SKIP_SAVE=false
CLEAN_WORKSPACE=false
VERBOSE=false
ENABLE_MONITOR=true
USE_RVIZ=true

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bag)
            BAG_FILE="$2"
            shift 2
            ;;
        --bag-file)
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
        --monitor)
            ENABLE_MONITOR=true
            shift
            ;;
        --no-monitor)
            ENABLE_MONITOR=false
            shift
            ;;
        --no-ui)
            USE_RVIZ=false
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

# 配置文件绝对路径（避免 Docker 下 SCRIPT_DIR+CONFIG 重复拼接导致路径错误）
if [[ "$CONFIG" == /* ]]; then
    CONFIG_ABS="$CONFIG"
else
    CONFIG_ABS="$SCRIPT_DIR/$CONFIG"
fi
# 记录启动时间
START_TIME=$(date '+%Y-%m-%d %H:%M:%S')

# 打印配置
print_config() {
    echo -e "${GREEN}AutoMap-Pro 完整建图配置（增强日志版）${NC}"
    echo -e "${CYAN}----------------------------------------${NC}"
    echo -e "${GREEN}脚本目录:${NC}           $SCRIPT_DIR"
    echo -e "${GREEN}Bag 文件:${NC}            $BAG_FILE"
    echo -e "${GREEN}配置文件:${NC}            $CONFIG"
    echo -e "${GREEN}输出目录:${NC}            $OUTPUT_DIR"
    echo -e "${GREEN}工作空间:${NC}            $WORKSPACE"
    echo -e "${GREEN}日志文件:${NC}            $LOG_FILE"
    echo -e "${GREEN}Launch 日志目录:${NC}      $LAUNCH_LOG_DIR"
    echo -e "${GREEN}节点监控:${NC}            $NODE_MONITOR_FILE"
    echo -e "${GREEN}话题监控:${NC}            $TOPIC_MONITOR_FILE"
    echo -e "${GREEN}进度追踪:${NC}            $PROGRESS_FILE"
    echo -e "${CYAN}----------------------------------------${NC}"
    
    # 选项状态
    echo -e "${GREEN}跳过编译:${NC}           $SKIP_COMPILE"
    echo -e "${GREEN}跳过转换:${NC}           $SKIP_CONVERT"
    echo -e "${GREEN}跳过建图:${NC}           $SKIP_MAPPING"
    echo -e "${GREEN}跳过保存:${NC}           $SKIP_SAVE"
    echo -e "${GREEN}清理工作空间:${NC}        $CLEAN_WORKSPACE"
    echo -e "${GREEN}详细输出:${NC}            $VERBOSE"
    echo -e "${GREEN}实时监控:${NC}            $ENABLE_MONITOR"
    echo -e "${GREEN}启动 RViz/UI:${NC}        $USE_RVIZ"
    echo -e "${CYAN}========================================${NC}\n"
}

# 步骤1: 环境检查（增强版）
check_environment() {
    log_link2 "step=1 check_environment config_abs=$CONFIG_ABS config_exists=$([ -f "$CONFIG_ABS" ] && echo true || echo false) bag_file=$BAG_FILE"
    print_header "步骤 1/6: 环境检查"
    START_TIME=$(date '+%s')
    
    log_section "检查 ROS2 环境"
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        log_error "ROS2 Humble 未安装"
        log_error "请先安装 ROS2 Humble"
        exit 1
    fi
    log_success "✓ ROS2 Humble 已安装"
    
    # 检查 Docker（用于 bag 转换）
    if ! command -v docker &> /dev/null; then
        log_warn "Docker 未安装，bag 转换可能受限"
    else
        log_success "✓ Docker 已安装: $(docker --version)"
    fi
    
    # 检查 Python3
    if ! command -v python3 &> /dev/null; then
        log_error "Python3 未安装"
        exit 1
    fi
    log_success "✓ Python3 已安装: $(python3 --version)"
    
    # 检查 colcon
    if ! command -v colcon &> /dev/null; then
        log_error "colcon 未安装"
        exit 1
    fi
    log_success "✓ colcon 已安装"
    
    log_section "检查输入文件"
    
    # 检查 bag：ROS1 为单文件 .bag，ROS2 为目录（含 metadata.yaml）
    if [ ! -f "$BAG_FILE" ] && [ ! -d "$BAG_FILE" ]; then
        log_error "Bag 文件不存在: $BAG_FILE"
        exit 1
    fi
    local bag_size=$(du -sh "$BAG_FILE" 2>/dev/null | cut -f1)
    local bag_info=""
    if command -v file >/dev/null 2>&1; then
        bag_info=$(file "$BAG_FILE" 2>/dev/null || true)
    else
        bag_info="(file 命令不可用，已跳过类型检测)"
    fi
    log_success "✓ Bag 文件存在: $BAG_FILE ($bag_size)"
    log_debug "  文件类型: ${bag_info:-未知}"
    
    # 检查配置文件
    if [ ! -f "$CONFIG_ABS" ]; then
        log_error "配置文件不存在: $CONFIG_ABS (原始 CONFIG=$CONFIG)"
        exit 1
    fi
    log_success "✓ 配置文件存在: $CONFIG_ABS"
    
    # 检查工作空间
    if [ ! -d "$WORKSPACE" ]; then
        log_warn "工作空间不存在，将创建"
    else
        log_success "✓ 工作空间存在: $WORKSPACE"
    fi
    
    # 检查并创建输出目录
    mkdir -p "$OUTPUT_DIR"
    log_success "✓ 输出目录已创建: $OUTPUT_DIR"
    
    # 检查系统资源
    check_system_resources
    
    log_section "环境检查完成"
    local elapsed=$(($(date '+%s') - START_TIME))
    log_info "耗时: ${elapsed}秒"
}

# 步骤2: 编译项目（增强版）
# 使用 PIPESTATUS 正确检测 make/colcon 退出码（管道后 || 拿到的是 tee 的退出码）
compile_project() {
    if [ "$SKIP_COMPILE" = true ]; then
        log_info "跳过编译步骤"
        return 0
    fi
    
    print_header "步骤 2/6: 编译项目"
    START_TIME=$(date '+%s')
    
    # 使用 automap_pro 下的 Makefile（setup/build-release/clean）
    local make_dir="$SCRIPT_DIR/automap_pro"
    if [ ! -f "$make_dir/Makefile" ]; then
        make_dir="$SCRIPT_DIR"
    fi
    
    # ─── 精准路径日志：便于确认「改的是否为起作用的文件」───
    log_section "编译步骤路径（用于精准定位）"
    local make_dir_abs="$make_dir"
    [[ "$make_dir" != /* ]] && make_dir_abs="$SCRIPT_DIR/$make_dir"
    log_info "[PATH] 脚本目录(SCRIPT_DIR)=$SCRIPT_DIR"
    log_info "[PATH] 工作空间(WORKSPACE)=$WORKSPACE"
    log_info "[PATH] Makefile 所在目录(make_dir)=$make_dir"
    log_info "[PATH] Makefile 绝对路径=$make_dir_abs/Makefile"
    log_info "[PATH] automap_pro 源码(用于编译)=$WORKSPACE/src/automap_pro 或 Makefile 指定"
    log_info "[PATH] 编译成功后 automap_pro 安装目录=$WORKSPACE/install/automap_pro"
    log_info "[PATH] 主日志文件(本步骤输出)=$LOG_FILE"
    
    # 清理工作空间（如果需要）
    if [ "$CLEAN_WORKSPACE" = true ]; then
        log_substep "清理工作空间..."
        make -C "$make_dir" clean 2>&1 | tee -a "$LOG_FILE" || true
    fi
    
    # 设置工作空间
    log_substep "设置工作空间..."
    log_info "执行: make -C $make_dir setup"
    make -C "$make_dir" setup 2>&1 | tee -a "$LOG_FILE"
    local setup_rc=${PIPESTATUS[0]}
    if [ "$setup_rc" -ne 0 ]; then
        log_error "工作空间设置失败 (exit code=$setup_rc)"
        log_error "[PATH] 失败命令: make -C $make_dir setup"
        log_error "[PATH] Makefile 文件: $make_dir_abs/Makefile"
        log_error "请查看日志文件: $LOG_FILE"
        exit 1
    fi
    log_success "✓ 工作空间设置完成"
    
    # 编译项目（必须用 PIPESTATUS 判断 make 是否成功，管道中 tee 会吞掉 make 的退出码）
    log_substep "编译项目（Release 模式）..."
    log_info "执行: make -C $make_dir build-release"
    local compile_start=$(date '+%s')
    make -C "$make_dir" build-release 2>&1 | tee -a "$LOG_FILE"
    local build_rc=${PIPESTATUS[0]}
    local compile_end=$(date '+%s')
    local compile_elapsed=$((compile_end - compile_start))
    if [ "$build_rc" -ne 0 ]; then
        log_error "项目编译失败 (exit code=$build_rc)"
        log_error "[PATH] 失败命令: make -C $make_dir build-release"
        log_error "[PATH] 使用的 Makefile: $make_dir_abs/Makefile"
        log_error "[PATH] 工作空间: $WORKSPACE（colcon 在该目录下 build/install）"
        log_error "请查看上方编译错误与日志文件: $LOG_FILE"
        exit 1
    fi
    log_success "✓ 项目编译完成 (耗时: ${compile_elapsed}秒)"
    
    # 检查编译产物并再次输出生效路径
    if [ -d "$WORKSPACE/install/automap_pro" ]; then
        log_success "✓ 安装目录已创建: $WORKSPACE/install/automap_pro"
        local node_path="$WORKSPACE/install/automap_pro/lib/automap_pro/automap_system_node"
        if [ -f "$node_path" ]; then
            log_info "[PATH] 建图将使用的 automap 节点(绝对)=$node_path"
        fi
    else
        log_warn "[PATH] 未找到 $WORKSPACE/install/automap_pro，建图可能失败"
    fi

    # 编译 fast_livo（parameter '' 修复依赖其 main.cpp 中 automatically_declare_parameters_from_overrides=false）
    local fast_livo_src="$SCRIPT_DIR/fast-livo2-humble"
    if [ -d "$fast_livo_src" ] && [ -f "$fast_livo_src/package.xml" ]; then
        log_substep "编译 fast_livo（修复 parameter ''，源码: $fast_livo_src）..."
        mkdir -p "$WORKSPACE/src"
        local link_path="$WORKSPACE/src/fast-livo2-humble"
        if [ ! -L "$link_path" ] && [ ! -d "$link_path" ]; then
            ln -sfn "$fast_livo_src" "$link_path"
            log_info "[BUILD_FAST_LIVO] 已软链: $fast_livo_src -> $link_path"
        else
            log_info "[BUILD_FAST_LIVO] 已存在: $link_path (无需软链)"
        fi
        # 清理 fast_livo 的 build/install，避免使用缓存的 /root/automap_ws 路径导致 colcon 报错
        if [ -d "$WORKSPACE/build/fast_livo" ] || [ -d "$WORKSPACE/install/fast_livo" ]; then
            log_info "[BUILD_FAST_LIVO] 清理旧 build/install 以使用当前 WORKSPACE 路径"
            rm -rf "$WORKSPACE/build/fast_livo" "$WORKSPACE/install/fast_livo"
        fi
        log_info "[BUILD_FAST_LIVO] 执行: cd $WORKSPACE && source install/setup.bash && colcon build --packages-select fast_livo ..."
        (cd "$WORKSPACE" && source /opt/ros/humble/setup.bash && source install/setup.bash 2>/dev/null || true && colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release) 2>&1 | tee -a "$LOG_FILE"
        local colcon_rc=${PIPESTATUS[0]}
        local fast_livo_exe=""
        [ -f "$WORKSPACE/install/fast_livo/lib/fast_livo/fastlivo_mapping" ] && fast_livo_exe="$WORKSPACE/install/fast_livo/lib/fast_livo/fastlivo_mapping"
        [ -z "$fast_livo_exe" ] && [ -f "$WORKSPACE/install/fast_livo/bin/fastlivo_mapping" ] && fast_livo_exe="$WORKSPACE/install/fast_livo/bin/fastlivo_mapping"
        if [ "$colcon_rc" -ne 0 ]; then
            log_warn "fast_livo 编译失败 (exit code=$colcon_rc)"
            log_warn "[PATH] 失败命令: cd $WORKSPACE && colcon build --packages-select fast_livo ..."
            log_warn "[PATH] 若 fast_livo 依赖缺失，请在同一 WORKSPACE 先完成 overlap_transformer_msgs 等依赖编译"
            log_warn "建图时 fast_livo 将使用已安装的旧二进制（若有），或报 parameter ''；可容器内手动: cd $WORKSPACE && colcon build --packages-select fast_livo"
        fi
        if [ -n "$fast_livo_exe" ]; then
            log_success "✓ fast_livo 已编译（parameter '' 修复将生效）"
            log_info "[PATH] 建图将使用的 fast_livo 可执行文件(绝对)=$fast_livo_exe"
        else
            log_warn "[BUILD_FAST_LIVO] 未找到 fastlivo_mapping；建图将使用的 fast_livo 可执行文件: (未找到，建图可能报 parameter '' 或启动失败)"
        fi
    else
        log_warn "[BUILD_FAST_LIVO] 未找到 fast_livo 源码 ($fast_livo_src)，跳过编译；若建图报 parameter ''，请将 fast-livo2-humble 置于 $SCRIPT_DIR 并重跑"
    fi
    
    local elapsed=$(($(date '+%s') - START_TIME))
    log_info "步骤2 总耗时: ${elapsed}秒"
}

# 步骤3: 转换 bag（增强版）
convert_bag() {
    # 跳过转换时：直接视为 ROS2，使用当前 BAG_FILE 作为建图输入
    if [ "$SKIP_CONVERT" = true ]; then
        log_info "跳过 bag 转换步骤，直接使用当前路径作为 ROS2 数据"
        BAG_FILE_ROS2="$BAG_FILE"
        return 0
    fi
    
    print_header "步骤 3/6: 检查并转换 Bag"
    START_TIME=$(date '+%s')
    
    # 解析绝对路径（用于目录检测）
    if [[ "$BAG_FILE" != /* ]]; then
        local BAG_ABS="$SCRIPT_DIR/$BAG_FILE"
    else
        local BAG_ABS="$BAG_FILE"
    fi
    
    # 检查 bag 格式（兼容容器内无 file 命令：优先目录 metadata.yaml → file → ros2 bag info → 兜底）
    log_substep "检查 bag 格式..."
    local file_type=""
    # 1) 若为目录且含 metadata.yaml，直接认定为 ROS2（不依赖 ros2 bag info，容器内更可靠）
    if [ -d "$BAG_FILE" ] && [ -f "$BAG_FILE/metadata.yaml" ]; then
        file_type="ROS2 (directory metadata.yaml)"
    elif [ -d "$BAG_ABS" ] && [ -f "$BAG_ABS/metadata.yaml" ]; then
        file_type="ROS2 (directory metadata.yaml)"
    fi
    if [ -z "$file_type" ] && command -v file >/dev/null 2>&1; then
        file_type=$(file "$BAG_FILE" 2>/dev/null | cut -d: -f2 | sed 's/^ *//')
    fi
    log_debug "  文件类型: $file_type"
    
    # 当仍未识别时：用 ros2 bag info 判断是否为 ROS2，否则按 ROS1 处理
    if [ -z "$file_type" ]; then
        if (source /opt/ros/humble/setup.bash 2>/dev/null && ros2 bag info "$BAG_FILE" >/dev/null 2>&1); then
            file_type="ROS2 (ros2 bag info 识别)"
        else
            # 容器内常见无 file：按兜底视为 ROS1
            file_type="data (按扩展名/兜底视为 ROS1)"
        fi
    fi
    
    if echo "$file_type" | grep -qi "SQLite\|ROS2"; then
        log_success "✓ 检测到 ROS2 格式，无需转换"
        BAG_FILE_ROS2="$BAG_FILE"
    elif echo "$file_type" | grep -qi "data"; then
        log_info "✓ 检测到 ROS1 格式，需要先转换为 ROS2 再建图"
        
        # 解析绝对路径（用于本机 rosbags-convert 或 Docker 挂载）
        if [[ "$BAG_FILE" != /* ]]; then
            BAG_ABS="$SCRIPT_DIR/$BAG_FILE"
        else
            BAG_ABS="$BAG_FILE"
        fi
        local bag_dir=$(dirname "$BAG_FILE")
        local bag_dir_abs=$(dirname "$BAG_ABS")
        local bag_name=$(basename "$BAG_FILE" .bag)
        BAG_FILE_ROS2="$bag_dir/${bag_name}_ros2"
        local ros2_dir_abs="$bag_dir_abs/${bag_name}_ros2"
        
        # 检查是否已经转换
        if [ -d "$ros2_dir_abs" ] && [ -f "$ros2_dir_abs/metadata.yaml" ]; then
            log_success "✓ 已存在 ROS2 转换结果: $BAG_FILE_ROS2，直接使用"
            BAG_FILE_ROS2="$ros2_dir_abs"
            return 0
        fi
        if [ -d "$ros2_dir_abs" ]; then
            log_substep "删除不完整的旧转换结果..."
            rm -rf "$ros2_dir_abs"
        fi
        
        # 方法1: 本机 rosbags-convert（无需 Docker）
        if command -v rosbags-convert &>/dev/null; then
            log_substep "使用本机 rosbags-convert 转换..."
            local convert_start=$(date '+%s')
            if rosbags-convert --src "$BAG_ABS" --dst "$ros2_dir_abs" 2>&1 | tee -a "$LOG_FILE"; then
                local convert_end=$(date '+%s')
                log_success "✓ Bag 转换完成 (耗时: $((convert_end - convert_start))秒)"
                BAG_FILE_ROS2="$ros2_dir_abs"
            else
                log_error "rosbags-convert 失败，请检查: pip install rosbags"
                exit 1
            fi
        # 方法2: Docker 转换
        elif command -v docker &> /dev/null; then
            log_substep "使用 Docker 转换 bag..."
            
            if ! docker images | grep -q "ros1-to-ros2-converter"; then
                log_substep "构建转换镜像..."
                (cd "$SCRIPT_DIR" && docker build -t ros1-to-ros2-converter -f docker/converter.Dockerfile .) \
                    2>&1 | tee -a "$LOG_FILE" || {
                    log_error "转换镜像构建失败"
                    exit 1
                }
            fi
            
            log_substep "运行 bag 转换..."
            local convert_start=$(date '+%s')
            docker run --rm \
                -v "$SCRIPT_DIR/data:/workspace/data" \
                ros1-to-ros2-converter \
                bash -c "
                    source /opt/ros/humble/setup.bash &&
                    ros2 bag convert /workspace/$BAG_FILE /workspace/$BAG_FILE_ROS2
                " 2>&1 | tee -a "$LOG_FILE" || {
                log_error "Bag 转换失败，可尝试先单独转换: ./convert_ros1_to_ros2.sh -b \"$BAG_FILE\""
                log_error "然后建图: $0 -b \"$BAG_FILE_ROS2\" --no-convert"
                exit 1
            }
            local convert_end=$(date '+%s')
            log_success "✓ Bag 转换完成 (耗时: $((convert_end - convert_start))秒)"
            BAG_FILE_ROS2="$ros2_dir_abs"
            if [ -d "$ros2_dir_abs" ]; then
                log_success "✓ 转换后大小: $(du -sh "$ros2_dir_abs" | cut -f1)"
            fi
        else
            log_error "未检测到 Docker 且本机无 rosbags-convert，无法转换 ROS1 bag。"
            log_info "请先完成转换再建图："
            echo "  1) 安装 Docker 后重新运行本脚本；或"
            echo "  2) 本机安装: pip install rosbags"
            echo "  3) 或先单独转换再建图："
            echo "      ./convert_ros1_to_ros2.sh -b \"$BAG_FILE\""
            echo "      $0 -b \"$BAG_FILE_ROS2\" --no-convert"
            exit 1
        fi
    else
        log_error "无法识别的 bag 格式: $file_type"
        exit 1
    fi
    
    local elapsed=$(($(date '+%s') - START_TIME))
    log_info "步骤3 总耗时: ${elapsed}秒"
}

# 步骤4: 启动建图（增强版）
run_mapping() {
    if [ "$SKIP_MAPPING" = true ]; then
        log_info "跳过建图步骤"
        return 0
    fi
    log_link2 "step=4 run_mapping config_abs=$CONFIG_ABS bag_ros2=$BAG_FILE_ROS2 output_dir=$OUTPUT_DIR launch_log_dir=$LAUNCH_LOG_DIR"
    
    print_header "步骤 4/6: 启动建图"
    START_TIME=$(date '+%s')
    
    log_substep "Source ROS2 和工作空间..."
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE/install/setup.bash"
    log_success "✓ 环境变量已加载"
    
    # 自动验证：建图 launch 使用 system_config 注入参数
    VERIFY_SCRIPT="$SCRIPT_DIR/automap_pro/scripts/verify_system_config_launch.sh"
    if [ -f "$VERIFY_SCRIPT" ]; then
        log_substep "运行 system_config 使用验证..."
        if bash "$VERIFY_SCRIPT" "$CONFIG_ABS" 2>&1 | tee -a "$LOG_FILE"; then
            log_success "✓ system_config 验证通过"
        else
            log_warn "system_config 验证未通过，继续建图（将优先使用源码 launch）"
        fi
    fi
    
    # ─── 建图步骤精准路径（便于确认「起作用的文件」与排查 parameter '' 等）───
    log_section "建图步骤路径（用于精准定位）"
    log_info "[PATH] 配置文件(绝对)=$CONFIG_ABS"
    log_info "[PATH] Bag/ROS2 数据(绝对)=$BAG_FILE_ROS2"
    [[ "$BAG_FILE_ROS2" != /* ]] && log_info "[PATH] Bag 解析后绝对路径=$SCRIPT_DIR/$BAG_FILE_ROS2"
    local params_file_expected="$SCRIPT_DIR/automap_pro/logs/fast_livo_params.yaml"
    log_info "[PATH] fast_livo 参数文件(launch 将写入)=$params_file_expected"
    log_info "[PATH] 若报 parameter ''，请检查该文件: grep -nE \"^[[:space:]]*:[^ ]|'':\" $params_file_expected"
    log_info "[PATH] 工作空间(WORKSPACE)=$WORKSPACE"
    log_info "[PATH] Launch 日志目录=$LAUNCH_LOG_DIR"
    log_info "[PATH] 主日志文件=$LOG_FILE"
    
    log_section "建图配置"
    log_info "Bag 文件: $BAG_FILE_ROS2"
    # 修复 rosbags 生成的 metadata.yaml：offered_qos_profiles: [] 会导致 ros2 bag play 报 yaml-cpp bad conversion
    BAG_ROS2_ABS="$BAG_FILE_ROS2"
    [[ "$BAG_FILE_ROS2" != /* ]] && BAG_ROS2_ABS="$SCRIPT_DIR/$BAG_FILE_ROS2"
    if [ -d "$BAG_ROS2_ABS" ] && [ -f "$BAG_ROS2_ABS/metadata.yaml" ]; then
        if grep -q "offered_qos_profiles: \[\]" "$BAG_ROS2_ABS/metadata.yaml" 2>/dev/null; then
            if sed -i.bak "s/offered_qos_profiles: \[\]/offered_qos_profiles: ''/g" "$BAG_ROS2_ABS/metadata.yaml" 2>/dev/null; then
                log_success "已修复 ROS2 bag metadata.yaml（offered_qos_profiles [] -> ''），避免 ros2 bag play 解析错误"
            else
                log_warn "无法写入 metadata.yaml（可能只读），若 ros2 bag play 报 bad conversion 请在可写环境执行: sed -i \"s/offered_qos_profiles: \\[\\]/offered_qos_profiles: ''/g\" <bag_dir>/metadata.yaml"
            fi
        fi
    fi
    log_info "配置文件(解析): $CONFIG_ABS"
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
    
    # 创建 launch 日志目录并导出，便于精准定位各节点日志（ros2 launch 会写入此处）
    mkdir -p "$LAUNCH_LOG_DIR"
    mkdir -p "$LOG_DIR"
    export ROS_LOG_DIR="$LAUNCH_LOG_DIR"
    export AUTOMAP_LOG_DIR="$LOG_DIR"
    log_info "[LAUNCH_LOG_DIR] $LAUNCH_LOG_DIR"
    log_info "[AUTOMAP_LOG_DIR] $LOG_DIR（fast_livo_params 等写入此处）"
    log_info "[CONFIG_RESOLVED] config:=$CONFIG_ABS"
    
    # 写入运行清单，便于事后关联日志与参数（含 fast_livo、params 文件路径，用于确认是否使用已修复 parameter '' 的二进制）
    fast_livo_prefix=$(ros2 pkg prefix fast_livo 2>/dev/null || true)
    params_file_expected="$LOG_DIR/fast_livo_params.yaml"
    {
        echo "run_start=$(date -Iseconds 2>/dev/null || date '+%Y-%m-%d %H:%M:%S')"
        echo "bag_file=$BAG_FILE_ROS2"
        echo "config_abs=$CONFIG_ABS"
        echo "output_dir=$OUTPUT_DIR"
        echo "log_file=$LOG_FILE"
        echo "launch_log_dir=$LAUNCH_LOG_DIR"
        echo "workspace=$WORKSPACE"
        echo "params_file_for_fast_livo=$params_file_expected"
        echo "fast_livo_install_prefix=${fast_livo_prefix:-未找到}"
        if [ -n "$fast_livo_prefix" ]; then
            echo "fast_livo_exe=${fast_livo_prefix}/lib/fast_livo/fastlivo_mapping"
        fi
        echo "source_launch_offline=$SCRIPT_DIR/automap_pro/launch/automap_offline.launch.py"
    } > "$RUN_MANIFEST_FILE"
    log_info "[RUN_MANIFEST] $RUN_MANIFEST_FILE"
    log_info "[RUN_MANIFEST] params_file_for_fast_livo=$params_file_expected（建图时 launch 会写入该文件；若报 parameter '' 请检查该文件是否含空键）"
    log_info "[RUN_MANIFEST] fast_livo 安装路径: ${fast_livo_prefix:-未找到}（建图启动后应看到 stderr: [fast_livo] main: automatically_declare_parameters_from_overrides=false）"
    
    log_substep "启动建图流程..."
    if [ "$USE_RVIZ" = true ]; then
        log_info "UI 界面: 已启用（RViz 将随建图启动）"
        if [ -z "${DISPLAY:-}" ]; then
            log_warn "DISPLAY 未设置，RViz 可能无法显示；如需无界面运行请使用 --no-ui"
        fi
    else
        log_info "UI 界面: 已禁用（--no-ui）"
    fi
    # 优先使用源码中的 launch，确保 fast_livo/overlap_transformer 参数从 system_config 注入（不依赖 install 是否更新）
    SOURCE_LAUNCH_OFFLINE="$SCRIPT_DIR/automap_pro/launch/automap_offline.launch.py"
    USE_SOURCE_LAUNCH=false
    if [ -f "$SOURCE_LAUNCH_OFFLINE" ] && grep -q "params_from_system_config\|OpaqueFunction" "$SOURCE_LAUNCH_OFFLINE" 2>/dev/null; then
        USE_SOURCE_LAUNCH=true
        log_success "使用源码 launch（system_config 注入）: $SOURCE_LAUNCH_OFFLINE"
    else
        log_warn "使用 install 空间 launch；若 fast_livo 未使用 system_config，请确保已 colcon build automap_pro"
    fi
    log_info "执行: ros2 launch automap_pro automap_offline.launch.py"
    log_debug "  参数: config:=$CONFIG_ABS"
    log_debug "  参数: $bag_arg"
    log_debug "  参数: rate:=1.0"
    log_debug "  参数: use_rviz:=$USE_RVIZ"
    log_debug "  参数: use_external_frontend:=true（使用已验证的 fast_livo 节点）"
    
    # 启动监控（如果启用）
    MONITOR_PID=""
    if [ "$ENABLE_MONITOR" = true ]; then
        log_substep "启动实时监控..."
        monitor_mapping_progress > /dev/null 2>&1 &
        MONITOR_PID=$!
        log_info "监控进程 PID: $MONITOR_PID"
    fi
    
    # 详细模式：诊断信息写入日志，便于精准分析（如 bag metadata 解析错误、配置错误）
    if [ "$VERBOSE" = true ]; then
        log_substep "写入诊断信息到主日志..."
        echo "[$(date '+%Y-%m-%d %H:%M:%S.%3N')] [DIAG] === ros2 bag info ===" >> "$LOG_FILE"
        (ros2 bag info "$BAG_FILE_ROS2" 2>&1 || true) >> "$LOG_FILE"
        echo "[$(date '+%Y-%m-%d %H:%M:%S.%3N')] [DIAG] === config file head (first 50 lines) ===" >> "$LOG_FILE"
        (head -n 50 "$CONFIG_ABS" 2>&1 || true) >> "$LOG_FILE"
        echo "[$(date '+%Y-%m-%d %H:%M:%S.%3N')] [DIAG] === end ===" >> "$LOG_FILE"
        log_info "[DIAG] 诊断已追加到 $LOG_FILE（可 grep \"[DIAG]\" 或 \"[LAUNCH_EXIT]\" 快速定位）"
    fi
    
    # 记录开始状态
    log_progress "建图开始: $(date '+%Y-%m-%d %H:%M:%S')"
    check_ros2_nodes
    check_ros2_topics
    
    # 启动建图（捕获退出码并写入日志，便于精准分析进程退出原因）
    cd "$WORKSPACE"
    log_link2 "launch_invoke use_source_launch=$USE_SOURCE_LAUNCH launch_file=${SOURCE_LAUNCH_OFFLINE:-automap_offline.launch.py} config:=$CONFIG_ABS bag_arg=$bag_arg"
    log_section "建图进行中（按 Ctrl+C 停止）"
    set +e
    if [ "$USE_SOURCE_LAUNCH" = true ]; then
        ros2 launch "$SOURCE_LAUNCH_OFFLINE" \
            config:="$CONFIG_ABS" \
            $bag_arg \
            rate:=1.0 \
            use_rviz:="$(echo "$USE_RVIZ" | tr '[:upper:]' '[:lower:]')" \
            use_external_frontend:=true \
            2>&1 | tee -a "$LOG_FILE"
    else
        ros2 launch automap_pro automap_offline.launch.py \
            config:="$CONFIG_ABS" \
            $bag_arg \
            rate:=1.0 \
            use_rviz:="$(echo "$USE_RVIZ" | tr '[:upper:]' '[:lower:]')" \
            use_external_frontend:=true \
            2>&1 | tee -a "$LOG_FILE"
    fi
    LAUNCH_EXIT=$?
    set -e
    log_link2 "launch_exit code=$LAUNCH_EXIT manifest=$RUN_MANIFEST_FILE"
    echo "[$(date '+%Y-%m-%d %H:%M:%S.%3N')] [LAUNCH_EXIT] code=$LAUNCH_EXIT" | tee -a "$LOG_FILE"
    echo "launch_exit_code=$LAUNCH_EXIT" >> "$RUN_MANIFEST_FILE"
    if [ "$LAUNCH_EXIT" -ne 0 ]; then
        log_warn "建图进程非零退出: $LAUNCH_EXIT，请查看 $LOG_FILE 与 $LAUNCH_LOG_DIR 排查"
    fi
    
    # 停止监控
    if [ -n "$MONITOR_PID" ]; then
        log_substep "停止监控进程..."
        kill $MONITOR_PID 2>/dev/null || true
        wait $MONITOR_PID 2>/dev/null || true
    fi
    
    # 记录结束状态
    log_progress "建图结束: $(date '+%Y-%m-%d %H:%M:%S')"
    check_ros2_nodes
    check_ros2_topics
    check_filesystem_status
    
    local elapsed=$(($(date '+%s') - START_TIME))
    log_info "步骤4 总耗时: ${elapsed}秒"
    log_success "✓ 建图完成"
}

# 步骤5: 保存地图（增强版）
save_map() {
    if [ "$SKIP_SAVE" = true ]; then
        log_info "跳过保存地图步骤"
        return 0
    fi
    
    print_header "步骤 5/6: 保存地图"
    START_TIME=$(date '+%s')
    
    # Source ROS2 和工作空间
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE/install/setup.bash"
    
    log_substep "检查 ROS2 服务..."
    check_ros2_services
    
    log_substep "保存地图到: $OUTPUT_DIR"
    
    # 调用保存服务
    log_info "执行: ros2 service call /automap/save_map"
    local save_start=$(date '+%s')
    ros2 service call /automap/save_map automap_pro/srv/SaveMap \
        "{output_dir: '$OUTPUT_DIR', save_pcd: true, save_ply: true, save_las: false, save_trajectory: true}" \
        2>&1 | tee -a "$LOG_FILE" || {
        log_warn "保存地图服务调用失败，可能需要手动保存"
        log_warn "请检查 /automap/save_map 服务是否可用"
        return 0
    }
    local save_end=$(date '+%s')
    local save_elapsed=$((save_end - save_start))
    
    log_success "✓ 地图保存请求已发送 (耗时: ${save_elapsed}秒)"
    
    # 检查保存结果
    log_substep "检查保存结果..."
    if [ -d "$OUTPUT_DIR/map" ]; then
        local map_files=$(find "$OUTPUT_DIR/map" -type f 2>/dev/null | wc -l)
        log_success "✓ 地图目录包含 $map_files 个文件"
        
        # 列出地图文件
        if [ -f "$OUTPUT_DIR/map/global_map.pcd" ]; then
            local size=$(du -h "$OUTPUT_DIR/map/global_map.pcd" | cut -f1)
            log_info "  - global_map.pcd ($size)"
        fi
        if [ -f "$OUTPUT_DIR/map/global_map.ply" ]; then
            local size=$(du -h "$OUTPUT_DIR/map/global_map.ply" | cut -f1)
            log_info "  - global_map.ply ($size)"
        fi
    fi
    
    if [ -d "$OUTPUT_DIR/trajectory" ]; then
        local traj_files=$(find "$OUTPUT_DIR/trajectory" -type f 2>/dev/null | wc -l)
        log_success "✓ 轨迹目录包含 $traj_files 个文件"
    fi
    
    local elapsed=$(($(date '+%s') - START_TIME))
    log_info "步骤5 总耗时: ${elapsed}秒"
}

# 步骤6: 显示结果（增强版）
show_results() {
    print_header "步骤 6/6: 显示结果"
    
    log_section "建图结果摘要"
    log_info "输出目录: $OUTPUT_DIR"
    
    # 检查输出目录
    if [ ! -d "$OUTPUT_DIR" ]; then
        log_warn "输出目录不存在或为空"
        return 0
    fi
    
    # 检查地图文件
    log_substep "检查地图文件..."
    if [ -f "$OUTPUT_DIR/map/global_map.pcd" ]; then
        local map_size=$(du -h "$OUTPUT_DIR/map/global_map.pcd" | cut -f1)
        log_success "✓ 地图文件: global_map.pcd ($map_size)"
    else
        log_warn "✗ 地图文件不存在: global_map.pcd"
    fi
    
    if [ -f "$OUTPUT_DIR/map/global_map.ply" ]; then
        local map_size=$(du -h "$OUTPUT_DIR/map/global_map.ply" | cut -f1)
        log_success "✓ 地图文件: global_map.ply ($map_size)"
    fi
    
    # 检查轨迹文件
    log_substep "检查轨迹文件..."
    if [ -f "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt" ]; then
        local traj_lines=$(wc -l < "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt")
        local traj_size=$(du -h "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt" | cut -f1)
        log_success "✓ 轨迹文件: optimized_trajectory_tum.txt ($traj_lines 行, $traj_size)"
    else
        log_warn "✗ 轨迹文件不存在: optimized_trajectory_tum.txt"
    fi
    
    # 检查子图
    log_substep "检查子图..."
    if [ -d "$OUTPUT_DIR/submaps" ]; then
        local submap_count=$(find "$OUTPUT_DIR/submaps" -name "*.pcd" 2>/dev/null | wc -l)
        log_success "✓ 子图数量: $submap_count"
    else
        log_warn "✗ 子图目录不存在"
    fi
    
    # 输出目录内容
    log_section "输出目录内容"
    ls -lh "$OUTPUT_DIR" 2>/dev/null || log_warn "无法列出目录内容"
    
    # 检查文件系统
    check_filesystem_status
    
    # 显示监控日志摘要
    if [ -f "$NODE_MONITOR_FILE" ]; then
        log_section "节点监控摘要"
        grep "\[✓\]" "$NODE_MONITOR_FILE" | tail -10 || true
        grep "\[✗\]" "$NODE_MONITOR_FILE" || true
    fi
    
    if [ -f "$TOPIC_MONITOR_FILE" ]; then
        log_section "话题监控摘要"
        grep "\[✓\]" "$TOPIC_MONITOR_FILE" | tail -10 || true
        grep "\[✗\]" "$TOPIC_MONITOR_FILE" || true
    fi
    
    if [ -f "$PROGRESS_FILE" ]; then
        log_section "进度追踪摘要"
        tail -20 "$PROGRESS_FILE" || true
    fi
}

# 显示执行摘要
show_summary() {
    END_TIME=$(date '+%Y-%m-%d %H:%M:%S')
    
    print_header "建图执行摘要"
    log_info "开始时间: $START_TIME"
    log_info "结束时间: $END_TIME"
    
    if [ -n "$START_TIME" ] && command -v datecalc &> /dev/null; then
        local total_seconds=$(($(date -d "$END_TIME" +%s) - $(date -d "$START_TIME" +%s)))
        local hours=$((total_seconds / 3600))
        local minutes=$(((total_seconds % 3600) / 60))
        local seconds=$((total_seconds % 60))
        log_info "总耗时: ${hours}小时 ${minutes}分钟 ${seconds}秒"
    fi
    
    log_section "日志文件位置"
    log_info "主日志: $LOG_FILE"
    log_info "Launch 日志目录: $LAUNCH_LOG_DIR"
    log_info "运行清单: $RUN_MANIFEST_FILE"
    log_info "节点监控: $NODE_MONITOR_FILE"
    log_info "话题监控: $TOPIC_MONITOR_FILE"
    log_info "进度追踪: $PROGRESS_FILE"
    log_info "排障提示: grep -E \"\\[LAUNCH_EXIT\\]|\\[DIAG\\]|\\[ERROR\\]\" $LOG_FILE"
    log_info "环节追踪: grep -E \"LINK_1_SCRIPT|LINK_2_CONTAINER|LINK_3_LAUNCH|LINK_4_PARAMS\" $LOG_FILE 可精准定位各层"
    
    log_section "输出目录"
    log_info "$OUTPUT_DIR"
}

# 主函数
main() {
    log_link2 "entry script_dir=$SCRIPT_DIR config_abs=$CONFIG_ABS bag_file=$BAG_FILE output_dir=$OUTPUT_DIR workspace=$WORKSPACE log_file=$LOG_FILE"
    print_header "AutoMap-Pro 完整建图一键脚本（增强日志版）"
    
    # 显示配置
    print_config
    
    # 确认开始（已注释：不再交互询问，直接开始建图；需恢复时可取消下方 else 内注释）
    if [ "$VERBOSE" = true ]; then
        log_info "详细模式已启用"
    # else
    #     read -p "是否开始建图? (y/n): " -n 1 -r
    #     echo
    #     if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    #         log_info "用户取消"
    #         exit 0
    #     fi
    fi
    
    # 执行步骤
    check_environment
    compile_project
    convert_bag
    run_mapping
    save_map
    show_results
    
    # 显示摘要
    show_summary
    
    # 完成
    print_header "建图完成！"
    
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
