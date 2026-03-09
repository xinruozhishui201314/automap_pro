#!/bin/bash
# AutoMap-Pro 逐环节验证脚本
# 在Docker容器内逐步执行建图流程并验证每个环节

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $@"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $@"; }
log_error() { echo -e "${RED}[ERROR]${NC} $@"; }
log_step() { echo -e "${CYAN}[STEP]${NC} $@"; }
log_substep() { echo -e "${MAGENTA}[  →]${NC} $@"; }
log_success() { echo -e "${GREEN}[✓]${NC} $@"; }
log_fail() { echo -e "${RED}[✗]${NC} $@"; }

# 配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${CONFIG:-automap_pro/config/system_config.yaml}"
BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2}"
OUTPUT_DIR="${OUTPUT_DIR:-/workspace/output}"
WORKSPACE="${WORKSPACE:-/workspace/automap_ws}"
LOG_DIR="${SCRIPT_DIR}/verify_logs"

# 创建日志目录
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/verify_$(date +%Y%m%d_%H%M%S).log"

# 打印标题
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# 步骤1: 环境验证
verify_environment() {
    print_header "步骤1: 环境验证"
    
    log_substep "检查ROS2..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        log_success "ROS2已安装: $(ros2 --version)"
    else
        log_fail "ROS2未安装"
        return 1
    fi
    
    log_substep "检查Python3..."
    if command -v python3 &> /dev/null; then
        log_success "Python3: $(python3 --version)"
    else
        log_fail "Python3未安装"
        return 1
    fi
    
    log_substep "检查colcon..."
    if command -v colcon &> /dev/null; then
        log_success "colcon: $(colcon --version)"
    else
        log_fail "colcon未安装"
        return 1
    fi
    
    log_substep "检查工作空间..."
    if [ -d "$WORKSPACE" ]; then
        log_success "工作空间存在: $WORKSPACE"
    else
        log_warn "工作空间不存在，将创建"
        mkdir -p "$WORKSPACE/src"
    fi
    
    log_success "步骤1完成"
}

# 步骤2: 数据验证
verify_data() {
    print_header "步骤2: 数据验证"
    
    log_substep "检查bag数据..."
    if [ -f "$BAG_FILE/metadata.yaml" ]; then
        log_success "ROS2数据存在: $BAG_FILE"
        
        # 检查数据大小
        local size=$(du -sh "$BAG_FILE" | cut -f1)
        log_info "数据大小: $size"
        
        # 检查话题列表
        log_substep "检查话题列表..."
        local topics=$(python3 -c "
import yaml
data = yaml.safe_load(open('$BAG_FILE/metadata.yaml'))
topics = data['rosbag2_bagfile_information']['topics_with_message_count']
for t in topics:
    print(f\"  {t['topic_metadata']['name']}: {t['message_count']}条\")
" 2>/dev/null || echo "无法解析话题列表")
        echo "$topics"
        
        # 检查关键话题
        log_substep "检查关键话题..."
        local key_topics=(
            "/os1_cloud_node1/points"
            "/imu/imu"
        )
        
        for topic in "${key_topics[@]}"; do
            if python3 -c "
import yaml
data = yaml.safe_load(open('$BAG_FILE/metadata.yaml'))
topics = [t['topic_metadata']['name'] for t in data['rosbag2_bagfile_information']['topics_with_message_count']]
exit(0 if '$topic' in topics else 1)
"; then
                log_success "话题存在: $topic"
            else
                log_fail "话题缺失: $topic"
                return 1
            fi
        done
    else
        log_fail "ROS2数据不存在: $BAG_FILE"
        return 1
    fi
    
    log_success "步骤2完成"
}

# 步骤3: 配置验证
verify_config() {
    print_header "步骤3: 配置验证"
    
    log_substep "检查配置文件..."
    if [ -f "$CONFIG" ]; then
        log_success "配置文件存在: $CONFIG"
    else
        log_fail "配置文件不存在: $CONFIG"
        return 1
    fi
    
    log_substep "验证YAML语法..."
    if python3 -c "import yaml; yaml.safe_load(open('$CONFIG')); print('语法正确')" 2>&1 | tee -a "$LOG_FILE"; then
        log_success "YAML语法正确"
    else
        log_fail "YAML语法错误"
        return 1
    fi
    
    log_substep "检查关键话题配置..."
    local lidar_topic=$(python3 -c "
import yaml
config = yaml.safe_load(open('$CONFIG'))
print(config.get('sensor', {}).get('lidar', {}).get('topic', 'NOT SET'))
")
    local imu_topic=$(python3 -c "
import yaml
config = yaml.safe_load(open('$CONFIG'))
print(config.get('sensor', {}).get('imu', {}).get('topic', 'NOT SET'))
")
    
    log_info "LiDAR话题: $lidar_topic"
    log_info "IMU话题: $imu_topic"
    
    if [ "$lidar_topic" = "NOT SET" ] || [ "$imu_topic" = "NOT SET" ]; then
        log_fail "关键话题未配置"
        return 1
    fi
    
    log_success "步骤3完成"
}

# 步骤4: 工作空间设置
setup_workspace() {
    print_header "步骤4: 工作空间设置"
    
    log_substep "创建工作空间结构..."
    mkdir -p "$WORKSPACE/src"
    
    log_substep "软链源码..."
    ln -sfn "$SCRIPT_DIR/automap_pro" "$WORKSPACE/src/automap_pro" || log_warn "软链已存在"
    
    if [ -d "$SCRIPT_DIR/fast-livo2-humble" ]; then
        ln -sfn "$SCRIPT_DIR/fast-livo2-humble" "$WORKSPACE/src/fast-livo2-humble" || log_warn "软链已存在"
    fi
    
    if [ -d "$SCRIPT_DIR/overlap_transformer_ros2" ]; then
        ln -sfn "$SCRIPT_DIR/overlap_transformer_ros2" "$WORKSPACE/src/overlap_transformer_ros2" || log_warn "软链已存在"
    fi
    
    if [ -d "$SCRIPT_DIR/HBA-main/HBA_ROS2" ]; then
        ln -sfn "$SCRIPT_DIR/HBA-main/HBA_ROS2" "$WORKSPACE/src/hba" || log_warn "软链已存在"
    fi
    
    log_substep "安装依赖..."
    cd "$WORKSPACE"
    source /opt/ros/humble/setup.bash
    rosdep update 2>&1 | head -5 || log_warn "rosdep更新失败"
    
    log_substep "安装依赖包..."
    rosdep install --from-paths src --ignore-src -r -y 2>&1 | tail -20 || log_warn "依赖安装失败"
    
    log_success "步骤4完成"
}

# 步骤5: 编译验证
compile_project() {
    print_header "步骤5: 编译验证"
    
    cd "$WORKSPACE"
    source /opt/ros/humble/setup.bash
    
    log_substep "编译automap_pro..."
    if colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "$LOG_FILE" | tail -50; then
        log_success "automap_pro编译成功"
    else
        log_fail "automap_pro编译失败"
        return 1
    fi
    
    log_substep "检查编译产物..."
    if [ -d "$WORKSPACE/install/automap_pro" ]; then
        log_success "安装目录存在"
        ls -lh "$WORKSPACE/install/automap_pro/lib/automap_pro" || true
    else
        log_fail "安装目录不存在"
        return 1
    fi
    
    log_substep "编译fast_livo..."
    if [ -d "$WORKSPACE/src/fast-livo2-humble" ]; then
        if colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee -a "$LOG_FILE" | tail -50; then
            log_success "fast_livo编译成功"
        else
            log_warn "fast_livo编译失败"
        fi
    else
        log_warn "fast_livo源码不存在"
    fi
    
    log_success "步骤5完成"
}

# 步骤6: 参数生成验证
verify_params() {
    print_header "步骤6: 参数生成验证"
    
    source "$WORKSPACE/install/setup.bash"
    
    log_substep "测试参数生成脚本..."
    cd "$SCRIPT_DIR/automap_pro/launch"
    
    if python3 -c "
from params_from_system_config import load_system_config, get_fast_livo2_params
config = load_system_config('$CONFIG')
params = get_fast_livo2_params(config)
print('参数生成成功')
print('关键参数:')
print('  lid_topic:', params.get('common', {}).get('lid_topic'))
print('  imu_topic:', params.get('common', {}).get('imu_topic'))
print('  img_topic:', params.get('common', {}).get('img_topic'))
" 2>&1 | tee -a "$LOG_FILE"; then
        log_success "参数生成测试通过"
    else
        log_fail "参数生成测试失败"
        return 1
    fi
    
    log_substep "生成fast_livo参数文件..."
    local params_file="$SCRIPT_DIR/logs/fast_livo_params.yaml"
    mkdir -p "$(dirname "$params_file")"
    
    if python3 -c "
from params_from_system_config import load_system_config, get_fast_livo2_params, write_fast_livo_params_file
config = load_system_config('$CONFIG')
write_fast_livo_params_file(config, '$params_file', '$CONFIG')
print('参数文件生成: $params_file')
" 2>&1 | tee -a "$LOG_FILE"; then
        log_success "参数文件生成成功"
        
        if [ -f "$params_file" ]; then
            log_info "参数文件大小: $(du -h "$params_file" | cut -f1)"
            log_info "参数文件行数: $(wc -l < "$params_file")"
        fi
    else
        log_fail "参数文件生成失败"
        return 1
    fi
    
    log_success "步骤6完成"
}

# 步骤7: Launch验证
verify_launch() {
    print_header "步骤7: Launch验证"
    
    source "$WORKSPACE/install/setup.bash"
    
    log_substep "检查launch文件..."
    local launch_file="$SCRIPT_DIR/automap_pro/launch/automap_offline.launch.py"
    
    if [ -f "$launch_file" ]; then
        log_success "launch文件存在: $launch_file"
    else
        log_fail "launch文件不存在: $launch_file"
        return 1
    fi
    
    log_substep "测试launch语法..."
    if python3 -m py_compile "$launch_file" 2>&1; then
        log_success "launch语法正确"
    else
        log_fail "launch语法错误"
        return 1
    fi
    
    log_substep "测试launch加载..."
    if python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR/automap_pro/launch')
from launch import LaunchDescription
from automap_offline import generate_launch_description
ld = generate_launch_description()
print('launch加载成功')
print('entities数量:', len(ld.entities))
" 2>&1 | tee -a "$LOG_FILE"; then
        log_success "launch加载测试通过"
    else
        log_warn "launch加载测试失败"
    fi
    
    log_success "步骤7完成"
}

# 步骤8: 建图执行
run_mapping() {
    print_header "步骤8: 建图执行"
    
    source "$WORKSPACE/install/setup.bash"
    
    log_substep "创建输出目录..."
    mkdir -p "$OUTPUT_DIR"
    
    log_substep "启动建图..."
    log_warn "建图过程可能需要较长时间，请耐心等待..."
    log_info "日志文件: $LOG_FILE"
    
    cd "$WORKSPACE"
    
    # 使用nohup后台运行
    log_info "启动命令:"
    echo "  ros2 launch automap_pro automap_offline.launch.py \\"
    echo "    config:=$CONFIG \\"
    echo "    bag_file:=$BAG_FILE \\"
    echo "    rate:=1.0 \\"
    echo "    use_rviz:=false \\"
    echo "    use_external_frontend:=true"
    
    echo ""
    log_warn "是否继续执行建图? (输入y继续，输入n跳过)"
    read -p "请选择 [y/n]: " -n 1 -r
    echo
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "跳过建图执行"
        return 0
    fi
    
    # 执行建图
    ros2 launch automap_pro automap_offline.launch.py \
        config:="$CONFIG" \
        bag_file:="$BAG_FILE" \
        rate:=1.0 \
        use_rviz:=false \
        use_external_frontend:=true \
        2>&1 | tee -a "$LOG_FILE"
    
    log_success "步骤8完成"
}

# 步骤9: 结果验证
verify_results() {
    print_header "步骤9: 结果验证"
    
    if [ ! -d "$OUTPUT_DIR" ]; then
        log_warn "输出目录不存在: $OUTPUT_DIR"
        return 0
    fi
    
    log_substep "检查输出目录结构..."
    log_info "输出目录: $OUTPUT_DIR"
    ls -lh "$OUTPUT_DIR" || true
    
    log_substep "检查地图文件..."
    if [ -f "$OUTPUT_DIR/map/global_map.pcd" ]; then
        local size=$(du -h "$OUTPUT_DIR/map/global_map.pcd" | cut -f1)
        log_success "全局地图: global_map.pcd ($size)"
    else
        log_warn "全局地图不存在: global_map.pcd"
    fi
    
    if [ -f "$OUTPUT_DIR/map/global_map.ply" ]; then
        local size=$(du -h "$OUTPUT_DIR/map/global_map.ply" | cut -f1)
        log_success "全局地图: global_map.ply ($size)"
    else
        log_warn "全局地图不存在: global_map.ply"
    fi
    
    log_substep "检查轨迹文件..."
    if [ -f "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt" ]; then
        local lines=$(wc -l < "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt")
        local size=$(du -h "$OUTPUT_DIR/trajectory/optimized_trajectory_tum.txt" | cut -f1)
        log_success "轨迹文件: optimized_trajectory_tum.txt ($lines行, $size)"
    else
        log_warn "轨迹文件不存在"
    fi
    
    log_substep "检查子图文件..."
    if [ -d "$OUTPUT_DIR/submaps" ]; then
        local count=$(find "$OUTPUT_DIR/submaps" -name "*.pcd" | wc -l)
        log_success "子图数量: $count"
    else
        log_warn "子图目录不存在"
    fi
    
    log_success "步骤9完成"
}

# 主函数
main() {
    print_header "AutoMap-Pro 逐环节验证"
    
    log_info "脚本目录: $SCRIPT_DIR"
    log_info "配置文件: $CONFIG"
    log_info "数据文件: $BAG_FILE"
    log_info "输出目录: $OUTPUT_DIR"
    log_info "工作空间: $WORKSPACE"
    log_info "日志文件: $LOG_FILE"
    
    echo ""
    log_info "开始验证流程..."
    
    verify_environment
    verify_data
    verify_config
    setup_workspace
    compile_project
    verify_params
    verify_launch
    run_mapping
    verify_results
    
    print_header "验证完成"
    log_info "日志文件: $LOG_FILE"
    log_info "查看日志: cat $LOG_FILE"
}

# 运行主函数
main "$@"
