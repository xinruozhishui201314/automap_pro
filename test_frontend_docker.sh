#!/bin/bash
# AutoMap-Pro 前端验证脚本 (Docker容器内)
# 功能：在ROS2 Humble Docker容器内测试验证Fast-LIVO2前端里程计

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $@"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $@"; }
log_error() { echo -e "${RED}[ERROR]${NC} $@"; }
log_step() { echo -e "${CYAN}[STEP]${NC} $@"; }
log_substep() { echo -e "${CYAN}  →${NC} $@"; }
log_success() { echo -e "${GREEN}[✓]${NC} $@"; }

# 配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="${CONFIG:-automap_pro/config/system_config.yaml}"
BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2}"
WORKSPACE="${WORKSPACE:-/workspace/automap_ws}"
LOG_DIR="${SCRIPT_DIR}/test_frontend_logs"
OUTPUT_DIR="${SCRIPT_DIR}/test_frontend_output"

# 创建日志目录
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/frontend_test_$(date +%Y%m%d_%H%M%S).log"

# 打印标题
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# 检查是否在Docker容器内
check_docker_env() {
    print_header "步骤1: 检查Docker环境"
    
    if [ -f /.dockerenv ]; then
        log_success "✓ 在Docker容器内运行"
        log_info "容器ID: $(cat /proc/self/cgroup | head -n 1 | cut -d/ -f3)"
    else
        log_warn "⚠ 不在Docker容器内，某些测试可能失败"
    fi
    
    log_info "当前用户: $(whoami)"
    log_info "工作目录: $(pwd)"
    log_info "Python版本: $(python3 --version)"
}

# 检查ROS2环境
check_ros2_env() {
    print_header "步骤2: 检查ROS2环境"
    
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        log_success "✓ ROS2 Humble 已安装"
        source /opt/ros/humble/setup.bash
        log_info "ROS2版本: $(ros2 --version)"
        log_info "ROS_DISTRO: $ROS_DISTRO"
    else
        log_error "✗ ROS2 Humble 未安装"
        return 1
    fi
}

# 检查工作空间
check_workspace() {
    print_header "步骤3: 检查工作空间"
    
    if [ -d "$WORKSPACE" ]; then
        log_success "✓ 工作空间存在: $WORKSPACE"
    else
        log_warn "⚠ 工作空间不存在: $WORKSPACE"
        mkdir -p "$WORKSPACE/src"
        log_info "已创建工作空间: $WORKSPACE"
    fi
    
    # 检查是否需要软链
    if [ -d "$SCRIPT_DIR/automap_pro" ] && [ ! -L "$WORKSPACE/src/automap_pro" ]; then
        log_substep "创建automap_pro软链..."
        ln -sfn "$SCRIPT_DIR/automap_pro" "$WORKSPACE/src/automap_pro"
        log_success "✓ 软链已创建"
    fi
    
    if [ -d "$SCRIPT_DIR/fast-livo2-humble" ] && [ ! -L "$WORKSPACE/src/fast-livo2-humble" ]; then
        log_substep "创建fast-livo2-humble软链..."
        ln -sfn "$SCRIPT_DIR/fast-livo2-humble" "$WORKSPACE/src/fast_livo2-humble"
        log_success "✓ 软链已创建"
    fi
    
    # 检查install目录
    if [ -d "$WORKSPACE/install" ]; then
        log_success "✓ Install目录存在"
        source "$WORKSPACE/install/setup.bash"
    else
        log_warn "⚠ Install目录不存在，需要先编译"
    fi
}

# 检查配置文件
check_config() {
    print_header "步骤4: 检查配置文件"
    
    CONFIG_ABS="$CONFIG"
    if [[ "$CONFIG" != /* ]]; then
        CONFIG_ABS="$SCRIPT_DIR/$CONFIG"
    fi
    
    if [ -f "$CONFIG_ABS" ]; then
        log_success "✓ 配置文件存在: $CONFIG_ABS"
    else
        log_error "✗ 配置文件不存在: $CONFIG_ABS"
        return 1
    fi
    
    # 验证YAML语法
    if python3 -c "import yaml; yaml.safe_load(open('$CONFIG_ABS')); print('语法正确')" 2>&1 | tee -a "$LOG_FILE"; then
        log_success "✓ YAML语法正确"
    else
        log_error "✗ YAML语法错误"
        return 1
    fi
}

# 检查bag数据
check_bag_data() {
    print_header "步骤5: 检查Bag数据"
    
    BAG_ABS="$BAG_FILE"
    if [[ "$BAG_FILE" != /* ]]; then
        BAG_ABS="$SCRIPT_DIR/$BAG_FILE"
    fi
    
    if [ -f "$BAG_ABS/metadata.yaml" ]; then
        log_success "✓ ROS2 Bag存在: $BAG_ABS"
        
        local size=$(du -sh "$BAG_ABS" | cut -f1)
        log_info "数据大小: $size"
        
        # 检查关键话题
        log_substep "检查关键话题..."
        python3 -c "
import yaml
data = yaml.safe_load(open('$BAG_ABS/metadata.yaml'))
topics = data['rosbag2_bagfile_information']['topics_with_message_count']

print('话题列表:')
for t in topics:
    print(f'  {t[\"topic_metadata\"][\"name\"]}: {t[\"message_count\"]}条')
" 2>&1 | tee -a "$LOG_FILE" | head -20
        
        # 检查LiDAR和IMU话题
        if python3 -c "
import yaml
data = yaml.safe_load(open('$BAG_ABS/metadata.yaml'))
topics = [t['topic_metadata']['name'] for t in data['rosbag2_bagfile_information']['topics_with_message_count']]
exit(0 if '/os1_cloud_node1/points' in topics else 1)
" 2>&1; then
            log_success "✓ LiDAR话题存在: /os1_cloud_node1/points"
        else
            log_warn "⚠ LiDAR话题不存在"
        fi
        
        if python3 -c "
import yaml
data = yaml.safe_load(open('$BAG_ABS/metadata.yaml'))
topics = [t['topic_metadata']['name'] for t in data['rosbag2_bagfile_information']['topics_with_message_count']]
exit(0 if '/imu/imu' in topics else 1)
" 2>&1; then
            log_success "✓ IMU话题存在: /imu/imu"
        else
            log_warn "⚠ IMU话题不存在"
        fi
    else
        log_error "✗ ROS2 Bag不存在: $BAG_ABS"
        return 1
    fi
}

# 检查Fast-LIVO2编译
check_fast_livo() {
    print_header "步骤6: 检查Fast-LIVO2编译"
    
    local fast_livo_exe="$WORKSPACE/install/fast_livo/lib/fast_livo/fastlivo_mapping"
    if [ ! -f "$fast_livo_exe" ]; then
        fast_livo_exe="$WORKSPACE/install/fast_livo/bin/fastlivo_mapping"
    fi
    
    if [ -f "$fast_livo_exe" ]; then
        log_success "✓ Fast-LIVO2已编译: $fast_livo_exe"
        
        # 检查可执行权限
        if [ -x "$fast_livo_exe" ]; then
            log_success "✓ 可执行权限正常"
        else
            log_warn "⚠ 没有可执行权限"
            chmod +x "$fast_livo_exe"
        fi
        
        # 检查binary是否是修复后的版本
        log_substep "检查parameter ''修复..."
        if strings "$fast_livo_exe" | grep -q "automatically_declare_parameters_from_overrides"; then
            log_success "✓ 包含parameter ''修复代码"
        else
            log_warn "⚠ 可能未包含parameter ''修复代码"
        fi
    else
        log_warn "⚠ Fast-LIVO2未编译"
        log_info "运行以下命令编译:"
        log_info "  cd $WORKSPACE && source /opt/ros/humble/setup.bash && colcon build --packages-select fast_livo"
        return 1
    fi
}

# 测试参数生成
test_params_generation() {
    print_header "步骤7: 测试参数生成"
    
    source "$WORKSPACE/install/setup.bash"
    
    local launch_dir="$SCRIPT_DIR/automap_pro/launch"
    
    if [ -f "$launch_dir/params_from_system_config.py" ]; then
        log_success "✓ 参数生成脚本存在"
        
        # 测试导入
        log_substep "测试Python导入..."
        if python3 -c "
import sys
sys.path.insert(0, '$launch_dir')
from params_from_system_config import (
    load_system_config,
    get_fast_livo2_params,
    get_overlap_transformer_params,
    get_hba_params
)
print('✓ 所有函数导入成功')
" 2>&1 | tee -a "$LOG_FILE"; then
            log_success "✓ 导入成功"
        else
            log_error "✗ 导入失败"
            return 1
        fi
        
        # 测试参数生成
        log_substep "测试参数生成..."
        if python3 -c "
import sys
sys.path.insert(0, '$launch_dir')
from params_from_system_config import load_system_config, get_fast_livo2_params

config = load_system_config('$CONFIG_ABS')
params = get_fast_livo2_params(config)

print('✓ 参数生成成功')
print('关键参数:')
print('  lid_topic:', params.get('common', {}).get('lid_topic'))
print('  imu_topic:', params.get('common', {}).get('imu_topic'))
print('  img_topic:', params.get('common', {}).get('img_topic'))
print('  voxel_size:', params.get('lio', {}).get('voxel_size'))
print('  max_iterations:', params.get('lio', {}).get('max_iterations'))
" 2>&1 | tee -a "$LOG_FILE"; then
            log_success "✓ 参数生成成功"
        else
            log_error "✗ 参数生成失败"
            return 1
        fi
        
        # 生成参数文件
        log_substep "生成参数文件..."
        local params_file="$LOG_DIR/test_fast_livo_params.yaml"
        
        if python3 -c "
import sys
sys.path.insert(0, '$launch_dir')
from params_from_system_config import load_system_config, get_fast_livo2_params, write_fast_livo_params_file

config = load_system_config('$CONFIG_ABS')
write_fast_livo_params_file(config, '$params_file', '$CONFIG_ABS')
print('✓ 参数文件生成: $params_file')
" 2>&1 | tee -a "$LOG_FILE"; then
            log_success "✓ 参数文件生成成功"
            
            if [ -f "$params_file" ]; then
                log_info "参数文件大小: $(du -h "$params_file" | cut -f1)"
                log_info "参数文件行数: $(wc -l < "$params_file")"
                
                # 检查是否有空键
                if grep -qE "^[[:space:]]*:[[:space:]]*$" "$params_file"; then
                    log_warn "⚠ 发现空键，可能导致parameter ''错误"
                else
                    log_success "✓ 没有发现空键"
                fi
            fi
        else
            log_error "✗ 参数文件生成失败"
            return 1
        fi
    else
        log_error "✗ 参数生成脚本不存在"
        return 1
    fi
}

# 测试节点可执行性
test_node_executable() {
    print_header "步骤8: 测试节点可执行性"
    
    source "$WORKSPACE/install/setup.bash"
    
    # 测试automap_system_node
    local automap_node="$WORKSPACE/install/automap_pro/lib/automap_pro/automap_system_node"
    if [ -f "$automap_node" ]; then
        log_success "✓ automap_system_node存在: $automap_node"
        
        if [ -x "$automap_node" ]; then
            log_success "✓ 可执行权限正常"
        else
            chmod +x "$automap_node"
            log_success "✓ 已设置可执行权限"
        fi
    else
        log_warn "⚠ automap_system_node不存在"
    fi
    
    # 测试fastlivo_mapping
    local fast_livo_exe="$WORKSPACE/install/fast_livo/lib/fast_livo/fastlivo_mapping"
    if [ ! -f "$fast_livo_exe" ]; then
        fast_livo_exe="$WORKSPACE/install/fast_livo/bin/fastlivo_mapping"
    fi
    
    if [ -f "$fast_livo_exe" ]; then
        log_success "✓ fastlivo_mapping存在: $fast_livo_exe"
        
        if [ -x "$fast_livo_exe" ]; then
            log_success "✓ 可执行权限正常"
        else
            chmod +x "$fast_livo_exe"
            log_success "✓ 已设置可执行权限"
        fi
        
        # 测试--help参数
        log_substep "测试fastlivo_mapping --help..."
        if timeout 5 "$fast_livo_exe" --help 2>&1 | head -20 | tee -a "$LOG_FILE"; then
            log_success "✓ fastlivo_mapping可以执行"
        else
            log_warn "⚠ fastlivo_mapping执行超时或失败"
        fi
    else
        log_warn "⚠ fastlivo_mapping不存在"
    fi
}

# 测试launch文件
test_launch_file() {
    print_header "步骤9: 测试Launch文件"
    
    source "$WORKSPACE/install/setup.bash"
    
    local launch_file="$SCRIPT_DIR/automap_pro/launch/automap_offline.launch.py"
    
    if [ -f "$launch_file" ]; then
        log_success "✓ Launch文件存在: $launch_file"
        
        # 测试Python语法
        if python3 -m py_compile "$launch_file" 2>&1; then
            log_success "✓ Launch文件语法正确"
        else
            log_error "✗ Launch文件语法错误"
            return 1
        fi
        
        # 测试导入
        if python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR/automap_pro/launch')
from automap_offline import generate_launch_description
ld = generate_launch_description()
print('✓ Launch文件可以导入')
print('Entities数量:', len(ld.entities))
" 2>&1 | tee -a "$LOG_FILE"; then
            log_success "✓ Launch文件可以导入"
        else
            log_error "✗ Launch文件导入失败"
            return 1
        fi
    else
        log_error "✗ Launch文件不存在"
        return 1
    fi
}

# 检查ROS2话题
check_ros2_topics() {
    print_header "步骤10: 检查ROS2话题"
    
    source /opt/ros/humble/setup.bash
    
    # 检查ros2命令
    if command -v ros2 &> /dev/null; then
        log_success "✓ ros2命令可用"
    else
        log_error "✗ ros2命令不可用"
        return 1
    fi
    
    # 检查话题列表（如果没有rosbag播放）
    log_substep "检查当前话题列表..."
    local topics=$(ros2 topic list 2>&1)
    if [ -n "$topics" ]; then
        log_info "当前话题数量: $(echo "$topics" | wc -l)"
    else
        log_info "当前没有发布的话题"
    fi
    
    # 检查节点列表
    log_substep "检查当前节点列表..."
    local nodes=$(ros2 node list 2>&1)
    if [ -n "$nodes" ]; then
        log_info "当前节点数量: $(echo "$nodes" | wc -l)"
    else
        log_info "当前没有运行的节点"
    fi
}

# 测试短时间建图
test_short_mapping() {
    print_header "步骤11: 测试短时间建图"
    
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE/install/setup.bash"
    
    log_warn "即将启动短时间建图测试(约30秒)..."
    log_info "日志文件: $LOG_FILE"
    log_info "输出目录: $OUTPUT_DIR"
    
    # 创建输出目录
    mkdir -p "$OUTPUT_DIR"
    
    log_warn "是否继续测试? (输入y继续，输入n跳过)"
    read -p "请选择 [y/n]: " -n 1 -r
    echo
    
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        log_info "跳过短时间建图测试"
        return 0
    fi
    
    log_step "启动ros2 bag play (限制10秒)..."
    
    # 启动ros2 bag play（只播放10秒）
    timeout 15 ros2 bag play "$BAG_ABS" --rate 1.0 --clock &
    local bag_pid=$!
    
    sleep 2
    
    # 启动fastlivo_mapping（使用参数文件）
    local params_file="$LOG_DIR/test_fast_livo_params.yaml"
    
    log_step "启动fastlivo_mapping..."
    timeout 12 ros2 run fast_livo fastlivo_mapping \
        --ros-args \
        -r __node:=laserMapping \
        -p use_sim_time:=true \
        --params-file "$params_file" \
        2>&1 | tee -a "$LOG_FILE" &
    local fast_livo_pid=$!
    
    # 监控进程
    log_info "监控进程(10秒)..."
    for i in {1..10}; do
        sleep 1
        if ! kill -0 $fast_livo_pid 2>/dev/null; then
            log_warn "fastlivo_mapping进程已退出"
            break
        fi
        echo -n "."
    done
    echo ""
    
    # 停止进程
    log_step "停止进程..."
    kill $bag_pid 2>/dev/null || true
    kill $fast_livo_pid 2>/dev/null || true
    wait $bag_pid 2>/dev/null || true
    wait $fast_livo_pid 2>/dev/null || true
    
    log_success "✓ 短时间建图测试完成"
    
    # 检查是否有错误
    if grep -q "ERROR" "$LOG_FILE"; then
        log_warn "⚠ 日志中发现错误，请检查: $LOG_FILE"
    else
        log_success "✓ 没有发现错误"
    fi
}

# 显示测试结果
show_results() {
    print_header "测试结果汇总"
    
    log_info "日志文件: $LOG_FILE"
    log_info "输出目录: $OUTPUT_DIR"
    
    log_step "检查日志摘要..."
    if [ -f "$LOG_FILE" ]; then
        local errors=$(grep -c "ERROR" "$LOG_FILE" || echo 0)
        local warnings=$(grep -c "WARN" "$LOG_FILE" || echo 0)
        
        log_info "错误数量: $errors"
        log_info "警告数量: $warnings"
        
        if [ "$errors" -eq 0 ]; then
            log_success "✓ 没有错误"
        else
            log_warn "⚠ 发现 $errors 个错误"
            log_info "查看错误: grep ERROR $LOG_FILE"
        fi
        
        if [ "$warnings" -eq 0 ]; then
            log_success "✓ 没有警告"
        else
            log_warn "⚠ 发现 $warnings 个警告"
        fi
    fi
}

# 主函数
main() {
    print_header "AutoMap-Pro 前端验证 (Docker容器内)"
    
    log_info "脚本目录: $SCRIPT_DIR"
    log_info "配置文件: $CONFIG"
    log_info "Bag文件: $BAG_FILE"
    log_info "工作空间: $WORKSPACE"
    log_info "日志文件: $LOG_FILE"
    
    echo ""
    
    # 执行测试
    check_docker_env
    check_ros2_env
    check_workspace
    check_config
    check_bag_data
    check_fast_livo
    test_params_generation
    test_node_executable
    test_launch_file
    check_ros2_topics
    test_short_mapping
    
    # 显示结果
    show_results
    
    print_header "前端验证完成"
    
    log_info "下一步操作:"
    log_info "  1. 查看日志: cat $LOG_FILE"
    log_info "  2. 完整建图: ./run_full_mapping_docker.sh -b $BAG_FILE"
    log_info "  3. 逐环节验证: ./verify_mapping_pipeline.sh"
}

# 运行主函数
main "$@"
