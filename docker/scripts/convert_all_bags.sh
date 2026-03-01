#!/bin/bash
# 批量转换ROS1 bag到ROS2格式的Shell脚本

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 默认参数
INPUT_DIR="${1:-/workspace/data/automap_input}"
OUTPUT_BASE="${2:-/workspace/data/automap_input}"
VERBOSE="${3:-false}"

# 显示帮助信息
show_help() {
    cat << EOF
批量转换 ROS1 Bag 到 ROS2 格式

用法: $0 [input_dir] [output_base] [verbose]

参数:
    input_dir       输入目录（默认: /workspace/data/automap_input）
    output_base     输出基目录（默认: /workspace/data/automap_input）
    verbose         是否显示详细输出（默认: false）

示例:
    # 使用默认参数
    $0

    # 指定输入输出目录
    $0 /data/bags /data/ros2_bags

    # 详细模式
    $0 /data/bags /data/ros2_bags true

环境变量:
    INPUT_DIR        输入目录
    OUTPUT_BASE      输出基目录
    VERBOSE          详细输出

注意:
    - 转换后的文件将放在子目录中，例如:
      input_dir/nya_02.bag -> output_base/nya_02/ros2/
    - 原始bag文件不会被修改
    - 需要ROS2环境正确配置

EOF
}

# 检查ROS2环境
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: ros2 命令不可用${NC}"
        echo -e "${YELLOW}请确保已 source /opt/ros/humble/setup.bash${NC}"
        return 1
    fi
    
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${RED}错误: ROS2 Humble 未安装${NC}"
        return 1
    fi
    
    echo -e "${GREEN}✓ ROS2 环境已配置${NC}"
    return 0
}

# 转换单个bag
convert_single_bag() {
    local input_bag="$1"
    local output_dir="$2"
    
    local bag_name=$(basename "$input_bag" .bag)
    
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}处理: ${bag_name}.bag${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # 检查输入文件
    if [ ! -f "$input_bag" ]; then
        echo -e "${RED}✗ 输入文件不存在: $input_bag${NC}"
        return 1
    fi
    
    # 检查文件格式
    local file_type=$(file "$input_bag" | cut -d: -f2)
    if echo "$file_type" | grep -q "SQLite"; then
        echo -e "${YELLOW}⚠ 已经是ROS2格式，跳过${NC}"
        return 0
    elif ! echo "$file_type" | grep -q "data"; then
        echo -e "${RED}✗ 无法识别的格式: $file_type${NC}"
        return 1
    fi
    
    # 创建输出目录
    mkdir -p "$output_dir"
    
    # 转换
    echo -e "${GREEN}开始转换...${NC}"
    
    if ros2 bag convert "$input_bag" "$output_dir" 2>&1; then
        echo -e "${GREEN}✓ 转换成功: $output_dir${NC}"
        
        # 显示转换后的信息
        if [ "$VERBOSE" = "true" ]; then
            echo -e "\n${YELLOW}转换后的bag信息:${NC}"
            ros2 bag info "$output_dir" | head -30
        fi
        
        return 0
    else
        echo -e "${RED}✗ 转换失败: $input_bag${NC}"
        return 1
    fi
}

# 主函数
main() {
    # 显示帮助
    if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
        show_help
        exit 0
    fi
    
    # 检查环境
    if ! check_ros2; then
        exit 1
    fi
    
    # 检查输入目录
    if [ ! -d "$INPUT_DIR" ]; then
        echo -e "${RED}错误: 输入目录不存在: $INPUT_DIR${NC}"
        exit 1
    fi
    
    # 创建输出基目录
    mkdir -p "$OUTPUT_BASE"
    
    # 查找所有bag文件
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}批量转换 ROS1 Bag${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}输入目录:${NC} $INPUT_DIR"
    echo -e "${GREEN}输出基目录:${NC} $OUTPUT_BASE"
    
    local bag_files=($(find "$INPUT_DIR" -maxdepth 2 -name "*.bag"))
    local bag_count=${#bag_files[@]}
    
    if [ "$bag_count" -eq 0 ]; then
        echo -e "${RED}✗ 未找到bag文件: $INPUT_DIR${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}找到 $bag_count 个bag文件${NC}"
    echo -e "${BLUE}========================================${NC}\n"
    
    # 逐个转换
    local success_count=0
    local failed_count=0
    
    for i in "${!bag_files[@]}"; do
        local bag_file="${bag_files[$i]}"
        local bag_name=$(basename "$bag_file" .bag)
        
        # 输出目录为 bag文件名
        local output_dir="$OUTPUT_BASE/$bag_name"
        
        echo -e "${YELLOW}[$((i+1))/$bag_count] ${bag_name}.bag${NC}"
        
        if convert_single_bag "$bag_file" "$output_dir"; then
            ((success_count++))
        else
            ((failed_count++))
        fi
    done
    
    # 总结
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}转换完成${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo -e "${GREEN}成功: $success_count / $bag_count${NC}"
    echo -e "${RED}失败: $failed_count / $bag_count${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    # 显示输出目录结构
    if [ "$success_count" -gt 0 ]; then
        echo -e "\n${YELLOW}输出目录结构:${NC}"
        find "$OUTPUT_BASE" -maxdepth 2 -type d | sort
    fi
    
    # 返回状态
    if [ "$failed_count" -gt 0 ]; then
        exit 1
    fi
}

# 执行主函数
main "$@"
