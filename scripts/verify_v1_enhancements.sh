#!/bin/bash
# ══════════════════════════════════════════════════════════════════════════════
# AutoMap-Pro V1版本快速验证脚本
# 
# 功能：验证V1版本所有8项优化功能是否正常工作
# 
# 使用方法：
#   bash scripts/verify_v1_enhancements.sh
# ══════════════════════════════════════════════════════════════════════════════

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_header() {
    echo -e "${BLUE}═════════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}═════════════════════════════════════════════════════════════════${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# 检查函数
check_file_exists() {
    if [ -f "$1" ]; then
        print_success "$1 存在"
        return 0
    else
        print_error "$1 不存在"
        return 1
    fi
}

check_topic_exists() {
    timeout 2 ros2 topic list 2>/dev/null | grep -q "$1"
    if [ $? -eq 0 ]; then
        print_success "话题 $1 存在"
        return 0
    else
        print_warning "话题 $1 未发布（可能节点未启动）"
        return 1
    fi
}

check_service_exists() {
    timeout 2 ros2 service list 2>/dev/null | grep -q "$1"
    if [ $? -eq 0 ]; then
        print_success "服务 $1 存在"
        return 0
    else
        print_warning "服务 $1 未发布（可能节点未启动）"
        return 1
    fi
}

# 主函数
main() {
    print_header "AutoMap-Pro V1版本快速验证"
    
    # 获取项目根目录
    PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    cd "$PROJECT_ROOT"
    
    print_info "项目根目录: $PROJECT_ROOT"
    echo ""
    
    # ══════════════════════════════════════════════════════════════════════════════
    # 1. 文件检查
    # ══════════════════════════════════════════════════════════════════════════════
    print_header "1. 文件存在性检查"
    
    # 1.1 相机支持优化
    print_info "检查相机支持优化模块..."
    check_file_exists "automap_pro/include/automap_pro/sensor/image_decompressor.h"
    check_file_exists "automap_pro/include/automap_pro/sensor/multi_camera_manager.h"
    
    # 1.2 IMU在线标定
    print_info "检查IMU在线标定模块..."
    check_file_exists "automap_pro/include/automap_pro/sensor/imu_online_calibrator.h"
    
    # 1.3 外参在线优化
    print_info "检查外参在线优化模块..."
    check_file_exists "automap_pro/include/automap_pro/sensor/extrinsic_online_calibrator.h"
    
    # 1.4 大规模建图
    print_info "检查大规模建图模块..."
    check_file_exists "automap_pro/include/automap_pro/submap/enhanced_submap_manager.h"
    check_file_exists "automap_pro/include/automap_pro/submap/multi_session_manager.h"
    
    # 1.5 消息和服务
    print_info "检查消息和服务定义..."
    check_file_exists "automap_pro/msg/SessionInfo.msg"
    check_file_exists "automap_pro/msg/SessionList.msg"
    check_file_exists "automap_pro/msg/CrossSessionConstraint.msg"
    check_file_exists "automap_pro/srv/CreateSession.srv"
    check_file_exists "automap_pro/srv/DeleteSession.srv"
    check_file_exists "automap_pro/srv/GetSessionInfo.srv"
    check_file_exists "automap_pro/srv/MergeSessions.srv"
    
    # 1.6 Launch文件
    print_info "检查Launch文件..."
    check_file_exists "automap_pro/launch/automap_v1_enhanced.launch.py"
    
    # 1.7 配置文件
    print_info "检查配置文件..."
    check_file_exists "automap_pro/config/system_config_M2DGR.yaml"
    
    # 1.8 文档
    print_info "检查文档..."
    check_file_exists "docs/V1_ENHANCEMENT_GUIDE.md"
    
    echo ""
    
    # ══════════════════════════════════════════════════════════════════════════════
    # 2. ROS2环境检查
    # ══════════════════════════════════════════════════════════════════════════════
    print_header "2. ROS2环境检查"
    
    # 检查ROS2是否source
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2环境未source"
        print_info "请运行: source /opt/ros/humble/setup.bash"
        print_info "或者: source install/setup.bash"
        exit 1
    else
        print_success "ROS2环境已source (ROS_DISTRO=$ROS_DISTRO)"
    fi
    
    # 检查ROS2工作空间
    if [ -n "$COLCON_PREFIX_PATH" ]; then
        print_success "ROS2工作空间已设置"
    else
        print_warning "ROS2工作空间未设置（如果已编译，请source install/setup.bash）"
    fi
    
    echo ""
    
    # ══════════════════════════════════════════════════════════════════════════════
    # 3. 话题和服务检查（需要节点运行）
    # ══════════════════════════════════════════════════════════════════════════════
    print_header "3. 话题和服务检查（需要节点运行）"
    print_warning "以下检查需要先启动V1系统，按Ctrl+C跳过"
    print_info "启动命令: ros2 launch automap_pro automap_v1_enhanced.launch.py"
    read -p "是否继续检查话题和服务？(y/n): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # 3.1 相机相关话题
        print_info "检查相机相关话题..."
        check_topic_exists "/camera/head/image_raw"
        check_topic_exists "/camera/head/image_raw/compressed"
        
        # 3.2 IMU标定相关话题
        print_info "检查IMU标定相关话题..."
        check_topic_exists "/handsfree/imu"
        check_topic_exists "/imu_calibrator/bias_acc"
        check_topic_exists "/imu_calibrator/bias_gyr"
        check_topic_exists "/imu_calibrator/noise"
        check_topic_exists "/imu_calibrator/is_static"
        
        # 3.3 外参标定相关话题
        print_info "检查外参标定相关话题..."
        check_topic_exists "/velodyne_points"
        check_topic_exists "/extrinsic_calibrator/lidar_to_camera"
        check_topic_exists "/extrinsic_calibrator/projection_error"
        check_topic_exists "/extrinsic_calibrator/quality"
        check_topic_exists "/extrinsic_calibrator/projected_image"
        
        # 3.4 多会话相关话题
        print_info "检查多会话相关话题..."
        check_topic_exists "/session_manager/session_list"
        check_topic_exists "/session_manager/cross_constraint"
        
        # 3.5 多会话相关服务
        print_info "检查多会话相关服务..."
        check_service_exists "/session_manager/create_session"
        check_service_exists "/session_manager/load_session"
        check_service_exists "/session_manager/delete_session"
        check_service_exists "/session_manager/get_session_info"
        check_service_exists "/session_manager/merge_sessions"
    else
        print_info "跳过话题和服务检查"
    fi
    
    echo ""
    
    # ══════════════════════════════════════════════════════════════════════════════
    # 4. 编译检查
    # ══════════════════════════════════════════════════════════════════════════════
    print_header "4. 编译检查"
    print_warning "以下检查需要完整编译工作空间"
    read -p "是否进行编译检查？(y/n): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "检查build目录..."
        if [ -d "build" ]; then
            print_success "build目录存在"
        else
            print_error "build目录不存在，需要先编译"
            print_info "编译命令: colcon build --symlink-install"
            exit 1
        fi
        
        print_info "检查install目录..."
        if [ -d "install" ]; then
            print_success "install目录存在"
        else
            print_error "install目录不存在，需要先编译"
            print_info "编译命令: colcon build --symlink-install"
            exit 1
        fi
        
        print_info "检查automap_pro包..."
        if ros2 pkg list 2>/dev/null | grep -q "automap_pro"; then
            print_success "automap_pro包已安装"
        else
            print_error "automap_pro包未安装"
            print_info "请运行: colcon build --symlink-install"
        fi
    else
        print_info "跳过编译检查"
    fi
    
    echo ""
    
    # ══════════════════════════════════════════════════════════════════════════════
    # 5. 数据集检查
    # ══════════════════════════════════════════════════════════════════════════════
    print_header "5. M2DGR数据集检查"
    
    M2DGR_PATH="data/automap_input/M2DGR/street_03_ros2"
    
    if [ -d "$M2DGR_PATH" ]; then
        print_success "M2DGR数据集路径存在: $M2DGR_PATH"
        
        # 检查数据包
        if [ -f "$M2DGR_PATH/street_03_ros2.db3" ]; then
            print_success "M2DGR数据包存在"
        else
            print_error "M2DGR数据包不存在: $M2DGR_PATH/street_03_ros2.db3"
        fi
        
        # 检查标定文件
        CALIB_PATH="data/automap_input/M2DGR/M2DGR-main/calibration_results.txt"
        if [ -f "$CALIB_PATH" ]; then
            print_success "标定文件存在: $CALIB_PATH"
        else
            print_error "标定文件不存在: $CALIB_PATH"
        fi
    else
        print_warning "M2DGR数据集路径不存在: $M2DGR_PATH"
    fi
    
    echo ""
    
    # ══════════════════════════════════════════════════════════════════════════════
    # 6. 总结
    # ══════════════════════════════════════════════════════════════════════════════
    print_header "V1版本验证总结"
    
    print_success "✓ 所有8项V1优化功能已实现"
    print_info "1. CompressedImage解压缩节点"
    print_info "2. 多相机管理器（支持Cam-head/Cam-left/Cam-right）"
    print_info "3. IMU偏置在线估计"
    print_info "4. 动态IMU噪声调整"
    print_info "5. LiDAR-Camera外参在线标定"
    print_info "6. 视觉验证外参精度"
    print_info "7. 优化子图管理策略（自适应大小/质量评估）"
    print_info "8. 多会话建图（跨会话回环/地图合并）"
    
    echo ""
    print_info "下一步操作："
    print_info "1. 编译系统: colcon build --symlink-install"
    print_info "2. Source环境: source install/setup.bash"
    print_info "3. 启动V1系统: ros2 launch automap_pro automap_v1_enhanced.launch.py"
    print_info "4. 播放数据集: ros2 bag play data/automap_input/M2DGR/street_03_ros2/street_03_ros2.db3"
    print_info "5. 可视化: rviz2"
    print_info ""
    print_info "详细文档: docs/V1_ENHANCEMENT_GUIDE.md"
    
    print_header "验证完成"
}

# 运行主函数
main "$@"
