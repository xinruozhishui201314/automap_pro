#!/bin/bash
# 修复后的重新编译和测试脚本

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $@"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $@"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $@"
}

# 1. 确认修改已同步
echo "========================================"
echo "确认 launch 文件修改"
echo "========================================"

if grep -q "移除固定 remappings" /home/wqs/Documents/github/automap_pro/automap_pro/launch/automap_offline.launch.py; then
    log_info "✓ automap_offline.launch.py 已修改"
else
    log_error "✗ automap_offline.launch.py 未修改，请确认文件内容"
    exit 1
fi

if grep -q "移除固定 remappings" /home/wqs/Documents/github/automap_pro/automap_pro/launch/automap_online.launch.py; then
    log_info "✓ automap_online.launch.py 已修改"
else
    log_error "✗ automap_online.launch.py 未修改，请确认文件内容"
    exit 1
fi

echo ""
echo "========================================"
echo "重新运行建图（Docker）"
echo "========================================"

cd /home/wqs/Documents/github/automap_pro

# 2. 运行建图
./run_full_mapping_docker.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2

echo ""
log_info "建图完成，检查日志:"
echo "  - 主日志: /home/wqs/Documents/github/automap_pro/logs/full_mapping_*.log"
echo "  - Launch 日志: /home/wqs/Documents/github/automap_pro/logs/launch_*.d/"
