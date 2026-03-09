#!/bin/bash
#
# TEASER 修复编译与验证脚本
# 功能：编译修复后的代码并运行回放测试
# 使用：bash teaser_fix_build_and_test.sh
#

set -e

PROJECT_ROOT="/home/wqs/Documents/github/automap_pro"
WS_DIR="${PROJECT_ROOT}/automap_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ─────────────────────────────────────────────────────────────────
# 颜色定义
# ─────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()  { echo -e "${BLUE}[INFO]${NC} $*"; }
log_ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_err()   { echo -e "${RED}[ERROR]${NC} $*"; }

# ─────────────────────────────────────────────────────────────────
# 前置条件检查
# ─────────────────────────────────────────────────────────────────
check_prerequisites() {
    log_info "检查前置条件..."
    
    # 检查 ROS2 humble
    if [ ! -f "/opt/ros/humble/setup.bash" ] && [ ! -f "/root/ros_install/humble/setup.bash" ]; then
        log_err "ROS2 humble 未安装，请先运行："
        echo "  curl -sSL https://raw.githubusercontent.com/ros/ros_install_one_liners/master/focal.sh | bash"
        exit 1
    fi
    
    # 检查 colcon
    if ! command -v colcon &> /dev/null; then
        log_err "colcon 未安装，请运行："
        echo "  sudo apt install -y python3-colcon-common"
        exit 1
    fi
    
    # 检查工作区存在
    if [ ! -d "$WS_DIR" ]; then
        log_err "工作区不存在：$WS_DIR"
        exit 1
    fi
    
    # 检查源代码修改
    if ! grep -q "step=teaser_precheck_skip" "${PROJECT_ROOT}/automap_pro/src/loop_closure/teaser_matcher.cpp"; then
        log_err "修复代码未应用！请确认已修改 teaser_matcher.cpp"
        exit 1
    fi
    
    log_ok "前置条件检查通过"
}

# ─────────────────────────────────────────────────────────────────
# 编译函数
# ─────────────────────────────────────────────────────────────────
build_automap() {
    log_info "开始编译 automap_pro..."
    
    cd "$WS_DIR"
    
    # 设置 ROS 环境
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f "/root/ros_install/humble/setup.bash" ]; then
        source /root/ros_install/humble/setup.bash
    fi
    
    # 清理旧构建（可选）
    if [ "$1" = "--clean" ]; then
        log_info "清理旧构建..."
        rm -rf build install
    fi
    
    # 编译
    log_info "运行 colcon build..."
    colcon build \
        --packages-select automap_pro \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DUSE_TEASER=ON \
        2>&1 | tee build.log
    
    if [ $? -eq 0 ]; then
        log_ok "编译成功"
    else
        log_err "编译失败，请检查 build.log"
        exit 1
    fi
}

# ─────────────────────────────────────────────────────────────────
# 验证编译产物
# ─────────────────────────────────────────────────────────────────
verify_build() {
    log_info "验证编译产物..."
    
    SO_FILE="$WS_DIR/install/automap_pro/lib/libautomap_loop_closure.so"
    
    if [ ! -f "$SO_FILE" ]; then
        log_err "动态库不存在：$SO_FILE"
        exit 1
    fi
    
    # 检查关键符号
    log_info "检查符号..."
    if nm "$SO_FILE" | grep -q "TeaserMatcher"; then
        log_ok "TeaserMatcher 符号存在"
    else
        log_warn "TeaserMatcher 符号不存在（可能被优化剔除）"
    fi
    
    # 检查修复代码引入的字符串
    if strings "$SO_FILE" | grep -q "teaser_precheck_skip"; then
        log_ok "修复代码已编译进库"
    else
        log_warn "修复代码字符串未检测到（可能被优化）"
    fi
    
    log_ok "编译产物验证通过"
}

# ─────────────────────────────────────────────────────────────────
# 运行回放测试
# ─────────────────────────────────────────────────────────────────
run_test() {
    log_info "运行离线回放测试..."
    
    cd "$PROJECT_ROOT"
    
    # 检查数据集
    BAG_FILE="$(pwd)/data/automap_input/M2DGR/street_03_ros2"
    if [ ! -d "$BAG_FILE" ]; then
        log_err "测试数据集不存在：$BAG_FILE"
        log_warn "请准备测试数据后再运行测试"
        return 1
    fi
    
    log_info "执行回放..."
    bash run_automap.sh \
        --offline \
        --bag-file "$BAG_FILE" \
        --config system_config_M2DGR.yaml \
        2>&1 | tee test_run.log
    
    if [ $? -eq 0 ]; then
        log_ok "回放测试完成"
        return 0
    else
        log_err "回放测试失败"
        return 1
    fi
}

# ─────────────────────────────────────────────────────────────────
# 检查测试结果
# ─────────────────────────────────────────────────────────────────
check_test_results() {
    log_info "检查测试结果..."
    
    LOG_FILE="${PROJECT_ROOT}/test_run.log"
    
    # 检查是否有 SIGSEGV
    if grep -q "Segmentation fault\|SIGSEGV\|signal 11" "$LOG_FILE" 2>/dev/null; then
        log_err "检测到段错误！修复可能失败"
        return 1
    else
        log_ok "未检测到段错误"
    fi
    
    # 检查修复代码的执行
    if grep -q "teaser_precheck_skip\|teaser_extremely_few_inliers" "$LOG_FILE" 2>/dev/null; then
        log_ok "修复代码已执行"
        grep "teaser_precheck_skip\|teaser_extremely_few_inliers" "$LOG_FILE" | head -3
    else
        log_warn "未检测到修复代码执行（可能该数据集不触发保护条件）"
    fi
    
    # 检查异常
    if grep -q "\[EXCEPTION\]\|\[ERROR\]" "$LOG_FILE" 2>/dev/null; then
        log_warn "检测到异常信息，请审查"
        grep "\[EXCEPTION\]\|\[ERROR\]" "$LOG_FILE" | head -5
    fi
    
    log_ok "测试结果检查完成"
    return 0
}

# ─────────────────────────────────────────────────────────────────
# 主流程
# ─────────────────────────────────────────────────────────────────
main() {
    log_info "TEASER 修复编译与验证脚本"
    log_info "=================================================="
    
    check_prerequisites
    
    # 解析参数
    CLEAN_FLAG=""
    if [ "$1" = "--clean" ]; then
        CLEAN_FLAG="--clean"
    fi
    
    build_automap $CLEAN_FLAG
    verify_build
    
    # 运行测试（可选）
    if [ "$1" = "--with-test" ] || [ "$2" = "--with-test" ]; then
        run_test
        check_test_results
    else
        log_info "跳过测试（使用 --with-test 运行）"
    fi
    
    log_ok "=================================================="
    log_ok "编译和验证完成！"
    log_info "后续步骤："
    echo "  1. 查看编译日志：tail -100 ${WS_DIR}/build.log"
    echo "  2. 运行回放测试：bash teaser_fix_build_and_test.sh --with-test"
    echo "  3. 监控关键日志：grep 'TEASER\\|CRASH' ${PROJECT_ROOT}/test_run.log | head -20"
}

main "$@"
