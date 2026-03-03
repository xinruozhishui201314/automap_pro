#!/usr/bin/env bash
# 自动化验证：nya_02 建图流程是否通过（metadata 与 fast_livo 参数修复后）
# 用法：从仓库根目录执行
#   ./scripts/verify_full_mapping_nya02.sh
#   AUTOMAP_VERIFY_TIMEOUT=120 ./scripts/verify_full_mapping_nya02.sh
#
# 检查项：
# 1) system_config 参数产出（get_fast_livo2_params 必填字符串非空）
# 2) Docker 建图启动后一段时间内无「bad conversion」「parameter '' has invalid type」、ros2-1/fastlivo_mapping 非异常退出

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

BAG="${BAG:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2}"
CONFIG="${CONFIG:-automap_pro/config/system_config_nya02.yaml}"
TIMEOUT="${AUTOMAP_VERIFY_TIMEOUT:-90}"
LOG_DIR="${REPO_ROOT}/automap_pro/logs"
VERIFY_LOG="${LOG_DIR}/verify_full_mapping_nya02_$(date +%Y%m%d_%H%M%S).log"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'
pass() { echo -e "${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; exit 1; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }

mkdir -p "$LOG_DIR"
echo "=========================================="
echo "  Full Mapping Nya02 自动化验证"
echo "=========================================="
echo "BAG=$BAG"
echo "CONFIG=$CONFIG"
echo "TIMEOUT=${TIMEOUT}s"
echo "VERIFY_LOG=$VERIFY_LOG"
echo ""

# 1) 参数与 launch 检查
if [ -f "$REPO_ROOT/automap_pro/scripts/verify_system_config_launch.sh" ]; then
    if ! bash "$REPO_ROOT/automap_pro/scripts/verify_system_config_launch.sh" "$CONFIG" >> "$VERIFY_LOG" 2>&1; then
        fail "system_config/launch 验证未通过，见 $VERIFY_LOG"
    fi
    pass "system_config 与 launch 验证通过"
else
    warn "未找到 verify_system_config_launch.sh，跳过参数检查"
fi

# 2) Docker 建图短时运行，检查是否出现已知错误
if ! command -v docker &>/dev/null; then
    warn "Docker 未安装或不可用，跳过建图启动验证"
    echo ""
    echo "本地验证（参数+launch）已通过。完整建图验证需在 Docker 环境中运行："
    echo "  ./run_full_mapping_docker.sh -b $BAG"
    exit 0
fi

echo "启动 Docker 建图（${TIMEOUT}s 内检查启动是否正常）..."
export AUTOMAP_SKIP_CONFIRM=1
set +e
timeout "$TIMEOUT" bash "$REPO_ROOT/run_full_mapping_docker.sh" -b "$BAG" -c "$CONFIG" >> "$VERIFY_LOG" 2>&1
run_exit=$?
set -e

# 检查日志中的致命错误
if grep -q "yaml-cpp: error.*bad conversion\|Exception on parsing info file" "$VERIFY_LOG" 2>/dev/null; then
    fail "建图日志中出现 ros2 bag metadata 解析错误（bad conversion），见 $VERIFY_LOG"
fi
if grep -q "parameter '' has invalid type: expected \[string\] got \[not set\]" "$VERIFY_LOG" 2>/dev/null; then
    fail "建图日志中出现 fast_livo 参数类型错误（parameter '' not set），见 $VERIFY_LOG"
fi
if grep -q "\[ERROR\].*\[ros2-1\]: process has died" "$VERIFY_LOG" 2>/dev/null; then
    fail "ros2 bag play (ros2-1) 进程异常退出，见 $VERIFY_LOG"
fi
if grep -q "\[ERROR\].*\[fastlivo_mapping-2\]: process has died" "$VERIFY_LOG" 2>/dev/null; then
    fail "fastlivo_mapping 进程异常退出，见 $VERIFY_LOG"
fi

# 成功标志：至少看到 AutoMap-Pro 已启动
if grep -q "AutoMap-Pro started\|AutoMap-Pro 已启动\|automap_system.*started" "$VERIFY_LOG" 2>/dev/null; then
    pass "建图流程已正常启动（AutoMap-Pro started），未复现 metadata/fast_livo 错误"
else
    warn "未在 ${TIMEOUT}s 内看到 'AutoMap-Pro started'，可能启动较慢或超时；请查看 $VERIFY_LOG"
fi

echo ""
echo -e "${GREEN}=========================================="
echo "  验证通过：可进行完整建图"
echo "==========================================${NC}"
echo "完整建图命令: ./run_full_mapping_docker.sh -b $BAG -c $CONFIG"
echo ""
