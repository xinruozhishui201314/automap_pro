#!/bin/bash
# ════════════════════════════════════════════════════════════════════════════
# AutoMap-Pro 方案D修复 编译与诊断脚本
# ════════════════════════════════════════════════════════════════════════════

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 目录定义
WORKSPACE_ROOT="/home/wqs/Documents/github/automap_pro"
BUILD_DIR="${WORKSPACE_ROOT}/build"
INSTALL_DIR="${WORKSPACE_ROOT}/install"
LOG_DIR="/tmp/automap_diag"

echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}AutoMap-Pro 方案D修复 编译与诊断脚本${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"

# 步骤 1：清理旧的构建
echo -e "\n${YELLOW}[1/4] 清理旧的构建文件...${NC}"
rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
mkdir -p "${LOG_DIR}"

# 步骤 2：检查修改
echo -e "\n${YELLOW}[2/4] 检查源代码修改...${NC}"
if grep -q "kf_skipped_null" "${WORKSPACE_ROOT}/automap_pro/src/submap/submap_manager.cpp"; then
    echo -e "${GREEN}✓ buildGlobalMap 已增强${NC}"
else
    echo -e "${RED}✗ buildGlobalMap 未增强，请检查修改${NC}"
    exit 1
fi

if grep -q "using_optimized" "${WORKSPACE_ROOT}/automap_pro/src/submap/submap_manager.cpp"; then
    echo -e "${GREEN}✓ 位姿选择逻辑已增强${NC}"
else
    echo -e "${RED}✗ 位姿选择逻辑未增强${NC}"
    exit 1
fi

# 步骤 3：编译
echo -e "\n${YELLOW}[3/4] 编译 automap_pro 包...${NC}"
cd "${WORKSPACE_ROOT}"
if colcon build \
    --packages-select automap_pro \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tee "${LOG_DIR}/build.log"; then
    echo -e "${GREEN}✓ 编译成功${NC}"
else
    echo -e "${RED}✗ 编译失败，查看详情：${LOG_DIR}/build.log${NC}"
    exit 1
fi

# 步骤 4：验证编译产物
echo -e "\n${YELLOW}[4/4] 验证编译产物...${NC}"
if [ -f "${INSTALL_DIR}/automap_pro/lib/automap_pro/automap_system_node" ]; then
    echo -e "${GREEN}✓ automap_system_node 构建成功${NC}"
else
    echo -e "${RED}✗ 找不到编译的可执行文件${NC}"
    exit 1
fi

echo -e "\n${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ 方案D修复编译完成！${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"

# 显示诊断信息
echo -e "\n${BLUE}【后续步骤】${NC}"
echo -e "${YELLOW}1. 启动系统并采集日志：${NC}"
echo -e "   ros2 launch automap_pro automap_online.launch.py 2>&1 | tee full.log &"
echo -e "\n${YELLOW}2. 等待 10-30 秒，然后提取诊断日志：${NC}"
echo -e "   grep 'GLOBAL_MAP_DIAG' full.log > diag.log"
echo -e "\n${YELLOW}3. 查看诊断结果：${NC}"
echo -e "   echo '=== Main Path ===' && grep 'path=from_kf\\|path=fallback' diag.log | head -5"
echo -e "   echo '=== Statistics ===' && grep 'kf_skipped\\|kf_fallback' diag.log"
echo -e "   echo '=== Final Result ===' && grep 'buildGlobalMap SUCCESS\\|buildGlobalMap exception' diag.log"

echo -e "\n${BLUE}【关键诊断指标】${NC}"
cat << 'EOF'
  • path=from_kf : 主路径成功
  • path=fallback_merged_cloud : 回退路径（可能混乱）
  • kf_skipped_null : 空指针关键帧数（应为 0）
  • kf_skipped_empty : 点云为空的关键帧数（应 < 5）
  • kf_fallback_unopt : 未被优化的关键帧数（应 < 5）
  • compression_ratio : 下采样压缩率（10%-20% 正常）
EOF

echo -e "\n${YELLOW}【编译日志位置】${NC}"
echo -e "   ${LOG_DIR}/build.log"

echo -e "\n${GREEN}✓ 脚本执行完成${NC}\n"
