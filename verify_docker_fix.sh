#!/bin/bash
# 验证 Docker 脚本修复
# 验证修复是否解决了 /workspace/install/setup.bash 不存在的问题

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Docker 脚本修复验证${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 1: 验证脚本修改
echo -e "${YELLOW}[检查 1] 验证 run_full_mapping_docker.sh 中的挂载配置${NC}"
if grep -q 'automap_ws:/workspace/automap_ws' "$SCRIPT_DIR/run_full_mapping_docker.sh"; then
    echo -e "${GREEN}✓ automap_ws 挂载已添加${NC}"
else
    echo -e "${RED}✗ automap_ws 挂载未找到${NC}"
    exit 1
fi

# 检查 2: 验证 setup.bash 路径修改
echo -e "${YELLOW}[检查 2] 验证 setup.bash source 路径${NC}"
if grep -q 'source /workspace/automap_ws/install/setup.bash' "$SCRIPT_DIR/run_full_mapping_docker.sh"; then
    echo -e "${GREEN}✓ setup.bash 路径已修正${NC}"
else
    echo -e "${RED}✗ setup.bash 路径未修正${NC}"
    exit 1
fi

# 检查 3: 验证 automap_ws 目录存在
echo -e "${YELLOW}[检查 3] 验证 automap_ws 目录结构${NC}"
if [ -d "$SCRIPT_DIR/automap_ws" ]; then
    echo -e "${GREEN}✓ automap_ws 目录存在${NC}"
else
    echo -e "${RED}✗ automap_ws 目录不存在${NC}"
    exit 1
fi

if [ -f "$SCRIPT_DIR/automap_ws/install/setup.bash" ]; then
    echo -e "${GREEN}✓ setup.bash 文件存在${NC}"
else
    echo -e "${RED}✗ setup.bash 文件不存在${NC}"
    exit 1
fi

# 检查 4: 验证 run_full_mapping_enhanced.sh 存在
echo -e "${YELLOW}[检查 4] 验证建图脚本存在${NC}"
if [ -f "$SCRIPT_DIR/run_full_mapping_enhanced.sh" ]; then
    echo -e "${GREEN}✓ run_full_mapping_enhanced.sh 存在${NC}"
else
    echo -e "${RED}✗ run_full_mapping_enhanced.sh 不存在${NC}"
    exit 1
fi

# 检查 5: 显示实际的挂载配置
echo ""
echo -e "${YELLOW}[检查 5] Docker 挂载配置预览${NC}"
echo -e "${BLUE}----------------------------------------${NC}"
grep -A 4 "# 构建挂载点" "$SCRIPT_DIR/run_full_mapping_docker.sh" | tail -n +2 | head -n 4
echo -e "${BLUE}----------------------------------------${NC}"

# 检查 6: 显示实际的 source 命令
echo ""
echo -e "${YELLOW}[检查 6] 容器内 source 命令预览${NC}"
echo -e "${BLUE}----------------------------------------${NC}"
grep "source /workspace" "$SCRIPT_DIR/run_full_mapping_docker.sh" | grep setup.bash
echo -e "${BLUE}----------------------------------------${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有验证通过！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}修复摘要：${NC}"
echo -e "1. 添加了 automap_ws 目录挂载："
echo -e "   ${BLUE}-v $SCRIPT_DIR/automap_ws:/workspace/automap_ws${NC}"
echo ""
echo -e "2. 修正了 setup.bash source 路径："
echo -e "   ${BLUE}source /workspace/automap_ws/install/setup.bash${NC}"
echo ""
echo -e "${YELLOW}下一步：${NC}"
echo -e "运行建图命令："
echo -e "${BLUE}./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag${NC}"
echo ""
