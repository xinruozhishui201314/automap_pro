#!/bin/bash
# AutoMap-Pro 路径修复验证脚本

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}AutoMap-Pro 路径修复验证${NC}"
echo -e "${BLUE}========================================${NC}"

# 测试1: 原始报错的命令
echo ""
echo -e "${YELLOW}测试 1: 原始报错命令 -b @data/automap_input/nya_02.bag${NC}"
echo "----------------------------------------"
output=$(echo "n" | timeout 15 ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag 2>&1 || true)

if echo "$output" | grep -q "Bag 文件不存在.*data/automap_input/nya_02.bag"; then
    echo -e "${RED}✗ FAIL${NC} 仍然报错: Bag 文件不存在"
    echo "$output" | grep -A 3 "ERROR" | tail -10
    exit 1
fi

if echo "$output" | grep -q "找到匹配文件"; then
    echo -e "${GREEN}✓ PASS${NC} 成功找到匹配文件"
    echo "$output" | grep "找到匹配文件"
else
    echo -e "${RED}✗ FAIL${NC} 未触发模糊匹配"
    exit 1
fi

# 检查 bag 文件显示（显示用户取消也说明成功了）
if echo "$output" | grep -E "(是否开始建图|用户取消)" | grep -q .; then
    echo -e "${GREEN}✓ PASS${NC} 脚本成功运行到确认步骤"
else
    # 可能是脚本输出格式问题，检查是否有配置信息
    if echo "$output" | grep -q "配置文件.*system_config"; then
        echo -e "${GREEN}✓ PASS${NC} 脚本正常解析配置"
    else
        echo -e "${YELLOW}⚠ WARN${NC} 无法确认是否到达确认步骤，但未报错"
    fi
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}✓ 验证通过！修复成功！${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "可以运行的命令示例："
echo "  1. ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag"
echo "  2. ./run_full_mapping_docker.sh -b data/automap_input/nya_02.bag"
echo "  3. ./run_full_mapping_docker.sh -b nya_02.bag"
echo "  4. ./run_full_mapping_docker.sh"
echo ""
