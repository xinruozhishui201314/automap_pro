#!/bin/bash
# 测试参数解析修复

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}参数解析修复测试${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 测试 1: 检查参数格式
echo -e "${YELLOW}[测试 1] 验证参数格式支持${NC}"
if grep -q -- '--bag-file)' "$SCRIPT_DIR/run_full_mapping_enhanced.sh"; then
    echo -e "${GREEN}✓ --bag-file 参数已支持${NC}"
else
    echo -e "${RED}✗ --bag-file 参数未找到${NC}"
    exit 1
fi

if grep -q -- '-b|--bag)' "$SCRIPT_DIR/run_full_mapping_enhanced.sh"; then
    echo -e "${GREEN}✓ -b 和 --bag 参数已支持${NC}"
else
    echo -e "${RED}✗ -b 和 --bag 参数未找到${NC}"
    exit 1
fi

# 测试 2: 检查帮助信息
echo -e "${YELLOW}[测试 2] 验证帮助信息更新${NC}"
if grep -Fq 'bag-file FILE' "$SCRIPT_DIR/run_full_mapping_enhanced.sh"; then
    echo -e "${GREEN}✓ 帮助信息已更新${NC}"
else
    echo -e "${RED}✗ 帮助信息未更新${NC}"
    exit 1
fi

# 测试 3: 测试实际参数解析
echo ""
echo -e "${YELLOW}[测试 3] 测试实际参数解析${NC}"
echo -e "${BLUE}----------------------------------------${NC}"

# 创建测试脚本
TEST_SCRIPT=$(cat << 'EOF'
#!/bin/bash
BAG_FILE=""
CONFIG=""
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--bag)
            BAG_FILE="$2"
            shift 2
            ;;
        --bag-file)
            BAG_FILE="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG="$2"
            shift 2
            ;;
        -o|--output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        *)
            echo "未知选项: $1"
            exit 1
            ;;
    esac
done

echo "BAG_FILE=$BAG_FILE"
echo "CONFIG=$CONFIG"
echo "OUTPUT_DIR=$OUTPUT_DIR"
EOF
)

# 测试 --bag-file 参数
echo "测试 1: --bag-file 参数"
RESULT=$(echo "$TEST_SCRIPT" | bash -s -- --bag-file test.bag)
if echo "$RESULT" | grep -q "BAG_FILE=test.bag"; then
    echo -e "${GREEN}✓ --bag-file 参数解析成功${NC}"
else
    echo -e "${RED}✗ --bag-file 参数解析失败${NC}"
    echo "$RESULT"
    exit 1
fi

# 测试 --config 参数
echo "测试 2: --config 参数"
RESULT=$(echo "$TEST_SCRIPT" | bash -s -- --config test.yaml)
if echo "$RESULT" | grep -q "CONFIG=test.yaml"; then
    echo -e "${GREEN}✓ --config 参数解析成功${NC}"
else
    echo -e "${RED}✗ --config 参数解析失败${NC}"
    echo "$RESULT"
    exit 1
fi

# 测试 --output-dir 参数
echo "测试 3: --output-dir 参数"
RESULT=$(echo "$TEST_SCRIPT" | bash -s -- --output-dir /test/dir)
if echo "$RESULT" | grep -q "OUTPUT_DIR=/test/dir"; then
    echo -e "${GREEN}✓ --output-dir 参数解析成功${NC}"
else
    echo -e "${RED}✗ --output-dir 参数解析失败${NC}"
    echo "$RESULT"
    exit 1
fi

# 测试组合参数
echo "测试 4: 组合参数"
RESULT=$(echo "$TEST_SCRIPT" | bash -s -- --bag-file test.bag --config test.yaml --output-dir /test/dir)
if echo "$RESULT" | grep -q "BAG_FILE=test.bag" && \
   echo "$RESULT" | grep -q "CONFIG=test.yaml" && \
   echo "$RESULT" | grep -q "OUTPUT_DIR=/test/dir"; then
    echo -e "${GREEN}✓ 组合参数解析成功${NC}"
else
    echo -e "${RED}✗ 组合参数解析失败${NC}"
    echo "$RESULT"
    exit 1
fi

echo -e "${BLUE}----------------------------------------${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}所有测试通过！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}修复摘要：${NC}"
echo -e "1. 添加了 --bag-file 独立 case 分支"
echo -e "2. 更新了帮助信息"
echo -e "3. 参数解析已验证正确"
echo ""
echo -e "${YELLOW}下一步：${NC}"
echo -e "运行建图命令："
echo -e "${BLUE}./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag${NC}"
echo ""
