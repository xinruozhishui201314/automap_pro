#!/bin/bash
# AutoMap-Pro 路径解析完整验证脚本
# 功能：测试所有路径解析场景并生成报告

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 统计
declare -a TEST_RESULTS
TOTAL_TESTS=0
PASSED_TESTS=0

log_info() {
    echo -e "${GREEN}[INFO]${NC} $@"
}

log_test() {
    echo -e "${CYAN}[TEST]${NC} $@"
}

log_pass() {
    echo -e "${GREEN}✓ PASS${NC} $@"
    ((PASSED_TESTS++))
    TEST_RESULTS+=("PASS: $1")
}

log_fail() {
    echo -e "${RED}✗ FAIL${NC} $@"
    TEST_RESULTS+=("FAIL: $1")
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_BAG="data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag"

print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# 测试场景
test_scenario() {
    local test_name="$1"
    local bag_path="$2"

    ((TOTAL_TESTS++))
    log_test "场景: $test_name"
    log_info "  输入: $bag_path"

    cd "$SCRIPT_DIR"

    local output
    output=$(echo "n" | timeout 15 bash run_full_mapping_docker.sh -b "$bag_path" 2>&1 || true)

    # 检查是否成功解析（没有报"Bag 文件不存在"错误）
    if echo "$output" | grep -q "Bag 文件不存在.*data/automap_input"; then
        log_fail "$test_name - 仍然报错 Bag 文件不存在"
        echo "$output" | grep -A 3 "ERROR" | tail -5
        return 1
    fi

    # 检查是否到达确认步骤
    if echo "$output" | grep -q "是否开始建图"; then
        local resolved_bag=$(echo "$output" | grep "Bag 文件" | grep -v "容器" | head -1)
        log_info "  解析结果: $resolved_bag"
        log_pass "$test_name - 成功解析"
        return 0
    fi

    log_fail "$test_name - 未到达预期步骤"
    return 1
}

main() {
    print_header "AutoMap-Pro 路径解析完整验证"

    log_info "测试环境:"
    log_info "  脚本目录: $SCRIPT_DIR"
    log_info "  真实 bag: $REAL_BAG"

    # 测试场景1: 原始报错的场景（@前缀短路径）
    print_header "测试场景 1: @data/automap_input/nya_02.bag（原始报错）"
    test_scenario "原始报错命令" "@data/automap_input/nya_02.bag"

    # 测试场景2: 不带@的短路径
    print_header "测试场景 2: data/automap_input/nya_02.bag（短路径）"
    test_scenario "短路径不带@" "data/automap_input/nya_02.bag"

    # 测试场景3: 完整路径
    print_header "测试场景 3: 完整正确路径"
    test_scenario "完整路径" "$REAL_BAG"

    # 测试场景4: 仅文件名
    print_header "测试场景 4: nya_02.bag（仅文件名）"
    test_scenario "仅文件名" "nya_02.bag"

    # 测试场景5: 默认参数
    print_header "测试场景 5: 默认参数（不指定 bag）"
    ((TOTAL_TESTS++))
    log_test "默认参数测试"
    cd "$SCRIPT_DIR"
    local output
    output=$(echo "n" | timeout 15 bash run_full_mapping_docker.sh 2>&1 || true)
    if echo "$output" | grep -q "是否开始建图"; then
        log_pass "默认参数 - 成功使用默认值"
    else
        log_fail "默认参数 - 失败"
    fi

    # 打印测试报告
    print_header "测试报告"
    echo "总测试数: $TOTAL_TESTS"
    echo -e "通过: ${GREEN}$PASSED_TESTS${NC}"
    echo -e "失败: ${RED}$((TOTAL_TESTS - PASSED_TESTS))${NC}"

    if [ $TOTAL_TESTS -gt 0 ]; then
        local pass_rate=$(( PASSED_TESTS * 100 / TOTAL_TESTS ))
        echo "通过率: $pass_rate%"
    fi

    echo ""
    echo "详细结果:"
    for result in "${TEST_RESULTS[@]}"; do
        if [[ "$result" == PASS* ]]; then
            echo -e "${GREEN}$result${NC}"
        else
            echo -e "${RED}$result${NC}"
        fi
    done
    echo ""

    if [ $PASSED_TESTS -eq $TOTAL_TESTS ]; then
        echo -e "${GREEN}✓ 所有测试通过！修复成功！${NC}"
        echo ""
        echo "现在可以运行建图命令："
        echo "  ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag"
        echo ""
        exit 0
    else
        echo -e "${RED}✗ 部分测试失败${NC}"
        exit 1
    fi
}

main "$@"
