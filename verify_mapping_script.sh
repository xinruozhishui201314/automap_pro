#!/bin/bash
# AutoMap-Pro 路径解析自动化验证脚本
# 功能：验证 run_full_mapping_docker.sh 的路径解析逻辑

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 统计
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $@"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $@"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $@"
}

log_test() {
    echo -e "${CYAN}[TEST]${NC} $@"
}

log_pass() {
    echo -e "${GREEN}✓ PASS${NC} $@"
    ((PASSED_TESTS++))
}

log_fail() {
    echo -e "${RED}✗ FAIL${NC} $@"
    ((FAILED_TESTS++))
}

# 配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REAL_BAG_PATH="data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag"

# 打印标题
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

# 测试场景1: 原始报错命令的 dry-run 测试
test_scenario_original_error() {
    print_header "场景 1: 原始报错命令 -b @data/automap_input/nya_02.bag"
    ((TOTAL_TESTS++))
    log_test "测试原始报错命令是否能正确解析"

    cd "$SCRIPT_DIR"

    # 使用 dry-run 模式（用户在确认时输入 'n' 来退出）
    local output
    output=$(echo "n" | timeout 10 bash run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag 2>&1 || true)

    # 检查输出中是否包含 "Bag 文件不存在" 错误
    if echo "$output" | grep -q "Bag 文件不存在"; then
        log_fail "脚本仍然报错：Bag 文件不存在"
        echo ""
        echo "错误输出:"
        echo "$output" | grep -B 5 -A 10 "ERROR\|Bag 文件不存在" | tail -20
        return 1
    fi

    # 检查是否成功解析并显示配置信息
    if echo "$output" | grep -q "Bag 文件.*data/automap_input"; then
        log_info "Bag 文件路径已成功解析"
        # 检查是否到达确认步骤
        if echo "$output" | grep -q "是否开始建图"; then
            log_pass "脚本成功解析路径并到达确认步骤"
            echo ""
            echo "配置信息摘要:"
            echo "$output" | grep -E "Bag 文件|配置文件|输出目录" | head -5
            return 0
        fi
    fi

    # 如果没有明确的成功或失败标记，显示完整输出
    log_warn "无法确定测试结果"
    echo ""
    echo "完整输出:"
    echo "$output"
    return 1
}

# 测试场景2: 测试完整的正确路径
test_scenario_full_path() {
    print_header "场景 2: 完整正确路径"
    ((TOTAL_TESTS++))
    log_test "测试完整路径: $REAL_BAG_PATH"

    cd "$SCRIPT_DIR"

    local output
    output=$(echo "n" | timeout 10 bash run_full_mapping_docker.sh -b "$REAL_BAG_PATH" 2>&1 || true)

    if echo "$output" | grep -q "Bag 文件不存在"; then
        log_fail "完整路径仍然报错"
        echo "$output" | grep -A 5 "ERROR" | tail -10
        return 1
    fi

    if echo "$output" | grep -q "是否开始建图"; then
        log_pass "完整路径测试成功"
        echo "$output" | grep "Bag 文件"
        return 0
    fi

    log_warn "无法确定完整路径测试结果"
    echo "$output" | head -20
    return 1
}

# 测试场景3: 测试不带 @ 的短路径
test_scenario_short_path() {
    print_header "场景 3: 短路径不带 @"
    ((TOTAL_TESTS++))
    log_test "测试短路径: data/automap_input/nya_02.bag"

    cd "$SCRIPT_DIR"

    local output
    output=$(echo "n" | timeout 10 bash run_full_mapping_docker.sh -b data/automap_input/nya_02.bag 2>&1 || true)

    if echo "$output" | grep -q "Bag 文件不存在"; then
        log_fail "短路径仍然报错"
        echo "$output" | grep -A 5 "ERROR" | tail -10
        return 1
    fi

    if echo "$output" | grep -q "是否开始建图"; then
        log_pass "短路径测试成功"
        echo "$output" | grep "Bag 文件"
        return 0
    fi

    log_warn "无法确定短路径测试结果"
    return 1
}

# 测试场景4: 列出所有可用的 bag 文件
test_scenario_list_bags() {
    print_header "场景 4: 列出所有可用的 bag 文件"
    ((TOTAL_TESTS++))
    log_test "查找项目中的所有 bag 文件"

    cd "$SCRIPT_DIR"

    local bag_files
    bag_files=$(find data -name "*.bag" -o -name "*.db3" 2>/dev/null)

    if [ -z "$bag_files" ]; then
        log_warn "未找到任何 bag 文件"
        return 0
    fi

    log_info "找到以下 bag 文件:"
    echo "$bag_files" | while read -r bag; do
        if [ -f "$bag" ]; then
            log_info "  ✓ $bag ($(stat -f%z "$bag" 2>/dev/null || stat -c%s "$bag" 2>/dev/null) bytes)"
        fi
    done

    local count=$(echo "$bag_files" | wc -l)
    log_pass "找到 $count 个 bag 文件"
    return 0
}

# 测试场景5: 验证真实 bag 文件存在
test_scenario_real_bag_exists() {
    print_header "场景 5: 验证真实 bag 文件"
    ((TOTAL_TESTS++))
    log_test "检查 $REAL_BAG_PATH 是否存在"

    cd "$SCRIPT_DIR"

    if [ -f "$REAL_BAG_PATH" ]; then
        local size=$(stat -f%z "$REAL_BAG_PATH" 2>/dev/null || stat -c%s "$REAL_BAG_PATH" 2>/dev/null)
        log_info "Bag 文件大小: $size bytes"
        log_pass "真实 bag 文件存在"
        return 0
    else
        log_fail "真实 bag 文件不存在"
        return 1
    fi
}

# 测试场景6: 测试默认参数（不指定 bag）
test_scenario_default() {
    print_header "场景 6: 默认参数"
    ((TOTAL_TESTS++))
    log_test "测试不指定 bag 文件时的默认行为"

    cd "$SCRIPT_DIR"

    local output
    output=$(echo "n" | timeout 10 bash run_full_mapping_docker.sh 2>&1 || true)

    if echo "$output" | grep -q "Bag 文件不存在"; then
        log_fail "默认参数解析失败"
        echo "$output" | grep -A 5 "ERROR" | tail -10
        return 1
    fi

    if echo "$output" | grep -q "是否开始建图"; then
        log_pass "默认参数测试成功"
        echo "$output" | grep "Bag 文件"
        return 0
    fi

    log_warn "无法确定默认参数测试结果"
    return 1
}

# 主测试函数
main() {
    print_header "AutoMap-Pro 路径解析自动化验证"

    log_info "测试环境:"
    log_info "  脚本目录: $SCRIPT_DIR"
    log_info "  真实 bag: $REAL_BAG_PATH"

    # 检查脚本是否存在
    if [ ! -f "$SCRIPT_DIR/run_full_mapping_docker.sh" ]; then
        log_error "主脚本不存在: run_full_mapping_docker.sh"
        exit 1
    fi

    log_info "✓ 主脚本存在"

    # 运行所有测试场景
    test_scenario_real_bag_exists
    test_scenario_list_bags
    test_scenario_full_path
    test_scenario_short_path
    test_scenario_original_error
    test_scenario_default

    # 打印测试总结
    print_header "测试总结"
    echo "总测试数: $TOTAL_TESTS"
    echo -e "通过: ${GREEN}$PASSED_TESTS${NC}"
    echo -e "失败: ${RED}$FAILED_TESTS${NC}"

    if [ $TOTAL_TESTS -gt 0 ]; then
        local pass_rate=$(( PASSED_TESTS * 100 / TOTAL_TESTS ))
        echo "通过率: $pass_rate%"
    fi

    echo ""

    if [ $FAILED_TESTS -eq 0 ]; then
        echo -e "${GREEN}✓ 所有测试通过！修复成功！${NC}"
        echo ""
        echo "现在可以运行建图命令："
        echo "  ./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag"
        echo ""
        exit 0
    else
        echo -e "${RED}✗ 部分测试失败${NC}"
        echo ""
        echo "建议检查："
        echo "  1. run_full_mapping_docker.sh 中的路径解析逻辑"
        echo "  2. 实际的 bag 文件位置"
        echo "  3. 脚本的权限和执行权限"
        echo ""
        exit 1
    fi
}

# 运行主函数
main "$@"
