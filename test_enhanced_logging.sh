#!/bin/bash
# AutoMap-Pro 增强日志系统测试脚本
# 验证所有功能是否正常工作

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 测试结果
PASSED=0
FAILED=0
TOTAL=0

# 测试函数
test_function() {
    local name="$1"
    local command="$2"
    local description="$3"
    
    TOTAL=$((TOTAL + 1))
    echo -e "\n${CYAN}[测试 $TOTAL]${NC} $name"
    echo -e "${BLUE}描述:${NC} $description"
    echo -e "${BLUE}命令:${NC} $command"
    
    if eval "$command" &> /dev/null; then
        echo -e "${GREEN}✓ PASS${NC} - $name"
        PASSED=$((PASSED + 1))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC} - $name"
        FAILED=$((FAILED + 1))
        return 1
    fi
}

# 显示测试结果
show_results() {
    echo -e "\n${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  测试结果摘要${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}通过: $PASSED${NC}"
    echo -e "${RED}失败: $FAILED${NC}"
    echo -e "${BLUE}总计: $TOTAL${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"
    
    if [ $FAILED -eq 0 ]; then
        echo -e "${GREEN}所有测试通过！${NC}\n"
        return 0
    else
        echo -e "${RED}有测试失败，请检查日志${NC}\n"
        return 1
    fi
}

# 主测试流程
main() {
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  AutoMap-Pro 增强日志系统测试${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    
    # 测试 1: 检查文件是否存在
    test_function \
        "文件存在性检查" \
        "test -f run_full_mapping_enhanced.sh && test -f view_logs.sh && test -f quick_start_enhanced.sh" \
        "验证所有新增脚本文件是否存在"
    
    # 测试 2: 检查可执行权限
    test_function \
        "可执行权限检查" \
        "test -x run_full_mapping_enhanced.sh && test -x view_logs.sh && test -x quick_start_enhanced.sh" \
        "验证所有脚本是否具有可执行权限"
    
    # 测试 3: 检查文档文件
    test_function \
        "文档文件检查" \
        "test -f docs/LOGGING_GUIDE.md && test -f docs/ENHANCED_LOGGING_SUMMARY.md" \
        "验证所有文档文件是否存在"
    
    # 测试 4: 检查日志目录创建
    test_function \
        "日志目录创建" \
        "mkdir -p logs/monitoring && test -d logs && test -d logs/monitoring" \
        "验证日志目录是否可以正常创建"
    
    # 测试 5: 检查帮助信息
    test_function \
        "增强脚本帮助信息" \
        "./run_full_mapping_enhanced.sh --help | grep -q 'AutoMap-Pro'" \
        "验证增强脚本的帮助信息是否正常显示"
    
    # 测试 6: 检查日志查看器帮助
    test_function \
        "日志查看器帮助信息" \
        "./view_logs.sh --help | grep -q 'AutoMap-Pro'" \
        "验证日志查看器的帮助信息是否正常显示"
    
    # 测试 7: 检查快速启动菜单
    test_function \
        "快速启动菜单" \
        "./quick_start_enhanced.sh --help | grep -q 'AutoMap-Pro'" \
        "验证快速启动菜单的帮助信息是否正常显示"
    
    # 测试 8: 检查日志文件创建
    echo -e "\n${CYAN}[测试 $((TOTAL + 1))]${NC} 日志文件创建测试"
    echo -e "${BLUE}描述:${NC} 验证日志文件是否可以正常创建"
    
    # 创建测试日志
    TEST_LOG="logs/test_$(date +%Y%m%d_%H%M%S).log"
    TEST_MONITOR_DIR="logs/monitoring"
    
    if echo "[INFO] 测试日志" > "$TEST_LOG" 2>/dev/null && \
       mkdir -p "$TEST_MONITOR_DIR" 2>/dev/null && \
       echo "[INFO] 测试监控" > "$TEST_MONITOR_DIR/test.log" 2>/dev/null; then
        echo -e "${GREEN}✓ PASS${NC} - 日志文件创建测试"
        PASSED=$((PASSED + 1))
        TOTAL=$((TOTAL + 1))
        
        # 清理测试文件
        rm -f "$TEST_LOG" 2>/dev/null
        rm -f "$TEST_MONITOR_DIR/test.log" 2>/dev/null
    else
        echo -e "${RED}✗ FAIL${NC} - 日志文件创建测试"
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
    fi
    
    # 测试 9: 检查日志查看器功能
    echo -e "\n${CYAN}[测试 $((TOTAL + 1))]${NC} 日志查看器功能测试"
    echo -e "${BLUE}描述:${NC} 验证日志查看器的基本功能"
    
    # 创建测试日志
    mkdir -p logs logs/monitoring
    TEST_MAIN_LOG="logs/test_main.log"
    TEST_NODES_LOG="logs/monitoring/test_nodes.log"
    TEST_TOPICS_LOG="logs/monitoring/test_topics.log"
    TEST_PROGRESS_LOG="logs/monitoring/test_progress.log"
    
    # 写入测试数据
    cat > "$TEST_MAIN_LOG" << 'EOF'
[2025-03-01 12:00:00] [INFO] 测试日志
[2025-03-01 12:00:01] [WARN] 测试警告
[2025-03-01 12:00:02] [ERROR] 测试错误
[2025-03-01 12:00:03] [DEBUG] 测试调试信息
[2025-03-01 12:00:04] [✓] 测试成功标记
EOF
    
    cat > "$TEST_NODES_LOG" << 'EOF'
[2025-03-01 12:00:00] 节点状态检查:
[2025-03-01 12:00:00] [✓] automap_system - 运行中
[2025-03-01 12:00:00] [✓] rviz - 运行中
EOF
    
    cat > "$TEST_TOPICS_LOG" << 'EOF'
[2025-03-01 12:00:00] 话题状态检查:
[2025-03-01 12:00:00] [✓] /livox/lidar
[2025-03-01 12:00:00] [✓] /livox/imu
EOF
    
    cat > "$TEST_PROGRESS_LOG" << 'EOF'
[PROGRESS] 建图开始: 2025-03-01 12:00:00
[PROGRESS] 已生成 5 个子图文件
[PROGRESS] 全局地图大小: 1.2G
EOF
    
    # 测试日志查看器
    if ./view_logs.sh -s &> /dev/null && \
       ./view_logs.sh -e &> /dev/null && \
       ./view_logs.sh -w &> /dev/null && \
       ./view_logs.sh -n &> /dev/null && \
       ./view_logs.sh -t &> /dev/null && \
       ./view_logs.sh -p &> /dev/null && \
       ./view_logs.sh -d &> /dev/null; then
        echo -e "${GREEN}✓ PASS${NC} - 日志查看器功能测试"
        PASSED=$((PASSED + 1))
        TOTAL=$((TOTAL + 1))
    else
        echo -e "${RED}✗ FAIL${NC} - 日志查看器功能测试"
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
    fi
    
    # 清理测试文件
    rm -f "$TEST_MAIN_LOG" "$TEST_NODES_LOG" "$TEST_TOPICS_LOG" "$TEST_PROGRESS_LOG" 2>/dev/null
    
    # 测试 10: 检查 Docker 脚本更新
    test_function \
        "Docker 脚本更新检查" \
        "grep -q 'run_full_mapping_enhanced.sh' run_full_mapping_docker.sh" \
        "验证 Docker 脚本是否已更新为使用增强版本"
    
    # 测试 11: 检查日志函数
    echo -e "\n${CYAN}[测试 $((TOTAL + 1))]${NC} 日志函数测试"
    echo -e "${BLUE}描述:${NC} 验证日志函数是否正常工作"
    
    # Source 增强脚本以获取日志函数
    source run_full_mapping_enhanced.sh 2>/dev/null
    
    # 测试日志函数
    TEST_LOG_OUTPUT=$(log_info "测试信息" 2>&1)
    if echo "$TEST_LOG_OUTPUT" | grep -q "\[INFO\]"; then
        echo -e "${GREEN}✓ PASS${NC} - 日志函数测试"
        PASSED=$((PASSED + 1))
        TOTAL=$((TOTAL + 1))
    else
        echo -e "${RED}✗ FAIL${NC} - 日志函数测试"
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
    fi
    
    # 测试 12: 检查配置参数
    echo -e "\n${CYAN}[测试 $((TOTAL + 1))]${NC} 配置参数测试"
    echo -e "${BLUE}描述:${NC} 验证默认配置参数是否正确"
    
    # Source 增强脚本以获取配置
    source run_full_mapping_enhanced.sh 2>/dev/null
    
    if [ -n "$SCRIPT_DIR" ] && [ -n "$LOG_DIR" ] && [ -n "$MONITOR_DIR" ]; then
        echo -e "${GREEN}✓ PASS${NC} - 配置参数测试"
        echo -e "${BLUE}  SCRIPT_DIR:${NC} $SCRIPT_DIR"
        echo -e "${BLUE}  LOG_DIR:${NC} $LOG_DIR"
        echo -e "${BLUE}  MONITOR_DIR:${NC} $MONITOR_DIR"
        PASSED=$((PASSED + 1))
        TOTAL=$((TOTAL + 1))
    else
        echo -e "${RED}✗ FAIL${NC} - 配置参数测试"
        FAILED=$((FAILED + 1))
        TOTAL=$((TOTAL + 1))
    fi
    
    # 显示测试结果
    show_results
}

# 运行测试
main "$@"
