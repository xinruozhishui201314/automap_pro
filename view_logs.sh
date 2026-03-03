#!/bin/bash
# AutoMap-Pro 日志查看工具
# 功能：快速查看和分析建图日志

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# 配置参数
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/logs"
MONITOR_DIR="${LOG_DIR}/monitoring"

# 日志文件
LATEST_LOG=""
LATEST_NODES=""
LATEST_TOPICS=""
LATEST_PROGRESS=""

# 获取最新的日志文件
get_latest_logs() {
    LATEST_LOG=$(ls -t "$LOG_DIR"/full_mapping_*.log 2>/dev/null | head -1)
    LATEST_NODES=$(ls -t "$MONITOR_DIR"/nodes_*.log 2>/dev/null | head -1)
    LATEST_TOPICS=$(ls -t "$MONITOR_DIR"/topics_*.log 2>/dev/null | head -1)
    LATEST_PROGRESS=$(ls -t "$MONITOR_DIR"/progress_*.log 2>/dev/null | head -1)
}

# 显示帮助信息
show_help() {
    cat << EOF
AutoMap-Pro 日志查看工具

用法: $0 [选项]

选项:
    -l, --latest             查看最新的完整日志（默认）
    -e, --errors             查看错误信息
    -w, --warnings           查看警告信息
    -n, --nodes             查看节点监控日志
    -t, --topics            查看话题监控日志
    -p, --progress          查看进度追踪日志
    -s, --summary           显示日志摘要
    -f, --follow            实时跟踪最新日志
    -d, --diagnose          运行诊断
    -c, --clear             清空日志目录
    -h, --help              显示此帮助信息

示例:
    # 查看最新日志
    $0

    # 查看错误信息
    $0 -e

    # 查看摘要
    $0 -s

    # 实时跟踪
    $0 -f

    # 运行诊断
    $0 -d

EOF
}

# 显示摘要
show_summary() {
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  AutoMap-Pro 建图日志摘要${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    if [ -z "$LATEST_LOG" ]; then
        echo -e "${YELLOW}未找到日志文件${NC}"
        return 0
    fi

    # 统计错误、警告、信息
    local errors=$(grep -c "\[ERROR\]" "$LATEST_LOG" 2>/dev/null || echo 0)
    local warnings=$(grep -c "\[WARN\]" "$LATEST_LOG" 2>/dev/null || echo 0)
    local steps=$(grep -c "\[STEP" "$LATEST_LOG" 2>/dev/null || echo 0)
    local successes=$(grep -c "\[✓\]" "$LATEST_LOG" 2>/dev/null || echo 0)

    echo -e "${GREEN}日志文件:${NC} $(basename "$LATEST_LOG")"
    echo -e "${GREEN}日志时间:${NC} $(stat -c %y "$LATEST_LOG" 2>/dev/null | cut -d. -f1)"
    echo -e "${GREEN}日志大小:${NC} $(du -h "$LATEST_LOG" | cut -f1)"
    echo ""
    echo -e "${CYAN}统计信息:${NC}"
    echo -e "  ${GREEN}[✓]${NC} 成功/完成: $successes"
    echo -e "  ${RED}[ERROR]${NC} 错误: $errors"
    echo -e "  ${YELLOW}[WARN]${NC} 警告: $warnings"
    echo -e "  ${CYAN}[STEP]${NC} 步骤: $steps"
    echo ""

    # 显示关键步骤
    if [ "$steps" -gt 0 ]; then
        echo -e "${CYAN}执行步骤:${NC}"
        grep "\[STEP" "$LATEST_LOG" | sed 's/^/  /'
        echo ""
    fi

    # 显示最近的状态
    echo -e "${CYAN}最近状态 (最后20行):${NC}"
    tail -20 "$LATEST_LOG" | sed 's/^/  /'
    echo ""
}

# 查看错误信息
show_errors() {
    if [ -z "$LATEST_LOG" ]; then
        echo -e "${YELLOW}未找到日志文件${NC}"
        return 0
    fi

    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${RED}  错误信息${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    local errors=$(grep "\[ERROR\]" "$LATEST_LOG" 2>/dev/null || true)
    if [ -z "$errors" ]; then
        echo -e "${GREEN}✓ 未发现错误${NC}"
    else
        echo "$errors" | sed 's/^/  /'
        echo ""
        echo -e "${RED}共 $(echo "$errors" | wc -l) 个错误${NC}"
    fi
    echo ""
}

# 查看警告信息
show_warnings() {
    if [ -z "$LATEST_LOG" ]; then
        echo -e "${YELLOW}未找到日志文件${NC}"
        return 0
    fi

    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}  警告信息${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    local warnings=$(grep "\[WARN\]" "$LATEST_LOG" 2>/dev/null || true)
    if [ -z "$warnings" ]; then
        echo -e "${GREEN}✓ 未发现警告${NC}"
    else
        echo "$warnings" | sed 's/^/  /'
        echo ""
        echo -e "${YELLOW}共 $(echo "$warnings" | wc -l) 个警告${NC}"
    fi
    echo ""
}

# 查看节点监控
show_nodes() {
    if [ -z "$LATEST_NODES" ]; then
        echo -e "${YELLOW}未找到节点监控日志${NC}"
        return 0
    fi

    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  节点监控日志${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    echo "$LATEST_NODES" | xargs -0 basename
    echo ""

    # 显示最近的节点状态
    echo -e "${CYAN}最近的节点状态 (最后30条):${NC}"
    tail -30 "$LATEST_NODES" | sed 's/^/  /'
    echo ""

    # 统计节点状态
    local running=$(grep -c "\[✓\]" "$LATEST_NODES" 2>/dev/null || echo 0)
    local stopped=$(grep -c "\[✗\]" "$LATEST_NODES" 2>/dev/null || echo 0)
    echo -e "${GREEN}运行中节点: $running${NC}"
    echo -e "${RED}已停止节点: $stopped${NC}"
    echo ""
}

# 查看话题监控
show_topics() {
    if [ -z "$LATEST_TOPICS" ]; then
        echo -e "${YELLOW}未找到话题监控日志${NC}"
        return 0
    fi

    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  话题监控日志${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    echo "$LATEST_TOPICS" | xargs -0 basename
    echo ""

    # 显示最近的话题状态
    echo -e "${CYAN}最近的话题状态 (最后30条):${NC}"
    tail -30 "$LATEST_TOPICS" | sed 's/^/  /'
    echo ""

    # 统计话题状态
    local published=$(grep -c "\[✓\]" "$LATEST_TOPICS" 2>/dev/null || echo 0)
    local not_published=$(grep -c "\[✗\]" "$LATEST_TOPICS" 2>/dev/null || echo 0)
    echo -e "${GREEN}已发布话题: $published${NC}"
    echo -e "${RED}未发布话题: $not_published${NC}"
    echo ""
}

# 查看进度追踪
show_progress() {
    if [ -z "$LATEST_PROGRESS" ]; then
        echo -e "${YELLOW}未找到进度追踪日志${NC}"
        return 0
    fi

    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  进度追踪日志${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    echo "$LATEST_PROGRESS" | xargs -0 basename
    echo ""

    # 显示最近的进度
    echo -e "${CYAN}最近进度 (最后30条):${NC}"
    tail -30 "$LATEST_PROGRESS" | sed 's/^/  /'
    echo ""
}

# 运行诊断
run_diagnose() {
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}  建图诊断${NC}"
    echo -e "${CYAN}══════════════════════════════════════════════════${NC}\n"

    # 检查日志文件
    echo -e "${CYAN}[1] 检查日志文件${NC}"
    if [ -d "$LOG_DIR" ]; then
        local log_count=$(ls "$LOG_DIR"/*.log 2>/dev/null | wc -l)
        echo -e "  ${GREEN}✓${NC} 日志目录存在: $LOG_DIR"
        echo -e "  ${GREEN}✓${NC} 发现 $log_count 个日志文件"
    else
        echo -e "  ${RED}✗${NC} 日志目录不存在: $LOG_DIR"
    fi
    echo ""

    # 检查最近日志
    if [ -n "$LATEST_LOG" ]; then
        echo -e "${CYAN}[2] 分析最新日志${NC}"
        local errors=$(grep -c "\[ERROR\]" "$LATEST_LOG" 2>/dev/null || echo 0)
        local warnings=$(grep -c "\[WARN\]" "$LATEST_LOG" 2>/dev/null || echo 0)

        if [ "$errors" -eq 0 ] && [ "$warnings" -eq 0 ]; then
            echo -e "  ${GREEN}✓${NC} 未发现错误或警告"
        else
            if [ "$errors" -gt 0 ]; then
                echo -e "  ${RED}✗${NC} 发现 $errors 个错误"
            fi
            if [ "$warnings" -gt 0 ]; then
                echo -e "  ${YELLOW}⚠${NC} 发现 $warnings 个警告"
            fi
        fi

        # 检查建图完成情况
        if grep -q "建图完成" "$LATEST_LOG" 2>/dev/null; then
            echo -e "  ${GREEN}✓${NC} 建图已成功完成"
        elif grep -q "步骤 4/6" "$LATEST_LOG" 2>/dev/null; then
            echo -e "  ${YELLOW}⚠${NC} 建图正在进行中"
        else
            echo -e "  ${RED}✗${NC} 建图未开始或已失败"
        fi
    else
        echo -e "${YELLOW}[2] 未找到日志文件${NC}"
    fi
    echo ""

    # 检查节点状态
    if [ -n "$LATEST_NODES" ]; then
        echo -e "${CYAN}[3] 检查节点状态${NC}"
        local automap_running=$(grep -c "automap_system.*运行中" "$LATEST_NODES" 2>/dev/null || echo 0)
        local rviz_running=$(grep -c "rviz.*运行中" "$LATEST_NODES" 2>/dev/null || echo 0)

        if [ "$automap_running" -gt 0 ]; then
            echo -e "  ${GREEN}✓${NC} automap_system 节点已运行"
        else
            echo -e "  ${RED}✗${NC} automap_system 节点未运行"
        fi

        if [ "$rviz_running" -gt 0 ]; then
            echo -e "  ${GREEN}✓${NC} RViz 节点已运行"
        else
            echo -e "  ${YELLOW}⚠${NC} RViz 节点未运行"
        fi
    else
        echo -e "${YELLOW}[3] 未找到节点监控日志${NC}"
    fi
    echo ""

    # 检查话题状态
    if [ -n "$LATEST_TOPICS" ]; then
        echo -e "${CYAN}[4] 检查话题状态${NC}"
        local lidar_published=$(grep -c "/livox/lidar.*发布" "$LATEST_TOPICS" 2>/dev/null || echo 0)
        local pose_published=$(grep -c "/optimized_pose.*发布" "$LATEST_TOPICS" 2>/dev/null || echo 0)

        if [ "$lidar_published" -gt 0 ]; then
            echo -e "  ${GREEN}✓${NC} 激光雷达数据已发布"
        else
            echo -e "  ${RED}✗${NC} 激光雷达数据未发布"
        fi

        if [ "$pose_published" -gt 0 ]; then
            echo -e "  ${GREEN}✓${NC} 位姿数据已发布"
        else
            echo -e "  ${YELLOW}⚠${NC} 位姿数据未发布"
        fi
    else
        echo -e "${YELLOW}[4] 未找到话题监控日志${NC}"
    fi
    echo ""

    # 检查输出文件
    echo -e "${CYAN}[5] 检查输出文件${NC}"
    local output_dir=$(grep "输出目录:" "$LATEST_LOG" | tail -1 | cut -d: -f2 | xargs)
    if [ -n "$output_dir" ] && [ -d "$output_dir" ]; then
        echo -e "  ${GREEN}✓${NC} 输出目录存在: $output_dir"

        if [ -f "$output_dir/map/global_map.pcd" ]; then
            local size=$(du -h "$output_dir/map/global_map.pcd" | cut -f1)
            echo -e "  ${GREEN}✓${NC} 全局地图: $size"
        else
            echo -e "  ${YELLOW}⚠${NC} 全局地图未生成"
        fi

        if [ -f "$output_dir/trajectory/optimized_trajectory_tum.txt" ]; then
            local lines=$(wc -l < "$output_dir/trajectory/optimized_trajectory_tum.txt")
            echo -e "  ${GREEN}✓${NC} 轨迹文件: $lines 行"
        else
            echo -e "  ${YELLOW}⚠${NC} 轨迹文件未生成"
        fi
    else
        echo -e "  ${RED}✗${NC} 输出目录不存在"
    fi
    echo ""

    # 诊断建议
    echo -e "${CYAN}[6] 诊断建议${NC}"
    if [ -n "$LATEST_LOG" ]; then
        local errors=$(grep -c "\[ERROR\]" "$LATEST_LOG" 2>/dev/null || echo 0)
        if [ "$errors" -gt 0 ]; then
            echo -e "  ${YELLOW}→${NC} 使用 '$0 -e' 查看详细错误信息"
        fi
    fi

    echo -e "  ${YELLOW}→${NC} 使用 '$0 -l' 查看完整日志"
    echo -e "  ${YELLOW}→${NC} 使用 '$0 -n' 查看节点状态"
    echo -e "  ${YELLOW}→${NC} 使用 '$0 -t' 查看话题状态"
    echo -e "  ${YELLOW}→${NC} 使用 '$0 -p' 查看建图进度"
    echo ""
}

# 清空日志
clear_logs() {
    read -p "确定要清空所有日志吗? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        return 0
    fi

    echo -e "${CYAN}清空日志文件...${NC}"
    rm -rf "$LOG_DIR"/*.log
    rm -rf "$MONITOR_DIR"/*.log
    echo -e "${GREEN}✓ 日志已清空${NC}"
}

# 主函数
main() {
    # 获取最新日志
    get_latest_logs

    # 解析命令行参数
    case "${1:-latest}" in
        -l|--latest)
            if [ -n "$LATEST_LOG" ]; then
                less "$LATEST_LOG"
            else
                echo -e "${YELLOW}未找到日志文件${NC}"
            fi
            ;;
        -e|--errors)
            show_errors
            ;;
        -w|--warnings)
            show_warnings
            ;;
        -n|--nodes)
            show_nodes
            ;;
        -t|--topics)
            show_topics
            ;;
        -p|--progress)
            show_progress
            ;;
        -s|--summary)
            show_summary
            ;;
        -f|--follow)
            if [ -n "$LATEST_LOG" ]; then
                echo -e "${CYAN}实时跟踪日志: $LATEST_LOG${NC}"
                echo -e "${CYAN}按 Ctrl+C 退出${NC}\n"
                tail -f "$LATEST_LOG"
            else
                echo -e "${YELLOW}未找到日志文件${NC}"
            fi
            ;;
        -d|--diagnose)
            run_diagnose
            ;;
        -c|--clear)
            clear_logs
            ;;
        -h|--help)
            show_help
            ;;
        *)
            show_summary
            ;;
    esac
}

# 运行主函数
main "$@"
