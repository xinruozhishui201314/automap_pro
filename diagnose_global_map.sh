#!/bin/bash
# ════════════════════════════════════════════════════════════════════════════
# 全局点云混乱问题 快速诊断脚本
# 使用方式: ./diagnose_global_map.sh <日志文件>
# 示例: ./diagnose_global_map.sh full.log
# ════════════════════════════════════════════════════════════════════════════

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

if [ $# -eq 0 ]; then
    echo -e "${YELLOW}使用方式:${NC} $0 <日志文件>"
    echo -e "${BLUE}示例:${NC} $0 full.log"
    echo -e "\n${BLUE}快速开始:${NC}"
    echo -e "  1. 运行系统并记录日志:"
    echo -e "     ${CYAN}ros2 launch automap_pro automap_online.launch.py 2>&1 | tee full.log &${NC}"
    echo -e "  2. 等待 10-30 秒"
    echo -e "  3. 运行诊断脚本:"
    echo -e "     ${CYAN}./diagnose_global_map.sh full.log${NC}"
    exit 0
fi

LOG_FILE="$1"

if [ ! -f "$LOG_FILE" ]; then
    echo -e "${RED}✗ 日志文件不存在: $LOG_FILE${NC}"
    exit 1
fi

# 提取诊断日志
DIAG_LOG=$(mktemp)
grep "GLOBAL_MAP_DIAG" "$LOG_FILE" > "$DIAG_LOG" 2>/dev/null || true

if [ ! -s "$DIAG_LOG" ]; then
    echo -e "${YELLOW}⚠️  未发现 GLOBAL_MAP_DIAG 日志${NC}"
    echo -e "   可能原因："
    echo -e "   1. 系统未启动或未运行足够长的时间"
    echo -e "   2. 日志级别未启用（检查 log_level 配置）"
    exit 1
fi

echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}【全局点云混乱问题 诊断报告】${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"

# 1. 主路径判断
echo -e "\n${MAGENTA}【1】主路径状态${NC}"
PATH_FROM_KF=$(grep -c "path=from_kf" "$DIAG_LOG" || echo 0)
PATH_FALLBACK=$(grep -c "path=fallback_merged_cloud" "$DIAG_LOG" || echo 0)

if [ "$PATH_FROM_KF" -gt 0 ]; then
    echo -e "${GREEN}✓ 主路径运行: ${PATH_FROM_KF} 次${NC}"
    
    # 提取最后一次的统计信息
    LAST_STATS=$(grep "path=from_kf" "$DIAG_LOG" | tail -1)
    if [ -n "$LAST_STATS" ]; then
        echo -e "   $LAST_STATS" | sed 's/.*path=//'
    fi
else
    echo -e "${YELLOW}⚠️  主路径未运行${NC}"
fi

if [ "$PATH_FALLBACK" -gt 0 ]; then
    echo -e "${RED}✗ 回退路径运行: ${PATH_FALLBACK} 次${NC}"
    echo -e "   ${RED}⚠️  全局图可能使用了旧世界系统，会与优化轨迹错位${NC}"
else
    echo -e "${GREEN}✓ 未使用回退路径${NC}"
fi

# 2. 关键帧统计
echo -e "\n${MAGENTA}【2】关键帧处理统计${NC}"

KF_SKIPPED_NULL=$(grep -oP 'kf_skipped_null=\K\d+' "$DIAG_LOG" | tail -1)
KF_SKIPPED_EMPTY=$(grep -oP 'kf_skipped_empty=\K\d+' "$DIAG_LOG" | tail -1)
KF_FALLBACK_UNOPT=$(grep -oP 'kf_fallback_unopt=\K\d+' "$DIAG_LOG" | tail -1)

if [ -z "$KF_SKIPPED_NULL" ]; then KF_SKIPPED_NULL="N/A"; fi
if [ -z "$KF_SKIPPED_EMPTY" ]; then KF_SKIPPED_EMPTY="N/A"; fi
if [ -z "$KF_FALLBACK_UNOPT" ]; then KF_FALLBACK_UNOPT="N/A"; fi

echo -e "   关键帧为null的数量: $KF_SKIPPED_NULL"
if [ "$KF_SKIPPED_NULL" != "N/A" ] && [ "$KF_SKIPPED_NULL" -gt 0 ]; then
    echo -e "   ${YELLOW}⚠️  这表示有关键帧对象被删除${NC}"
fi

echo -e "   关键帧点云为空的数量: $KF_SKIPPED_EMPTY"
if [ "$KF_SKIPPED_EMPTY" != "N/A" ] && [ "$KF_SKIPPED_EMPTY" -gt 5 ]; then
    echo -e "   ${YELLOW}⚠️  如果 retain_cloud_body=true 但仍有很多点云被清空，说明内存压力严重${NC}"
fi

echo -e "   未被优化的关键帧数: $KF_FALLBACK_UNOPT"
if [ "$KF_FALLBACK_UNOPT" != "N/A" ] && [ "$KF_FALLBACK_UNOPT" -gt 5 ]; then
    echo -e "   ${YELLOW}⚠️  多个关键帧未被优化，可能影响全局图精度${NC}"
fi

# 3. 点云统计
echo -e "\n${MAGENTA}【3】点云统计${NC}"

COMBINED_PTS=$(grep -oP 'combined_pts=\K\d+' "$DIAG_LOG" | tail -1)
OUTPUT_PTS=$(grep -oP 'after_downsample out_pts=\K\d+' "$DIAG_LOG" | tail -1)

if [ -n "$COMBINED_PTS" ]; then
    echo -e "   合并前总点数: ${CYAN}$COMBINED_PTS${NC}"
fi

if [ -n "$OUTPUT_PTS" ]; then
    echo -e "   下采样后点数: ${CYAN}$OUTPUT_PTS${NC}"
    if [ -n "$COMBINED_PTS" ] && [ "$COMBINED_PTS" -gt 0 ]; then
        RATIO=$(echo "scale=1; $OUTPUT_PTS * 100 / $COMBINED_PTS" | bc 2>/dev/null || echo "?")
        echo -e "   压缩率: ${CYAN}${RATIO}%${NC}"
        if [ "$RATIO" != "?" ] && (( $(echo "$RATIO < 1" | bc -l 2>/dev/null || echo 0) )); then
            echo -e "   ${YELLOW}⚠️  压缩率过低，检查体素大小参数${NC}"
        elif [ "$RATIO" != "?" ] && (( $(echo "$RATIO > 50" | bc -l 2>/dev/null || echo 0) )); then
            echo -e "   ${YELLOW}⚠️  压缩率过高，体素大小可能太大（全局图易模糊/块状）${NC}"
        fi
    fi
fi

VOXEL_SIZE=$(grep -oP 'buildGlobalMap enter voxel_size=\K[\d.]+' "$DIAG_LOG" | tail -1)
if [ -n "$VOXEL_SIZE" ]; then
    echo -e "   全局图体素 voxel_size: ${CYAN}${VOXEL_SIZE} m${NC}"
    if (( $(echo "$VOXEL_SIZE > 0.3" | bc -l 2>/dev/null || echo 0) )); then
        echo -e "   ${YELLOW}⚠️  体素较大，若观感模糊可尝试减小 map.voxel_size（如 0.2）${NC}"
    fi
fi

# 3.5 模糊精准定位（源码 [GLOBAL_MAP_BLUR] 单行汇总）
BLUR_LINE=$(grep "\[GLOBAL_MAP_BLUR\] path=" "$LOG_FILE" | tail -1)
if [ -n "$BLUR_LINE" ]; then
    echo -e "\n${MAGENTA}【3.5】模糊精准定位 (GLOBAL_MAP_BLUR)${NC}"
    echo -e "   $BLUR_LINE"
    if echo "$BLUR_LINE" | grep -q "blur_risk=yes"; then
        echo -e "   ${YELLOW}⚠️  blur_risk=yes → 见 automap_pro/docs/GLOBAL_MAP_BLUR_ANALYSIS.md${NC}"
    fi
fi
SPARSE_CNT=$(grep -c "\[GLOBAL_MAP_BLUR\] sparse_keyframe" "$LOG_FILE" 2>/dev/null || echo 0)
if [ "$SPARSE_CNT" -gt 0 ]; then
    echo -e "   稀疏关键帧告警次数: ${YELLOW}$SPARSE_CNT${NC} (单帧点数<500)"
fi

# 4. 包围盒检查
echo -e "\n${MAGENTA}【4】包围盒分析${NC}"

BBOX_LINE=$(grep "combined_pts=.*bbox=" "$DIAG_LOG" | tail -1)
if [ -n "$BBOX_LINE" ]; then
    echo -e "   $BBOX_LINE" | sed 's/.*bbox=/   bbox=/' | sed 's/combined_pts=//'
    
    # 检查是否包含 NaN 或 Inf
    if echo "$BBOX_LINE" | grep -qE "nan|inf|-nan|NaN"; then
        echo -e "   ${RED}✗ 检测到 NaN 或 Inf，坐标系可能有问题${NC}"
    fi
fi

# 5. 错误与警告
echo -e "\n${MAGENTA}【5】错误与警告${NC}"

ERRORS=$(grep -c "\[GLOBAL_MAP_DIAG\].*❌\|ERROR\|FALLBACK" "$DIAG_LOG" || echo 0)
WARNS=$(grep -c "\[GLOBAL_MAP_DIAG\].*⚠️\|WARN" "$DIAG_LOG" || echo 0)

if [ "$ERRORS" -gt 0 ]; then
    echo -e "${RED}✗ 错误数量: $ERRORS${NC}"
    grep "\[GLOBAL_MAP_DIAG\].*❌\|ERROR" "$DIAG_LOG" | head -5 | sed 's/^/   /'
fi

if [ "$WARNS" -gt 0 ]; then
    echo -e "${YELLOW}⚠️  警告数量: $WARNS${NC}"
    grep "\[GLOBAL_MAP_DIAG\].*⚠️\|T_w_b_optimized=Identity" "$DIAG_LOG" | head -5 | sed 's/^/   /'
fi

# 6. 综合诊断
echo -e "\n${MAGENTA}【6】综合诊断${NC}"

DIAGNOSIS_RESULT=0

# 检查主路径
if [ "$PATH_FROM_KF" -eq 0 ] && [ "$PATH_FALLBACK" -eq 0 ]; then
    echo -e "${RED}✗ 问题: buildGlobalMap 未成功执行${NC}"
    DIAGNOSIS_RESULT=1
elif [ "$PATH_FALLBACK" -gt 0 ]; then
    echo -e "${RED}✗ 问题: 使用了回退路径（合并旧世界系 merged_cloud）${NC}"
    echo -e "   ${YELLOW}可能导致: 全局点云与优化轨迹错位/混乱${NC}"
    echo -e "   ${BLUE}解决方案:${NC}"
    echo -e "     1. 检查 retain_cloud_body 是否为 true"
    echo -e "     2. 检查系统是否有足够内存"
    echo -e "     3. 参考 GLOBAL_MAP_FIX_PLAN_D.md"
    DIAGNOSIS_RESULT=1
else
    echo -e "${GREEN}✓ 主路径正常运行${NC}"
fi

# 检查关键帧统计
if [ "$KF_FALLBACK_UNOPT" != "N/A" ] && [ "$KF_FALLBACK_UNOPT" -gt 10 ]; then
    echo -e "${YELLOW}⚠️  问题: 大量关键帧未被优化 (${KF_FALLBACK_UNOPT})${NC}"
    DIAGNOSIS_RESULT=1
fi

# 最终结论
echo -e "\n${MAGENTA}【最终结论】${NC}"

if [ "$DIAGNOSIS_RESULT" -eq 0 ]; then
    echo -e "${GREEN}✓ 全局点云构建正常，混乱问题应已解决${NC}"
    echo -e "\n   下一步验证:"
    echo -e "   1. 在 RViz 中查看全局点云"
    echo -e "   2. 对比 /automap/odom_path 和 /automap/optimized_path"
    echo -e "   3. 观察回环处点云是否对齐（无重影/错位）"
else
    echo -e "${RED}✗ 全局点云仍可能混乱，需要进一步诊断${NC}"
    echo -e "\n   建议:"
    echo -e "   1. 查看完整诊断日志:"
    echo -e "      ${CYAN}grep 'GLOBAL_MAP_DIAG' full.log | less${NC}"
    echo -e "   2. 参考诊断文档:"
    echo -e "      ${CYAN}cat GLOBAL_MAP_DIAGNOSIS.md${NC}"
    echo -e "   3. 参考修复方案:"
    echo -e "      ${CYAN}cat GLOBAL_MAP_FIX_PLAN_D.md${NC}"
    echo -e "   4. 若全局图「模糊」而非混乱，参考:"
    echo -e "      ${CYAN}automap_pro/docs/GLOBAL_MAP_BLUR_ANALYSIS.md${NC}"
fi

# 清理临时文件
rm -f "$DIAG_LOG"

echo -e "\n${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}诊断完成${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════════${NC}\n"
