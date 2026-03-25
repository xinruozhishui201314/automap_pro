#!/bin/bash
# AutoMap-Pro 一体化故障诊断工具
# 用途：快速定位系统启动失败的根本原因
# 使用：bash scripts/diagnose_automap.sh [--log-dir /tmp/automap_logs] [--verbose]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="${1:-/tmp/automap_logs}"
VERBOSE="${2:-}"

_LP="[diagnose_automap.sh]"

# ============================================================================
# 功能：格式化输出
# ============================================================================
print_info() { echo "$_LP [INFO] $*"; }
print_warn() { echo "$_LP [WARN] $*" >&2; }
print_error() { echo "$_LP [ERROR] $*" >&2; }
print_section() { echo ""; echo "════════════════════════════════════════"; echo "$_LP $*"; echo "════════════════════════════════════════"; }

# ============================================================================
# 第 1 部分：Bag 诊断
# ============================================================================
diagnose_bag() {
    print_section "【1】ROS2 Bag 诊断"
    
    if [ -z "$1" ]; then
        print_info "跳过（未指定 --bag-file）"
        return
    fi
    
    BAG_DIR="$1"
    if [ -f "$BAG_DIR" ]; then
        BAG_DIR="$(dirname "$BAG_DIR")"
    fi
    
    if [ ! -d "$BAG_DIR" ]; then
        print_error "Bag 目录不存在: $BAG_DIR"
        return 1
    fi
    
    print_info "Bag 目录: $BAG_DIR"
    
    # 检查 metadata.yaml
    METADATA="$BAG_DIR/metadata.yaml"
    if [ ! -f "$METADATA" ]; then
        print_error "metadata.yaml 不存在"
        return 1
    fi
    
    print_info "Metadata 文件大小: $(stat -f%z "$METADATA" 2>/dev/null || stat -c%s "$METADATA" 2>/dev/null || echo 'unknown') 字节"
    print_info "Metadata 权限: $(ls -l "$METADATA" | awk '{print $1}')"
    
    # 检查 bad conversion 问题（第 25 行附近）
    print_info "检查 offered_qos_profiles 问题..."
    if grep -n "offered_qos_profiles:\s*\[\]" "$METADATA" > /tmp/check_qos.txt 2>&1; then
        print_warn "发现问题: offered_qos_profiles: []（将导致 yaml-cpp bad conversion）"
        print_warn "  $(cat /tmp/check_qos.txt | head -1)"
        print_warn "  → 执行修复: python3 ${PROJECT_DIR}/scripts/fix_ros2_bag_metadata.py $BAG_DIR"
    else
        print_info "offered_qos_profiles 格式正常 ✓"
    fi
    
    # 用 Python yaml 检查
    if python3 -c "
import yaml
try:
    with open('$METADATA') as f:
        yaml.safe_load(f)
except Exception as e:
    print(f'YAML 解析失败: {e}')
    exit(1)
" 2>/tmp/yaml_check.err; then
        print_info "YAML 解析通过 ✓"
    else
        print_error "YAML 解析失败："
        cat /tmp/yaml_check.err | sed 's/^/  /'
    fi
    
    # 列出 topic
    print_info "Topic 列表："
    python3 -c "
import yaml
try:
    with open('$METADATA') as f:
        data = yaml.safe_load(f)
    topics = data.get('rosbag2_bagfile_information', {}).get('topics_with_message_count', [])
    for t in topics:
        name = t.get('topic_metadata', {}).get('name', '?')
        count = t.get('message_count', 0)
        print(f'  {name}: {count}')
except Exception as e:
    print(f'  [ERROR] 无法提取 topic: {e}')
" | head -20
    
    # 检查存储格式
    if ls "$BAG_DIR"/*.db3 > /dev/null 2>&1; then
        print_info "存储格式: SQLite3"
        DB_SIZE=$(du -sh "$BAG_DIR"/*.db3 | awk '{print $1}')
        print_info "  数据库大小: $DB_SIZE"
    elif ls "$BAG_DIR"/*.mcap > /dev/null 2>&1; then
        print_info "存储格式: MCAP"
        MCAP_SIZE=$(du -sh "$BAG_DIR"/*.mcap | awk '{print $1}')
        print_info "  数据文件大小: $MCAP_SIZE"
    else
        print_error "未找到存储文件 (*.db3 或 *.mcap)"
        return 1
    fi
}

# ============================================================================
# 第 2 部分：快速日志分析（假设已运行过）
# ============================================================================
diagnose_logs() {
    print_section "【2】运行日志诊断"
    
    if [ ! -d "$LOG_DIR" ]; then
        print_info "日志目录不存在: $LOG_DIR（首次运行？）"
        return
    fi
    
    print_info "日志目录: $LOG_DIR"
    LOG_FILES=$(find "$LOG_DIR" -name "*.log" -type f 2>/dev/null | wc -l)
    print_info "日志文件数: $LOG_FILES"
    
    # === ros2 bag play 诊断 ===
    print_info ""
    print_info "【2.1】ros2 bag play 状态"
    if find "$LOG_DIR" -name "*.log" -exec grep -l "Exception on parsing info file" {} \; 2>/dev/null | head -1 | xargs -I {} grep "Exception on parsing info file" {} | head -3 | sed 's/^/  /'; then
        print_error "检测到 bad conversion 异常！"
        print_warn "  解决方法: python3 scripts/fix_ros2_bag_metadata.py <bag_dir>"
    fi
    
    if find "$LOG_DIR" -name "*.log" -exec grep -l "\[BAG\] ros2 bag play" {} \; 2>/dev/null | head -1 | xargs -I {} grep "\[BAG\] ros2 bag play" {} | tail -1 | sed 's/^/  /'; then
        print_info "Bag 启动命令已记录"
    fi
    
    # === fast_livo 诊断 ===
    print_info ""
    print_info "【2.2】fast_livo 状态"
    if find "$LOG_DIR" -name "*.log" -exec grep -l "fast_livo.*ERROR\|fast_livo.*FATAL" {} \; 2>/dev/null | head -1; then
        print_error "fast_livo 有错误日志"
        find "$LOG_DIR" -name "*.log" -exec grep "fast_livo.*ERROR\|fast_livo.*FATAL" {} \; 2>/dev/null | head -3 | sed 's/^/  /'
    else
        print_info "fast_livo 运行状态: 未检测到错误"
    fi
    
    # 检查位姿输出
    if find "$LOG_DIR" -name "*.log" -exec grep -l "pose.json" {} \; 2>/dev/null | head -1 | xargs -I {} grep "pose.json" {} | tail -1 | sed 's/^/  /'; then
        print_info "pose.json 日志已记录"
    fi
    
    # === HBA 诊断 ===
    print_info ""
    print_info "【2.3】HBA 状态"
    if find "$LOG_DIR" -name "*.log" -exec grep -l "\[HBA\] \[FATAL\]" {} \; 2>/dev/null | head -1 | xargs -I {} grep "\[HBA\] \[FATAL\]" {} | sed 's/^/  /'; then
        print_error "HBA 启动失败（位姿或 GPS 数据不足）"
    else
        print_info "HBA 启动: 无 FATAL 错误"
    fi
    
    # === automap_system 诊断 ===
    print_info ""
    print_info "【2.4】automap_system 状态"
    if find "$LOG_DIR" -name "*.log" -exec grep -l "undefined symbol" {} \; 2>/dev/null | head -1 | xargs -I {} grep "undefined symbol" {} | sed 's/^/  /'; then
        print_error "编译问题：存在未定义符号（需重新编译）"
    else
        print_info "无符号链接问题"
    fi
    
    if find "$LOG_DIR" -name "*.log" -exec grep -l "AutoMapSystem READY" {} \; 2>/dev/null | head -1; then
        print_info "AutoMapSystem 启动成功 ✓"
    fi
}

# ============================================================================
# 第 3 部分：运行前检查
# ============================================================================
diagnose_precheck() {
    print_section "【3】运行前环境检查"
    
    # Docker 镜像
    if docker images | grep -qE "automap-env|isaac/ros|nvcr.io/nvidia/isaac"; then
        print_info "Docker 镜像: 已检测到 automap-env 或 NGC Isaac 相关镜像 ✓"
    else
        print_warn "Docker 镜像未找到（首次运行需构建）"
    fi
    
    # 工作空间
    if [ -d "$PROJECT_DIR/automap_ws/src/automap_pro" ]; then
        print_info "工作空间已链接 ✓"
    else
        print_warn "工作空间未准备好"
    fi
    
    # 数据目录
    if [ -d "$PROJECT_DIR/data" ]; then
        print_info "数据目录: $(du -sh "$PROJECT_DIR/data" | awk '{print $1}')"
    fi
}

# ============================================================================
# 第 4 部分：详细诊断汇总
# ============================================================================
print_summary() {
    print_section "【诊断总结】"
    
    print_info "常见问题排查清单："
    echo ""
    echo "❌ ros2 bag play 报 'Exception on parsing info file: bad conversion (line 25, column 29)'"
    echo "   ├─ 原因: metadata.yaml 中 offered_qos_profiles: []"
    echo "   └─ 解决: python3 scripts/fix_ros2_bag_metadata.py <bag_dir>"
    echo ""
    echo "❌ HBA 报 'insufficient poses (pose_size < WIN_SIZE)'"
    echo "   ├─ 原因: fast_livo 无位姿输出（bag 回放失败）"
    echo "   ├─ 检查项："
    echo "   │   1. ros2 bag play 是否成功？(无 bad conversion)"
    echo "   │   2. fast_livo 订阅的 topic 与 bag 是否匹配？"
    echo "   │   3. fast_livo 是否有数据处理？(检查日志)"
    echo "   │   4. 临时数据目录 (/tmp/hba_data) 是否有 pose.json？"
    echo "   └─ 建议: 降低 bag 回放速率 (--rate 0.5) 或增加系统资源"
    echo ""
    echo "❌ automap_system 报 'undefined symbol buildGlobalMap'"
    echo "   ├─ 原因: 编译产物过期或不完整"
    echo "   └─ 解决: bash run_automap.sh --clean --offline --bag-file <path>"
    echo ""
}

# ============================================================================
# 主流程
# ============================================================================
print_info "AutoMap-Pro 诊断工具启动"
print_info "日志目录: $LOG_DIR"
echo ""

BAG_FILE="${BAG_FILE:-}"
diagnose_bag "$BAG_FILE"
diagnose_logs
diagnose_precheck
print_summary

print_info "诊断完成"
