#!/bin/bash
# AutoMap-Pro 自动修复脚本
# 用途：自动检测并修复 Bag metadata 问题，一键启动建图
# 使用：bash scripts/quick_fix_and_run.sh --offline --bag-file <path> --config <yaml>

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

_LP="[quick_fix_and_run]"

print_info() { echo "$_LP [INFO] $*"; }
print_success() { echo "$_LP [✓] $*"; }
print_error() { echo "$_LP [✗] $*" >&2; }
print_warn() { echo "$_LP [!] $*" >&2; }

# ============================================================================
# 第 1 步：识别并修复 Bag 问题
# ============================================================================
fix_bag_and_report() {
    BAG_FILE="$1"
    
    if [ -z "$BAG_FILE" ]; then
        print_error "未指定 --bag-file"
        return 1
    fi
    
    BAG_DIR="$BAG_FILE"
    [ -f "$BAG_DIR" ] && BAG_DIR="$(dirname "$BAG_DIR")"
    
    if [ ! -d "$BAG_DIR" ]; then
        print_error "Bag 目录不存在: $BAG_DIR"
        return 1
    fi
    
    print_info "Bag 目录: $BAG_DIR"
    
    # 使用 Python 诊断脚本
    DIAG_SCRIPT="${PROJECT_DIR}/scripts/diagnose_and_fix_bag.py"
    if [ ! -f "$DIAG_SCRIPT" ]; then
        print_warn "诊断脚本不存在，跳过"
        return 0
    fi
    
    # 运行诊断并尝试修复
    print_info "正在检查并修复 Bag metadata..."
    if python3 "$DIAG_SCRIPT" "$BAG_DIR" --fix 2>&1 | tee /tmp/bag_diag.log; then
        DIAG_EXIT=$?
    else
        DIAG_EXIT=$?
    fi
    
    case $DIAG_EXIT in
        0)
            print_success "Bag 诊断通过，可继续启动"
            ;;
        1|2)
            print_error "Bag 参数或格式问题"
            return 1
            ;;
        3)
            print_warn "权限问题 - 尝试 chmod"
            METADATA="$BAG_DIR/metadata.yaml"
            if sudo chmod u+w "$METADATA" 2>/dev/null; then
                print_info "已修复权限，重试修复..."
                python3 "$DIAG_SCRIPT" "$BAG_DIR" --fix || print_error "修复失败"
            else
                print_error "无法获取权限修复 metadata"
                return 1
            fi
            ;;
        4)
            print_error "metadata 格式异常，可能需要手动处理"
            return 1
            ;;
        *)
            print_warn "诊断返回未知代码: $DIAG_EXIT"
            ;;
    esac
    
    # 检查修复结果
    if grep -q "Exception on parsing info file" /tmp/bag_diag.log 2>/dev/null; then
        print_error "Bag 仍存在 bad conversion 问题"
        print_error "请手动执行: chmod u+w ${BAG_DIR}/metadata.yaml && python3 ${PROJECT_DIR}/scripts/fix_ros2_bag_metadata.py ${BAG_DIR}"
        return 1
    fi
    
    print_success "Bag 准备完毕"
}

# ============================================================================
# 第 2 步：启动建图系统
# ============================================================================
run_automap() {
    print_info "启动 AutoMap-Pro 建图系统..."
    
    # 转发所有参数到 run_automap.sh
    cd "$PROJECT_DIR"
    bash "${PROJECT_DIR}/run_automap.sh" "$@"
}

# ============================================================================
# 主流程
# ============================================================================
print_info "AutoMap-Pro 快速启动脚本"

# 解析参数中的 --bag-file
BAG_FILE=""
for arg in "$@"; do
    if [ "$prev" = "--bag-file" ]; then
        BAG_FILE="$arg"
        break
    fi
    prev="$arg"
done

# 修复 Bag
if [ -n "$BAG_FILE" ]; then
    fix_bag_and_report "$BAG_FILE" || exit 1
fi

# 启动建图
print_info ""
run_automap "$@"
