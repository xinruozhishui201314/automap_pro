#!/bin/bash
# Git LFS 管理脚本
# 用于管理大文件的追踪、上传和下载

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示帮助信息
show_help() {
    cat << EOF
Git LFS 管理脚本 - 自动化大文件版本控制

用法: $0 [命令] [选项]

命令:
    init            初始化 Git LFS
    track           追踪大文件到 LFS
    status          显示 LFS 状态
    migrate         迁移现有大文件到 LFS
    pull            拉取所有 LFS 文件
    push            推送所有 LFS 文件
    clean           清理未追踪的大文件
    info            显示 LFS 使用情况
    help            显示此帮助信息

选项:
    --dry-run       预览操作但不执行
    --size MB       指定文件大小阈值（MB）
    --pattern PAT   指定文件模式

示例:
    $0 init                     # 初始化 Git LFS
    $0 track --size 10          # 追踪大于 10MB 的文件
    $0 migrate                  # 迁移现有大文件
    $0 status                   # 显示 LFS 状态
    $0 info                     # 显示 LFS 使用情况

EOF
}

# 检查 Git LFS 是否安装
check_lfs() {
    if ! command -v git-lfs &> /dev/null; then
        echo -e "${RED}错误: Git LFS 未安装${NC}"
        echo "请安装: sudo apt install git-lfs (Ubuntu/Debian)"
        echo "       brew install git-lfs (macOS)"
        exit 1
    fi
}

# 初始化 Git LFS
lfs_init() {
    echo -e "${BLUE}初始化 Git LFS...${NC}"
    git lfs install
    echo -e "${GREEN}✓ Git LFS 已初始化${NC}"
}

# 追踪大文件
lfs_track() {
    local SIZE_THRESHOLD=${2:-10}  # 默认 10MB
    local DRY_RUN=false

    if [[ "$*" == *"--dry-run"* ]]; then
        DRY_RUN=true
        echo -e "${YELLOW}预览模式: 不会实际执行${NC}"
    fi

    echo -e "${BLUE}查找大于 ${SIZE_THRESHOLD}MB 的文件...${NC}"

    # 查找大文件
    local LARGE_FILES=$(find . -type f ! -path "./.git/*" ! -path "./backup-*/*" \
        -exec sh -c 'size=$(stat -f%z "$1" 2>/dev/null || stat -c%s "$1" 2>/dev/null || echo 0); [ "$size" -gt $((SIZE_THRESHOLD * 1024 * 1024)) ] && echo "$1"' _ {} \;)

    if [ -z "$LARGE_FILES" ]; then
        echo -e "${GREEN}没有找到大于 ${SIZE_THRESHOLD}MB 的文件${NC}"
        return 0
    fi

    echo "找到以下大文件:"
    echo "$LARGE_FILES" | while read -r file; do
        local size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null || echo 0)
        local size_mb=$((size / 1024 / 1024))
        printf "  - %s (%d MB)\n" "$file" "$size_mb"
    done

    if [ "$DRY_RUN" = false ]; then
        echo -e "${YELLOW}这些文件将由 .gitattributes 中的规则自动处理${NC}"
        echo -e "${YELLOW}请检查 .gitattributes 确保文件类型已配置 LFS${NC}"
    fi
}

# 显示 LFS 状态
lfs_status() {
    echo -e "${BLUE}Git LFS 状态${NC}"
    echo "================================"

    # 检查 LFS 是否初始化
    if [ ! -f .git/hooks/pre-push ]; then
        echo -e "${YELLOW}⚠ Git LFS 未初始化${NC}"
        echo "运行: $0 init"
        return 1
    fi

    # 显示追踪的文件类型
    echo -e "\n${GREEN}追踪的文件类型:${NC}"
    if [ -f .gitattributes ]; then
        grep "filter=lfs" .gitattributes | awk '{print "  - " $1}'
    fi

    # 显示当前 LFS 文件
    echo -e "\n${GREEN}当前的 LFS 文件:${NC}"
    git lfs ls-files | head -20
    local count=$(git lfs ls-files | wc -l)
    if [ "$count" -gt 20 ]; then
        echo "  ... 还有 $((count - 20)) 个文件"
    fi

    # 显示未追踪的大文件
    echo -e "\n${YELLOW}未追踪的大文件 (> 10MB):${NC}"
    local UNTRACKED=$(find . -type f ! -path "./.git/*" ! -path "./backup-*/*" \
        -exec sh -c 'size=$(stat -f%z "$1" 2>/dev/null || stat -c%s "$1" 2>/dev/null || echo 0); [ "$size" -gt 10485760 ] && echo "$1"' _ {} \; \
        | while read -r file; do
            if ! git lfs ls-files "$file" &> /dev/null && ! git check-ignore -q "$file"; then
                echo "$file"
            fi
        done)

    if [ -z "$UNTRACKED" ]; then
        echo -e "${GREEN}✓ 所有大于 10MB 的文件都已追踪${NC}"
    else
        echo "$UNTRACKED" | head -10
        local untracked_count=$(echo "$UNTRACKED" | wc -l)
        if [ "$untracked_count" -gt 10 ]; then
            echo "  ... 还有 $((untracked_count - 10)) 个文件"
        fi
        echo -e "\n${YELLOW}建议: 使用 git add <file> 提交这些文件${NC}"
    fi

    echo "================================"
}

# 迁移现有大文件到 LFS
lfs_migrate() {
    local SIZE_THRESHOLD=${2:-10}
    local DRY_RUN=false

    if [[ "$*" == *"--dry-run"* ]]; then
        DRY_RUN=true
        echo -e "${YELLOW}预览模式: 不会实际迁移${NC}"
    fi

    echo -e "${BLUE}迁移大文件到 Git LFS...${NC}"
    echo -e "${YELLOW}警告: 此操作将重写 Git 历史记录${NC}"

    if [ "$DRY_RUN" = false ]; then
        read -p "是否继续? (yes/no): " confirm
        if [ "$confirm" != "yes" ]; then
            echo "操作已取消"
            return 0
        fi
    fi

    # 查找需要迁移的文件
    echo "查找大于 ${SIZE_THRESHOLD}MB 的已提交文件..."

    local FILES_TO_MIGRATE=$(git rev-list --objects --all | git cat-file --batch-check='%(objecttype) %(objectname) %(objectsize) %(rest)' \
        | awk '/^blob / {print $4 "\t" $3}' \
        | awk -v threshold=$((SIZE_THRESHOLD * 1024 * 1024)) '$2 > threshold {print $1}')

    if [ -z "$FILES_TO_MIGRATE" ]; then
        echo -e "${GREEN}没有找到需要迁移的文件${NC}"
        return 0
    fi

    local count=$(echo "$FILES_TO_MIGRATE" | wc -l)
    echo "找到 $count 个文件需要迁移"
    echo "$FILES_TO_MIGRATE" | head -10

    if [ "$DRY_RUN" = true ]; then
        return 0
    fi

    # 创建临时文件列表
    local TEMP_FILE=$(mktemp)
    echo "$FILES_TO_MIGRATE" > "$TEMP_FILE"

    # 执行迁移
    echo "开始迁移..."
    git lfs migrate import --include-from="$TEMP_FILE" --everything

    # 清理临时文件
    rm -f "$TEMP_FILE"

    echo -e "${GREEN}✓ 迁移完成${NC}"
    echo -e "${YELLOW}注意: 请强制推送到远程: git push origin --force${NC}"
}

# 拉取所有 LFS 文件
lfs_pull() {
    echo -e "${BLUE}拉取所有 LFS 文件...${NC}"
    git lfs pull --all
    echo -e "${GREEN}✓ 拉取完成${NC}"
}

# 推送所有 LFS 文件
lfs_push() {
    echo -e "${BLUE}推送所有 LFS 文件...${NC}"
    git lfs push --all origin main
    echo -e "${GREEN}✓ 推送完成${NC}"
}

# 显示 LFS 使用情况
lfs_info() {
    echo -e "${BLUE}Git LFS 使用情况${NC}"
    echo "================================"

    # 显示版本
    git lfs version

    # 显示存储使用情况（需要与远程交互）
    echo -e "\n${GREEN}本地 LFS 文件统计:${NC}"
    local lfs_count=$(git lfs ls-files | wc -l)
    echo "  LFS 文件数量: $lfs_count"

    if [ "$lfs_count" -gt 0 ]; then
        local lfs_size=$(du -sh $(git lfs ls-files | awk '{print $NF}') 2>/dev/null | awk '{sum+=$1} END {print sum}' | numfmt --to=iec-i --suffix=B)
        echo "  LFS 文件总大小: $(du -ch $(git lfs ls-files | awk '{print $NF}') 2>/dev/null | tail -1 | awk '{print $1}')"
    fi

    # 显示远程存储配额（如果有）
    if git remote -v | grep -q github.com; then
        echo -e "\n${YELLOW}GitHub LFS 配额:${NC}"
        echo "  请访问: https://github.com/settings/billing"
        echo "  查看您的 LFS 存储和带宽使用情况"
    fi

    echo "================================"
}

# 主函数
main() {
    check_lfs

    case "${1:-}" in
        init)
            lfs_init
            ;;
        track)
            lfs_track "$@"
            ;;
        status)
            lfs_status
            ;;
        migrate)
            lfs_migrate "$@"
            ;;
        pull)
            lfs_pull
            ;;
        push)
            lfs_push
            ;;
        info)
            lfs_info
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            echo -e "${RED}错误: 未知命令 '${1:-}'${NC}"
            show_help
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@"
