#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# 使用 Git LFS 将工程安全上传到 GitHub
#
# 承诺：本脚本**不会删除任何本地代码或文件**，仅做 LFS 初始化和推送准备。
# 大文件（.db3/.bag/.pth/.tar 等）由 .gitattributes 自动走 LFS。
#
# 用法：
#   bash scripts/upload_to_github_with_lfs.sh [--commit-and-push]
#
# 若不加 --commit-and-push，仅检查环境并打印后续步骤。
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

echo "══════════════════════════════════════════════"
echo "  Git LFS 上传检查（不删除任何本地文件）"
echo "══════════════════════════════════════════════"

# 1. 确保 Git LFS 已安装并初始化
if ! command -v git-lfs &>/dev/null; then
    echo "[ERROR] 未检测到 git-lfs，请先安装："
    echo "  Ubuntu/Debian: sudo apt install git-lfs"
    echo "  然后执行: git lfs install"
    exit 1
fi
git lfs install
echo "[OK] Git LFS 已就绪"

# 2. 确认 .gitattributes 存在且含 LFS 规则
if ! grep -q "filter=lfs" .gitattributes 2>/dev/null; then
    echo "[ERROR] .gitattributes 中未找到 LFS 规则，请检查 .gitattributes"
    exit 1
fi
echo "[OK] .gitattributes 已配置 LFS"

# 3. 列出当前会被 LFS 追踪的扩展名（供核对）
echo ""
echo "当前由 Git LFS 追踪的规则："
git lfs track 2>/dev/null | head -20
echo "  ..."

# 4. 可选：仅提交 .gitattributes 的变更（不 touch 其他文件）
if [[ "${1:-}" == "--commit-and-push" ]]; then
    if git diff --quiet .gitattributes 2>/dev/null && git diff --cached --quiet .gitattributes 2>/dev/null; then
        echo "[INFO] .gitattributes 无变更，跳过提交"
    else
        git add .gitattributes
        git commit -m "chore: update Git LFS rules (.gitattributes)" || true
        echo "[OK] 已提交 .gitattributes"
    fi
    echo ""
    echo "接下来可手动执行："
    echo "  git add .                    # 添加所有变更（大文件会自动走 LFS）"
    echo "  git status                  # 确认后再提交"
    echo "  git commit -m 'your msg'"
    echo "  git push -u origin main      # 推送到 GitHub"
    echo ""
    echo "说明：上述 git add/commit 不会删除本地任何文件。"
    exit 0
fi

# 5. 仅检查模式：打印后续步骤
echo ""
echo "──────────────────────────────────────────────"
echo "后续步骤（不会删除任何本地文件）："
echo "──────────────────────────────────────────────"
echo "1. 添加要提交的文件（大文件会自动用 LFS）："
echo "   git add ."
echo ""
echo "2. 查看状态确认："
echo "   git status"
echo "   git lfs status   # 查看哪些文件将用 LFS"
echo ""
echo "3. 提交并推送："
echo "   git commit -m 'feat: sync project with LFS'"
echo "   git push -u origin main"
echo ""
echo "若只想先提交 .gitattributes 的修改，可运行："
echo "   bash scripts/upload_to_github_with_lfs.sh --commit-and-push"
echo ""
