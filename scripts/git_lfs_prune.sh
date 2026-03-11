#!/usr/bin/env bash
# =============================================================================
# 定期执行 git lfs prune 的脚本
# 用途：释放 .git/lfs 中未被当前分支引用的 LFS 对象，节省磁盘空间。
# 使用：可手动执行，或通过 cron 定期执行（见脚本末尾说明）。
# =============================================================================

set -e

# 默认：不校验远程（速度快；若需更安全可加 --verify-remote）
VERIFY_REMOTE=""
DRY_RUN=""
VERBOSE=""
LOG_FILE=""
REPO_DIR=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS] [REPO_DIR]

  REPO_DIR    可选，Git 仓库根目录；不传则使用当前目录（或自动向上查找 .git）

Options:
  --dry-run       仅打印将执行的操作，不实际 prune
  --verbose       输出详细日志（传给 git lfs prune）
  --verify-remote prune 前校验对象在远程存在（更安全但更慢）
  --log FILE      将输出追加到 FILE（便于 cron 审计）
  -h, --help      显示此帮助

Cron 示例（每周日凌晨 3 点执行）：
  ###########################################################################
  # 在项目根目录执行 LFS prune
  ###########################################################################
  0 3 * * 0 /home/wqs/Documents/github/automap_pro/scripts/git_lfs_prune.sh --log /home/wqs/Documents/github/automap_pro/logs/git_lfs_prune.log

  或仅对本用户生效：crontab -e 添加上面一行（路径按实际修改）。
EOF
  exit 0
}

# 解析参数
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)    usage ;;
    --dry-run)    DRY_RUN=1; shift ;;
    --verbose)    VERBOSE=1; shift ;;
    --verify-remote) VERIFY_REMOTE="--verify-remote"; shift ;;
    --log)        LOG_FILE="$2"; shift 2 ;;
    -*)
      echo "Unknown option: $1" >&2
      usage
      ;;
    *)
      REPO_DIR="$1"
      shift
      ;;
  esac
done

# 若未指定 REPO_DIR，使用当前目录或向上查找含 .git 的目录
if [[ -z "$REPO_DIR" ]]; then
  REPO_DIR="$PWD"
  while [[ "$REPO_DIR" != "/" ]]; do
    [[ -d "$REPO_DIR/.git" ]] && break
    REPO_DIR="$(dirname "$REPO_DIR")"
  done
  if [[ "$REPO_DIR" == "/" ]]; then
    echo "Error: not inside a Git repository." >&2
    exit 1
  fi
fi

if [[ ! -d "$REPO_DIR/.git" ]]; then
  echo "Error: not a Git repo: $REPO_DIR" >&2
  exit 1
fi

# 依赖检查
if ! command -v git &>/dev/null; then
  echo "Error: git not found." >&2
  exit 1
fi
if ! git lfs version &>/dev/null; then
  echo "Error: git lfs not installed or not in PATH." >&2
  exit 1
fi

# 可选：记录到日志文件
log() {
  local msg="[$(date '+%Y-%m-%d %H:%M:%S')] $*"
  echo "$msg"
  if [[ -n "$LOG_FILE" ]]; then
    mkdir -p "$(dirname "$LOG_FILE")"
    echo "$msg" >> "$LOG_FILE"
  fi
}

cd "$REPO_DIR"

# 若无 LFS 或 .git/lfs 不存在，跳过并退出 0
if [[ ! -d ".git/lfs" ]]; then
  log "Skip: no .git/lfs in $REPO_DIR"
  exit 0
fi

# 清理前占用（可选，便于审计）
LFS_SIZE_BEFORE=""
if command -v du &>/dev/null; then
  LFS_SIZE_BEFORE=$(du -sh .git/lfs 2>/dev/null | cut -f1)
  log "LFS size before: $LFS_SIZE_BEFORE"
fi

if [[ -n "$DRY_RUN" ]]; then
  log "Dry-run: would run in $REPO_DIR: git lfs prune $VERIFY_REMOTE ${VERBOSE:+--verbose}"
  exit 0
fi

PRUNE_OPTS=($VERIFY_REMOTE)
[[ -n "$VERBOSE" ]] && PRUNE_OPTS+=(--verbose)

log "Running: git lfs prune ${PRUNE_OPTS[*]}"
PRUNE_OUTPUT=$(git lfs prune "${PRUNE_OPTS[@]}" 2>&1) || exit_code=$?
while IFS= read -r line; do log "$line"; done <<< "$PRUNE_OUTPUT"
if [[ -n "${exit_code:-}" ]]; then
  log "git lfs prune exited with code $exit_code"
  exit "$exit_code"
fi

# 清理后占用
if command -v du &>/dev/null && [[ -d ".git/lfs" ]]; then
  LFS_SIZE_AFTER=$(du -sh .git/lfs 2>/dev/null | cut -f1)
  log "LFS size after:  $LFS_SIZE_AFTER (before: $LFS_SIZE_BEFORE)"
fi

log "Done."
exit 0
