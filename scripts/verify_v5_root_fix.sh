#!/usr/bin/env bash
# 验证 V5 根本修复（结构因子入图）是否生效
# 用法: bash scripts/verify_v5_root_fix.sh [full.log 路径]
#  若不传参则使用最新 logs/run_*/full.log

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_FILE="${1:-}"

if [ -z "$LOG_FILE" ]; then
  LATEST=$(ls -td "${REPO_DIR}"/logs/run_*/full.log 2>/dev/null | head -1)
  if [ -z "$LATEST" ]; then
    echo "[ERROR] 未找到 logs/run_*/full.log，请指定: bash $0 <path/to/full.log>"
    exit 1
  fi
  LOG_FILE="$LATEST"
  echo "[INFO] 使用最新日志: $LOG_FILE"
fi

if [ ! -f "$LOG_FILE" ]; then
  echo "[ERROR] 文件不存在: $LOG_FILE"
  exit 1
fi

echo "========================================="
echo " V5 根本修复验证"
echo " 日志: $LOG_FILE"
echo "========================================="

PASS=0
FAIL=0

# 1) 应出现：V5 两阶段注入成功
if grep -q "first_update_v5_post_inject_factors" "$LOG_FILE" && grep -q "structural factors" "$LOG_FILE"; then
  echo "[PASS] V5 两阶段（values + 结构因子）已执行"
  ((PASS++))
else
  echo "[FAIL] 未发现 first_update_v5_post_inject_factors / structural factors（可能未跑到首子图冻结或日志不全）"
  ((FAIL++))
fi

# 2) 不应出现：Indeterminant linear system（第二次 forceUpdate 奇异）
COUNT_INDET=$(grep -c "Indeterminant linear system" "$LOG_FILE" 2>/dev/null || echo 0)
if [ "$COUNT_INDET" -eq 0 ]; then
  echo "[PASS] 无 Indeterminant linear system 异常"
  ((PASS++))
else
  echo "[FAIL] 发现 $COUNT_INDET 处 Indeterminant linear system"
  ((FAIL++))
fi

# 3) 不应出现：Removed stale node_exists_（断链）
COUNT_STALE=$(grep -c "Removed stale node_exists_ entry" "$LOG_FILE" 2>/dev/null || echo 0)
if [ "$COUNT_STALE" -eq 0 ]; then
  echo "[PASS] 无 Removed stale node_exists_（子图链未误删）"
  ((PASS++))
else
  echo "[FAIL] 发现 $COUNT_STALE 处 Removed stale node_exists_"
  ((FAIL++))
fi

# 4) 不应出现：addOdomFactor_skip from_exists=0
COUNT_SKIP=$(grep -c "addOdomFactor_skip.*from_exists=0" "$LOG_FILE" 2>/dev/null || echo 0)
if [ "$COUNT_SKIP" -eq 0 ]; then
  echo "[PASS] 无 addOdomFactor_skip from_exists=0"
  ((PASS++))
else
  echo "[FAIL] 发现 $COUNT_SKIP 处 addOdomFactor_skip from_exists=0"
  ((FAIL++))
fi

# 5) 应有：addOdomFactor_added（子图间 Odom 正常添加）
COUNT_ADDED=$(grep -c "addOdomFactor_added" "$LOG_FILE" 2>/dev/null || echo 0)
echo "[INFO] addOdomFactor_added 出现次数: $COUNT_ADDED"

echo "========================================="
echo " 结果: PASS=$PASS FAIL=$FAIL"
echo "========================================="
[ "$FAIL" -eq 0 ] && echo "总体: 通过" && exit 0
echo "总体: 未通过（见上方 FAIL 项）"
exit 1
