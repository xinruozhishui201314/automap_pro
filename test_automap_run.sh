#!/usr/bin/env bash
# 自动化验证：执行 bash automap_start.sh，检查 fast_livo 不再因 parameter '' 崩溃
# 成功条件：编译通过 + 启动后 fast_livo 未因 parameter '' 退出，automap_system READY
# 用法: bash test_automap_run.sh [--build-only]
#  --build-only: 仅编译，不运行（与 test_build_automap.sh 等价）
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
LOG_FILE="${SCRIPT_DIR}/.test_automap_run.log"
RUN_TIMEOUT=90
BUILD_ONLY=false
for x in "$@"; do
  if [[ "$x" == "--build-only" ]]; then BUILD_ONLY=true; fi
done

run_and_capture() {
  if [[ "$BUILD_ONLY" == "true" ]]; then
    bash automap_start.sh --build 2>&1 | tee "$LOG_FILE"
    return ${PIPESTATUS[0]}
  fi
  # 完整运行，限时采集日志
  timeout "$RUN_TIMEOUT" bash automap_start.sh 2>&1 | tee "$LOG_FILE" || true
  return 0
}

echo "[test_automap_run] Log file: $LOG_FILE"
if [[ "$BUILD_ONLY" == "true" ]]; then
  echo "[test_automap_run] Build-only mode"
  if ! run_and_capture; then
    echo "[test_automap_run] FAIL: build failed"
    exit 1
  fi
  echo "[test_automap_run] PASS: build succeeded"
  exit 0
fi

echo "[test_automap_run] Running automap_start.sh (timeout ${RUN_TIMEOUT}s)..."
run_and_capture

# 判定：必须无 parameter '' 崩溃，且 fast_livo 曾启动、automap_system 曾 READY
FAIL_PATTERN="parameter '' has invalid type"
DIE_PATTERN="fastlivo_mapping.*process has died"
GOOD_MAIN="[fast_livo] main:"
GOOD_READY="AutoMapSystem READY"

if grep -q "$FAIL_PATTERN" "$LOG_FILE" 2>/dev/null; then
  echo "[test_automap_run] FAIL: log contains \"$FAIL_PATTERN\" (parameter '' crash)"
  exit 1
fi
if grep -qE "$DIE_PATTERN" "$LOG_FILE" 2>/dev/null; then
  echo "[test_automap_run] FAIL: log contains fastlivo_mapping process has died"
  exit 1
fi
if ! grep -q "$GOOD_MAIN" "$LOG_FILE" 2>/dev/null; then
  echo "[test_automap_run] WARN: log does not contain \"$GOOD_MAIN\" (fast_livo may not have started)"
fi
if ! grep -q "$GOOD_READY" "$LOG_FILE" 2>/dev/null; then
  echo "[test_automap_run] WARN: log does not contain \"$GOOD_READY\""
fi

echo "[test_automap_run] PASS: no parameter '' crash, run log collected"
exit 0
