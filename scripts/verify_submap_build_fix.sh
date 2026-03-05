#!/usr/bin/env bash
# 验证 submap/structured_logger/metrics/health_monitor 相关编译修复是否生效
# 成功条件：构建日志中出现 "Built target automap_submap" 且无 submap_manager/structured_logger 相关 error
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_FILE="${LOG_FILE:-$REPO_ROOT/build_verify.log}"

cd "$REPO_ROOT"
echo "[verify] Running build (log: $LOG_FILE)..."
if bash run_automap.sh --build-only --clean > "$LOG_FILE" 2>&1; then
    echo "[verify] Full build succeeded."
    exit 0
fi

# 构建失败时，检查是否为我们修复的模块报错
if grep -q "submap_manager\.cpp.*error:" "$LOG_FILE" 2>/dev/null; then
    echo "[verify] FAIL: submap_manager.cpp still has errors."
    grep "submap_manager.cpp.*error:" "$LOG_FILE" | head -5
    exit 1
fi
if grep -q "structured_logger\.h.*error:" "$LOG_FILE" 2>/dev/null; then
    echo "[verify] FAIL: structured_logger.h still has errors."
    grep "structured_logger\.h.*error:" "$LOG_FILE" | head -5
    exit 1
fi
if grep -q "metrics\.h.*error:" "$LOG_FILE" 2>/dev/null; then
    echo "[verify] FAIL: metrics.h still has errors."
    grep "metrics\.h.*error:" "$LOG_FILE" | head -5
    exit 1
fi
if grep -q "health_monitor\.h.*error:" "$LOG_FILE" 2>/dev/null; then
    echo "[verify] FAIL: health_monitor.h still has errors."
    grep "health_monitor\.h.*error:" "$LOG_FILE" | head -5
    exit 1
fi

# 检查 submap 目标是否已成功编译
if grep -q "Built target automap_submap" "$LOG_FILE" 2>/dev/null; then
    echo "[verify] OK: automap_submap built successfully (submap/metrics/logger/health fixes verified)."
    echo "[verify] Remaining failures are in other targets (backend/map/loop_closure etc.)."
    exit 0
fi

echo "[verify] FAIL: automap_submap did not build. See $LOG_FILE"
exit 1
