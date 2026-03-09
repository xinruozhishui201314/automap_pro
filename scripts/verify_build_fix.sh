#!/usr/bin/env bash
# 验证 run_automap.sh --build-only --clean 报错的修复是否生效
# 修复范围：pose_graph, global_map, optimizer, map_builder, map_exporter, icp_refiner, utils
# 成功条件：全量构建通过，或上述模块无 error 且 automap_backend/automap_map/automap_loop_closure/automap_core 已 Built
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

# 构建失败时，检查本次修复的文件是否仍报错
FIXED_FILES=(
    "pose_graph.cpp"
    "global_map"
    "optimizer.cpp"
    "map_builder.cpp"
    "map_exporter.cpp"
    "icp_refiner"
    "utils.cpp"
)
for f in "${FIXED_FILES[@]}"; do
    if grep -q "$f.*error:" "$LOG_FILE" 2>/dev/null; then
        echo "[verify] FAIL: $f still has errors."
        grep "$f.*error:" "$LOG_FILE" | head -5
        exit 1
    fi
done

# 检查已修复的 target 是否已成功编译
REQUIRED_TARGETS="automap_core|automap_backend|automap_map|automap_loop_closure"
MISSING=""
for t in automap_core automap_backend automap_map automap_loop_closure; do
    if ! grep -q "Built target $t" "$LOG_FILE" 2>/dev/null; then
        MISSING="$MISSING $t"
    fi
done
if [[ -n "$MISSING" ]]; then
    echo "[verify] FAIL: expected targets not built:$MISSING (see $LOG_FILE)"
    exit 1
fi

echo "[verify] OK: pose_graph/global_map/optimizer/map_builder/map_exporter/icp_refiner/utils fixes verified (no errors in fixed files; backend/map/loop_closure/core built)."
echo "[verify] Remaining failures (if any) are in other targets (e.g. automap_system_component). See $LOG_FILE"
exit 0
