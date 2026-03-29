#!/usr/bin/env bash
# Audit loop-closure related lines in automap / full logs (run after offline mapping).
# Usage: ./scripts/loop_closure_log_audit.sh [DIR_OR_FULL_LOG]
# Default DIR: logs/run_20260329_213708 (override with first arg)

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIR="${1:-$ROOT/logs/run_20260329_213708}"
FULL="$DIR/full.log"
AUTO="$DIR/automap.log"

echo "=== loop_closure_log_audit ==="
echo "DIR=$DIR"

for f in "$FULL" "$AUTO"; do
  if [[ ! -f "$f" ]]; then
    echo "SKIP (missing): $f"
    continue
  fi
  echo ""
  echo "--- $(basename "$f") ---"
  echo "INTRA_LOOP][SUMMARY] count: $(grep -c '\[INTRA_LOOP\]\[SUMMARY\]' "$f" 2>/dev/null || true)"
  echo "INTRA_LOOP][DETECTED count: $(grep -c '\[INTRA_LOOP\]\[DETECTED\]' "$f" 2>/dev/null || true)"
  echo "INTER_KF][LOOP_ACCEPTED count: $(grep -c '\[INTER_KF\]\[LOOP_ACCEPTED\]' "$f" 2>/dev/null || true)"
  echo "[LOOP_ACCEPTED] (RCLCPP) count: $(grep -c '\[LOOP_ACCEPTED\]' "$f" 2>/dev/null || true)"
  echo "addLoopFactorBetweenKeyframes ENTER count: $(grep -c 'addLoopFactorBetweenKeyframes ENTER' "$f" 2>/dev/null || true)"
done

echo ""
echo "=== Frame contract sample (V3 CONTRACT vs GHOSTING) ==="
if [[ -f "$FULL" ]]; then
  echo "Unique cloud_frame= in CONTRACT (first 20 lines):"
  grep '\[V3\]\[CONTRACT\].*cloud_frame=' "$FULL" | sed -n 's/.*cloud_frame=\([^ ]*\).*/\1/p' | sort -u | head -20 || true
  echo "GHOSTING_TRACE merge pose_source= (expect T_map_b_optimized):"
  grep 'GHOSTING_TRACE.*mergeCloudToSubmap' "$FULL" | grep -o 'pose_source=[^ ]*' | sort -u | head -5 || true
fi

echo ""
echo "=== Code note (V3) ==="
echo "Intra-submap detection runs only when LoopModule receives IntraLoopTaskEvent."
echo "MapRegistry::addKeyFrame publishes that event when loop_closure.intra_submap_enabled is true."
echo "If SUMMARY/DETECTED stay 0 on an old log, the binary may predate that publish (grep this script's git history)."
