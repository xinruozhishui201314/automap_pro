#!/usr/bin/env bash
# Summarize primary_reason=... from [TEASER_REJECT] and [LOOP_REJECTED] lines.
# Usage: ./scripts/quantify_teaser_reject.sh /path/to/full.log

set -euo pipefail
LOG="${1:?usage: $0 /path/to/full.log}"

if [[ ! -f "$LOG" ]]; then
  echo "File not found: $LOG" >&2
  exit 1
fi

echo "=== TEASER_REJECT primary_reason (RCLCPP) ==="
grep '\[TEASER_REJECT\]' "$LOG" | sed -n 's/.*primary_reason=\([^ ]*\).*/\1/p' | sort | uniq -c | sort -nr || true

echo ""
echo "=== LOOP_REJECTED primary_reason (LoopDetector ALOG mirror) ==="
grep '\[LOOP_REJECTED\]' "$LOG" | sed -n 's/.*primary_reason=\([^ ]*\).*/\1/p' | sort | uniq -c | sort -nr || true

echo ""
echo "=== TEASER_SUMMARY totals ==="
echo "TEASER_SUMMARY lines: $(grep -c '\[TEASER_SUMMARY\]' "$LOG" || true)"
echo "TEASER_SUMMARY with accepted=0: $(grep '\[TEASER_SUMMARY\]' "$LOG" | grep -c 'accepted=0' || true)"

echo ""
echo "Compare thresholds in YAML: loop_closure.teaser.min_inlier_ratio, min_safe_inliers, max_rmse_m"
echo "Source: automap_pro/src/loop_closure/loop_detector.cpp (processMatchTask)"
