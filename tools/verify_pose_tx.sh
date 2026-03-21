#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <full.log>"
  exit 1
fi

LOG_FILE="$1"
if [[ ! -f "$LOG_FILE" ]]; then
  echo "Log file not found: $LOG_FILE"
  exit 1
fi

echo "== Pose Transaction Signals =="
SEARCH_BIN="rg"
if ! command -v rg >/dev/null 2>&1; then
  SEARCH_BIN="grep"
fi

if [[ "$SEARCH_BIN" == "rg" ]]; then
  rg -n "POSE_TX\\]\\[APPLY_BEGIN|POSE_TX\\]\\[APPLY_END|POSE_TX\\]\\[DEFER|POSE_TX\\]\\[FLUSH|POSE_TX\\]\\[BARRIER|POSE_TX\\]\\[INVARIANT" "$LOG_FILE" | head -n 200 || true
else
  grep -nE "POSE_TX\\]\\[APPLY_BEGIN|POSE_TX\\]\\[APPLY_END|POSE_TX\\]\\[DEFER|POSE_TX\\]\\[FLUSH|POSE_TX\\]\\[BARRIER|POSE_TX\\]\\[INVARIANT" "$LOG_FILE" | head -n 200 || true
fi

echo
echo "== Legacy Risk Signals =="
if [[ "$SEARCH_BIN" == "rg" ]]; then
  rg -n "POSE_JUMP|GPS_TRANSFORM|LOOP\\]\\[POSE_DIAG" "$LOG_FILE" | head -n 200 || true
else
  grep -nE "POSE_JUMP|GPS_TRANSFORM|LOOP\\]\\[POSE_DIAG" "$LOG_FILE" | head -n 200 || true
fi

echo
echo "== Counts =="
if [[ "$SEARCH_BIN" == "rg" ]]; then
  echo -n "APPLY_BEGIN: "; rg -c "POSE_TX\\]\\[APPLY_BEGIN" "$LOG_FILE" || true
  echo -n "APPLY_END  : "; rg -c "POSE_TX\\]\\[APPLY_END" "$LOG_FILE" || true
  echo -n "DEFER      : "; rg -c "POSE_TX\\]\\[DEFER" "$LOG_FILE" || true
  echo -n "FLUSH      : "; rg -c "POSE_TX\\]\\[FLUSH" "$LOG_FILE" || true
  echo -n "BARRIER on : "; rg -c "POSE_TX\\]\\[BARRIER\\] activated" "$LOG_FILE" || true
  echo -n "BARRIER off: "; rg -c "POSE_TX\\]\\[BARRIER\\] deactivated" "$LOG_FILE" || true
  echo -n "INVARIANT  : "; rg -c "POSE_TX\\]\\[INVARIANT" "$LOG_FILE" || true
else
  echo -n "APPLY_BEGIN: "; grep -cE "POSE_TX\\]\\[APPLY_BEGIN" "$LOG_FILE" || true
  echo -n "APPLY_END  : "; grep -cE "POSE_TX\\]\\[APPLY_END" "$LOG_FILE" || true
  echo -n "DEFER      : "; grep -cE "POSE_TX\\]\\[DEFER" "$LOG_FILE" || true
  echo -n "FLUSH      : "; grep -cE "POSE_TX\\]\\[FLUSH" "$LOG_FILE" || true
  echo -n "BARRIER on : "; grep -cE "POSE_TX\\]\\[BARRIER\\] activated" "$LOG_FILE" || true
  echo -n "BARRIER off: "; grep -cE "POSE_TX\\]\\[BARRIER\\] deactivated" "$LOG_FILE" || true
  echo -n "INVARIANT  : "; grep -cE "POSE_TX\\]\\[INVARIANT" "$LOG_FILE" || true
fi
