#!/usr/bin/env bash

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

pass() { echo -e "${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; exit 1; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

LOG_FILE=""
SKIP_BUILD_CHECK=0
SKIP_TEST_CHECK=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --log)
      LOG_FILE="${2:-}"
      shift 2
      ;;
    --skip-build-check)
      SKIP_BUILD_CHECK=1
      shift
      ;;
    --skip-test-check)
      SKIP_TEST_CHECK=1
      shift
      ;;
    *)
      fail "Unknown argument: $1 (supported: --log <file> --skip-build-check --skip-test-check)"
      ;;
  esac
done

CFG_H="${PROJECT_ROOT}/include/automap_pro/core/config_manager.h"
CFG_CPP="${PROJECT_ROOT}/src/core/config_manager.cpp"
CFG_YAML="${PROJECT_ROOT}/config/system_config_M2DGR.yaml"
LOOP_CPP="${PROJECT_ROOT}/src/loop_closure/loop_detector.cpp"
TEASER_CPP="${PROJECT_ROOT}/src/loop_closure/teaser_matcher.cpp"

for f in "$CFG_H" "$CFG_CPP" "$CFG_YAML" "$LOOP_CPP" "$TEASER_CPP"; do
  [[ -f "$f" ]] || fail "Required file missing: $f"
done

SEARCH_TOOL=""
if command -v rg >/dev/null 2>&1; then
  SEARCH_TOOL="rg"
elif command -v grep >/dev/null 2>&1; then
  SEARCH_TOOL="grep"
else
  fail "Neither rg nor grep is available"
fi

search() {
  local pattern="$1"
  local target="$2"
  if [[ "$SEARCH_TOOL" == "rg" ]]; then
    rg -n "$pattern" "$target" >/dev/null
  else
    grep -nE "$pattern" "$target" >/dev/null
  fi
}

BUILD_DIR="${PROJECT_ROOT}/build"
if [[ ! -d "$BUILD_DIR" ]]; then
  BUILD_DIR="$(cd "${PROJECT_ROOT}/.." && pwd)/build"
fi

echo "=============================================="
echo " V3 Stability Baseline Checker"
echo " PROJECT_ROOT=${PROJECT_ROOT}"
echo "=============================================="

echo ""
echo "0) Build success gate (mandatory)"
if [[ "$SKIP_BUILD_CHECK" -eq 0 ]]; then
  if command -v cmake >/dev/null 2>&1; then
    [[ -d "$BUILD_DIR" ]] || fail "Build directory not found: $BUILD_DIR"
    cmake --build "$BUILD_DIR" -j"$(nproc)" --target automap_system_component >/tmp/v3_baseline_build.log 2>&1 \
      && pass "build success gate passed (automap_system_component)" \
      || {
        if command -v rg >/dev/null 2>&1; then
          rg -n "error:|failed|undefined reference" /tmp/v3_baseline_build.log || true
        else
          grep -nE "error:|failed|undefined reference" /tmp/v3_baseline_build.log || true
        fi
        fail "build success gate failed, see /tmp/v3_baseline_build.log"
      }
  elif command -v colcon >/dev/null 2>&1; then
    colcon build --packages-select automap_pro >/tmp/v3_baseline_build.log 2>&1 \
      && pass "build success gate passed (colcon build --packages-select automap_pro)" \
      || {
        if command -v rg >/dev/null 2>&1; then
          rg -n "error:|failed|undefined reference" /tmp/v3_baseline_build.log || true
        else
          grep -nE "error:|failed|undefined reference" /tmp/v3_baseline_build.log || true
        fi
        fail "build success gate failed, see /tmp/v3_baseline_build.log"
      }
  else
    fail "Neither cmake nor colcon is available for build gate"
  fi
else
  warn "Build success gate skipped by --skip-build-check"
fi

echo ""
echo "0.1) Tests gate (ctest)"
if [[ "$SKIP_TEST_CHECK" -eq 0 ]]; then
  if command -v ctest >/dev/null 2>&1 && [[ -f "${BUILD_DIR}/CTestTestfile.cmake" ]]; then
    ctest --test-dir "$BUILD_DIR" --output-on-failure >/tmp/v3_baseline_test.log 2>&1 \
      && pass "ctest gate passed" \
      || {
        if command -v rg >/dev/null 2>&1; then
          rg -n "FAILED|Error|error" /tmp/v3_baseline_test.log || true
        else
          grep -nE "FAILED|Error|error" /tmp/v3_baseline_test.log || true
        fi
        fail "ctest gate failed, see /tmp/v3_baseline_test.log"
      }
  else
    warn "No CTest metadata found under ${BUILD_DIR}; test gate skipped"
  fi
else
  warn "Tests gate skipped by --skip-test-check"
fi

echo ""
echo "1) Config default strictness gates"

search "scancontext\.enabled\", false" "$CFG_H" \
  && pass "scancontext default disabled in config manager" \
  || fail "scancontext default is not disabled in config manager"

search "auto_enable_scancontext_on_ot_failure\", false" "$CFG_H" \
  && pass "auto-enable ScanContext fallback default disabled" \
  || fail "auto-enable ScanContext fallback default is not disabled"

search "allow_sc_fallback\", false" "$CFG_H" \
  && pass "allow_sc_fallback default disabled" \
  || fail "allow_sc_fallback default is not disabled"

search "allow_descriptor_fallback\", false" "$CFG_H" \
  && pass "allow_descriptor_fallback default disabled" \
  || fail "allow_descriptor_fallback default is not disabled"

search "allow_svd_geom_fallback\", false" "$CFG_H" \
  && pass "allow_svd_geom_fallback default disabled" \
  || fail "allow_svd_geom_fallback default is not disabled"

search "loop_closure\.flow_mode\", \"safe_degraded\"" "$CFG_H" \
  && pass "loop_flow_mode default is safe_degraded" \
  || fail "loop_flow_mode default is not safe_degraded"

echo ""
echo "2) Model path baseline gate"

search "models/overlapTransformer\.pt" "$CFG_CPP" \
  && pass "default OT model path points to overlapTransformer.pt" \
  || fail "default OT model path is not set to overlapTransformer.pt"

echo ""
echo "3) YAML strict policy gates"

search "auto_enable_scancontext_on_ot_failure:\s*false" "$CFG_YAML" \
  && pass "YAML disables auto ScanContext fallback" \
  || fail "YAML does not disable auto ScanContext fallback"

search "allow_sc_fallback:\s*false" "$CFG_YAML" \
  && pass "YAML disables ScanContext fallback" \
  || fail "YAML does not disable ScanContext fallback"

search "allow_descriptor_fallback:\s*false" "$CFG_YAML" \
  && pass "YAML disables descriptor fallback" \
  || fail "YAML does not disable descriptor fallback"

search "allow_svd_geom_fallback:\s*false" "$CFG_YAML" \
  && pass "YAML disables SVD geometry fallback" \
  || fail "YAML does not disable SVD geometry fallback"

search "flow_mode:\s*(strict|safe_degraded)" "$CFG_YAML" \
  && pass "YAML loop flow mode configured" \
  || fail "YAML loop flow mode missing"

echo ""
echo "4) Source-level strict and observability gates"

search "LOOP_FLOW\]\[STRICT\]" "$LOOP_CPP" \
  && pass "strict mode explicit runtime error logs exist" \
  || fail "strict mode runtime error logs missing"

search "descriptor_fallback_disabled" "$LOOP_CPP" \
  && pass "descriptor fallback rejection guard exists" \
  || fail "descriptor fallback rejection guard missing"

search "CONSTRAINT_KPI\]\[LOOP\]" "$LOOP_CPP" \
  && pass "loop KPI log exists" \
  || fail "loop KPI log missing"

search "accept_ratio_window|teaser_p95_ms|teaser_inflight_peak|desc_queue_drop|match_queue_drop" "$LOOP_CPP" \
  && pass "loop structured KPI fields exist" \
  || fail "loop structured KPI fields missing"

search "TEASER_PATH" "$TEASER_CPP" \
  && pass "TEASER path metadata log exists" \
  || fail "TEASER path metadata log missing"

echo ""
if [[ -n "$LOG_FILE" ]]; then
  echo "5) Runtime log evidence gates"
  [[ -f "$LOG_FILE" ]] || fail "Log file not found: $LOG_FILE"

  search "LOOP_FLOW" "$LOG_FILE" \
    && pass "runtime LOOP_FLOW evidence found" \
    || fail "runtime LOOP_FLOW evidence missing"

  search "TEASER_PATH" "$LOG_FILE" \
    && pass "runtime TEASER_PATH evidence found" \
    || fail "runtime TEASER_PATH evidence missing"

  search "CONSTRAINT_KPI\]\[LOOP\]" "$LOG_FILE" \
    && pass "runtime loop KPI evidence found" \
    || fail "runtime loop KPI evidence missing"

  if search "LOOP_FLOW\]\[STRICT\]|descriptor_fallback_disabled" "$LOG_FILE"; then
    pass "runtime strict/fallback guard evidence found"
  else
    warn "No strict guard event in this log (may be normal if pipeline healthy)"
  fi
else
  echo "5) Runtime log evidence gates"
  warn "No --log provided, runtime evidence checks skipped"
fi

echo ""
echo -e "${GREEN}All enabled V3 stability baseline checks passed.${NC}"
