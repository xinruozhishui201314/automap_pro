#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

RUNTIME_LOG="${1:-${V3_RUNTIME_LOG:-}}"
if [[ -z "${RUNTIME_LOG}" ]]; then
  echo "[CI_GATE][FAIL] runtime log path is required (arg1 or V3_RUNTIME_LOG)" >&2
  exit 1
fi

"${SCRIPT_DIR}/check_v3_stability_baseline.sh" --log "${RUNTIME_LOG}"
echo "[CI_GATE] V3 gate passed: build + tests + baseline + runtime_log"
