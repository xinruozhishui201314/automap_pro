#!/usr/bin/env bash
# 自动化测试：验证 automap_pro 编译通过（含 LivoBridge atomic 等修复）
# 用法: bash test_build_automap.sh
# 成功: exit 0；失败: exit 1
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
echo "[test_build_automap] Running: bash automap_start.sh --build"
if bash automap_start.sh --build; then
  echo "[test_build_automap] PASS: build succeeded"
  exit 0
else
  echo "[test_build_automap] FAIL: build failed"
  exit 1
fi
