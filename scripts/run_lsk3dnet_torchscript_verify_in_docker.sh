#!/usr/bin/env bash
# 在与 run_automap.sh / run_export_onnx_in_docker.sh 相同的 Docker 镜像中（默认 NGC Isaac / Jazzy）
# 运行 LSK3DNet-main/scripts/verify_torchscript_libtorch.py（步骤 1→2），并提示步骤 3（C++）。
#
# 依赖（推荐）:
#   - 编译：`bash run_automap.sh --build-only` 默认会创建 install_deps/lsk3dnet_venv；其它编译路径可设 AUTOMAP_SETUP_LSK3DNET_VENV=1（见 setup_lsk3dnet_venv.sh）
#   - 宿主机: export AUTOMAP_LSK3DNET_PYTHON=/path/to/venv/bin/python3
#     或在容器内该 venv 已存在时，run_automap 会自动默认 AUTOMAP_LSK3DNET_PYTHON
#   - 勿使用 LSK3DNet 根目录 requirements.txt（torch1.11+cu113 与 cu12x 栈冲突）
#
# 默认挂载：
#   automap_pro/thrid_party/LSK3DNet-main → /workspace/LSK3DNet（rw，便于写报告与 .pt 到 /data）
#
# 用法
#   bash scripts/run_lsk3dnet_torchscript_verify_in_docker.sh -- [传给 Python 的参数...]
#
# 无额外参数时默认：
#   --config config/lk-semantickitti_erk_finetune.yaml --skip-checkpoint
#   --output /data/lsk3dnet_traced.pt --report /data/lsk3dnet_torchscript_report.json
#
# 示例（带权重，需权重在挂载路径内或 /data）：
#   bash scripts/run_lsk3dnet_torchscript_verify_in_docker.sh -- \\
#     --config config/lk-semantickitti_erk_finetune.yaml \\
#     --checkpoint /data/models/your.pt \\
#     --output /data/lsk3dnet_traced.pt --report /data/lsk3dnet_torchscript_report.json

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LSK_HOST="${LSK_HOST:-$REPO/automap_pro/thrid_party/LSK3DNet-main}"

if [[ "${1:-}" == "--" ]]; then
  shift
fi

if [[ $# -eq 0 ]]; then
  set -- \
    --config config/lk-semantickitti_erk_finetune.yaml \
    --skip-checkpoint \
    --output /data/lsk3dnet_traced.pt \
    --report /data/lsk3dnet_torchscript_report.json
fi

CMD_STR='cd /workspace/LSK3DNet && PY="${AUTOMAP_LSK3DNET_PYTHON:-python3}" && if ! "$PY" -c "import spconv" 2>/dev/null; then pip install -q easydict 2>/dev/null || true; fi && exec "$PY" scripts/verify_torchscript_libtorch.py'
for arg in "$@"; do
  CMD_STR+=" $(printf '%q' "$arg")"
done

exec bash "$REPO/scripts/run_export_onnx_in_docker.sh" \
  --extra-mount "${LSK_HOST}:/workspace/LSK3DNet:rw" \
  --workdir /workspace/LSK3DNet \
  -- \
  bash -lc "$CMD_STR"
