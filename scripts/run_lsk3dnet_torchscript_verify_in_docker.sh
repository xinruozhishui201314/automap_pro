#!/usr/bin/env bash
# 在与 run_automap.sh / run_export_onnx_in_docker.sh 相同的 automap-env:humble 镜像中
# 运行 LSK3DNet-main/scripts/verify_torchscript_libtorch.py（步骤 1→2），并提示步骤 3（C++）。
#
# 依赖说明（必读）:
#   - LSK3DNet 需要 spconv、torch_scatter 等与当前 PyTorch/CUDA 匹配的包。
#   - 仓库内 requirements.txt 固定为 torch1.11+cu113，与 automap 镜像中的 torch2.x+cu118 冲突，
#     因此本脚本不会执行 pip install -r requirements.txt。
#   - 在镜像内请先自行安装与 torch 版本一致的 spconv（例如查阅 spconv 官方 wheel），
#     或使用与 LSK 训练完全一致的 Conda 环境，再挂载该环境的 python 来运行验证脚本。
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

REPO="$(cd "$(dirname "${BASH_SOURCE[0]})/.." && pwd)"
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

CMD_STR="cd /workspace/LSK3DNet && pip install -q easydict 2>/dev/null || true && python3 scripts/verify_torchscript_libtorch.py"
for arg in "$@"; do
  CMD_STR+=" $(printf '%q' "$arg")"
done

exec bash "$REPO/scripts/run_export_onnx_in_docker.sh" \
  --extra-mount "${LSK_HOST}:/workspace/LSK3DNet:rw" \
  --workdir /workspace/LSK3DNet \
  -- \
  bash -lc "$CMD_STR"
