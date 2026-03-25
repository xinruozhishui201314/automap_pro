#!/usr/bin/env bash
# =============================================================================
# 在「与 run_automap.sh 相同」的 automap-env:humble 镜像中执行命令，便于导出 ONNX 等。
#
# 挂载与 run_automap 构建/运行一致：
#   - automap_ws → /root/automap_ws
#   - automap_pro 源码 → /root/automap_ws/src/automap_pro（只读）
#   - 本仓库 scripts → /root/scripts（只读）
#   - data → /data（可读写，适合把 .pt / .onnx 放在 data/models 下）
#
# 用法
#   bash scripts/run_export_onnx_in_docker.sh [本脚本选项] -- <容器内命令...>
#
# 选项
#   --extra-mount HOST:CONTAINER:MODE   可重复；MODE 默认 ro（例如 ro|rw）
#   --workdir DIR                       容器内工作目录（默认 /root/automap_ws）
#   --no-gpu                            不加 --gpus all（CPU 导出时用）
#
# 示例 1：通用 TorchScript → ONNX（dummy 形状需与模型一致）
#   bash scripts/run_export_onnx_in_docker.sh -- \\
#     python3 /root/scripts/export_pt_to_onnx_generic.py \\
#       --input /data/models/mymodel.ts \\
#       --output /data/models/mymodel.onnx \\
#       --torchscript \\
#       --shapes 1,4,64,2048
#
# 示例 2：挂载宿主机上的 LSK3DNet 仓库，在容器内跑官方/自写导出脚本
#   bash scripts/run_export_onnx_in_docker.sh \\
#     --extra-mount \"$(pwd)/third_party/LSK3DNet:/workspace/LSK3DNet:ro\" -- \\
#     bash -lc 'cd /workspace/LSK3DNet && pip install -q -r requirements.txt && python3 tools/export_onnx.py'
#
# 注意：LSK3DNet 若含 spconv 稀疏卷积，torch.onnx.export 可能失败，需专用部署路径或 TorchScript。
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="${AUTOMAP_DOCKER_IMAGE:-automap-env:humble}"
WORKSPACE_DIR="${SCRIPT_DIR}/automap_ws"
PROJECT_DIR="${SCRIPT_DIR}/automap_pro"
DATA_DIR="${SCRIPT_DIR}/data"

TIME_VOLUMES="-v /etc/localtime:/etc/localtime:ro"
[[ -f /etc/timezone ]] && TIME_VOLUMES="${TIME_VOLUMES} -v /etc/timezone:/etc/timezone:ro"

EXTRA_MOUNTS=()
WORKDIR="/root/automap_ws"
USE_GPU=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --extra-mount)
      [[ $# -ge 2 ]] || { echo "Missing value for --extra-mount"; exit 1; }
      EXTRA_MOUNTS+=( -v "$2" )
      shift 2
      ;;
    --workdir)
      [[ $# -ge 2 ]] || { echo "Missing value for --workdir"; exit 1; }
      WORKDIR="$2"
      shift 2
      ;;
    --no-gpu)
      USE_GPU=0
      shift
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "Unknown option: $1 (use -- before the container command)" >&2
      exit 1
      ;;
  esac
done

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 [--extra-mount H:C:mode]... [--workdir DIR] [--no-gpu] -- <command...>" >&2
  exit 1
fi

mkdir -p "${DATA_DIR}" "${WORKSPACE_DIR}"

GPU_ARGS=( )
if [[ "${USE_GPU}" -eq 1 ]]; then
  GPU_ARGS=( --gpus all )
fi

# 与 run_automap 一致：可选加载 ROS/工作空间（导出一般不需要 install，但保留 source 无害）
INNER_PREFIX='set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f /root/automap_ws/install/setup.bash ]]; then source /root/automap_ws/install/setup.bash; fi
if [[ -f /root/automap_ws/install_deps/setup.bash ]]; then source /root/automap_ws/install_deps/setup.bash; fi
'

CMD_STR=
for arg in "$@"; do
  CMD_STR+=" $(printf '%q' "$arg")"
done

docker run --rm \
  ${TIME_VOLUMES} \
  "${GPU_ARGS[@]}" \
  --net=host \
  -e OMP_NUM_THREADS=1 \
  -e MKL_NUM_THREADS=1 \
  -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
  -v "${PROJECT_DIR}:/root/automap_ws/src/automap_pro:ro" \
  -v "${SCRIPT_DIR}/scripts:/root/scripts:ro" \
  -v "${DATA_DIR}:/data:rw" \
  "${EXTRA_MOUNTS[@]}" \
  -w "${WORKDIR}" \
  "${IMAGE_NAME}" \
  /bin/bash -lc "${INNER_PREFIX} exec${CMD_STR}"
