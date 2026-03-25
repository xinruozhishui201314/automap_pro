#!/bin/bash
# 在 Docker 容器内执行 OverlapTransformer .pth.tar → .pt 转换（宿主机无需 PyTorch）
# 与 run_automap.sh 使用同一 Docker 镜像（默认 NGC Isaac），挂载仓库后运行转换脚本，
# 生成的 overlapTransformer.pt 会写回宿主机 automap_pro/models/。
# 需在容器内安装 PyTorch 时使用国内 pip 镜像，可通过 PIP_INDEX 覆盖（默认清华）。
# pip 缓存默认 thrid_party/automap_cache/pip（与 run_automap 一致），首次下载后复用本地缓存。
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/automap_docker_defaults.sh
source "${SCRIPT_DIR}/scripts/automap_docker_defaults.sh"
IMAGE_NAME="${AUTOMAP_DOCKER_IMAGE}"
# 国内 pip 镜像（仅当容器内需安装 torch 时使用）：清华 | 阿里云 | 豆瓣 等，可设 PIP_INDEX 覆盖
PIP_INDEX="${PIP_INDEX:--i https://mirrors.aliyun.com/pypi/simple/}"
# pip 缓存：与 run_automap 共用 thrid_party/automap_cache/pip（可设 PIP_CACHE_DIR 覆盖宿主机目录）
AUTOMAP_CACHE_ROOT="${SCRIPT_DIR}/thrid_party/automap_cache"
mkdir -p "${AUTOMAP_CACHE_ROOT}/pip"
PIP_CACHE_HOST="${PIP_CACHE_DIR:-${AUTOMAP_CACHE_ROOT}/pip}"
PKG_DIR="$SCRIPT_DIR/automap_pro"
PTH_FILE="${1:-$PKG_DIR/models/pretrained_overlap_transformer.pth.tar}"
OUT_FILE="${2:-$PKG_DIR/models/overlapTransformer.pt}"

# 容器内路径（仓库根挂载到 /workspace/automap_pro；仅支持输入/输出在仓库内的路径）
MOUNT_ROOT="/workspace/automap_pro"
case "$PTH_FILE" in
  "$SCRIPT_DIR"/*) PTH_CONTAINER="$MOUNT_ROOT/${PTH_FILE#$SCRIPT_DIR/}" ;;
  *) echo "Error: input path must be under repo: $SCRIPT_DIR" >&2; exit 1 ;;
esac
case "$OUT_FILE" in
  "$SCRIPT_DIR"/*) OUT_CONTAINER="$MOUNT_ROOT/${OUT_FILE#$SCRIPT_DIR/}" ;;
  *) echo "Error: output path must be under repo: $SCRIPT_DIR" >&2; exit 1 ;;
esac

if [ ! -f "$PTH_FILE" ]; then
  echo "Error: .pth.tar not found: $PTH_FILE" >&2
  echo "Usage: $0 [path/to/pretrained_overlap_transformer.pth.tar] [path/to/overlapTransformer.pt]" >&2
  echo "Default: $PKG_DIR/models/pretrained_overlap_transformer.pth.tar -> $PKG_DIR/models/overlapTransformer.pt" >&2
  exit 1
fi

if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
  if [[ "$IMAGE_NAME" == nvcr.io/* ]] || [[ "$IMAGE_NAME" == ghcr.io/* ]]; then
    echo "Pulling $IMAGE_NAME ..."
    docker pull "$IMAGE_NAME"
  else
    echo "Error: Docker image not found: $IMAGE_NAME" >&2
    echo "Load or build it first (e.g. docker load -i docker/automap-env_humble.tar)." >&2
    exit 2
  fi
fi

mkdir -p "$(dirname "$OUT_FILE")"
mkdir -p "$PIP_CACHE_HOST"
echo "Converting in container (image: $IMAGE_NAME)..."
echo "  input:  $PTH_FILE"
echo "  output: $OUT_FILE"
echo "  pip cache: $PIP_CACHE_HOST (reused on next run; shared with run_automap automap_cache/pip)"
# 若镜像内无 PyTorch（如从旧 tar 加载），用国内镜像 pip 安装再执行转换
# 禁用容器内可能损坏的 pip 配置（PIP_CONFIG_FILE=/dev/null），避免 "Source contains parsing errors: pip.conf"
docker run --rm \
  -v "$SCRIPT_DIR:$MOUNT_ROOT" \
  -v "$PIP_CACHE_HOST:/workspace/pip_cache" \
  -e "PIP_INDEX=$PIP_INDEX" \
  -e "PIP_CONFIG_FILE=/dev/null" \
  -e "PIP_CACHE_DIR=/workspace/pip_cache" \
  "$IMAGE_NAME" \
  bash -c 'if ! python3 -c "import torch" 2>/dev/null; then echo "PyTorch not in image, installing with pip (using domestic index, cache: /workspace/pip_cache)..."; (mv /root/.pip/pip.conf /root/.pip/pip.conf.bak 2>/dev/null || true); pip3 install --user --cache-dir /workspace/pip_cache torch pyyaml '"$PIP_INDEX"' 2>/dev/null || pip3 install --cache-dir /workspace/pip_cache torch pyyaml '"$PIP_INDEX"'; fi && python3 '"$MOUNT_ROOT"'/automap_pro/scripts/convert_ot_pth_to_torchscript.py '"$PTH_CONTAINER"' '"$OUT_CONTAINER"''

echo "Done. TorchScript model: $OUT_FILE"
