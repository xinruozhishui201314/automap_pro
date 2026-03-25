#!/usr/bin/env bash
# =============================================================================
# 为 LSK3DNet hybrid / verify 脚本创建独立 venv：PyTorch + spconv + torch-scatter（与容器 CUDA 对齐）
#
# 官方 requirements.txt 固定 torch1.11+cu113，与 automap 默认 LibTorch 2.5/cu12x 不一致；
# 本脚本按 nvcc（或 LSK_CUDA_TAG）选择 cu118/cu121/cu124 预编译轮，供 lsk3dnet_hybrid_worker.py 使用。
#
# 用法（容器内，install_deps 可写）:
#   bash run_automap.sh --build-only   # 默认已启用 LSK venv；关闭: AUTOMAP_SETUP_LSK3DNET_VENV=0 …（由 build_inside_container 调用）
# 或手动:
#   bash scripts/setup_lsk3dnet_venv.sh /root/automap_ws/install_deps/lsk3dnet_venv
#
# 环境变量:
#   LSK_CUDA_TAG       强制 cu118 | cu121 | cu124（不设则根据 nvcc 推断）
#   LSK_TORCH_VER      默认 2.5.1（须与 PyG torch-scatter 轮匹配）
#   LSK_VENV_BASE_PYTHON  创建 venv 用的 python，默认 python3
#   PIP_INDEX          例如 -i https://mirrors.aliyun.com/pypi/simple/（不设则随 automap_download_defaults，默认阿里云）
#   LSK_PYTORCH_WHL_INDEX  覆盖 torch 的 pip --index-url（默认上海交大 mirror…/pytorch-wheels/{cu}）
#   LSK_SKIP_IF_OK=1   若 venv 已存在且含 .lsk3dnet_venv_ok 则跳过
# =============================================================================
set -euo pipefail

_SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${_SETUP_DIR}/automap_download_defaults.sh" ]]; then
  # shellcheck source=scripts/automap_download_defaults.sh
  source "${_SETUP_DIR}/automap_download_defaults.sh"
fi
if ! type automap_log_progress &>/dev/null; then
  automap_log_progress() { echo "[PROGRESS] $*"; }
fi

VENV_DIR="${1:-${AUTOMAP_LSK3DNET_VENV:-}}"
if [[ -z "${VENV_DIR}" ]]; then
  echo "Usage: $0 <venv_dir>   or   AUTOMAP_LSK3DNET_VENV=/path $0" >&2
  exit 2
fi

MARKER="${VENV_DIR}/.lsk3dnet_venv_ok"
if [[ "${LSK_SKIP_IF_OK:-0}" = "1" ]] && [[ -f "${MARKER}" ]] && [[ -x "${VENV_DIR}/bin/python3" ]]; then
  echo "[INFO] LSK3DNet venv 已就绪（${MARKER}），跳过 setup"
  exit 0
fi

BASE_PY="${LSK_VENV_BASE_PYTHON:-python3}"
if ! command -v "${BASE_PY}" >/dev/null 2>&1; then
  echo "[ERROR] 未找到解释器: ${BASE_PY}" >&2
  exit 1
fi

resolve_cuda_tag() {
  if [[ -n "${LSK_CUDA_TAG:-}" ]]; then
    echo "${LSK_CUDA_TAG}"
    return
  fi
  local nvcc_bin=""
  if command -v nvcc >/dev/null 2>&1; then
    nvcc_bin=$(command -v nvcc)
  elif [[ -n "${CUDA_HOME:-}" ]] && [[ -x "${CUDA_HOME}/bin/nvcc" ]]; then
    nvcc_bin="${CUDA_HOME}/bin/nvcc"
  else
    for d in /usr/local/cuda /usr/local/cuda-12 /usr/local/cuda-11; do
      if [[ -x "${d}/bin/nvcc" ]]; then nvcc_bin="${d}/bin/nvcc"; break; fi
    done
  fi
  local ver=""
  if [[ -n "${nvcc_bin}" ]]; then
    ver=$("${nvcc_bin}" --version 2>/dev/null | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p' | head -n1)
  fi
  case "${ver}" in
    11.*) echo "cu118" ;;
    12.0|12.1|12.2|12.3) echo "cu121" ;;
    *) echo "cu124" ;;
  esac
}

CUDA_TAG="$(resolve_cuda_tag)"
TORCH_VER="${LSK_TORCH_VER:-2.5.1}"
if [[ -n "${LSK_PYTORCH_WHL_INDEX:-}" ]]; then
  PYTORCH_WHL_INDEX="${LSK_PYTORCH_WHL_INDEX}"
else
  _PT_BASE="${LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE:-https://mirror.sjtu.edu.cn/pytorch-wheels}"
  PYTORCH_WHL_INDEX="${_PT_BASE}/${CUDA_TAG}"
fi
SPCONV_PKG="spconv-${CUDA_TAG}"
_PYG_BASE="${AUTOMAP_PYG_WHEEL_BASE:-https://data.pyg.org/whl}"
PYG_INDEX="${_PYG_BASE}/torch-${TORCH_VER}+${CUDA_TAG}.html"

echo "[INFO] LSK3DNet venv → ${VENV_DIR}"
echo "[INFO] 使用 CUDA 标签: ${CUDA_TAG}（torch==${TORCH_VER}，${SPCONV_PKG}，PyG: ${PYG_INDEX}）"

if [[ ! -d "${VENV_DIR}" ]]; then
  "${BASE_PY}" -m venv "${VENV_DIR}"
fi
# shellcheck source=/dev/null
source "${VENV_DIR}/bin/activate"

automap_log_progress "pip：升级 pip/setuptools/wheel…"
pip install -U pip wheel setuptools ${PIP_INDEX:-}

automap_log_progress "pip：安装 PyTorch ${TORCH_VER}（可能数分钟）…"
echo "[INFO] 安装 PyTorch (${PYTORCH_WHL_INDEX})..."
pip install "torch==${TORCH_VER}" torchvision ${PIP_INDEX:-} --index-url "${PYTORCH_WHL_INDEX}"

automap_log_progress "pip：安装 ${SPCONV_PKG}…"
echo "[INFO] 安装 ${SPCONV_PKG}..."
pip install "${SPCONV_PKG}" ${PIP_INDEX:-}

automap_log_progress "pip：安装 torch-scatter（PyG）…"
echo "[INFO] 安装 torch-scatter（PyG 扩展索引）..."
if ! pip install "torch-scatter" -f "${PYG_INDEX}" ${PIP_INDEX:-}; then
  _MAJMIN="${TORCH_VER%.*}"
  echo "[WARN] torch-scatter 首选索引失败，尝试 torch-${_MAJMIN}+${CUDA_TAG}..."
  ALT_INDEX="${_PYG_BASE}/torch-${_MAJMIN}+${CUDA_TAG}.html"
  pip install "torch-scatter" -f "${ALT_INDEX}" ${PIP_INDEX:-} || {
    echo "[ERROR] torch-scatter 安装失败；请检查 PyG 是否提供 torch-${TORCH_VER}+${CUDA_TAG} 的 wheel，或调整 LSK_TORCH_VER" >&2
    exit 1
  }
fi

automap_log_progress "pip：安装 easydict / PyYAML / numpy…"
echo "[INFO] 安装 LSK worker 轻量依赖..."
pip install easydict PyYAML numpy ${PIP_INDEX:-}

automap_log_progress "pip：安装 pybind11/cmake（用于编译 c_gen_normal_map）…"
echo "[INFO] 安装 pybind11/cmake..."
pip install pybind11 cmake ${PIP_INDEX:-}

automap_log_progress "编译 LSK c_utils: c_gen_normal_map（range 法线，训练一致）…"
LSK_REPO_ROOT="${LSK_REPO_ROOT:-/root/automap_ws/src/automap_pro/thrid_party/LSK3DNet-main}"
if [[ -d "${LSK_REPO_ROOT}/c_utils" ]]; then
  if python3 -c "import c_gen_normal_map" >/dev/null 2>&1; then
    echo "[INFO] c_gen_normal_map 已可 import，跳过编译"
  else
    _PYBIND_CMAKE_DIR="$("${VENV_DIR}/bin/python3" -m pybind11 --cmakedir 2>/dev/null || true)"
    if [[ -z "${_PYBIND_CMAKE_DIR}" ]]; then
      echo "[ERROR] 无法获取 pybind11 cmake dir（python -m pybind11 --cmakedir 失败）" >&2
      exit 1
    fi
    # Out-of-source build: LSK repo is often mounted read-only (e.g. /root/mapping or /root/automap_ws/src/automap_pro),
    # so never write build artifacts under ${LSK_REPO_ROOT}.
    _LSK_CUTILS_SRC="${LSK_REPO_ROOT}/c_utils"
    _LSK_CUTILS_BUILD="${VENV_DIR}/.build_lsk3dnet_c_utils"
    rm -rf "${_LSK_CUTILS_BUILD}" 2>/dev/null || true
    mkdir -p "${_LSK_CUTILS_BUILD}"
    cmake -S "${_LSK_CUTILS_SRC}" -B "${_LSK_CUTILS_BUILD}" -Dpybind11_DIR="${_PYBIND_CMAKE_DIR}" -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    cmake --build "${_LSK_CUTILS_BUILD}" -j"$(nproc)"
    _SO="$(ls -1 "${_LSK_CUTILS_BUILD}"/c_gen_normal_map*.so 2>/dev/null | head -n1 || true)"
    if [[ -z "${_SO}" ]]; then
      echo "[ERROR] 未生成 c_gen_normal_map*.so（编译失败或输出路径变化）" >&2
      exit 1
    fi
    _SITE="$("${VENV_DIR}/bin/python3" -c "import site; print(site.getsitepackages()[0])")"
    cp -f "${_SO}" "${_SITE}/"
    python3 -c "import c_gen_normal_map; print('[OK] import c_gen_normal_map')"
  fi
else
  echo "[WARN] 未找到 LSK3DNet c_utils 目录: ${LSK_REPO_ROOT}/c_utils（将无法使用 normal_mode=range）" >&2
fi

automap_log_progress "校验 torch / spconv / torch_scatter import…"
echo "[INFO] 校验 import（CUDA 可用性取决于镜像与 --gpus）..."
python3 - <<'PY'
import torch
import spconv.pytorch  # noqa: F401
import torch_scatter  # noqa: F401
print("[OK] torch", torch.__version__, "cuda_available=", torch.cuda.is_available())
PY

touch "${MARKER}"
echo "[INFO] LSK3DNet venv 完成: ${VENV_DIR}/bin/python3"
echo "[INFO] 请在 semantic.lsk3dnet.python 中配置该路径，或设置环境变量 AUTOMAP_LSK3DNET_PYTHON（运行 run_automap 时若 venv 存在会自动默认）"
