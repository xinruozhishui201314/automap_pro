#!/usr/bin/env bash
# 使用预编译 LibTorch（与当前 PyTorch 主版本一致、cxx11 ABI）编译 cpp_jit_smoke，并对给定 .pt 执行 load。
#
#   export LIBTORCH_HOME=/path/to/libtorch
#   bash scripts/build_cpp_jit_smoke.sh /tmp/lsk3dnet_traced.pt
#   bash scripts/build_cpp_jit_smoke.sh /tmp/lsk3dnet_traced.pt --forward
#
# 若未设置 LIBTORCH_HOME，会依次尝试：
#   仓库根/automap_ws/install_deps/libtorch
#   $HOME/libtorch
#   /opt/libtorch

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LSK_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$LSK_ROOT/../../.." && pwd)"

PT="${1:-}"
shift || true
if [[ -z "$PT" ]]; then
  echo "usage: $0 <model.pt> [--forward ...]" >&2
  exit 1
fi

LIBTORCH_HOME="${LIBTORCH_HOME:-}"
if [[ -z "$LIBTORCH_HOME" ]]; then
  for cand in \
    "${REPO_ROOT}/automap_ws/install_deps/libtorch" \
    "${REPO_ROOT}/install_deps/libtorch" \
    "${HOME}/libtorch" \
    "/opt/libtorch"; do
    if [[ -d "$cand" && -d "$cand/lib" ]]; then
      LIBTORCH_HOME="$cand"
      break
    fi
  done
fi
if [[ -z "${LIBTORCH_HOME}" || ! -d "${LIBTORCH_HOME}/lib" ]]; then
  echo "Set LIBTORCH_HOME to LibTorch root (contains include/ and lib/). Tried REPO=${REPO_ROOT}" >&2
  exit 3
fi

OUT="${SCRIPT_DIR}/cpp_jit_smoke"
CXXFLAGS=( -std=c++17 -O2 -D_GLIBCXX_USE_CXX11_ABI=1 )
INCLUDES=( -I"${LIBTORCH_HOME}/include" -I"${LIBTORCH_HOME}/include/torch/csrc/api/include" )
LIBS=( -L"${LIBTORCH_HOME}/lib" -Wl,-rpath,"${LIBTORCH_HOME}/lib" -ltorch -ltorch_cpu -lc10 )

if [[ -f "${LIBTORCH_HOME}/lib/libtorch_cuda.so" ]]; then
  LIBS+=( -ltorch_cuda -lc10_cuda )
fi

echo "[build] LIBTORCH_HOME=${LIBTORCH_HOME}"
g++ "${CXXFLAGS[@]}" "${INCLUDES[@]}" "${SCRIPT_DIR}/cpp_jit_smoke.cpp" "${LIBS[@]}" -o "${OUT}"
echo "[build] -> ${OUT}"
exec "${OUT}" "${PT}" "$@"
