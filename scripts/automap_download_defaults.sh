#!/usr/bin/env bash
# 容器内下载默认值：国内镜像优先 + 缓存目录（由宿主机挂载 thrid_party/automap_cache）
# 由 build_inside_container.sh / setup_lsk3dnet_venv.sh source；勿单独依赖 bash -l 登录环境。
#
# 缓存布局（均在 AUTOMAP_DOWNLOAD_CACHE 下，默认 /root/automap_download_cache）:
#   pip/          pip wheel 缓存（PIP_CACHE_DIR）
#   libtorch/     LibTorch zip（sha256 命名，见 build_inside_container.sh）
#   git/          ONNX Runtime 等源码快照（免重复 git clone）
#   apt/archives  apt 下载的 .deb（Dir::Cache::archives）
#   （不把 Dir::State::lists 指到挂载目录：_apt 对宿主机挂载的 lists/partial 常无写权限，会导致 update 失败、pcl_ros 等装不上）
#   xdg/          XDG_CACHE_HOME，供通用工具复用缓存（可选）
# Docker 镜像层仍由 docker pull 本地保存，不在此目录。
#
# 环境变量（可选）:
#   AUTOMAP_DOWNLOAD_CACHE   缓存根目录，默认 /root/automap_download_cache
#   AUTOMAP_USE_OFFICIAL_PYPI=1  不使用国内 PyPI 镜像（仍可用 PIP_CACHE_DIR 缓存）
#   AUTOMAP_PIP_INDEX        覆盖默认 PyPI 镜像参数（默认阿里云 simple）
#   PIP_INDEX                若已非空则不再覆盖
#   LSK_PYTORCH_WHL_INDEX    覆盖 LSK 脚本中 torch 的 --index-url（默认上海交大 pytorch-wheels/{cu118|cu121|cu124}）
#   AUTOMAP_ONNXRUNTIME_GIT_URL  覆盖 ONNX Runtime git 地址（默认经 ghfast 加速 GitHub）
#   AUTOMAP_UBUNTU_MIRROR        apt 用 Ubuntu 仓库根 URL（默认阿里云；官方 archive.ubuntu.com 可自设）
#   AUTOMAP_LIBTORCH_DOWNLOAD_BASE  LibTorch zip 根 URL（与官方路径 libtorch/cu128/... 对齐；默认即官方 download.pytorch.org）
#   AUTOMAP_USE_OFFICIAL_LIBTORCH=1  解析 URL 时不做镜像替换（默认同官方时等价）
#   AUTOMAP_PYG_WHEEL_BASE     PyG -f 索引根（默认 data.pyg.org/whl；国内无通用镜像时保持官方）
# shellcheck shell=bash

CACHE_ROOT="${AUTOMAP_DOWNLOAD_CACHE:-/root/automap_download_cache}"
mkdir -p "${CACHE_ROOT}/pip" "${CACHE_ROOT}/libtorch" "${CACHE_ROOT}/git" \
  "${CACHE_ROOT}/apt/archives/partial" "${CACHE_ROOT}/xdg"

export PIP_CACHE_DIR="${PIP_CACHE_DIR:-${CACHE_ROOT}/pip}"
export XDG_CACHE_HOME="${XDG_CACHE_HOME:-${CACHE_ROOT}/xdg}"

# pip (>=23/24+) 会在 cache dir 不归当前用户所有时禁用缓存。
# 这里做 best-effort 修复：将挂载的 cache 目录 chown 为当前容器用户，避免出现：
#   "WARNING: The directory '.../pip' is not owned or is not writable ... The cache has been disabled."
# 说明：该目录通常是宿主机挂载，chown 会同步到宿主机（缓存目录可接受）。
if command -v id >/dev/null 2>&1; then
  _uid="$(id -u 2>/dev/null || true)"
  _gid="$(id -g 2>/dev/null || true)"
  if [ -n "${_uid}" ] && [ -n "${_gid}" ]; then
    mkdir -p "${PIP_CACHE_DIR}" "${XDG_CACHE_HOME}" 2>/dev/null || true
    chown -R "${_uid}:${_gid}" "${PIP_CACHE_DIR}" "${XDG_CACHE_HOME}" 2>/dev/null || true
    chmod -R u+rwX,go+rX "${PIP_CACHE_DIR}" "${XDG_CACHE_HOME}" 2>/dev/null || true
  fi
fi

# 长耗时步骤前调用，便于在 build.log / automap.log 中区分「进行中」与「卡住」（宿主 tee 会为每行加时间戳）
automap_log_progress() {
  echo "[PROGRESS] $*"
}

if [ "${AUTOMAP_USE_OFFICIAL_PYPI:-0}" = "1" ]; then
  :
elif [ -z "${PIP_INDEX:-}" ]; then
  export PIP_INDEX="${AUTOMAP_PIP_INDEX:--i https://mirrors.aliyun.com/pypi/simple/}"
fi

# LSK venv：torch/torchvision 的 wheel 索引（上海交大镜像，与官方 cu 版本对应）
export LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE="${LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE:-https://mirror.sjtu.edu.cn/pytorch-wheels}"

# LibTorch 预编译 zip（build_inside_container）：默认官方（清华 tuna pytorch-wheels/libtorch 无有效 cu* 目录，会 404）
export AUTOMAP_LIBTORCH_DOWNLOAD_BASE="${AUTOMAP_LIBTORCH_DOWNLOAD_BASE:-https://download.pytorch.org/libtorch}"
export AUTOMAP_USE_OFFICIAL_LIBTORCH="${AUTOMAP_USE_OFFICIAL_LIBTORCH:-0}"

# PyTorch Geometric 扩展 wheel 目录索引（torch-scatter 等；国内无稳定全量镜像时默认仍指向官方）
export AUTOMAP_PYG_WHEEL_BASE="${AUTOMAP_PYG_WHEEL_BASE:-https://data.pyg.org/whl}"

# ONNX Runtime 源码克隆（可被 AUTOMAP_ONNXRUNTIME_GIT_URL 完全覆盖）
export AUTOMAP_ONNXRUNTIME_GIT_URL="${AUTOMAP_ONNXRUNTIME_GIT_URL:-https://ghfast.top/https://github.com/microsoft/onnxruntime.git}"

# Isaac/NGC 等镜像常只带 NVIDIA 源，无完整 Ubuntu 索引，导致 libgeographiclib-dev / ros-* 等无安装候选。
# 仅写 universe 不够：依赖链涉及 main；需与官方 sources.list 对齐的 main restricted universe multiverse。
# 需在首次 apt-get update 之前调用。
automap_ensure_ubuntu_universe_for_apt() {
  if ! command -v apt-get >/dev/null 2>&1; then
    return 0
  fi
  if [ ! -f /etc/os-release ]; then
    return 0
  fi
  # shellcheck source=/dev/null
  . /etc/os-release
  if [ "${ID:-}" != "ubuntu" ] || [ -z "${VERSION_CODENAME:-}" ]; then
    return 0
  fi
  if [ ! -w /etc/apt/sources.list.d ] 2>/dev/null; then
    echo "[WARN] 无法写入 /etc/apt/sources.list.d，跳过 Ubuntu 源补充" >&2
    return 0
  fi
  local mirror="${AUTOMAP_UBUNTU_MIRROR:-http://mirrors.aliyun.com/ubuntu}"
  local mark="/etc/apt/sources.list.d/99-automap-ubuntu.list"
  # 与 Ubuntu 官方 desktop 一致的组件（阿里云等国内镜像同步 dists/<codename>/）
  local comps="main restricted universe multiverse"
  {
    echo "deb ${mirror} ${VERSION_CODENAME} ${comps}"
    echo "deb ${mirror} ${VERSION_CODENAME}-updates ${comps}"
    echo "deb ${mirror} ${VERSION_CODENAME}-security ${comps}"
    echo "deb ${mirror} ${VERSION_CODENAME}-backports ${comps}"
  } >"${mark}"
  echo "[INFO] 已写入 Ubuntu 完整组件源（${VERSION_CODENAME}，${comps}）→ ${mark}"
}

# Isaac ROS apt 元数据（release-4.3 noble）仅含 Components: main；旧版 NGC 镜像 sources 可能仍带 external-main，
# 会导致 “doesn't have the component 'external-main'”。见官方仓库 Release 与 Isaac Apt 文档。
# 须在 apt-get update 之前调用。
automap_fix_isaac_ros_apt_sources() {
  if ! command -v apt-get >/dev/null 2>&1; then
    return 0
  fi
  if [ ! -w /etc/apt ] 2>/dev/null; then
    return 0
  fi
  local f
  for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
    [ -f "$f" ] || continue
    if ! grep -qE 'isaac\.download\.nvidia\.(com|cn)/isaac-ros' "$f" 2>/dev/null; then
      continue
    fi
    if ! grep -q 'external-main' "$f" 2>/dev/null; then
      continue
    fi
    if [[ "$f" == *.sources ]]; then
      sed -i '/^Components:/s/[[:space:]]*external-main//g' "$f"
    else
      sed -i '/isaac\.download\.nvidia\.\(com\|cn\)\/isaac-ros/s/[[:space:]]*external-main//g' "$f"
    fi
    echo "[INFO] 已从 Isaac ROS apt 配置移除过时的 external-main 组件 → ${f}" >&2
  done
}

# 将 apt 的 .deb 写到挂载缓存，避免重复下载；lists 仍用容器内 /var/lib/apt/lists（避免 _apt 无法写挂载目录）。
# 需在首次 apt-get 之前（root、可写 /etc/apt/apt.conf.d）调用一次。
automap_configure_apt_cache() {
  automap_log_progress "apt：配置缓存目录、Ubuntu 源、修正 Isaac ROS 源（随后 apt-get update/install）"
  local root="${AUTOMAP_DOWNLOAD_CACHE:-/root/automap_download_cache}"
  mkdir -p "${root}/apt/archives/partial" "${root}/xdg" 2>/dev/null || true
  automap_ensure_ubuntu_universe_for_apt
  automap_fix_isaac_ros_apt_sources
  if ! command -v apt-get >/dev/null 2>&1; then
    return 0
  fi
  if [ ! -w /etc/apt/apt.conf.d ] 2>/dev/null; then
    echo "[WARN] 无法写入 /etc/apt/apt.conf.d，跳过 apt .deb 缓存重定向" >&2
    return 0
  fi
  local cf="/etc/apt/apt.conf.d/99automap-cache"
  echo "Dir::Cache::archives \"${root}/apt/archives\";" >"${cf}"
  # 宿主机挂载的目录常为 root:root，apt 获取 .deb 时若以 _apt 写入 partial 会失败
  if id _apt &>/dev/null; then
    chown -R _apt:root "${root}/apt/archives" 2>/dev/null || true
    chmod -R u+rwX,g+rX "${root}/apt/archives" 2>/dev/null || true
  fi
  echo "[INFO] apt .deb 缓存: ${root}/apt/archives（lists 使用容器默认路径）"
}
