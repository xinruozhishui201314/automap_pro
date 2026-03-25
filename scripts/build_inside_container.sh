#!/bin/bash
set -e

# automap_download_defaults 未挂载时的兜底（与其中 automap_log_progress 行为一致）
automap_log_progress() { echo "[PROGRESS] $*"; }
# 误写或旧片段若调用 _log_progress，仍可用（须委托给 automap_log_progress，见下方 source 后再绑一次）
_log_progress() { automap_log_progress "$@"; }

ROS_DISTRO_NAME="${AUTOMAP_ROS_DISTRO:-${ROS_DISTRO:-humble}}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [ ! -f "${ROS_SETUP}" ]; then
  echo "[ERROR] 未找到 ${ROS_SETUP}；请设置 AUTOMAP_ROS_DISTRO（Isaac 镜像一般为 jazzy）。" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${ROS_SETUP}"
cd /root/automap_ws

# CMake (>=3.18) initializes CMAKE_CUDA_ARCHITECTURES from env `CUDAARCHS`.
# If `CUDAARCHS` is defined but empty, CMake errors out ("must be non-empty if set").
# Hardening: clear empty vars to keep CPU-only builds working in generic containers.
if [ "${CUDAARCHS+x}" = "x" ] && [ -z "${CUDAARCHS}" ]; then
  unset CUDAARCHS
fi
if [ "${CMAKE_CUDA_ARCHITECTURES+x}" = "x" ] && [ -z "${CMAKE_CUDA_ARCHITECTURES}" ]; then
  unset CMAKE_CUDA_ARCHITECTURES
fi

# 须在首次 apt-get 之前：pip/LibTorch/git/apt 缓存目录（宿主机 thrid_party/automap_cache → /root/automap_download_cache）
if [ -f /root/scripts/automap_download_defaults.sh ]; then
  # shellcheck source=scripts/automap_download_defaults.sh
  source /root/scripts/automap_download_defaults.sh
  automap_configure_apt_cache
  echo "[INFO] 下载缓存: AUTOMAP_DOWNLOAD_CACHE=${AUTOMAP_DOWNLOAD_CACHE:-/root/automap_download_cache} PIP_CACHE_DIR=${PIP_CACHE_DIR} XDG_CACHE_HOME=${XDG_CACHE_HOME}"
fi
# source 会覆盖 automap_log_progress；_log_progress 须始终委托到当前实现
_log_progress() { automap_log_progress "$@"; }

# 与 build.ros.org 发布的 ros-jazzy-cv-bridge 官方二进制 deb 版本一致（apt show / dpkg -l 中的版本串）。
# 覆盖示例: export AUTOMAP_CV_BRIDGE_OFFICIAL_DEB_VERSION=…
# 直接指定 .deb URL: export AUTOMAP_CV_BRIDGE_OFFICIAL_DEB_URL=https://...
# Pool 基址（国内可换镜像）: export AUTOMAP_ROS2_POOL_BASE=http://packages.ros.org/ros2/ubuntu/pool/main
# pool 文件名后缀：x86_64 PC / 多数 Docker 为 amd64（ros-*_*_amd64.deb）；默认 dpkg --print-architecture，可 export AUTOMAP_CV_BRIDGE_DEB_ARCH=amd64 强制
: "${AUTOMAP_ROS2_POOL_BASE:=http://packages.ros.org/ros2/ubuntu/pool/main}"

# 是否已在 apt 中声明 packages.ros.org 的 ros2 源（含 deb822 .sources 与 one-line .list）
_automap_ros2_repo_declared() {
  local _f
  if [ -f /etc/apt/sources.list ] && grep -qE 'https?://[^[:space:]#]*packages\.ros\.org/[^[:space:]#]*ros2' /etc/apt/sources.list 2>/dev/null; then
    return 0
  fi
  shopt -s nullglob
  for _f in /etc/apt/sources.list.d/*; do
    [ -f "${_f}" ] || continue
    # deb822 可能多行；整文件压成一行再匹配 URIs
    if tr -s '\n\r\t' ' ' < "${_f}" | grep -qE 'https?://[^[:space:]#]*packages\.ros\.org/[^[:space:]#]*ros2'; then
      shopt -u nullglob
      return 0
    fi
  done
  shopt -u nullglob
  return 1
}

# 官方 ROS2 deb / colcon 安装仅保证 cv_bridge.hpp 树；仓库内多处 #include <cv_bridge/cv_bridge.h>。在 <prefix>/include/cv_bridge/cv_bridge.h 写入转调官方 hpp 的兼容层。
# 参数: $1=distro（仅用于日志，可 jazzy）; $2=安装前缀，默认 /opt/ros/<distro>（与官方 deb 布局一致）
automap_write_cv_bridge_ros1_shim() {
  local _d="${1:-jazzy}"
  local _prefix="${2:-/opt/ros/${_d}}"
  local _inc="${_prefix}/include/cv_bridge"
  local _hpp="${_inc}/cv_bridge/cv_bridge.hpp"
  if [ ! -f "${_hpp}" ]; then
    return 1
  fi
  if [ -f "${_inc}/cv_bridge.h" ]; then
    return 0
  fi
  mkdir -p "${_inc}"
  cat > "${_inc}/cv_bridge.h" << 'EOF'
/* Compat: official ros-jazzy-cv-bridge installs cv_bridge/cv_bridge/cv_bridge.hpp (pool deb); this maps ROS1-style include. */
#ifndef CV_BRIDGE_CV_BRIDGE_H_COMPAT_
#define CV_BRIDGE_CV_BRIDGE_H_COMPAT_
#include <cv_bridge/cv_bridge/cv_bridge.hpp>
#endif
EOF
  echo "[INFO] 已写入 ${_inc}/cv_bridge.h（转调 ${_hpp}）"
  return 0
}

# apt 后若头文件不在默认路径，尝试在 /opt/ros/<distro> 下定位官方 cv_bridge.hpp 并写兼容层
automap_refresh_cv_bridge_shim_after_apt() {
  local _d="${1:-jazzy}"
  local _pfx="/opt/ros/${_d}"
  local _std="${_pfx}/include/cv_bridge/cv_bridge/cv_bridge.hpp"
  if [ -f "${_std}" ]; then
    automap_write_cv_bridge_ros1_shim "${_d}" "${_pfx}" || true
    return 0
  fi
  local _any
  _any="$(find "${_pfx}" -type f -path '*/cv_bridge/cv_bridge/cv_bridge.hpp' 2>/dev/null | head -1)"
  if [ -n "${_any}" ] && [ -f "${_any}" ]; then
    echo "[INFO] 在非常规路径发现 cv_bridge.hpp: ${_any}，仍向 ${_pfx} 写兼容头（依赖标准 include 布局）"
    automap_write_cv_bridge_ros1_shim "${_d}" "${_pfx}" || true
  fi
  return 0
}

# 从 packages.ros.org pool 下载指定版本的 ros-*-cv-bridge .deb，合并到 /opt/ros/<distro>（与 apt 安装的官方文件一致）
automap_merge_cv_bridge_from_official_pool_deb() {
  local _distro="${ROS_DISTRO_NAME:-jazzy}"
  local _deb_ver="${AUTOMAP_CV_BRIDGE_OFFICIAL_DEB_VERSION}"
  if [ "${_distro}" = "jazzy" ] && [ -z "${_deb_ver}" ]; then
    _deb_ver="4.1.0-1noble.20260126.175659"
  fi
  if [ -z "${_deb_ver}" ]; then
    echo "[INFO] 非 jazzy 或未配置 AUTOMAP_CV_BRIDGE_OFFICIAL_DEB_VERSION，跳过 pool .deb 合并。"
    return 1
  fi
  local _arch="${AUTOMAP_CV_BRIDGE_DEB_ARCH:-$(dpkg --print-architecture)}"
  local _pkg="ros-${_distro}-cv-bridge"
  local _deb="${_pkg}_${_deb_ver}_${_arch}.deb"
  local _url="${AUTOMAP_CV_BRIDGE_OFFICIAL_DEB_URL}"
  if [ -z "${_url}" ]; then
    _url="${AUTOMAP_ROS2_POOL_BASE}/r/${_pkg}/${_deb}"
  fi
  local _work
  _work="$(mktemp -d)"
  echo "[INFO] cv_bridge: 从 ROS buildfarm pool 拉取官方 .deb（与 apt 包内容一致）→ ${_deb}"
  automap_log_progress "下载官方 pool deb ${_deb} 并合并至 /opt/ros/${_distro} …"
  if ! curl -fsSL "${_url}" -o "${_work}/${_deb}"; then
    echo "[WARN] 无法下载 ${_url}（当前 deb 架构后缀=${_arch}，常见 PC 为 amd64）；将尝试其它兜底。" >&2
    rm -rf "${_work}"
    return 1
  fi
  dpkg-deb -x "${_work}/${_deb}" "${_work}/extract" || {
    rm -rf "${_work}"
    return 1
  }
  local _root="${_work}/extract/opt/ros/${_distro}"
  if [ ! -d "${_root}" ]; then
    echo "[ERROR] .deb 内未找到 opt/ros/${_distro}" >&2
    rm -rf "${_work}"
    return 1
  fi
  mkdir -p "/opt/ros/${_distro}"
  cp -a "${_root}/." "/opt/ros/${_distro}/"
  rm -rf "${_work}"
  automap_write_cv_bridge_ros1_shim "${_distro}" || true
  return 0
}

# 从源码构建 cv_bridge 到 install_deps 下 overlay（apt / 官方 pool 均失败时的最后兜底）
# 注意：GitHub ros-perception/vision_opencv 无 jazzy 分支，Jazzy 应对应 rolling（或 iron/humble 回退）
automap_build_cv_bridge_overlay() {
  local _distro="${ROS_DISTRO_NAME:-jazzy}"
  local _ws="/root/automap_ws"
  local _ov="${_ws}/install_deps/cv_bridge_overlay"
  local _root="${_ws}/src_vision_opencv"
  local _pj="${AUTOMAP_BUILD_JOBS:-${HOST_NPROC:-$(nproc)}}"
  local _git_url="${AUTOMAP_VISION_OPENCV_GIT_URL:-https://github.com/ros-perception/vision_opencv.git}"
  mkdir -p "${_root}" "${_ws}/install_deps"
  if [ ! -f "${_root}/vision_opencv/cv_bridge/CMakeLists.txt" ]; then
    automap_log_progress "克隆 vision_opencv 并编译 cv_bridge（Jazzy 使用 rolling 分支，非 jazzy 用发行版名）…"
    rm -rf "${_root}/vision_opencv"
    local _branches="${AUTOMAP_VISION_OPENCV_GIT_BRANCH}"
    if [ -z "${_branches}" ]; then
      case "${_distro}" in
        jazzy) _branches="rolling iron humble" ;;
        *) _branches="${_distro} rolling humble" ;;
      esac
    fi
    local _b _ok=0
    for _b in ${_branches}; do
      echo "[INFO] 尝试 git clone vision_opencv 分支: ${_b}"
      if git clone --depth 1 --branch "${_b}" "${_git_url}" "${_root}/vision_opencv" 2>/dev/null; then
        _ok=1
        echo "[INFO] vision_opencv 克隆成功: 分支 ${_b}"
        break
      fi
      rm -rf "${_root}/vision_opencv"
    done
    if [ "${_ok}" != 1 ]; then
      echo "[ERROR] git clone vision_opencv 失败（已尝试分支: ${_branches}）。可设 AUTOMAP_VISION_OPENCV_GIT_BRANCH / AUTOMAP_VISION_OPENCV_GIT_URL。" >&2
      return 1
    fi
  fi
  cd "${_ws}" || return 1
  # shellcheck source=/dev/null
  source "/opt/ros/${_distro}/setup.bash"
  automap_log_progress "colcon：仅构建 cv_bridge → ${_ov} …"
  colcon build \
    --base-paths "${_root}" \
    --packages-select cv_bridge \
    --install-base "${_ov}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers "${_pj}" || return 1
  automap_write_cv_bridge_ros1_shim "${_distro}" "${_ov}" || true
  export CMAKE_PREFIX_PATH="${_ov}:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${_ov}/lib:${LD_LIBRARY_PATH}"
  return 0
}

# 保证 cv_bridge 头文件在磁盘上（部分镜像的 ros-*-cv-bridge 仅有 CMake/share 无 include）
automap_ensure_cv_bridge_headers() {
  local _distro="${ROS_DISTRO_NAME:-jazzy}"
  local _ws="/root/automap_ws"
  local _h="/opt/ros/${_distro}/include/cv_bridge/cv_bridge.h"
  local _hpp="/opt/ros/${_distro}/include/cv_bridge/cv_bridge/cv_bridge.hpp"
  if [ -f "${_hpp}" ] && [ ! -f "${_h}" ]; then
    automap_write_cv_bridge_ros1_shim "${_distro}" || true
  fi
  if [ -f "${_h}" ] || [ -f /usr/include/cv_bridge/cv_bridge.h ]; then
    return 0
  fi
  # 若上次运行留下了本脚本添加的 .list，与镜像 deb822 内联 Signed-By 并存会导致 apt 报错
  if _automap_ros2_repo_declared && [ -f /etc/apt/sources.list.d/ros2-packages-org.list ]; then
    echo "[INFO] 检测到已有 ros2 apt 源，移除 automap 重复的 ros2-packages-org.list（避免 Signed-By 冲突）"
    rm -f /etc/apt/sources.list.d/ros2-packages-org.list
  fi
  echo "[INFO] 未找到 cv_bridge.h，尝试 reinstall / apt 更新…"
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "[ERROR] 无 apt-get，无法补装 cv_bridge 头文件。" >&2
    exit 1
  fi
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --reinstall "ros-${_distro}-cv-bridge" 2>/dev/null || true
  automap_refresh_cv_bridge_shim_after_apt "${_distro}"
  if [ -f "${_h}" ] || [ -f /usr/include/cv_bridge/cv_bridge.h ]; then
    return 0
  fi
  # 镜像 apt 可能装到不完整包：尽早从官方 pool 合并与 apt 同版本的 .deb
  if automap_merge_cv_bridge_from_official_pool_deb; then
    automap_refresh_cv_bridge_shim_after_apt "${_distro}"
    if [ -f "${_h}" ] || [ -f "${_hpp}" ]; then
      echo "[INFO] cv_bridge 已由官方 pool deb 合并至 /opt/ros/${_distro}"
      return 0
    fi
  fi
  local _codename
  _codename="$( (grep -E '^VERSION_CODENAME=' /etc/os-release 2>/dev/null | cut -d= -f2 | tr -d '"') || true)"
  if [ -z "${_codename}" ] && command -v lsb_release >/dev/null 2>&1; then
    _codename="$(lsb_release -cs 2>/dev/null || true)"
  fi
  if [ -z "${_codename}" ]; then
    echo "[ERROR] 无法检测 Ubuntu codename。" >&2
    exit 1
  fi
  if _automap_ros2_repo_declared; then
    automap_log_progress "已存在 packages.ros.org ros2 apt 源，跳过重复添加（避免 Signed-By 冲突）"
    DEBIAN_FRONTEND=noninteractive apt-get update -qq || true
  else
    automap_log_progress "添加 packages.ros.org ros2 apt（${_codename}）…"
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq curl ca-certificates 2>/dev/null || true
    install -d -m 0755 /usr/share/keyrings
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${_codename} main" > /etc/apt/sources.list.d/ros2-packages-org.list
    DEBIAN_FRONTEND=noninteractive apt-get update -qq
  fi
  automap_log_progress "安装/重装 ros-${_distro}-cv-bridge（优先固定为官方 deb 版本串）…"
  local _pin="${AUTOMAP_CV_BRIDGE_OFFICIAL_DEB_VERSION}"
  if [ "${_distro}" = "jazzy" ] && [ -z "${_pin}" ]; then
    _pin="4.1.0-1noble.20260126.175659"
  fi
  if [ -n "${_pin}" ]; then
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "ros-${_distro}-cv-bridge=${_pin}" 2>/dev/null || true
  fi
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "ros-${_distro}-cv-bridge" || true
  automap_refresh_cv_bridge_shim_after_apt "${_distro}"
  if [ -f "${_h}" ] || [ -f /usr/include/cv_bridge/cv_bridge.h ]; then
    return 0
  fi
  # 第二次尝试 pool（若前面因网络失败）；合并后写兼容 cv_bridge.h
  if automap_merge_cv_bridge_from_official_pool_deb; then
    automap_refresh_cv_bridge_shim_after_apt "${_distro}"
    if [ -f "${_h}" ] || [ -f "${_hpp}" ]; then
      echo "[INFO] cv_bridge 已由官方 pool deb 合并至 /opt/ros/${_distro}"
      return 0
    fi
  fi
  # apt / pool 仍失败：从 vision_opencv 编译 overlay
  if automap_build_cv_bridge_overlay; then
    if [ -f "${_ws}/install_deps/cv_bridge_overlay/include/cv_bridge/cv_bridge.h" ] || \
       [ -f "${_ws}/install_deps/cv_bridge_overlay/include/cv_bridge/cv_bridge/cv_bridge.hpp" ]; then
      echo "[INFO] cv_bridge 已由源码 overlay 提供: ${_ws}/install_deps/cv_bridge_overlay"
      return 0
    fi
  fi
  echo "[ERROR] 仍无法找到 cv_bridge 头（期望 ${_h}、${_hpp} 或 overlay）。" >&2
  exit 1
}

# ONNX Runtime CUDA EP 要求 ${CUDNN_HOME}/include/cudnn_version.h 与 cudnn.h；Debian 多架构常仅装在 include/x86_64-linux-gnu/
automap_ensure_cudnn_std_include() {
  local d="/usr/include/x86_64-linux-gnu"
  [ -d "${d}" ] || return 0
  local f base
  base=$(basename "${d}")
  for f in cudnn_version.h cudnn.h; do
    if [ ! -f "/usr/include/${f}" ] && [ -f "${d}/${f}" ]; then
      ln -sf "${base}/${f}" "/usr/include/${f}" 2>/dev/null || true
      echo "[INFO] ONNX/cuDNN: 已链接 /usr/include/${f} → ${d}/${f}（供 CUDNN_HOME=/usr 与 ORT --use_cuda）"
    fi
  done
}

# 解析 ONNX Runtime --use_cuda 所需的 CUDA_HOME / CUDNN_HOME（成功时设置 _ORT_CUDA、_ORT_CUDNN，返回 0）
automap_resolve_ort_cuda_cudnn_paths() {
  automap_ensure_cudnn_std_include
  _ORT_CUDA="${CUDA_HOME:-/usr/local/cuda}"
  _ORT_CUDNN=""
  if [ -n "${CUDNN_HOME:-}" ] && [ -f "${CUDNN_HOME}/include/cudnn_version.h" ]; then
    _ORT_CUDNN="${CUDNN_HOME}"
  elif [ -f "${_ORT_CUDA}/include/cudnn_version.h" ]; then
    _ORT_CUDNN="${_ORT_CUDA}"
  elif [ -f /usr/include/cudnn_version.h ]; then
    _ORT_CUDNN="/usr"
  else
    _ORT_CUDNN_H=""
    for _c in "/usr/include/x86_64-linux-gnu/cudnn_version.h" "${_ORT_CUDA}/include/cudnn_version.h" "/usr/local/cuda/include/cudnn_version.h"; do
      [ -f "${_c}" ] && _ORT_CUDNN_H="${_c}" && break
    done
    if [ -z "${_ORT_CUDNN_H}" ] && command -v find >/dev/null 2>&1; then
      _ORT_CUDNN_H=$(find /usr /opt /usr/local -maxdepth 8 -name cudnn_version.h 2>/dev/null | head -1)
    fi
    if [ -n "${_ORT_CUDNN_H}" ] && [ -f "${_ORT_CUDNN_H}" ]; then
      _inc=$(dirname "${_ORT_CUDNN_H}")
      if [ "$(basename "${_inc}")" = "x86_64-linux-gnu" ]; then
        automap_ensure_cudnn_std_include
        if [ -f /usr/include/cudnn_version.h ]; then
          _ORT_CUDNN="/usr"
        fi
      elif [ "$(basename "${_inc}")" = "include" ]; then
        _ORT_CUDNN=$(dirname "${_inc}")
      fi
    fi
  fi
  if [ ! -d "${_ORT_CUDA}" ] || [ -z "${_ORT_CUDNN}" ] || [ ! -d "${_ORT_CUDNN}" ]; then
    return 1
  fi
  if [ ! -f "${_ORT_CUDNN}/include/cudnn_version.h" ] && [ ! -f "${_ORT_CUDA}/include/cudnn_version.h" ]; then
    return 1
  fi
  return 0
}

# CUDA 版 LibTorch：CMake find_package(Torch) 会拉 Caffe2Config，要求本机存在 CUDA 工具链（nvcc、cudart 头文件等）。
# NGC/最小镜像常仅有驱动 + 预编译 cu LibTorch，无 nvcc → Caffe2Config.cmake 直接 FATAL_ERROR。
automap_export_cuda_home_for_cmake() {
  if ! command -v nvcc >/dev/null 2>&1; then
    if [ -n "${CUDA_HOME:-}" ] && [ -x "${CUDA_HOME}/bin/nvcc" ]; then
      export PATH="${CUDA_HOME}/bin:${PATH}"
    else
      return 1
    fi
  fi
  if [ -f /usr/lib/cuda/include/cuda.h ]; then
    export CUDA_HOME=/usr/lib/cuda
  elif [ -f /usr/local/cuda/include/cuda.h ]; then
    export CUDA_HOME=/usr/local/cuda
  elif [ -f /usr/include/cuda.h ]; then
    export CUDA_HOME=/usr
  else
    local _nv
    _nv=$(command -v nvcc)
    export CUDA_HOME
    CUDA_HOME=$(dirname "$(dirname "$(readlink -f "${_nv}" 2>/dev/null || echo "${_nv}")")")
  fi
  export PATH="${CUDA_HOME}/bin:${PATH}"
  echo "[INFO] CUDA_HOME=${CUDA_HOME}（供 CMake find_package(Torch)/CUDA）"
  return 0
}

automap_ensure_cuda_toolkit_for_libtorch_cmake() {
  local lt="${LIBTORCH_INSTALL_DIR:-}"
  [ -d "${lt}/lib" ] || return 0
  if [ ! -f "${lt}/lib/libtorch_cuda.so" ] && [ ! -f "${lt}/lib/libc10_cuda.so" ]; then
    return 0
  fi
  if automap_export_cuda_home_for_cmake 2>/dev/null; then
    return 0
  fi
  if [ "${AUTOMAP_SKIP_CUDA_TOOLKIT_APT:-0}" = "1" ]; then
    echo "[WARN] LibTorch 为 CUDA 构建但容器内无 nvcc，且 AUTOMAP_SKIP_CUDA_TOOLKIT_APT=1 未尝试 apt 安装。colcon 可能在 find_package(Torch) 失败；请挂载含 nvcc 的 CUDA 或导出 CUDA_HOME。" >&2
    return 0
  fi
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "[WARN] LibTorch 为 CUDA 构建但无 nvcc 且无 apt-get；请安装 CUDA toolkit。" >&2
    return 0
  fi
  echo "[INFO] LibTorch CUDA 版需要 nvcc 供 CMake：安装 nvidia-cuda-toolkit（体积较大；已有工具链可设 AUTOMAP_SKIP_CUDA_TOOLKIT_APT=1）…"
  automap_log_progress "apt-get install nvidia-cuda-toolkit（LibTorch CMake 依赖 nvcc）…"
  # 与 automap_configure_apt_cache 一致：_apt 需可写 partial，否则出现 unsandboxed download 警告
  _apt_cache_root="${AUTOMAP_DOWNLOAD_CACHE:-/root/automap_download_cache}"
  if id _apt &>/dev/null; then
    mkdir -p "${_apt_cache_root}/apt/archives/partial" 2>/dev/null || true
    chown -R _apt:root "${_apt_cache_root}/apt/archives" 2>/dev/null || true
    chmod -R u+rwX,g+rX "${_apt_cache_root}/apt/archives" 2>/dev/null || true
  fi
  DEBIAN_FRONTEND=noninteractive apt-get update -qq
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq nvidia-cuda-toolkit || {
    echo "[WARN] nvidia-cuda-toolkit 安装失败；若 find_package(Torch) 仍失败，请手动安装 CUDA 并设置 CUDA_HOME。" >&2
    return 0
  }
  automap_export_cuda_home_for_cmake || true
}

automap_log_progress "容器内编译流程开始：apt → ROS/系统包 → 第三方库 → colcon（各阶段可能数分钟无新输出，属正常）"

# Jazzy/Isaac：automap_pro CMake 依赖 pcl_ros / cv_bridge / image_transport / message_filters；基础镜像常未预装
# libpcap-dev：PCL 的 FindPcap（Velodyne 等 IO）在最小镜像中会缺 PCAP_LIBRARIES
# libgeographiclib-dev：Ubuntu 24.04+ 包名（GeographicLib）；旧名 libgeographic-dev 在 noble 无候选
# CMakeLists find_package(GeographicLib)（/usr/share/cmake/geographiclib）
# unzip：LibTorch 等为 zip 预编译包，最小容器常未预装
if command -v apt-get >/dev/null 2>&1 && [ "${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" != "1" ]; then
  # 镜像常已含 deb822 的 ros2 源；若上次运行留下 ros2-packages-org.list，会与内联 Signed-By 冲突导致 apt 无法 update
  if _automap_ros2_repo_declared; then
    rm -f /etc/apt/sources.list.d/ros2-packages-org.list
  fi
  echo "[INFO] 安装 ROS 与系统依赖（${ROS_DISTRO_NAME}）：pcl_ros、cv_bridge、…、libgeographiclib-dev、libceres-dev（sloam/Ceres；无 thrid_party/ceres-solver 时使用系统 Ceres）"
  automap_log_progress "apt-get update（镜像/网络较慢时可能 1–5 分钟，请等待）…"
  DEBIAN_FRONTEND=noninteractive apt-get update -qq
  automap_log_progress "apt-get install ROS 与系统包（包较多时可能 5–15 分钟）…"
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    "ros-${ROS_DISTRO_NAME}-pcl-ros" \
    "ros-${ROS_DISTRO_NAME}-pcl-conversions" \
    "ros-${ROS_DISTRO_NAME}-cv-bridge" \
    "ros-${ROS_DISTRO_NAME}-image-transport" \
    "ros-${ROS_DISTRO_NAME}-message-filters" \
    "ros-${ROS_DISTRO_NAME}-sophus" \
    "ros-${ROS_DISTRO_NAME}-rviz2" \
    libpcap-dev \
    libgeographiclib-dev \
    libceres-dev \
    unzip # LibTorch / 其他 zip 解压
  export CMAKE_PREFIX_PATH="/opt/ros/${ROS_DISTRO_NAME}:${CMAKE_PREFIX_PATH}"
  automap_ensure_cv_bridge_headers
  automap_log_progress "apt 依赖已安装，进入编译阶段"
elif [ "${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" = "1" ]; then
  echo "[INFO] AUTOMAP_PREBUILT_INSTALL_DEPS=1：跳过 apt 安装 ROS/系统依赖（沿用 install_deps 与镜像现有环境）"
fi

# 防止 colcon 将 ONNX 源码树当作包扫描（setup.py 会触发 python_setup_py 且污染日志）
if [ -d /root/automap_ws/src_onnxruntime ]; then
  : > /root/automap_ws/src_onnxruntime/COLCON_IGNORE
fi

# 充分发挥宿主机性能：优先使用宿主机传入的核数（AUTOMAP_BUILD_JOBS 或 HOST_NPROC），否则容器内 nproc
PARALLEL_JOBS="${AUTOMAP_BUILD_JOBS:-${HOST_NPROC:-$(nproc)}}"
export CMAKE_BUILD_PARALLEL_LEVEL="${PARALLEL_JOBS}"
COLCON_PARALLEL="--parallel-workers ${PARALLEL_JOBS}"
# status+：ROS2 Jazzy 自带 colcon 使用 status+（非旧版 status_line+）
COLCON_EVENT_HANDLERS="${COLCON_EVENT_HANDLERS:---event-handlers status+}"
echo "[INFO] 并行编译: 每包 ${PARALLEL_JOBS} 线程, colcon --parallel-workers ${PARALLEL_JOBS}"

# 使用 Ninja 生成器可显著加速编译（镜像已装 ninja-build）
NINJA_CMAKE_ARG=""
if command -v ninja >/dev/null 2>&1; then
  NINJA_CMAKE_ARG="-G Ninja"
  echo "[INFO] 使用 Ninja 生成器以加速编译"
fi

# 第三方库（GTSAM/TEASER++/vikit）安装到 install_deps，--clean 时不删除，编译一次后续跳过
INSTALL_DEPS="/root/automap_ws/install_deps"
mkdir -p "${INSTALL_DEPS}"
# 防止 colcon 递归扫描 install_deps（其中可能含 Python venv/site-packages，会触发 python_setup_py 识别异常并污染日志）
: > "${INSTALL_DEPS}/COLCON_IGNORE"
# LSK3DNet venv（若启用）同样不应被当作 workspace 包扫描
if [ -d "${INSTALL_DEPS}/lsk3dnet_venv" ]; then
  : > "${INSTALL_DEPS}/lsk3dnet_venv/COLCON_IGNORE"
fi
# 宿主机已预置 install_deps（含 libtorch/onnxruntime 等）时：不下载 LibTorch、不 apt 装 nvidia-cuda-toolkit；可单独覆盖 LIBTORCH_SKIP_DOWNLOAD / AUTOMAP_SKIP_CUDA_TOOLKIT_APT
if [ "${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" = "1" ]; then
  : "${LIBTORCH_SKIP_DOWNLOAD:=1}"
  : "${AUTOMAP_SKIP_CUDA_TOOLKIT_APT:=1}"
  echo "[INFO] AUTOMAP_PREBUILT_INSTALL_DEPS=1：假定 ${INSTALL_DEPS} 已含预置依赖；跳过 LibTorch 下载与 apt 安装 nvidia-cuda-toolkit（请自行保证 nvcc/CUDA_HOME 与 cu128 等匹配 Blackwell）"
fi

# 若 CMake 缓存为其他路径创建，清理（不删 install_deps）
if [ -d build ] && find build -name CMakeCache.txt -exec grep -l '/workspace/' {} \; 2>/dev/null | grep -q .; then
  echo '[WARN] 检测到 CMake 缓存路径不一致，清理 build/install/log'
  rm -rf build install log build_teaserpp build_sophus build_ceres
fi

# 检测 automap_pro/CMakeLists.txt 是否比 build 目录更新，如果是则清理缓存
if [ -d build/automap_pro ]; then
  CMAKE_CACHE_TIME=$(find build/automap_pro -name CMakeCache.txt -type f 2>/dev/null | head -1 | xargs stat -c %Y 2>/dev/null || echo 0)
  CMAKE_FILE_TIME=$(find automap_pro/CMakeLists.txt -type f 2>/dev/null | head -1 | xargs stat -c %Y 2>/dev/null || echo 0)
  if [ "${CMAKE_FILE_TIME}" -gt "${CMAKE_CACHE_TIME}" ]; then
    echo '[INFO] CMakeLists.txt 已更新，清理 automap_pro 编译缓存'
    rm -rf build/automap_pro
  fi
fi

# 链入 overlap_transformer_msgs / overlap_transformer_ros2 / hba（若存在）
[ -d /root/mapping/overlap_transformer_msgs ] && ln -sf /root/mapping/overlap_transformer_msgs src/ 2>/dev/null || true
[ -d /root/mapping/overlap_transformer_ros2 ] && ln -sf /root/mapping/overlap_transformer_ros2 src/ 2>/dev/null || true
[ -d /root/mapping/HBA-main/HBA_ROS2 ]        && ln -sf /root/mapping/HBA-main/HBA_ROS2 src/hba 2>/dev/null || true

# ScanContext 在 automap_pro 编译时通过 -DSCANCONTEXT_ROOT 参数传入

if [ -d src/overlap_transformer_msgs ]; then
  echo '========================================'
  echo '编译 overlap_transformer_msgs'
  echo '========================================'
  automap_log_progress "colcon：overlap_transformer_msgs"
  colcon build ${COLCON_PARALLEL} ${COLCON_EVENT_HANDLERS} --packages-select overlap_transformer_msgs --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
  automap_log_progress "overlap_transformer_msgs 完成"
fi

if [ -d src/overlap_transformer_ros2 ]; then
  echo '========================================'
  echo '编译 overlap_transformer_ros2'
  echo '========================================'
  automap_log_progress "colcon：overlap_transformer_ros2"
  colcon build ${COLCON_PARALLEL} ${COLCON_EVENT_HANDLERS} --packages-select overlap_transformer_ros2 --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
  automap_log_progress "overlap_transformer_ros2 完成"
fi

HBA_INSTALL_DIR="${INSTALL_DEPS}/hba"
if [ -d src/hba ]; then
  echo '========================================'
  echo 'Compile GTSAM first (disable TBB to avoid SIGSEGV)'
  echo '========================================'
  GTSAM_INSTALL_DIR="${INSTALL_DEPS}/gtsam"
  NEED_GTSAM_BUILD=false
  if [ ! -f "${GTSAM_INSTALL_DIR}/lib/libgtsam.so" ]; then
    NEED_GTSAM_BUILD=true
  else
    # 已安装的 GTSAM 若与当前环境 Eigen 版本不一致会导致 hba 编译报错，需强制重编
    CURRENT_EIGEN_WORLD=$(grep -m1 '#define EIGEN_WORLD_VERSION' /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | sed 's/.*EIGEN_WORLD_VERSION[^0-9]*\([0-9]*\).*/\1/' || echo "")
    CURRENT_EIGEN_MAJOR=$(grep -m1 '#define EIGEN_MAJOR_VERSION' /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | sed 's/.*EIGEN_MAJOR_VERSION[^0-9]*\([0-9]*\).*/\1/' || echo "")
    GTSAM_CFG="${GTSAM_INSTALL_DIR}/include/gtsam/config.h"
    if [ -n "${CURRENT_EIGEN_WORLD}${CURRENT_EIGEN_MAJOR}" ] && [ -f "${GTSAM_CFG}" ]; then
      GTSAM_EIGEN_WORLD=$(grep -m1 '#define GTSAM_EIGEN_VERSION_WORLD' "${GTSAM_CFG}" 2>/dev/null | sed 's/.*GTSAM_EIGEN_VERSION_WORLD[^0-9]*\([0-9]*\).*/\1/' || echo "")
      GTSAM_EIGEN_MAJOR=$(grep -m1 '#define GTSAM_EIGEN_VERSION_MAJOR' "${GTSAM_CFG}" 2>/dev/null | sed 's/.*GTSAM_EIGEN_VERSION_MAJOR[^0-9]*\([0-9]*\).*/\1/' || echo "")
      if [ "${CURRENT_EIGEN_WORLD}" != "${GTSAM_EIGEN_WORLD}" ] || [ "${CURRENT_EIGEN_MAJOR}" != "${GTSAM_EIGEN_MAJOR}" ]; then
        echo "[INFO] Eigen 版本与已安装 GTSAM 不一致 (当前 ${CURRENT_EIGEN_WORLD}.${CURRENT_EIGEN_MAJOR} vs GTSAM 构建时 ${GTSAM_EIGEN_WORLD}.${GTSAM_EIGEN_MAJOR})，将重新编译 GTSAM"
        rm -rf "${GTSAM_INSTALL_DIR}" /root/automap_ws/build_gtsam_no_tbb
        NEED_GTSAM_BUILD=true
      fi
    fi
  fi
  if [ "$NEED_GTSAM_BUILD" = true ]; then
      echo "[INFO] Building GTSAM without TBB..."
      automap_log_progress "GTSAM：cmake 配置…"
      if [ -d /root/automap_ws/build_gtsam_no_tbb ]; then
          rm -rf /root/automap_ws/build_gtsam_no_tbb
      fi
      mkdir -p /root/automap_ws/build_gtsam_no_tbb
      cd /root/automap_ws/build_gtsam_no_tbb
      cmake /root/automap_ws/src/automap_pro/thrid_party/gtsam \
          ${NINJA_CMAKE_ARG} \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_INSTALL_PREFIX=${GTSAM_INSTALL_DIR} \
          -DGTSAM_WITH_TBB=OFF \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DBUILD_SHARED_LIBS=ON \
          -DBUILD_TESTS=OFF \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_PYTHON=OFF
      automap_log_progress "GTSAM：编译与安装（可能数分钟）…"
      if [ -n "${NINJA_CMAKE_ARG}" ]; then ninja -j${PARALLEL_JOBS}; ninja install; else make -j${PARALLEL_JOBS}; make install; fi
      mkdir -p ${GTSAM_INSTALL_DIR}/share/gtsam
      echo '<?xml version="1.0"?>' > ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '<package format="3">' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '  <name>gtsam</name>' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '  <version>4.2.0</version>' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      echo '</package>' >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.xml
      # colcon 依赖检查需要 package.sh（ament 风格）
      printf '# ament package env for gtsam\n' > ${GTSAM_INSTALL_DIR}/share/gtsam/package.sh
      printf 'export CMAKE_PREFIX_PATH="%s:$CMAKE_PREFIX_PATH"\n' "${GTSAM_INSTALL_DIR}" >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.sh
      printf 'export LD_LIBRARY_PATH="%s/lib:$LD_LIBRARY_PATH"\n' "${GTSAM_INSTALL_DIR}" >> ${GTSAM_INSTALL_DIR}/share/gtsam/package.sh
      echo "[INFO] GTSAM compiled and installed"
      cd /root/automap_ws
  else
      echo "[INFO] GTSAM 已安装于 install_deps，跳过"
  fi

  # 若缺少 package.sh（旧安装或 colcon 依赖检查需要）则补写
  if [ ! -f "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh" ]; then
    mkdir -p "${GTSAM_INSTALL_DIR}/share/gtsam"
    printf '# ament package env for gtsam\n' > "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh"
    printf 'export CMAKE_PREFIX_PATH="%s:$CMAKE_PREFIX_PATH"\n' "${GTSAM_INSTALL_DIR}" >> "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh"
    printf 'export LD_LIBRARY_PATH="%s/lib:$LD_LIBRARY_PATH"\n' "${GTSAM_INSTALL_DIR}" >> "${GTSAM_INSTALL_DIR}/share/gtsam/package.sh"
  fi
  export CMAKE_PREFIX_PATH="${GTSAM_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"

  # 让 colcon 在 workspace install 下找到 gtsam（hba 依赖）
  mkdir -p /root/automap_ws/install
  ln -sfn /root/automap_ws/install_deps/gtsam /root/automap_ws/install/gtsam

  # hba：已安装则跳过（colcon 产出 setup.bash，--clean 不删 install_deps）
  NEED_HBA_BUILD=true
  if [ -f "${HBA_INSTALL_DIR}/setup.bash" ] || [ -f "${HBA_INSTALL_DIR}/lib/libhba_core.so" ] || [ -d "${HBA_INSTALL_DIR}/lib" ]; then
    NEED_HBA_BUILD=false
  fi
  if [ "$NEED_HBA_BUILD" = true ]; then
    echo '========================================'
    echo '编译 hba (HBA-main)（安装到 install_deps/hba）'
    echo '========================================'
    automap_log_progress "colcon：hba（依赖 GTSAM）"
    mkdir -p "${HBA_INSTALL_DIR}"
    # colcon 在 --install-base 下查找依赖，需在 hba 目录内提供 gtsam 符号链接
    ln -sfn ../gtsam "${HBA_INSTALL_DIR}/gtsam" 2>/dev/null || true
    colcon build ${COLCON_PARALLEL} ${COLCON_EVENT_HANDLERS} --install-base "${HBA_INSTALL_DIR}" --packages-select hba --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] hba 已安装于 install_deps/hba"
  else
    echo "[INFO] hba 已安装于 install_deps，跳过"
  fi
  [ -d "${HBA_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${HBA_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${HBA_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${HBA_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
  ln -sfn "${HBA_INSTALL_DIR}" /root/automap_ws/install/hba 2>/dev/null || true
fi

# TEASER++ 源码（安装到 install_deps，已安装则跳过）
TEASER_SRC=""
[ -d /root/mapping/TEASER-plusplus-master ] && TEASER_SRC=/root/mapping/TEASER-plusplus-master
[ -z "${TEASER_SRC}" ] && [ -d /root/mapping/automap_pro/src/modular/TEASER-plusplus-master ] && TEASER_SRC=/root/mapping/automap_pro/src/modular/TEASER-plusplus-master

TEASER_INSTALL_DIR="${INSTALL_DEPS}/teaserpp"
if [ -n "${TEASER_SRC}" ]; then
  if [ -f "${TEASER_INSTALL_DIR}/lib/cmake/teaserpp/teaserppTargets.cmake" ]; then
    echo "[INFO] TEASER++ 已安装于 install_deps，跳过"
  else
    echo '========================================'
    echo '编译 TEASER++'
    echo '========================================'
    automap_log_progress "TEASER++：cmake…"
    TEASER_BUILD=/root/automap_ws/build_teaserpp
    mkdir -p "${TEASER_INSTALL_DIR}"
    mkdir -p "${TEASER_BUILD}" && cd "${TEASER_BUILD}"
    MAP_THRID=/root/mapping/automap_pro/thrid_party
    [ ! -d "${MAP_THRID}/pmc-master" ] && MAP_THRID=/root/mapping/thrid_party
    # 使用本地 thrid_party 的 pmc/tinyply/spectra，避免 FetchContent 访问 GitHub 超时/离线失败
    TEASER_CMAKE_EXTRA=""
    [ -d "${MAP_THRID}/spectra" ] && TEASER_CMAKE_EXTRA="-DFETCHCONTENT_SOURCE_DIR_SPECTRA=${MAP_THRID}/spectra"
    cmake "${TEASER_SRC}" \
        ${NINJA_CMAKE_ARG} \
        -DCMAKE_INSTALL_PREFIX="${TEASER_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DPMC_SOURCE_DIR="${MAP_THRID}/pmc-master" \
        -DTINYPLY_SOURCE_DIR="${MAP_THRID}/tinyply" \
        -DFETCHCONTENT_SOURCE_DIR_PMC="${MAP_THRID}/pmc-master" \
        -DFETCHCONTENT_SOURCE_DIR_TINYPLY="${MAP_THRID}/tinyply" \
        ${TEASER_CMAKE_EXTRA}
    automap_log_progress "TEASER++：编译安装…"
    if [ -n "${NINJA_CMAKE_ARG}" ]; then ninja -j${PARALLEL_JOBS}; ninja install; else make -j${PARALLEL_JOBS}; make install; fi
    echo "[INFO] TEASER++ 已安装于 install_deps"
  fi
  cd /root/automap_ws
fi

# Ceres Solver（与 GTSAM 一致：安装到 install_deps/ceres，首次编译后后续跳过）
CERES_INSTALL_DIR="${INSTALL_DEPS}/ceres"
CERES_SRC="/root/automap_ws/src/automap_pro/thrid_party/ceres-solver"
if [ -d "${CERES_SRC}" ] && [ -f "${CERES_SRC}/CMakeLists.txt" ]; then
  # 仅以 CeresConfig.cmake 判定“已安装”：仅有 libceres.so 而无 CMake 包会导致 colcon 找不到 Ceres::ceres
  NEED_CERES_BUILD=false
  if [ ! -f "${CERES_INSTALL_DIR}/lib/cmake/Ceres/CeresConfig.cmake" ] \
      && [ ! -f "${CERES_INSTALL_DIR}/lib64/cmake/Ceres/CeresConfig.cmake" ]; then
    NEED_CERES_BUILD=true
  fi
  if [ "$NEED_CERES_BUILD" = true ]; then
    echo '========================================'
    echo '编译 Ceres Solver（安装到 install_deps/ceres）'
    echo '========================================'
    automap_log_progress "Ceres：cmake…"
    CERES_BUILD=/root/automap_ws/build_ceres
    rm -rf "${CERES_BUILD}"
    mkdir -p "${CERES_BUILD}" && cd "${CERES_BUILD}"
    # Ubuntu suitesparse 常缺 METIS，FindSuiteSparse 会得到畸形版本 ".." 并禁用 SUITESPARSE；此时若仍启用 CXSPARSE，
    # Ceres 2.1 可能引用未创建的 CXSparse::CXSparse 导入目标导致 CMake Configure 失败（见 build.log）。
    # 关闭二者，仅依赖 Eigen 稀疏（EIGENSPARSE 默认 ON），与日志中 CERES_NO_SUITESPARSE 路径一致且可链接。
    cmake "${CERES_SRC}" \
        ${NINJA_CMAKE_ARG} \
        -DCMAKE_INSTALL_PREFIX="${CERES_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_TESTING=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_BENCHMARKS=OFF \
        -DBUILD_DOCUMENTATION=OFF \
        -DSUITESPARSE=OFF \
        -DCXSPARSE=OFF
    automap_log_progress "Ceres：编译安装（可能数分钟）…"
    if [ -n "${NINJA_CMAKE_ARG}" ]; then ninja -j${PARALLEL_JOBS}; ninja install; else make -j${PARALLEL_JOBS}; make install; fi
    echo "[INFO] Ceres Solver 已安装于 install_deps/ceres"
    cd /root/automap_ws
  else
    echo "[INFO] Ceres Solver 已安装于 install_deps，跳过"
  fi
  if [ -d "${CERES_INSTALL_DIR}/lib" ] || [ -d "${CERES_INSTALL_DIR}/lib64" ]; then
    export CMAKE_PREFIX_PATH="${CERES_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
    [ -d "${CERES_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${CERES_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
    [ -d "${CERES_INSTALL_DIR}/lib64" ] && export LD_LIBRARY_PATH="${CERES_INSTALL_DIR}/lib64:${LD_LIBRARY_PATH}"
  fi
fi

# vikit_common / vikit_ros（与 GTSAM 一致：安装到 install_deps/vikit 子目录，只编译一次，已安装则跳过）
VIKIT_INSTALL_DIR="${INSTALL_DEPS}/vikit"
VIKIT_SRC=""
[ -d src/thrid_party/rpg_vikit_ros2 ] && VIKIT_SRC="src/thrid_party/rpg_vikit_ros2"
[ -z "${VIKIT_SRC}" ] && [ -d src/automap_pro/thrid_party/rpg_vikit_ros2 ] && VIKIT_SRC="src/automap_pro/thrid_party/rpg_vikit_ros2"
if [ -n "${VIKIT_SRC}" ]; then
  NEED_VIKIT_BUILD=true
  # 已安装则跳过（colcon 产出 setup.bash，--clean 不删 install_deps）
  if [ -f "${VIKIT_INSTALL_DIR}/setup.bash" ] || [ -f "${VIKIT_INSTALL_DIR}/lib/libvikit_common.so" ] || [ -d "${VIKIT_INSTALL_DIR}/lib" ]; then
    NEED_VIKIT_BUILD=false
  elif [ -f "${INSTALL_DEPS}/lib/libvikit_common.so" ]; then
    NEED_VIKIT_BUILD=false
    VIKIT_INSTALL_DIR="${INSTALL_DEPS}"
  fi
  if [ "$NEED_VIKIT_BUILD" = true ]; then
    echo '========================================'
    echo '编译 vikit（安装到 install_deps/vikit，与 GTSAM 一致）'
    echo '========================================'
    automap_log_progress "colcon：vikit_common / vikit_ros"
    mkdir -p "${VIKIT_INSTALL_DIR}"
    colcon build ${COLCON_PARALLEL} ${COLCON_EVENT_HANDLERS} --install-base "${VIKIT_INSTALL_DIR}" --paths "${VIKIT_SRC}/vikit_common" "${VIKIT_SRC}/vikit_ros" --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] vikit 已安装于 install_deps/vikit"
  else
    echo "[INFO] vikit 已安装于 install_deps，跳过"
  fi
  # 让 colcon/ament 找到 vikit（与 GTSAM 一致）
  [ -d "${VIKIT_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${VIKIT_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${VIKIT_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${VIKIT_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
  # 独立 vikit 目录时创建符号链接，便于 overlay
  if [ "${VIKIT_INSTALL_DIR}" != "${INSTALL_DEPS}" ]; then
    mkdir -p /root/automap_ws/install
    ln -sfn "${VIKIT_INSTALL_DIR}" /root/automap_ws/install/vikit 2>/dev/null || true
  fi
fi

# ---- LibTorch 预编译包 URL（OverlapTransformer / LibTorch C++）----
# 优先级: LIBTORCH_URL > LIBTORCH_PREFER=cpu > nvcc 检测 CUDA 版本 > 默认 cu124（新 GPU / CUDA 12.x）
# 默认使用官方 https://download.pytorch.org/libtorch（清华 tuna …/pytorch-wheels/libtorch 下已无有效 libtorch/cu* 树，会 404）
# 可选: AUTOMAP_LIBTORCH_DOWNLOAD_BASE=与官方同路径的镜像根；AUTOMAP_USE_OFFICIAL_LIBTORCH=1 强制不用镜像重写
# 镜像下载失败时自动再试官方同路径（禁回退: AUTOMAP_LIBTORCH_NO_OFFICIAL_FALLBACK=1）
# 纯 CPU 环境可设: LIBTORCH_PREFER=cpu 或显式 LIBTORCH_URL=...cpu...
automap_resolve_libtorch_url() {
  local _off="https://download.pytorch.org/libtorch"
  local _mir="${AUTOMAP_LIBTORCH_DOWNLOAD_BASE:-${_off}}"
  local u=""
  # 直配版本：LIBTORCH_VERSION=2.7.0+cu128 或 2.5.1+cpu（无需手写完整 URL）
  # 优先级低于 LIBTORCH_URL（显式 URL 始终最高）
  if [ -n "${LIBTORCH_VERSION:-}" ]; then
    local _lt_ver="${LIBTORCH_VERSION}"
    local _lt_variant="${_lt_ver##*+}"
    if [ "${_lt_variant}" = "${_lt_ver}" ]; then
      echo "[ERROR] LIBTORCH_VERSION 格式无效: ${LIBTORCH_VERSION}（期望如 2.7.0+cu128 或 2.5.1+cpu）" >&2
      return 1
    fi
    local _lt_path="${_lt_variant}"
    if [ "${_lt_variant}" = "cpu" ]; then
      _lt_path="cpu"
    elif ! printf '%s' "${_lt_variant}" | grep -qE '^cu[0-9]{3}$'; then
      echo "[ERROR] LIBTORCH_VERSION 后缀无效: ${_lt_variant}（仅支持 cpu 或 cuXXX）" >&2
      return 1
    fi
    local _lt_ver_enc="${_lt_ver//+/%2B}"
    u="${_off}/${_lt_path}/libtorch-cxx11-abi-shared-with-deps-${_lt_ver_enc}.zip"
    if [ "${AUTOMAP_USE_OFFICIAL_LIBTORCH:-0}" != "1" ] && [ -n "$u" ]; then
      u="${u/${_off}/${_mir}}"
    fi
    printf '%s' "$u"
    return 0
  fi
  if [ -n "${LIBTORCH_URL:-}" ]; then
    printf '%s' "${LIBTORCH_URL}"
    return
  fi
  if [ "${LIBTORCH_PREFER:-}" = "cpu" ]; then
    u="${_off}/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcpu.zip"
  else
    local nvcc_bin=""
    if command -v nvcc >/dev/null 2>&1; then
      nvcc_bin=$(command -v nvcc)
    elif [ -n "${CUDA_HOME:-}" ] && [ -x "${CUDA_HOME}/bin/nvcc" ]; then
      nvcc_bin="${CUDA_HOME}/bin/nvcc"
    else
      for d in /usr/local/cuda /usr/local/cuda-12 /usr/local/cuda-11; do
        if [ -x "${d}/bin/nvcc" ]; then nvcc_bin="${d}/bin/nvcc"; break; fi
      done
    fi
    local ver=""
    if [ -n "${nvcc_bin}" ]; then
      ver=$("$nvcc_bin" --version 2>/dev/null | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p' | head -n1)
    fi
    # 驱动 / GPU 优先于 nvcc：apt 的 nvidia-cuda-toolkit 常为 nvcc 12.0，若据此选 cu121 会在 Blackwell（RTX 50）上错配；需 cu128。
    local smi_cuda="" gpu_line=""
    if command -v nvidia-smi >/dev/null 2>&1; then
      smi_cuda=$(nvidia-smi 2>/dev/null | sed -n 's/.*CUDA Version: *\([0-9]*\.[0-9]*\).*/\1/p' | head -1)
      if [ -z "${smi_cuda}" ]; then
        smi_cuda=$(nvidia-smi -q 2>/dev/null | grep -m1 'CUDA Version' | sed -n 's/.*: *\([0-9.]*\).*/\1/p' | head -1)
      fi
      gpu_line=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)
    fi
    u=""
    if echo "${gpu_line}" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
      u="${_off}/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip"
    elif [ -n "${smi_cuda}" ]; then
      case "${smi_cuda}" in
        12.8|12.9|13.*)
          u="${_off}/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip"
          ;;
      esac
    fi
    if [ -z "${u}" ]; then
      case "${ver}" in
        11.*)
          u="${_off}/cu118/libtorch-cxx11-abi-shared-with-deps-2.1.2%2Bcu118.zip"
          ;;
        12.0|12.1|12.2|12.3)
          u="${_off}/cu121/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcu121.zip"
          ;;
        12.4|12.5|12.6|12.7)
          u="${_off}/cu124/libtorch-cxx11-abi-shared-with-deps-2.5.1%2Bcu124.zip"
          ;;
        12.8|12.9|13.*)
          u="${_off}/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip"
          ;;
        *)
          u="${_off}/cu128/libtorch-cxx11-abi-shared-with-deps-2.7.0%2Bcu128.zip"
          ;;
      esac
    fi
  fi
  if [ "${AUTOMAP_USE_OFFICIAL_LIBTORCH:-0}" != "1" ] && [ -n "$u" ]; then
    u="${u/${_off}/${_mir}}"
  fi
  printf '%s' "$u"
}

# 下载大文件：先 wget（可重试、超时），再 curl；用于 LibTorch 等与镜像/网络抖动场景。
automap_download_to_file() {
  local _url="$1"
  local _dest="$2"
  if command -v wget >/dev/null 2>&1; then
    wget -q --show-progress --timeout=120 --tries=3 -O "${_dest}" "${_url}" && return 0
  fi
  if command -v curl >/dev/null 2>&1; then
    curl -fL --connect-timeout 30 --retry 3 --retry-delay 2 -o "${_dest}" "${_url}" && return 0
  fi
  return 1
}

# RTX 50 / Blackwell（sm_120）：要求 cu128 LibTorch + CUDA 12.8+；错配时直接失败（仅当容器内可见 GPU 且名称匹配）
# AUTOMAP_STRICT_BLACKWELL_STACK=0 可跳过（不推荐）
automap_verify_blackwell_accel_stack() {
  [ "${AUTOMAP_STRICT_BLACKWELL_STACK:-1}" = "0" ] && return 0
  command -v nvidia-smi >/dev/null 2>&1 || return 0
  local gpu_name
  gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)
  [ -z "${gpu_name}" ] && return 0
  if ! echo "${gpu_name}" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
    return 0
  fi
  local nvcc_bin=""
  if command -v nvcc >/dev/null 2>&1; then
    nvcc_bin=$(command -v nvcc)
  elif [ -n "${CUDA_HOME:-}" ] && [ -x "${CUDA_HOME}/bin/nvcc" ]; then
    nvcc_bin="${CUDA_HOME}/bin/nvcc"
  else
    for d in /usr/local/cuda/bin /usr/local/cuda-12.8/bin /usr/local/cuda-12/bin /usr/local/cuda-13/bin; do
      if [ -x "${d}/nvcc" ]; then nvcc_bin="${d}/nvcc"; break; fi
    done
    if [ -z "${nvcc_bin}" ] && [ -d /usr/local ]; then
      # shellcheck disable=SC2012
      for d in $(ls -d /usr/local/cuda-12.* /usr/local/cuda-13.* 2>/dev/null | sort -V); do
        if [ -x "${d}/bin/nvcc" ]; then nvcc_bin="${d}/bin/nvcc"; break; fi
      done
    fi
  fi
  local nvcc_v=""
  if [ -n "${nvcc_bin}" ]; then
    nvcc_v=$("${nvcc_bin}" --version 2>/dev/null | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p' | head -n1)
  fi
  # 驱动报告的 CUDA 版本（Blackwell 门控与 nvcc 取 max：apt 的 nvidia-cuda-toolkit 常为 nvcc 12.0，仅够 CMake find_package(Torch)；运行仍由驱动+cu128 LibTorch 决定）
  local smi_cuda=""
  smi_cuda=$(nvidia-smi 2>/dev/null | sed -n 's/.*CUDA Version: *\([0-9]*\.[0-9]*\).*/\1/p' | head -1)
  if [ -z "${smi_cuda}" ]; then
    smi_cuda=$(nvidia-smi -q 2>/dev/null | grep -m1 'CUDA Version' | sed -n 's/.*: *\([0-9.]*\).*/\1/p' | head -1)
  fi
  local ver_check=""
  local cuda_ver_note=""
  if [ -n "${nvcc_v}" ] && [ -n "${smi_cuda}" ]; then
    ver_check=$(printf '%s\n' "${nvcc_v}" "${smi_cuda}" | sort -V | tail -1)
    cuda_ver_note="门控=${ver_check}（nvcc=${nvcc_v} @ ${nvcc_bin}，nvidia-smi=${smi_cuda}）"
  elif [ -n "${nvcc_v}" ]; then
    ver_check="${nvcc_v}"
    cuda_ver_note="nvcc=${nvcc_v} (${nvcc_bin})"
  elif [ -n "${smi_cuda}" ]; then
    ver_check="${smi_cuda}"
    cuda_ver_note="nvidia-smi CUDA Version=${smi_cuda}（无 nvcc，预编译 LibTorch/多数推理仍可用）"
  fi
  if [ -n "${ver_check}" ]; then
    case "${ver_check}" in
      12.8|12.9|13.*) ;;
      *)
        echo "[ERROR] Blackwell GPU（${gpu_name}）需要 CUDA 运行时/工具链 ≥12.8；当前 ${cuda_ver_note:-ver=${ver_check}}。请升级驱动/镜像或设置 CUDA_HOME 指向含 12.8+ 的 toolkit。" >&2
        return 1
        ;;
    esac
  else
    echo "[ERROR] Blackwell GPU（${gpu_name}）：未找到 nvcc，且无法从 nvidia-smi 解析 CUDA Version。请安装 cuda-toolkit 或换用带完整 CUDA 的镜像；临时跳过: AUTOMAP_STRICT_BLACKWELL_STACK=0" >&2
    return 1
  fi
  if [ -z "${nvcc_v}" ] && [ -n "${smi_cuda}" ]; then
    echo "[WARN] ${cuda_ver_note}" >&2
  fi
  local lt_url="${LIBTORCH_URL:-}"
  if [ -z "${lt_url}" ] && [ "${LIBTORCH_PREFER:-}" != "cpu" ]; then
    lt_url="$(automap_resolve_libtorch_url)" || true
  fi
  local lt_ok=0
  case "${lt_url}" in
    *cu128*) lt_ok=1 ;;
  esac
  if [ "${lt_ok}" -eq 0 ] && [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ]; then
    if grep -qE 'cu128|2\.7\.0' "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" 2>/dev/null; then
      lt_ok=1
    fi
  fi
  if [ "${lt_ok}" -eq 0 ] && [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ] && command -v strings >/dev/null 2>&1; then
    if strings "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" 2>/dev/null | grep -q 'cu128'; then
      lt_ok=1
    fi
  fi
  if [ "${lt_ok}" -eq 0 ]; then
    echo "[ERROR] Blackwell（${gpu_name}）需要 LibTorch cu128（含 sm_120）。当前 URL=${lt_url:-未设置}，且未在 ${LIBTORCH_INSTALL_DIR} 检测到 cu128 构建。" >&2
    echo "[ERROR] 请删除 install_deps/libtorch 后重编，或设置 LIBTORCH_URL 指向 *cu128* 的 zip；勿使用 LIBTORCH_PREFER=cpu。跳过检查（不推荐）: AUTOMAP_STRICT_BLACKWELL_STACK=0" >&2
    return 1
  fi
  if [ "${ONNXRUNTIME_USE_CUDA:-0}" = "1" ]; then
    local _cudnn_h=""
    for _c in "${CUDNN_HOME:+${CUDNN_HOME}/include/cudnn_version.h}" \
              "${CUDA_HOME:-/usr/local/cuda}/include/cudnn_version.h" \
              "/usr/local/cuda/include/cudnn_version.h" \
              "/usr/include/cudnn_version.h" \
              "/usr/include/x86_64-linux-gnu/cudnn_version.h"; do
      [ -n "${_c}" ] && [ -f "${_c}" ] && _cudnn_h="${_c}" && break
    done
    if [ -z "${_cudnn_h}" ] && command -v find >/dev/null 2>&1; then
      _cudnn_h=$(find /usr /opt /usr/local -maxdepth 8 -name cudnn_version.h 2>/dev/null | head -1)
    fi
    if [ -z "${_cudnn_h}" ] || [ ! -f "${_cudnn_h}" ]; then
      # 与下方 ONNX 段一致：无头文件则仅 WARN，ORT 仍可能 CPU 构建，不因 NGC 缺 dev 包而中断整次编译
      echo "[WARN] ONNXRUNTIME_USE_CUDA=1 但未找到 cudnn_version.h（Isaac/NGC 常仅含运行时）。将尝试 CPU 构建 ORT；若需 GPU EP 请安装 cuDNN 开发头或设置 CUDNN_HOME。" >&2
    else
      echo "[INFO] cuDNN 头文件: ${_cudnn_h}"
    fi
  fi
  echo "[INFO] Blackwell 加速栈校验通过：GPU=${gpu_name} ${cuda_ver_note:-CUDA ok} LibTorch 含 cu128"
  return 0
}

# 统一 CUDA 栈：要求 libtorch / nvcc / libnvrtc 版本对齐，避免运行期 NVRTC 架构不支持导致崩溃
# 可通过 AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED=0 临时关闭（不推荐）
automap_verify_cuda_stack_unified() {
  [ "${AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED:-1}" = "0" ] && return 0

  local lt_tag=""
  local lt_mm=""
  local nvcc_mm=""
  local nvrtc_mm=""
  local gpu_name=""

  # 1) 解析 LibTorch CUDA 变体：优先 URL，其次 TorchConfig/libtorch.so
  if [ -n "${LIBTORCH_URL:-}" ]; then
    lt_tag=$(printf '%s' "${LIBTORCH_URL}" | sed -n 's/.*\(cu[0-9][0-9][0-9]\).*/\1/p' | head -n1)
  fi
  if [ -z "${lt_tag}" ] && [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ]; then
    lt_tag=$(grep -oE 'cu[0-9]{3}' "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" | head -n1 || true)
  fi
  if [ -z "${lt_tag}" ] && [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ] && command -v strings >/dev/null 2>&1; then
    lt_tag=$(strings "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" 2>/dev/null | grep -oE 'cu[0-9]{3}' | head -n1 || true)
  fi

  # CPU LibTorch 不参与 CUDA 栈一致性检查
  if [ -z "${lt_tag}" ]; then
    echo "[INFO] CUDA 栈一致性检查：当前 LibTorch 未识别到 cuXXX（可能为 CPU 版），跳过 nvcc/libnvrtc 对齐校验"
    return 0
  fi

  case "${lt_tag}" in
    cu121) lt_mm="12.1" ;;
    cu124) lt_mm="12.4" ;;
    cu126) lt_mm="12.6" ;;
    cu128) lt_mm="12.8" ;;
    *)
      echo "[WARN] CUDA 栈一致性检查：无法识别 LibTorch CUDA 变体 ${lt_tag}，跳过强一致校验"
      return 0
      ;;
  esac

  # 2) nvcc 版本
  local nvcc_bin=""
  if command -v nvcc >/dev/null 2>&1; then
    nvcc_bin=$(command -v nvcc)
  elif [ -n "${CUDA_HOME:-}" ] && [ -x "${CUDA_HOME}/bin/nvcc" ]; then
    nvcc_bin="${CUDA_HOME}/bin/nvcc"
  fi
  if [ -n "${nvcc_bin}" ]; then
    nvcc_mm=$("${nvcc_bin}" --version 2>/dev/null | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p' | head -n1)
  fi

  # 3) libnvrtc 版本（通过官方 NVRTC API nvrtcVersion 读取，避免仅看文件名）
  if command -v python3 >/dev/null 2>&1; then
    nvrtc_mm=$(python3 - <<'PY'
import ctypes
import sys

libs = ("libnvrtc.so", "libnvrtc.so.13", "libnvrtc.so.12", "libnvrtc.so.11")
for name in libs:
    try:
        lib = ctypes.CDLL(name)
        major = ctypes.c_int()
        minor = ctypes.c_int()
        fn = lib.nvrtcVersion
        fn.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        fn.restype = ctypes.c_int
        rc = fn(ctypes.byref(major), ctypes.byref(minor))
        if rc == 0:
            print(f"{major.value}.{minor.value}")
            raise SystemExit(0)
    except Exception:
        continue
sys.exit(1)
PY
) || true
  fi

  if [ -z "${nvcc_mm}" ]; then
    echo "[ERROR] CUDA 栈一致性检查失败：未找到 nvcc，无法确认与 LibTorch(${lt_tag}) 的工具链版本一致性。" >&2
    echo "[ERROR] 请安装与 LibTorch 对齐的 CUDA toolkit（期望 ${lt_mm}），或切换为 CPU LibTorch。" >&2
    return 1
  fi
  if [ -z "${nvrtc_mm}" ]; then
    echo "[ERROR] CUDA 栈一致性检查失败：未找到可用 libnvrtc（nvrtcVersion 读取失败）。" >&2
    echo "[ERROR] 请确保 LD_LIBRARY_PATH/系统库中存在与 LibTorch(${lt_tag}) 对齐的 libnvrtc（期望 ${lt_mm}）。" >&2
    return 1
  fi

  if [ "${nvcc_mm}" != "${lt_mm}" ]; then
    echo "[ERROR] CUDA 栈版本不一致：LibTorch=${lt_tag}(${lt_mm})，nvcc=${nvcc_mm}。" >&2
    echo "[ERROR] 必须统一为同一 CUDA 小版本，避免 JIT/NVRTC 运行期崩溃。" >&2
    return 1
  fi
  if [ "${nvrtc_mm}" != "${lt_mm}" ]; then
    echo "[ERROR] CUDA 栈版本不一致：LibTorch=${lt_tag}(${lt_mm})，libnvrtc=${nvrtc_mm}。" >&2
    echo "[ERROR] 必须统一为同一 CUDA 小版本，避免 nvrtc: invalid --gpu-architecture。" >&2
    return 1
  fi

  # 4) Blackwell 额外门控：libnvrtc 必须至少 12.8（支持 sm_120 目标架构）
  if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)
    if echo "${gpu_name}" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
      case "${nvrtc_mm}" in
        12.8|12.9|13.*) ;;
        *)
          echo "[ERROR] Blackwell GPU（${gpu_name}）要求 libnvrtc >= 12.8；当前 libnvrtc=${nvrtc_mm}。" >&2
          echo "[ERROR] 请升级 CUDA 运行时并与 LibTorch/nvcc 保持同版。" >&2
          return 1
          ;;
      esac
    fi
  fi

  echo "[INFO] CUDA 栈一致性校验通过：LibTorch=${lt_tag} nvcc=${nvcc_mm} libnvrtc=${nvrtc_mm}"
  return 0
}

# 在系统存在多套 CUDA 时，优先切换到与当前 LibTorch 版本匹配的工具链（nvcc + libnvrtc）。
# 目标：避免 nvcc/libnvrtc 与 LibTorch cuXXX 不一致导致 NVRTC 运行期失败。
automap_try_align_cuda_toolchain_with_libtorch() {
  local lt_tag=""
  local lt_mm=""
  local cur_nvcc_mm=""
  local c p v

  if [ -n "${LIBTORCH_URL:-}" ]; then
    lt_tag=$(printf '%s' "${LIBTORCH_URL}" | sed -n 's/.*\(cu[0-9][0-9][0-9]\).*/\1/p' | head -n1)
  fi
  if [ -z "${lt_tag}" ] && [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ]; then
    lt_tag=$(grep -oE 'cu[0-9]{3}' "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" | head -n1 || true)
  fi
  if [ -z "${lt_tag}" ] && [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ] && command -v strings >/dev/null 2>&1; then
    lt_tag=$(strings "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" 2>/dev/null | grep -oE 'cu[0-9]{3}' | head -n1 || true)
  fi
  [ -z "${lt_tag}" ] && return 0

  case "${lt_tag}" in
    cu121) lt_mm="12.1" ;;
    cu124) lt_mm="12.4" ;;
    cu126) lt_mm="12.6" ;;
    cu128) lt_mm="12.8" ;;
    *) return 0 ;;
  esac

  if command -v nvcc >/dev/null 2>&1; then
    cur_nvcc_mm=$(nvcc --version 2>/dev/null | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p' | head -n1)
  fi
  if [ -n "${cur_nvcc_mm}" ] && [ "${cur_nvcc_mm}" = "${lt_mm}" ]; then
    return 0
  fi

  # 常见安装路径优先；仅在版本完全匹配时切换
  for c in "/usr/local/cuda-${lt_mm}" "/usr/local/cuda-${lt_mm%.*}" "/usr/local/cuda" "/usr/lib/cuda"; do
    [ -x "${c}/bin/nvcc" ] || continue
    v=$("${c}/bin/nvcc" --version 2>/dev/null | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p' | head -n1)
    [ "${v}" = "${lt_mm}" ] || continue
    if [ -f "${c}/lib64/libnvrtc.so" ] || [ -f "${c}/targets/x86_64-linux/lib/libnvrtc.so" ]; then
      export CUDA_HOME="${c}"
      export PATH="${CUDA_HOME}/bin:${PATH}"
      if [ -d "${CUDA_HOME}/lib64" ]; then
        export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"
      fi
      if [ -d "${CUDA_HOME}/targets/x86_64-linux/lib" ]; then
        export LD_LIBRARY_PATH="${CUDA_HOME}/targets/x86_64-linux/lib:${LD_LIBRARY_PATH}"
      fi
      echo "[INFO] 已切换 CUDA 工具链以匹配 LibTorch(${lt_tag}): CUDA_HOME=${CUDA_HOME} nvcc=${v}"
      return 0
    fi
  done

  echo "[WARN] 未找到与 LibTorch(${lt_tag}) 匹配的本地 CUDA 工具链（期望 ${lt_mm}）。若后续 NVRTC 报错，请安装对应 CUDA toolkit 并设置 CUDA_HOME。" >&2
  return 0
}

# LibTorch（OverlapTransformer 推理）：与 GTSAM 一致，安装到 install_deps，仅下载解压一次，不随工程重复编译
LIBTORCH_INSTALL_DIR="${INSTALL_DEPS}/libtorch"
LIBTORCH_URL_RECORD_FILE="${INSTALL_DEPS}/.libtorch_source_url"
NEED_LIBTORCH_DOWNLOAD=false
if [ ! -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] && [ ! -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; then
  NEED_LIBTORCH_DOWNLOAD=true
fi
if [ "${LIBTORCH_SKIP_DOWNLOAD:-0}" = "1" ]; then
  NEED_LIBTORCH_DOWNLOAD=false
  echo "[INFO] LIBTORCH_SKIP_DOWNLOAD=1，跳过 LibTorch 下载（请确保已手动放置到 ${LIBTORCH_INSTALL_DIR}）"
fi
# 目标 URL 变化时：删除本地旧版本后重下，并把新 URL 落盘，下次直接复用
if [ "${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" != "1" ] && [ "${LIBTORCH_SKIP_DOWNLOAD:-0}" != "1" ]; then
  _lt_url_from_env=0
  [ -n "${LIBTORCH_URL:-}" ] && _lt_url_from_env=1
  if [ -z "${LIBTORCH_URL:-}" ]; then
    LIBTORCH_URL="$(automap_resolve_libtorch_url)"
  fi
  if [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] || [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; then
    _lt_recorded_url=""
    if [ -f "${LIBTORCH_URL_RECORD_FILE}" ]; then
      _lt_recorded_url="$(tr -d '\r\n' < "${LIBTORCH_URL_RECORD_FILE}")"
    fi
    if [ -n "${_lt_recorded_url}" ] && [ "${_lt_recorded_url}" != "${LIBTORCH_URL}" ]; then
      echo "[INFO] 检测到 LibTorch 目标版本变化（旧 URL: ${_lt_recorded_url}；新 URL: ${LIBTORCH_URL}），将删除本地旧版本并重新下载"
      rm -rf "${LIBTORCH_INSTALL_DIR}"
      NEED_LIBTORCH_DOWNLOAD=true
    elif [ -z "${_lt_recorded_url}" ] && [ "${_lt_url_from_env}" = "1" ]; then
      echo "[INFO] 检测到显式 LIBTORCH_URL 且本地无 URL 记录，为避免复用旧版本将删除本地 LibTorch 并重新下载"
      rm -rf "${LIBTORCH_INSTALL_DIR}"
      NEED_LIBTORCH_DOWNLOAD=true
    fi
  fi
fi
# Blackwell + 已存在的非 cu128 LibTorch（曾由 apt nvcc 12.0 误选 cu121）：删除并强制重下 cu128
if [ "${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" != "1" ] && \
   [ "${LIBTORCH_SKIP_DOWNLOAD:-0}" != "1" ] && \
   { [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] || [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; }; then
  if command -v nvidia-smi >/dev/null 2>&1; then
    _lt_bw_gpu=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)
    if echo "${_lt_bw_gpu}" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
      _lt_has_cu128=0
      if [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] && grep -qE 'cu128|2\.7\.0' "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" 2>/dev/null; then
        _lt_has_cu128=1
      fi
      if [ "${_lt_has_cu128}" -eq 0 ] && [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ] && command -v strings >/dev/null 2>&1; then
        strings "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" 2>/dev/null | grep -q 'cu128' && _lt_has_cu128=1 || true
      fi
      if [ "${_lt_has_cu128}" -eq 0 ]; then
        echo "[INFO] Blackwell GPU：install_deps/libtorch 非 cu128（可能为误选 cu121）；将删除并重新下载 cu128 LibTorch"
        rm -rf "${LIBTORCH_INSTALL_DIR}"
        NEED_LIBTORCH_DOWNLOAD=true
      fi
    fi
  fi
fi
if [ "$NEED_LIBTORCH_DOWNLOAD" = true ]; then
  echo '========================================'
  echo '安装 LibTorch 到 install_deps（下载预编译包，仅执行一次）'
  echo '========================================'
  automap_log_progress "LibTorch：准备下载或从缓存复制（大文件时请等待）…"
  if [ -z "${LIBTORCH_URL:-}" ]; then
    echo "[INFO] LibTorch URL（自动解析，可用 LIBTORCH_URL / LIBTORCH_PREFER=cpu 覆盖）: ${LIBTORCH_URL}"
  else
    echo "[INFO] LibTorch URL（来自环境变量 LIBTORCH_URL）: ${LIBTORCH_URL}"
  fi
  LIBTORCH_ZIP="/tmp/libtorch-download.zip"
  mkdir -p "${INSTALL_DEPS}"
  if [ -d "${LIBTORCH_INSTALL_DIR}" ]; then
    rm -rf "${LIBTORCH_INSTALL_DIR}"
  fi
  _LT_CACHE_ROOT="${AUTOMAP_DOWNLOAD_CACHE:-/root/automap_download_cache}/libtorch"
  mkdir -p "${_LT_CACHE_ROOT}"
  _LT_KEY=$(printf '%s' "${LIBTORCH_URL}" | sha256sum | awk '{print $1}')
  _LT_CACHED="${_LT_CACHE_ROOT}/${_LT_KEY}.zip"
  if [ -f "${_LT_CACHED}" ] && [ -s "${_LT_CACHED}" ]; then
    echo "[INFO] LibTorch 使用本地缓存（跳过网络）: ${_LT_CACHED}"
    cp -f "${_LT_CACHED}" "${LIBTORCH_ZIP}"
  else
    if ! command -v wget >/dev/null 2>&1 && ! command -v curl >/dev/null 2>&1; then
      echo "[ERROR] 需要 wget 或 curl 以下载 LibTorch"; exit 1
    fi
    echo "[INFO] 下载 LibTorch（成功后写入缓存）: ${LIBTORCH_URL}"
    _LT_OFF_BASE="https://download.pytorch.org/libtorch"
    _LT_MIR_BASE="${AUTOMAP_LIBTORCH_DOWNLOAD_BASE:-${_LT_OFF_BASE}}"
    rm -f "${LIBTORCH_ZIP}"
    if ! automap_download_to_file "${LIBTORCH_URL}" "${LIBTORCH_ZIP}"; then
      rm -f "${LIBTORCH_ZIP}"
      _LT_FALLBACK=""
      if [ "${AUTOMAP_LIBTORCH_NO_OFFICIAL_FALLBACK:-0}" != "1" ] && [ -n "${LIBTORCH_URL}" ] \
          && printf '%s' "${LIBTORCH_URL}" | grep -qF "${_LT_MIR_BASE}"; then
        _LT_FALLBACK="${LIBTORCH_URL//${_LT_MIR_BASE}/${_LT_OFF_BASE}}"
      fi
      if [ -n "${_LT_FALLBACK}" ] && [ "${_LT_FALLBACK}" != "${LIBTORCH_URL}" ]; then
        echo "[WARN] LibTorch 镜像下载失败，尝试官方源（设 AUTOMAP_LIBTORCH_NO_OFFICIAL_FALLBACK=1 可禁用）: ${_LT_FALLBACK}" >&2
        if automap_download_to_file "${_LT_FALLBACK}" "${LIBTORCH_ZIP}"; then
          LIBTORCH_URL="${_LT_FALLBACK}"
          _LT_KEY=$(printf '%s' "${LIBTORCH_URL}" | sha256sum | awk '{print $1}')
          _LT_CACHED="${_LT_CACHE_ROOT}/${_LT_KEY}.zip"
        else
          rm -f "${LIBTORCH_ZIP}"
        fi
      fi
      if [ ! -f "${LIBTORCH_ZIP}" ] || [ ! -s "${LIBTORCH_ZIP}" ]; then
        echo "[ERROR] LibTorch 下载失败（含 wget/curl 重试；若 URL 为镜像站则已尝试官方源）。可：① LIBTORCH_URL=可访问的 zip；② 纯 CPU 设 LIBTORCH_PREFER=cpu；③ 手动解压到 ${LIBTORCH_INSTALL_DIR} 后 LIBTORCH_SKIP_DOWNLOAD=1；④ 预置缓存 ${_LT_CACHE_ROOT}/<sha256>.zip" >&2
        exit 1
      fi
    fi
    cp -f "${LIBTORCH_ZIP}" "${_LT_CACHED}"
  fi
  automap_log_progress "LibTorch：解压 zip 到 install_deps…"
  echo "[INFO] 解压到 ${LIBTORCH_INSTALL_DIR}"
  if ! command -v unzip >/dev/null 2>&1; then
    if command -v apt-get >/dev/null 2>&1; then
      echo "[WARN] 未检测到 unzip，正在 apt-get install unzip…" >&2
      DEBIAN_FRONTEND=noninteractive apt-get update -qq 2>/dev/null || true
      DEBIAN_FRONTEND=noninteractive apt-get install -y -qq unzip || {
        echo "[ERROR] 无法安装 unzip，无法解压 LibTorch" >&2
        exit 1
      }
    else
      echo "[ERROR] 需要 unzip 以解压 LibTorch（最小镜像请预装 unzip 或手动解压到 ${LIBTORCH_INSTALL_DIR}）" >&2
      exit 1
    fi
  fi
  unzip -q -o "${LIBTORCH_ZIP}" -d "${INSTALL_DEPS}"
  rm -f "${LIBTORCH_ZIP}"
  if [ -d "${INSTALL_DEPS}/libtorch" ]; then
    printf '%s\n' "${LIBTORCH_URL}" > "${LIBTORCH_URL_RECORD_FILE}"
    echo "[INFO] LibTorch 已安装于 install_deps/libtorch"
  else
    echo "[ERROR] 解压后未找到 ${INSTALL_DEPS}/libtorch"; exit 1
  fi
else
  if [ -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] || [ -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; then
    if [ -n "${LIBTORCH_URL:-}" ]; then
      printf '%s\n' "${LIBTORCH_URL}" > "${LIBTORCH_URL_RECORD_FILE}"
    fi
    echo "[INFO] LibTorch 已安装于 install_deps，跳过下载与解压（删除 ${LIBTORCH_INSTALL_DIR} 后才会重新拉取）"
  fi
fi
if [ "${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" = "1" ]; then
  if [ ! -f "${LIBTORCH_INSTALL_DIR}/share/cmake/Torch/TorchConfig.cmake" ] && [ ! -f "${LIBTORCH_INSTALL_DIR}/lib/libtorch.so" ]; then
    echo "[ERROR] AUTOMAP_PREBUILT_INSTALL_DEPS=1 但 ${LIBTORCH_INSTALL_DIR} 无有效 LibTorch（需 TorchConfig.cmake 或 lib/libtorch.so）" >&2
    exit 1
  fi
fi
if [ -d "${LIBTORCH_INSTALL_DIR}/lib" ]; then
  export LIBTORCH_HOME="${LIBTORCH_INSTALL_DIR}"
  export CMAKE_PREFIX_PATH="${LIBTORCH_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${LIBTORCH_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
fi

automap_ensure_cuda_toolkit_for_libtorch_cmake
automap_try_align_cuda_toolchain_with_libtorch

automap_verify_blackwell_accel_stack || exit 1
automap_verify_cuda_stack_unified || exit 1

# 快速修复模式：仅修 CUDA/LibTorch 栈并做一致性校验，不进入 ONNX/colcon 全量编译。
# 用途：run_automap 运行前 NVRTC 预检失败时，先快速恢复 GPU 推理能力，减少恢复时间。
if [ "${AUTOMAP_REPAIR_CUDA_STACK_ONLY:-0}" = "1" ]; then
  echo "[INFO] AUTOMAP_REPAIR_CUDA_STACK_ONLY=1：CUDA/LibTorch 快速修复完成，跳过 ONNX 与全量编译。"
  exit 0
fi

# ONNX Runtime（SLOAM 语义分割）：与 GTSAM 一致，安装到 install_deps，首次编译后后续跳过
ONNXRUNTIME_INSTALL_DIR="${INSTALL_DEPS}/onnxruntime"
NEED_ONNXRUNTIME_BUILD=false
if [ ! -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so" ]; then
  NEED_ONNXRUNTIME_BUILD=true
fi
if [ "${ONNXRUNTIME_SKIP_BUILD:-0}" = "1" ]; then
  NEED_ONNXRUNTIME_BUILD=false
  echo "[INFO] ONNXRUNTIME_SKIP_BUILD=1，跳过 ONNX Runtime 源码编译（须已有 install_deps/onnxruntime 或 ONNXRUNTIME_HOME/系统路径中的 libonnxruntime.so，否则 CMake 将失败）"
fi
# 仅 CPU 的 ORT 已存在但请求 CUDA EP：若能解析 cuDNN 则强制重编（否则「已安装」会跳过，永远不生成 providers_cuda）
if [ "${ONNXRUNTIME_USE_CUDA:-0}" = "1" ] && [ "${ONNXRUNTIME_SKIP_BUILD:-0}" != "1" ] && \
   [ -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so" ] && \
   [ ! -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime_providers_cuda.so" ]; then
  if automap_resolve_ort_cuda_cudnn_paths; then
    NEED_ONNXRUNTIME_BUILD=true
    echo "[INFO] ONNXRUNTIME_USE_CUDA=1：install_deps 仅有 CPU 版 ORT，将重新编译以生成 libonnxruntime_providers_cuda.so"
    rm -rf /root/automap_ws/src_onnxruntime/build 2>/dev/null || true
    echo "[INFO] ONNX Runtime: 已清理 src_onnxruntime/build（避免 CMake 缓存仍为 CPU-only）"
  fi
fi
if [ "$NEED_ONNXRUNTIME_BUILD" = true ]; then
  echo '========================================'
  echo '编译 ONNX Runtime 到 install_deps（首次执行，后续跳过）'
  echo '========================================'
  automap_log_progress "ONNX Runtime：准备源码（git clone 或缓存拷贝可能较久）…"
  ONNX_SRC_DIR="/root/automap_ws/src_onnxruntime"
  ONNX_DEPS_MOUNTED="/root/mapping/docker/deps/onnxruntime"
  mkdir -p "${INSTALL_DEPS}"
  if [ ! -d "${ONNX_SRC_DIR}" ] || [ ! -f "${ONNX_SRC_DIR}/build.sh" ]; then
    if [ -d "${ONNX_DEPS_MOUNTED}" ] && [ -f "${ONNX_DEPS_MOUNTED}/build.sh" ]; then
      echo "[INFO] 使用挂载的 ONNX Runtime 源码: ${ONNX_DEPS_MOUNTED}（无需 git clone）"
      rm -rf "${ONNX_SRC_DIR}"
      cp -a "${ONNX_DEPS_MOUNTED}" "${ONNX_SRC_DIR}"
    else
      _ONNX_GIT_CACHE="${AUTOMAP_DOWNLOAD_CACHE:-/root/automap_download_cache}/git/onnxruntime_v1_8_2"
      if [ -f "${_ONNX_GIT_CACHE}/build.sh" ]; then
        echo "[INFO] ONNX Runtime 使用本地 git 缓存（跳过 clone）: ${_ONNX_GIT_CACHE}"
        rm -rf "${ONNX_SRC_DIR}"
        cp -a "${_ONNX_GIT_CACHE}" "${ONNX_SRC_DIR}"
      else
        _ONNX_GIT="${AUTOMAP_ONNXRUNTIME_GIT_URL:-https://ghfast.top/https://github.com/microsoft/onnxruntime.git}"
        echo "[INFO] 克隆 ONNX Runtime v1.8.2（含 submodule），源: ${_ONNX_GIT}"
        git clone --depth 1 --branch v1.8.2 --recursive "${_ONNX_GIT}" "${ONNX_SRC_DIR}" || { echo "[ERROR] git clone onnxruntime 失败"; exit 1; }
        mkdir -p "$(dirname "${_ONNX_GIT_CACHE}")"
        rm -rf "${_ONNX_GIT_CACHE}"
        cp -a "${ONNX_SRC_DIR}" "${_ONNX_GIT_CACHE}"
        echo "[INFO] ONNX Runtime 源码已缓存至 ${_ONNX_GIT_CACHE}（下次构建免 clone）"
      fi
    fi
  fi
  : > "${ONNX_SRC_DIR}/COLCON_IGNORE"
  cd "${ONNX_SRC_DIR}"
  # ORT v1.8.2：① flake8 作为 pep8_check 的 ALL 依赖时，Noble/Jazzy 会对 tools/ 下脚本失败并中断 gmake。
  # ② bundled flatbuffers 在 GNU 分支强制 -Werror；GCC 13 + libstdc++ 对 reflection.cpp 误报 stringop-overflow（CXXFLAGS 无法覆盖子目录 set）。
  _ORT_RECONF=false
  _ORT_FLAKE8="${ONNX_SRC_DIR}/cmake/flake8.cmake"
  if [ -f "${_ORT_FLAKE8}" ] && grep -A3 'add_custom_target(pep8_check' "${_ORT_FLAKE8}" | grep -q '^[[:space:]]*ALL[[:space:]]*$'; then
    python3 -c "
import re, sys
p = sys.argv[1]
t = open(p, encoding='utf-8').read()
n = re.sub(
    r'(add_custom_target\(pep8_check\s*\n)\s*ALL\s*\n',
    r'\1',
    t,
    count=1,
)
if n != t:
    open(p, 'w', encoding='utf-8').write(n)
    print('[INFO] ONNX Runtime: 已从 pep8_check 移除 ALL（避免 flake8 阻断编译）')
" "${_ORT_FLAKE8}" || true
    _ORT_RECONF=true
  fi
  _ORT_FB_CMAKE="${ONNX_SRC_DIR}/cmake/external/flatbuffers/CMakeLists.txt"
  if [ -f "${_ORT_FB_CMAKE}" ] && ! grep -q 'Wno-error=stringop-overflow' "${_ORT_FB_CMAKE}"; then
    if python3 - "${_ORT_FB_CMAKE}" <<'PY'
import sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
needle = (
    "  set(CMAKE_CXX_FLAGS\n"
    '    "${CMAKE_CXX_FLAGS} -fsigned-char")\n'
    "\n"
    'elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")'
)
if needle not in text:
    print("[WARN] ONNX flatbuffers CMakeLists.txt 与 v1.8.2 预期片段不一致，未注入 stringop 抑制（若 GCC13 仍失败请删 src_onnxruntime 重拉）", file=sys.stderr)
    raise SystemExit(1)
repl = (
    "  set(CMAKE_CXX_FLAGS\n"
    '    "${CMAKE_CXX_FLAGS} -fsigned-char")\n'
    "  # automap_pro: GCC 12+ -Wstringop-overflow false positive in reflection.cpp under -Werror\n"
    "  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12)\n"
    '    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=stringop-overflow -Wno-stringop-overflow")\n'
    "  endif()\n"
    "\n"
    'elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")'
)
open(path, "w", encoding="utf-8").write(text.replace(needle, repl, 1))
print("[INFO] ONNX Runtime flatbuffers: 已注入 GCC12+ stringop-overflow 抑制（与仓库 thrid_party 补丁一致）")
PY
    then
      _ORT_RECONF=true
    fi
  fi
  if [ "${_ORT_RECONF}" = true ] && [ -d "${ONNX_SRC_DIR}/build" ]; then
    echo "[INFO] ONNX Runtime: 清理 build/（源码已打补丁，需重新 CMake 配置）…"
    rm -rf "${ONNX_SRC_DIR}/build"
  fi
  export CXXFLAGS="${CXXFLAGS:+$CXXFLAGS }-Wno-error=stringop-overflow -Wno-stringop-overflow"
  automap_log_progress "ONNX Runtime：运行 build.sh（首次可能 10–30+ 分钟，请耐心等待）…"
  # 可选 GPU 版 ORT（SLOAM 语义 AUTOMAP_SLOAM_ONNX_CUDA=1 需要 libonnxruntime_providers_cuda.so）
  ONNX_ORT_EXTRA_ARGS=()
  _ORT_INVOKED_CUDA=0
  if [ "${ONNXRUNTIME_USE_CUDA:-0}" = "1" ]; then
    if automap_resolve_ort_cuda_cudnn_paths; then
      export CUDA_HOME="${_ORT_CUDA}"
      export CUDNN_HOME="${_ORT_CUDNN}"
      ONNX_ORT_EXTRA_ARGS=(--use_cuda --cuda_home="${_ORT_CUDA}" --cudnn_home="${_ORT_CUDNN}")
      _ORT_INVOKED_CUDA=1
      echo "[INFO] ONNX Runtime 将使用 CUDA 构建: cuda_home=${_ORT_CUDA} cudnn_home=${_ORT_CUDNN}"
    else
      echo "[WARN] ONNXRUNTIME_USE_CUDA=1 但未解析到可用的 CUDA/cuDNN 路径（需 CUDA_HOME 目录存在，且 cudnn_version.h 位于 CUDNN_HOME/include 或通过多架构链接可用）→ 仍用 CPU 构建 ORT"
    fi
  fi
  # --skip_tests 只跳过「执行」测试；默认仍会编译 onnxruntime_test_all。GCC 13 在 loop_test 等会 -Werror=maybe-uninitialized。
  _ORT_CMAKECACHE="${ONNX_SRC_DIR}/build/Linux/RelWithDebInfo/CMakeCache.txt"
  if [ -f "${_ORT_CMAKECACHE}" ] && grep -q '^onnxruntime_BUILD_UNIT_TESTS:BOOL=ON' "${_ORT_CMAKECACHE}"; then
    echo "[INFO] ONNX Runtime: 清理 build/（将关闭单元测试编译，避免旧 CMake 缓存仍为 ON）…"
    rm -rf "${ONNX_SRC_DIR}/build"
  fi
  ./build.sh \
    --config RelWithDebInfo \
    --build_shared_lib \
    --skip_tests \
    --parallel "${PARALLEL_JOBS}" \
    "${ONNX_ORT_EXTRA_ARGS[@]}" \
    --cmake_extra_defines \
      "CMAKE_INSTALL_PREFIX=${ONNXRUNTIME_INSTALL_DIR}" \
      "onnxruntime_BUILD_UNIT_TESTS=OFF"
  automap_log_progress "ONNX Runtime：cmake --build install…"
  cd build/Linux/RelWithDebInfo
  cmake --build . --target install
  cd /root/automap_ws
  echo "[INFO] ONNX Runtime 已安装于 install_deps/onnxruntime"
  if [ "${ONNXRUNTIME_USE_CUDA:-0}" = "1" ]; then
    if [ -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime_providers_cuda.so" ]; then
      echo "[INFO] ONNX Runtime CUDA Execution Provider: libonnxruntime_providers_cuda.so 已安装"
    elif [ "${_ORT_INVOKED_CUDA:-0}" != "1" ]; then
      echo "[INFO] ONNXRUNTIME_USE_CUDA=1 但本次未传入 --use_cuda（未检测到可用 cuDNN 开发头/CUDNN_HOME）；ORT 为 CPU 构建，SLOAM 使用 CPU EP。安装 libcudnn*-dev 或设置 CUDNN_HOME 并清理 ONNX 构建目录后重编可启用 GPU EP"
    else
      echo "[WARN] ONNXRUNTIME_USE_CUDA=1 且已尝试 CUDA 构建，但未见 libonnxruntime_providers_cuda.so；请检查 CUDA/cuDNN 与 ORT 构建日志，SLOAM 将无法使用 GPU EP"
    fi
  fi
else
  if [ -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime.so" ]; then
    echo "[INFO] ONNX Runtime 已安装于 install_deps，跳过"
    if [ "${ONNXRUNTIME_USE_CUDA:-0}" = "1" ] && [ ! -f "${ONNXRUNTIME_INSTALL_DIR}/lib/libonnxruntime_providers_cuda.so" ]; then
      echo "[INFO] ONNXRUNTIME_USE_CUDA=1 但无 libonnxruntime_providers_cuda.so（本次未重编 ORT）。若已安装 cuDNN 开发头，删除 ${ONNXRUNTIME_INSTALL_DIR} 与 src_onnxruntime/build 后重编以启用 CUDA EP"
    fi
  fi
fi
if [ -d "${ONNXRUNTIME_INSTALL_DIR}/lib" ]; then
  export ONNXRUNTIME_HOME="${ONNXRUNTIME_INSTALL_DIR}"
  export LD_LIBRARY_PATH="${ONNXRUNTIME_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
fi

# LSK3DNet hybrid：独立 Python venv（torch/spconv/torch-scatter 与 CUDA 对齐，避免官方 requirements 的 cu113 冲突）
LSK_VENV_DIR="${INSTALL_DEPS}/lsk3dnet_venv"
if [ "${AUTOMAP_SETUP_LSK3DNET_VENV:-0}" = "1" ]; then
  if [ -f /root/scripts/setup_lsk3dnet_venv.sh ]; then
    chmod +x /root/scripts/setup_lsk3dnet_venv.sh 2>/dev/null || true
    automap_log_progress "LSK3DNet venv：pip 安装 torch/spconv 等（可能 5–20 分钟）…"
    LSK_SKIP_IF_OK=1 bash /root/scripts/setup_lsk3dnet_venv.sh "${LSK_VENV_DIR}" || {
      echo "[ERROR] LSK3DNet venv 安装失败（检查网络、CUDA/cuDNN、PyG 轮是否与 LSK_TORCH_VER 一致）" >&2
      exit 1
    }
  else
    echo "[WARN] 未挂载 scripts/setup_lsk3dnet_venv.sh，跳过 AUTOMAP_SETUP_LSK3DNET_VENV"
  fi
fi

# 后续 fast_livo / automap_pro 需要找到 gtsam、ceres、teaserpp、vikit、hba、fast_livo、libtorch、onnxruntime
[ -f "${INSTALL_DEPS}/setup.bash" ] && source "${INSTALL_DEPS}/setup.bash"
# vikit / hba / fast_livo 安装到 install_deps 子目录（与 GTSAM 一致），需显式加入路径
_VIKIT_PREFIX=""
_HBA_PREFIX=""
_FAST_LIVO_PREFIX=""
_CERES_PREFIX=""
# 以 lib 或 ament 的 vikit_commonConfig 判定前缀（仅 lib 时某些布局仍缺 share，下面 fast_livo 会再 source setup.bash）
if [ -d "${VIKIT_INSTALL_DIR}/lib" ] 2>/dev/null || \
    [ -f "${VIKIT_INSTALL_DIR}/share/vikit_common/cmake/vikit_commonConfig.cmake" ] 2>/dev/null; then
  _VIKIT_PREFIX="${VIKIT_INSTALL_DIR}"
fi
[ -d "${HBA_INSTALL_DIR}/lib" ] 2>/dev/null && _HBA_PREFIX="${HBA_INSTALL_DIR}"
[ -d "${INSTALL_DEPS}/fast_livo/lib" ] 2>/dev/null && _FAST_LIVO_PREFIX="${INSTALL_DEPS}/fast_livo"
if [ -f "${INSTALL_DEPS}/ceres/lib/cmake/Ceres/CeresConfig.cmake" ] || [ -f "${INSTALL_DEPS}/ceres/lib64/cmake/Ceres/CeresConfig.cmake" ]; then
  _CERES_PREFIX="${INSTALL_DEPS}/ceres"
fi
export CMAKE_PREFIX_PATH="${INSTALL_DEPS}/gtsam:${_CERES_PREFIX:+${_CERES_PREFIX}:}${INSTALL_DEPS}/teaserpp:${_VIKIT_PREFIX:+${_VIKIT_PREFIX}:}${_HBA_PREFIX:+${_HBA_PREFIX}:}${_FAST_LIVO_PREFIX:+${_FAST_LIVO_PREFIX}:}${INSTALL_DEPS}/libtorch:${INSTALL_DEPS}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${INSTALL_DEPS}/gtsam/lib:${INSTALL_DEPS}/teaserpp/lib:${_VIKIT_PREFIX:+${_VIKIT_PREFIX}/lib:}${_HBA_PREFIX:+${_HBA_PREFIX}/lib:}${_FAST_LIVO_PREFIX:+${_FAST_LIVO_PREFIX}/lib:}${INSTALL_DEPS}/libtorch/lib:${INSTALL_DEPS}/onnxruntime/lib:${LD_LIBRARY_PATH}"
# Ceres：与源码安装布局一致，同时 prepend lib 与 lib64（与 run_automap.sh 运行时注入一致）
if [ -n "${_CERES_PREFIX}" ]; then
  [ -d "${_CERES_PREFIX}/lib64" ] && export LD_LIBRARY_PATH="${_CERES_PREFIX}/lib64:${LD_LIBRARY_PATH}"
  [ -d "${_CERES_PREFIX}/lib" ] && export LD_LIBRARY_PATH="${_CERES_PREFIX}/lib:${LD_LIBRARY_PATH}"
fi

# fast_livo：与 GTSAM/vikit/hba 一致，安装到 install_deps/fast_livo，已安装则跳过
FAST_LIVO_INSTALL_DIR="${INSTALL_DEPS}/fast_livo"
FAST_LIVO_PATH=""
[ -d src/automap_pro/src/modular/fast-livo2-humble ] && FAST_LIVO_PATH="src/automap_pro/src/modular/fast-livo2-humble"
[ -z "${FAST_LIVO_PATH}" ] && [ -d src/fast_livo ] && FAST_LIVO_PATH="src/fast_livo"
echo "[INFO] fast_livo 源码检查: in-tree=$([ -d src/automap_pro/src/modular/fast-livo2-humble ] && echo 存在 || echo 不存在), src/fast_livo=$([ -d src/fast_livo ] && echo 存在 || echo 不存在) → FAST_LIVO_PATH=${FAST_LIVO_PATH:-未设置}"
if [ -n "${FAST_LIVO_PATH}" ]; then
  NEED_FAST_LIVO_BUILD=true
  # 已安装则跳过（colcon 产出 setup.bash，--clean 不删 install_deps）
  if [ -f "${FAST_LIVO_INSTALL_DIR}/setup.bash" ] || [ -d "${FAST_LIVO_INSTALL_DIR}/fast_livo" ] || [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] || [ -d "${FAST_LIVO_INSTALL_DIR}/share" ]; then
    NEED_FAST_LIVO_BUILD=false
  fi
  if [ "$NEED_FAST_LIVO_BUILD" = true ]; then
    echo '========================================'
    echo '编译 fast_livo（安装到 install_deps/fast_livo）'
    echo '========================================'
    automap_log_progress "colcon：fast_livo（耗时可能较长）"
    echo "[INFO] 使用路径: ${FAST_LIVO_PATH}（避免 symlink 导致 colcon 0 packages）"
    mkdir -p "${FAST_LIVO_INSTALL_DIR}"
    # find_package(vikit_common) 走 ament：仅 CMAKE_PREFIX_PATH 往往不够，需 source setup 以设置 AMENT_PREFIX_PATH
    _VIKIT_SETUP=""
    if [ -f "${INSTALL_DEPS}/vikit/setup.bash" ]; then
      _VIKIT_SETUP="${INSTALL_DEPS}/vikit/setup.bash"
    elif [ -n "${VIKIT_INSTALL_DIR:-}" ] && [ -f "${VIKIT_INSTALL_DIR}/setup.bash" ]; then
      _VIKIT_SETUP="${VIKIT_INSTALL_DIR}/setup.bash"
    fi
    if [ -n "${_VIKIT_SETUP}" ]; then
      echo "[INFO] source ${_VIKIT_SETUP}（供 fast_livo 解析 vikit_common / vikit_ros）"
      # shellcheck source=/dev/null
      source "${_VIKIT_SETUP}"
    else
      echo "[WARN] 未找到 vikit 的 setup.bash（期望 ${INSTALL_DEPS}/vikit/setup.bash），fast_livo 可能 CMake 找不到 vikit_common" >&2
    fi
    colcon build ${COLCON_PARALLEL} ${COLCON_EVENT_HANDLERS} --install-base "${FAST_LIVO_INSTALL_DIR}" --paths "${FAST_LIVO_PATH}" --cmake-args ${NINJA_CMAKE_ARG} -DCMAKE_BUILD_TYPE=Release
    _fl_ok=false
    [ -f "${FAST_LIVO_INSTALL_DIR}/setup.bash" ] && _fl_ok=true
    [ -f "${FAST_LIVO_INSTALL_DIR}/share/fast_livo/package.xml" ] && _fl_ok=true
    [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] && _fl_ok=true
    [ -d "${FAST_LIVO_INSTALL_DIR}/share" ] && _fl_ok=true
    [ -d "${FAST_LIVO_INSTALL_DIR}/fast_livo" ] && _fl_ok=true
    if [ "$_fl_ok" = false ]; then
      echo "[WARN] fast_livo 编译完成但未在预期路径找到产物，检查: ${FAST_LIVO_INSTALL_DIR}"
      ls -la "${FAST_LIVO_INSTALL_DIR}" 2>/dev/null || true
    fi
    echo "[INFO] fast_livo 已安装于 install_deps/fast_livo"
  else
    echo "[INFO] fast_livo 已安装于 install_deps，跳过"
  fi
  [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] && export CMAKE_PREFIX_PATH="${FAST_LIVO_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"
  [ -d "${FAST_LIVO_INSTALL_DIR}/lib" ] && export LD_LIBRARY_PATH="${FAST_LIVO_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}"
  mkdir -p /root/automap_ws/install
  ln -sfn "${FAST_LIVO_INSTALL_DIR}" /root/automap_ws/install/fast_livo 2>/dev/null || true
else
  echo '[WARN] 未找到 fast_livo 源码（src/automap_pro/src/modular/fast-livo2-humble 或 src/fast_livo），跳过；运行时会报 package fast_livo not found'
fi

# automap_pro

# ============================================================
# ScanContext 详细定位日志
# ============================================================
automap_log_progress "ScanContext：检查路径与符号链接（随后编译 automap_pro）"
echo "========================================"
echo "ScanContext 定位调试"
echo "========================================"

# 检查可能的源码位置
echo "[DEBUG-1] 检查 /root/automap_ws/src/ 目录结构:"
ls -la /root/automap_ws/src/ 2>&1 || echo "[DEBUG-1] /root/automap_ws/src/ 不存在"

echo ""
echo "[DEBUG-2] 检查 /root/mapping/ 目录结构 (如果存在):"
ls -la /root/mapping/ 2>&1 || echo "[DEBUG-2] /root/mapping/ 不存在"

echo ""
echo "[DEBUG-3] 在 /root/mapping 下搜索 scancontext_tro:"
find /root/mapping -maxdepth 3 -type d -name "scancontext*" 2>/dev/null || echo "[DEBUG-3] 未找到"

echo ""
echo "[DEBUG-4] 在 /root/automap_ws 下搜索 scancontext_tro:"
find /root/automap_ws -maxdepth 4 -type d -name "scancontext*" 2>/dev/null || echo "[DEBUG-4] 未找到"

echo ""
echo "[DEBUG-5] 检查 automap_pro 源码目录:"
if [ -d "/root/automap_ws/src/automap_pro" ]; then
    echo "[DEBUG-5] /root/automap_ws/src/automap_pro 存在"
    ls -la /root/automap_ws/src/automap_pro/ | head -20
    echo ""
    echo "[DEBUG-5b] 在 automap_pro 下搜索 scancontext:"
    find /root/automap_ws/src/automap_pro -maxdepth 5 -type d -name "scancontext*" 2>/dev/null || echo "未找到"
else
    echo "[DEBUG-5] /root/automap_ws/src/automap_pro 不存在"
fi

# 修复 ScanContext 头文件路径：在可写目录创建符号链接
# 因为容器内 automap_pro 是只读挂载，需要在 /root/automap_ws 下创建链接
SC_SOURCE=""
SC_TARGET="/root/automap_ws/automap_pro_thrid_party_scancontext"

# 尝试多个可能的源码位置
echo ""
echo "[DEBUG-6] 尝试确定 SC_SOURCE:"
if [ -d "/root/automap_ws/src/automap_pro/thrid_party/scancontext_tro" ]; then
    SC_SOURCE="/root/automap_ws/src/automap_pro/thrid_party/scancontext_tro"
    echo "[DEBUG-6a] 使用 SC_SOURCE=/root/automap_ws/src/automap_pro/thrid_party/scancontext_tro"
elif [ -d "/root/automap_ws/src/thrid_party/scancontext_tro" ]; then
    SC_SOURCE="/root/automap_ws/src/thrid_party/scancontext_tro"
    echo "[DEBUG-6b] 使用 SC_SOURCE=/root/automap_ws/src/thrid_party/scancontext_tro"
elif [ -d "/root/mapping/scancontext_tro" ]; then
    SC_SOURCE="/root/mapping/scancontext_tro"
    echo "[DEBUG-6c] 使用 SC_SOURCE=/root/mapping/scancontext_tro"
else
    # 搜索所有可能的 scancontext 目录
    SC_SOURCE=$(find /root/automap_ws /root/mapping -maxdepth 4 -type d -name "scancontext_tro" 2>/dev/null | head -1)
    if [ -n "${SC_SOURCE}" ]; then
        echo "[DEBUG-6d] 使用自动搜索 SC_SOURCE=${SC_SOURCE}"
    else
        echo "[DEBUG-6e] 无法找到 scancontext_tro 目录!"
    fi
fi

echo ""
echo "[DEBUG-7] SC_SOURCE=${SC_SOURCE}, SC_TARGET=${SC_TARGET}"

if [ -n "${SC_SOURCE}" ] && [ -d "${SC_SOURCE}" ]; then
  echo "[DEBUG-7a] SC_SOURCE 目录存在，验证内部结构:"
  ls -la "${SC_SOURCE}/" 2>&1 | head -20
  echo ""
  echo "[DEBUG-7b] 搜索 Scancontext.h:"
  find "${SC_SOURCE}" -name "Scancontext.h" 2>/dev/null

  if [ ! -L "${SC_TARGET}" ] && [ ! -d "${SC_TARGET}" ]; then
    echo ""
    echo "[INFO] 创建 scancontext_tro 符号链接: ${SC_TARGET} -> ${SC_SOURCE}"
    ln -sfn "${SC_SOURCE}" "${SC_TARGET}"
  elif [ -L "${SC_TARGET}" ]; then
    echo "[INFO] scancontext_tro 符号链接已存在: ${SC_TARGET}"
  fi
  # 验证链接是否正确
  if [ -d "${SC_TARGET}/cpp/module/Scancontext" ]; then
    echo "[OK] ScanContext 路径验证成功: ${SC_TARGET}/cpp/module/Scancontext"
    echo "[OK] 列出 Scancontext 目录内容:"
    ls -la "${SC_TARGET}/cpp/module/Scancontext/" 2>&1
  else
    echo "[ERROR] ScanContext 路径验证失败!"
    echo "[ERROR] 检查 ${SC_TARGET} 内容:"
    ls -la "${SC_TARGET}/" 2>&1 || true
    echo ""
    echo "[ERROR] 搜索 Scancontext.h 位置:"
    find "${SC_TARGET}" -name "Scancontext.h" 2>/dev/null || echo "未找到"
  fi
else
  echo "[ERROR] ScanContext 源目录不存在: ${SC_SOURCE}"
fi

# 任一环节异常则立即退出，避免无意义的 colcon
if [ -z "${SC_SOURCE}" ] || [ ! -d "${SC_SOURCE}" ]; then
  echo "[ERROR] ScanContext 源目录不可用，中止编译" >&2
  exit 1
fi
if [ ! -d "${SC_TARGET}/cpp/module/Scancontext" ]; then
  echo "[ERROR] ScanContext 未就绪: ${SC_TARGET}/cpp/module/Scancontext 不存在，中止编译" >&2
  exit 1
fi

echo '========================================'
echo '编译 automap_pro'
echo '========================================'
echo "[INFO] SCANCONTEXT_ROOT=/root/automap_ws/automap_pro_thrid_party_scancontext"
echo "[INFO] NLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3"

# 清理 automap_pro 的 CMake 缓存以确保使用新的 SCANCONTEXT_ROOT 参数
if [ -d build/automap_pro ]; then
  echo "[INFO] 清理 automap_pro CMake 缓存"
  rm -rf build/automap_pro
fi

# 编译前再次确认 scancontext 链接和目录
echo ""
echo "[INFO] 编译前确认 ScanContext 状态:"
if [ -L "/root/automap_ws/automap_pro_thrid_party_scancontext" ]; then
    echo "[INFO] 符号链接存在:"
    ls -la /root/automap_ws/automap_pro_thrid_party_scancontext
    echo ""
    echo "[INFO] 链接目标内容:"
    ls -la /root/automap_ws/automap_pro_thrid_party_scancontext/cpp/module/Scancontext/ 2>&1 || echo "目录不存在"
elif [ -d "/root/automap_ws/automap_pro_thrid_party_scancontext" ]; then
    echo "[INFO] 目录存在:"
    ls -la /root/automap_ws/automap_pro_thrid_party_scancontext/cpp/module/Scancontext/ 2>&1 || echo "目录不存在"
else
    echo "[ERROR] scancontext 链接/目录不存在!"
    exit 1
fi

if [ ! -d "/root/automap_ws/automap_pro_thrid_party_scancontext/cpp/module/Scancontext" ]; then
  echo "[ERROR] 编译前检查 ScanContext 未通过，中止" >&2
  exit 1
fi

echo ""
echo "[INFO] 开始编译 automap_pro..."
automap_log_progress "colcon：automap_pro（主工程，日志同时写入 /tmp/automap_build.log）…"

# 强制统一 Ceres：automap_pro 必须绑定 install_deps/ceres，避免复用旧缓存中的系统 Ceres_DIR（如 /usr/.../Ceres -> libceres.so.4）。
_CERES_CMAKE_DIR=""
if [ -f "${INSTALL_DEPS}/ceres/lib/cmake/Ceres/CeresConfig.cmake" ]; then
  _CERES_CMAKE_DIR="${INSTALL_DEPS}/ceres/lib/cmake/Ceres"
elif [ -f "${INSTALL_DEPS}/ceres/lib64/cmake/Ceres/CeresConfig.cmake" ]; then
  _CERES_CMAKE_DIR="${INSTALL_DEPS}/ceres/lib64/cmake/Ceres"
fi
if [ -n "${_CERES_CMAKE_DIR}" ]; then
  echo "[INFO] automap_pro Ceres_DIR 强制为: ${_CERES_CMAKE_DIR}"
fi

# 无 pipefail 时 colcon|tee 的管道退出码为 tee(0)，set -e 不会失败退出；子 shell + pipefail 保证 colcon 非零即失败
set +e
(
  set -o pipefail
  colcon build \
    ${COLCON_PARALLEL} \
    --event-handlers status+ console_direct+ \
    --packages-select automap_pro \
    --cmake-force-configure \
    --cmake-args ${NINJA_CMAKE_ARG} \
      -DCMAKE_BUILD_TYPE=Release \
      -DNLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3 \
      -DSCANCONTEXT_ROOT=/root/automap_ws/automap_pro_thrid_party_scancontext \
      ${_CERES_CMAKE_DIR:+-DCeres_DIR=${_CERES_CMAKE_DIR}} \
    2>&1 | tee /tmp/automap_build.log
)
BUILD_EXIT_CODE=$?
set -e
echo ""
echo "[INFO] 编译命令退出码: ${BUILD_EXIT_CODE}"

if [ "${BUILD_EXIT_CODE}" -ne 0 ]; then
    # If the build directory was deleted while a compiler job had it as CWD,
    # we may see "getcwd() failed" and many cascading "No such file" errors.
    # This is non-deterministic and typically fixed by cleaning the package build dir and retrying once
    # with reduced parallelism to lower filesystem pressure.
    if grep -q "getcwd() failed: No such file or directory" /tmp/automap_build.log 2>/dev/null; then
        echo ""
        echo "========================================"
        echo "[WARN] 检测到 getcwd() failed（疑似 build 目录在并行构建中被清理/失效），将清理 build/automap_pro 并降并行重试一次"
        echo "========================================"
        echo ""
        rm -rf build/automap_pro
        _RETRY_JOBS="${AUTOMAP_AUTOMAP_PRO_RETRY_JOBS:-8}"
        if [ "${_RETRY_JOBS}" -lt 1 ]; then _RETRY_JOBS=1; fi
        echo "[INFO] retry: CMAKE_BUILD_PARALLEL_LEVEL=${_RETRY_JOBS} (original=${PARALLEL_JOBS})"
        set +e
        (
          set -o pipefail
          export CMAKE_BUILD_PARALLEL_LEVEL="${_RETRY_JOBS}"
          colcon build \
            --parallel-workers 1 \
            ${COLCON_EVENT_HANDLERS} \
            --packages-select automap_pro \
            --cmake-force-configure \
            --cmake-args ${NINJA_CMAKE_ARG} \
              -DCMAKE_BUILD_TYPE=Release \
              -DNLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3 \
              -DSCANCONTEXT_ROOT=/root/automap_ws/automap_pro_thrid_party_scancontext \
              ${_CERES_CMAKE_DIR:+-DCeres_DIR=${_CERES_CMAKE_DIR}} \
            2>&1 | tee /tmp/automap_build_retry.log
        )
        BUILD_EXIT_CODE=$?
        set -e
        echo ""
        echo "[INFO] retry 编译命令退出码: ${BUILD_EXIT_CODE}"
        # Merge retry log into main log path expected by run_automap.sh
        cat /tmp/automap_build_retry.log >> /tmp/automap_build.log 2>/dev/null || true
    fi

    echo ""
    echo "========================================"
    echo "编译失败，分析错误原因"
    echo "========================================"
    echo ""
    echo "[ERROR-1] 检查 CMake 配置阶段是否检测到 ScanContext:"
    grep -i "scancontext" /tmp/automap_build.log | head -20 || echo "未找到 ScanContext 相关日志"
    echo ""
    echo "[ERROR-2] 检查 CMake 生成的 include directories:"
    grep -i "include.*dir" /tmp/automap_build.log | head -20 || echo "未找到 include dir 日志"
    echo ""
    echo "[ERROR-3] 检查编译错误:"
    grep -i "error:" /tmp/automap_build.log | head -10 || echo "未找到错误日志"
    echo ""
    echo "[ERROR-4] 检查 loop_detector 编译详情:"
    grep -A5 "loop_detector" /tmp/automap_build.log | head -20 || echo "未找到 loop_detector 日志"
    echo ""
    echo "[ERROR] colcon 编译 automap_pro 失败（退出码 ${BUILD_EXIT_CODE}），中止" >&2
    exit 1
fi

echo '========================================'
echo '验证编译产物'
echo '========================================'
if [ ! -f install/automap_pro/share/automap_pro/package.xml ]; then
  echo '✗ automap_pro 未找到安装产物'
  exit 1
fi
echo '✓ automap_pro 已安装'
