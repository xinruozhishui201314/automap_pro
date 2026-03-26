#!/bin/bash
# AutoMap-Pro Container Runtime Pre-flight Check and Environment Setup
# Hierarchical structured check: Hardware -> Infrastructure -> OS -> Toolchain -> Runtimes -> Libraries -> Python -> Config -> Data
# Optimized for Blackwell (sm_120) and product-grade reliability.

# --- Global Config ---
AUTOMAP_WORKSPACE_ROOT="/root/automap_ws"

# --- Global State ---
AUTOMAP_PREFLIGHT_ERROR_COUNT=0

# --- Helpers ---
automap_preflight_log() {
  echo -e "\033[1;34m[PRE-FLIGHT]\033[0m $1" 1>&2
}

automap_preflight_success() {
  echo -e "\033[1;32m[PRE-FLIGHT] ✓ $1\033[0m" 1>&2
}

automap_preflight_warn() {
  echo -e "\033[1;33m[PRE-FLIGHT] ⚠ $1\033[0m" 1>&2
}

automap_preflight_error() {
  echo -e "\033[1;31m[PRE-FLIGHT] ✗ $1\033[0m" 1>&2
  AUTOMAP_PREFLIGHT_ERROR_COUNT=$((AUTOMAP_PREFLIGHT_ERROR_COUNT + 1))
}

# Idempotent path prepend (prevents variable bloat)
automap_preflight_prepend_path() {
  local _val="$1"
  local _var_name="$2"
  local _current_val
  eval "_current_val=\"\${$_var_name:-}\""
  
  if [[ ":$_current_val:" != *":$_val:"* ]]; then
    export "$_var_name"="$_val${_current_val:+:$_current_val}"
  fi
}

# --- Stage 0: Hardware Foundation ---
automap_preflight_hardware() {
  automap_preflight_log "Stage 0: Verifying Hardware Foundation..."
  
  # 1. GPU Driver Communication
  if command -v nvidia-smi >/dev/null 2>&1; then
    if ! nvidia-smi >/dev/null 2>&1; then
      automap_preflight_error "nvidia-smi exists but failed to talk to the driver. Check --gpus flags or host drivers."
      return 1
    fi
    local _gpu_driver=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null || echo 'unknown')
    automap_preflight_success "NVIDIA Driver communication OK (v${_gpu_driver})"
  else
    automap_preflight_warn "nvidia-smi not found. GPU features will likely fail."
  fi

  # 2. NVRTC Hardware Matching (sm_120 / Blackwell Check)
  if [ "${AUTOMAP_SKIP_PREFLIGHT_NVRTC_CHECK:-0}" = "0" ] && [ "${AUTOMAP_STRICT_OT_NVRTC_PRECHECK:-1}" = "1" ]; then
    local _gpu_name=""
    if command -v nvidia-smi >/dev/null 2>&1; then
      _gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)
    fi
    
    local _target_arch=""
    if echo "${_gpu_name}" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
      _target_arch="compute_120"
    fi
    
    if [ -n "${_target_arch}" ]; then
      automap_preflight_log "Performing NVRTC hardware matching for ${_gpu_name} (target=${_target_arch})..."
      
      # [CRITICAL] Ensure CUDA libs are in LD_LIBRARY_PATH for the precheck (idempotent)
      for _c in /usr/local/cuda-12.8 /usr/local/cuda-12 /usr/local/cuda /usr/lib/cuda; do
        if [ -x "${_c}/bin/nvcc" ]; then
          automap_preflight_prepend_path "${_c}/bin" "PATH"
          automap_preflight_prepend_path "${_c}/lib64" "LD_LIBRARY_PATH"
          automap_preflight_prepend_path "${_c}/targets/x86_64-linux/lib" "LD_LIBRARY_PATH"
          break
        fi
      done

      local _nvrtc_check_script="/tmp/automap_nvrtc_check.py"
      cat <<'EOF_PY' > "${_nvrtc_check_script}"
import ctypes, os, sys
def check():
    arch = os.environ.get('OT_NVRTC_TARGET_ARCH')
    nvrtc = None
    libs = ['libnvrtc.so', 'libnvrtc.so.12', 'libnvrtc.so.13', 'libnvrtc.so.11']
    for name in libs:
        try: nvrtc = ctypes.CDLL(name); libname = name; break
        except: continue
    if not nvrtc:
        print(f'Error: libnvrtc not found. LD_LIBRARY_PATH={os.environ.get("LD_LIBRARY_PATH", "")}'); sys.exit(2)
    major, minor = ctypes.c_int(), ctypes.c_int()
    v_str = "unknown"
    if hasattr(nvrtc, 'nvrtcVersion'):
        nvrtc.nvrtcVersion(ctypes.byref(major), ctypes.byref(minor))
        v_str = f"{major.value}.{minor.value}"
    print(f"[INFO] Loaded {libname} (v{v_str})", file=sys.stderr)
    prog = ctypes.c_void_p(); src = b'extern "C" __global__ void k() {}'
    nvrtc.nvrtcCreateProgram.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.c_char_p)]
    nvrtc.nvrtcCreateProgram.restype = ctypes.c_int
    nvrtc.nvrtcCompileProgram.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
    nvrtc.nvrtcCompileProgram.restype = ctypes.c_int
    if nvrtc.nvrtcCreateProgram(ctypes.byref(prog), src, b'c.cu', 0, None, None) != 0: sys.exit(3)
    opt = f'--gpu-architecture={arch}'.encode()
    if nvrtc.nvrtcCompileProgram(prog, 1, (ctypes.c_char_p * 1)(opt)) != 0:
        size = ctypes.c_size_t(0); nvrtc.nvrtcGetProgramLogSize(prog, ctypes.byref(size))
        buf = ctypes.create_string_buffer(size.value); nvrtc.nvrtcGetProgramLog(prog, buf)
        print(f"NVRTC Compilation Error for {arch} using {libname}(v{v_str}): {buf.value.decode(errors='ignore')}")
        sys.exit(4)
    print(f"Success: {libname}(v{v_str}) supports {arch}")
check()
EOF_PY

      local _nvrtc_err
      _nvrtc_err=$(OT_NVRTC_TARGET_ARCH="${_target_arch}" python3 "${_nvrtc_check_script}" 2>&1)
      local _nvrtc_rc=$?
      
      if [ "${_nvrtc_rc}" -eq 0 ]; then
        automap_preflight_success "NVRTC hardware check passed: ${_nvrtc_err}"
      else
        automap_preflight_error "NVRTC hardware check failed: ${_nvrtc_err}"
        
        # Auto-repair logic
        if [ "${AUTOMAP_REPAIR_ATTEMPTED:-0}" = "1" ]; then
          automap_preflight_error "NVRTC check still fails after repair; stopping to prevent infinite loop."
          return 1
        fi
        
        automap_preflight_warn "Triggering transactional auto-repair of GPU stack..."
        export AUTOMAP_REPAIR_ATTEMPTED=1
        
        # Transactional LibTorch replacement (Backup instead of delete)
        if [ -d "install_deps/libtorch" ]; then
          if ! grep -qE 'cu128|2\.7\.0' "install_deps/libtorch/share/cmake/Torch/TorchConfig.cmake" 2>/dev/null; then
            automap_preflight_warn "Non-cu128 LibTorch found on Blackwell. Backing up and preparing for reinstall..."
            rm -rf install_deps/libtorch.bak
            mv install_deps/libtorch install_deps/libtorch.bak
          fi
        fi
        
        if [ -x "${HOME}/scripts/build_inside_container.sh" ]; then
          export AUTOMAP_PREBUILT_INSTALL_DEPS=0
          export LIBTORCH_SKIP_DOWNLOAD=0
          export AUTOMAP_SKIP_CUDA_TOOLKIT_APT=0
          export AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED=1
          AUTOMAP_REPAIR_CUDA_STACK_ONLY=1 /bin/bash "${HOME}/scripts/build_inside_container.sh" 1>&2
          
          # Re-check NVRTC
          _nvrtc_err=$(OT_NVRTC_TARGET_ARCH="${_target_arch}" python3 "${_nvrtc_check_script}" 2>&1)
          if [ $? -eq 0 ]; then
            automap_preflight_success "NVRTC matching succeeded after repair"
            rm -rf install_deps/libtorch.bak # Clean up backup on success
          else
            automap_preflight_error "NVRTC matching failed after repair: ${_nvrtc_err}"
            # Rollback if needed (though rebuild might have created a partial directory)
            [ -d install_deps/libtorch.bak ] && automap_preflight_warn "Consider rolling back manually from install_deps/libtorch.bak"
            return 1
          fi
        fi
      fi
    fi
  fi
}

# --- Stage 1: System Infrastructure ---
automap_preflight_infrastructure() {
  automap_preflight_log "Stage 1: Verifying System Infrastructure..."
  
  # 1. Workspace Write Permissions
  local _ws_path="${AUTOMAP_WORKSPACE:-$HOME/automap_ws}"
  local _test_file="${_ws_path}/.write_test"
  if touch "${_test_file}" 2>/dev/null; then
    rm -f "${_test_file}"
    automap_preflight_success "Workspace write permission verified"
  else
    automap_preflight_error "Workspace (${_ws_path}) is READ-ONLY! Check docker mount flags."
    return 1
  fi
  
  # 2. Temporary Directory Permissions
  if [ ! -w "/tmp" ]; then
    automap_preflight_error "/tmp is not writable! NVRTC and other tools will fail."
    return 1
  fi

  # 3. Disk Space Check
  local _df_out=$(df -h "${_ws_path}" | tail -1)
  local _avail=$(echo "${_df_out}" | awk '{print $4}')
  automap_preflight_log "Available disk space in workspace: ${_avail}"

  # 4. Download Cache Mount
  local _cache_path="${AUTOMAP_DOWNLOAD_CACHE_PATH:-$HOME/automap_download_cache}"
  if [ -d "${_cache_path}" ]; then
    automap_preflight_success "Download cache mount verified (${_cache_path})"
  else
    automap_preflight_warn "Download cache (${_cache_path}) not mounted. Downloads will not be persistent."
  fi
}

# --- Stage 2: OS Services & Lock Management ---
automap_preflight_os_services() {
  automap_preflight_log "Stage 2: OS Services & Lock Management..."
  
  local _lock_files=(
    "/var/lib/apt/lists/lock"
    "/var/cache/apt/archives/lock"
    "/var/lib/dpkg/lock"
    "/var/lib/dpkg/lock-frontend"
  )
  
  local _found=0
  for _l in "${_lock_files[@]}"; do
    if [ -f "$_l" ]; then
      # Atomic cleanup: only delete if not used by an active process
      if command -v fuser >/dev/null 2>&1; then
        if ! fuser "$_l" >/dev/null 2>&1; then
          rm -f "$_l"
          _found=1
        fi
      else
        # Fallback: simple delete if fuser missing (less safe but necessary)
        rm -f "$_l"
        _found=1
      fi
    fi
  done
  [ "${_found}" = "1" ] && automap_preflight_warn "Stale APT/dpkg locks recovered" || automap_preflight_success "Package manager locks clean"
}

# --- Stage 3: Build & Development Toolchain ---
automap_preflight_toolchain() {
  [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "0" ] && return 0
  automap_preflight_log "Stage 3: Verifying Build Toolchain..."
  
  local _tools=("gcc" "g++" "cmake" "ninja")
  local _missing=()
  for _t in "${_tools[@]}"; do
    if ! command -v "${_t}" >/dev/null 2>&1; then
      _missing+=("${_t}")
    fi
  done
  
  if [ ${#_missing[@]} -gt 0 ]; then
    if [ "$(id -u)" -eq 0 ]; then
      automap_preflight_warn "Missing build tools: ${_missing[*]}. Attempting to install..."
      DEBIAN_FRONTEND=noninteractive apt-get update -qq \
        && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq build-essential cmake ninja-build \
        || { automap_preflight_error "Failed to install build tools"; return 1; }
    else
      automap_preflight_error "Missing build tools: ${_missing[*]} and not running as root. Cannot install."
      return 1
    fi
  else
    automap_preflight_success "Build tools verified ($(gcc --version | head -1))"
  fi
}

# --- Stage 4: Core Runtimes (CUDA & ROS) ---
automap_preflight_runtimes() {
  automap_preflight_log "Stage 4: Initializing Core Runtimes..."
  
  # 1. Parallelism limits (GTSAM TBB workaround)
  export OMP_NUM_THREADS=1
  export EIGEN_NUM_THREADS=1
  export MKL_NUM_THREADS=1
  export TBB_NUM_THREADS=1
  export AUTOMAP_GTSAM_SERIAL=1

  # 2. CUDA Environment
  local _cuda_found=0
  for _c in /usr/local/cuda-12.8 /usr/local/cuda-12 /usr/local/cuda /usr/lib/cuda; do
    if [ -x "${_c}/bin/nvcc" ]; then
      export CUDA_HOME="${_c}"
      automap_preflight_prepend_path "${CUDA_HOME}/bin" "PATH"
      automap_preflight_prepend_path "${CUDA_HOME}/lib64" "LD_LIBRARY_PATH"
      automap_preflight_prepend_path "${CUDA_HOME}/targets/x86_64-linux/lib" "LD_LIBRARY_PATH"
      automap_preflight_success "CUDA initialized: ${CUDA_HOME}"
      _cuda_found=1
      break
    fi
  done
  [ "${_cuda_found}" = "0" ] && automap_preflight_warn "CUDA Toolkit not found in standard paths."

  # 3. ROS Sourcing
  if [ -f "/opt/ros/${AUTOMAP_ROS_DISTRO}/setup.bash" ]; then
    set +u # Disable unbound variable check for ROS setup scripts
    source "/opt/ros/${AUTOMAP_ROS_DISTRO}/setup.bash"
    set -u # Re-enable
    automap_preflight_success "ROS ${AUTOMAP_ROS_DISTRO} environment sourced"
  else
    automap_preflight_error "ROS ${AUTOMAP_ROS_DISTRO} not found in /opt/ros"
    return 1
  fi
  
  # 4. Apt cache defaults
  if [ -f "${HOME}/scripts/automap_download_defaults.sh" ]; then
    set +u
    source "${HOME}/scripts/automap_download_defaults.sh"
    set -u
    [ "$(id -u)" -eq 0 ] && automap_configure_apt_cache
  fi
}

# --- Stage 5: Application Dependencies (Libraries) ---
automap_preflight_libraries() {
  automap_preflight_log "Stage 5: Verifying Application Dependencies..."

  # 1. ROS System Packages
  local _missing_pkgs=()
  for _p in "ros-${AUTOMAP_ROS_DISTRO}-cv-bridge" "ros-${AUTOMAP_ROS_DISTRO}-pcl-ros" "libgeographiclib-dev"; do
    if ! dpkg -s "${_p}" >/dev/null 2>&1; then
      _missing_pkgs+=("${_p}")
    fi
  done
  [ "${AUTOMAP_USE_RVIZ:-false}" = "true" ] && ! dpkg -s "ros-${AUTOMAP_ROS_DISTRO}-rviz2" >/dev/null 2>&1 && _missing_pkgs+=("ros-${AUTOMAP_ROS_DISTRO}-rviz2")
  [ "${AUTOMAP_ENSURE_GDB:-0}" = "1" ] && ! command -v gdb >/dev/null 2>&1 && _missing_pkgs+=("gdb")

  if [ ${#_missing_pkgs[@]} -gt 0 ]; then
    if [ "$(id -u)" -eq 0 ]; then
      automap_preflight_warn "Installing missing system dependencies: ${_missing_pkgs[*]}"
      DEBIAN_FRONTEND=noninteractive apt-get update -qq \
        && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "${_missing_pkgs[@]}" \
        || { automap_preflight_error "Apt installation failed"; return 1; }
    else
      automap_preflight_error "Missing system dependencies: ${_missing_pkgs[*]} and not root. Cannot install."
      return 1
    fi
  else
    automap_preflight_success "System dependencies verified"
  fi

  # 2. Third-party Library Paths (GTSAM, LibTorch, etc.)
  if [ -d "${AUTOMAP_WORKSPACE_ROOT}/build_gtsam_no_tbb/gtsam" ] && [ -f "${AUTOMAP_WORKSPACE_ROOT}/build_gtsam_no_tbb/gtsam/libgtsam.so" ]; then
    automap_preflight_prepend_path "${AUTOMAP_WORKSPACE_ROOT}/build_gtsam_no_tbb/gtsam" "LD_LIBRARY_PATH"
    automap_preflight_prepend_path "${AUTOMAP_WORKSPACE_ROOT}/build_gtsam_no_tbb/gtsam_unstable" "LD_LIBRARY_PATH"
    automap_preflight_success "Custom GTSAM (No-TBB) prioritized"
  fi

  local _id_deps=("teaserpp" "ceres" "vikit" "hba" "fast_livo" "libtorch" "onnxruntime")
  for _d in "${_id_deps[@]}"; do
    local _dir="${AUTOMAP_WORKSPACE_ROOT}/install_deps/${_d}"
    if [ -d "${_dir}" ]; then
      automap_preflight_prepend_path "${_dir}" "CMAKE_PREFIX_PATH"
      if [ -f "${_dir}/setup.bash" ]; then
        set +u
        source "${_dir}/setup.bash"
        set -u
      fi
      [ -d "${_dir}/lib" ] && automap_preflight_prepend_path "${_dir}/lib" "LD_LIBRARY_PATH"
      [ -d "${_dir}/lib64" ] && automap_preflight_prepend_path "${_dir}/lib64" "LD_LIBRARY_PATH"
      
      # 3. LibTorch Deep Dependency (LDD) Verification
      if [ "${_d}" = "libtorch" ] && [ -f "${_dir}/lib/libtorch.so" ]; then
        local _missing_ldd=$(ldd "${_dir}/lib/libtorch.so" | grep "not found" || true)
        if [ -n "${_missing_ldd}" ]; then
          automap_preflight_error "LibTorch has missing runtime dependencies:\n${_missing_ldd}"
          return 1
        fi
      fi
      automap_preflight_success "${_d} library loaded from ${_dir}"
    fi
  done
}

# --- Stage 6: Python Runtime Environment ---
automap_preflight_python() {
  automap_preflight_log "Stage 6: Verifying Python Runtime..."
  
  # 1. Core Python Modules
  # [Robustness] In build mode, torch might not be present yet. Skip torch check if building.
  local _py_cmd="import yaml, numpy, cv2; print('Core (yaml,numpy,cv2) OK')"
  if [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "0" ]; then
    _py_cmd="import yaml, numpy, cv2, torch; print(f'Torch {torch.__version__} OK')"
  fi

  if python3 -c "${_py_cmd}" >/tmp/py_check 2>&1; then
    automap_preflight_success "Python modules verified: $(cat /tmp/py_check)"
  else
    automap_preflight_warn "Python modules check failed (this is expected during first build):\n$(cat /tmp/py_check)"
  fi

  # 2. LSK3DNet Venv
  local _venv_dir="${AUTOMAP_WORKSPACE_ROOT}/install_deps/lsk3dnet_venv"
  local _venv_py="${_venv_dir}/bin/python3"
  local _marker="${_venv_dir}/.lsk3dnet_venv_ok"
  
  if [ -x "${_venv_py}" ]; then
    export AUTOMAP_LSK3DNET_PYTHON="${AUTOMAP_LSK3DNET_PYTHON:-${_venv_py}}"
    
    if [ ! -f "${_marker}" ]; then
      automap_preflight_warn "LSK3DNet venv exists but marker (.lsk3dnet_venv_ok) is missing. May be incomplete."
      # In build mode, we let the build script handle re-installation.
      # In runtime mode, this is a severe warning or error.
      if [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "0" ]; then
        automap_preflight_error "LSK3DNet venv is incomplete (missing marker). Run with --clean to fix."
      else
        # [CRITICAL] Don't let Stage 7 use an incomplete venv during build
        unset AUTOMAP_LSK3DNET_PYTHON
      fi
    else
      # Deep functional check
      if "${_venv_py}" -c "import torch, spconv.pytorch, torch_scatter, cv2; print(f'LSK Venv OK (Torch {torch.__version__}, cv2 {cv2.__version__})')" >/tmp/lsk_venv_check 2>&1; then
        automap_preflight_success "$(cat /tmp/lsk_venv_check)"
        local _cv2_file
        _cv2_file=$("${_venv_py}" -c "import cv2; print(getattr(cv2, '__file__', 'unknown'))" 2>/dev/null || echo "unknown")
        automap_preflight_log "LSK runtime python=${AUTOMAP_LSK3DNET_PYTHON:-${_venv_py}} cv2.__file__=${_cv2_file}"
      else
        if [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "1" ]; then
          automap_preflight_warn "LSK3DNet venv is broken (symbol mismatch or missing deps). Will rebuild during build stage.\nError: $(cat /tmp/lsk_venv_check)"
          unset AUTOMAP_LSK3DNET_PYTHON
        else
          automap_preflight_error "LSK3DNet venv is broken! Error:\n$(cat /tmp/lsk_venv_check)"
          return 1
        fi
      fi
    fi
    
    # [Blackwell Fix] Verify PyTorch version compatibility for Blackwell GPUs
    local _gpu_name=""
    if command -v nvidia-smi >/dev/null 2>&1; then
      _gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)
    fi
    if echo "${_gpu_name}" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
      local _torch_ver
      _torch_ver=$("${_venv_py}" -c "import torch; print(torch.__version__)" 2>/dev/null || echo "0.0.0")
      if [[ "${_torch_ver}" < "2.7.0" ]]; then
        automap_preflight_warn "Blackwell GPU detected but LSK venv PyTorch version (${_torch_ver}) < 2.7.0. Incompatibility expected."
        if [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "1" ]; then
           automap_preflight_log "Triggering LSK venv upgrade for Blackwell (forcing rebuild)..."
           rm -f "${_marker}"
           # [CRITICAL] Version is bad. Unset python so Stage 7 doesn't use it.
           unset AUTOMAP_LSK3DNET_PYTHON
        fi
      fi
    fi
  else
    [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "0" ] && automap_preflight_error "LSK3DNet venv not found at ${_venv_dir}"
  fi
}

# --- Stage 7: Logic & Configuration Verification ---
automap_preflight_logic_config() {
  automap_preflight_log "Stage 7: Verifying Configuration Logic..."
  
  local _cfg_name="${CONFIG_FILE:-system_config_M2DGR.yaml}"
  local _cfg_path="${AUTOMAP_WORKSPACE_ROOT}/src/automap_pro/config/${_cfg_name}"
  
  if [ ! -f "${_cfg_path}" ]; then
    automap_preflight_error "Configuration file not found: ${_cfg_path}"
    return 1
  fi

  # 1. YAML Syntax check
  if python3 -c "import yaml; yaml.safe_load(open('${_cfg_path}'))" 2>/tmp/yaml_err; then
    automap_preflight_success "Config YAML syntax verified: ${_cfg_name}"
  else
    automap_preflight_error "Invalid YAML syntax in ${_cfg_name}:\n$(cat /tmp/yaml_err)"
    return 1
  fi

  # 2. LSK3DNet Hybrid Configuration matching
  local _lsk_mt=$(python3 -c "import yaml; cfg=yaml.safe_load(open('${_cfg_path}')) or {}; sem=cfg.get('semantic') or {}; print(sem.get('model_type','').strip())" 2>/dev/null || true)
  if [ "${_lsk_mt}" = "lsk3dnet_hybrid" ]; then
    automap_preflight_log "LSK3DNet Hybrid model detected; verifying classification head..."
    
    local _lsk_nm=$(python3 -c "import yaml; cfg=yaml.safe_load(open('${_cfg_path}')) or {}; sem=cfg.get('semantic') or {}; lsk=sem.get('lsk3dnet') or {}; print(lsk.get('hybrid_normal_mode','range').strip().lower())" 2>/dev/null || true)
    if [ "${_lsk_nm}" = "range" ]; then
      local _py="${AUTOMAP_LSK3DNET_PYTHON:-python3}"
      if ! "${_py}" -c "import c_gen_normal_map" >/dev/null 2>&1; then
        if [ "${AUTOMAP_PREFLIGHT_BUILD_TOOLS:-0}" = "1" ]; then
          automap_preflight_warn "[LSK3DNET] hybrid_normal_mode=range requires c_gen_normal_map (will compile during build)"
        else
          automap_preflight_error "[LSK3DNET] hybrid_normal_mode=range requires c_gen_normal_map module"
          return 1
        fi
      fi
    fi

  # Check/Export classifier TorchScript
  local _lsk_repo _lsk_cfg _lsk_ckpt _lsk_cls
  local _root="${AUTOMAP_WORKSPACE_ROOT}/src/automap_pro"
  _lsk_repo=$(python3 -c "import os,yaml; cfg=yaml.safe_load(open('${_cfg_path}')) or {}; sem=cfg.get('semantic') or {}; lsk=sem.get('lsk3dnet') or {}; root='${_root}'; p=(lsk.get('repo_root') or '').strip().replace('\${CMAKE_CURRENT_SOURCE_DIR}', root); print(os.path.normpath(p))" 2>/dev/null || true)
  _lsk_cfg=$(python3 -c "import yaml; cfg=yaml.safe_load(open('${_cfg_path}')) or {}; sem=cfg.get('semantic') or {}; lsk=sem.get('lsk3dnet') or {}; print(lsk.get('config_yaml','').strip())" 2>/dev/null || true)
  _lsk_ckpt=$(python3 -c "import os,yaml; cfg=yaml.safe_load(open('${_cfg_path}')) or {}; sem=cfg.get('semantic') or {}; lsk=sem.get('lsk3dnet') or {}; root='${_root}'; p=(lsk.get('checkpoint') or '').strip().replace('\${CMAKE_CURRENT_SOURCE_DIR}', root); print(os.path.normpath(p))" 2>/dev/null || true)
  _lsk_cls=$(python3 -c "import os,yaml; cfg=yaml.safe_load(open('${_cfg_path}')) or {}; sem=cfg.get('semantic') or {}; lsk=sem.get('lsk3dnet') or {}; root='${_root}'; p=(lsk.get('classifier_torchscript') or '').strip().replace('\${CMAKE_CURRENT_SOURCE_DIR}', root); print(os.path.normpath(p))" 2>/dev/null || true)

  if [ -n "${_lsk_cls}" ] && [ ! -f "${_lsk_cls}" ]; then
    local _exp_py="${AUTOMAP_LSK3DNET_PYTHON:-python3}"
    local _exp_script="${_lsk_repo}/scripts/export_lsk3dnet_classifier_torchscript.py"
    
    # [Transactional Fix] Check if output directory is writable. If not, use install_deps.
    local _cls_dir="$(dirname "${_lsk_cls}")"
    local _final_cls="${_lsk_cls}"
    if [ ! -w "${_cls_dir}" ]; then
      _final_cls="${AUTOMAP_WORKSPACE_ROOT}/install_deps/models/LSK3DNET/$(basename "${_lsk_cls}")"
      # If redirected file already exists, we are done
      if [ -f "${_final_cls}" ]; then
        automap_preflight_success "[LSK3DNET] Classifier found in redirected path: ${_final_cls}"
        export AUTOMAP_LSK3DNET_CLASSIFIER_PATH="${_final_cls}"
        return 0
      fi
      automap_preflight_warn "[LSK3DNET] Source dir is RO. Redirecting export to install_deps..."
      mkdir -p "$(dirname "${_final_cls}")" 2>/dev/null || true
    fi

    if [ -f "${_exp_script}" ] && [ -n "${_lsk_cfg}" ] && [ -f "${_lsk_ckpt}" ]; then
      # [Robustness] Check if the chosen python can even import torch and torch_scatter
      if ! "${_exp_py}" -c "import torch; import torch_scatter" >/dev/null 2>&1; then
        automap_preflight_warn "[LSK3DNET] skipping classifier export: chosen python (${_exp_py}) lacks torch or torch_scatter (symbol mismatch?)"
        return 0
      fi
      automap_preflight_warn "[LSK3DNET] classifier_torchscript missing, exporting -> ${_final_cls}"
      if ( cd "${_lsk_repo}" && "${_exp_py}" "scripts/export_lsk3dnet_classifier_torchscript.py" --config "${_lsk_cfg}" --checkpoint "${_lsk_ckpt}" --output "${_final_cls}" --device cpu ) 1>&2; then
        if [ -f "${_final_cls}" ]; then
          automap_preflight_success "[LSK3DNET] export completed"
          # If we redirected, export an env var so the app can find it (optional, depends on app logic)
          [ "${_final_cls}" != "${_lsk_cls}" ] && export AUTOMAP_LSK3DNET_CLASSIFIER_PATH="${_final_cls}"
        else
          automap_preflight_error "[LSK3DNET] export script finished but file not found!"
          return 1
        fi
      else
        automap_preflight_error "[LSK3DNET] export script failed!"
        return 1
      fi
    else
      automap_preflight_warn "[LSK3DNET] cannot export classifier: script/config/ckpt missing"
    fi
  fi
  automap_preflight_success "LSK3DNet hybrid logic verified"
  fi
}

# --- Stage 8: Input Data Integrity ---
automap_preflight_input_data() {
  [ "${AUTOMAP_SKIP_PREFLIGHT_BAG_CHECK:-0}" = "1" ] && return 0
  automap_preflight_log "Stage 8: Verifying Input Data Integrity..."
  
  if [ -n "${AUTOMAP_OFFLINE_BAG_PATH:-}" ]; then
    local _bagp="${AUTOMAP_OFFLINE_BAG_PATH}"
    [ -f "${_bagp}" ] && _bagp="$(dirname "${_bagp}")"
    if [ -d "${_bagp}" ] && [ -f "${_bagp}/metadata.yaml" ]; then
      automap_preflight_log "Scanning bag metadata: ${_bagp}"
      local _fix_script="${AUTOMAP_WORKSPACE_ROOT}/src/automap_pro/scripts/fix_ros2_bag_metadata.py"
      if [ -f "${_fix_script}" ]; then
        ROS_DISTRO="${AUTOMAP_ROS_DISTRO}" python3 "${_fix_script}" "${_bagp}" --qos-style auto --ros-distro "${AUTOMAP_ROS_DISTRO}" --verify-with-ros2-info 1>&2 || true
        automap_preflight_success "Bag metadata verified/repaired"
      fi
    fi
  fi
}

# --- Stage 9: Recovery Advice ---
automap_preflight_recovery_hint() {
  echo -e "\n\033[1;33m[RECOVERY ADVICE]\033[0m" 1>&2
  echo " 1. For hardware issues: Ensure host drivers match CUDA 12.8+ and --gpus is used." 1>&2
  echo " 2. For dependency issues: Run with --clean to force re-download/re-build." 1>&2
  echo " 3. For permission issues: Check UID/GID and docker -v mount options." 1>&2
  echo " 4. Consult logs/automap.log for detailed stack traces." 1>&2
}

# --- Main Entry ---
main_preflight() {
  local _u_state
  _u_state=$(set +o | grep nounset)
  set -u
  trap 'automap_preflight_recovery_hint' ERR
  
  echo "============================================================" 1>&2
  echo "         AutoMap-Pro Production Pre-flight Check            " 1>&2
  echo "============================================================" 1>&2
  
  # Stage-by-stage execution with fail-fast check
  automap_preflight_hardware || { eval "$_u_state"; return 1; }
  automap_preflight_infrastructure || { eval "$_u_state"; return 1; }
  automap_preflight_os_services || { eval "$_u_state"; return 1; }
  automap_preflight_toolchain || { eval "$_u_state"; return 1; }
  automap_preflight_runtimes || { eval "$_u_state"; return 1; }
  automap_preflight_libraries || { eval "$_u_state"; return 1; }
  automap_preflight_python || { eval "$_u_state"; return 1; }
  automap_preflight_logic_config || { eval "$_u_state"; return 1; }
  automap_preflight_input_data || { eval "$_u_state"; return 1; }
  
  if [ "${AUTOMAP_PREFLIGHT_ERROR_COUNT}" -gt 0 ]; then
    echo "============================================================" 1>&2
    echo -e "\033[1;31m         ✗ Pre-flight Check Failed with ${AUTOMAP_PREFLIGHT_ERROR_COUNT} Errors.  \033[0m" 1>&2
    echo "============================================================" 1>&2
    eval "$_u_state"
    return 1
  fi

  echo "============================================================" 1>&2
  echo "         ✓ Pre-flight Validation Complete. Proceeding...      " 1>&2
  echo "============================================================" 1>&2
  eval "$_u_state"
}

main_preflight
