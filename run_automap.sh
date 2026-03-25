#!/bin/bash
# ==========================================================================
# AutoMap-Pro 一键编译和运行脚本
#
# 功能：
#   1. 检查 Docker 镜像是否存在，不存在则构建
#   2. 启动 Docker 容器并挂载必要目录
#   3. 在容器内编译项目（首次运行）
#   4. 启动建图系统（在线/离线模式可选）
#
# 使用方法:
#   bash run_automap.sh [选项]
#
# 选项:
#   --online           在线模式（实时建图，默认）
#   --offline          离线模式（回放 rosbag）
#   --bag-file <path>  指定 rosbag 文件路径（离线模式使用）
#   --gdb              以 GDB 启动 automap_system_node，崩溃时打印 backtrace
#   --gdb-frontend     以 GDB 启动 fastlivo_mapping（前端），用于定位 frame=10 等 SIGSEGV
#   --build-only       仅编译不运行
#   --run-only         仅运行不编译（假设已编译）
#   --no-rviz          不启动 RViz 可视化
#   --no-external-frontend 使用自研前端（默认使用 Fast-LIVO2）
#   --external-overlap  使用 OverlapTransformer 服务作为回环描述子
#   --clean            清理编译产物后重新编译
#   --no-clean-output  离线模式时不清空输出目录（默认离线会清空 data/automap_output，保证每次重新建图）
#   --log-dir <path>   将运行日志全部保存到宿主机目录（容器内 stdout/stderr 会 tee 到该目录下的 full.log）
#   --help             显示帮助信息
#
# 日志与时间：所有写入 automap.log / build.log / full.log / clean.log / image.log 的日志行均带时间戳（YYYY-MM-DD HH:MM:SS）；
# 容器通过挂载宿主机 /etc/localtime（及 /etc/timezone）与宿主机时间同步，不修改 Dockerfile/镜像，仅在脚本内生效。
#
# 示例:
#   bash run_automap.sh                    # 在线模式建图
#   bash run_automap.sh --offline --bag-file /data/record.mcap  # 离线模式回放
#   bash run_automap.sh --build-only      # 仅编译（默认创建 LSK3DNet venv；关闭: AUTOMAP_SETUP_LSK3DNET_VENV=0）
#   bash run_automap.sh --offline --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" --config system_config_M2DGR.yaml --gdb --clean
#   bash run_automap.sh --run-only --no-rviz  # 运行但不启动 RViz
#
# 作者: AutoMap-Pro Team
# 日期: 2025-02-28
# ==========================================================================

set -e  # 遇到错误立即退出
# 异常时打印失败命令与行号，便于看到问题原因（set -e 会随后退出）
trap 'echo "[ERROR] 脚本异常退出: 最后命令=\"$BASH_COMMAND\" 行号=$LINENO"; exit 1' ERR
clear && echo "=========================================" && echo "AutoMap-Pro 一键编译和运行脚本" && echo "=========================================" && echo ""
# ==================== 配置参数 ====================
# 工作空间与数据目录放在项目下，避免 $HOME 下因权限（如曾由容器 root 创建）导致 mkdir 失败；
# 源码与编译产物均通过挂载进入容器，无需修改镜像。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/automap_docker_defaults.sh
source "${SCRIPT_DIR}/scripts/automap_docker_defaults.sh"
WORKSPACE_DIR="${SCRIPT_DIR}/automap_ws"
PROJECT_DIR="${SCRIPT_DIR}/automap_pro"
DATA_DIR="${SCRIPT_DIR}/data"
OUTPUT_DIR="${DATA_DIR}/automap_output"
# 国内镜像 + 持久化缓存（pip / LibTorch zip / ONNX 源码快照），挂载到容器 /root/automap_download_cache
AUTOMAP_CACHE_HOST="${SCRIPT_DIR}/thrid_party/automap_cache"
mkdir -p "${AUTOMAP_CACHE_HOST}/pip" "${AUTOMAP_CACHE_HOST}/libtorch" "${AUTOMAP_CACHE_HOST}/git" \
  "${AUTOMAP_CACHE_HOST}/apt/archives/partial" "${AUTOMAP_CACHE_HOST}/xdg"
AUTOMAP_CACHE_MOUNT="-v ${AUTOMAP_CACHE_HOST}:/root/automap_download_cache:rw"

# 基础镜像名（供 ensure_image 在「未使用快照」时回退；勿与下方快照 tag 混淆）
AUTOMAP_BASE_DOCKER_IMAGE="${IMAGE_NAME}"
# 运行时快照：首次「编译 + 正常运行」成功后 commit + save 到本地 tar；下次优先 load，减少 pull/apt
AUTOMAP_RUNTIME_SNAPSHOT_DIR="${AUTOMAP_CACHE_HOST}/docker"
mkdir -p "${AUTOMAP_RUNTIME_SNAPSHOT_DIR}"
AUTOMAP_RUNTIME_SNAPSHOT_TAR="${AUTOMAP_RUNTIME_SNAPSHOT_TAR:-${AUTOMAP_RUNTIME_SNAPSHOT_DIR}/automap-pro-runtime.tar}"
AUTOMAP_RUNTIME_SNAPSHOT_TAG="${AUTOMAP_RUNTIME_SNAPSHOT_TAG:-automap-pro-runtime:latest}"
AUTOMAP_RUNTIME_CONTAINER_NAME="${AUTOMAP_RUNTIME_CONTAINER_NAME:-automap-pro-last-run}"
# 编译阶段快照：编译容器使用独立容器名，成功则 commit+save 到与运行时相同的 tar/tag（下次 ensure_image 优先加载）
AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME="${AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME:-automap-pro-build-snapshot}"

# 将已停止的容器提交为 AUTOMAP_RUNTIME_SNAPSHOT_TAG 并写入 AUTOMAP_RUNTIME_SNAPSHOT_TAR（供 ensure_image 优先 load）
automap_commit_and_save_runtime_snapshot() {
    local _cname="$1"
    local _suffix="${2:-snapshot}"
    if ! docker container inspect "${_cname}" &>/dev/null; then
        print_warning "⚠ 未找到容器 ${_cname}，跳过快照"
        return 1
    fi
    print_header "保存 Docker 快照（${_suffix}）"
    if docker commit -m "automap-pro ${_suffix} $(date -Iseconds)" "${_cname}" "${AUTOMAP_RUNTIME_SNAPSHOT_TAG}"; then
        docker rm -f "${_cname}"
        mkdir -p "$(dirname "${AUTOMAP_RUNTIME_SNAPSHOT_TAR}")"
        local _snap_tmp="${AUTOMAP_RUNTIME_SNAPSHOT_TAR}.tmp.$$"
        print_info "导出 ${AUTOMAP_RUNTIME_SNAPSHOT_TAR}（可能数 GB，请稍候）…"
        if docker save "${AUTOMAP_RUNTIME_SNAPSHOT_TAG}" -o "${_snap_tmp}"; then
            mv -f "${_snap_tmp}" "${AUTOMAP_RUNTIME_SNAPSHOT_TAR}"
            print_success "✓ 快照已写入 ${AUTOMAP_RUNTIME_SNAPSHOT_TAR}；下次将优先加载（AUTOMAP_PREFER_RUNTIME_SNAPSHOT=1）"
            return 0
        fi
        rm -f "${_snap_tmp}" 2>/dev/null || true
        print_warning "⚠ docker save 失败，已保留本地镜像标签 ${AUTOMAP_RUNTIME_SNAPSHOT_TAG}"
        return 0
    fi
    print_warning "⚠ docker commit 失败，正在删除临时容器 ${_cname}"
    docker rm -f "${_cname}" 2>/dev/null || true
    return 1
}

# 运行模式
MODE="online"
BAG_FILE=""
CONFIG_FILE=""
BAG_RATE="0.5"  # 默认 0.5 倍速回放，缓解 LIO 处理压力；可改为 1.0 实时
BUILD_ONLY=false
RUN_ONLY=false
USE_RVIZ=true
# --no-rviz / --rviz 显式设置时为 true（用于离线模式默认关闭 RViz）
RVIZ_CLI_SET=false
CLEAN_BUILD=false
USE_EXTERNAL_FRONTEND=true
USE_EXTERNAL_OVERLAP=false
RUN_AUTOMAP_UNDER_GDB=false
RUN_FAST_LIVO_UNDER_GDB=false
# --gdb 调试默认镜像（可用 AUTOMAP_GDB_DOCKER_IMAGE 覆盖）
AUTOMAP_GDB_DOCKER_IMAGE="${AUTOMAP_GDB_DOCKER_IMAGE:-automap-pro:latest}"
# 默认开启构建阶段（build_inside_container）以保证工程可正常编译；
# 依赖安装是否跳过由 AUTOMAP_PREBUILT_INSTALL_DEPS 控制（如仅跳过 LibTorch/cuda-toolkit 安装）。
# 如确需完全跳过编译/依赖安装，可显式设 AUTOMAP_ENABLE_BUILD_STEPS=0。
AUTOMAP_ENABLE_BUILD_STEPS="${AUTOMAP_ENABLE_BUILD_STEPS:-1}"
# 离线模式是否在启动前清空输出目录（默认 true=每次离线重新建图，避免沿用上次结果）
CLEAN_OUTPUT_ON_OFFLINE=true
# 宿主机日志目录：编译输出 → build.log，运行输出 → full.log
# 默认使用带时间戳子目录 logs/run_YYYYMMDD_HHMMSS/，便于区分每次运行；可用 --log-dir 覆盖为固定目录
LOG_DIR="${SCRIPT_DIR}/logs"
LOG_USE_TIMESTAMP_SUBDIR=true
LOG_DIR_USER_SET=false

# ==================== 帮助信息 ====================
show_help() {
    cat << EOF
AutoMap-Pro 一键编译和运行脚本

使用方法:
    bash run_automap.sh [选项]

  选项:
    --online           在线模式（实时建图，默认）
    --offline          离线模式（回放 rosbag）
    --bag-file <path>  指定 rosbag 目录或文件路径（离线模式使用）
    --bag-rate <rate>  Bag 回放速率，0.1-1.0（默认 0.5=半速）。需实时可传 1.0
    --config <file>    指定配置文件（如 system_config_M2DGR.yaml，默认 system_config.yaml）
    --gdb              以 GDB 启动 automap_system_node，崩溃时自动打印完整 backtrace（容器内需已安装 gdb）
    --gdb-frontend     以 GDB 启动 fastlivo_mapping（前端）；用于定位前端 SIGSEGV（如 frame=10 崩溃）时抓 backtrace
    --build-only       仅编译不运行
    --run-only         仅运行不编译（假设已编译）
    --no-rviz          不启动 RViz 可视化
    --rviz             启动 RViz（离线模式默认不启 RViz 时用于强制开启）
    --no-external-frontend 使用自研前端（默认使用 fast-livo2-humble）
    --external-overlap  使用 OverlapTransformer 服务作为回环描述子（需 config 中 loop_closure.overlap_transformer.mode: external_service）
    --clean            清理编译产物后重新编译
    --no-clean-output  离线时不清空输出目录（默认离线会清空 data/automap_output，保证每次重新建图）
            --log-dir <path>   覆盖默认日志目录（默认: 项目根/logs/run_YYYYMMDD_HHMMSS/；编译→build.log，运行→full.log）
    --help             显示帮助信息

示例:
    bash run_automap.sh                                        # 在线模式建图
    bash run_automap.sh --offline --bag-file /data/record.mcap # 离线模式回放
    bash run_automap.sh --build-only                          # 仅编译
    bash run_automap.sh --offline --bag-file \"\$(pwd)/data/automap_input/M2DGR/street_03_ros2\" --config system_config_M2DGR.yaml --gdb --clean
    bash run_automap.sh --run-only --no-rviz                   # 运行但不启动 RViz
    bash run_automap.sh --offline --bag-file /data/xxx --log-dir \$(pwd)/logs   # 日志保存到宿主机 ./logs/full.log

目录说明:
    - 工作空间: ${WORKSPACE_DIR}
    - 数据目录: ${DATA_DIR}
    - 输出目录: ${OUTPUT_DIR}（离线模式默认启动前清空，保证每次重新建图；可用 --no-clean-output 保留上次结果）
    - 日志目录: 默认 ${SCRIPT_DIR}/logs/run_YYYYMMDD_HHMMSS/（宿主机，每次运行独立目录）；--log-dir 可指定固定目录
        automap.log  全程总日志  build.log  编译  full.log  运行  clean.log  清理  image.log  镜像加载/构建

编译加速:
    - 默认使用宿主机 CPU 核数并行编译，并启用 Ninja 生成器；可设置环境变量覆盖线程数，例如: AUTOMAP_BUILD_JOBS=16 bash run_automap.sh --build-only --clean

Docker / GPU 环境变量（可选）:
    - AUTOMAP_DOCKER_IMAGE   覆盖默认镜像（默认 NGC Isaac ROS，适配 RTX 50 等；旧环境可设为 automap-env:humble）
    - AUTOMAP_GDB_DOCKER_IMAGE  带 --gdb/--gdb-frontend 时使用的镜像（默认 automap-pro:latest）
    - AUTOMAP_ENABLE_BUILD_STEPS  是否执行 build_inside_container 编译/依赖安装（默认 1=执行；设 0 可完全跳过）
    - AUTOMAP_ROS_DISTRO     容器内 ROS 发行版（默认随镜像推断：Isaac→jazzy，自建→humble）
    - AUTOMAP_IMAGE_ARCHIVE  本地 docker load 的 .tar 路径；设为空字符串可跳过 tar 仅 pull
    - AUTOMAP_PREFER_RUNTIME_SNAPSHOT  自建 humble + tar 流程默认 1（优先 load 快照）；NGC Isaac 默认镜像下默认 0（始终用 pull 的基础镜像，不写 runtime tar）
    - AUTOMAP_SAVE_RUNTIME_SNAPSHOT  同上：Isaac 默认 0（运行结束不 commit/save）；humble 默认 1
    - AUTOMAP_SAVE_SNAPSHOT_AFTER_BUILD  默认与 AUTOMAP_SAVE_RUNTIME_SNAPSHOT 相同：编译成功后将编译容器 commit+save 为同一路径 tar（下次直接 load）；置 0 可禁用。成功后会将本次会话的运行镜像改为快照标签，使同一次命令中的「运行」也基于最新层
    - AUTOMAP_SNAPSHOT_SAVE_MODE  默认 first：仅当尚无 automap-pro-runtime.tar 时保存（省磁盘/时间）；设为 always 则每次成功运行都更新快照 tar
    - AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME  编译阶段临时容器名（默认 automap-pro-build-snapshot）
    - AUTOMAP_RUNTIME_SNAPSHOT_TAR / AUTOMAP_RUNTIME_SNAPSHOT_TAG / AUTOMAP_RUNTIME_CONTAINER_NAME  可覆盖快照路径、镜像名、临时运行容器名
    - AUTOMAP_UBUNTU_MIRROR  容器内 apt 补充 universe 源（默认 http://mirrors.aliyun.com/ubuntu；需官方源可设 http://archive.ubuntu.com/ubuntu）

编译期（传入 build 容器，见 scripts/build_inside_container.sh）:
    - AUTOMAP_PREBUILT_INSTALL_DEPS=1  宿主机 workspace 已含完整 install_deps（如 libtorch、onnxruntime）：跳过 LibTorch 下载与 apt 安装 nvidia-cuda-toolkit；等价于默认 LIBTORCH_SKIP_DOWNLOAD=1 且 AUTOMAP_SKIP_CUDA_TOOLKIT_APT=1（Blackwell 不会删误装 libtorch）
    - 未显式设置 AUTOMAP_PREBUILT_INSTALL_DEPS 且宿主机已有 install_deps/libtorch 时，脚本会自动设为 1（换新基础镜像但保留工作空间时可避免重复拉取依赖）
    - AUTOMAP_ALWAYS_BUILD=1  每次启动均执行完整编译（忽略「已安装」跳过逻辑；与 --clean 不同，不删除 install_deps）
    - LIBTORCH_SKIP_DOWNLOAD=1  不下载 LibTorch（须已有 install_deps/libtorch）
    - AUTOMAP_SKIP_CUDA_TOOLKIT_APT=1  不 apt 安装 nvidia-cuda-toolkit（须已有 nvcc 或挂载 CUDA）
    - LIBTORCH_URL           固定 LibTorch zip URL（不设则按 GPU/nvcc 选包；默认从 download.pytorch.org/libtorch 拉取，可用 AUTOMAP_LIBTORCH_DOWNLOAD_BASE 换镜像根）
    - LIBTORCH_PREFER=cpu    强制下载 CPU 版 LibTorch（纯 CPU 机器）
    - ONNXRUNTIME_USE_CUDA   Isaac 镜像未设置时默认为 1（尝试 CUDA EP）；显式 0 可强制 CPU 构建 ORT（需 cuDNN 头文件才可能产出 GPU EP）
    - AUTOMAP_STRICT_BLACKWELL_STACK  默认 1：检测到 RTX 50 系时校验 CUDA≥12.8（优先 nvcc；NGC 无 nvcc 时用 nvidia-smi 的 CUDA Version）、LibTorch cu128、ONNX CUDA 时存在 cuDNN。设为 0 可跳过（不推荐）
    - AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED 默认 1：强制 libtorch / nvcc / libnvrtc 为同一 CUDA 小版本（例如都为 12.8），并在 Blackwell 上要求 libnvrtc>=12.8；不满足直接失败，避免 nvrtc -arch 运行时崩溃
运行期（传入 run 容器，SLOAM 语义）:
    - AUTOMAP_SLOAM_ONNX_CUDA   默认为 1（尝试 ORT CUDA EP）；设为 0 强制 CPU 推理；无 libonnxruntime_providers_cuda.so 时 C++ 内仍会回退 CPU
    - AUTOMAP_ONNXRUNTIME_LIB_DIR  显式指定含 libonnxruntime*.so 的目录（可选，默认同 ONNXRUNTIME_HOME/lib）

说明: 若 install_deps 里已是旧版 LibTorch / ORT，切换 CUDA 变体或升级 cu128（RTX 50）前请删除对应目录后重编:
    rm -rf automap_ws/install_deps/libtorch automap_ws/install_deps/onnxruntime
    若 LibTorch 从 2.5.x 升到 2.7.x（cu128）后 OverlapTransformer 加载失败，请用容器内脚本重新导出 overlapTransformer.pt。

LSK3DNet（hybrid / 验证脚本）:
    - --build-only 时默认启用 AUTOMAP_SETUP_LSK3DNET_VENV（容器内创建 install_deps/lsk3dnet_venv：torch+spconv+torch-scatter，与 nvcc 对齐）；跳过请设 AUTOMAP_SETUP_LSK3DNET_VENV=0
    - 非 --build-only 的编译流程仍默认为关闭，需 LSK venv 时请显式 AUTOMAP_SETUP_LSK3DNET_VENV=1
    - AUTOMAP_LSK3DNET_PYTHON       覆盖 semantic.lsk3dnet.python；若 venv 存在且未设，运行时会默认指向该 venv
    - LSK_CUDA_TAG / LSK_TORCH_VER   可选，强制 cu118|cu121|cu124 与 torch 版本（默认自动 + 2.5.1）
    - PIP_INDEX                      可选，覆盖默认 PyPI 镜像（默认清华 simple，见 scripts/automap_download_defaults.sh）
    - LSK_PYTORCH_WHL_INDEX          可选，覆盖 LSK venv 中 torch 的 --index-url（默认上海交大 pytorch-wheels/{cu}）

下载缓存与镜像（默认启用）:
    - 宿主机目录: thrid_party/automap_cache/ → 容器 /root/automap_download_cache（pip、LibTorch zip、ONNX git 快照、apt .deb、XDG 杂项缓存）；docker/automap-pro-runtime.tar 为可选「运行时快照」供下次 ensure_image 优先加载
    - AUTOMAP_USE_OFFICIAL_PYPI=1    不使用国内 PyPI 镜像（仍保留 pip 本地缓存）
    - AUTOMAP_PIP_INDEX              自定义 PyPI 镜像（默认阿里云 simple；官方: https://pypi.org/simple）
    - AUTOMAP_LIBTORCH_DOWNLOAD_BASE LibTorch zip 根路径（默认官方 download.pytorch.org/libtorch；有可用镜像时自设并保证目录结构与官方一致）
    - AUTOMAP_PYG_WHEEL_BASE         PyG torch-scatter 等 -f 索引根（默认 data.pyg.org/whl）
    - AUTOMAP_ONNXRUNTIME_GIT_URL    自定义 ONNX Runtime git 地址（默认经 ghfast.top 代理 GitHub）
故障排查:
    - 第三方库（GTSAM/Ceres/TEASER++/vikit）安装在 automap_ws/install_deps，--clean 不会删除；需强制重编第三方时请手动: rm -rf automap_ws/install_deps
    - 若编译报「CMakeCache.txt directory ... is different than ...」：脚本已自动检测并清理含 /workspace/ 的缓存；仍失败时请加 --clean 后重跑
    - 若 ros2 bag 报「yaml-cpp: bad conversion」：检查 bag 元数据或换 --storage-preset mcap 重录
    - 若 HBA 报 vector::_M_default_append：多为尚无 LIO 位姿，请确保 --config 与 bag 话题一致且 fast_livo 已正常输出
    - 若 automap_system 报 undefined symbol keyframeCount：需重新编译 automap_pro（已补全 SubMapManager 实现）

EOF
}

# 调试模式镜像策略：
# 带 --gdb / --gdb-frontend 时，按用户要求优先使用 automap-pro:latest（或 AUTOMAP_GDB_DOCKER_IMAGE）
apply_gdb_image_policy() {
    if [ "$RUN_AUTOMAP_UNDER_GDB" = true ] || [ "$RUN_FAST_LIVO_UNDER_GDB" = true ]; then
        IMAGE_NAME="${AUTOMAP_GDB_DOCKER_IMAGE}"
        AUTOMAP_BASE_DOCKER_IMAGE="${AUTOMAP_GDB_DOCKER_IMAGE}"
        # 调试场景固定用指定镜像，避免被 runtime snapshot 覆盖。
        AUTOMAP_PREFER_RUNTIME_SNAPSHOT=0
        print_info "GDB 调试模式：使用指定镜像 ${IMAGE_NAME}"
    fi
}

# ==================== 解析命令行参数 ====================
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --online)
                MODE="online"
                shift
                ;;
            --offline)
                MODE="offline"
                shift
                ;;
            --bag-file)
                BAG_FILE="$2"
                shift 2
                ;;
            --bag-rate)
                BAG_RATE="$2"
                shift 2
                ;;
            --config)
                CONFIG_FILE="$2"
                shift 2
                ;;
            --build-only)
                BUILD_ONLY=true
                shift
                ;;
            --run-only)
                RUN_ONLY=true
                shift
                ;;
            --no-rviz)
                USE_RVIZ=false
                RVIZ_CLI_SET=true
                shift
                ;;
            --rviz)
                USE_RVIZ=true
                RVIZ_CLI_SET=true
                shift
                ;;
            --no-external-frontend)
                USE_EXTERNAL_FRONTEND=false
                shift
                ;;
            --external-overlap)
                USE_EXTERNAL_OVERLAP=true
                shift
                ;;
            --clean)
                CLEAN_BUILD=true
                shift
                ;;
            --no-clean-output)
                CLEAN_OUTPUT_ON_OFFLINE=false
                shift
                ;;
            --log-dir)
                LOG_DIR="$2"
                LOG_DIR_USER_SET=true
                shift 2
                ;;
            --gdb)
                RUN_AUTOMAP_UNDER_GDB=true
                shift
                ;;
            --gdb-frontend)
                RUN_FAST_LIVO_UNDER_GDB=true
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                echo "[ERROR] 未知选项: $1"
                echo "使用 --help 查看帮助信息"
                exit 1
                ;;
        esac
    done
}

# ==================== 颜色输出函数 ====================
print_info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

print_warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

print_error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1"
}

print_header() {
    echo ""
    echo -e "\033[1;36m========================================\033[0m"
    echo -e "\033[1;36m$1\033[0m"
    echo -e "\033[1;36m========================================\033[0m"
}

# 为每行日志添加时间戳（宿主机/容器时间与宿主机同步后一致），便于排查与回放
add_timestamp() {
    while IFS= read -r line; do
        echo "$(date '+%Y-%m-%d %H:%M:%S') $line"
    done
    # read hits EOF with status 1; force success so pipefail reflects real upstream failures.
    return 0
}

# 检测 fast_livo 是否已安装到工作空间（merged / isolated / install_deps 分步安装均覆盖）
automap_fast_livo_installed() {
    local base="${WORKSPACE_DIR}"
    [ -f "${base}/install/share/fast_livo/package.xml" ] && return 0
    [ -f "${base}/install/fast_livo/share/fast_livo/package.xml" ] && return 0
    [ -f "${base}/install_deps/fast_livo/share/fast_livo/package.xml" ] && return 0
    [ -f "${base}/install_deps/fast_livo/fast_livo/share/fast_livo/package.xml" ] && return 0
    if command -v find >/dev/null 2>&1; then
        local _fl_roots=()
        [ -d "${base}/install" ] && _fl_roots+=("${base}/install")
        [ -d "${base}/install_deps/fast_livo" ] && _fl_roots+=("${base}/install_deps/fast_livo")
        if [ "${#_fl_roots[@]}" -gt 0 ]; then
            local _f
            _f=$(find "${_fl_roots[@]}" -path '*/share/fast_livo/package.xml' 2>/dev/null | head -1)
            [ -n "${_f}" ] && return 0
        fi
    fi
    return 1
}

# 编译后复检：读取 automap_system 的 NEEDED=libceres.so.X。
# - 若 install_deps/ceres 中存在同名 SONAME，则验证 install_deps 可满足依赖（避免运行期 dlopen 崩溃）
# - 若 install_deps/ceres 中不存在，则认为将使用镜像系统 Ceres（例如 Jazzy/noble 常为 libceres.so.4），仅告警不失败
automap_verify_ceres_needed_postbuild() {
    local so_path="${WORKSPACE_DIR}/install/automap_pro/lib/libautomap_system_component.so"
    if [ ! -f "${so_path}" ]; then
        print_error "编译后复检失败：未找到 ${so_path}"
        return 1
    fi
    if ! command -v readelf >/dev/null 2>&1; then
        print_warning "未找到 readelf，跳过 Ceres NEEDED 复检"
        return 0
    fi
    local needed
    needed="$(readelf -d "${so_path}" 2>/dev/null | sed -n "s/.*Shared library: \[\(libceres\.so\.[0-9][0-9]*\)\].*/\1/p" | head -n1)"
    if [ -z "${needed}" ]; then
        print_error "编译后复检失败：${so_path} 未检出 libceres.so.X 依赖"
        return 1
    fi
    local match=""
    [ -f "${WORKSPACE_DIR}/install_deps/ceres/lib/${needed}" ] && match="${WORKSPACE_DIR}/install_deps/ceres/lib/${needed}"
    [ -z "${match}" ] && [ -f "${WORKSPACE_DIR}/install_deps/ceres/lib64/${needed}" ] && match="${WORKSPACE_DIR}/install_deps/ceres/lib64/${needed}"
    if [ -z "${match}" ]; then
        print_warning "编译后复检提示：NEEDED=${needed}，install_deps/ceres 下无同名库（lib/lib64）。将按运行环境使用镜像系统 Ceres。"
        return 0
    fi
    print_success "✓ 编译后复检通过：automap_system NEEDED=${needed}，install_deps 可用库=${match}"
    return 0
}

# 容器与宿主机时间同步：挂载宿主机时区与时间（不修改镜像，仅脚本内生效）
TIME_VOLUMES="-v /etc/localtime:/etc/localtime:ro"
[ -f /etc/timezone ] && TIME_VOLUMES="${TIME_VOLUMES} -v /etc/timezone:/etc/timezone:ro"

# ==================== 检查依赖 ====================
check_dependencies() {
    print_header "检查依赖"

    # 检查 Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装，请先安装 Docker"
        exit 1
    fi
    print_success "✓ Docker 已安装"

    # 检查 Docker 是否运行
    if ! docker info &> /dev/null; then
        print_error "Docker 未运行，请先启动 Docker 服务"
        exit 1
    fi
    print_success "✓ Docker 服务运行中"

    # 检查 NVIDIA Docker 支持（使用当前工程镜像，避免与宿主机 CUDA 基线不一致）
    print_info "检测 GPU 与 NVIDIA 容器支持（启动临时容器执行 nvidia-smi）…"
    if ! docker run --rm --gpus all "${IMAGE_NAME}" nvidia-smi &> /dev/null; then
        print_warning "⚠ NVIDIA Docker 支持检查失败（镜像不存在时会误报；可先拉取/加载镜像后再试），可能影响 GPU 功能"
    else
        print_success "✓ NVIDIA Docker 支持正常"
    fi

    # 创建必要的目录（含宿主机日志目录，编译与运行日志均写入此处）
    print_info "创建必要的目录..."
    mkdir -p "${WORKSPACE_DIR}/src"
    mkdir -p "${DATA_DIR}"
    mkdir -p "${OUTPUT_DIR}"
    mkdir -p "${LOG_DIR}"

    print_success "✓ 目录创建完成"
    print_info "日志目录（宿主机）: ${LOG_DIR}"
    print_info "  → automap.log  全程总日志（脚本+编译+运行）  build.log  编译  full.log  运行  clean.log  清理  image.log  镜像加载/构建"
}

# ==================== 确保 Docker 镜像 ====================
ensure_image() {
    print_header "检查 Docker 镜像"
    print_info "阶段：镜像（load/tar/pull/build 可能耗时很长，终端无新输出时通常仍在进行）"

    # 1) 优先使用本地运行时快照（含上次成功运行后写入容器层的 apt 等；工作空间仍来自宿主机挂载）
    if [ "${AUTOMAP_PREFER_RUNTIME_SNAPSHOT:-1}" = "1" ]; then
        if [ -f "${AUTOMAP_RUNTIME_SNAPSHOT_TAR}" ]; then
            print_info "发现本地运行时快照 tar，优先加载: ${AUTOMAP_RUNTIME_SNAPSHOT_TAR}"
            print_info "docker load 进行中（大 tar 时可能数分钟无新行）…"
            set +e
            docker load -i "${AUTOMAP_RUNTIME_SNAPSHOT_TAR}" 2>&1 | add_timestamp | tee "${LOG_DIR}/image.log"
            SNAP_LOAD_EXIT=${PIPESTATUS[0]}
            set -e
            if [ "${SNAP_LOAD_EXIT}" -ne 0 ]; then
                print_error "快照 tar 加载失败（详见 ${LOG_DIR}/image.log）"
                exit 1
            fi
            if docker image inspect "${AUTOMAP_RUNTIME_SNAPSHOT_TAG}" &> /dev/null; then
                IMAGE_NAME="${AUTOMAP_RUNTIME_SNAPSHOT_TAG}"
                print_success "✓ 已使用快照镜像 ${IMAGE_NAME}（跳过基础镜像 pull）"
                return 0
            fi
            print_error "docker load 成功但未找到镜像标签 ${AUTOMAP_RUNTIME_SNAPSHOT_TAG}（见 ${LOG_DIR}/image.log）"
            exit 1
        elif docker image inspect "${AUTOMAP_RUNTIME_SNAPSHOT_TAG}" &> /dev/null; then
            IMAGE_NAME="${AUTOMAP_RUNTIME_SNAPSHOT_TAG}"
            print_success "✓ 使用本地已有快照镜像 ${IMAGE_NAME}"
            return 0
        fi
    fi

    IMAGE_NAME="${AUTOMAP_BASE_DOCKER_IMAGE}"

    # 2) 检查基础镜像是否已存在
    if docker image inspect "${IMAGE_NAME}" &> /dev/null; then
        print_success "✓ 镜像 ${IMAGE_NAME} 已存在于环境中"
        return 0
    fi

    # 尝试从归档文件加载
    if [ -f "${IMAGE_ARCHIVE}" ]; then
        print_info "从归档文件加载镜像: ${IMAGE_ARCHIVE}（输出同时写入 ${LOG_DIR}/image.log）"
        set +e
        docker load -i "${IMAGE_ARCHIVE}" 2>&1 | add_timestamp | tee "${LOG_DIR}/image.log"
        LOAD_EXIT=${PIPESTATUS[0]}
        set -e
        if [ "$LOAD_EXIT" -eq 0 ]; then
            print_success "✓ 镜像加载完成"
            return 0
        else
            print_error "镜像加载失败（详见 ${LOG_DIR}/image.log）"
            exit 1
        fi
    fi

    # 远程 NGC 等镜像：直接拉取（不自建 docker/dockerfile）
    if [[ "${IMAGE_NAME}" == nvcr.io/* ]] || [[ "${IMAGE_NAME}" == ghcr.io/* ]]; then
        print_info "拉取远程镜像: ${IMAGE_NAME}（输出同时写入 ${LOG_DIR}/image.log）"
        print_info "docker pull 进行中（镜像体积大时可能 10–30+ 分钟，进度见 image.log）…"
        set +e
        docker pull "${IMAGE_NAME}" 2>&1 | add_timestamp | tee "${LOG_DIR}/image.log"
        PULL_EXIT=${PIPESTATUS[0]}
        set -e
        if [ "$PULL_EXIT" -eq 0 ] && docker image inspect "${IMAGE_NAME}" &> /dev/null; then
            print_success "✓ 镜像拉取完成"
            return 0
        fi
        print_error "镜像拉取失败（详见 ${LOG_DIR}/image.log）"
        exit 1
    fi

    # 构建镜像（仅针对本仓库 docker/dockerfile → automap-env:humble）
    print_warning "⚠ 镜像不存在，开始构建..."
    print_info "构建时间预计: 30-45 分钟（输出同时写入 ${LOG_DIR}/image.log）"

    cd "${SCRIPT_DIR}/docker"
    set +e
    docker build -t "${IMAGE_NAME}" . 2>&1 | add_timestamp | tee "${LOG_DIR}/image.log"
    IMAGE_EXIT=${PIPESTATUS[0]}
    set -e

    if [ "$IMAGE_EXIT" -ne 0 ]; then
        print_error "镜像构建失败（详见 ${LOG_DIR}/image.log）"
        exit 1
    fi

    print_success "✓ 镜像构建完成"
}

# ==================== 准备工作空间 ====================
prepare_workspace() {
    print_header "准备工作空间"

    # 创建符号链接
    print_info "创建符号链接..."
    if [ ! -L "${WORKSPACE_DIR}/src/automap_pro" ]; then
        ln -sf "${PROJECT_DIR}" "${WORKSPACE_DIR}/src/automap_pro"
        print_success "✓ 符号链接创建完成: ${WORKSPACE_DIR}/src/automap_pro -> ${PROJECT_DIR}"
    else
        print_info "符号链接已存在"
    fi

    # 可选：fast-livo2-humble（仓库根 或 automap_pro/src/modular 下）
    FAST_LIVO_DIR=""
    FAST_LIVO_LINK_RELATIVE=""   # 非空时使用相对路径链入（容器内可解析到 src/automap_pro/...）
    [ -d "${SCRIPT_DIR}/fast-livo2-humble" ] && FAST_LIVO_DIR="${SCRIPT_DIR}/fast-livo2-humble"
    [ -z "${FAST_LIVO_DIR}" ] && [ -d "${SCRIPT_DIR}/fast-livo2-humble.disabled" ] && FAST_LIVO_DIR="${SCRIPT_DIR}/fast-livo2-humble.disabled"
    [ -z "${FAST_LIVO_DIR}" ] && [ -d "${PROJECT_DIR}/src/modular/fast-livo2-humble" ] && FAST_LIVO_DIR="${PROJECT_DIR}/src/modular/fast-livo2-humble" && FAST_LIVO_LINK_RELATIVE="automap_pro/src/modular/fast-livo2-humble"
    [ -z "${FAST_LIVO_DIR}" ] && [ -d "${PROJECT_DIR}/src/modular/fast-livo2-humble.disabled" ] && FAST_LIVO_DIR="${PROJECT_DIR}/src/modular/fast-livo2-humble.disabled" && FAST_LIVO_LINK_RELATIVE="automap_pro/src/modular/fast-livo2-humble.disabled"
    if [ -n "${FAST_LIVO_DIR}" ]; then
        # 若已有链接/目录但指向无效（如曾指向仓库根下已删除的目录），则重建
        NEED_LINK=true
        if [ -L "${WORKSPACE_DIR}/src/fast_livo" ] || [ -d "${WORKSPACE_DIR}/src/fast_livo" ]; then
            [ -d "${WORKSPACE_DIR}/src/fast_livo" ] && NEED_LINK=false || true
        fi
        if [ "$NEED_LINK" = true ]; then
            rm -f "${WORKSPACE_DIR}/src/fast_livo"
            if [ -n "${FAST_LIVO_LINK_RELATIVE}" ]; then
                (cd "${WORKSPACE_DIR}/src" && ln -sf "${FAST_LIVO_LINK_RELATIVE}" "fast_livo")
            else
                ln -sf "${FAST_LIVO_DIR}" "${WORKSPACE_DIR}/src/fast_livo"
            fi
            print_success "✓ 已链入 $(basename "${FAST_LIVO_DIR}") → src/fast_livo"
        else
            print_info "fast_livo 已存在"
        fi
    fi
}

# ==================== 编译项目 ====================
build_project() {
    print_header "编译项目"

    # 如果是 run-only 模式，跳过编译
    if [ "$RUN_ONLY" = true ]; then
        print_info "--run-only 模式，跳过编译"
        return 0
    fi

    # 清理编译产物（在容器内执行，避免 build 由容器 root 创建导致主机用户无法删除）
    if [ "$CLEAN_BUILD" = true ]; then
        print_info "清理编译产物...（输出同时写入 ${LOG_DIR}/clean.log）"
        set +e
        docker run --rm \
            ${TIME_VOLUMES} \
            -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
            "${IMAGE_NAME}" \
            /bin/bash -c "set -e; rm -rf /root/automap_ws/build /root/automap_ws/build_teaserpp /root/automap_ws/build_sophus /root/automap_ws/build_ceres /root/automap_ws/install /root/automap_ws/log; echo 'cleaned (install_deps 保留)'" 2>&1 | add_timestamp | tee "${LOG_DIR}/clean.log"
        CLEAN_EXIT=${PIPESTATUS[0]}
        set -e
        if [ "$CLEAN_EXIT" -eq 0 ]; then
            print_success "✓ 清理完成"
        else
            print_error "✗ 清理失败（详见 ${LOG_DIR}/clean.log；若曾用 sudo 编译，请在本机执行: sudo rm -rf ${WORKSPACE_DIR}/build ${WORKSPACE_DIR}/build_teaserpp ${WORKSPACE_DIR}/install ${WORKSPACE_DIR}/log）"
            exit 1
        fi
    fi

    # 检查是否已经编译（automap_pro 必选；若存在 fast_livo 源码则其也需已安装）
    NEED_BUILD=false
    if [ "${AUTOMAP_ALWAYS_BUILD:-0}" = "1" ]; then
        NEED_BUILD=true
        print_info "AUTOMAP_ALWAYS_BUILD=1：强制完整编译（不删除 install_deps）"
    fi
    if [ ! -f "${WORKSPACE_DIR}/install/automap_pro/share/automap_pro/package.xml" ]; then
        NEED_BUILD=true
    fi
    # 若 automap_pro 核心源文件比 install 的 .so 新，强制触发重新编译（避免 undefined symbol buildGlobalMap）
    AUTOMAP_SRC="${PROJECT_DIR}/src/submap/submap_manager.cpp"
    AUTOMAP_SO="${WORKSPACE_DIR}/install/automap_pro/lib/libautomap_system_component.so"
    if [ "$NEED_BUILD" = false ] && [ -f "${AUTOMAP_SRC}" ] && [ -f "${AUTOMAP_SO}" ]; then
        if [ "${AUTOMAP_SRC}" -nt "${AUTOMAP_SO}" ]; then
            NEED_BUILD=true
            print_info "检测到 automap_pro 源文件已更新，将重新编译（避免 undefined symbol）"
        fi
    fi
    HAVE_FAST_LIVO_SRC=false
    [ -d "${SCRIPT_DIR}/fast-livo2-humble" ] || [ -d "${SCRIPT_DIR}/fast-livo2-humble.disabled" ] || [ -d "${PROJECT_DIR}/src/modular/fast-livo2-humble" ] || [ -d "${PROJECT_DIR}/src/modular/fast-livo2-humble.disabled" ] && HAVE_FAST_LIVO_SRC=true
    if [ "$NEED_BUILD" = false ] && [ "$HAVE_FAST_LIVO_SRC" = true ]; then
        # merged: install/share/fast_livo/；isolated: install/fast_livo/share/...；分步安装到 install_deps/fast_livo
        if ! automap_fast_livo_installed; then
            NEED_BUILD=true
            print_info "检测到 fast-livo2-humble 源码，但未找到 fast_livo 安装产物，将执行完整编译"
        fi
    fi
    # Ceres ABI/SONAME 预检：若 install 的 automap_system 依赖 libceres.so.X，
    # 但 install_deps/ceres 下无对应 SONAME，会在运行期 class_loader/dlopen 崩溃。
    # 这里提前触发重编译，让链接阶段与 install_deps/ceres 保持一致。
    if [ "$NEED_BUILD" = false ] && [ -f "${AUTOMAP_SO}" ]; then
        if command -v readelf >/dev/null 2>&1; then
            CERES_NEEDED="$(readelf -d "${AUTOMAP_SO}" 2>/dev/null | sed -n "s/.*Shared library: \[\(libceres\.so\.[0-9][0-9]*\)\].*/\1/p" | head -n1)"
            if [ -n "${CERES_NEEDED}" ]; then
                CERES_MATCH=""
                [ -f "${WORKSPACE_DIR}/install_deps/ceres/lib/${CERES_NEEDED}" ] && CERES_MATCH="${WORKSPACE_DIR}/install_deps/ceres/lib/${CERES_NEEDED}"
                [ -z "${CERES_MATCH}" ] && [ -f "${WORKSPACE_DIR}/install_deps/ceres/lib64/${CERES_NEEDED}" ] && CERES_MATCH="${WORKSPACE_DIR}/install_deps/ceres/lib64/${CERES_NEEDED}"
                if [ -z "${CERES_MATCH}" ]; then
                    NEED_BUILD=true
                    print_warning "检测到 automap_system 依赖 ${CERES_NEEDED}，但 install_deps/ceres 未找到同名库；将重新编译以消除 Ceres SONAME 不匹配"
                    print_info "当前 install_deps/ceres 可用库: $(ls "${WORKSPACE_DIR}/install_deps/ceres/lib"/libceres.so* "${WORKSPACE_DIR}/install_deps/ceres/lib64"/libceres.so* 2>/dev/null | xargs -r -n1 basename | tr '\n' ' ')"
                fi
            fi
        fi
    fi
    if [ "$NEED_BUILD" = false ] && [ "$CLEAN_BUILD" = false ]; then
        print_info "项目已编译，跳过编译步骤"
        print_info "如需重新编译，请使用 --clean 参数"
        return 0
    fi

    # 与 AUTOMAP_SAVE_RUNTIME_SNAPSHOT 默认一致：编译成功后把容器层写入 automap-pro-runtime.tar，下次 ensure_image 直接 load
    _SAVE_SNAP_AFTER_BUILD="${AUTOMAP_SAVE_SNAPSHOT_AFTER_BUILD:-${AUTOMAP_SAVE_RUNTIME_SNAPSHOT:-1}}"
    BUILD_DOCKER_EXTRA=( --rm )
    if [ "${_SAVE_SNAP_AFTER_BUILD}" = "1" ]; then
        docker rm -f "${AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME}" 2>/dev/null || true
        BUILD_DOCKER_EXTRA=( --name "${AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME}" )
        print_info "编译成功后将保存 Docker 快照至 ${AUTOMAP_RUNTIME_SNAPSHOT_TAR}（AUTOMAP_SAVE_SNAPSHOT_AFTER_BUILD=${_SAVE_SNAP_AFTER_BUILD}；置 0 可禁用）"
    fi

    print_info "开始编译项目..."
    BUILD_JOBS="${AUTOMAP_BUILD_JOBS:-$(nproc)}"
    print_info "并行编译: ${BUILD_JOBS} 线程（可通过环境变量 AUTOMAP_BUILD_JOBS 覆盖，如 AUTOMAP_BUILD_JOBS=16）"
    print_info "预计编译时间: 5-10 分钟（使用 Ninja 时更快）"
    print_info "进度说明：build.log 中带 [PROGRESS] 的行表示当前阶段；apt/ONNX 首次编译等可能长时间无新行，属正常"

    # 挂载仓库根目录，供容器内链入四项目并编译
    MAPPING_MOUNT="-v ${SCRIPT_DIR}:/root/mapping:ro"
    FAST_LIVO_MOUNT=""
    FAST_LIVO_DIR=""
    [ -d "${SCRIPT_DIR}/fast-livo2-humble" ] && FAST_LIVO_DIR="${SCRIPT_DIR}/fast-livo2-humble"
    [ -z "${FAST_LIVO_DIR}" ] && [ -d "${SCRIPT_DIR}/fast-livo2-humble.disabled" ] && FAST_LIVO_DIR="${SCRIPT_DIR}/fast-livo2-humble.disabled"
    [ -z "${FAST_LIVO_DIR}" ] && [ -d "${PROJECT_DIR}/src/modular/fast-livo2-humble" ] && FAST_LIVO_DIR="${PROJECT_DIR}/src/modular/fast-livo2-humble"
    [ -z "${FAST_LIVO_DIR}" ] && [ -d "${PROJECT_DIR}/src/modular/fast-livo2-humble.disabled" ] && FAST_LIVO_DIR="${PROJECT_DIR}/src/modular/fast-livo2-humble.disabled"
    if [ -n "${FAST_LIVO_DIR}" ]; then
        # 必须挂载为容器内 src/fast_livo，否则 colcon 会把「指向 automap_pro/...」的符号链接当成 automap_pro 子目录，忽略 fast_livo 包
        FAST_LIVO_MOUNT="-v ${FAST_LIVO_DIR}:/root/automap_ws/src/fast_livo:ro"
        print_info "将编译 fast_livo ($(basename "${FAST_LIVO_DIR}"))"
    fi

    # thrid_party：优先使用含 Sophus 的路径（本仓库为 automap_pro/thrid_party），否则仓库根
    THRID_PARTY_DIR="${PROJECT_DIR}/thrid_party"
    [ ! -d "${THRID_PARTY_DIR}/Sophus" ] && [ -d "${SCRIPT_DIR}/thrid_party" ] && THRID_PARTY_DIR="${SCRIPT_DIR}/thrid_party"
    THRID_PARTY_MOUNT="-v ${THRID_PARTY_DIR}:/root/automap_ws/src/thrid_party:ro"

    # 编译输出同时写入宿主机日志目录，便于排查（管道后需用 PIPESTATUS 获取 docker 退出码）
    print_info "编译日志将保存到: ${LOG_DIR}/build.log"
    [ -f "${SCRIPT_DIR}/scripts/build_inside_container.sh" ] && chmod +x "${SCRIPT_DIR}/scripts/build_inside_container.sh"
    # Isaac / NGC ROS 镜像：未显式设置 ONNXRUNTIME_USE_CUDA 时默认尝试 CUDA EP（无 cuDNN 时 build 脚本会回退 CPU）
    if [ -z "${ONNXRUNTIME_USE_CUDA+x}" ]; then
        _ONNXRUNTIME_USE_CUDA_VAL=0
        if [[ "${IMAGE_NAME}" == *"/isaac/"* ]] || [[ "${IMAGE_NAME}" == *"isaac/ros"* ]]; then
            _ONNXRUNTIME_USE_CUDA_VAL=1
            print_info "Isaac 镜像：默认 ONNXRUNTIME_USE_CUDA=1（SLOAM 语义 GPU EP；缺少 cuDNN 时构建会回退 CPU）"
        fi
    else
        _ONNXRUNTIME_USE_CUDA_VAL="${ONNXRUNTIME_USE_CUDA}"
    fi
    set +e
    # pipefail：管道以 tee 结束时默认 $? 为 0，会误判成功；子 shell + pipefail 使 docker 非零即失败
    (
      set -o pipefail
      docker run "${BUILD_DOCKER_EXTRA[@]}" \
          ${TIME_VOLUMES} \
          ${AUTOMAP_CACHE_MOUNT} \
          --gpus all \
          --net=host \
          -e DISPLAY=$DISPLAY \
          -e HOST_NPROC=$(nproc) \
          -e AUTOMAP_BUILD_JOBS="${AUTOMAP_BUILD_JOBS:-$(nproc)}" \
          -e AUTOMAP_ROS_DISTRO="${AUTOMAP_ROS_DISTRO}" \
          -e AUTOMAP_DOWNLOAD_CACHE=/root/automap_download_cache \
          -e PIP_CACHE_DIR=/root/automap_download_cache/pip \
          -e AUTOMAP_UBUNTU_MIRROR="${AUTOMAP_UBUNTU_MIRROR:-http://mirrors.aliyun.com/ubuntu}" \
          -e AUTOMAP_LIBTORCH_DOWNLOAD_BASE="${AUTOMAP_LIBTORCH_DOWNLOAD_BASE:-https://download.pytorch.org/libtorch}" \
          -e AUTOMAP_USE_OFFICIAL_LIBTORCH="${AUTOMAP_USE_OFFICIAL_LIBTORCH:-0}" \
          -e AUTOMAP_PYG_WHEEL_BASE="${AUTOMAP_PYG_WHEEL_BASE:-https://data.pyg.org/whl}" \
          -e AUTOMAP_USE_OFFICIAL_PYPI="${AUTOMAP_USE_OFFICIAL_PYPI:-0}" \
          -e AUTOMAP_PIP_INDEX="${AUTOMAP_PIP_INDEX:-}" \
          -e AUTOMAP_ONNXRUNTIME_GIT_URL="${AUTOMAP_ONNXRUNTIME_GIT_URL:-}" \
          -e LSK_PYTORCH_WHL_INDEX="${LSK_PYTORCH_WHL_INDEX:-}" \
          -e LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE="${LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE:-}" \
          -e LIBTORCH_URL="${LIBTORCH_URL:-}" \
          -e LIBTORCH_PREFER="${LIBTORCH_PREFER:-}" \
          -e AUTOMAP_PREBUILT_INSTALL_DEPS="${AUTOMAP_PREBUILT_INSTALL_DEPS:-0}" \
          -e LIBTORCH_SKIP_DOWNLOAD="${LIBTORCH_SKIP_DOWNLOAD:-0}" \
          -e AUTOMAP_SKIP_CUDA_TOOLKIT_APT="${AUTOMAP_SKIP_CUDA_TOOLKIT_APT:-0}" \
          -e ONNXRUNTIME_USE_CUDA="${_ONNXRUNTIME_USE_CUDA_VAL}" \
          -e AUTOMAP_STRICT_BLACKWELL_STACK="${AUTOMAP_STRICT_BLACKWELL_STACK:-1}" \
          -e AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED="${AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED:-1}" \
          -e CUDA_HOME="${CUDA_HOME:-}" \
          -e CUDNN_HOME="${CUDNN_HOME:-}" \
          -e AUTOMAP_SETUP_LSK3DNET_VENV="${AUTOMAP_SETUP_LSK3DNET_VENV:-0}" \
          -e LSK_CUDA_TAG="${LSK_CUDA_TAG:-}" \
          -e LSK_TORCH_VER="${LSK_TORCH_VER:-}" \
          $( [ -n "${PIP_INDEX+x}" ] && [ -n "${PIP_INDEX}" ] && printf '%s\n' "-e PIP_INDEX=${PIP_INDEX}" ) \
          -e OMP_NUM_THREADS=1 \
          -e EIGEN_NUM_THREADS=1 \
          -e MKL_NUM_THREADS=1 \
          -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
          -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
          -v "${PROJECT_DIR}:/root/automap_ws/src/automap_pro:ro" \
          -v "${SCRIPT_DIR}/scripts:/root/scripts:ro" \
          ${THRID_PARTY_MOUNT} \
          ${MAPPING_MOUNT} \
          ${FAST_LIVO_MOUNT} \
          "${IMAGE_NAME}" \
          /bin/bash /root/scripts/build_inside_container.sh 2>&1 | add_timestamp | tee "${LOG_DIR}/build.log"
    )
    BUILD_EXIT=$?
    set -e

    if [ "$BUILD_EXIT" -ne 0 ]; then
        if [ "${_SAVE_SNAP_AFTER_BUILD}" = "1" ]; then
            docker rm -f "${AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME}" 2>/dev/null || true
        fi
        print_error "✗ 项目编译失败（详见 ${LOG_DIR}/build.log）"
        exit 1
    fi

    print_success "✓ 项目编译成功"
    if ! automap_verify_ceres_needed_postbuild; then
        if [ "${_SAVE_SNAP_AFTER_BUILD}" = "1" ]; then
            docker rm -f "${AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME}" 2>/dev/null || true
        fi
        print_error "✗ Ceres 编译后复检未通过。建议执行: bash run_automap.sh --clean（不要 --run-only）"
        exit 1
    fi
    if [ "${_SAVE_SNAP_AFTER_BUILD}" = "1" ]; then
        if automap_commit_and_save_runtime_snapshot "${AUTOMAP_BUILD_SNAPSHOT_CONTAINER_NAME}" "post-build"; then
            IMAGE_NAME="${AUTOMAP_RUNTIME_SNAPSHOT_TAG}"
            print_info "本次后续运行将使用刚保存的快照镜像: ${IMAGE_NAME}"
        fi
    fi
}

# ==================== 运行系统 ====================
run_system() {
    # 如果是 build-only 模式，跳过运行
    if [ "$BUILD_ONLY" = true ]; then
        print_info "--build-only 模式，跳过运行"
        return 0
    fi

    print_header "启动建图系统"

    # 配置文件路径：支持 --config 指定（如 system_config_M2DGR.yaml）
    if [ -n "${CONFIG_FILE}" ]; then
        if [ -z "${CONFIG_FILE##*/*}" ]; then
            CONFIG_PATH="${CONFIG_FILE}"
        else
            CONFIG_PATH="${PROJECT_DIR}/config/${CONFIG_FILE}"
        fi
        [ ! -f "${CONFIG_PATH}" ] && print_error "配置文件不存在: ${CONFIG_PATH}" && exit 1
        CONFIG_BASENAME="$(basename "${CONFIG_PATH}")"
        CONTAINER_CONFIG_PATH="/root/automap_ws/src/automap_pro/config/${CONFIG_BASENAME}"
    else
        CONFIG_PATH="${PROJECT_DIR}/config/system_config.yaml"
        CONTAINER_CONFIG_PATH="/root/automap_ws/src/automap_pro/config/system_config.yaml"
    fi
    print_info "配置文件: ${CONFIG_PATH}"

    # 离线模式：默认在启动前清空输出目录，保证每次都是重新建图（不沿用上次 global_map.pcd / session_* 等）
    if [ "$MODE" = "offline" ] && [ "$CLEAN_OUTPUT_ON_OFFLINE" = true ]; then
        if [ -d "${OUTPUT_DIR}" ]; then
            print_info "清空输出目录以重新建图: ${OUTPUT_DIR}"
            rm -rf "${OUTPUT_DIR:?}"/* 2>/dev/null || true
            # 若目录内仍有文件（多为容器/root 创建），用 sudo 再删一次以彻底清空
            if [ -n "$(ls -A "${OUTPUT_DIR}" 2>/dev/null)" ]; then
                if sudo -n rm -rf "${OUTPUT_DIR:?}"/* 2>/dev/null; then
                    print_success "✓ 输出目录已清空（已使用 sudo 删除无权限文件），本次为全新建图"
                else
                    print_warning "⚠ 部分文件无权限删除且 sudo 不可用或需密码。请手动执行后重试："
                    echo "    sudo rm -rf ${OUTPUT_DIR}/*"
                    print_warning "或使用 --no-clean-output 跳过清空（将沿用目录内已有数据）。建图将继续启动。"
                fi
            else
                print_success "✓ 输出目录已清空，本次为全新建图"
            fi
        fi
        mkdir -p "${OUTPUT_DIR}"
    fi

    # 构建启动命令
    # 离线模式强制使用「源码」下的 launch 文件，避免 install 目录中旧版 launch 固定使用 system_config.yaml 导致 --config 失效
    LAUNCH_FILE="automap_online.launch.py"
    LAUNCH_SPEC="automap_pro automap_online.launch.py"
    LAUNCH_ARGS="config:=${CONFIG_PATH}"

    if [ "$MODE" = "offline" ]; then
        LAUNCH_FILE="automap_offline.launch.py"
        # 离线模式强制使用「源码」下的 launch 文件，确保 run-only 时也能立即生效（不依赖 install 目录是否更新）。
        # 这样也便于将 patched config 写入 output_dir/run_*（宿主机可见），定位 system/api_version 丢失等问题。
        LAUNCH_SPEC="/root/automap_ws/src/automap_pro/launch/automap_offline.launch.py"
        if [ -n "$BAG_FILE" ]; then
            LAUNCH_ARGS="${LAUNCH_ARGS} bag_file:=${BAG_FILE}"
            print_info "离线模式，回放: ${BAG_FILE}"
        else
            print_warning "⚠ 未指定 rosbag，使用默认: ${DATA_DIR}/mapping"
            LAUNCH_ARGS="${LAUNCH_ARGS} bag_file:=${DATA_DIR}/mapping"
        fi
    else
        LAUNCH_SPEC="automap_pro automap_online.launch.py"
        print_info "在线模式，实时建图"
    fi

    # 是否启动 RViz
    RVIZ_ARG="use_rviz:=$(echo "$USE_RVIZ" | tr '[:upper:]' '[:lower:]')"
    LAUNCH_ARGS="${LAUNCH_ARGS} ${RVIZ_ARG}"

    # 是否使用外部前端 / 外部 OverlapTransformer
    EXT_FRONTEND_ARG="use_external_frontend:=$(echo "$USE_EXTERNAL_FRONTEND" | tr '[:upper:]' '[:lower:]')"
    EXT_OVERLAP_ARG="use_external_overlap:=$(echo "$USE_EXTERNAL_OVERLAP" | tr '[:upper:]' '[:lower:]')"
    LAUNCH_ARGS="${LAUNCH_ARGS} ${EXT_FRONTEND_ARG} ${EXT_OVERLAP_ARG}"

    # 容器内使用的启动参数（config 用容器内路径；bag_file 用 /data 挂载路径）
    CONTAINER_LAUNCH_ARGS="config:=${CONTAINER_CONFIG_PATH} rate:=${BAG_RATE} ${RVIZ_ARG} ${EXT_FRONTEND_ARG} ${EXT_OVERLAP_ARG}"
    # 传入容器，在 ros2 launch 前对 bag 目录再跑一次 metadata 修复（补宿主机权限或未修复之隙）
    CONTAINER_OFFLINE_BAG_PATH=""
    if [ "$MODE" = "offline" ] && [ -n "$BAG_FILE" ]; then
        if [[ "$BAG_FILE" == "${DATA_DIR}"* ]]; then
            CONTAINER_BAG_FILE="/data/${BAG_FILE#${DATA_DIR}/}"
        else
            CONTAINER_BAG_FILE="$BAG_FILE"
        fi
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} bag_file:=${CONTAINER_BAG_FILE}"
        CONTAINER_OFFLINE_BAG_PATH="${CONTAINER_BAG_FILE}"
        # 修复 ros2 bag metadata.yaml 中 offered_qos_profiles: [] 导致的 yaml-cpp "bad conversion"（line 25 附近）
        BAG_DIR="$BAG_FILE"
        [ -f "$BAG_DIR" ] && BAG_DIR="$(dirname "$BAG_DIR")"
        BAG_META="${BAG_DIR}/metadata.yaml"
        if [ -d "$BAG_DIR" ] && [ -f "$BAG_META" ]; then
            print_info "[BAG] bag_dir=${BAG_DIR} metadata=${BAG_META} writable=$([ -w "$BAG_META" ] && echo 'yes' || echo 'no')"
            # 尽量在宿主机可写，以便修复脚本生效（避免容器内创建导致宿主机不可写）
            if [ ! -w "$BAG_META" ]; then
                if chmod u+w "$BAG_META" 2>/dev/null; then
                    print_info "[BAG] 已 chmod u+w metadata.yaml，继续尝试修复"
                else
                    print_warning "[BAG] metadata.yaml 不可写（可能由容器 root 创建），将尝试修复；若失败请宿主机执行: chmod u+w ${BAG_META} && python3 scripts/fix_ros2_bag_metadata.py ${BAG_DIR}"
                fi
            fi
            FIX_SCRIPT="${SCRIPT_DIR}/scripts/fix_ros2_bag_metadata.py"
            if [ -f "$FIX_SCRIPT" ]; then
                if ROS_DISTRO="${AUTOMAP_ROS_DISTRO}" python3 "$FIX_SCRIPT" "$BAG_DIR" --qos-style auto --ros-distro "${AUTOMAP_ROS_DISTRO}" --verify-with-ros2-info 2>/dev/null; then
                    print_info "[BAG] 已修复 metadata.yaml（避免 ros2 bag play bad conversion）"
                else
                    FIX_ERR="$?"
                    if [ ! -w "$BAG_META" ]; then
                        print_warning "[BAG] 修复未执行或未改动（exit=$FIX_ERR）。若出现 bad conversion 请执行: chmod u+w ${BAG_META} && python3 $FIX_SCRIPT $BAG_DIR"
                    else
                        print_info "[BAG] metadata 无需修改或修复脚本未改动: ${BAG_META}"
                    fi
                fi
            else
                print_info "[BAG] 未找到 $FIX_SCRIPT，若出现 'Exception on parsing info file: bad conversion' 请手动执行: python3 scripts/fix_ros2_bag_metadata.py $BAG_DIR"
            fi
            # 预检查：宿主机用 Python 解析 metadata，便于提前发现 bad conversion（yaml-cpp 报错多在 line 25 附近）
            if python3 -c "
import sys
try:
    import yaml
except ImportError:
    sys.exit(0)
p = sys.argv[1]
try:
    with open(p, encoding='utf-8') as f:
        yaml.safe_load(f)
except Exception as e:
    print('[BAG] [PRECHECK] metadata 解析异常:', str(e), file=sys.stderr)
    sys.exit(1)
" "$BAG_META"; then
                print_info "[BAG] [PRECHECK] metadata.yaml 解析通过（宿主机），若容器内仍报 bad conversion 请检查挂载与权限"
                print_info "[BAG] [DIAG] precheck=metadata_yaml_parse result=pass bag_dir=${BAG_DIR}"
            else
                print_warning "[BAG] [PRECHECK] metadata.yaml 解析失败（将导致 ros2 bag play 报 bad conversion，上方有解析异常详情）"
                print_warning "[BAG] 建议宿主机执行: chmod u+w ${BAG_META} && python3 $FIX_SCRIPT $BAG_DIR"
                print_error "[BAG] [DIAG] precheck=metadata_yaml_parse result=fail impact=bag_play_will_fail action=abort_before_launch"
                exit 1
            fi
            # 高优先级预检查：若宿主机可用 ros2，直接用 ros2 bag info 验证可播放性（与 ros2 bag play 同解析链路）
            if command -v ros2 >/dev/null 2>&1; then
                if ros2 bag info "$BAG_DIR" >/tmp/automap_bag_info_precheck.log 2>&1; then
                    print_info "[BAG] [PRECHECK] ros2 bag info 解析通过（与 ros2 bag play 一致）"
                    print_info "[BAG] [DIAG] precheck=ros2_bag_info result=pass parser_chain=rosbag2_storage+yaml-cpp"
                else
                    print_error "[BAG] [PRECHECK] ros2 bag info 失败，关键输出：$(sed -n '1,3p' /tmp/automap_bag_info_precheck.log | tr '\n' ' ')"
                    print_error "[BAG] metadata 仍不可被 rosbag2 解析，已中止启动；请先修复 ${BAG_META}"
                    print_error "[BAG] [DIAG] precheck=ros2_bag_info result=fail impact=no_sensor_data action=abort_before_launch"
                    exit 1
                fi
            else
                print_warning "[BAG] [PRECHECK] 宿主机无 ros2 命令，跳过 ros2 bag info 预检查（将依赖 launch 阶段校验）"
                print_warning "[BAG] [DIAG] precheck=ros2_bag_info result=skipped reason=no_ros2_on_host fallback=launch_runtime_guard"
            fi
        else
            print_info "[BAG] 未检测到目录或 metadata.yaml（path=${BAG_DIR}），跳过 metadata 修复与预检查"
        fi
    elif [ "$MODE" = "offline" ]; then
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} bag_file:=/data/mapping"
        CONTAINER_OFFLINE_BAG_PATH="/data/mapping"
    fi
    if [ "$RUN_AUTOMAP_UNDER_GDB" = true ]; then
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} run_automap_under_gdb:=true"
        LAUNCH_ARGS="${LAUNCH_ARGS} run_automap_under_gdb:=true"
        print_info "GDB 调试: automap_system_node 将在 GDB 下运行，崩溃时自动打印 bt full"
    fi
    if [ "$RUN_FAST_LIVO_UNDER_GDB" = true ]; then
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} run_fast_livo_under_gdb:=true"
        LAUNCH_ARGS="${LAUNCH_ARGS} run_fast_livo_under_gdb:=true"
        print_info "GDB 调试: fastlivo_mapping 将在 GDB 下运行，崩溃时自动打印 bt full（用于定位前端 SIGSEGV）"
    fi
    # 运行前校验：优先使用 install_deps/ceres；若缺失则回退镜像系统 Ceres（按用户运行环境继续启动）。
    if [ -f "${WORKSPACE_DIR}/install/automap_pro/lib/libautomap_system_component.so" ] && command -v readelf >/dev/null 2>&1; then
        _need_ceres="$(readelf -d "${WORKSPACE_DIR}/install/automap_pro/lib/libautomap_system_component.so" 2>/dev/null | sed -n "s/.*Shared library: \[\(libceres\.so\.[0-9][0-9]*\)\].*/\1/p" | head -n1)"
        if [ -n "${_need_ceres}" ]; then
            _has_ceres=""
            [ -f "${WORKSPACE_DIR}/install_deps/ceres/lib/${_need_ceres}" ] && _has_ceres="${WORKSPACE_DIR}/install_deps/ceres/lib/${_need_ceres}"
            [ -z "${_has_ceres}" ] && [ -f "${WORKSPACE_DIR}/install_deps/ceres/lib64/${_need_ceres}" ] && _has_ceres="${WORKSPACE_DIR}/install_deps/ceres/lib64/${_need_ceres}"
            if [ -z "${_has_ceres}" ]; then
                print_warning "automap_system 依赖 ${_need_ceres}，但 install_deps/ceres 下不存在同名库；将回退使用镜像系统 Ceres。"
                print_warning "如后续运行出现找不到 ${_need_ceres}，再执行重编译：bash run_automap.sh --clean（不要 --run-only）。"
            else
                print_info "检测到本地 Ceres 库：${_has_ceres}"
            fi
        fi
    fi

    print_info "启动命令: ros2 launch ${LAUNCH_SPEC} ${LAUNCH_ARGS}"

    # 使用外部前端时必须有 fast_livo 已编译，否则 launch 会跳过该节点、后端无数据
    if [ "$USE_EXTERNAL_FRONTEND" = true ]; then
        if ! automap_fast_livo_installed; then
            print_error "未找到 fast_livo 安装（install/fast_livo 或 install_deps/fast_livo），无法启动前端节点，后端将一直等待数据。"
            echo ""
            echo "  若源码已在 automap_pro/src/modular/fast-livo2-humble 下，请执行完整编译（会编译 fast_livo 并安装）："
            echo "    bash run_automap.sh --offline --bag-file \"\$(pwd)/data/automap_input/M2DGR/street_03_ros2\" --config system_config_M2DGR.yaml --clean"
            echo "  编译时请查看 build.log 中 \"[INFO] fast_livo 源码检查\" 和 \"编译 fast_livo\" 是否出现；若无则容器内未找到该路径。"
            echo ""
            echo "  或使用内部前端：加参数 --no-external-frontend 再运行（若支持）。"
            echo "  亦支持 merged 布局: ${WORKSPACE_DIR}/install/share/fast_livo/package.xml"
            echo "  当前检查: install/、install/share/fast_livo/、install_deps/fast_livo/（及 find */share/fast_livo/package.xml）"
            exit 1
        fi
    fi

    # 配置 X11 转发
    xhost +local:docker &> /dev/null || true

    # 宿主机日志目录挂载到容器，运行日志写入 full.log（LOG_DIR 默认已为项目根/logs）
    mkdir -p "${LOG_DIR}"
    LOG_DIR_ABS="$(cd "${LOG_DIR}" && pwd)"
    LOG_MOUNT="-v ${LOG_DIR_ABS}:/root/run_logs:rw"
    # 容器内日志带时间戳（容器已挂载宿主机 /etc/localtime，时间与宿主机一致）
    CONTAINER_LAUNCH_CMD="ros2 launch ${LAUNCH_SPEC} ${CONTAINER_LAUNCH_ARGS} 2>&1 | while IFS= read -r line; do echo \"\$(date '+%Y-%m-%d %H:%M:%S') \$line\"; done | tee /root/run_logs/full.log"
    print_success "✓ 运行日志将保存到宿主机: ${LOG_DIR_ABS}/full.log"

    # 启动前检测并停止使用本镜像的正在运行容器，保证每次运行相互独立
    EXISTING=$(docker ps -q --filter "ancestor=${IMAGE_NAME}" 2>/dev/null || true)
    if [ -n "$EXISTING" ]; then
        print_info "检测到使用镜像 ${IMAGE_NAME} 的正在运行容器，正在停止..."
        echo "$EXISTING" | xargs -r docker stop
        print_success "✓ 已停止旧容器，本次将启动全新实例"
    fi

    # --gdb / --gdb-frontend：NGC Isaac 基础镜像可能未预装 gdb，传入标志由容器内尝试 apt 安装
    AUTOMAP_ENSURE_GDB=0
    if [ "$RUN_AUTOMAP_UNDER_GDB" = true ] || [ "$RUN_FAST_LIVO_UNDER_GDB" = true ]; then
        AUTOMAP_ENSURE_GDB=1
    fi

    print_info "启动容器..."
    print_info "按 Ctrl+C 停止系统"
    print_info "运行阶段：ros2 launch / bag 回放启动后，等待传感器或 bag 首帧时可能短暂无输出，请查看 ${LOG_DIR}/full.log"
    # 诊断提示：若日志出现 undefined symbol buildGlobalMap 或 Camera model not specified，见下方说明
    echo ""
    echo "[DIAG] 故障定位: grep -E 'LINK_4_PARAMS|FAST_LIVO_PARAMS|BAG|HBA|config_file param|undefined symbol|camera_loader|parameter_blackboard' 可精准查看参数与符号问题"
    echo "[DIAG] 若出现 undefined symbol buildGlobalMap → 请勿使用 --run-only，重新执行本脚本以触发 automap_pro 完整编译"
    echo "[DIAG] 若出现 Camera model not specified → 查看 [FAST_LIVO_PARAMS] parameter_blackboard.model 及 [fast_livo] [DIAG] parameter_blackboard.model"
    echo "[DIAG] 若出现 Exception on parsing info file / bad conversion → 宿主机与容器启动前均会尝试修复 metadata；未修复时请: chmod u+w <bag>/metadata.yaml && python3 scripts/fix_ros2_bag_metadata.py <bag目录>"
    echo "[DIAG] 若出现 libceres.so 找不到 → 确认 install_deps/ceres 下 lib 或 lib64 已安装，且运行脚本已为 LD_LIBRARY_PATH 注入（与 build_inside_container 行为一致）"
    echo "[DIAG] 若出现 HBA vector::_M_default_append 或 pose_size=0 → 查看 [HBA] [DATA] lio_pose_orig.size 与 [HBA] [FATAL]；确保 bag 回放且 fast_livo 先输出位姿"
    echo "[DIAG] 若无 GPS（轨迹 CSV 无 gps_x/gps_y/gps_z）：grep -E 'LivoBridge\\[GPS\\]|GPS_DIAG|TRAJ_LOG no GPS' 日志；M2DGR 使用 sensor.gps.topic=/ublox/fix，须与 bag 话题一致"
    echo ""

    # 运行容器：默认 --rm；仅在「将保存快照」时使用固定容器名（无 --rm），便于 docker commit + save
    AUTOMAP_SHOULD_SAVE_SNAPSHOT=false
    if [ "${AUTOMAP_SAVE_RUNTIME_SNAPSHOT:-1}" = "1" ]; then
        case "${AUTOMAP_SNAPSHOT_SAVE_MODE:-first}" in
            always)
                AUTOMAP_SHOULD_SAVE_SNAPSHOT=true
                ;;
            first|*)
                [ ! -f "${AUTOMAP_RUNTIME_SNAPSHOT_TAR}" ] && AUTOMAP_SHOULD_SAVE_SNAPSHOT=true
                ;;
        esac
    fi
    DOCKER_RUN_EXTRA=( )
    if [ "$AUTOMAP_SHOULD_SAVE_SNAPSHOT" = true ]; then
        docker rm -f "${AUTOMAP_RUNTIME_CONTAINER_NAME}" 2>/dev/null || true
        DOCKER_RUN_EXTRA=( --name "${AUTOMAP_RUNTIME_CONTAINER_NAME}" )
        if [ "${AUTOMAP_SNAPSHOT_SAVE_MODE:-first}" = "always" ]; then
            print_info "本次运行结束后将更新运行时快照至 ${AUTOMAP_RUNTIME_SNAPSHOT_TAR}（AUTOMAP_SNAPSHOT_SAVE_MODE=always）"
        else
            print_info "本次运行结束后将首次保存运行时快照至 ${AUTOMAP_RUNTIME_SNAPSHOT_TAR}（已有 tar 时默认跳过；AUTOMAP_SNAPSHOT_SAVE_MODE=always 可每次更新）"
        fi
    else
        DOCKER_RUN_EXTRA=( --rm )
    fi

    # 运行容器并启动建图系统（挂载 automap_pro 以便容器内能访问 config 等路径）
    # 若指定了 --log-dir，则 LOG_MOUNT 已包含 -v 宿主机目录:/root/run_logs，容器内 tee 写入 full.log 即保存到宿主机
    set +e
    # Some environments (CI / Cursor / non-interactive shells) don't provide a TTY.
    # `docker run -t` will fail with "the input device is not a TTY" and exit 1.
    DOCKER_TTY_ARGS=( )
    if [ -t 0 ] && [ -t 1 ]; then
        DOCKER_TTY_ARGS=( -it )
    else
        DOCKER_TTY_ARGS=( -i )
    fi
    docker run "${DOCKER_TTY_ARGS[@]}" "${DOCKER_RUN_EXTRA[@]}" \
        ${TIME_VOLUMES} \
        ${AUTOMAP_CACHE_MOUNT} \
        -v "${SCRIPT_DIR}/scripts:/root/scripts:ro" \
        --gpus all \
        --privileged \
        --net=host \
        --ipc=host \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e AUTOMAP_DOWNLOAD_CACHE=/root/automap_download_cache \
        -e PIP_CACHE_DIR=/root/automap_download_cache/pip \
        -e AUTOMAP_UBUNTU_MIRROR="${AUTOMAP_UBUNTU_MIRROR:-http://mirrors.aliyun.com/ubuntu}" \
        -e AUTOMAP_LIBTORCH_DOWNLOAD_BASE="${AUTOMAP_LIBTORCH_DOWNLOAD_BASE:-https://download.pytorch.org/libtorch}" \
        -e AUTOMAP_USE_OFFICIAL_LIBTORCH="${AUTOMAP_USE_OFFICIAL_LIBTORCH:-0}" \
        -e AUTOMAP_PYG_WHEEL_BASE="${AUTOMAP_PYG_WHEEL_BASE:-https://data.pyg.org/whl}" \
        -e AUTOMAP_USE_OFFICIAL_PYPI="${AUTOMAP_USE_OFFICIAL_PYPI:-0}" \
        -e AUTOMAP_PIP_INDEX="${AUTOMAP_PIP_INDEX:-}" \
        -e AUTOMAP_ONNXRUNTIME_GIT_URL="${AUTOMAP_ONNXRUNTIME_GIT_URL:-}" \
        -e LSK_PYTORCH_WHL_INDEX="${LSK_PYTORCH_WHL_INDEX:-}" \
        -e LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE="${LSK_PYTORCH_WHL_INDEX_DEFAULT_BASE:-}" \
        $( [ -n "${PIP_INDEX+x}" ] && [ -n "${PIP_INDEX}" ] && printf '%s\n' "-e PIP_INDEX=${PIP_INDEX}" ) \
        -e OMP_NUM_THREADS=1 \
        -e EIGEN_NUM_THREADS=1 \
        -e MKL_NUM_THREADS=1 \
        -e TBB_NUM_THREADS=1 \
        -e AUTOMAP_GTSAM_SERIAL=1 \
        -e AUTOMAP_ROS_DISTRO="${AUTOMAP_ROS_DISTRO}" \
        -e AUTOMAP_USE_RVIZ="${USE_RVIZ}" \
        -e AUTOMAP_SLOAM_ONNX_CUDA="${AUTOMAP_SLOAM_ONNX_CUDA:-1}" \
        -e AUTOMAP_ENSURE_GDB="${AUTOMAP_ENSURE_GDB}" \
        -e AUTOMAP_OFFLINE_BAG_PATH="${CONTAINER_OFFLINE_BAG_PATH}" \
        -e AUTOMAP_ONNXRUNTIME_LIB_DIR="${AUTOMAP_ONNXRUNTIME_LIB_DIR:-}" \
        -e AUTOMAP_LSK3DNET_PYTHON="${AUTOMAP_LSK3DNET_PYTHON:-}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
        -v "${PROJECT_DIR}:/root/automap_ws/src/automap_pro:ro" \
        -v "${DATA_DIR}:/data:rw" \
        ${LOG_MOUNT} \
        -w /root/automap_ws \
        "${IMAGE_NAME}" \
        /bin/bash -c "
            # ✅ 修复：限制线程数以避免 GTSAM TBB 并行导致 SIGSEGV / double free (borglab/gtsam#1189)
            export OMP_NUM_THREADS=1
            export EIGEN_NUM_THREADS=1
            export MKL_NUM_THREADS=1
            export TBB_NUM_THREADS=1
            export AUTOMAP_GTSAM_SERIAL=1

            if [ -f /root/scripts/automap_download_defaults.sh ]; then
              source /root/scripts/automap_download_defaults.sh
              automap_configure_apt_cache
            fi
            source /opt/ros/${AUTOMAP_ROS_DISTRO}/setup.bash
            # 与 build_inside_container 一致：运行期需 pcl_ros / cv_bridge 等共享库（仅缺任一关键包时 apt）
            if command -v apt-get >/dev/null 2>&1 && command -v dpkg >/dev/null 2>&1; then
              if ! dpkg -s \"ros-\${AUTOMAP_ROS_DISTRO}-cv-bridge\" >/dev/null 2>&1 \
                || ! dpkg -s \"ros-\${AUTOMAP_ROS_DISTRO}-pcl-ros\" >/dev/null 2>&1 \
                || ! dpkg -s libgeographiclib-dev >/dev/null 2>&1; then
                echo \"[INFO] 安装 ros-\${AUTOMAP_ROS_DISTRO}-* 与 libpcap-dev、libgeographiclib-dev …\" 1>&2
                DEBIAN_FRONTEND=noninteractive apt-get update -qq \
                  && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
                    \"ros-\${AUTOMAP_ROS_DISTRO}-pcl-ros\" \
                    \"ros-\${AUTOMAP_ROS_DISTRO}-pcl-conversions\" \
                    \"ros-\${AUTOMAP_ROS_DISTRO}-cv-bridge\" \
                    \"ros-\${AUTOMAP_ROS_DISTRO}-image-transport\" \
                    \"ros-\${AUTOMAP_ROS_DISTRO}-message-filters\" \
                    libpcap-dev \
                    libgeographiclib-dev \
                  || echo \"[WARN] apt 安装 ROS/系统依赖失败（若启动报缺库请检查网络）\" 1>&2
              fi
            fi
            # --gdb 需要 gdb；部分 NGC 镜像未预装
            if [ \"\${AUTOMAP_ENSURE_GDB}\" = \"1\" ]; then
              if ! command -v gdb >/dev/null 2>&1; then
                if command -v apt-get >/dev/null 2>&1; then
                  echo \"[INFO] AUTOMAP_ENSURE_GDB=1：正在安装 gdb...\" 1>&2
                  DEBIAN_FRONTEND=noninteractive apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq gdb \
                    || echo \"[WARN] gdb 安装失败，请手工在容器内 apt install gdb\" 1>&2
                else
                  echo \"[WARN] 未找到 apt-get，无法自动安装 gdb\" 1>&2
                fi
              fi
            fi
            # 启用 RViz 时运行期安装 ros-<distro>-rviz2（与手动执行 apt install ros-jazzy-rviz2 等价；精简镜像常缺该包）
            if [ \"\${AUTOMAP_USE_RVIZ}\" = \"true\" ]; then
              if command -v apt-get >/dev/null 2>&1 && command -v dpkg >/dev/null 2>&1; then
                if ! dpkg -s \"ros-\${AUTOMAP_ROS_DISTRO}-rviz2\" >/dev/null 2>&1; then
                  echo \"[INFO] AUTOMAP_USE_RVIZ=true：正在 apt install ros-\${AUTOMAP_ROS_DISTRO}-rviz2 …\" 1>&2
                  DEBIAN_FRONTEND=noninteractive apt-get update -qq \
                    && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \"ros-\${AUTOMAP_ROS_DISTRO}-rviz2\" \
                    || echo \"[WARN] ros-\${AUTOMAP_ROS_DISTRO}-rviz2 安装失败，可加 --no-rviz 或检查 apt 源/网络\" 1>&2
                fi
              fi
            fi
            # 优先使用 build_gtsam_no_tbb 编译的 GTSAM，避免与镜像/install_deps 中的 GTSAM 冲突（如 undefined symbol: NonlinearFactor::rekey）
            # 编译 GTSAM：在 automap_ws 下执行 scripts/build_gtsam_no_tbb.sh，或容器内 /root/automap_ws 下已有该目录
            if [ -d build_gtsam_no_tbb/gtsam ] && [ -f build_gtsam_no_tbb/gtsam/libgtsam.so ]; then
              export LD_LIBRARY_PATH=\"build_gtsam_no_tbb/gtsam:build_gtsam_no_tbb/gtsam_unstable:\$LD_LIBRARY_PATH\"
              echo \"[INFO] 使用 build_gtsam_no_tbb 的 GTSAM 库（覆盖镜像/install_deps）\" 1>&2
            fi
            # 先加载 install_deps（TEASER++/vikit 等），再加载主工作空间。
            # ⚠️ 重要：不要默认强行使用 install_deps/gtsam/lib。
            #     该目录下的预编译 GTSAM 可能与系统 Eigen/PCL flags 不一致，导致启动时在 lago.cpp 静态初始化触发
            #     double free or corruption (out)（见 docs/FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md）。
            #
            # 如确实需要使用 install_deps 里的 GTSAM，请显式设置：
            #   export AUTOMAP_USE_INSTALL_DEPS_GTSAM=1
            if [ -f install_deps/setup.bash ]; then source install_deps/setup.bash; fi
            if [ -d install_deps/teaserpp/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/teaserpp/lib:\$LD_LIBRARY_PATH\"
            fi
            # Ceres Solver：与 scripts/build_inside_container.sh 一致，须同时覆盖 lib 与 lib64（仅一侧存在时以往会漏设 LD_LIBRARY_PATH → libceres.so 找不到）
            if [ -d install_deps/ceres ]; then
              export CMAKE_PREFIX_PATH=\"install_deps/ceres:\$CMAKE_PREFIX_PATH\"
              [ -d install_deps/ceres/lib ] && export LD_LIBRARY_PATH=\"install_deps/ceres/lib:\$LD_LIBRARY_PATH\"
              [ -d install_deps/ceres/lib64 ] && export LD_LIBRARY_PATH=\"install_deps/ceres/lib64:\$LD_LIBRARY_PATH\"
            fi
            # 离线 bag：容器内再修复 metadata（宿主机未写回或权限失败时，/data 常为 rw）
            if [ -n \"\${AUTOMAP_OFFLINE_BAG_PATH:-}\" ]; then
              _bagp=\"\${AUTOMAP_OFFLINE_BAG_PATH}\"
              [ -f \"\${_bagp}\" ] && _bagp=\"\$(dirname \"\${_bagp}\")\"
              if [ -d \"\${_bagp}\" ] && [ -f \"\${_bagp}/metadata.yaml\" ] && [ -f /root/automap_ws/src/automap_pro/scripts/fix_ros2_bag_metadata.py ]; then
                ROS_DISTRO=\"${AUTOMAP_ROS_DISTRO}\" python3 /root/automap_ws/src/automap_pro/scripts/fix_ros2_bag_metadata.py \"\${_bagp}\" --qos-style auto --ros-distro \"${AUTOMAP_ROS_DISTRO}\" --verify-with-ros2-info 1>&2 || true
              fi
            fi
            # hba（全局优化节点）：与 GTSAM 一致，安装到 install_deps/hba
            if [ -f install_deps/hba/setup.bash ]; then
              source install_deps/hba/setup.bash
            elif [ -d install_deps/hba/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/hba/lib:\$LD_LIBRARY_PATH\"
              export CMAKE_PREFIX_PATH=\"install_deps/hba:\$CMAKE_PREFIX_PATH\"
            fi
            # vikit（fast_livo 依赖）：与 GTSAM 一致，安装到 install_deps/vikit
            if [ -f install_deps/vikit/setup.bash ]; then
              source install_deps/vikit/setup.bash
            elif [ -d install_deps/vikit/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/vikit/lib:\$LD_LIBRARY_PATH\"
              export CMAKE_PREFIX_PATH=\"install_deps/vikit:\$CMAKE_PREFIX_PATH\"
            elif [ -d install_deps/lib ] && [ -f install_deps/lib/libvikit_common.so ]; then
              export LD_LIBRARY_PATH=\"install_deps/lib:\$LD_LIBRARY_PATH\"
            fi
            # fast_livo（前端节点）：与 GTSAM 一致，安装到 install_deps/fast_livo
            if [ -f install_deps/fast_livo/setup.bash ]; then
              source install_deps/fast_livo/setup.bash
            elif [ -d install_deps/fast_livo/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/fast_livo/lib:\$LD_LIBRARY_PATH\"
              export CMAKE_PREFIX_PATH=\"install_deps/fast_livo:\$CMAKE_PREFIX_PATH\"
            fi
            # LibTorch（OverlapTransformer）：与 GTSAM 一致，使用 install_deps 下预装库，避免每次编译
            if [ -d install_deps/libtorch/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/libtorch/lib:\$LD_LIBRARY_PATH\"
            fi
            # ONNX Runtime（SLOAM 语义分割）：与 GTSAM 一致，install_deps 安装后直接使用
            if [ -d install_deps/onnxruntime/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/onnxruntime/lib:\$LD_LIBRARY_PATH\"
              export ONNXRUNTIME_HOME=\"\${ONNXRUNTIME_HOME:-\$PWD/install_deps/onnxruntime}\"
            fi
            if [ -x install_deps/lsk3dnet_venv/bin/python3 ]; then
              export AUTOMAP_LSK3DNET_PYTHON=\"\${AUTOMAP_LSK3DNET_PYTHON:-\$PWD/install_deps/lsk3dnet_venv/bin/python3}\"
            fi
            # LSK3DNet hybrid: 若配置为 lsk3dnet_hybrid 且分类头 TorchScript 缺失，则自动导出（仅首次）。
            # 这样可以确保“必须使用 hybrid”不依赖手动预处理。
            if [ -f \"${CONTAINER_CONFIG_PATH}\" ]; then
              _lsk_mt=\"\$(python3 - <<'PY' 2>/dev/null || true
import yaml
cfg = yaml.safe_load(open(\"${CONTAINER_CONFIG_PATH}\",\"r\",encoding=\"utf-8\")) or {}
sem = cfg.get(\"semantic\") or {}
print((sem.get(\"model_type\") or \"\").strip())
PY
              )\"
              if [ \"\${_lsk_mt}\" = \"lsk3dnet_hybrid\" ]; then
                # 计算准确性硬约束：hybrid_normal_mode=range 时必须能 import c_gen_normal_map。
                _lsk_nm=\"\$(python3 - <<'PY' 2>/dev/null || true
import yaml
cfg = yaml.safe_load(open(\"${CONTAINER_CONFIG_PATH}\",\"r\",encoding=\"utf-8\")) or {}
sem = cfg.get(\"semantic\") or {}
lsk = sem.get(\"lsk3dnet\") or {}
print((lsk.get(\"hybrid_normal_mode\") or \"range\").strip().lower())
PY
                )\"
                if [ \"\${_lsk_nm}\" = \"range\" ]; then
                  _py=\"\${AUTOMAP_LSK3DNET_PYTHON:-python3}\"
                  if ! \"\${_py}\" -c \"import c_gen_normal_map\" >/dev/null 2>&1; then
                    echo \"[FATAL] [LSK3DNET] hybrid_normal_mode=range requires python module c_gen_normal_map, but import failed.\" 1>&2
                    echo \"[FATAL] [LSK3DNET] Fix: re-run with build steps enabled so install_deps/lsk3dnet_venv compiles c_utils, e.g.:\" 1>&2
                    echo \"        AUTOMAP_SETUP_LSK3DNET_VENV=1 bash run_automap.sh --offline --bag-file ... --config ${CONFIG_BASENAME:-system_config_M2DGR.yaml} --clean\" 1>&2
                    exit 1
                  fi
                fi

                _lsk_repo=\"\$(python3 - <<'PY' 2>/dev/null || true
import os,yaml
cfg = yaml.safe_load(open(\"${CONTAINER_CONFIG_PATH}\",\"r\",encoding=\"utf-8\")) or {}
sem = cfg.get(\"semantic\") or {}
lsk = sem.get(\"lsk3dnet\") or {}
root = \"/root/automap_ws/src/automap_pro\"
def r(p):
  if not p: return \"\"
  p = p.replace(\"${CMAKE_CURRENT_SOURCE_DIR}\", root)
  return os.path.normpath(p)
print(r((lsk.get(\"repo_root\") or \"\").strip()))
PY
                )\"
                _lsk_cfg=\"\$(python3 - <<'PY' 2>/dev/null || true
import os,yaml
cfg = yaml.safe_load(open(\"${CONTAINER_CONFIG_PATH}\",\"r\",encoding=\"utf-8\")) or {}
sem = cfg.get(\"semantic\") or {}
lsk = sem.get(\"lsk3dnet\") or {}
print((lsk.get(\"config_yaml\") or \"\").strip())
PY
                )\"
                _lsk_ckpt=\"\$(python3 - <<'PY' 2>/dev/null || true
import os,yaml
cfg = yaml.safe_load(open(\"${CONTAINER_CONFIG_PATH}\",\"r\",encoding=\"utf-8\")) or {}
sem = cfg.get(\"semantic\") or {}
lsk = sem.get(\"lsk3dnet\") or {}
root = \"/root/automap_ws/src/automap_pro\"
p = (lsk.get(\"checkpoint\") or \"\").strip().replace(\"${CMAKE_CURRENT_SOURCE_DIR}\", root)
print(os.path.normpath(p))
PY
                )\"
                _lsk_cls=\"\$(python3 - <<'PY' 2>/dev/null || true
import os,yaml
cfg = yaml.safe_load(open(\"${CONTAINER_CONFIG_PATH}\",\"r\",encoding=\"utf-8\")) or {}
sem = cfg.get(\"semantic\") or {}
lsk = sem.get(\"lsk3dnet\") or {}
root = \"/root/automap_ws/src/automap_pro\"
p = (lsk.get(\"classifier_torchscript\") or \"\").strip().replace(\"${CMAKE_CURRENT_SOURCE_DIR}\", root)
print(os.path.normpath(p))
PY
                )\"
                if [ -n \"\${_lsk_cls}\" ] && [ ! -f \"\${_lsk_cls}\" ]; then
                  _exp_py=\"\${AUTOMAP_LSK3DNET_PYTHON:-python3}\"
                  _exp_script=\"\${_lsk_repo}/scripts/export_lsk3dnet_classifier_torchscript.py\"
                  if [ -f \"\${_exp_script}\" ] && [ -n \"\${_lsk_cfg}\" ] && [ -f \"\${_lsk_ckpt}\" ]; then
                    echo \"[INFO] [LSK3DNET] classifier_torchscript missing, exporting -> \${_lsk_cls}\" 1>&2
                    mkdir -p \"\$(dirname \"\${_lsk_cls}\")\" 2>/dev/null || true
                    ( cd \"\${_lsk_repo}\" && \"\${_exp_py}\" \"scripts/export_lsk3dnet_classifier_torchscript.py\" \\
                        --config \"\${_lsk_cfg}\" --checkpoint \"\${_lsk_ckpt}\" --output \"\${_lsk_cls}\" --device cpu ) 1>&2
                    if [ -f \"/root/automap_ws/src/automap_pro/scripts/verify_lsk3dnet_hybrid_classifier.py\" ]; then
                      \"\${_exp_py}\" /root/automap_ws/src/automap_pro/scripts/verify_lsk3dnet_hybrid_classifier.py --classifier \"\${_lsk_cls}\" 1>&2 || true
                    fi
                  else
                    echo \"[WARN] [LSK3DNET] cannot export classifier: script/config/ckpt missing\" 1>&2
                  fi
                fi
              fi
            fi
            if [ \"\${AUTOMAP_USE_INSTALL_DEPS_GTSAM:-0}\" = \"1\" ] && [ -d install_deps/gtsam/lib ]; then
              export LD_LIBRARY_PATH=\"install_deps/gtsam/lib:\$LD_LIBRARY_PATH\"
              echo \"[WARN] AUTOMAP_USE_INSTALL_DEPS_GTSAM=1: forcing LD_LIBRARY_PATH prepend install_deps/gtsam/lib\" 1>&2
            fi

            # 启动前 NVRTC 架构硬校验（Blackwell 重点）：
            # 若 libnvrtc 不支持目标架构（如 compute_120），优先自动重装/重建 GPU 栈；禁止静默切 CPU OT。
            if [ \"\${AUTOMAP_STRICT_OT_NVRTC_PRECHECK:-1}\" = \"1\" ]; then
              _gpu_name=\"\"
              if command -v nvidia-smi >/dev/null 2>&1; then
                _gpu_name=\"\$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1 | tr -d '\r' || true)\"
              fi
              _need_check=0
              _target_arch=\"\"
              if echo \"\${_gpu_name}\" | grep -qE 'RTX[[:space:]]*50[0-9]{2,4}'; then
                _need_check=1
                _target_arch=\"compute_120\"
              fi
              if [ \"\${_need_check}\" = \"1\" ] && command -v python3 >/dev/null 2>&1; then
                _nvrtc_err=\"\"
                _nvrtc_err=\"\$(OT_NVRTC_TARGET_ARCH=\"\${_target_arch}\" python3 - <<'PY' 2>&1
import ctypes
import os
import sys

target_arch = (os.environ.get(\"OT_NVRTC_TARGET_ARCH\") or \"\").strip()
if not target_arch:
    sys.exit(0)

src = b'extern \"C\" __global__ void k() {}'
prog = ctypes.c_void_p()

def _load_nvrtc():
    for name in (\"libnvrtc.so\", \"libnvrtc.so.13\", \"libnvrtc.so.12\", \"libnvrtc.so.11\"):
        try:
            return ctypes.CDLL(name), name
        except Exception:
            continue
    return None, None

nvrtc, libname = _load_nvrtc()
if nvrtc is None:
    print(\"libnvrtc not found\", end=\"\")
    sys.exit(2)

nvrtc.nvrtcCreateProgram.argtypes = [
    ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_char_p,
    ctypes.c_int, ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.c_char_p)
]
nvrtc.nvrtcCreateProgram.restype = ctypes.c_int
nvrtc.nvrtcCompileProgram.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
nvrtc.nvrtcCompileProgram.restype = ctypes.c_int
nvrtc.nvrtcGetProgramLogSize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t)]
nvrtc.nvrtcGetProgramLogSize.restype = ctypes.c_int
nvrtc.nvrtcGetProgramLog.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
nvrtc.nvrtcGetProgramLog.restype = ctypes.c_int
nvrtc.nvrtcDestroyProgram.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
nvrtc.nvrtcDestroyProgram.restype = ctypes.c_int

rc = nvrtc.nvrtcCreateProgram(ctypes.byref(prog), src, b\"ot_nvrtc_check.cu\", 0, None, None)
if rc != 0:
    print(f\"nvrtcCreateProgram failed rc={rc}\", end=\"\")
    sys.exit(3)

opt = f\"--gpu-architecture={target_arch}\".encode(\"utf-8\")
opts = (ctypes.c_char_p * 1)(opt)
rc = nvrtc.nvrtcCompileProgram(prog, 1, opts)
if rc != 0:
    size = ctypes.c_size_t(0)
    nvrtc.nvrtcGetProgramLogSize(prog, ctypes.byref(size))
    log = b\"\"
    if size.value > 0:
        buf = ctypes.create_string_buffer(size.value)
        nvrtc.nvrtcGetProgramLog(prog, buf)
        log = buf.value
    nvrtc.nvrtcDestroyProgram(ctypes.byref(prog))
    msg = log.decode(\"utf-8\", errors=\"ignore\").strip() if log else f\"compile rc={rc}\"
    print(f\"{libname} does not support {target_arch}: {msg}\", end=\"\")
    sys.exit(4)

nvrtc.nvrtcDestroyProgram(ctypes.byref(prog))
sys.exit(0)
PY
)\"
                _nvrtc_rc=$?
                if [ \"\${_nvrtc_rc}\" -ne 0 ]; then
                  echo -e \"\\033[1;31m[CUDA_BLOCK][OT] NVRTC arch precheck failed on GPU '\${_gpu_name}' (target=\${_target_arch})\\033[0m\" 1>&2
                  echo -e \"\\033[1;31m[CUDA_BLOCK][OT] reason=\${_nvrtc_err:-nvrtc precheck failed}; start auto-reinstall GPU stack (CUDA/LibTorch)...\\033[0m\" 1>&2

                  # 强制重装/重建依赖栈（不接受 CPU OT 降级）
                  export AUTOMAP_PREBUILT_INSTALL_DEPS=0
                  export LIBTORCH_SKIP_DOWNLOAD=0
                  export AUTOMAP_SKIP_CUDA_TOOLKIT_APT=0
                  export AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED=1
                  export AUTOMAP_STRICT_BLACKWELL_STACK=1

                  # Blackwell：已存在旧版 LibTorch 时必须重下 cu128，避免继续命中 NVRTC 架构不支持
                  if [ -d \"install_deps/libtorch\" ]; then
                    rm -rf install_deps/libtorch
                  fi

                  if [ -x /root/scripts/build_inside_container.sh ]; then
                    echo \"[CUDA_BLOCK][OT] Try fast repair first: CUDA/LibTorch only (no full compile).\" 1>&2
                    AUTOMAP_REPAIR_CUDA_STACK_ONLY=1 /bin/bash /root/scripts/build_inside_container.sh 1>&2 || {
                      echo \"[CUDA_BLOCK][OT] Fast repair failed; fallback to full rebuild.\" 1>&2
                      AUTOMAP_REPAIR_CUDA_STACK_ONLY=0 /bin/bash /root/scripts/build_inside_container.sh 1>&2
                    }
                  else
                    echo -e \"\\033[1;31m[CUDA_BLOCK][OT][FATAL] /root/scripts/build_inside_container.sh not found; cannot auto-reinstall GPU stack\\033[0m\" 1>&2
                    exit 1
                  fi

                  # 重装后再次探测；仍失败则直接阻断启动（不切 CPU）
                  _nvrtc_err2=\"\"
                  _nvrtc_err2=\"\$(OT_NVRTC_TARGET_ARCH=\"\${_target_arch}\" python3 - <<'PY' 2>&1
import ctypes
import os
import sys

target_arch = (os.environ.get('OT_NVRTC_TARGET_ARCH') or '').strip()
if not target_arch:
    sys.exit(0)
src = b'extern \"C\" __global__ void k() {}'
prog = ctypes.c_void_p()
for name in ('libnvrtc.so', 'libnvrtc.so.13', 'libnvrtc.so.12', 'libnvrtc.so.11'):
    try:
        nvrtc = ctypes.CDLL(name)
        break
    except Exception:
        nvrtc = None
if nvrtc is None:
    print('libnvrtc not found', end='')
    sys.exit(2)
nvrtc.nvrtcCreateProgram.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p), ctypes.POINTER(ctypes.c_char_p)]
nvrtc.nvrtcCreateProgram.restype = ctypes.c_int
nvrtc.nvrtcCompileProgram.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
nvrtc.nvrtcCompileProgram.restype = ctypes.c_int
nvrtc.nvrtcGetProgramLogSize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t)]
nvrtc.nvrtcGetProgramLogSize.restype = ctypes.c_int
nvrtc.nvrtcGetProgramLog.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
nvrtc.nvrtcGetProgramLog.restype = ctypes.c_int
nvrtc.nvrtcDestroyProgram.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
nvrtc.nvrtcDestroyProgram.restype = ctypes.c_int
rc = nvrtc.nvrtcCreateProgram(ctypes.byref(prog), src, b'ot_nvrtc_check_retry.cu', 0, None, None)
if rc != 0:
    print(f'nvrtcCreateProgram failed rc={rc}', end='')
    sys.exit(3)
opt = f'--gpu-architecture={target_arch}'.encode('utf-8')
opts = (ctypes.c_char_p * 1)(opt)
rc = nvrtc.nvrtcCompileProgram(prog, 1, opts)
if rc != 0:
    size = ctypes.c_size_t(0)
    nvrtc.nvrtcGetProgramLogSize(prog, ctypes.byref(size))
    log = b''
    if size.value > 0:
        buf = ctypes.create_string_buffer(size.value)
        nvrtc.nvrtcGetProgramLog(prog, buf)
        log = buf.value
    nvrtc.nvrtcDestroyProgram(ctypes.byref(prog))
    msg = log.decode('utf-8', errors='ignore').strip() if log else f'compile rc={rc}'
    print(msg, end='')
    sys.exit(4)
nvrtc.nvrtcDestroyProgram(ctypes.byref(prog))
sys.exit(0)
PY
)\"
                  _nvrtc_rc2=$?
                  if [ \"\${_nvrtc_rc2}\" -ne 0 ]; then
                    echo -e \"\\033[1;31m[CUDA_BLOCK][OT][FATAL] NVRTC precheck still fails after auto-reinstall. reason=\${_nvrtc_err2:-unknown}\\033[0m\" 1>&2
                    echo -e \"\\033[1;31m[CUDA_BLOCK][OT][FATAL] GPU realtime inference is required; startup aborted (no CPU fallback).\\033[0m\" 1>&2
                    exit 1
                  fi
                  echo \"[CUDA_BLOCK][OT] Auto-reinstall succeeded; NVRTC arch precheck passed, keep GPU OT enabled.\" 1>&2
                fi
              fi
            fi
            source install/setup.bash
            ${CONTAINER_LAUNCH_CMD}
        "
    DOCKER_EXIT=$?
    set -e
    if [ "$DOCKER_EXIT" -ne 0 ]; then
        if [ "$AUTOMAP_SHOULD_SAVE_SNAPSHOT" = true ]; then
            docker rm -f "${AUTOMAP_RUNTIME_CONTAINER_NAME}" 2>/dev/null || true
        fi
        echo ""
        print_error "容器退出码: ${DOCKER_EXIT}（非 0 表示异常退出）"
        echo "[ERROR] 常见原因与排查:"
        echo "  - 退出码 1:    launch/节点启动失败 → 查看上方 [automap_offline][EXCEPTION] 或 [params_from_system_config][EXCEPTION]"
        echo "  - 退出码 127:  undefined symbol → 请勿用 --run-only，重新执行以触发 automap_pro 编译"
        echo "  - 退出码 245:  fast_livo segfault → 检查 [FAST_LIVO_PARAMS] parameter_blackboard.model 及 config 中 fast_livo 节"
        echo "  - 退出码 -6:    SIGABRT(如 HBA vector::_M_default_append) → 查看 [HBA] [FATAL]/[HBA][layer] 或 [camera_loader]；pose_size=0 时需 bag 回放且 fast_livo 先出位姿"
        echo "  - yaml-cpp bad conversion: 脚本已在宿主机/容器内尝试修复 metadata；未生效时请: chmod u+w <bag>/metadata.yaml && python3 scripts/fix_ros2_bag_metadata.py <bag目录>"
        echo "  - libceres.so.* not found: 优先检查镜像系统是否已安装对应 SONAME；若需本地覆盖再确认 install_deps/ceres/lib 与 lib64 已注入 LD_LIBRARY_PATH"
        echo "[ERROR] 精准定位: grep -E 'EXCEPTION|ERROR|FATAL|undefined symbol|what\\(\\)|\\[HBA\\]|\\[camera_loader\\]|\\[BAG\\]' 上述输出"
        exit "$DOCKER_EXIT"
    fi

    # 正常运行结束：提交容器层并写入本地 tar，供下次 ensure_image 优先加载（workspace 仍挂载，不在镜像层内）
    if [ "$AUTOMAP_SHOULD_SAVE_SNAPSHOT" = true ] && docker container inspect "${AUTOMAP_RUNTIME_CONTAINER_NAME}" &>/dev/null; then
        automap_commit_and_save_runtime_snapshot "${AUTOMAP_RUNTIME_CONTAINER_NAME}" "runtime" || true
    fi
}

# ==================== 主函数 ====================
main() {
    print_header "AutoMap-Pro 一键编译和运行脚本"

    # 解析参数
    parse_args "$@"
    apply_gdb_image_policy

    # 若配置要求 LSK3DNet hybrid，则自动启用构建阶段创建 lsk3dnet_venv（torch+spconv+torch-scatter）。
    # 规则：仅在用户未显式设置 AUTOMAP_SETUP_LSK3DNET_VENV 时自动开启，避免覆盖用户意图。
    if [ -z "${AUTOMAP_SETUP_LSK3DNET_VENV+x}" ]; then
        _CFG_PATH_FOR_LSK=""
        if [ -n "${CONFIG_FILE:-}" ]; then
            if [ -z "${CONFIG_FILE##*/*}" ]; then
                _CFG_PATH_FOR_LSK="${CONFIG_FILE}"
            else
                _CFG_PATH_FOR_LSK="${PROJECT_DIR}/config/${CONFIG_FILE}"
            fi
        else
            _CFG_PATH_FOR_LSK="${PROJECT_DIR}/config/system_config.yaml"
        fi

        if [ -f "${_CFG_PATH_FOR_LSK}" ] && AUTOMAP_CFG_PATH="${_CFG_PATH_FOR_LSK}" python3 - <<'PY' 2>/dev/null
import os, sys, yaml
p = os.environ.get("AUTOMAP_CFG_PATH","")
if not p or not os.path.isfile(p):
    raise SystemExit(1)
cfg = yaml.safe_load(open(p, "r", encoding="utf-8")) or {}
sem = cfg.get("semantic") or {}
mt = (sem.get("model_type") or "").strip()
raise SystemExit(0 if mt == "lsk3dnet_hybrid" else 1)
PY
        then
            AUTOMAP_SETUP_LSK3DNET_VENV=1
            print_info "检测到 semantic.model_type=lsk3dnet_hybrid，自动启用 AUTOMAP_SETUP_LSK3DNET_VENV=1（创建 install_deps/lsk3dnet_venv）"
        fi
    fi

    # 未指定 --log-dir 时使用带时间戳子目录，便于区分每次运行
    if [ "${LOG_DIR_USER_SET}" != "true" ] && [ "${LOG_USE_TIMESTAMP_SUBDIR}" = "true" ]; then
        LOG_DIR="${SCRIPT_DIR}/logs/run_$(date +%Y%m%d_%H%M%S)"
    fi

    # 确保日志目录存在，并将本次运行的全部输出带时间戳写入 logs/automap.log，便于精准分析
    mkdir -p "${LOG_DIR}"
    exec > >(add_timestamp | tee "${LOG_DIR}/automap.log") 2>&1

    print_info "本次运行全部日志同时写入: ${LOG_DIR}/automap.log"

    # 关键开关与环境变量（最终生效值），便于排查「为何执行/跳过了某阶段」
    print_header "关键开关（最终生效值）"
    print_info "[DIAG] AUTOMAP_ENABLE_BUILD_STEPS=${AUTOMAP_ENABLE_BUILD_STEPS:-<unset>}"
    print_info "[DIAG] AUTOMAP_PREBUILT_INSTALL_DEPS=${AUTOMAP_PREBUILT_INSTALL_DEPS:-<unset>}"
    print_info "[DIAG] AUTOMAP_ALWAYS_BUILD=${AUTOMAP_ALWAYS_BUILD:-0} CLEAN_BUILD=${CLEAN_BUILD} BUILD_ONLY=${BUILD_ONLY} RUN_ONLY=${RUN_ONLY}"
    print_info "[DIAG] MODE=${MODE} BAG_FILE=${BAG_FILE:-<unset>} CONFIG_FILE=${CONFIG_FILE:-<unset>} BAG_RATE=${BAG_RATE}"
    print_info "[DIAG] USE_RVIZ=${USE_RVIZ} USE_EXTERNAL_FRONTEND=${USE_EXTERNAL_FRONTEND} USE_EXTERNAL_OVERLAP=${USE_EXTERNAL_OVERLAP}"
    print_info "[DIAG] AUTOMAP_ROS_DISTRO=${AUTOMAP_ROS_DISTRO:-<unset>} IMAGE_NAME=${IMAGE_NAME:-<unset>}"
    print_info "[DIAG] ONNXRUNTIME_USE_CUDA=${ONNXRUNTIME_USE_CUDA:-<unset>} AUTOMAP_SLOAM_ONNX_CUDA=${AUTOMAP_SLOAM_ONNX_CUDA:-<unset>}"
    print_info "[DIAG] LIBTORCH_SKIP_DOWNLOAD=${LIBTORCH_SKIP_DOWNLOAD:-<unset>} AUTOMAP_SKIP_CUDA_TOOLKIT_APT=${AUTOMAP_SKIP_CUDA_TOOLKIT_APT:-<unset>}"
    print_info "[DIAG] AUTOMAP_STRICT_BLACKWELL_STACK=${AUTOMAP_STRICT_BLACKWELL_STACK:-1} AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED=${AUTOMAP_ENFORCE_CUDA_STACK_UNIFIED:-1}"

    # --build-only 且未显式设置时，默认创建 LSK3DNet venv（与此前需写 AUTOMAP_SETUP_LSK3DNET_VENV=1 等价）
    if [ "$BUILD_ONLY" = true ] && [ -z "${AUTOMAP_SETUP_LSK3DNET_VENV+x}" ]; then
        AUTOMAP_SETUP_LSK3DNET_VENV=1
        print_info "已默认启用 AUTOMAP_SETUP_LSK3DNET_VENV=1（仅 --build-only）；跳过请: AUTOMAP_SETUP_LSK3DNET_VENV=0 bash run_automap.sh --build-only"
    fi

    # 执行流程（先确保镜像已 pull/load，再用同一镜像做 nvidia-smi 检测）
    ensure_image
    check_dependencies
    prepare_workspace

    # 换新 Docker 基础镜像但保留 automap_ws 时：避免重复下载 LibTorch、apt 安装 nvidia-cuda-toolkit（显式设 AUTOMAP_PREBUILT_INSTALL_DEPS=0 可强制走全量依赖流程）
    if [ -z "${AUTOMAP_PREBUILT_INSTALL_DEPS+x}" ]; then
        if [ -f "${WORKSPACE_DIR}/install_deps/libtorch/share/cmake/Torch/TorchConfig.cmake" ] || [ -f "${WORKSPACE_DIR}/install_deps/libtorch/lib/libtorch.so" ]; then
            AUTOMAP_PREBUILT_INSTALL_DEPS=1
            print_info "宿主机已有 install_deps/libtorch，已设置 AUTOMAP_PREBUILT_INSTALL_DEPS=1（跳过 LibTorch 下载与 nvidia-cuda-toolkit apt；需要重装依赖时请 export AUTOMAP_PREBUILT_INSTALL_DEPS=0）"
        fi
    fi

    if [ "${AUTOMAP_ENABLE_BUILD_STEPS}" = "1" ]; then
        build_project
    else
        if [ "$BUILD_ONLY" = true ]; then
            print_warning "已关闭构建阶段（AUTOMAP_ENABLE_BUILD_STEPS=0）；--build-only 将不执行任何 build.log 对应操作。"
            print_info "如需恢复执行，请显式设置: AUTOMAP_ENABLE_BUILD_STEPS=1"
            exit 0
        fi
        print_info "已关闭 build_inside_container 相关操作（AUTOMAP_ENABLE_BUILD_STEPS=0），跳过编译/依赖安装阶段。"
    fi
    run_system

    print_success "✓ 所有操作完成"
}

# ==================== 执行 ====================
main "$@"
