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
#   bash run_automap.sh --build-only      # 仅编译
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
IMAGE_NAME="automap-env:humble"
IMAGE_ARCHIVE="${SCRIPT_DIR}/docker/automap-env_humble.tar"
WORKSPACE_DIR="${SCRIPT_DIR}/automap_ws"
PROJECT_DIR="${SCRIPT_DIR}/automap_pro"
DATA_DIR="${SCRIPT_DIR}/data"
OUTPUT_DIR="${DATA_DIR}/automap_output"

# 运行模式
MODE="online"
BAG_FILE=""
CONFIG_FILE=""
BAG_RATE="0.5"  # 默认 0.5 倍速回放，缓解 LIO 处理压力；可改为 1.0 实时
BUILD_ONLY=false
RUN_ONLY=false
USE_RVIZ=true
CLEAN_BUILD=false
USE_EXTERNAL_FRONTEND=true
USE_EXTERNAL_OVERLAP=false
RUN_AUTOMAP_UNDER_GDB=false
RUN_FAST_LIVO_UNDER_GDB=false
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
    bash run_automap.sh --run-only --no-rviz                   # 运行但不启动 RViz
    bash run_automap.sh --offline --bag-file /data/xxx --log-dir \$(pwd)/logs   # 日志保存到宿主机 ./logs/full.log

目录说明:
    - 工作空间: ${WORKSPACE_DIR}
    - 数据目录: ${DATA_DIR}
    - 输出目录: ${OUTPUT_DIR}（离线模式默认启动前清空，保证每次重新建图；可用 --no-clean-output 保留上次结果）
    - 日志目录: 默认 ${SCRIPT_DIR}/logs/run_YYYYMMDD_HHMMSS/（宿主机，每次运行独立目录）；--log-dir 可指定固定目录
        automap.log  全程总日志  build.log  编译  full.log  运行  clean.log  清理  image.log  镜像加载/构建

故障排查:
    - 第三方库（GTSAM/TEASER++/vikit）安装在 automap_ws/install_deps，--clean 不会删除；需强制重编第三方时请手动: rm -rf automap_ws/install_deps
    - 若编译报「CMakeCache.txt directory ... is different than ...」：脚本已自动检测并清理含 /workspace/ 的缓存；仍失败时请加 --clean 后重跑
    - 若 ros2 bag 报「yaml-cpp: bad conversion」：检查 bag 元数据或换 --storage-preset mcap 重录
    - 若 HBA 报 vector::_M_default_append：多为尚无 LIO 位姿，请确保 --config 与 bag 话题一致且 fast_livo 已正常输出
    - 若 automap_system 报 undefined symbol keyframeCount：需重新编译 automap_pro（已补全 SubMapManager 实现）

EOF
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

    # 检查 NVIDIA Docker 支持
    if ! docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
        print_warning "⚠ NVIDIA Docker 支持检查失败，可能影响 GPU 功能"
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

    # 检查镜像是否已存在
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

    # 构建镜像
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
            /bin/bash -c "rm -rf /root/automap_ws/build /root/automap_ws/build_teaserpp /root/automap_ws/build_sophus /root/automap_ws/build_ceres /root/automap_ws/install /root/automap_ws/log && echo 'cleaned (install_deps 保留，第三方库不重编)'" 2>&1 | add_timestamp | tee "${LOG_DIR}/clean.log"
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
        if [ ! -f "${WORKSPACE_DIR}/install/fast_livo/share/fast_livo/package.xml" ]; then
            NEED_BUILD=true
            print_info "检测到 fast-livo2-humble 源码，但未编译，将执行完整编译"
        fi
    fi
    if [ "$NEED_BUILD" = false ] && [ "$CLEAN_BUILD" = false ]; then
        print_info "项目已编译，跳过编译步骤"
        print_info "如需重新编译，请使用 --clean 参数"
        return 0
    fi

    print_info "开始编译项目..."
    print_info "使用 $(nproc) 个线程并行编译"
    print_info "预计编译时间: 5-10 分钟"

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
    set +e
    docker run --rm \
        ${TIME_VOLUMES} \
        --gpus all \
        --net=host \
        -e DISPLAY=$DISPLAY \
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
    BUILD_EXIT=${PIPESTATUS[0]}
    set -e

    if [ "$BUILD_EXIT" -eq 0 ]; then
        print_success "✓ 项目编译成功"
    else
        print_error "✗ 项目编译失败（详见 ${LOG_DIR}/build.log）"
        exit 1
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
        # 使用「包名 + 文件名」从 install 加载，避免 PythonExpression/IfCondition 在 Humble 报错；请先执行一次 build 以安装修复后的 launch
        LAUNCH_SPEC="automap_pro automap_offline.launch.py"
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
    if [ "$MODE" = "offline" ] && [ -n "$BAG_FILE" ]; then
        if [[ "$BAG_FILE" == "${DATA_DIR}"* ]]; then
            CONTAINER_BAG_FILE="/data/${BAG_FILE#${DATA_DIR}/}"
        else
            CONTAINER_BAG_FILE="$BAG_FILE"
        fi
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} bag_file:=${CONTAINER_BAG_FILE}"
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
                if python3 "$FIX_SCRIPT" "$BAG_DIR" 2>/dev/null; then
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
            else
                print_warning "[BAG] [PRECHECK] metadata.yaml 解析失败（将导致 ros2 bag play 报 bad conversion，上方有解析异常详情）"
                print_warning "[BAG] 建议宿主机执行: chmod u+w ${BAG_META} && python3 $FIX_SCRIPT $BAG_DIR"
            fi
        else
            print_info "[BAG] 未检测到目录或 metadata.yaml（path=${BAG_DIR}），跳过 metadata 修复与预检查"
        fi
    elif [ "$MODE" = "offline" ]; then
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} bag_file:=/data/mapping"
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

    print_info "启动命令: ros2 launch ${LAUNCH_SPEC} ${LAUNCH_ARGS}"

    # 使用外部前端时必须有 fast_livo 已编译，否则 launch 会跳过该节点、后端无数据
    if [ "$USE_EXTERNAL_FRONTEND" = true ]; then
        if [ ! -d "${WORKSPACE_DIR}/install/fast_livo" ] && [ ! -f "${WORKSPACE_DIR}/install/fast_livo/share/fast_livo/package.xml" ]; then
            print_error "未找到 fast_livo 安装（install/fast_livo），无法启动前端节点，后端将一直等待数据。"
            echo ""
            echo "  若源码已在 automap_pro/src/modular/fast-livo2-humble 下，请执行完整编译（会编译 fast_livo 并安装）："
            echo "    bash run_automap.sh --offline --bag-file \"\$(pwd)/data/automap_input/M2DGR/street_03_ros2\" --config system_config_M2DGR.yaml --clean"
            echo "  编译时请查看 build.log 中 \"[INFO] fast_livo 源码检查\" 和 \"编译 fast_livo\" 是否出现；若无则容器内未找到该路径。"
            echo ""
            echo "  或使用内部前端：加参数 --no-external-frontend 再运行（若支持）。"
            echo "  当前检查路径: ${WORKSPACE_DIR}/install/fast_livo"
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

    print_info "启动容器..."
    print_info "按 Ctrl+C 停止系统"
    # 诊断提示：若日志出现 undefined symbol buildGlobalMap 或 Camera model not specified，见下方说明
    echo ""
    echo "[DIAG] 故障定位: grep -E 'LINK_4_PARAMS|FAST_LIVO_PARAMS|BAG|HBA|config_file param|undefined symbol|camera_loader|parameter_blackboard' 可精准查看参数与符号问题"
    echo "[DIAG] 若出现 undefined symbol buildGlobalMap → 请勿使用 --run-only，重新执行本脚本以触发 automap_pro 完整编译"
    echo "[DIAG] 若出现 Camera model not specified → 查看 [FAST_LIVO_PARAMS] parameter_blackboard.model 及 [fast_livo] [DIAG] parameter_blackboard.model"
    echo "[DIAG] 若出现 Exception on parsing info file / bad conversion → 脚本已尝试修复 metadata；未修复时请手动: python3 scripts/fix_ros2_bag_metadata.py <bag目录>"
    echo "[DIAG] 若出现 HBA vector::_M_default_append 或 pose_size=0 → 查看 [HBA] [DATA] lio_pose_orig.size 与 [HBA] [FATAL]；确保 bag 回放且 fast_livo 先输出位姿"
    echo "[DIAG] 若无 GPS（轨迹 CSV 无 gps_x/gps_y/gps_z）：grep -E 'LivoBridge\\[GPS\\]|GPS_DIAG|TRAJ_LOG no GPS' 日志；M2DGR 使用 sensor.gps.topic=/ublox/fix，须与 bag 话题一致"
    echo ""

    # 运行容器并启动建图系统（挂载 automap_pro 以便容器内能访问 config 等路径）
    # 若指定了 --log-dir，则 LOG_MOUNT 已包含 -v 宿主机目录:/root/run_logs，容器内 tee 写入 full.log 即保存到宿主机
    set +e
    docker run -it --rm \
        ${TIME_VOLUMES} \
        --gpus all \
        --privileged \
        --net=host \
        --ipc=host \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -e OMP_NUM_THREADS=1 \
        -e EIGEN_NUM_THREADS=1 \
        -e MKL_NUM_THREADS=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
        -v "${PROJECT_DIR}:/root/automap_ws/src/automap_pro:ro" \
        -v "${DATA_DIR}:/data:rw" \
        ${LOG_MOUNT} \
        -w /root/automap_ws \
        "${IMAGE_NAME}" \
        /bin/bash -c "
            # ✅ 修复：限制线程数以避免 GTSAM TBB 并行导致 SIGSEGV
            export OMP_NUM_THREADS=1
            export EIGEN_NUM_THREADS=1
            export MKL_NUM_THREADS=1

            source /opt/ros/humble/setup.bash
            # 先加载 install_deps（GTSAM/TEASER++/vikit），再加载主工作空间
            if [ -f install_deps/setup.bash ]; then source install_deps/setup.bash; fi
            for d in install_deps/gtsam/lib install_deps/teaserpp/lib; do [ -d \"\$d\" ] && export LD_LIBRARY_PATH=\"\$d:\$LD_LIBRARY_PATH\"; done
            source install/setup.bash
            ${CONTAINER_LAUNCH_CMD}
        "
    DOCKER_EXIT=$?
    set -e
    if [ "$DOCKER_EXIT" -ne 0 ]; then
        echo ""
        print_error "容器退出码: ${DOCKER_EXIT}（非 0 表示异常退出）"
        echo "[ERROR] 常见原因与排查:"
        echo "  - 退出码 1:    launch/节点启动失败 → 查看上方 [automap_offline][EXCEPTION] 或 [params_from_system_config][EXCEPTION]"
        echo "  - 退出码 127:  undefined symbol → 请勿用 --run-only，重新执行以触发 automap_pro 编译"
        echo "  - 退出码 245:  fast_livo segfault → 检查 [FAST_LIVO_PARAMS] parameter_blackboard.model 及 config 中 fast_livo 节"
        echo "  - 退出码 -6:    SIGABRT(如 HBA vector::_M_default_append) → 查看 [HBA] [FATAL]/[HBA][layer] 或 [camera_loader]；pose_size=0 时需 bag 回放且 fast_livo 先出位姿"
        echo "  - yaml-cpp bad conversion: 脚本已尝试修复 metadata；未生效时请: python3 scripts/fix_ros2_bag_metadata.py <bag目录>"
        echo "[ERROR] 精准定位: grep -E 'EXCEPTION|ERROR|FATAL|undefined symbol|what\\(\\)|\\[HBA\\]|\\[camera_loader\\]|\\[BAG\\]' 上述输出"
        exit "$DOCKER_EXIT"
    fi
}

# ==================== 主函数 ====================
main() {
    print_header "AutoMap-Pro 一键编译和运行脚本"

    # 解析参数
    parse_args "$@"

    # 未指定 --log-dir 时使用带时间戳子目录，便于区分每次运行
    if [ "${LOG_DIR_USER_SET}" != "true" ] && [ "${LOG_USE_TIMESTAMP_SUBDIR}" = "true" ]; then
        LOG_DIR="${SCRIPT_DIR}/logs/run_$(date +%Y%m%d_%H%M%S)"
    fi

    # 确保日志目录存在，并将本次运行的全部输出带时间戳写入 logs/automap.log，便于精准分析
    mkdir -p "${LOG_DIR}"
    exec > >(add_timestamp | tee "${LOG_DIR}/automap.log") 2>&1

    print_info "本次运行全部日志同时写入: ${LOG_DIR}/automap.log"

    # 执行流程
    check_dependencies
    ensure_image
    prepare_workspace
    build_project
    run_system

    print_success "✓ 所有操作完成"
}

# ==================== 执行 ====================
main "$@"
