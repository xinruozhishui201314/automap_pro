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
#   --build-only       仅编译不运行
#   --run-only         仅运行不编译（假设已编译）
#   --no-rviz          不启动 RViz 可视化
#   --no-external-frontend 使用自研前端（默认使用 Fast-LIVO2）
#   --external-overlap  使用 OverlapTransformer 服务作为回环描述子
#   --clean            清理编译产物后重新编译
#   --help             显示帮助信息
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
BUILD_ONLY=false
RUN_ONLY=false
USE_RVIZ=true
CLEAN_BUILD=false
USE_EXTERNAL_FRONTEND=true
USE_EXTERNAL_OVERLAP=false

# ==================== 帮助信息 ====================
show_help() {
    cat << EOF
AutoMap-Pro 一键编译和运行脚本

使用方法:
    bash run_automap.sh [选项]

  选项:
    --online           在线模式（实时建图，默认）
    --offline          离线模式（回放 rosbag）
    --bag-file <path>  指定 rosbag 文件路径（离线模式使用）
    --build-only       仅编译不运行
    --run-only         仅运行不编译（假设已编译）
    --no-rviz          不启动 RViz 可视化
    --no-external-frontend 使用自研前端（默认使用 fast-livo2-humble）
    --external-overlap  使用 OverlapTransformer 服务作为回环描述子（需 config 中 loop_closure.overlap_transformer.mode: external_service）
    --clean            清理编译产物后重新编译
    --help             显示帮助信息

示例:
    bash run_automap.sh                                        # 在线模式建图
    bash run_automap.sh --offline --bag-file /data/record.mcap # 离线模式回放
    bash run_automap.sh --build-only                          # 仅编译
    bash run_automap.sh --run-only --no-rviz                   # 运行但不启动 RViz

目录说明:
    - 工作空间: ${WORKSPACE_DIR}
    - 数据目录: ${DATA_DIR}
    - 输出目录: ${OUTPUT_DIR}

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

    # 创建必要的目录
    print_info "创建必要的目录..."
    mkdir -p "${WORKSPACE_DIR}/src"
    mkdir -p "${DATA_DIR}"
    mkdir -p "${OUTPUT_DIR}"

    print_success "✓ 目录创建完成"
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
        print_info "从归档文件加载镜像: ${IMAGE_ARCHIVE}"
        docker load -i "${IMAGE_ARCHIVE}"
        if [ $? -eq 0 ]; then
            print_success "✓ 镜像加载完成"
            return 0
        else
            print_error "镜像加载失败"
            exit 1
        fi
    fi

    # 构建镜像
    print_warning "⚠ 镜像不存在，开始构建..."
    print_info "构建时间预计: 30-45 分钟"

    cd "${SCRIPT_DIR}/docker"
    docker build -t "${IMAGE_NAME}" .

    if [ $? -ne 0 ]; then
        print_error "镜像构建失败"
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
        print_info "清理编译产物..."
        if docker run --rm \
            -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
            "${IMAGE_NAME}" \
            /bin/bash -c "rm -rf /root/automap_ws/build /root/automap_ws/build_teaserpp /root/automap_ws/build_sophus /root/automap_ws/build_ceres /root/automap_ws/install /root/automap_ws/log && echo 'cleaned'"; then
            print_success "✓ 清理完成"
        else
            print_error "✗ 清理失败（若曾用 sudo 编译，请在本机执行: sudo rm -rf ${WORKSPACE_DIR}/build ${WORKSPACE_DIR}/build_teaserpp ${WORKSPACE_DIR}/install ${WORKSPACE_DIR}/log）"
            exit 1
        fi
    fi

    # 检查是否已经编译（automap_pro 必选；若存在 fast_livo 源码则其也需已安装）
    NEED_BUILD=false
    if [ ! -f "${WORKSPACE_DIR}/install/automap_pro/share/automap_pro/package.xml" ]; then
        NEED_BUILD=true
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

    docker run --rm \
        --gpus all \
        --net=host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
        -v "${PROJECT_DIR}:/root/automap_ws/src/automap_pro:ro" \
        ${THRID_PARTY_MOUNT} \
        ${MAPPING_MOUNT} \
        ${FAST_LIVO_MOUNT} \
        "${IMAGE_NAME}" \
        /bin/bash -c "
            set -e
            source /opt/ros/humble/setup.bash
            cd /root/automap_ws

            # 链入 overlap_transformer_msgs / overlap_transformer_ros2 / hba（若存在）
            [ -d /root/mapping/overlap_transformer_msgs ] && ln -sf /root/mapping/overlap_transformer_msgs src/ 2>/dev/null || true
            [ -d /root/mapping/overlap_transformer_ros2 ] && ln -sf /root/mapping/overlap_transformer_ros2 src/ 2>/dev/null || true
            [ -d /root/mapping/HBA-main/HBA_ROS2 ]        && ln -sf /root/mapping/HBA-main/HBA_ROS2 src/hba 2>/dev/null || true

            if [ -d src/overlap_transformer_msgs ]; then
              echo '========================================'
              echo '编译 overlap_transformer_msgs'
              echo '========================================'
              colcon build --packages-select overlap_transformer_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
            fi
            if [ -d src/overlap_transformer_ros2 ]; then
              echo '========================================'
              echo '编译 overlap_transformer_ros2'
              echo '========================================'
              colcon build --packages-select overlap_transformer_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
            fi
            if [ -d src/hba ]; then
              echo '========================================'
              echo '编译 hba (HBA-main)'
              echo '========================================'
              colcon build --packages-select hba --cmake-args -DCMAKE_BUILD_TYPE=Release
            fi
            # TEASER++ 源码：优先仓库根，否则 automap_pro/src/modular（本仓库实际路径）
            TEASER_SRC=\"\"
            [ -d /root/mapping/TEASER-plusplus-master ] && TEASER_SRC=/root/mapping/TEASER-plusplus-master
            [ -z \"\${TEASER_SRC}\" ] && [ -d /root/mapping/automap_pro/src/modular/TEASER-plusplus-master ] && TEASER_SRC=/root/mapping/automap_pro/src/modular/TEASER-plusplus-master
            if [ -n \"\${TEASER_SRC}\" ]; then
              echo '========================================'
              echo '编译 TEASER++ (上游源码，优先于系统版本）'
              echo '========================================'
              # 源码在只读挂载 /root/mapping 下，必须在可写的 automap_ws 内建 build 目录
              TEASER_BUILD=/root/automap_ws/build_teaserpp
              mkdir -p /root/automap_ws/install/teaserpp
              mkdir -p \"\${TEASER_BUILD}\" && cd \"\${TEASER_BUILD}\"
              # thrid_party：优先使用含 pmc-master 的路径（本仓库为 automap_pro/thrid_party）
              MAP_THRID=/root/mapping/automap_pro/thrid_party
              [ ! -d \"\${MAP_THRID}/pmc-master\" ] && MAP_THRID=/root/mapping/thrid_party
              PMC_SRC=\"\${MAP_THRID}/pmc-master\"
              TINYPLY_SRC=\"\${MAP_THRID}/tinyply\"
              SPECTRA_SRC=\"\${MAP_THRID}/spectra\"
              GOOGLETEST_SRC=\"\${MAP_THRID}/googletest\"
              if [ ! -d \"\${PMC_SRC}\" ]; then
                echo \"✗ 错误: PMC 源码不存在于 \${PMC_SRC}\"
                exit 1
              fi
              if [ ! -d \"\${TINYPLY_SRC}\" ]; then
                echo \"✗ 错误: tinyply 源码不存在于 \${TINYPLY_SRC}\"
                exit 1
              fi
              if [ ! -d \"\${SPECTRA_SRC}\" ]; then
                echo \"✗ 错误: spectra 源码不存在于 \${SPECTRA_SRC}\"
                exit 1
              fi
              if [ ! -d \"\${GOOGLETEST_SRC}\" ]; then
                echo \"✗ 错误: googletest 源码不存在于 \${GOOGLETEST_SRC}\"
                exit 1
              fi
              echo \"使用本地 PMC 源码: \${PMC_SRC}\"
              echo \"使用本地 tinyply 源码: \${TINYPLY_SRC}\"
              echo \"使用本地 spectra 源码: \${SPECTRA_SRC}\"
              echo \"使用本地 googletest 源码: \${GOOGLETEST_SRC}\"
              cmake -DCMAKE_BUILD_TYPE=Release \
                    -DCMAKE_INSTALL_PREFIX=/root/automap_ws/install/teaserpp \
                    -DBUILD_TEASER_FPFH=ON \
                    -DBUILD_TESTS=OFF \
                    -DBUILD_PYTHON_BINDINGS=OFF \
                    -DFETCHCONTENT_SOURCE_DIR_PMC=\"\${PMC_SRC}\" \
                    -DFETCHCONTENT_SOURCE_DIR_TINYPLY=\"\${TINYPLY_SRC}\" \
                    -DFETCHCONTENT_SOURCE_DIR_SPECTRA=\"\${SPECTRA_SRC}\" \
                    -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=\"\${GOOGLETEST_SRC}\" \
                    -DCMAKE_PREFIX_PATH=\"\" \
                    \"\${TEASER_SRC}\"
              make -j\$(nproc) && make install
              cd /root/automap_ws
              # 设置 CMAKE_PREFIX_PATH，确保工作空间版本优先
              export CMAKE_PREFIX_PATH=/root/automap_ws/install/teaserpp:\$CMAKE_PREFIX_PATH
              echo \"✓ TEASER++ 源码编译完成，安装到工作空间\"
            else
              echo \"========================================\"
              echo \"TEASER-plusplus-master 未找到，将使用系统安装的 TEASER++（如果存在）\"
              echo \"========================================\"
            fi
            # 确保 src/fast_livo 存在：优先使用挂载；若无则从 mapping 链入（automap_pro/src/modular/fast-livo2-humble）
            if [ ! -f src/fast_livo/package.xml ] && [ -d /root/mapping/automap_pro/src/modular/fast-livo2-humble ]; then
              rm -f src/fast_livo 2>/dev/null || true
              ln -sfn /root/mapping/automap_pro/src/modular/fast-livo2-humble src/fast_livo
              echo \"[INFO] 已从 mapping 链入 fast-livo2-humble → src/fast_livo\"
            fi
            if [ -f src/fast_livo/package.xml ]; then
              echo '========================================'
              echo '编译 vikit_common / vikit_ros (rpg_vikit_ros2 本地)'
              echo '========================================'
              colcon build --paths src/thrid_party/rpg_vikit_ros2/vikit_common src/thrid_party/rpg_vikit_ros2/vikit_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
              source install/setup.bash
              echo '========================================'
              echo '编译 Sophus (fast_livo 依赖)'
              echo '========================================'
              # Sophus 的 test/ceres 需要 Ceres（ceres/manifold.h）。若使用本地 ceres-solver 则先编译并安装，再让 Sophus 链接；否则关闭 Sophus 测试。
              CERES_SRC=/root/automap_ws/src/thrid_party/ceres-solver
              CERES_INSTALL=/root/automap_ws/install/ceres
              SOPHUS_CMAKE_EXTRA=\"-DBUILD_SOPHUS_TESTS=OFF\"
              if [ -d \"\${CERES_SRC}\" ]; then
                echo '使用本地 ceres-solver，先编译并安装到 install/ceres'
                mkdir -p /root/automap_ws/build_ceres \"\${CERES_INSTALL}\"
                (cd /root/automap_ws/build_ceres && cmake -DCMAKE_BUILD_TYPE=Release \
                  -DCMAKE_INSTALL_PREFIX=\"\${CERES_INSTALL}\" \
                  -DBUILD_TESTING=OFF \
                  -DBUILD_EXAMPLES=OFF \
                  -DBUILD_BENCHMARKS=OFF \
                  -DMINIGLOG=ON \
                  -DCERES_ADD_GERRIT_COMMIT_HOOK=OFF \
                  \"\${CERES_SRC}\" && make -j\$(nproc) && make install)
                if [ -f \"\${CERES_INSTALL}/lib/cmake/Ceres/CeresConfig.cmake\" ] || [ -f \"\${CERES_INSTALL}/lib64/cmake/Ceres/CeresConfig.cmake\" ]; then
                  CERES_CMAKE_DIR=\"\${CERES_INSTALL}/lib/cmake/Ceres\"
                  [ -f \"\${CERES_INSTALL}/lib64/cmake/Ceres/CeresConfig.cmake\" ] && CERES_CMAKE_DIR=\"\${CERES_INSTALL}/lib64/cmake/Ceres\"
                  SOPHUS_CMAKE_EXTRA=\"-DCeres_DIR=\${CERES_CMAKE_DIR}\"
                  echo \"✓ 本地 Ceres 已安装，Sophus 将使用 Ceres_DIR=\${CERES_CMAKE_DIR}\"
                else
                  echo \"⚠ Ceres 安装未找到 CeresConfig.cmake，Sophus 将关闭测试编译\"
                fi
              fi
              # Sophus 无 package.xml，不能用 colcon；用 cmake 单独编译并安装到 install/sophus
              SOPHUS_SRC=/root/automap_ws/src/thrid_party/Sophus
              SOPHUS_BUILD=/root/automap_ws/build_sophus
              SOPHUS_INSTALL=/root/automap_ws/install/sophus
              mkdir -p \"\${SOPHUS_BUILD}\" \"\${SOPHUS_INSTALL}\"
              (cd \"\${SOPHUS_BUILD}\" && cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=\"\${SOPHUS_INSTALL}\" -DSOPHUS_INSTALL=ON \${SOPHUS_CMAKE_EXTRA} \"\${SOPHUS_SRC}\" && make -j\$(nproc) && make install)
              export CMAKE_PREFIX_PATH=\"\${SOPHUS_INSTALL}:\$CMAKE_PREFIX_PATH\"
              source install/setup.bash
              echo '========================================'
              echo '编译 fast_livo (fast-livo2-humble)'
              echo '========================================'
              colcon build --paths src/fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release -DSophus_DIR=\"\${SOPHUS_INSTALL}/share/sophus/cmake\"
            fi
            echo '========================================'
            echo '编译 automap_pro'
            echo '========================================'
            echo '[INFO] automap_pro 包较大，将实时输出编译进度（可能需 3～8 分钟）...'
            if [ -d /root/automap_ws/install/teaserpp ]; then
              export CMAKE_PREFIX_PATH=/root/automap_ws/install/teaserpp:\$CMAKE_PREFIX_PATH
            fi
            colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release -DNLOHMANN_JSON_LOCAL=/root/automap_ws/src/thrid_party/nlohmann-json3 --event-handlers console_direct+

            echo ''
            echo '========================================'
            echo '验证编译产物'
            echo '========================================'
            if [ -d src/fast_livo ]; then
              if [ -f install/fast_livo/share/fast_livo/package.xml ]; then
                echo '✓ fast_livo (fast-livo2-humble) 已安装'
                source install/setup.bash && (ros2 pkg list | grep -q '^fast_livo\$') && echo '✓ fast_livo 已进入工作空间 overlay' || echo '⚠ fast_livo 未在 ros2 pkg list 中可见（请检查 install/setup.bash）'
              else
                echo '✗ fast_livo 未找到安装产物 (install/fast_livo/share/fast_livo/package.xml)'
                exit 1
              fi
            fi
            if [ ! -f install/automap_pro/share/automap_pro/package.xml ]; then
              echo '✗ automap_pro 未找到安装产物'
              exit 1
            fi
            echo '✓ automap_pro 已安装'
            echo '✓ 验证通过'
            echo '✓ 编译完成'
        "

    if [ $? -eq 0 ]; then
        print_success "✓ 项目编译成功"
    else
        print_error "✗ 项目编译失败"
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

    # 构建启动命令
    LAUNCH_FILE="automap_online.launch.py"
    LAUNCH_ARGS="config:=${PROJECT_DIR}/config/system_config.yaml"

    if [ "$MODE" = "offline" ]; then
        LAUNCH_FILE="automap_offline.launch.py"
        if [ -n "$BAG_FILE" ]; then
            LAUNCH_ARGS="${LAUNCH_ARGS} bag_file:=${BAG_FILE}"
            print_info "离线模式，回放文件: ${BAG_FILE}"
        else
            print_warning "⚠ 未指定 rosbag 文件，使用默认路径: ${DATA_DIR}/mapping"
            LAUNCH_ARGS="${LAUNCH_ARGS} bag_file:=${DATA_DIR}/mapping"
        fi
    else
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
    CONTAINER_LAUNCH_ARGS="config:=/root/automap_ws/src/automap_pro/config/system_config.yaml ${RVIZ_ARG} ${EXT_FRONTEND_ARG} ${EXT_OVERLAP_ARG}"
    if [ "$MODE" = "offline" ] && [ -n "$BAG_FILE" ]; then
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} bag_file:=${BAG_FILE}"
    elif [ "$MODE" = "offline" ]; then
        CONTAINER_LAUNCH_ARGS="${CONTAINER_LAUNCH_ARGS} bag_file:=/data/mapping"
    fi

    print_info "启动命令: ros2 launch automap_pro ${LAUNCH_FILE} ${LAUNCH_ARGS}"

    # 配置 X11 转发
    xhost +local:docker &> /dev/null || true

    print_info "启动容器..."
    print_info "按 Ctrl+C 停止系统"
    echo ""

    # 运行容器并启动建图系统（挂载 automap_pro 以便容器内能访问 config 等路径）
    docker run -it --rm \
        --gpus all \
        --privileged \
        --net=host \
        --ipc=host \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v "${WORKSPACE_DIR}:/root/automap_ws:rw" \
        -v "${PROJECT_DIR}:/root/automap_ws/src/automap_pro:ro" \
        -v "${DATA_DIR}:/data:rw" \
        -w /root/automap_ws \
        "${IMAGE_NAME}" \
        /bin/bash -c "
            source /opt/ros/humble/setup.bash
            source install/setup.bash
            ros2 launch automap_pro ${LAUNCH_FILE} ${CONTAINER_LAUNCH_ARGS}
        "
}

# ==================== 主函数 ====================
main() {
    print_header "AutoMap-Pro 一键编译和运行脚本"

    # 解析参数
    parse_args "$@"

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
