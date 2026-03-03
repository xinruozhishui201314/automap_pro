# AutoMap-Pro 编译/部署/运行说明

> 重构约定：唯一配置、统一 Launch、三方库路径、日志路径、Docker 挂载

---

## 0. Executive Summary

- **唯一配置**：工程启动仅使用 `automap_pro/config/system_config.yaml`，overlap_transformer / HBA / fast-livo2 参数均由此文件生成。
- **Launch**：入口 launch 均位于 `automap_pro/launch/`（offline / online / incremental / visualization）。
- **三方库**：工程依赖统一放在 `thrid_party/`（优先 repo 根目录，否则 `automap_pro/thrid_party`），Docker 内挂载到工作空间 `src/thrid_party`。
- **日志**：统一写入 `logs/`（宿主机与容器一致）；launch 内 fast_livo 参数文件等通过环境变量 `AUTOMAP_LOG_DIR` 指定。
- **Docker**：镜像 `automap-env:humble` 仅提供编译与运行环境，**所有编译与运行产物均在挂载目录**（不写入镜像）。
- **前端与 fast_livo**：建图使用 **modular 中已验证的 fast_livo**（FAST-LIVO2 官方 LIVMapper）。进程内 **FastLIVO2Wrapper** 为自研 ESIKF 前端，非同一算法，仅作 fallback。

---

## 1. 工程目录约定

| 路径 | 说明 |
|------|------|
| `automap_pro/config/system_config.yaml` | 唯一系统配置文件 |
| `automap_pro/launch/` | 入口 launch 文件（automap_offline / online / incremental / visualization），**统一启动 modular 节点**：fast_livo、overlap_transformer_ros2、HBA |
| `thrid_party/` 或 `automap_pro/thrid_party/` | 三方库（nlohmann-json3、ceres-solver、Sophus 等） |
| `logs/` | 统一日志目录（建图脚本日志、fast_livo_params.yaml、launch 子目录等） |
| `data/` | 输入 bag 与建图数据 |
| `automap_ws/` | ROS2 工作空间（build/install/log 均在此，挂载到容器） |

### 1.1 FastLIVO2Wrapper 与 前端 fast_livo 的关系

| 名称 | 位置 | 说明 |
|------|------|------|
| **fast_livo**（已验证） | modular fast-livo2-humble，ROS2 包 `fast_livo`，节点 `fastlivo_mapping` | FAST-LIVO2 官方实现（LIVMapper），LiDAR-Inertial-Visual Odometry，**精度已验证**，建图工程默认使用。 |
| **FastLIVO2Wrapper** | automap_pro 进程内 | 自研 ESIKF LiDAR-IMU 里程计（点面残差 + 体素局部图），**非** fast_livo 官方算法，精度未与 fast_livo 对齐，仅作无 fast_livo 节点时的 fallback。 |

- 配置 `frontend.mode: external_fast_livo` 且 launch `use_external_frontend:=true` 时：由 **fast_livo 节点**订阅 lidar/IMU/image，发布 odom+cloud，automap_system 用 FastLIVO2Adapter 消费，再供给子图/回环/HBA。
- 数据流：原始数据 → **fast_livo 节点**（唯一数据入口）→ odom/cloud → automap_system → 其他模块。

---

## 2. 环境要求

- **OS**：Linux（推荐 Ubuntu 22.04）
- **ROS2**：Humble
- **Docker**（可选）：用于一键建图时，需已拉取或构建镜像 `automap-env:humble`
- **依赖**：colcon、Python3、yaml、Eigen、PCL、OpenCV、GeographicLib 等（见 `automap_pro/package.xml` 与各子模块）

---

## 3. 依赖安装（宿主机）

```bash
cd /path/to/automap_pro
# 若使用 automap_ws 且需从 thrid_party 编译
# 确保 thrid_party 在 repo 根或 automap_pro/thrid_party 存在
sudo apt update
rosdep update
rosdep install --from-paths automap_pro --ignore-src -r -y
```

---

## 4. 构建

### 4.1 宿主机构建

```bash
cd /path/to/automap_pro/automap_ws
source /opt/ros/humble/setup.bash
# 若 nlohmann_json 使用本地 thrid_party（thrid_party 需在 src 下或通过 -D 指定）
colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
# 可选：-DNLOHMANN_JSON_LOCAL=/path/to/thrid_party/nlohmann-json3
source install/setup.bash
```

### 4.2 Docker 内构建（推荐）

由 `run_full_mapping_docker.sh` 挂载工程后，在容器内执行：

```bash
cd /workspace/automap_pro
export WORKSPACE=/workspace/automap_ws
ln -sfn /workspace/automap_ws $HOME/automap_ws 2>/dev/null || true
source /opt/ros/humble/setup.bash
cd $WORKSPACE && colcon build --packages-select automap_pro --cmake-args -DCMAKE_BUILD_TYPE=Release
```

产物在挂载的 `automap_ws/build` 与 `automap_ws/install`，不在镜像内。

---

## 5. 运行

### 5.1 使用唯一配置文件

默认即使用 `automap_pro/config/system_config.yaml`：

```bash
# 宿主机
./run_full_mapping_enhanced.sh

# Docker 一键建图（推荐）
./run_full_mapping_docker.sh
# 或指定 bag
./run_full_mapping_docker.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

如需临时使用其他配置（不推荐，仅调试）：

```bash
CONFIG=automap_pro/config/system_config_nya02.yaml ./run_full_mapping_docker.sh -b ...
```

### 5.2 直接调用 launch

```bash
source /path/to/automap_ws/install/setup.bash
export AUTOMAP_LOG_DIR=/path/to/automap_pro/logs   # 可选，不设则用 automap_pro/logs
ros2 launch automap_pro automap_offline.launch.py \
  config:=/path/to/automap_pro/config/system_config.yaml \
  bag_file:=/path/to/your.bag
```

**Modular 节点开关**（参数均来自 system_config.yaml）：

- `use_external_frontend:=true`（默认）— 启动 fast_livo
- `use_external_overlap:=true` — 启动 overlap_transformer descriptor_server
- `use_hba:=true`（默认）— 启动 HBA 后端节点
- `use_hba_cal_mme:=true` — 启动 HBA cal_MME 节点
- `use_hba_visualize:=true` — 启动 HBA 可视化节点

---

## 6. Docker 挂载说明（automap-env:humble）

镜像 **仅提供**：ROS2 Humble、系统依赖、编译工具链。**不包含**工程源码与产物。

| 宿主机路径 | 容器内路径 | 说明 |
|------------|------------|------|
| `$SCRIPT_DIR` | `/workspace/automap_pro` | 工程根（含 config、launch、scripts） |
| `$SCRIPT_DIR/automap_ws` | `/workspace/automap_ws` | 工作空间（build/install/log） |
| `$SCRIPT_DIR/data` | `/workspace/data` | 数据与 bag |
| `$SCRIPT_DIR/logs` | `/workspace/automap_pro/logs` | 统一日志目录 |
| `$OUTPUT_DIR_LOCAL` | `/workspace/output` | 建图输出 |
| `thrid_party` 或 `automap_pro/thrid_party` | `/workspace/automap_ws/src/thrid_party` | 三方库 |

容器内工作目录为 `/workspace/automap_pro`，会设置 `WORKSPACE=/workspace/automap_ws`、`CONFIG`、`BAG_FILE`、`OUTPUT_DIR`、`AUTOMAP_LOG_DIR`。

---

## 7. 验证

- **配置**：`automap_pro/scripts/verify_system_config_launch.sh [config.yaml]`
- **建图流水线**：`./verify_mapping_pipeline.sh`
- **日志**：主日志在 `logs/full_mapping_*.log`，launch 子目录在 `logs/launch_*.d/`，fast_livo 参数文件在 `logs/fast_livo_params.yaml`

---

## 8. 常见问题

| 现象 | 处理 |
|------|------|
| 找不到 `system_config.yaml` | 确认使用默认配置或 `-c automap_pro/config/system_config.yaml` |
| fast_livo 报 `parameter ''` | 检查 `logs/fast_livo_params.yaml` 是否含空键；确认 launch 使用 params_from_system_config 生成参数 |
| 容器内无 thrid_party | 确认 repo 根存在 `thrid_party` 或 `automap_pro/thrid_party`，脚本会择一挂载 |
| 日志不在宿主机 | 确认已挂载 `logs`，且容器内 `AUTOMAP_LOG_DIR=/workspace/automap_pro/logs` |

---

## 9. 变更清单（本次重构）

- 默认配置由 `system_config_nya02.yaml` 改为 `system_config.yaml`（`run_full_mapping_docker.sh`、`run_full_mapping_enhanced.sh`、`docker/mapping.sh`）。
- 日志统一到 `logs/`：`run_full_mapping_enhanced.sh` 导出 `AUTOMAP_LOG_DIR`，`automap_offline.launch.py` 优先使用该环境变量。
- Docker 增加 `logs` 挂载；三方库支持 repo 根 `thrid_party` 与 `automap_pro/thrid_party` 二选一挂载。
- `verify_mapping_pipeline.sh` 中 fast_livo 参数文件路径改为 `SCRIPT_DIR/logs/fast_livo_params.yaml`。
