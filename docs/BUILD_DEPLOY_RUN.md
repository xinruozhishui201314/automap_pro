# AutoMap-Pro 编译/部署/运行说明

> **主入口**：**run_automap.sh**（支持 --offline/--config/--bag-file/--bag-rate/--gdb）。可选：automap_start.sh。唯一主配置 `system_config.yaml`；Docker 镜像 `automap-env:humble`；工作空间 `automap_ws`。

---

## 0. Executive Summary

- **推荐入口**：仓库根目录执行 `bash run_automap.sh`（或 `bash run_automap.sh --offline --bag-file <path> --config <yaml>`），一键完成编译（容器内）与运行（离线/在线 + 建图 + RViz2）。
- **主配置**：`automap_pro/config/system_config.yaml`，可通过 `--config` 指定其它（如 `system_config_M2DGR.yaml`）；overlap_transformer / HBA / fast-livo2 参数均由此或由其生成。
- **Launch**：离线为 `automap_offline.launch.py`，在线为 `automap_online.launch.py`；由 `run_automap.sh` 根据模式选择。
- **路径（run_automap.sh）**：宿主机 `automap_ws/`、`automap_pro/`、`data/`、`logs/`；建图输出由配置 `system.output_dir` 决定（默认 `/data/automap_output`，容器内挂载为 `data/automap_output`）。日志：宿主机 `logs/`（automap.log、build.log、full.log），容器内运行日志 tee 到 `/root/run_logs/full.log` 挂载回宿主机。
- **前端**：默认 Fast-LIVO2，由 `system_config.yaml` 与 launch 参数 `use_external_frontend` 控制。

---

## 1. 工程目录约定

| 路径 | 说明 |
|------|------|
| `run_automap.sh` | **主入口**：一键编译 & 运行（--offline/--config/--bag-file/--bag-rate/--gdb） |
| `automap_start.sh` | 可选入口：简化 build/run（默认 nya_02_ros2） |
| `automap_ws/` | ROS2 工作空间（build/install 在此；Docker 挂载） |
| `automap_pro/` | 主包源码（挂载到容器内 `automap_ws/src/automap_pro`） |
| `automap_pro/config/system_config.yaml` | 系统主配置（可 --config 指定其它 yaml） |
| `automap_pro/launch/` | automap_offline / automap_online |
| `data/` | 输入 bag 与数据集；建图输出默认 `data/automap_output/` |
| `logs/` | 运行日志（automap.log、build.log、full.log；可用 --log-dir 覆盖） |
| `docker/` | 镜像与 Dockerfile（`automap-env:humble`） |
| `scripts/` | 辅助脚本（Bag 修复、诊断、验证等） |

### 1.1 容器内路径（run_automap.sh）

| 容器内路径 | 宿主机路径 |
|------------|------------|
| `/root/automap_ws` | `automap_ws/` |
| `/root/automap_ws/src/automap_pro` | `automap_pro/` |
| `/data` | `data/`（含 bag 与 automap_output） |
| `/root/run_logs` | `logs/`（或 --log-dir 指定；full.log） |
| `/root/automap_ws/src/fast_livo` | 来自 `automap_pro/src/modular/fast-livo2-humble` 或仓库根 |
| `/root/automap_ws/src/hba` | 来自 `HBA-main/HBA_ROS2`（脚本链入） |

---

## 2. 环境要求

- **OS**：Linux（推荐 Ubuntu 22.04）
- **ROS2**：Humble（容器内已装）
- **Docker**：推荐使用，镜像 `automap-env:humble`（可从 `docker/automap-env_humble.tar` 加载）
- **GPU**：可选，无则 CPU 模式

---

## 3. 构建

### 3.1 使用 run_automap.sh（推荐）

```bash
# 在仓库根目录：仅编译
bash run_automap.sh --build-only

# 清理后重新编译
bash run_automap.sh --build-only --clean
```

在容器内执行：source ROS2、链入依赖、colcon build（overlap_transformer_msgs → overlap_transformer_ros2 → hba → TEASER++ → fast_livo → automap_pro），产物在挂载的 `automap_ws/build` 与 `automap_ws/install`。

### 3.2 宿主机本地构建

```bash
cd automap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select automap_pro fast_livo hba hba_api --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 4. 运行

### 4.1 使用 run_automap.sh（推荐）

```bash
# 一键编译 + 运行（在线）
bash run_automap.sh

# 仅运行（须先编译）
bash run_automap.sh --run-only

# 离线回放 + 指定 bag 与配置
bash run_automap.sh --offline --bag-file data/automap_input/M2DGR/street_03_ros2 --config system_config_M2DGR.yaml --bag-rate 0.5

# 不启 RViz
bash run_automap.sh --offline --bag-file <path> --no-rviz
```

脚本会：挂载 volume、启动容器、离线时播放 bag、执行 `ros2 launch automap_pro automap_offline.launch.py`（或 online）并传入 config、bag_file、rate、use_rviz 等。

### 4.2 直接调用 launch（宿主机或容器内）

```bash
source install/setup.bash
export AUTOMAP_LOG_DIR=/path/to/logs   # 可选

ros2 launch automap_pro automap_composable.launch.py \
  config:=/path/to/automap_pro/config/system_config.yaml \
  use_rviz:=true
```

需另终端播放 bag（如 `ros2 bag play ... --clock`）。

---

## 5. Docker 挂载说明（automap_start.sh）

镜像 **仅提供**：ROS2 Humble、CUDA、系统依赖、编译工具链。**不包含**工程源码与编译产物，均通过挂载写入宿主机。

| 宿主机 | 容器内 |
|--------|--------|
| `automap_ws/` | `/workspace/automap_ws` |
| `automap_pro/` | `/workspace/automap_ws/src/automap_pro` |
| `data/.../nya_02_ros2/`（bag 目录） | `/workspace/data/bag` |
| `logs/automap_*/` | `/workspace/logs` |
| `output/` | `/workspace/output` |

工作目录：容器内 `-w /workspace/automap_ws`。

---

## 6. 配置与模块开关

- **主配置**：`automap_pro/config/system_config.yaml`
- **Launch 参数**：`config`、`use_rviz`、`composable`、`fast_livo_config` 等；composable 默认使用 `automap_composable.launch.py` 内默认路径（非空，避免 [Errno 21]）。

Modular 节点由 system_config 与 launch 参数控制（如 use_external_frontend、use_hba 等），详见 launch 文件与 CONFIG_SUMMARY.md。

---

## 7. 验证

- **配置**：`automap_pro/scripts/verify_system_config_launch.sh [config.yaml]`（若存在）
- **建图流水线**：`./verify_mapping_pipeline.sh`（若存在）
- **日志**：宿主机 `logs/automap_YYYYMMDD_HHMMSS/`，容器内 `/workspace/logs`

---

## 8. 常见问题

| 现象 | 处理 |
|------|------|
| 找不到 `system_config.yaml` | 确认使用 `automap_start.sh` 或传入 `config:=/path/to/automap_pro/config/system_config.yaml` |
| Launch 报 [Errno 21] Is a directory | 已修复；确保使用当前 `automap_pro/launch/automap_composable.launch.py` 与 `rviz/automap.rviz` |
| 容器内无源码 | 确认宿主机存在 `automap_pro/`、`automap_ws/`，脚本会挂载到容器 |
| 日志不在宿主机 | 确认脚本挂载了 `logs`，且容器内 `AUTOMAP_LOG_DIR=/workspace/logs` |

---

## 9. 变更记录（重构后）

- 推荐入口统一为 **automap_start.sh**（Docker + automap_ws + automap_pro 挂载）。
- 默认 launch 为 **automap_composable.launch.py**（Composable 零拷贝），配置与 RViz 路径使用非空默认值。
- 日志目录：宿主机 `logs/automap_YYYYMMDD_HHMMSS/`，容器 `/workspace/logs`。
- 唯一主配置：`automap_pro/config/system_config.yaml`。

---

文档版本：v2.0（重构后）  
更新日期：2026-03-03
