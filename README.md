# AutoMap-Pro - 高精度自动化点云建图系统

> 基于 ROS2 Humble + CUDA 的实时建图系统，支持在线/离线模式，适用于城市道路、园区、隧道、矿山等复杂场景。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![CUDA](https://img.shields.io/badge/CUDA-11.8-green.svg)](https://developer.nvidia.com/cuda-toolkit)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## ✨ 特性

- 🚀 **实时建图**：前端频率 ≥10Hz，支持在线实时建图
- 🎯 **高精度**：全局一致性误差 <0.3%
- 🔄 **增量式**：支持多次采集数据的增量式拼接建图
- 🤖 **自动化**：端到端无人工干预完成建图
- 🌐 **多场景**：适应 GPS 信号时好时坏、退化场景（长走廊、隧道）
- 🐳 **Docker 一键**：`run_automap.sh` 一键编译与运行，支持在线/离线、指定配置与 bag

---

## 🚀 快速开始

### 前置要求

- Ubuntu 20.04 / 22.04
- Docker 20.10+
- NVIDIA GPU + Driver 470+（可选，无 GPU 时以 CPU 模式运行）
- NVIDIA Container Runtime（使用 GPU 时）

### 一键编译并运行（推荐）

**主入口脚本**：`run_automap.sh`（仓库根目录执行）

```bash
# 克隆项目
git clone <repository-url> automap_pro
cd automap_pro

# 一键编译 + 运行（在线模式，默认）
bash run_automap.sh

# 离线回放建图（指定 bag 与配置）
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --bag-rate 0.5
```

首次运行会：检查 Docker 与镜像、在容器内编译 `automap_ws`、播放 ROS2 bag（离线时）、启动建图与 RViz2。

### 常用选项

```bash
# 仅编译
bash run_automap.sh --build-only

# 仅运行（须先编译）
bash run_automap.sh --run-only

# 清理后重新编译
bash run_automap.sh --clean --build-only

# 不启动 RViz
bash run_automap.sh --no-rviz

# 离线 + 指定 rosbag 目录/文件
bash run_automap.sh --offline --bag-file /path/to/bag_dir_or.db3

# 指定配置文件（如 M2DGR / nya02 数据集）
bash run_automap.sh --offline --bag-file <path> --config system_config_M2DGR.yaml

# GDB 调试（崩溃时打印 backtrace）
bash run_automap.sh --offline --bag-file <path> --gdb

# 查看全部选项
bash run_automap.sh --help
```

**可选入口**：`automap_start.sh` 提供简化的一键 build/run（默认回放 nya_02_ros2），路径与选项与 `run_automap.sh` 略有不同，详见脚本内注释。

### 数据与输出路径

| 用途     | 宿主机路径（run_automap.sh） | 说明 |
|----------|------------------------------|------|
| 默认 bag | `data/automap_input/...`（需用 `--bag-file` 指定） | 离线时必指定 |
| 日志     | `logs/`（automap.log、build.log、full.log） | 每次运行写入 |
| 建图输出 | `data/automap_output/` | 点云、轨迹等（由 system_config 中 output_dir 决定） |

---

## 📚 文档

| 文档 | 说明 |
|------|------|
| [QUICK_START.md](QUICK_START.md) | 快速开始、常用命令与错误排查速查 |
| [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) | 编译/部署/运行说明（入口脚本、路径、Docker） |
| [docs/README.md](docs/README.md) | 文档索引（配置、建图流程、日志、故障排查） |
| [automap_pro/docs/TROUBLESHOOTING.md](automap_pro/docs/TROUBLESHOOTING.md) | 系统启动故障排查（Bag/HBA/级联故障） |
| [README_LFS.md](README_LFS.md) | Git LFS 与 GitHub 上传说明 |
| [高精度高性能自动化点云建图系统架构文档.md](高精度高性能自动化点云建图系统架构文档.md) | 系统架构与数据流 |

---

## 🏗️ 系统架构

```
传感器数据 (LiDAR/IMU/GPS)
         ↓
   前端里程计 (Fast-LIVO2，Composable 或独立进程)
         ↓
   AutoMapSystem（子图 / 回环 / 优化）
         ↓
   回环检测 (OverlapTransformer + TEASER++)
         ↓
   全局优化 (HBA)
         ↓
   地图输出
```

### 核心模块

| 模块         | 技术选型           | 说明 |
|--------------|--------------------|------|
| 前端里程计   | Fast-LIVO2         | LiDAR-IMU-(Visual) 紧耦合，Composable 零拷贝 |
| 主控         | automap_pro        | 子图管理、回环、iSAM2/HBA 优化 |
| 回环粗匹配   | OverlapTransformer | 深度学习位置识别 |
| 回环精匹配   | TEASER++           | 鲁棒点云配准 |
| 全局优化     | HBA                | 分层位姿图优化 |

---

## 📁 项目结构（重构后）

```
automap_pro/                    # 仓库根目录
├── run_automap.sh              # 🚀 一键编译 & 运行（主入口，支持 --offline/--config/--bag-file）
├── automap_start.sh            # 可选：简化一键 build/run（默认 nya_02_ros2）
├── automap_ws/                 # ROS2 工作空间（build/install 在此，Docker 挂载）
│   └── src/                    # 编译时使用的源码（含 automap_pro、fast_livo、hba 等）
├── automap_pro/                # AutoMap-Pro 主包源码（挂载到容器内 automap_ws/src/automap_pro）
│   ├── config/                 # 系统配置（system_config.yaml 为主配置，可 --config 指定其它）
│   ├── launch/                 # Launch 文件（automap_offline / automap_online）
│   ├── src/                    # 源码与 modular 子模块
│   │   └── modular/            # fast-livo2-humble、HBA-main、TEASER、overlap_transformer 等
│   ├── docs/                   # 包内文档（故障排查、坐标系、全局地图诊断等）
│   └── rviz/
├── data/                       # 数据与 rosbag、建图输出（automap_output）
│   └── automap_input/
├── docker/                     # Docker 镜像（automap-env:humble）
├── docs/                       # 工程文档索引与说明
├── scripts/                    # 辅助脚本（Bag 修复、诊断、验证等）
└── logs/                       # 运行日志（automap.log、build.log、full.log）
```

---

## 🛠️ 系统要求

| 组件 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU  | x86_64，4 核 | x86_64，8 核以上 |
| 内存 | 16GB | 32GB 以上 |
| GPU  | NVIDIA RTX 3060（可选） | NVIDIA RTX 4060 或更高 |
| 存储 | 50GB 可用 | 100GB 可用 |
| 系统 | Ubuntu 20.04 / 22.04 | Ubuntu 22.04 |

---

## 🔧 编译与运行（无 Docker）

若在宿主机直接编译运行，建议使用 `run_automap.sh` 在 Docker 内完成编译与运行；若必须本机编译：

```bash
cd automap_ws
source /opt/ros/humble/setup.bash
# 编译顺序见 run_automap.sh 内（overlap_transformer_msgs → overlap_transformer_ros2 → hba → TEASER++ → fast_livo → automap_pro）
colcon build --packages-select automap_pro fast_livo hba --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 播放 bag 与启动建图（需另终端播放 bag）
ros2 launch automap_pro automap_offline.launch.py config:=/path/to/automap_pro/config/system_config.yaml bag_file:=/path/to/bag use_rviz:=true
```

详见 [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md)。

---

## ❓ 常见问题

### 首次运行需要什么？

- 已安装 Docker，并加载或构建镜像 `automap-env:humble`（`run_automap.sh` 会检查，无则从 `docker/automap-env_humble.tar` 加载）。
- **离线模式**：必须用 `--bag-file <目录或.db3>` 指定 bag；可选 `--config <yaml>`（如 `system_config_M2DGR.yaml`）。
- **在线模式**：直接 `bash run_automap.sh`，需另有数据源发布话题。

### 如何查看建图结果？

- 建图输出在宿主机 `data/automap_output/`（由 `system_config.yaml` 中 `system.output_dir` 决定）。日志在 `logs/`（automap.log、build.log、full.log）。

### Bag 报错 yaml-cpp bad conversion / HBA pose_size=0？

- 见 [QUICK_START.md](QUICK_START.md) 错误排查速查表与 [automap_pro/docs/TROUBLESHOOTING.md](automap_pro/docs/TROUBLESHOOTING.md)：修复 metadata、降速 `--bag-rate 0.5`、检查配置与话题。

更多见 [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) 与 [QUICK_START.md](QUICK_START.md)。

---

## 📖 详细文档

- [QUICK_START.md](QUICK_START.md) - 快速开始、常用命令与错误速查
- [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) - 编译/部署/运行（run_automap.sh、路径、Docker）
- [docs/README.md](docs/README.md) - 文档索引
- [automap_pro/docs/TROUBLESHOOTING.md](automap_pro/docs/TROUBLESHOOTING.md) - 系统启动故障排查
- [README_LFS.md](README_LFS.md) - Git LFS 与推送说明
- [docker/DOCKER_USAGE.md](docker/DOCKER_USAGE.md) - Docker 使用说明
- [高精度高性能自动化点云建图系统架构文档.md](高精度高性能自动化点云建图系统架构文档.md) - 系统架构

---

## 🤝 贡献

欢迎提交 Issue 与 Pull Request。

---

## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

**Made with ❤️ by AutoMap-Pro Team**

文档版本：v2.1（run_automap.sh 为主入口）  
更新日期：2026-03-07
