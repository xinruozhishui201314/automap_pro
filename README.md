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
- 🐳 **Docker 一键**：`automap_start.sh` 一键编译与运行，开箱即用

---

## 🚀 快速开始

### 前置要求

- Ubuntu 20.04 / 22.04
- Docker 20.10+
- NVIDIA GPU + Driver 470+（可选，无 GPU 时以 CPU 模式运行）
- NVIDIA Container Runtime（使用 GPU 时）

### 一键编译并运行（推荐）

```bash
# 克隆项目
git clone <repository-url> automap_pro
cd automap_pro

# 一键编译 + 运行（使用 Docker 镜像 automap-env:humble，默认回放 nya_02_ros2）
bash automap_start.sh
```

首次运行会：检查 Docker 与数据、在容器内编译 `automap_ws`、播放 ROS2 bag、启动建图与 RViz2。

### 常用选项

```bash
# 仅编译
bash automap_start.sh --build

# 仅运行（须先编译）
bash automap_start.sh --run

# 清理后重新编译
bash automap_start.sh --clean --build

# 不启动 RViz2
bash automap_start.sh --no-rviz

# 指定 rosbag 路径
bash automap_start.sh --bag /path/to/your.db3

# Debug 模式编译
bash automap_start.sh --debug --build
```

### 数据与输出路径

| 用途     | 宿主机路径 | 说明 |
|----------|------------|------|
| 默认 bag | `data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/nya_02_ros2.db3` | 须事先准备 |
| 日志     | `logs/automap_YYYYMMDD_HHMMSS/` | 每次运行新建目录 |
| 建图输出 | `output/` | 点云、轨迹等 |

---

## 📚 文档

| 文档 | 说明 |
|------|------|
| [QUICK_START.md](QUICK_START.md) | 快速开始与验证步骤 |
| [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) | 编译/部署/运行说明（重构约定） |
| [docs/README.md](docs/README.md) | 文档索引 |
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
├── automap_start.sh            # 🚀 一键编译 & 运行（主入口）
├── automap_ws/                 # ROS2 工作空间（build/install 在此，Docker 挂载）
│   └── src/                    # 编译时使用的源码（含 automap_pro、fast_livo、hba 等）
├── automap_pro/                # AutoMap-Pro 主包源码（挂载到容器内 automap_ws/src/automap_pro）
│   ├── config/                 # 系统配置（system_config.yaml 为唯一主配置）
│   ├── launch/                 # Launch 文件（automap_composable / offline / online）
│   ├── src/                    # 源码与 modular 子模块
│   │   └── modular/            # fast-livo2-humble、HBA-main、TEASER、overlap_transformer 等
│   └── rviz/
├── data/                       # 数据与 rosbag
│   └── automap_input/
├── docker/                     # Docker 镜像（automap-env:humble）
├── docs/                       # 文档
├── scripts/                    # 辅助脚本（上传 GitHub、验证等）
├── logs/                       # 运行日志（自动创建）
└── output/                     # 建图输出
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

若在宿主机直接编译运行：

```bash
cd automap_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select automap_pro fast_livo hba hba_api --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 播放 bag 与启动建图（需另终端播放 bag）
export AUTOMAP_LOG_DIR=$(pwd)/../logs
ros2 launch automap_pro automap_composable.launch.py config:=/path/to/automap_pro/config/system_config.yaml use_rviz:=true
```

详见 [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md)。

---

## ❓ 常见问题

### 首次运行需要什么？

- 已安装 Docker，并加载或构建镜像 `automap-env:humble`（脚本会检查，无则从 `docker/automap-env_humble.tar` 加载）。
- 默认 bag 存在：`data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/nya_02_ros2.db3`，或使用 `--bag <path>` 指定。

### 如何查看建图结果？

- 建图输出在宿主机 `output/`（容器内 `/workspace/output`）。日志在 `logs/automap_YYYYMMDD_HHMMSS/`。

### Launch 报错 [Errno 21] Is a directory?

- 已修复：`automap_composable.launch.py` 使用非空默认配置路径。若仍遇问题，请确认使用当前 `automap_pro/launch` 与 `automap_pro/rviz/automap.rviz`。

更多见 [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) 与 [QUICK_START.md](QUICK_START.md)。

---

## 📖 详细文档

- [QUICK_START.md](QUICK_START.md) - 快速开始与验证
- [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) - 编译/部署/运行（重构约定）
- [docs/README.md](docs/README.md) - 文档索引
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

文档版本：v2.0（重构后）  
更新日期：2026-03-03
