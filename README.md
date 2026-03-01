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
- 🐳 **Docker 支持**：一键编译和运行，开箱即用

---

## 🚀 快速开始

### 前置要求

- Ubuntu 20.04 / 22.04
- Docker 20.10+
- NVIDIA GPU + Driver 470+
- NVIDIA Container Runtime

### 一键启动（在线模式）

```bash
# 克隆项目（如需要）
git clone <repository-url> mapping
cd mapping

# 一键启动（首次运行会自动构建镜像和编译项目）
# 默认使用 Fast-LIVO2 作为前端
bash run_automap.sh
```

**注意：** 默认配置使用 Fast-LIVO2 作为前端。如需使用自研前端，请：
1. 编辑 `automap_pro/config/system_config.yaml`，将 `frontend.mode` 改为 `"internal"`
2. 或使用 `--no-external-frontend` 参数启动

### 离线模式

```bash
# 离线回放 rosbag
bash run_automap.sh --offline --bag-file /data/record.mcap
```

### 其他常用命令

```bash
# 仅编译项目
bash run_automap.sh --build-only

# 仅运行（跳过编译）
bash run_automap.sh --run-only

# 不启动可视化
bash run_automap.sh --no-rviz

# 清理后重新编译
bash run_automap.sh --clean

# 查看系统状态
bash scripts/status.sh
```

---

## 📚 文档

| 文档 | 说明 |
|------|------|
| [快速启动指南](QUICKSTART.md) | 详细的安装、配置和使用说明 |
| [架构文档](高精度高性能自动化点云建图系统架构文档.md) | 系统架构、模块设计和数据流 |

---

## 🏗️ 系统架构

```
传感器数据 (LiDAR/IMU/GPS)
         ↓
   前端里程计 (Fast-LIVO2 默认)
         ↓
   GPS 融合模块
         ↓
   增量子地图管理 (MS-Mapping)
         ↓
   回环检测 (OverlapTransformer + TEASER++)
         ↓
   全局优化 (HBA)
         ↓
   地图输出
```

**前端可选：**
- Fast-LIVO2（默认）：高性能 LiDAR-IMU-Visual 紧耦合里程计
- 自研 ESIKF：可配置为 `frontend.mode: "internal"`

### 核心模块

| 模块 | 技术选型 | 说明 |
|------|---------|------|
| 前端里程计 | Fast-LIVO2 | LiDAR-IMU-(Visual) 紧耦合状态估计 |
| GPS 融合 | 自适应因子图 | 弱/强 GPS 信号自适应融合 |
| 增量子地图 | MS-Mapping | 多会话增量建图、子地图管理 |
| 回环粗匹配 | OverlapTransformer | 基于深度学习的位置识别 |
| 回环精匹配 | TEASER++ | 鲁棒点云配准 |
| 全局优化 | HBA | 分层位姿图优化 + BA |

---

## 📁 项目结构

```
mapping/
├── run_automap.sh              # 🚀 一键编译和运行脚本
├── automap_pro/                # AutoMap-Pro 源码
│   ├── src/                    # 源代码
│   ├── include/                # 头文件
│   ├── launch/                 # 启动文件
│   └── config/                 # 配置文件
├── docker/                     # Docker 镜像
│   ├── dockerfile             # 镜像定义
│   └── deps/                  # 预编译依赖
├── scripts/                    # 辅助脚本
│   ├── build_in_container.sh  # 容器内编译
│   ├── clean.sh               # 清理工具
│   └── status.sh              # 状态检查
├── fast-livo2-humble/          # Fast-LIVO2
├── OverlapTransformer-master/  # OverlapTransformer
├── TEASER-plusplus-master/     # TEASER++
└── HBA-main/                   # HBA
```

---

## 🛠️ 系统要求

| 组件 | 最低配置 | 推荐配置 |
|------|---------|---------|
| CPU | x86_64，4核心 | x86_64，8核心以上 |
| 内存 | 16GB | 32GB 以上 |
| GPU | NVIDIA RTX 3060 | NVIDIA RTX 4060 或更高 |
| 存储 | 50GB 可用空间 | 100GB 可用空间 |
| 系统 | Ubuntu 20.04 / 22.04 | Ubuntu 22.04 |

---

## 📊 性能指标

| 指标 | 目标值 |
|------|--------|
| 前端频率 | ≥10 Hz |
| 回环检测延迟 | <1 s |
| 全局优化时间 | <5 s |
| 全局一致性误差 | <0.3% |
| 内存占用 | <8 GB |

---

## 🔧 常用命令

### 编译相关

```bash
# 编译项目
bash scripts/build_in_container.sh

# 清理后重新编译
bash scripts/build_in_container.sh --clean

# 仅编译指定包
bash scripts/build_in_container.sh --package automap_pro
```

### 清理相关

```bash
# 清理工作空间
bash scripts/clean.sh --workspace

# 清理 Docker 资源
bash scripts/clean.sh --docker

# 清理所有
bash scripts/clean.sh --all
```

### 状态检查

```bash
# 查看系统状态
bash scripts/status.sh
```

### ROS2 服务

```bash
# 保存地图
ros2 service call /automap/save_map automap_pro/srv/SaveMap \
  "{output_dir: '/data/automap_output', save_pcd: true, save_ply: true, save_las: false, save_trajectory: true}"

# 触发全局优化
ros2 service call /automap/trigger_optimize automap_pro/srv/TriggerOptimize \
  "{full_optimization: true, max_iterations: 100}"

# 查看状态
ros2 service call /automap/get_status automap_pro/srv/GetStatus "{}"
```

---

## ❓ 常见问题

### Q：首次构建 Docker 镜像需要多长时间？

**A：** 首次构建通常需要 30-45 分钟，主要耗时在安装 ROS2、编译依赖库和安装 PyTorch。

### Q：如何加快首次构建速度？

**A：** 使用预下载的依赖库，避免重复下载。将依赖库放置在 `docker/deps/` 目录下。

### Q：离线模式需要准备什么？

**A：** 需要准备 rosbag 数据文件，包含 LiDAR、IMU 和 GPS 数据。将数据文件放置在 `${HOME}/data/` 目录下。

### Q：如何查看建图结果？

**A：** 建图结果保存在 `${HOME}/data/automap_output/` 目录下，包含点云地图、轨迹和子地图。

更多常见问题请查看 [快速启动指南](QUICKSTART.md)。

---

## 🐛 故障排查

### Docker 镜像构建失败

```bash
# 查看详细构建日志
cd docker
docker build --no-cache -t automap-env:humble .
```

### 项目编译失败

```bash
# 清理并重新编译
bash scripts/clean.sh --workspace
bash scripts/build_in_container.sh --clean
```

### GPU 不可用

```bash
# 检查 NVIDIA 驱动
nvidia-smi

# 检查 Docker GPU 支持
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

---

## 📖 详细文档

- 📖 [快速启动指南](QUICKSTART.md) - 详细的安装、配置和使用说明
- 📐 [架构文档](高精度高性能自动化点云建图系统架构文档.md) - 系统架构、模块设计和数据流
- 📦 [Docker 使用说明](docker/DOCKER_USAGE.md) - Docker 环境配置和使用

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 👥 联系方式

- 项目地址：[GitHub Repository]
- 文档版本：v1.0
- 更新日期：2025-02-28

---

## ⭐ Star History

如果这个项目对你有帮助，请给它一个星标！

---

**Made with ❤️ by AutoMap-Pro Team**
