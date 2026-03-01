# AutoMap-Pro 快速启动指南

## 📋 目录

- [概述](#概述)
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [详细使用说明](#详细使用说明)
- [脚本说明](#脚本说明)
- [常见问题](#常见问题)
- [故障排查](#故障排查)

---

## 概述

AutoMap-Pro 是一个高精度自动化点云建图系统，支持在线实时建图和离线回放建图两种模式。

本项目提供了一键编译和运行的脚本，大大简化了使用流程。

**主要特性：**
- ✅ 一键编译和运行
- ✅ 支持在线/离线模式
- ✅ 自动管理 Docker 镜像
- ✅ 自动配置工作空间
- ✅ 支持增量式建图
- ✅ 高性能实时建图（≥10Hz）

---

## 系统要求

### 硬件要求

| 组件 | 最低配置 | 推荐配置 |
|------|---------|---------|
| CPU | x86_64，4核心 | x86_64，8核心以上 |
| 内存 | 16GB | 32GB 以上 |
| GPU | NVIDIA RTX 3060 | NVIDIA RTX 4060 或更高 |
| 存储 | 50GB 可用空间 | 100GB 可用空间 |
| 网络 | - | 用于下载依赖和更新 |

### 软件要求

| 组件 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 20.04 / 22.04 |
| Docker | 20.10+ |
| NVIDIA Docker Runtime | 2.0+ |
| NVIDIA 驱动 | 470+ |

### 检查系统环境

运行以下命令检查系统是否满足要求：

```bash
# 检查 Docker
docker --version

# 检查 NVIDIA 驱动
nvidia-smi

# 检查系统资源
free -h  # 内存
df -h    # 存储
```

---

## 快速开始

### 1. 克隆项目（如需要）

```bash
cd ~/Documents/github
git clone <repository-url> mapping
cd mapping
```

### 2. 一键启动（在线模式）

```bash
bash run_automap.sh
```

**首次运行会自动执行以下步骤：**
1. 检查 Docker 镜像，不存在则构建（30-45 分钟）
2. 准备工作空间
3. 编译项目（5-10 分钟）
4. 启动在线建图系统

**注意：** 默认配置使用 Fast-LIVO2 作为前端。如需使用自研前端，请：
1. 编辑 `automap_pro/config/system_config.yaml`，将 `frontend.mode` 改为 `"internal"`
2. 或使用 `--no-external-frontend` 参数启动

### 3. 离线模式启动

```bash
bash run_automap.sh --offline --bag-file /data/record.mcap
```

### 4. 查看系统状态

```bash
bash scripts/status.sh
```

---

## 详细使用说明

### 主脚本：run_automap.sh

#### 基本语法

```bash
bash run_automap.sh [选项]
```

#### 可用选项

| 选项 | 说明 | 示例 |
|------|------|------|
| `--online` | 在线模式（实时建图，默认） | `bash run_automap.sh --online` |
| `--offline` | 离线模式（回放 rosbag） | `bash run_automap.sh --offline` |
| `--bag-file <path>` | 指定 rosbag 文件路径 | `--bag-file /data/record.mcap` |
| `--build-only` | 仅编译不运行 | `bash run_automap.sh --build-only` |
| `--run-only` | 仅运行不编译（假设已编译） | `bash run_automap.sh --run-only` |
| `--no-rviz` | 不启动 RViz 可视化 | `bash run_automap.sh --no-rviz` |
| `--no-external-frontend` | 使用自研前端（默认使用 Fast-LIVO2） | `bash run_automap.sh --no-external-frontend` |
| `--external-overlap` | 使用 OverlapTransformer 服务 | `bash run_automap.sh --external-overlap` |
| `--clean` | 清理编译产物后重新编译 | `bash run_automap.sh --clean` |
| `--help` | 显示帮助信息 | `bash run_automap.sh --help` |

#### 使用示例

**示例 1：首次完整流程**

```bash
bash run_automap.sh
```

**示例 2：离线建图**

```bash
bash run_automap.sh --offline --bag-file /data/record.mcap
```

**示例 3：仅编译项目**

```bash
bash run_automap.sh --build-only
```

**示例 4：运行但不启动可视化**

```bash
bash run_automap.sh --run-only --no-rviz
```

**示例 5：清理后重新编译**

```bash
bash run_automap.sh --clean
```

---

## 脚本说明

### 1. run_automap.sh（主脚本）

一键编译和运行的主脚本，集成了以下功能：
- Docker 镜像检查和构建
- 工作空间准备
- 项目编译
- 系统启动

**参数：**
- 模式选择（在线/离线）
- 编译控制（仅编译/仅运行）
- 可视化控制（启动/不启动 RViz）
- 清理选项

### 2. scripts/build_in_container.sh

在 Docker 容器内编译项目的辅助脚本。

**使用方法：**
```bash
bash scripts/build_in_container.sh [选项]
```

**选项：**
- `--clean`：清理编译产物后重新编译
- `--release`：Release 模式编译（默认）
- `--debug`：Debug 模式编译
- `--package <name>`：仅编译指定包

**示例：**
```bash
# Release 模式编译
bash scripts/build_in_container.sh

# Debug 模式编译
bash scripts/build_in_container.sh --debug

# 清理后重新编译
bash scripts/build_in_container.sh --clean

# 仅编译 automap_pro
bash scripts/build_in_container.sh --package automap_pro
```

### 3. scripts/clean.sh

清理编译产物和 Docker 资源。

**使用方法：**
```bash
bash scripts/clean.sh [选项]
```

**选项：**
- `--workspace`：清理工作空间（build、install、log）
- `--docker`：清理 Docker 容器和镜像
- `--all`：清理所有（工作空间 + Docker）

**示例：**
```bash
# 清理工作空间
bash scripts/clean.sh --workspace

# 清理 Docker 资源
bash scripts/clean.sh --docker

# 清理所有
bash scripts/clean.sh --all
```

### 4. scripts/status.sh

检查系统状态和配置。

**使用方法：**
```bash
bash scripts/status.sh
```

**输出信息：**
- Docker 镜像状态
- 工作空间状态
- 编译状态
- 数据目录内容
- GPU 支持
- Docker GPU 支持

---

## 常见问题

### Q1：首次构建 Docker 镜像需要多长时间？

**A：** 首次构建 Docker 镜像通常需要 30-45 分钟，主要耗时在：
- 安装 ROS2 Humble（5-10 分钟）
- 编译 GTSAM、Open3D 等依赖库（10-15 分钟）
- 安装 PyTorch（5-10 分钟）
- 安装其他 Python 库（5-10 分钟）

### Q2：编译项目需要多长时间？

**A：** 编译项目通常需要 5-10 分钟，具体时间取决于：
- CPU 核心数和性能
- 是否增量编译
- 编译模式（Release/Debug）
- TEASER++ 是否需要从源码编译（首次 2-3 分钟）

### Q3：TEASER++ 使用源码还是系统版本？

**A：** 优先使用从 `TEASER-plusplus-master` 源码编译的版本，而非系统安装的版本。这样可以：
- 更好的版本控制（可以追踪和回退到特定提交版本）
- 调试便利（可以直接修改源码进行调试和优化）
- 避免版本冲突（避免系统版本过旧或依赖不兼容）
- 保证环境一致性（确保所有环境使用相同的 TEASER++ 版本）

如果源码不存在，才会回退到系统版本（并给出警告）。

### Q4：如何加快首次构建速度？

**A：** 可以使用预下载的依赖库：
- 将 `docker/deps/` 目录中的预编译依赖库准备好
- Docker 构建时会优先使用本地依赖，避免重复下载

### Q4：离线模式需要准备什么？

**A：** 离线模式需要准备：
- rosbag 数据文件（`.mcap`、`.db3` 或 `.bag` 格式）
- 数据文件需要包含 LiDAR、IMU 和 GPS 数据
- 将数据文件放置在 `${HOME}/data/` 目录下

### Q5：如何查看建图结果？

**A：** 建图结果会保存在 `${HOME}/data/automap_output/` 目录下，包含：
- 全局点云地图（PCD/PLY 格式）
- 优化后轨迹（TUM 格式）
- 子地图集合
- 位姿图（g2o 格式）

可以使用 RViz、CloudCompare 或自定义工具进行可视化。

### Q6：如何保存地图？

**A：** 可以通过 ROS2 服务保存地图：

```bash
# 在容器内执行
ros2 service call /automap/save_map automap_pro/srv/SaveMap \
  "{output_dir: '/data/automap_output', save_pcd: true, save_ply: true, save_las: false, save_trajectory: true}"
```

### Q7：如何触发全局优化？

**A：** 可以通过 ROS2 服务触发优化：

```bash
# 在容器内执行
ros2 service call /automap/trigger_optimize automap_pro/srv/TriggerOptimize \
  "{full_optimization: true, max_iterations: 100}"
```

---

## 故障排查

### 问题 1：Docker 镜像构建失败

**症状：** 运行 `run_automap.sh` 时提示镜像构建失败

**可能原因：**
- 网络问题，导致依赖下载失败
- 磁盘空间不足
- Docker 配置错误

**解决方法：**
```bash
# 检查磁盘空间
df -h

# 清理 Docker 缓存
docker system prune -a

# 查看详细构建日志
cd docker
docker build --no-cache -t automap-env:humble .
```

### 问题 2：项目编译失败

**症状：** 编译过程中报错

**可能原因：**
- 依赖库未正确安装
- 源码不完整
- 编译环境问题

**解决方法：**
```bash
# 清理编译产物
bash scripts/clean.sh --workspace

# 重新编译
bash scripts/build_in_container.sh --clean

# 查看详细编译日志
docker run -it --rm \
    -v ${HOME}/automap_ws:/root/automap_ws \
    automap-env:humble \
    /bin/bash
# 然后在容器内手动编译，查看详细错误信息
```

### 问题 3：GPU 不可用

**症状：** 提示 GPU 不可用或性能很差

**可能原因：**
- NVIDIA 驱动未安装或版本不匹配
- Docker NVIDIA 运行时未配置
- GPU 资源被占用

**解决方法：**
```bash
# 检查 NVIDIA 驱动
nvidia-smi

# 检查 Docker GPU 支持
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# 配置 NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 问题 4：运行时节点崩溃或报错

**症状：** 系统启动后某个节点崩溃或报错

**解决方法：**
```bash
# 查看日志
docker logs <container_id>

# 进入容器调试
docker exec -it <container_id> /bin/bash

# 检查 ROS2 节点状态
ros2 node list
ros2 topic list
ros2 topic echo /topic_name
```

### 问题 5：X11 转发失败（无法启动 RViz）

**症状：** RViz 无法启动或报错

**解决方法：**
```bash
# 检查 DISPLAY 变量
echo $DISPLAY

# 允许 X11 转发
xhost +local:docker

# 如果仍然失败，尝试使用远程桌面或 VNC
```

---

## 目录结构

```
mapping/
├── run_automap.sh              # 主脚本：一键编译和运行
├── automap_pro/                # AutoMap-Pro 源码
│   ├── src/                    # 源代码
│   ├── include/                # 头文件
│   ├── launch/                 # 启动文件
│   ├── config/                 # 配置文件
│   └── CMakeLists.txt         # 编译配置
├── docker/                     # Docker 相关
│   ├── dockerfile             # Docker 镜像定义
│   ├── docker_build.sh        # Docker 构建脚本
│   ├── deps/                  # 预编译依赖
│   └── scripts/               # Docker 辅助脚本
└── scripts/                    # 项目辅助脚本
    ├── build_in_container.sh  # 容器内编译脚本
    ├── clean.sh               # 清理脚本
    └── status.sh              # 状态检查脚本
```

---

## 数据流说明

### 在线模式数据流

```
传感器数据 (LiDAR/IMU/GPS)
         ↓
   前端里程计 (Fast-LIVO2 默认，可选自研 ESIKF)
         ↓
   GPS 融合模块
         ↓
   增量子地图管理 (MS-Mapping)
         ↓
   回环检测 (可选 OverlapTransformer + TEASER++)
         ↓
   全局优化 (HBA)
         ↓
   地图输出
```

### 离线模式数据流

```
rosbag 回放
         ↓
   前端里程计 (Fast-LIVO2 默认，可选自研 ESIKF)
         ↓
   GPS 融合模块
         ↓
   增量子地图管理 (MS-Mapping)
         ↓
   回环检测 (可选 OverlapTransformer + TEASER++)
         ↓
   全局优化 (HBA)
         ↓
   地图输出
```

---

## 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 前端频率 | ≥10 Hz | 实时建图 |
| 回环检测延迟 | <1 s | 及时检测 |
| 全局优化时间 | <5 s | 快速收敛 |
| 全局一致性误差 | <0.3% | 高精度 |
| 内存占用 | <8 GB | 低资源消耗 |

---

## 下一步

- 阅读 [高精度高性能自动化点云建图系统架构文档.md](./高精度高性能自动化点云建图系统架构文档.md) 了解系统架构
- 查看 `automap_pro/config/system_config.yaml` 了解配置选项
- 运行 `bash scripts/status.sh` 检查系统状态

---

## 联系方式

如有问题或建议，请联系：
- 项目地址：[GitHub Repository]
- 文档版本：v1.0
- 更新日期：2025-02-28

---

## 附录

### A. ROS2 常用命令

```bash
# 查看节点列表
ros2 node list

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /topic_name

# 查看话题数据
ros2 topic echo /topic_name

# 查看服务列表
ros2 service list

# 查看节点关系图
ros2 run rqt_graph rqt_graph
```

### B. Docker 常用命令

```bash
# 查看运行的容器
docker ps

# 查看所有容器
docker ps -a

# 查看镜像
docker images

# 进入容器
docker exec -it <container_id> /bin/bash

# 查看容器日志
docker logs <container_id>

# 停止容器
docker stop <container_id>

# 删除容器
docker rm <container_id>

# 删除镜像
docker rmi <image_name>
```

---

**祝使用愉快！**
