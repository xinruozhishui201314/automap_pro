# AutoMap-Pro Docker 环境使用说明

> **重构后推荐**：在仓库根目录使用 **`bash automap_start.sh`** 一键完成镜像检查、编译与建图运行，无需手动挂载与启动。以下为镜像与手动运行参考。

## 环境概述

- **镜像名称**: `automap-env:humble`（可由 `docker/automap-env_humble.tar` 加载）
- **基础镜像**: `nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04`
- **ROS2 版本**: Humble
- **CUDA 版本**: 11.8
- **用途**: 纯运行/编译环境，源码通过 -v 挂载进容器；**推荐通过 automap_start.sh 使用**

## 目录结构

```
docker/
├── dockerfile                          # Docker 镜像构建文件
├── docker_build.sh                     # 构建和运行脚本
├── scripts/
│   ├── build_ros2_drivers.sh           # ROS2 驱动编译脚本
│   └── entrypoint.sh                   # 容器启动脚本
└── deps/                               # 预编译依赖库
    ├── gtsam/                          # GTSAM 4.2 (预编译)
    ├── Sophus/                         # Sophus (预编译)
    ├── ikd-Tree/                       # ikd-Tree (预编译)
    ├── Livox-SDK2/                     # Livox-SDK2 (预编译)
    ├── livox_ros_driver2/             # ROS2 驱动 (运行时编译)
    ├── TEASER-plusplus/               # TEASER++ (预编译)
    ├── pmc/                           # PMC (预编译)
    └── Open3D/                        # Open3D (预编译)
```

## 构建镜像

```bash
cd /home/wqs/Documents/github/mapping/docker
docker build -t automap-env:humble .
```

构建时间约 15-20 分钟，主要耗时在：
- 安装 ROS2 Humble
- 编译 GTSAM
- 编译 Open3D
- 安装 PyTorch

## 运行容器

### 基础运行（只启动容器）

```bash
docker run -it --rm \
  --gpus all \
  --net=host \
  automap-env:humble
```

### 挂载工作空间并自动编译 livox_ros_driver2

```bash
docker run -it --rm \
  --gpus all \
  --net=host \
  -v /home/wqs/Documents/github/mapping/docker/deps/livox_ros_driver2:/root/automap_ws/src/livox_ros_driver2 \
  -v /home/wqs/Documents/github/mapping/fast-livo2-humble:/root/automap_ws/src/fast-livo2 \
  automap-env:humble
```

### 完整运行（含 X11、设备挂载）

```bash
xhost +local:docker

docker run -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /home/wqs/Documents/github/mapping/docker/deps/livox_ros_driver2:/root/automap_ws/src/livox_ros_driver2 \
  -v /home/wqs/Documents/github/mapping/fast-livo2-humble:/root/automap_ws/src/fast-livo2 \
  -v $HOME/automap_ws:/root/automap_ws:rw \
  -v $HOME/data:/data:rw \
  automap-env:humble
```

## ROS2 驱动编译

### 自动编译（推荐）

容器启动时会自动检测并编译 `livox_ros_driver2`：

```bash
# 启动时自动编译
docker run -it ... automap-env:humble
```

### 手动编译

```bash
# 在容器内执行
/opt/scripts/build_ros2_drivers.sh
```

### 禁用自动编译

```bash
docker run -it ... -e AUTO_BUILD_LIVOX=0 automap-env:humble
```

## 编译 Fast-LIVO2

### 首次编译

```bash
# 1. 确保已挂载 fast-livo2 源码到工作空间
docker run -it ... -v /path/to/fast-livo2:/root/automap_ws/src/fast-livo2 ...

# 2. 在容器内编译
cd /root/automap_ws
colcon build --packages-select fast_livo2

# 3. Source 工作空间
source install/setup.bash

# 4. 运行
ros2 launch fast_livo2 <launch_file>
```

### 清理并重新编译

```bash
rm -rf build install log
colcon build --packages-select fast_livo2
```

## 常用命令

### 容器内常用命令

```bash
# Source ROS2 环境
source /opt/ros/humble/setup.bash

# Source 工作空间
source /root/automap_ws/install/setup.bash

# 检查已编译的包
ros2 pkg list | grep livox
ros2 pkg list | grep fast_livo2

# 查看消息定义
ros2 interface show livox_ros_driver2/msg/CustomMsg

# 检查节点
ros2 node list
ros2 topic list
```

### 容器管理

```bash
# 查看运行中的容器
docker ps

# 查看容器日志
docker logs <container_id>

# 进入运行中的容器
docker exec -it <container_id> bash

# 停止容器
docker stop <container_id>

# 删除容器
docker rm <container_id>

# 删除镜像
docker rmi automap-env:humble
```

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROS_DISTRO` | `humble` | ROS2 发行版 |
| `CUDA_HOME` | `/usr/local/cuda` | CUDA 安装路径 |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS 实现 |
| `AUTO_BUILD_LIVOX` | `1` | 是否自动编译 livox_ros_driver2 |

## 常见问题

### Q1: livox_ros_driver2 编译失败

**原因**: Livox-SDK2 未安装或源码路径错误

**解决**:
```bash
# 检查 Livox-SDK2 是否安装
ls -l /usr/local/lib/liblivox_lidar_sdk_shared.so

# 检查源码路径
ls -l /root/automap_ws/src/livox_ros_driver2

# 手动重新编译
rm -rf /root/automap_ws/build/livox_ros_driver2 /root/automap_ws/install/livox_ros_driver2
/opt/scripts/build_ros2_drivers.sh
```

### Q2: 找不到 livox_ros_driver2 的消息类型

**原因**: 未 source 工作空间

**解决**:
```bash
source /root/automap_ws/install/setup.bash
```

### Q3: GPU 不可用

**原因**: 未启用 GPU 或驱动不匹配

**解决**:
```bash
# 启动时添加 --gpus all
docker run -it --gpus all ...

# 检查容器内 GPU
nvidia-smi
```

### Q4: 编译速度慢

**解决**:
```bash
# 使用多核编译
colcon build --packages-select fast_livo2 --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=$(nproc)

# 或使用 ccache
export CCACHE_DIR=/root/.ccache
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

## 性能优化

### CycloneDDS 配置（大数据量点云优化）

镜像中已配置 `/etc/cyclonedds/cyclonedds.xml`，优化参数：
- `MaxMessageSize`: 65500B
- `SocketReceiveBufferSize`: 10MB
- `WhcHigh`: 500kB

### Docker 资源限制

```bash
# 限制 CPU 和内存
docker run -it --cpus="4" --memory="8g" ... automap-env:humble

# 设置共享内存（点云处理需要）
docker run -it --shm-size=8g ... automap-env:humble
```

## 故障排查

### 查看编译日志

```bash
# livox_ros_driver2 编译日志
cat /root/automap_ws/log/latest_build/livox_ros_driver2/stdout_stderr.log

# fast_livo2 编译日志
cat /root/automap_ws/log/latest_build/fast_livo2/stdout_stderr.log
```

### 检查依赖

```bash
# 检查 ROS2 依赖
ros2 doctor --report

# 检查依赖关系
ros2 pkg xml <package_name>
```

## 更新与维护

### 更新依赖

```bash
# 更新 deps 目录内容后，重新构建镜像
docker build -t automap-env:humble .

# 或使用 BuildKit 加速
DOCKER_BUILDKIT=1 docker build -t automap-env:humble .
```

### 清理未使用的资源

```bash
# 清理悬空镜像
docker image prune

# 清理未使用的卷
docker volume prune

# 清理所有未使用的资源
docker system prune -a
```

## 参考资源

- [ROS2 Humble 官方文档](https://docs.ros.org/en/humble/)
- [Livox SDK 文档](https://github.com/Livox-SDK/Livox-SDK2)
- [Fast-LIVO2 GitHub](https://github.com/hku-mars/fast_livo)
- [CUDA 文档](https://docs.nvidia.com/cuda/)

## 技术支持

如有问题，请检查：
1. Docker 日志: `docker logs <container_id>`
2. 编译日志: `/root/automap_ws/log/latest_build/`
3. 终端输出中的错误信息
