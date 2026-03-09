# Docker 路径问题快速修复

## 问题描述

运行 `docker/mapping.sh -v` 时出现错误：
```
[ERROR] Bag 文件不存在: /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

**原因**：bag 文件在 `data/automap_input/` 目录下，但默认配置使用的路径不正确。

---

## 快速修复

### 方法1: 使用 Makefile（最简单）⭐⭐⭐⭐

```bash
cd /home/wqs/Documents/github/automap_pro

# Docker 一键建图（会自动处理路径）
make docker-mapping
```

### 方法2: 更新脚本默认配置

编辑 `run_full_mapping_docker.sh`，找到默认 BAG_FILE 配置：

```bash
# 找到这一行（约第 20 行）
BAG_FILE="${BAG_FILE:-data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"

# 改为绝对路径
BAG_FILE="${BAG_FILE:-@data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag}"
```

然后运行：
```bash
./run_full_mapping_docker.sh
```

### 方法3: 手动指定 bag 文件

```bash
cd /home/wqs/Documents/github/automap_pro

# 明确指定 bag 文件
./run_full_mapping_docker.sh -b @data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 方法4: 进入容器调试

```bash
cd /home/wqs/Documents/github/automap_pro

# 启动容器 shell
./run_full_mapping_docker.sh --shell

# 在容器内查看文件
ls -lh /workspace/data/
ls -lh /workspace/data/automap_input/

# 检查 bag 文件
ls -lh /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

---

## 详细步骤

### 步骤1: 检查本地文件

```bash
cd /home/wqs/Documents/github/automap_pro

# 检查 bag 文件是否存在
ls -lh data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 检查 data 目录结构
ls -lh data/automap_input/
```

### 步骤2: 启动容器查看

```bash
# 启动容器（不自动运行建图）
docker run --rm -it \
    --name automap_debug \
    -v $(pwd):/workspace/automap_pro \
    automap_pro:latest \
    bash
```

在容器内：
```bash
# 查看挂载的目录
ls -lh /workspace/

# 查看数据目录
ls -lh /workspace/automap_pro/data/

# 查找 bag 文件
find /workspace -name "*.bag" -o -name "*.db3"
```

### 步骤3: 运行建图（使用正确路径）

根据步骤2找到的实际路径，运行建图：

```bash
cd /home/wqs/Documents/github/automap_pro

# 如果 bag 文件在 data/automap_input/nya_02_slam_imu_to_lidar/
./run_full_mapping_docker.sh -b @data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 如果 bag 文件在其他位置
./run_full_mapping_docker.sh -b @/actual/path/to/bag.bag
```

---

## 路径说明

### Docker 挂载点

| 本地路径 | 容器路径 | 说明 |
|---------|----------|------|
| `$(pwd)` | `/workspace/automap_pro` | 项目目录 |
| `$(pwd)/data` | `/workspace/data` | 数据目录 |

### Bag 文件查找

```bash
# 方法1: 使用绝对路径 + @ 前缀
./run_full_mapping_docker.sh -b @data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 方法2: 使用完整绝对路径
./run_full_mapping_docker.sh -b /home/wqs/Documents/github/automap_pro/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag

# 方法3: 进入容器后使用相对路径
docker exec -it automap_debug bash
cd /workspace/automap_pro/docker
./mapping.sh -b /workspace/data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

---

## 验证修复

### 检查 bag 文件是否被找到

```bash
# 运行建图（应该显示找到 bag 文件）
cd /home/wqs/Documents/github/automap_pro
./run_full_mapping_docker.sh

# 应该看到类似的输出：
# [INFO] ✓ 检测到 ROS1 格式，需要转换
# [INFO] ✓ Bag 文件存在: .../nya_02.bag (9.4G)
```

### 如果仍然报错

```bash
# 1. 检查本地文件
ls -lh data/automap_input/nya_02_slam_imu_to_lidar/

# 2. 启动容器调试
./run_full_mapping_docker.sh --shell

# 3. 在容器内查看文件
ls -lh /workspace/data/
ls -lh /workspace/data/automap_input/

# 4. 找到实际的 bag 文件路径
find /workspace -name "*.bag"

# 5. 使用找到的路径运行建图
# 退出容器后
exit

# 6. 使用实际路径
./run_full_mapping_docker.sh -b @/actual/path/to/bag.bag
```

---

## 路径规则

### 本地路径

- ✅ 相对路径：`data/automap_input/nya_02.bag`
- ✅ 绝对路径：`/home/wqs/Documents/github/automap_pro/data/automap_input/nya_02.bag`
- ✅ 带@前缀：`@data/automap_input/nya_02.bag`

### Docker 容器内路径

- ✅ 项目目录：`/workspace/automap_pro`
- ✅ 数据目录：`/workspace/data`
- ✅ Bag 文件：`/workspace/data/automap_input/nya_02.bag`

---

## 总结

### 推荐解决方案（按优先级）

| 方法 | 复杂度 | 推荐度 |
|------|--------|--------|
| **使用 Makefile** | ⭐ | ⭐⭐⭐⭐ |
| **使用 @ 前缀** | ⭐ | ⭐⭐⭐⭐ |
| **使用绝对路径** | ⭐ | ⭐⭐⭐ |
| **进入容器调试** | ⭐⭐ | ⭐⭐ |

### 最简单的命令

```bash
cd /home/wqs/Documents/github/automap_pro

# 方法1: 使用 Makefile（推荐）
make docker-mapping

# 方法2: 使用 @ 前缀
./run_full_mapping_docker.sh -b @data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

---

**维护者**: Automap Pro Team
**最后更新**: 2026-03-01
**版本**: 1.0
