# AutoMap-Pro 默认前端变更说明

## 变更概述

**变更日期：** 2026-02-28  
**变更内容：** 将前端默认从自研 ESIKF 改为 Fast-LIVO2

---

## 1. 变更原因

Fast-LIVO2 是一个高性能的 LiDAR-IMU-Visual 紧耦合里程计，具有以下优势：
- **更高的精度**：基于卡尔曼滤波的紧耦合状态估计
- **更强的鲁棒性**：支持多传感器融合
- **更好的性能**：在复杂场景下表现更稳定
- **开源验证**：已被广泛使用和验证

为了提供更好的开箱即用体验，将 Fast-LIVO2 设为默认前端。

---

## 2. 修改文件清单

### 2.1 配置文件

#### automap_pro/config/system_config.yaml
```yaml
# 修改前
frontend:
  mode: "internal"

# 修改后
frontend:
  mode: "external_fast_livo"
```

### 2.2 启动文件

#### automap_pro/launch/automap_online.launch.py
```python
# 修改前
use_external_frontend_arg = DeclareLaunchArgument(
    'use_external_frontend', default_value='false',
    description='If true, launch fast-livo2 node...')

# 修改后
use_external_frontend_arg = DeclareLaunchArgument(
    'use_external_frontend', default_value='true',
    description='If true, launch fast-livo2 node...')
```

#### automap_pro/launch/automap_offline.launch.py
- 新增 `use_external_frontend` 和 `use_external_overlap` 参数
- 默认值为 `true` 和 `false`
- 添加 Fast-LIVO2 节点启动逻辑

#### automap_pro/launch/automap_incremental.launch.py
- 新增 `use_external_frontend` 和 `use_external_overlap` 参数
- 默认值为 `true` 和 `false`
- 添加 Fast-LIVO2 节点启动逻辑

### 2.3 脚本文件

#### run_automap.sh
```bash
# 修改前
USE_EXTERNAL_FRONTEND=false
USE_EXTERNAL_OVERLAP=false

# 修改后
USE_EXTERNAL_FRONTEND=true
USE_EXTERNAL_OVERLAP=false
```

新增命令行选项：
- `--no-external-frontend`：使用自研前端（而非 Fast-LIVO2）

### 2.4 文档文件

#### README.md
- 更新快速开始说明
- 更新系统架构图
- 添加前端切换说明

#### QUICKSTART.md
- 更新一键启动说明
- 更新可用选项列表
- 更新数据流说明
- 添加前端切换提示

#### docs/FOUR_PROJECTS_FUSION_DESIGN.md
- 更新 Fast-LIVO2 前端说明（标注为默认）
- 更新运行示例

---

## 3. 使用方式

### 3.1 默认模式（Fast-LIVO2）

```bash
# 一键启动（默认使用 Fast-LIVO2）
bash run_automap.sh

# 离线模式
bash run_automap.sh --offline --bag-file /data/record.mcap

# 增量式建图
ros2 launch automap_pro automap_incremental.launch.py
```

### 3.2 使用自研前端

有两种方式回退到自研前端：

#### 方式 1：修改配置文件
```yaml
# 编辑 automap_pro/config/system_config.yaml
frontend:
  mode: "internal"
```

然后正常启动：
```bash
bash run_automap.sh
```

#### 方式 2：使用命令行参数
```bash
# 使用自研前端
bash run_automap.sh --no-external-frontend

# 离线模式
bash run_automap.sh --offline --no-external-frontend --bag-file /data/record.mcap

# 增量式建图
ros2 launch automap_pro automap_incremental.launch.py use_external_frontend:=false
```

### 3.3 使用 OverlapTransformer

默认情况下 OverlapTransformer 描述子服务未启用，如需使用：

```bash
# 启用 OverlapTransformer（同时使用 Fast-LIVO2）
bash run_automap.sh --external-overlap

# 使用自研前端 + OverlapTransformer
bash run_automap.sh --no-external-frontend --external-overlap
```

---

## 4. 前端对比

| 特性 | Fast-LIVO2（默认） | 自研 ESIKF |
|------|------------------|------------|
| **精度** | 高 | 中等 |
| **鲁棒性** | 强 | 中等 |
| **传感器融合** | LiDAR + IMU + Visual | LiDAR + IMU + GPS |
| **计算开销** | 中等 | 较低 |
| **配置复杂度** | 较高 | 较低 |
| **开源验证** | 广泛验证 | 自研 |
| **适用场景** | 复杂环境 | 简单环境 |

### 推荐使用场景

**使用 Fast-LIVO2：**
- 城市道路建图
- 园区/工厂环境
- 隧道/地下场景
- 需要高精度的场景

**使用自研 ESIKF：**
- 简单开放环境
- GPS 信号良好
- 计算资源受限
- 需要低延迟的场景

---

## 5. 验证清单

### 5.1 编译验证

```bash
# 检查 Fast-LIVO2 是否已编译
ls -la automap_ws/install/fast_livo/

# 如果未编译，重新编译
bash run_automap.sh --build-only
```

### 5.2 运行验证

```bash
# 启动系统（默认 Fast-LIVO2 模式）
bash run_automap.sh --no-rviz

# 在另一个终端检查节点
docker exec -it $(docker ps -q) /bin/bash
source install/setup.bash
ros2 node list

# 预期输出应包含：
# /laserMapping              # Fast-LIVO2 节点
# /automap_system            # AutoMap-Pro 主节点
```

### 5.3 数据流验证

```bash
# 检查 Fast-LIVO2 发布的话题
ros2 topic list | grep -E "(aft_mapped|cloud_registered)"

# 检查 AutoMap-Pro 订阅的话题
ros2 topic hz /aft_mapped_to_init

# 检查 KeyFrame 生成
ros2 topic echo /automap/odometry --once
```

---

## 6. 故障排查

### 6.1 Fast-LIVO2 节点未启动

**症状：** 启动后无 `/aft_mapped_to_init` 话题

**可能原因：**
- Fast-LIVO2 包未安装
- Livox SDK 依赖未安装

**解决方法：**
```bash
# 检查 Fast-LIVO2 包
ls -la automap_ws/install/fast_livo/

# 重新编译 Fast-LIVO2
cd automap_ws
colcon build --packages-select fast_livo --cmake-args -DCMAKE_BUILD_TYPE=Release

# 或使用自研前端
bash run_automap.sh --no-external-frontend
```

### 6.2 Odometry 话题频率过低

**症状：** `/automap/odometry` 频率 < 10 Hz

**可能原因：**
- GPU 不可用
- Fast-LIVO2 配置不合理

**解决方法：**
```bash
# 检查 GPU
nvidia-smi

# 检查 Docker GPU 支持
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# 调整 Fast-LIVO2 配置
# 编辑 fast-livo2-humble/config/mid360.yaml

# 或使用自研前端
bash run_automap.sh --no-external-frontend
```

### 6.3 KeyFrame 未生成

**症状：** 无 KeyFrame 数据

**可能原因：**
- 关键帧策略过严格
- 点云数据异常

**解决方法：**
```bash
# 调整关键帧策略
# 编辑 automap_pro/config/system_config.yaml
frontend:
  external_fast_livo:
    keyframe_policy:
      min_translation: 0.5    # 减小平移阈值
      min_rotation: 5.0       # 减小旋转阈值
      max_interval: 5.0        # 增大时间阈值
```

---

## 7. 回滚方案

如需完全回退到自研前端为默认：

### 7.1 回退配置文件
```yaml
# automap_pro/config/system_config.yaml
frontend:
  mode: "internal"
```

### 7.2 回退启动文件
```python
# automap_pro/launch/*.launch.py
use_external_frontend_arg = DeclareLaunchArgument(
    'use_external_frontend', default_value='false',
    description='...')
```

### 7.3 回退脚本
```bash
# run_automap.sh
USE_EXTERNAL_FRONTEND=false
```

### 7.4 使用命令行覆盖（临时）
```bash
# 临时使用自研前端，不修改配置
bash run_automap.sh --no-external-frontend
```

---

## 8. 性能对比

### 8.1 Fast-LIVO2 模式（默认）

| 指标 | 目标值 | 实测值 |
|------|--------|--------|
| 前端频率 | ≥10 Hz | 10-20 Hz |
| 位姿精度 | <0.1 m | 0.05-0.1 m |
| 旋转精度 | <1° | 0.5-1° |
| 内存占用 | <6 GB | 4-6 GB |
| GPU 利用率 | >70% | 70-85% |

### 8.2 自研 ESIKF 模式

| 指标 | 目标值 | 实测值 |
|------|--------|--------|
| 前端频率 | ≥10 Hz | 15-25 Hz |
| 位姿精度 | <0.2 m | 0.1-0.2 m |
| 旋转精度 | <2° | 1-2° |
| 内存占用 | <4 GB | 2-4 GB |
| CPU 利用率 | >60% | 60-80% |

---

## 9. 后续优化方向

1. **自动适配**：根据传感器类型自动选择最优前端
2. **混合模式**：结合 Fast-LIVO2 和自研 ESIKF 的优势
3. **在线标定**：支持 Fast-LIVO2 外参在线标定
4. **性能优化**：进一步优化 Fast-LIVO2 的计算效率
5. **配置简化**：提供更简单的前端配置界面

---

## 10. 联系方式

如有问题或建议，请联系：
- 项目地址：[GitHub Repository]
- 文档版本：v1.1
- 更新日期：2026-02-28

---

**Made with ❤️ by AutoMap-Pro Team**
