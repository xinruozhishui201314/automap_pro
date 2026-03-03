# AutoMap-Pro 增强日志系统使用指南

## 概述

AutoMap-Pro 现在配备了全面的日志记录系统，可以实时监控建图过程的各个环节，包括：

- **详细的进度追踪**：每个步骤的开始、结束和耗时
- **节点状态监控**：实时检查 ROS2 节点的运行状态
- **话题状态监控**：监控关键话题的数据发布情况
- **系统资源监控**：CPU、内存、GPU 使用率
- **错误和警告收集**：自动汇总所有错误和警告信息

## 快速开始

### 1. 使用增强日志版本运行建图

#### Docker 模式（推荐）

```bash
# 运行建图（自动使用增强日志）
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag
```

#### 本地模式

```bash
# 运行建图（增强日志版）
./run_full_mapping_enhanced.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 2. 查看日志

```bash
# 查看日志摘要
./view_logs.sh

# 查看详细日志
./view_logs.sh -l

# 查看错误信息
./view_logs.sh -e

# 查看警告信息
./view_logs.sh -w

# 查看节点监控
./view_logs.sh -n

# 查看话题监控
./view_logs.sh -t

# 查看进度追踪
./view_logs.sh -p

# 实时跟踪日志
./view_logs.sh -f

# 运行诊断
./view_logs.sh -d
```

## 日志文件结构

```
automap_pro/
├── logs/
│   ├── full_mapping_20250301_123456.log    # 主日志文件
│   ├── run_manifest_20250301_123456.txt     # 运行清单（参数、路径、退出码）
│   ├── launch_20250301_123456.d/            # ROS2 各节点日志（ROS_LOG_DIR）
│   └── monitoring/
│       ├── nodes_20250301_123456.log        # 节点监控日志
│       ├── topics_20250301_123456.log      # 话题监控日志
│       └── progress_20250301_123456.log    # 进度追踪日志
```

## 精准排障：结构化标签与快速定位

为便于分析建图失败原因，脚本在日志中写入以下**固定标签**，可用 `grep` 快速定位：

| 标签 | 含义 | 示例 |
|------|------|------|
| `[CONFIG_RESOLVED]` | 实际使用的配置文件绝对路径 | 排查 "Config not found" 时核对路径是否重复 |
| `[LAUNCH_LOG_DIR]` | 本次 ROS2 节点日志目录 | 查看各进程 stderr（如 fastlivo、ros2 bag） |
| `[LAUNCH_EXIT]` | ros2 launch 退出码 | `code=0` 正常，非 0 表示有进程异常退出 |
| `[DIAG]` | 详细模式下的诊断信息 | 含 `ros2 bag info`、配置文件前 50 行，用于分析 yaml 解析错误 |
| `[RUN_MANIFEST]` | 运行清单文件路径 | 内含 bag、config_abs、output_dir、launch_exit_code |

**推荐排障命令：**

```bash
# 查看退出码与诊断
grep -E '\[LAUNCH_EXIT\]|\[DIAG\]|\[CONFIG_RESOLVED\]' logs/full_mapping_*.log

# 只看错误与进程退出
grep -E '\[ERROR\]|process has died|LAUNCH_EXIT' logs/full_mapping_*.log

# 查看本次运行的参数与路径
cat logs/run_manifest_*.txt
```

**配置文件路径说明：** 当通过环境变量传入**绝对路径**（如 Docker 中 `CONFIG=/workspace/automap_pro/.../system_config_nya02.yaml`）时，脚本会直接使用该路径，不再拼接 `SCRIPT_DIR`，避免出现 `.../workspace/automap_pro//workspace/automap_pro/...` 重复路径导致 "Config not found"。

## 日志级别说明

| 级别 | 颜色 | 说明 |
|------|------|------|
| [INFO] | 绿色 | 一般信息消息 |
| [WARN] | 黄色 | 警告信息（不影响运行） |
| [ERROR] | 红色 | 错误信息（可能导致失败） |
| [DEBUG] | 青色 | 调试信息（仅在 --verbose 模式下） |
| [STEP] | 青色 | 步骤标记 |
| [  →] | 紫色 | 子步骤标记 |
| [✓] | 绿色 | 成功标记 |
| [PROGRESS] | 蓝色 | 进度更新 |

## 建图环节详细说明

### 步骤 1/6: 环境检查

**检查内容**：
- ✓ ROS2 Humble 安装
- ✓ Docker 安装（用于 bag 转换）
- ✓ Python3 安装
- ✓ colcon 安装
- ✓ Bag 文件存在性和大小
- ✓ 配置文件存在性
- ✓ 工作空间状态
- ✓ 输出目录创建
- ✓ 系统资源（CPU、内存、GPU）
- ✓ 磁盘空间

**预期输出**：
```
[STEP 1/6] 环境检查
══════════════════════════════════════════════════
  检查 ROS2 环境
══════════════════════════════════════════════════

[✓] ROS2 Humble 已安装
[✓] Docker 已安装: Docker version 29.2.1, build a5c7197
[✓] Python3 已安装: Python 3.10.12
[✓] colcon 已安装: colcon version 0.10.0

══════════════════════════════════════════════════
  检查输入文件
══════════════════════════════════════════════════

[✓] Bag 文件存在: data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag (9.4G)
  [DEBUG]   文件类型: data
[✓] 配置文件存在: automap_pro/config/system_config_nya02.yaml
[✓] 工作空间存在: /home/wqs/automap_ws
[✓] 输出目录已创建: /data/automap_output/nya_02

[  →] 检查系统资源...
[INFO] CPU 负载: 0.15, 0.10, 0.05
[INFO] 内存使用: 12.3GiB / 31.3GiB (39.4%)
[INFO] GPU 使用: 0%, 2456MiB / 24576MiB

══════════════════════════════════════════════════
环境检查完成
══════════════════════════════════════════════════

[INFO] 耗时: 2秒
```

### 步骤 2/6: 编译项目

**检查内容**：
- 工作空间清理（如果 --clean）
- 工作空间设置（make setup）
- 项目编译（make build-release）
- 编译产物验证

**预期输出**：
```
[STEP 2/6] 编译项目

[  →] 设置工作空间...
[INFO] 执行: make setup
... 编译输出 ...
[✓] 工作空间设置完成

[  →] 编译项目（Release 模式）...
[INFO] 执行: make build-release
... 编译输出 ...
[✓] 项目编译完成 (耗时: 125秒)

[✓] 安装目录已创建: /home/wqs/automap_ws/install/automap_pro

[INFO] 步骤2 总耗时: 127秒
```

### 步骤 3/6: 检查并转换 Bag

**检查内容**：
- Bag 格式检测（ROS1 vs ROS2）
- Bag 转换（如果是 ROS1）
- 转换结果验证

**预期输出**：
```
[STEP 3/6: 检查并转换 Bag]

[  →] 检查 bag 格式...
  [DEBUG]   文件类型: SQLite 3.x database
[✓] 检测到 ROS2 格式，无需转换

[INFO] 步骤3 总耗时: 0秒
```

**如果需要转换（ROS1 → ROS2）**：
```
[INFO] ✓ 检测到 ROS1 格式，需要转换
[  →] 使用 Docker 转换 bag...
[  →] 构建转换镜像...
... Docker 构建输出 ...
[  →] 运行 bag 转换...
... 转换输出 ...
[✓] Bag 转换完成 (耗时: 45秒)
[✓] 转换后大小: 8.7G
```

### 步骤 4/6: 启动建图

**检查内容**：
- 环境变量加载
- ROS2 节点启动
- ROS2 话题监控
- 建图进度追踪
- 系统资源监控

**实时监控**（后台运行）：
- 每 10 秒检查一次节点状态
- 每 10 秒检查一次话题状态
- 每 10 秒更新一次建图进度

**预期输出**：
```
[STEP 4/6: 启动建图]

[  →] Source ROS2 和工作空间...
[✓] 环境变量已加载

══════════════════════════════════════════════════
  建图配置
══════════════════════════════════════════════════

[INFO] Bag 文件: /workspace/data/nya_02.bag
[INFO] 配置文件: /workspace/automap_pro/automap_pro/config/system_config_nya02.yaml
[INFO] 输出目录: /workspace/output

[  →] 启动建图流程...
[INFO] 执行: ros2 launch automap_pro automap_offline.launch.py
  [DEBUG]   参数: config:=/workspace/automap_pro/automap_pro/config/system_config_nya02.yaml
  [DEBUG]   参数: bag_file:=/workspace/data/nya_02.bag
  [DEBUG]   参数: rate:=1.0
  [DEBUG]   参数: use_rviz:=true
  [DEBUG]   参数: use_external_frontend:=true

[  →] 启动实时监控...
[INFO] 监控进程 PID: 12345

[PROGRESS] 建图开始: 2025-03-01 12:34:56

... 节点和话题监控日志 ...

══════════════════════════════════════════════════
  建图进行中（按 Ctrl+C 停止）
══════════════════════════════════════════════════

... 建图输出 ...

[PROGRESS] 建图结束: 2025-03-01 14:56:23

[  →] 停止监控进程...

... 节点和话题监控日志 ...

[  →] 检查文件系统状态...
[INFO] 磁盘空间使用率: 67%
[INFO] 输出目录大小: 12G

[INFO] 步骤4 总耗时: 8687秒
[✓] 建图完成
```

**节点监控示例**（`nodes_*.log`）：
```
[2025-03-01 12:35:00] ROS2 节点状态检查:
/automap_system
/laserMapping
/rviz
/rosbag_player

[INFO] 关键节点状态:
  [✓] automap_system - 运行中
  [✓] laserMapping - 运行中
  [✓] rviz - 运行中
  [✓] rosbag_player - 运行中
```

**话题监控示例**（`topics_*.log`）：
```
[2025-03-01 12:35:00] ROS2 话题状态检查:
/livox/lidar
/livox/imu
/optimized_pose
/submap_map
/global_map
/odom
/clock

[INFO] 关键话题状态:
  [✓] /livox/lidar
      Subscription count: 2
      Publisher count: 1
  [✓] /livox/imu
      Subscription count: 2
      Publisher count: 1
  [✓] /optimized_pose
      Subscription count: 1
      Publisher count: 1
  [✓] /submap_map
      Subscription count: 1
      Publisher count: 1
```

**进度追踪示例**（`progress_*.log`）：
```
[PROGRESS] 建图开始: 2025-03-01 12:34:56
[PROGRESS] 已生成 0 个子图文件
[PROGRESS] 已生成 5 个子图文件
[PROGRESS] 已生成 10 个子图文件
...
[PROGRESS] 已生成 150 个子图文件
[PROGRESS] 全局地图大小: 2.3G
...
[PROGRESS] 已生成 320 个子图文件
[PROGRESS] 全局地图大小: 8.7G
[PROGRESS] 建图结束: 2025-03-01 14:56:23
```

### 步骤 5/6: 保存地图

**检查内容**：
- ROS2 服务检查
- 保存地图服务调用
- 保存结果验证

**预期输出**：
```
[STEP 5/6: 保存地图]

[  →] 检查 ROS2 服务...
[  →] 检查 ROS2 服务...
[INFO] 发现 5 个服务
[✓] 保存地图服务可用

[  →] 保存地图到: /workspace/output
[INFO] 执行: ros2 service call /automap/save_map
requester: making request: automap_pro.srv.SaveMap_Request()
response:
  automap_pro.srv.SaveMap_Response(success=True, message='Map saved successfully')
[✓] 地图保存请求已发送 (耗时: 3秒)

[  →] 检查保存结果...
[✓] 地图目录包含 3 个文件
[INFO]   - global_map.pcd (8.7G)
[INFO]   - global_map.ply (8.7G)
[✓] 轨迹目录包含 1 个文件

[INFO] 步骤5 总耗时: 5秒
```

### 步骤 6/6: 显示结果

**检查内容**：
- 地图文件验证
- 轨迹文件验证
- 子图数量统计
- 输出目录内容
- 文件系统状态
- 监控日志摘要

**预期输出**：
```
[STEP 6/6: 显示结果]

══════════════════════════════════════════════════
  建图结果摘要
══════════════════════════════════════════════════

[INFO] 输出目录: /workspace/output

[  →] 检查地图文件...
[✓] 地图文件: global_map.pcd (8.7G)
[✓] 地图文件: global_map.ply (8.7G)

[  →] 检查轨迹文件...
[✓] 轨迹文件: optimized_trajectory_tum.txt (12453 行, 456K)

[  →] 检查子图...
[✓] 子图数量: 320

══════════════════════════════════════════════════
  输出目录内容
══════════════════════════════════════════════════

drwxr-xr-x 2 wqs wqs 4.0K Mar  1 14:56 map
drwxr-xr-x 2 wqs wqs 4.0K Mar  1 14:56 trajectory
drwxr-xr-x 2 wqs wqs  12K Mar  1 14:56 submaps

[  →] 检查文件系统状态...
[INFO] 磁盘空间使用率: 67%
[INFO] 输出目录大小: 12G

══════════════════════════════════════════════════
  节点监控摘要
══════════════════════════════════════════════════

  [✓] automap_system - 运行中
  [✓] laserMapping - 运行中
  [✓] rviz - 运行中
  [✓] rosbag_player - 运行中

══════════════════════════════════════════════════
  话题监控摘要
══════════════════════════════════════════════════

  [✓] /livox/lidar
  [✓] /livox/imu
  [✓] /optimized_pose
  [✓] /submap_map
  [✓] /global_map

══════════════════════════════════════════════════
  进度追踪摘要
══════════════════════════════════════════════════

[PROGRESS] 已生成 300 个子图文件
[PROGRESS] 全局地图大小: 8.2G
[PROGRESS] 已生成 310 个子图文件
[PROGRESS] 全局地图大小: 8.4G
[PROGRESS] 已生成 320 个子图文件
[PROGRESS] 全局地图大小: 8.7G
[PROGRESS] 建图结束: 2025-03-01 14:56:23
```

## 常见问题诊断

### 问题 1: 节点未启动

**症状**：
```
[✗] automap_system - 未运行
[✗] rviz - 未运行
```

**可能原因**：
1. 建图脚本未执行
2. 环境变量未正确加载
3. 节点启动失败

**解决方法**：
1. 检查主日志中的错误信息：`./view_logs.sh -e`
2. 确认工作空间已编译
3. 检查配置文件是否正确

### 问题 2: 话题未发布

**症状**：
```
[✗] /livox/lidar - 未发布
[✗] /livox/imu - 未发布
```

**可能原因**：
1. Bag 文件未开始播放
2. Bag 文件中不包含这些话题
3. Topic remapping 错误

**解决方法**：
1. 检查 bag 文件内容：`ros2 bag info <bag_file>`
2. 检查 rosbag 节点是否运行：`./view_logs.sh -n`
3. 检查配置文件中的话题名称

### 问题 3: 建图进度停滞

**症状**：
```
[PROGRESS] 已生成 150 个子图文件
（长时间无更新）
```

**可能原因**：
1. Bag 播放已结束
2. 程序崩溃或卡死
3. 数据质量问题

**解决方法**：
1. 检查系统资源：`./view_logs.sh -d`
2. 查看完整日志：`./view_logs.sh -l`
3. 检查是否有错误信息：`./view_logs.sh -e`

### 问题 4: 磁盘空间不足

**症状**：
```
[WARN] 磁盘空间使用率过高: 95%
```

**解决方法**：
1. 清理旧的输出目录
2. 清理日志文件（可选）：`./view_logs.sh -c`
3. 使用较小的 bag 文件进行测试

## 高级用法

### 1. 实时监控

```bash
# 在一个终端中运行建图
./run_full_mapping_enhanced.sh -b data/your.bag

# 在另一个终端中实时查看日志
./view_logs.sh -f
```

### 2. 调试模式

```bash
# 启用详细日志输出
./run_full_mapping_enhanced.sh -b data/your.bag --verbose
```

### 3. 自定义监控间隔

编辑 `run_full_mapping_enhanced.sh` 中的 `monitor_mapping_progress` 函数：

```bash
local monitor_interval=10  # 修改为所需间隔（秒）
```

### 4. 导出日志用于分析

```bash
# 打包所有日志文件
cd logs
tar -czf mapping_logs_$(date +%Y%m%d_%H%M%S).tar.gz full_mapping_*.log monitoring/

# 分析日志
grep "ERROR" full_mapping_*.log > errors.txt
grep "WARN" full_mapping_*.log > warnings.txt
```

## 日志分析最佳实践

1. **建图前**：检查系统资源和磁盘空间
2. **建图中**：定期查看进度和节点状态
3. **建图后**：验证输出文件完整性
4. **遇到问题时**：
   - 首先运行诊断：`./view_logs.sh -d`
   - 查看错误信息：`./view_logs.sh -e`
   - 查看相关环节的日志

## 性能优化建议

1. **减少日志输出**：删除 `--verbose` 参数
2. **调整监控间隔**：增加 `monitor_interval` 值
3. **清理旧日志**：定期运行 `./view_logs.sh -c`

## 技术支持

如遇到问题，请提供以下信息：

1. 日志摘要：`./view_logs.sh -s > log_summary.txt`
2. 诊断结果：`./view_logs.sh -d > diagnose.txt`
3. 错误信息：`./view_logs.sh -e > errors.txt`
4. 配置文件
5. 系统环境：OS、ROS2 版本、Docker 版本等
