# AutoMap-Pro 增强日志系统 - 快速开始

## 概述

AutoMap-Pro 现已配备完整的日志记录和监控系统，可以实时追踪建图的每个环节。

## 快速开始

### 方式 1：使用 Docker（推荐）

```bash
# 运行建图（自动使用增强日志）
./run_full_mapping_docker.sh -b @data/automap_input/nya_02.bag
```

### 方式 2：本地运行

```bash
# 运行建图（增强日志版）
./run_full_mapping_enhanced.sh -b data/automap_input/nya_02_slam_imu_to_lidar/nya_02.bag
```

### 方式 3：交互式菜单

```bash
# 启动快速启动菜单
./quick_start_enhanced.sh
```

## 查看日志

```bash
# 查看日志摘要
./view_logs.sh

# 查看错误信息
./view_logs.sh -e

# 查看节点监控
./view_logs.sh -n

# 查看话题监控
./view_logs.sh -t

# 查看进度追踪
./view_logs.sh -p

# 运行诊断
./view_logs.sh -d

# 实时跟踪日志
./view_logs.sh -f
```

## 日志内容

### 6 个建图环节

| 步骤 | 说明 |
|------|------|
| 步骤 1/6 | 环境检查（ROS2、Docker、文件、资源） |
| 步骤 2/6 | 编译项目（make setup、make build-release） |
| 步骤 3/6 | 转换 Bag（ROS1 → ROS2） |
| 步骤 4/6 | 启动建图（ros2 launch、监控） |
| 步骤 5/6 | 保存地图（服务调用） |
| 步骤 6/6 | 显示结果（验证、统计） |

### 实时监控

- **节点监控**：每 10 秒检查 ROS2 节点状态
- **话题监控**：每 10 秒检查关键话题发布情况
- **进度追踪**：每 10 秒更新子图数量和地图大小
- **系统资源**：CPU、内存、GPU 使用率

## 日志文件

```
logs/
├── full_mapping_YYYYMMDD_HHMMSS.log      # 主日志
└── monitoring/
    ├── nodes_YYYYMMDD_HHMMSS.log         # 节点监控
    ├── topics_YYYYMMDD_HHMMSS.log        # 话题监控
    └── progress_YYYYMMDD_HHMMSS.log      # 进度追踪
```

## 日志级别

| 级别 | 颜色 | 说明 |
|------|------|------|
| [INFO] | 绿色 | 一般信息 |
| [WARN] | 黄色 | 警告 |
| [ERROR] | 红色 | 错误 |
| [DEBUG] | 青色 | 调试信息（--verbose 模式） |
| [STEP] | 青色 | 步骤标记 |
| [  →] | 紫色 | 子步骤 |
| [✓] | 绿色 | 成功 |
| [PROGRESS] | 蓝色 | 进度更新 |

## 故障排查

### 快速诊断

```bash
# 运行诊断
./view_logs.sh -d
```

诊断内容包括：
- 日志文件检查
- 最新日志分析
- 节点状态检查
- 话题状态检查
- 输出文件检查
- 诊断建议

### 常见问题

**1. 节点未启动**
```bash
# 查看节点状态
./view_logs.sh -n

# 查看错误信息
./view_logs.sh -e
```

**2. 话题未发布**
```bash
# 查看话题状态
./view_logs.sh -t

# 检查 bag 文件
ros2 bag info <bag_file>
```

**3. 建图进度停滞**
```bash
# 查看进度
./view_logs.sh -p

# 查看系统资源
./view_logs.sh -d

# 查看完整日志
./view_logs.sh -l
```

## 文档

- **详细使用指南**：`docs/LOGGING_GUIDE.md`
- **功能总结文档**：`docs/ENHANCED_LOGGING_SUMMARY.md`

## 测试

```bash
# 运行测试脚本
./test_enhanced_logging.sh
```

## 清理日志

```bash
# 清空所有日志
./view_logs.sh -c
```

## 性能优化

### 减少日志开销

```bash
# 使用普通版（无监控）
./run_full_mapping.sh

# 或禁用监控
./run_full_mapping_enhanced.sh -b <bag> --no-monitor
```

### 调整监控间隔

编辑 `run_full_mapping_enhanced.sh` 中的 `monitor_mapping_progress` 函数：

```bash
# 原值：10 秒
local monitor_interval=10

# 改为：30 秒（减少开销）
local monitor_interval=30
```

## 技术支持

遇到问题时，请提供：

1. 日志摘要：`./view_logs.sh -s > log_summary.txt`
2. 诊断结果：`./view_logs.sh -d > diagnose.txt`
3. 错误信息：`./view_logs.sh -e > errors.txt`
4. 系统环境：OS、ROS2 版本、Docker 版本

## 示例

### 完整工作流程

```bash
# 终端 1：运行建图
./run_full_mapping_enhanced.sh -b data/.../nya_02.bag

# 终端 2：实时跟踪日志
./view_logs.sh -f

# 终端 3：查看进度
watch -n 5 './view_logs.sh -p'
```

### 建图后验证

```bash
# 运行诊断
./view_logs.sh -d

# 查看摘要
./view_logs.sh -s

# 检查输出
ls -lh /data/automap_output/nya_02/
```

## 文件清单

### 新增文件

| 文件 | 说明 |
|------|------|
| `run_full_mapping_enhanced.sh` | 增强版建图脚本 |
| `view_logs.sh` | 日志查看和诊断工具 |
| `quick_start_enhanced.sh` | 快速启动菜单 |
| `test_enhanced_logging.sh` | 测试脚本 |
| `docs/LOGGING_GUIDE.md` | 详细使用指南 |
| `docs/ENHANCED_LOGGING_SUMMARY.md` | 功能总结文档 |

### 修改文件

| 文件 | 修改 |
|------|------|
| `run_full_mapping_docker.sh` | 使用增强版脚本 |

## 版本

**版本**：v1.0.0
**更新日期**：2025-03-01
