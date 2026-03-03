# AutoMap-Pro 快速开始

> 重构后以 **automap_start.sh** 为唯一推荐入口，Docker 镜像 `automap-env:humble`，工作空间 `automap_ws`。

---

## 1. 前置条件

- **系统**：Ubuntu 20.04 / 22.04
- **Docker**：已安装并运行，可执行 `docker info`
- **数据**：默认使用 `data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/nya_02_ros2.db3`，或通过 `--bag <path>` 指定
- **镜像**：`automap-env:humble`（脚本会尝试从 `docker/automap-env_humble.tar` 加载）

---

## 2. 一键运行

```bash
# 在仓库根目录执行：编译 + 运行（含 RViz2）
bash automap_start.sh
```

首次会进行容器内编译（约数分钟），随后自动播放 bag 并启动建图与 RViz。

---

## 3. 常用命令

| 命令 | 说明 |
|------|------|
| `bash automap_start.sh` | 一键编译并运行 |
| `bash automap_start.sh --build` | 仅编译 |
| `bash automap_start.sh --run` | 仅运行（须已编译） |
| `bash automap_start.sh --clean --build` | 清理后重新编译 |
| `bash automap_start.sh --no-rviz` | 不启动 RViz2 |
| `bash automap_start.sh --bag /path/to/xxx.db3` | 指定 ROS2 bag |
| `bash automap_start.sh --help` | 显示帮助 |

---

## 4. 路径说明

| 用途 | 路径 |
|------|------|
| 默认 bag | `data/automap_input/nya_02_slam_imu_to_lidar/nya_02_ros2/nya_02_ros2.db3` |
| 系统配置 | `automap_ws/src/automap_pro/config/system_config.yaml`（容器内同路径为 `/workspace/automap_ws/src/automap_pro/config/`） |
| 日志 | `logs/automap_YYYYMMDD_HHMMSS/` |
| 建图输出 | `output/` |

---

## 5. 验证

- **编译成功**：终端出现 `BUILD SUCCESS ✓`。
- **运行正常**：无 `[Errno 21] Is a directory`；RViz2 打开；bag 播放且建图节点无异常退出。
- **日志**：`logs/automap_*/` 下可见 spdlog 与 launch 日志。

---

## 6. 故障排查

| 现象 | 处理 |
|------|------|
| 找不到 bag | 检查 `data/automap_input/.../nya_02_ros2.db3` 是否存在，或使用 `--bag <path>` |
| 镜像不存在 | 将 `docker/automap-env_humble.tar` 放在仓库根同级，或先构建镜像 |
| Launch 报错 Is a directory | 确认使用当前 `automap_pro/launch/automap_composable.launch.py`（已修复默认配置路径） |
| 无 GPU | 脚本会检测，无 GPU 时以 CPU 模式运行 |

更多见 [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md)。

---

## 7. 相关文档

- [README.md](README.md) - 项目总览
- [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) - 编译/部署/运行详细说明
- [docs/README.md](docs/README.md) - 文档索引

---

文档版本：v2.0（重构后）  
更新日期：2026-03-03
