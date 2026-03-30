# AutoMap-Pro 文档索引

> 以下路径均以**仓库根目录**为基准。主入口脚本：**run_automap.sh**（可选：automap_start.sh）。

---

## 入口与快速开始

| 文档 | 说明 |
|------|------|
| [../README.md](../README.md) | 项目总览、特性、一键开始 |
| [../readme.txt](../readme.txt) | 纯文本速查与近期变更摘要 |
| [../QUICK_START.md](../QUICK_START.md) | 快速开始、常用命令与错误排查速查 |
| [BUILD_DEPLOY_RUN.md](BUILD_DEPLOY_RUN.md) | 编译/部署/运行（run_automap.sh、路径、Docker） |

---

## 配置与流程

| 文档 | 说明 |
|------|------|
| [CONFIG_SUMMARY.md](CONFIG_SUMMARY.md) | 配置文件汇总（system_config、传感器、模块） |
| [MAPPING_WORKFLOW.md](MAPPING_WORKFLOW.md) | 建图流程（数据准备、在线/离线/增量） |
| [MAPPING_PIPELINE_ARCHITECTURE_AND_FAST_LIVO_FIX.md](MAPPING_PIPELINE_ARCHITECTURE_AND_FAST_LIVO_FIX.md) | 建图流水线架构与 Fast-LIVO 相关修复 |

---

## 日志与调试

| 文档 | 说明 |
|------|------|
| [LOGGING_GUIDE.md](LOGGING_GUIDE.md) | 日志配置与查看 |
| [ENHANCED_LOGGING_DELIVERY.md](ENHANCED_LOGGING_DELIVERY.md) | 增强日志交付说明 |
| [ENHANCED_LOGGING_SUMMARY.md](ENHANCED_LOGGING_SUMMARY.md) | 增强日志汇总 |

---

## 版本与迁移

| 文档 | 说明 |
|------|------|
| [ROS1_TO_ROS2_MIGRATION.md](ROS1_TO_ROS2_MIGRATION.md) | ROS1 到 ROS2 迁移 |
| [GIT_LFS_GUIDE.md](GIT_LFS_GUIDE.md) | Git LFS 使用指南 |
| [../README_LFS.md](../README_LFS.md) | LFS 配置与 GitHub 上传（含仅推代码） |

---

## 设计与实现

| 文档 | 说明 |
|------|------|
| [FOUR_PROJECTS_FUSION_DESIGN.md](FOUR_PROJECTS_FUSION_DESIGN.md) | 四项目融合设计 |
| [DEFAULT_FRONTEND_CHANGE.md](DEFAULT_FRONTEND_CHANGE.md) | 默认前端变更说明 |
| [TEASER_BUILD_FIX.md](TEASER_BUILD_FIX.md) | TEASER 构建修复 |
| [TEASERPP_SOURCE_ONLY.md](TEASERPP_SOURCE_ONLY.md) | TEASER++ 仅源码构建 |
| [ENGINEERING_AND_PRODUCT_OPTIMIZATION.md](ENGINEERING_AND_PRODUCT_OPTIMIZATION.md) | 工程与产品优化 |
| [COORDINATE_AND_PROTOCOL_CONTRACTS_20260323.md](COORDINATE_AND_PROTOCOL_CONTRACTS_20260323.md) | 坐标系契约与微服务接口契约更新（2026-03-23） |

---

## 包内文档（automap_pro/docs）

| 文档 | 说明 |
|------|------|
| [../automap_pro/docs/V3_BARRIER_AND_META_CONTRACTS.md](../automap_pro/docs/V3_BARRIER_AND_META_CONTRACTS.md) | V3 事件屏障与 EventMeta 契约 |
| [../automap_pro/docs/v3_stability_baseline.md](../automap_pro/docs/v3_stability_baseline.md) | V3 稳定性基线与自检 |
| [../automap_pro/docs/LOGGING_AND_DIAGNOSIS.md](../automap_pro/docs/LOGGING_AND_DIAGNOSIS.md) | 日志与诊断约定 |
| [../automap_pro/docs/HBA_AND_LOOP_DESIGN.md](../automap_pro/docs/HBA_AND_LOOP_DESIGN.md) | HBA 与回环设计笔记 |
| [../automap_pro/docs/GLOBAL_MAP_MESSY_ANALYSIS.md](../automap_pro/docs/GLOBAL_MAP_MESSY_ANALYSIS.md) | 全局地图杂乱/重影分析 |
| [../automap_pro/docs/TROUBLESHOOTING.md](../automap_pro/docs/TROUBLESHOOTING.md) | 系统启动故障排查（Bag/HBA/级联故障） |
| [../automap_pro/docs/MAP_FRAME_CONFIG.md](../automap_pro/docs/MAP_FRAME_CONFIG.md) | 地图坐标系与配置 |
| [../automap_pro/docs/DEBUG_WITH_GDB.md](../automap_pro/docs/DEBUG_WITH_GDB.md) | GDB 调试与 core 分析 |
| [../automap_pro/docs/CONFIG_PARAM_MAPPING.md](../automap_pro/docs/CONFIG_PARAM_MAPPING.md) | 配置参数映射 |
| [../automap_pro/docs/GLOBAL_MAP_DIAGNOSIS.md](../automap_pro/docs/GLOBAL_MAP_DIAGNOSIS.md) | 全局地图诊断 |
| [../automap_pro/docs/CRASH_ANALYSIS_AND_HARDENING.md](../automap_pro/docs/CRASH_ANALYSIS_AND_HARDENING.md) | 崩溃分析与加固 |

## 其他

| 文档 | 说明 |
|------|------|
| [../docker/DOCKER_USAGE.md](../docker/DOCKER_USAGE.md) | Docker 使用说明 |
| [../高精度高性能自动化点云建图系统架构文档.md](../高精度高性能自动化点云建图系统架构文档.md) | 系统架构（仓库根目录） |

---

**文档版本**：v2.4  
**更新日期**：2026-03-30（与 `README.md` 对齐：MapOrchestrator、M2DGR HDOP、`docs/BUILD_DEPLOY_RUN.md` 容器路径）
