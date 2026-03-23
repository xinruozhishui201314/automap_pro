# AutoMap-Pro 工程化与产品化优化建议

> 基于工程化、产品化标准，对当前工程缺口与改进项做系统梳理，便于按优先级落地。

---

## 0. Executive Summary

| 类别 | 现状 | 建议优先级 | 核心动作 |
|------|------|------------|----------|
| **CI/CD** | 无仓库级 CI | P0 | 增加 GitHub Actions：构建、测试、Docker 构建 |
| **测试** | 有单测骨架，未在流水线跑全 | P0 | 打通 colcon test、加冒烟/集成测试 |
| **配置** | 无 schema/启动前校验 | P1 | 配置 schema + 启动前 validate |
| **版本与发布** | 无 CHANGELOG/语义化版本 | P1 | CHANGELOG.md + 版本号与 tag 规范 |
| **文档与社区** | 有设计 doc，缺贡献/安全策略 | P1 | CONTRIBUTING.md、SECURITY.md、术语表 |
| **可观测性** | GetStatus 有，无指标/健康探针 | P1 | ROS2 diagnostic + 可选 Prometheus |
| **安全与合规** | 未显式声明 | P2 | 依赖审计、许可证清单、脱敏说明 |
| **产品体验** | 一键脚本完善，缺 Runbook | P1 | 故障 Runbook、环境检查脚本 |
| **代码质量** | 有 lint 依赖，未强制 | P2 | pre-commit / CI lint、格式化 |
| **部署与交付** | Docker + run_automap，无 K8s/离线包 | P2 | 可选 compose 一键、离线安装包 |

---

## 0.1 2026-03-23 已落地项（增量）

本次已按“最小改动、契约优先”落地以下能力：

- **接口契约 SSoT**：新增 `protocol_contract.h`，统一管理 API 版本、topics、services。
- **启动期 Fail-Fast**：`ConfigManager::load` 增加 `system.api_version` 校验，不兼容时直接阻断启动。
- **位姿坐标系契约**：`SyncedFrameEvent` / `OptimizationResultEvent` / `KeyFrame` / `SubMap` 补充 `PoseFrame` 语义。
- **网关校核**：`MappingModule` / `VisualizationModule` 在接收位姿时按 `PoseFrame` 做坐标系核校与转换，避免双重变换与语义漂移。
- **防错增强**：优化结果入库前增加有限值和合理性检查，异常数据直接拒绝。

详见：`docs/COORDINATE_AND_PROTOCOL_CONTRACTS_20260323.md`

---

## 1. CI/CD 与自动化

### 1.1 现状

- 仓库根目录无 `.github/workflows`，无统一 CI。
- 编译、测试依赖本地或手动在 Docker 内执行。

### 1.2 建议

| 项 | 内容 |
|----|------|
| **GitHub Actions** | 在 `.github/workflows/` 下新增：<br>• `build.yml`：在 Ubuntu 22.04 + ROS2 Humble 下 colcon build（可选 Docker 内），至少构建 automap_pro 及依赖包。<br>• `test.yml`：colcon test，收集结果并上报状态。<br>• `docker.yml`：在 PR/merge 到 main 时构建 `automap-env:humble` 镜像（可推送到 GHCR 或自建仓库）。 |
| **触发策略** | PR 与 push 到 main 触发 build + test；tag 或 release 触发 Docker 构建并推送。 |
| **分支策略** | 建议 main 保护、PR 必须通过 CI；可选 develop 用于集成。 |
| **与 run_automap.sh 对齐** | CI 内编译顺序、CMake 参数与 `run_automap.sh` 中一致（含 TEASER++、overlap_transformer_msgs 等），避免“本地能过、CI 不过”。 |

### 1.3 可选

- 使用 `actions/cache` 缓存 `automap_ws/install`、TEASER++ 等，缩短 CI 时间。
- 若存在仿真或回放数据，增加“带 bag 的冒烟测试” job（仅在有资源时）。

---

## 2. 测试

### 2.1 现状

- `automap_pro` 有 test 目录（如 test_data_types、test_gps_fusion、test_loop_closure、test_robustness_framework）。
- CMake 中 BUILD_TESTING 下用 ament_cmake_gtest 仅注册了 test_data_types；其余未在 CMake 中统一注册或可能未参与 `colcon test`。

### 2.2 建议

| 项 | 内容 |
|----|------|
| **统一注册** | 在 CMakeLists.txt 的 BUILD_TESTING 分支中，把所有需运行的 gtest 可执行文件用 ament_add_gtest 注册，并 `ament_target_dependencies`，保证 `colcon test --packages-select automap_pro` 能跑全。 |
| **CI 必跑** | test.yml 中执行 `colcon test --packages-select automap_pro`，失败则 PR 不通过。 |
| **冒烟测试** | 增加一个“启动 automap_system_node 几秒后退出”的脚本（或 launch + timeout），用于验证启动路径无崩溃（可放在 scripts/smoke_test.sh）。 |
| **集成测试（可选）** | 若有小 bag 或合成数据，增加“离线回放 + 触发 SaveMap/TriggerHBA + 检查输出目录”的脚本，作为集成测试。 |
| **测试数据** | 在 repo 或 Git LFS 中提供最小测试 bag 或说明获取方式，便于复现。 |

---

## 3. 配置与校验

### 3.1 现状

- `system_config.yaml` 等通过 ConfigManager 读取，缺 key 时用默认值；无集中 schema 或启动前校验，错误配置可能运行时才暴露。

### 3.2 建议

| 项 | 内容 |
|----|------|
| **配置 schema（推荐）** | 为 `system_config.yaml` 编写 JSON Schema（或 YAML 兼容的 schema），在启动时（或单独脚本）用 yamllint / jsonschema 做校验，错误时打印清晰信息并退出。 |
| **启动前校验** | 在 `automap_system_node` 的 init 中或单独 `config_validator` 工具：检查必填项（如 frontend.mode、topic 名）、数值范围（如 frequency > 0）、路径存在性（如 output_dir 可写）。 |
| **配置文档** | 在 docs 中增加“配置项说明”表：键路径、类型、默认值、含义、示例。 |
| **多环境配置** | 可选提供 `system_config.online.yaml` / `system_config.offline.yaml` 或按环境变量选择 config 文件，便于产品化多环境部署。 |

---

## 4. 版本与发布

### 4.1 现状

- 无 CHANGELOG；package.xml / 文档中有版本号，但无统一的语义化版本与发布流程。

### 4.2 建议

| 项 | 内容 |
|----|------|
| **CHANGELOG.md** | 在仓库根目录维护 CHANGELOG.md，按 [Keep a Changelog](https://keepachangelog.com/) 格式，区分 Added / Changed / Fixed / Security，每个 release 一段。 |
| **语义化版本** | 采用语义化版本（如 1.0.0）；在 package.xml、README、文档中统一版本号来源（如从 package.xml 或 CMake 读取）。 |
| **Git tag** | 发布时打 tag（如 v1.0.0），CI 可据此构建 Docker 镜像并命名。 |
| **Release 说明** | 在 GitHub Release 中附简短说明与安装/运行指引，并链到 CHANGELOG 对应段落。 |

---

## 5. 文档与社区

### 5.1 现状

- 有 README、QUICKSTART、FOUR_PROJECTS_FUSION_DESIGN 等；缺贡献指南、安全策略、行为准则。

### 5.2 建议

| 项 | 内容 |
|----|------|
| **CONTRIBUTING.md** | 说明：分支策略、PR 流程、代码风格（如 clang-format）、测试要求、如何跑本地 CI（Docker/build/test）。 |
| **SECURITY.md** | 说明：如何报告安全问题、支持范围、漏洞处理流程；若涉及敏感数据（如日志、地图），简要说明脱敏与存储策略。 |
| **术语表（Glossary）** | 在 docs 中增加术语表：KeyFrame、Submap、HBA、TEASER++、OverlapTransformer、external_fast_livo 等，便于新成员与文档一致。 |
| **架构图与数据流** | 在 FOUR_PROJECTS_FUSION_DESIGN 或单独 doc 中保持 Mermaid 架构图、数据流、状态机（若有）与文字一致并随代码更新。 |
| **License 显式声明** | 根目录 LICENSE 文件；README 中“License”小节；若包含第三方代码，在 NOTICE 或 docs 中列出并注明许可证。 |

---

## 6. 可观测性与运维

### 6.1 现状

- 有 GetStatus 服务（系统状态、关键帧数、子图数、回环数、前端频率、内存等），便于基础监控。
- 无标准健康探针、无 ROS2 diagnostic 聚合、无 Prometheus 等指标导出。

### 6.2 建议

| 项 | 内容 |
|----|------|
| **健康探针** | 提供“健康”语义：例如基于 GetStatus 的脚本或小节点，返回 0/1（或 HTTP 200/503），供容器编排或负载均衡使用；可约定“无 keyframe 超过 N 秒视为不健康”等简单规则。 |
| **ROS2 diagnostic** | 将关键指标（前端频率、队列长度、延迟、内存）以 diagnostic_msgs/DiagnosticArray 发布，便于用 rqt_runtime_monitor 或自研监控查看。 |
| **结构化日志** | 关键路径（如回环、HBA 触发、保存地图）使用结构化字段（如 JSON 或固定 key=value），便于集中采集与检索。 |
| **可选 Prometheus** | 若需与现有监控栈集成，可增加 prometheus_ros2_exporter 或自定义节点暴露指标（如 keyframe_total、loop_closures_total、optimization_duration_seconds）。 |
| **Runbook** | 在 docs 中写“故障排查 Runbook”：常见现象（如无点云、无位姿、回环失败、HBA 失败）、可能原因、检查命令、处理步骤、回滚方式；与 QUICKSTART 故障排查区分（Runbook 更偏运维与线上）。 |

---

## 7. 安全与合规

### 7.1 现状

- 未显式涉及安全与合规文档；依赖多为常见开源库。

### 7.2 建议

| 项 | 内容 |
|----|------|
| **依赖审计** | 定期（如 CI 或季度）用 `rosdep`、`pip audit`、或 Dependabot 检查已知漏洞；对 Docker 基础镜像与关键依赖做清单管理。 |
| **许可证清单** | 列出 automap_pro 及四项目（fast-livo2、HBA、overlap_transformer_ros2、TEASER++）和主要第三方库的许可证，确保兼容（如 Apache-2.0、BSD、MIT）；在 NOTICE 或 docs/THIRD_PARTY_LICENSES.md 中说明。 |
| **数据与隐私** | 若产品涉及日志、地图、轨迹上传，在文档中说明：哪些数据会落盘/上报、是否含个人信息、脱敏策略、保留期限。 |
| **密钥与配置** | 避免在 config 或代码中写死密钥；若需 API Key 等，通过环境变量或外部密钥服务注入，并在文档中说明。 |

---

## 8. 产品体验

### 8.1 现状

- `run_automap.sh` 已支持在线/离线、build-only、external-frontend/overlap 等，体验较好。
- 缺“环境就绪”的集中检查、缺统一 Runbook、缺对首次使用的引导。

### 8.2 建议

| 项 | 内容 |
|----|------|
| **环境检查脚本** | 提供 `scripts/check_env.sh`（或与 status.sh 合并）：检查 Docker、NVIDIA 驱动、磁盘、端口、挂载等，输出明确“通过/不通过”及修复建议；run_automap.sh 可选在启动前调用。 |
| **首次运行引导** | README 或 QUICKSTART 中增加“首次运行清单”：克隆/拉取、数据目录、可选 bag 路径、首次执行命令、预期输出与下一步（如查看 RViz、保存地图）。 |
| **错误信息** | 关键失败点（如 config 缺失、topic 无数据、服务超时）返回明确错误码与提示文案，便于 Runbook 和用户自助排查。 |
| **Runbook 与 QUICKSTART** | 故障排查集中到 Runbook；QUICKSTART 侧重“正常路径”与常用命令；两者互相引用。 |

---

## 9. 代码质量与规范

### 9.1 现状

- CMake 中依赖 ament_lint_auto；未强制 pre-commit 或 CI 中必须通过 lint。

### 9.2 建议

| 项 | 内容 |
|----|------|
| **CI 中 Lint** | 在 GitHub Actions 中增加 step：`colcon build --packages-select automap_pro` 且启用 ament_lint（或单独 `ament_cmake_lint_cmake` 等），lint 失败则失败。 |
| **格式化** | 约定 C++ 用 clang-format，Python 用 black/ruff；在 CONTRIBUTING 中说明，并提供 format 脚本或 pre-commit config。 |
| **Pre-commit（可选）** | 使用 pre-commit 在提交前跑格式与基础检查，减少 CI 失败与评审成本。 |
| **静态分析（可选）** | 若有资源，可在 CI 中增加 clang-tidy 或 cppcheck，仅对 automap_pro 或关键模块。 |

---

## 10. 部署与交付

### 10.1 现状

- 通过 Docker + run_automap.sh 一键运行；无 docker-compose 编排、无 K8s 示例、无离线安装包。

### 10.2 建议

| 项 | 内容 |
|----|------|
| **Docker Compose** | 提供 `docker-compose.yml`：automap 服务 + 可选 rosbag 播放、RViz；与 run_automap.sh 行为对齐，便于本地或服务器一键 up。 |
| **镜像版本与仓库** | 镜像打 tag（如 v1.0.0、latest），并说明推送到的仓库（如 GHCR）；文档中注明推荐镜像与最低版本。 |
| **离线安装包（可选）** | 若有内网或离线部署需求，提供“依赖清单 + 离线镜像/包”的说明或脚本，便于在无外网环境复现。 |
| **资源与限流** | 在文档或 compose 中说明推荐 CPU/内存/GPU、以及可选的 cgroup/limits，便于运维 sizing。 |

---

## 11. 优先级与实施顺序建议

| 阶段 | 内容 | 预期产出 |
|------|------|----------|
| **P0** | CI 构建 + 测试必跑 | `.github/workflows/build.yml`、`test.yml`，PR 必须通过 |
| **P0** | 测试可重复跑全 | CMake 中测试统一注册，CI 中 colcon test |
| **P1** | 配置校验 + 文档 | schema 或校验脚本、配置说明表、Runbook |
| **P1** | 版本与发布规范 | CHANGELOG.md、tag 策略、Release 说明 |
| **P1** | 社区与合规文档 | CONTRIBUTING.md、SECURITY.md、术语表、LICENSE/NOTICE |
| **P1** | 健康与 Runbook | 健康探针脚本、故障 Runbook、环境检查脚本 |
| **P2** | 可观测性增强 | diagnostic_msgs 或 Prometheus、结构化日志 |
| **P2** | 代码规范强制 | CI lint、clang-format、pre-commit（可选） |
| **P2** | 部署与交付 | docker-compose、镜像版本说明、可选离线包 |

---

## 12. 总结

在保持当前“四项目融合 + 一键运行”的基础上，优先补齐 **CI/CD、测试、配置校验、版本与发布、文档与社区、可观测性与 Runbook**，即可显著提升工程化与产品化水平；安全与合规、代码规范、部署形态可按实际需求分阶段实施。以上建议均可按条目拆分为具体 issue 或 task 逐步落地。
