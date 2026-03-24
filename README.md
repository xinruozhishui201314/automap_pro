# AutoMap-Pro - 高精度自动化点云建图系统

> 基于 ROS2 Humble + CUDA 的实时建图系统，支持在线/离线模式，适用于城市道路、园区、隧道、矿山等复杂场景。

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![CUDA](https://img.shields.io/badge/CUDA-11.8-green.svg)](https://developer.nvidia.com/cuda-toolkit)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## ✨ 特性

- 🚀 **实时建图**：前端频率 >=10Hz，支持在线实时建图
- 🎯 **高精度**：支持回环 + 全局优化的一致性地图构建
- 🔄 **增量式**：支持多次采集数据的增量拼接建图
- 🤖 **自动化**：`run_automap.sh` 统一编译、运行、日志输出流程
- 🌐 **复杂场景适配**：GPS 时好时坏、长走廊、隧道等退化场景
- 🧩 **模块化演进**：V3 微内核模块 + 事件总线 + 地图注册中心

---

## 🚀 快速开始

### 前置要求

- Ubuntu 20.04 / 22.04
- Docker 20.10+
- NVIDIA GPU + Driver 470+（可选，无 GPU 时可 CPU 模式）
- NVIDIA Container Runtime（使用 GPU 时）

### 主入口（推荐）

主入口脚本：`run_automap.sh`（在仓库根目录执行）

```bash
# 克隆项目
git clone <repository-url> automap_pro
cd automap_pro

# 一键编译 + 运行（在线模式，默认）
bash run_automap.sh

# 离线回放建图（指定 bag 与配置）
bash run_automap.sh --offline \
  --bag-file data/automap_input/M2DGR/street_03_ros2 \
  --config system_config_M2DGR.yaml \
  --bag-rate 0.5
```

首次运行会执行：镜像检查 -> 容器内编译 `automap_ws` -> 启动建图流程（离线时自动回放 bag）-> 输出日志与结果。

### 常用命令

```bash
# 仅编译
bash run_automap.sh --build-only

# 仅运行（须先成功编译）
bash run_automap.sh --run-only

# 清理后重编译
bash run_automap.sh --clean --build-only

# 离线回放，不开 RViz
bash run_automap.sh --offline --bag-file <path> --no-rviz

# 后端 gdb（抓 automap_system_node backtrace）
bash run_automap.sh --offline --bag-file <path> --gdb

# 前端 gdb（抓 fastlivo_mapping 崩溃）
bash run_automap.sh --offline --bag-file <path> --gdb-frontend

# 查看全部参数
bash run_automap.sh --help
```

`automap_start.sh` 仍可作为兼容入口，但默认推荐统一使用 `run_automap.sh`。

---

## 🏗️ Architecture At A Glance

V3 系统采用微内核式模块注册与事件驱动，主数据流如下：

```mermaid
flowchart TD
    A[LiDAR / IMU / GPS] --> B[FrontEndModule]
    B --> C[SyncedFrameEvent]
    C --> D[DynamicFilterModule]
    D --> E[FilteredFrameEventRequiredDs]
    E --> F[MappingModule]
    F --> G[OptimizerModule]
    G --> H[HBA / iSAM2]
    H --> I[OptimizationResultEvent]
    I --> J[MapRegistry]
    J --> K[VisualizationModule]
    J --> L[Map / Trajectory Output]

    C --> M[SemanticModule]
    M --> N[SemanticProcessor<br/>SLOAM + ONNX]
    N --> O[SemanticLandmarkEvent]
    O --> F

    F --> P[LoopModule]
    P --> G
```

对应文本版主链路：

```text
传感器数据 (LiDAR/IMU/GPS)
        ↓
FrontEndModule (同步帧)
        ↓
DynamicFilterModule (降采样/动态滤除)
        ↓
MappingModule (关键帧/子图/图优化任务生产)
        ↓
OptimizerModule + HBA / iSAM2 (全局一致性优化)
        ↓
MapRegistry (统一状态写入与版本推进)
        ↓
输出地图/轨迹/可视化
```

语义分支（必选）：

```text
SyncedFrameEvent
   ↓
SemanticModule / SemanticProcessor (SLOAM + ONNX，禁止 Stub)
   ↓
SemanticLandmarkEvent
   ↓
MappingModule -> CYLINDER_LANDMARK_FACTOR -> Optimizer
```

---

## 🏛️ 系统架构（详细）

### 1) 模块启动与依赖顺序

系统按以下顺序完成模块注册与启动（先生产数据、后消费数据）：

1. `FrontEndModule`
2. `SemanticModule`
3. `DynamicFilterModule`
4. `GPSModule`
5. `LoopModule`
6. `VisualizationModule`
7. `OptimizerModule`
8. `MappingModule`

该顺序用于保证：

- 前端同步帧优先进入事件流
- 语义与动态滤波可在建图前准备好输入
- 优化器与建图调度在完整输入链路就绪后再工作

### 2) 部署拓扑（默认 Docker 模式）

```mermaid
flowchart LR
    subgraph Host[Host Ubuntu]
        H1[run_automap.sh]
        H2[data/]
        H3[logs/]
    end

    subgraph C[Docker container automap-env:humble]
        C1[ROS2 Launch]
        C2[FrontEnd + AutoMapSystem]
        C3[Semantic Runtime<br/>ONNX + sloam_segmentation]
        C4[RViz2 optional]
    end

    H1 --> C1
    H2 <--> C2
    C2 --> C3
    C2 --> C4
    C1 --> C2
    C2 --> H3
```

```text
Host (Ubuntu)
 ├─ run_automap.sh
 ├─ data/                      (输入 bag + 输出地图)
 ├─ logs/                      (build.log / full.log)
 └─ Docker container (automap-env:humble)
     ├─ ROS2 launch (offline/online)
     ├─ FrontEnd + AutoMapSystem (V3 modules)
     ├─ Optional Semantic Runtime (ONNX + sloam_segmentation)
     └─ RViz2 (可选)
```

### 3) 关键事件契约（Event Contracts）

- `SyncedFrameEvent`：前端时序对齐后的核心输入事件
- `FilteredFrameEventRequiredDs`：动态滤波后的建图输入事件
- `SemanticLandmarkEvent`：语义模块输出的地标事件
- `OptimizationResultEvent`：优化器输出的位姿更新事件

约定：

- 事件必须带时间戳并保持单向数据流（sensor -> frontend -> backend -> output）
- 跨模块共享状态优先通过 `MapRegistry` 快照/写入接口，不直接绕过状态中心

### 4) 并发与队列模型（运维视角）

- 模块内部采用 worker + queue 的异步处理模式，避免在 ROS 回调中做重计算
- 队列必须有边界与超时策略，防止离线高倍速回放导致内存/延迟失控
- 日志定位优先看 `logs/full.log` 的模块 INIT/RUN 标记，确认链路是否卡在某一阶段

---

## 🧭 模块边界与职责（当前实现约定）

- `MapRegistry`：关键帧、子图、版本号、对齐 epoch 的统一状态中心（Single Source of Truth）
- `MappingModule`：关键帧与子图生成、优化任务组织、语义地标入图
- `OptimizerModule`：消费图优化任务并更新位姿结果
- `SemanticModule`：异步语义处理与语义事件发布（必须真实生效，禁止 Stub）
- `LoopModule`：回环候选与几何验证链路（含 OverlapTransformer/TEASER++）
- `VisualizationModule`：RViz 数据发布与观测输出

**约束建议**：共享位姿写入尽量经 `MapRegistry` 统一入口，降低跨模块并发写冲突与状态漂移风险。

---

## 🧠 语义管线（必选项清单）

> 当前工程策略：**语义模块必须真实处理数据并发挥作用，禁止以 Stub 模式运行。**

### A. 构建前必选依赖（缺一不可）

- `thrid_party/sloam_rec/sloam/include/segmentation/inference.h` 必须存在（vendor 目录）
- ONNX Runtime 必须可被 CMake 发现（`ONNXRUNTIME_HOME` 或 `install_deps/onnxruntime`）
- `sloam_segmentation` 动态库必须可被 CMake 找到

### B. 构建期强约束（必须满足）

- CMake 默认启用 `AUTOMAP_REQUIRE_SEMANTIC=ON`
- 若 ONNX Runtime 或 `sloam_segmentation` 缺失，配置阶段直接 `FATAL_ERROR`
- 只有依赖齐全时，才会定义 `AUTOMAP_USE_SLOAM_SEMANTIC=1`

### C. 构建验收清单（必须逐条确认）

执行：

```bash
bash run_automap.sh --build-only --clean
```

在构建日志中必须看到：

- `ONNX Runtime + sloam_segmentation found: SLOAM semantic ENABLED`
- `AUTOMAP_USE_SLOAM_SEMANTIC=1`（可在编译命令或 CMake 输出中确认）

若看到以下任一信息即为失败（不得继续运行）：

- `Semantic pipeline is REQUIRED but ONNX Runtime was not found`
- `Semantic pipeline is REQUIRED but sloam_segmentation library is missing`

### D. 运行验收清单（必须逐条确认）

离线运行后检查 `logs/full.log`，必须满足：

- 出现 `\[SEMANTIC\]\[Processor\]\[INIT\] step=ok`
- 出现 `\[SEMANTIC\]\[Module\]\[INIT\] step=ok enabled=1`
- 出现 `\[SEMANTIC\]\[Module\]\[RUN\] step=done ... landmarks=...`

若出现以下任一信息即为失败（视为发布阻断）：

- `stub mode`
- `SemanticProcessor requires AUTOMAP_USE_SLOAM_SEMANTIC=1`
- `SemanticModule init failed: runtime semantic capability is unavailable`

### E. 日常排查命令（建议固定执行）

```bash
# 1) 快速确认语义初始化
rg "\[SEMANTIC\]\[Processor\]\[INIT\]|\[SEMANTIC\]\[Module\]\[INIT\]" logs/full.log

# 2) 确认语义持续产出（landmark 数）
rg "\[SEMANTIC\]\[Module\]\[RUN\] step=done" logs/full.log

# 3) 发现即失败：任何 stub/降级痕迹
rg "stub mode|runtime semantic capability is unavailable|AUTOMAP_USE_SLOAM_SEMANTIC=1" logs/full.log logs/build.log
```

---

## 📦 Third-Party / Vendoring Policy

当前工程存在多个第三方目录（如 `thrid_party/` 与 `automap_pro/thrid_party/`）。为降低长期维护风险，建议遵循以下约束：

1. 新增依赖前先确认“唯一权威目录”（避免重复 vendor）
2. 记录来源版本、补丁、构建选项、升级策略
3. 大模型/大二进制使用 Git LFS 管理（见 `README_LFS.md`）
4. 保持 `run_automap.sh`、CMake 与文档中的依赖路径一致

---

## 🧪 Build & Run 真值表（推荐用法）

| 场景 | 命令 | 说明 |
|------|------|------|
| 默认在线 | `bash run_automap.sh` | 编译 + 启动在线流程 |
| 离线建图 | `bash run_automap.sh --offline --bag-file <path> --config <yaml>` | 回放 bag 并建图 |
| 仅编译 | `bash run_automap.sh --build-only` | 用于 CI/预热 |
| 仅运行 | `bash run_automap.sh --run-only` | 仅在已完成完整编译时使用 |
| 后端调试 | `bash run_automap.sh ... --gdb` | 捕获后端 backtrace |
| 前端调试 | `bash run_automap.sh ... --gdb-frontend` | 捕获 fastlivo_mapping 崩溃栈 |

---

## 📁 项目结构（简化）

```text
automap_pro/                    # 仓库根目录
├── run_automap.sh              # 主入口（编译/运行/日志）
├── automap_start.sh            # 兼容入口（历史脚本）
├── automap_ws/                 # ROS2 工作空间
├── automap_pro/                # 主包源码（config/launch/include/src）
├── data/                       # 输入数据与建图输出
├── logs/                       # build.log / automap.log / full.log
├── docs/                       # 文档目录
└── docker/                     # 镜像与容器相关脚本
```

---

## 🛠️ 系统要求

| 组件 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU  | x86_64，4 核 | x86_64，8 核以上 |
| 内存 | 16GB | 32GB 以上 |
| GPU  | NVIDIA RTX 3060（可选） | NVIDIA RTX 4060 或更高 |
| 存储 | 50GB 可用 | 100GB 可用 |
| 系统 | Ubuntu 20.04 / 22.04 | Ubuntu 22.04 |

---

## ⚠️ 运行契约与常见故障

- 离线模式必须提供 `--bag-file`，并确保 metadata 可读可解析
- 若出现 `yaml-cpp bad conversion`，优先检查 bag metadata 完整性
- 若出现 `pose_vec.size()=0` 或优化阶段空输入，优先检查前端输出与时间同步
- 若出现 `undefined symbol`，避免直接 `--run-only`，先做完整编译

建议首先查看：

- `logs/full.log`
- `logs/build.log`
- `automap_pro/docs/TROUBLESHOOTING.md`

---

## 📚 文档索引

| 文档 | 说明 |
|------|------|
| [QUICK_START.md](QUICK_START.md) | 快速开始、常用命令与错误排查 |
| [docs/BUILD_DEPLOY_RUN.md](docs/BUILD_DEPLOY_RUN.md) | 编译/部署/运行说明 |
| [docs/README.md](docs/README.md) | 文档索引 |
| [automap_pro/docs/TROUBLESHOOTING.md](automap_pro/docs/TROUBLESHOOTING.md) | 系统故障排查 |
| [README_LFS.md](README_LFS.md) | Git LFS 使用说明 |
| [docker/DOCKER_USAGE.md](docker/DOCKER_USAGE.md) | Docker 使用说明 |
| [高精度高性能自动化点云建图系统架构文档.md](高精度高性能自动化点云建图系统架构文档.md) | 架构与数据流 |

---

## 📋 文档一致性矩阵（本次更新建议）

| 文档 | 当前状态 | 建议 |
|------|----------|------|
| `README.md` | 已更新为 `run_automap.sh` 主入口 | 持续同步模块边界与语义管线 |
| `docs/BUILD_DEPLOY_RUN.md` | 存在主入口描述冲突（部分段落提到 `automap_start.sh` 为推荐） | 统一为 `run_automap.sh` 主入口 |
| `docs/CONFIG_SUMMARY.md` | 仍偏向历史入口描述 | 与当前脚本参数和配置方式对齐 |
| `docs/MAPPING_WORKFLOW.md` | 包含历史启动流示例 | 增补 V3 模块链路与新入口 |

---

## 🤝 贡献

欢迎提交 Issue 与 Pull Request。

---

## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

**Made with ❤️ by AutoMap-Pro Team**

文档版本：v2.2（V3 架构与语义管线说明）  
更新日期：2026-03-24
