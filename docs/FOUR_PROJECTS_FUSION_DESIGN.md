# 四开源项目融合方案设计

## 0. Executive Summary

| 项目 | 集成方式 | 参与方式 | 收益 |
|------|----------|----------|------|
| **fast-livo2-humble** | 独立 ROS2 节点 + automap_pro 适配器 | 替代自研前端，订阅其 Odometry/点云生成 KeyFrame | 使用原版高性能 LiDAR-IMU-Visual 里程计 |
| **OverlapTransformer-master** | Python ROS2 服务节点 或 C++ LibTorch 同模型 | 回环粗匹配描述子由上游模型计算 | 原版重叠估计与检索性能 |
| **TEASER-plusplus-master** | 工作空间内编译为库，automap_pro 链接 | 回环精匹配已用 TEASER++，改为用上游源码构建 | 与上游一致、便于修参与扩展 |
| **HBA-main** | 批处理桥接：automap_pro 导出 HBA 数据格式 → 调用 HBA → 读回位姿 | 在线仍用自研 HBAWrapper；可选周期/会话结束跑 HBA 提升全局一致性 | 利用 HBA 分层 BA 与 GPS 插值 |

**风险与回滚**：通过配置开关（如 `use_external_frontend`、`use_external_overlap`）可在自研与开源实现间切换；HBA 批处理为可选后处理，不影响在线主链路。

---

## 1. 背景与目标

- **目标**：在现有 AutoMap-Pro 流水线上，让四个高性能开源项目真正参与运行，在其基础上做融合与最小修改。
- **约束**：保持安全关键与工程可回滚；尽量不重写四个项目核心，以适配层与数据桥接为主。

---

## 2. 假设与待确认

| 假设 | 影响 |
|------|------|
| Fast-LIVO2 与当前 Livox/IMU topic 命名兼容 | 否则在 launch 中做 remap |
| OverlapTransformer 使用 KITTI 风格 range image (64×900) | 与 automap_pro 现有距离图参数一致或可配置 |
| HBA 仅作批处理（会话结束或周期触发）可接受 | 若需严格在线 HBA，需将 HBA 核心改为库 API |
| 运行环境具备 Python3 + PyTorch（OverlapTransformer 服务） | 否则采用 C++ LibTorch + TorchScript 导出 |

---

## 3. 架构总览（融合后）

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                    Layer 0: 传感器                        │
                    │   /livox/lidar   /livox/imu   /gps/fix  (可选 /camera)   │
                    └───────────────────────────┬───────────────────────────────┘
                                                │
        ┌───────────────────────────────────────┼───────────────────────────────────────┐
        │                                       ▼                                       │
        │   ┌─────────────────────────────────────────────────────────────────────┐   │
        │   │  fast-livo2-humble (独立节点)                                          │   │
        │   │  fastlivo_mapping: 订阅 LiDAR/IMU/Image → 发布 /aft_mapped_to_init,     │   │
        │   │                    /path, /cloud_registered                             │   │
        │   └───────────────────────────────────┬─────────────────────────────────────┘   │
        │                                       │                                         │
        │                                       ▼                                         │
        │   ┌─────────────────────────────────────────────────────────────────────┐   │
        │   │  automap_pro: FastLIVO2Adapter                                        │   │
        │   │  订阅 /aft_mapped_to_init (+ /cloud_registered) → KeyFrame + 位姿     │   │
        │   └───────────────────────────────────┬─────────────────────────────────────┘   │
        └───────────────────────────────────────┼─────────────────────────────────────────┘
                                                │
                    ┌───────────────────────────▼───────────────────────────┐
                    │  automap_pro: SubmapManager → LoopDetector             │
                    │  子图描述子: 可选 OverlapTransformer 服务 (Python 节点)  │
                    │  精匹配: TEASER++ (源码编译，优先于系统版本)     │
                    └───────────────────────────┬───────────────────────────┘
                                                │
                    ┌───────────────────────────▼───────────────────────────┐
                    │  在线优化: automap_pro HBAWrapper (GTSAM/Ceres)       │
                    │  可选批处理: 导出 → HBA-main (data_path) → 读回 pose    │
                    └─────────────────────────────────────────────────────┘
```

---

## 4. 分项设计

### 4.1 Fast-LIVO2 (fast-livo2-humble)

**接口**（上游）：

- 订阅：LiDAR（`livox_ros_driver2::msg::CustomMsg` 或 `sensor_msgs::msg::PointCloud2`）、`sensor_msgs::msg::Imu`、可选 `sensor_msgs::msg::Image`
- 发布：`/aft_mapped_to_init` (Odometry)、`/path` (Path)、`/cloud_registered` (PointCloud2)

**集成方式**：

1. **构建**：将 `fast-livo2-humble` 放入 `automap_ws/src`，与 `automap_pro`、`livox_ros_driver2` 一起 `colcon build`。
2. **运行**：launch 中先启动 `fast_livo` 的 `fastlivo_mapping` 节点（或使用其现有 launch 中节点），再启动 automap_pro。
3. **automap_pro 侧**：
   - 新增 **FastLIVO2Adapter**：订阅 `/aft_mapped_to_init`（以及可选 `/cloud_registered` 做关键帧点云），将 Odometry 转为 `Pose3d`，按关键帧策略生成 `KeyFrame::Ptr`，调用现有 `KeyFrameCallback` / `PoseCallback`，下游 SubmapManager、LoopDetector、HBA 不变。
   - 配置项：`frontend.mode: "external_fast_livo"` 时使用 Adapter；`"internal"` 时保留当前自研 ESIKF 前端。
4. **数据流**：LiDAR/IMU → Fast-LIVO2 → Odometry/Path/Cloud → Adapter → KeyFrame 流 → 与现有一致。

**关键文件**（见下节变更清单）。

---

### 4.2 OverlapTransformer (OverlapTransformer-master)

**接口**（上游）：Python，输入为 range image（如 64×900），输出 256 维描述子；推理入口为 `featureExtracter.forward(combined_tensor)`，前处理为 `range_projection` + 构建 tensor。

**集成方式（推荐：Python ROS2 服务）**：

1. 新建 ROS2 Python 包 `overlap_transformer_ros2`：
   - 提供 **服务** `ComputeDescriptor`：请求为 `sensor_msgs/PointCloud2` 或自定义 `RangeImage`，响应为 `std_msgs/Float32MultiArray`（256 维）或自定义 `Descriptor.msg`。
   - 内部：点云 → `range_projection`（与上游 utils 一致）→ 转 tensor → 模型 forward → 返回描述子。
2. **automap_pro**：
   - 配置项 `loop_closure.overlap_transformer.mode: "external_service"`。
   - `LoopDetector` 中：若为 external，则对每个待计算描述子的子图调用该服务（异步或同步，建议异步+缓存），将返回描述子挂到 SubMap；检索仍可在 automap_pro 内用 L2 距离做 TopK。
3. **备选**：将 OverlapTransformer 导出为 TorchScript，在 automap_pro 的 C++ LibTorch 中加载并调用，无需 Python 节点；需维护与上游一致的前处理（range_projection 参数）。

**关键文件**：新增 `overlap_transformer_ros2` 包（见变更清单）。

---

### 4.3 TEASER++ (TEASER-plusplus-master)

**现状**：automap_pro 已通过 `find_package(teaserpp)` 链接 TEASER++，Docker 中来自 `docker/deps/TEASER-plusplus`。

**集成方式**：

1. **工作空间内使用上游**：将 `TEASER-plusplus-master` 放入 `automap_ws/src`（或通过 `add_subdirectory` 在 automap_pro 内构建），先于 automap_pro 编译并 `install`，使 `find_package(teaserpp)` 找到工作空间版本。
2. **Docker**：镜像中可改为从 `TEASER-plusplus-master` 拷贝到 `/tmp/TEASER-plusplus` 再编译安装，与当前 docker/deps 流程一致，仅改 COPY 源。
3. **代码**：automap_pro 的 `teaser_matcher.cpp` 无需改逻辑，仅保证链接的 TEASER++ 来自上游仓库即可。

---

### 4.4 HBA (HBA-main)

**上游接口**：HBA_ROS2 节点从 `data_path` 读取：

- `pose.json`：每行 `lidar_time tx ty tz qx qy qz qw`
- `pcd/`：按帧编号的 PCD 点云
- `gps_imu_data.json` 或 `gps_raw_data.json`（可选）

输出：同目录下 `pose_trans.json` 等。

**集成方式（批处理桥接）**：

1. **导出**：在 automap_pro 中新增模块（或节点）`HBAExport`：
   - 按当前 KeyFrame 序列 + 子图点云，写出 HBA 所需目录结构：`data_path/pose.json`、`data_path/pcd/xxxx.pcd`、`data_path/gps_imu_data.json`（若有）。
   - 位姿顺序与 HBA `read_pose` 一致（lt tx ty tz qx qy qz qw）。
2. **调用**：通过 **服务** 或 **命令行** 触发 HBA 可执行文件（或 HBA 节点配置 `data_path` 指向该目录），HBA 写回 `pose_trans.json`。
3. **读回**：automap_pro 读取 `pose_trans.json`，将优化后位姿写回 KeyFrame/Submap 的 `T_w_b_optimized`，并触发地图重投影与发布。
4. **触发时机**：会话结束（SaveMap 时）、或周期（如每 N 分钟）、或显式服务 `TriggerHBA`。
5. **在线**：在线仍使用现有 HBAWrapper（GTSAM 位姿图），HBA-main 仅作离线/批处理增强。

**关键文件**：automap_pro 内 `hba_bridge` 导出/读回逻辑 + 配置 + 可选 HBA 启动方式（见变更清单）。

---

## 5. 变更清单（文件/模块）

| 类型 | 路径 | 说明 |
|------|------|------|
| 新增 | `automap_pro/include/automap_pro/frontend/fast_livo2_adapter.h` | Fast-LIVO2 话题适配器声明 |
| 新增 | `automap_pro/src/frontend/fast_livo2_adapter.cpp` | 实现：订阅 Odometry/Cloud → KeyFrame + PoseCallback |
| 修改 | `automap_pro/src/nodes/automap_system_node.cpp` | 根据配置选择 Frontend 或 FastLIVO2Adapter |
| 修改 | `automap_pro/config/system_config.yaml` | 增加 `frontend.mode`、`loop_closure.overlap_transformer.mode` 等 |
| 新增 | `automap_pro/src/backend/hba_export.cpp` / `.h` | 导出 pose.json + pcd/ + gps 到 HBA data_path |
| 新增 | `automap_pro/src/backend/hba_result_loader.cpp` / `.h` | 读取 pose_trans.json 写回优化位姿 |
| 新增 | `overlap_transformer_ros2/` (Python 包) | 服务 ComputeDescriptor，内部调用 OverlapTransformer |
| 修改 | `automap_pro/src/loop_closure/loop_detector.cpp` | external 时调用 Overlap 服务取描述子 |
| 修改 | `automap_pro/launch/automap_online.launch.py` | 可选启动 fast_livo、overlap_transformer_ros2 |
| 修改 | `run_automap.sh` | 准备工作空间时链入 fast-livo2、TEASER-plusplus-master、OverlapTransformer、HBA；构建顺序与 launch 模式 |

---

## 6. 编译/部署/运行说明（融合模式）

- **依赖**：ROS2 Humble、Livox driver、与当前一致；额外需 Python3、PyTorch（若用 OverlapTransformer 服务）。
- **工作空间**：
  - `automap_ws/src/` 下：`livox_ros_driver2`、`automap_pro`、`fast_livo`（指向 fast-livo2-humble）、可选 `overlap_transformer_ros2`、可选 `hba`（HBA_ROS2）。
  - TEASER++：从 `TEASER-plusplus-master` 构建并 install 到工作空间或系统，供 automap_pro 的 find_package 使用。
- **构建顺序**：`teaserpp`（若在 ws）→ `fast_livo` → `livox_ros_driver2` → `automap_pro`；`overlap_transformer_ros2` 与 `hba` 独立。
- **启动**：`ros2 launch automap_pro automap_online.launch.py use_external_frontend:=true use_external_overlap:=true`（参数名示例），内部会启动 fast_livo 与 overlap 服务（或按需启动）。

---

## 7. 验证与回滚

- **验证**：同一 bag 下对比 `frontend.mode: internal` 与 `external_fast_livo` 轨迹与关键帧数；对比自研描述子与 OverlapTransformer 服务描述子回环召回；对比仅 HBAWrapper 与 +HBA 批处理后的全局误差。
- **回滚**：配置改回 `internal` / 自研描述子、禁用 HBA 导出即可恢复当前行为。

---

## 8. 后续演进（MVP → V1 → V2）

- **MVP**：仅接入 Fast-LIVO2（Adapter + launch）+ TEASER++ 工作空间构建。
- **V1**：OverlapTransformer 服务 + HBA 批处理桥接（导出/读回）。
- **V2**：HBA 核心改为库 API 供 automap_pro 在线调用；OverlapTransformer 可选 TorchScript 内嵌 C++ 减少进程间调用。

---

## 9. 四项目融合模式使用说明（已全部实现）

### 9.1 目录与构建

将四个开源工程与本次新增接口包放在仓库根下（与 `automap_pro/` 同级）：

- `fast-livo2-humble/` — 前端
- `OverlapTransformer-master/` — 回环描述子（Python 服务用）
- `TEASER-plusplus-master/` — 回环精匹配（工作空间内编译并 install）
- `HBA-main/` — 批处理优化（HBA_ROS2）
- `overlap_transformer_msgs/` — 描述子服务接口（本次新增）
- `overlap_transformer_ros2/` — 描述子服务节点（本次新增）

一键编译（会按需链入并编译上述包及 TEASER++ 安装到工作空间）：

```bash
bash run_automap.sh --build-only
```

### 9.2 Fast-LIVO2 前端（默认启用）

**注意：** 默认配置已设置为使用 Fast-LIVO2 前端。

1. 配置 `config/system_config.yaml`：`frontend.mode: "external_fast_livo"`（默认值）。
2. 运行：`bash run_automap.sh`（或 `bash run_automap.sh --external-frontend`）。
3. 回滚：将配置改为 `frontend.mode: "internal"`，或使用 `bash run_automap.sh --no-external-frontend`。

### 9.3 OverlapTransformer 描述子服务

1. 配置：`loop_closure.overlap_transformer.mode: "external_service"`。
2. 运行：`bash run_automap.sh --external-overlap`（或 `use_external_overlap:=true`）。launch 会启动 `overlap_transformer_ros2` 的 `descriptor_server`。
3. 模型：将 OverlapTransformer 预训练权重路径通过节点参数 `model_path` 传入（或设环境 `OVERLAP_TRANSFORMER_ROOT` 指向 `OverlapTransformer-master`）。
4. 回滚：`mode: "internal"`，去掉 `--external-overlap`。

### 9.4 TEASER++（源码优先）

**说明：** AutoMap-Pro 优先使用从 `TEASER-plusplus-master` 源码编译的 TEASER++ 库，而非系统安装的版本。

**优先级：**
1. 工作空间源码编译版本（`automap_ws/install/teaserpp/`）- 优先
2. 系统安装版本（`/usr/local/`）- 回退选项（警告提示）

**优势：**
- 版本控制：可以追踪和回退到特定提交版本
- 调试便利：可以直接修改源码进行调试和优化
- 避免冲突：避免系统版本过旧或依赖不兼容
- 一致性：确保所有环境使用相同的 TEASER++ 版本

- 若存在 `TEASER-plusplus-master/`，脚本会在工作空间内编译并 install 到 `install/teaserpp`，automap_pro 会通过 `find_package(teaserpp)` 链接该版本。
- 未提供时仍使用镜像内已安装的 TEASER++。

### 9.5 HBA-main 批处理

1. 配置：`backend.hba_bridge.export_path`、可选 `run_after_export: true` 与 `run_command`。
2. 调用服务：`ros2 service call /automap/trigger_hba automap_pro/srv/TriggerHBA "{data_path: ''}"`。  
   - 会导出当前 KeyFrame 序列为 HBA 目录（pose.json + pcd/），可选执行 HBA 进程，读回 `pose_trans.json` 并写回 KeyFrame 与子图位姿，再重投影地图。
3. 若未设 `run_after_export`，可手动运行 HBA 后再次调用同一 `data_path` 仅做“读回并应用”。

### 9.6 一键四项目融合运行示例

```bash
# 默认已启用 Fast-LIVO2 前端
bash run_automap.sh

# 完整融合模式（Fast-LIVO2 + OverlapTransformer）
bash run_automap.sh --external-overlap

# 或使用自研前端
bash run_automap.sh --no-external-frontend
```

按需再调用 `/automap/trigger_hba` 做 HBA 批处理。
