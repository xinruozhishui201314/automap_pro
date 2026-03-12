# HBA 约束说明与「后端优化后再 HBA」的影响

## 1. HBA 优化都有哪些约束？

当前 automap_pro 中 **HBA** 有两种实现路径，约束对比如下。

### 1.1 GTSAM Fallback（无 hba_api 时）

代码位置：`src/backend/hba_optimizer.cpp` → `runGTSAMFallback()`。

| 约束类型 | 含义 | 说明 |
|----------|------|------|
| **Prior** | 首关键帧 (k0) 固定 | 1 个 PriorFactor，强约束第一帧位姿，方差 1e-8 |
| **Between（里程计链）** | 相邻关键帧相对位姿 | rel = T_{i-1}^{-1} * T_i，方差 0.01，形成链式约束 |
| **GPS** | 有有效 GPS 的关键帧位置约束 | GPSFactor(key, position_enu, 对角协方差)，仅约束 XYZ |
| **回环** | 当前 GTSAM fallback **无** | 仅 Prior + Between + GPS，不加入回环边 |

即：**Prior + 相邻 Between + 可选 GPS**，无回环。

### 1.2 hba_api（外部 HBA 模块）

若编译并启用 hba_api，HBA 使用外部多层级 BA，约束由该模块内部定义（通常也包含 Prior、相对位姿、可选 GPS 等），与 GTSAM fallback 的因子类型和拓扑可能不同，但语义上仍是「先验 + 相对约束 + 可选 GPS」。

### 1.3 与后端（ISAM2）约束对比

| 约束 | 后端 ISAM2 | HBA (GTSAM fallback) |
|------|------------|----------------------|
| 节点 | 子图（submap） | 关键帧（keyframe） |
| Prior | 首子图 | 首关键帧 |
| Between | 相邻子图 odom | 相邻关键帧 odom |
| 回环 | 有（Huber 鲁棒） | **无** |
| GPS | 有（子图级） | 有（关键帧级） |

---

## 2. 是否可以在后端优化后再进行 HBA 优化？

可以，且当前设计上**已经允许**「先有后端、再跑 HBA」，只是**初始值**尚未用后端结果。

### 2.1 当前时序（已实现：严格先后端再 HBA + 释放 GTSAM 再跑 HBA）

- **所有 HBA 触发点**（子图冻结、GPS 对齐、sensor_idle、finish_mapping、TriggerHBA 服务）在调用 `hba_optimizer_.triggerAsync` 前都会先调用 **`ensureBackendCompletedAndFlushBeforeHBA()`**：  
  - `isam2_optimizer_.waitForPendingTasks()`，等待 ISAM2 队列空；  
  - 若 `hasPendingFactorsOrValues()` 则 `forceUpdate()`，把 pending 提交并释放 GTSAM 相关变量。  
  这样**严格**「后端优化完成并释放 GTSAM 后再跑 HBA」，避免两路 GTSAM（ISAM2 与 HBA 的 LM）竞争导致崩溃。
- **HBA 初始值**：`runGTSAMFallback()` 中已用后端结果做初始值——当关键帧的 `T_w_b_optimized` 与 `T_w_b` 不同时（表示已被后端或上一轮 HBA 更新），用 **T_w_b_optimized** 填入 GTSAM `initial` 并计算 Between 的 rel；否则用 **T_w_b**。
- **HBA 约束**：仍包含 **GPS 约束**（`task.enable_gps && gps_aligned_` 时加入关键帧级 GPS 因子），与 Prior、Between 一起优化。

### 2.2 「后端优化后再 HBA」的两种做法

1. **只改触发顺序（严格先后端、再 HBA）**  
   - 例如：周期 HBA 改为「在 ISAM2 本周期 update 完成之后」再 `triggerAsync`，或仅在「有回环/GPS 且 ISAM2 已 commitAndUpdate」后触发 HBA。  
   - 这样 HBA 与后端在逻辑上严格「先后端、再 HBA」，避免同一时刻两路同时写优化结果。

2. **用后端结果做 HBA 的初始值（推荐）**  
   - 在 `runGTSAMFallback()` 中，构建 `initial` 和 Between 的 rel 时：  
     - 若关键帧已有有效的 `T_w_b_optimized`（例如来自本周期或上一周期的 ISAM2 更新），则用 **T_w_b_optimized**；  
     - 否则退化为 **T_w_b**。  
   - 这样在「后端优化后再进行 HBA 优化」时，HBA 的起点就是后端结果，数值上更合理。

---

## 3. 这样对全局地图精度有什么影响？

### 3.1 理论层面

- HBA 的**约束集**不变（仍是 Prior + Between + GPS，无回环）时，最优解由因子图唯一决定，**理论最优值不变**。
- 「后端优化后再 HBA」且用后端结果做初始值，主要影响的是：  
  - **收敛速度**：初始更接近最优，LM 迭代更少、更稳。  
  - **数值表现**：减少陷入局部极小或收敛到次优的可能。  

因此：**同一套约束下，全局地图（由 HBA 输出位姿决定）的理论精度不变，实际精度不变或略有提升**（更好收敛、更少数值问题）。

### 3.2 当前两轨设计下的精度关系

- **显示/导出**：已使用 HBA 结果（`T_w_b_optimized` 等），全局地图精度由 **HBA 输出**决定。
- **后端 ISAM2**：单独维护一条轨迹，用于快速约束（回环、GPS）；HBA 完成后会尝试用 HBA 结果更新 ISAM2 的线性化点（`addSubMapNode(..., pose_w_anchor_optimized)`），但节点已存在时不会改写，因此两轨会有一致性差异（见日志中的 HBA-iSAM2 separation）。

若采用「后端优化后再 HBA」并用后端结果做 HBA 初始值：

- 对**全局地图精度**：不变或略有提升（同上）。  
- 对**两轨一致性**：HBA 起点更接近 ISAM2，HBA 结果与 ISAM2 的差异可能略小，但不会改变「显示用 HBA、因子图用 ISAM2」的架构。

### 3.3 小结

| 问题 | 结论 |
|------|------|
| 是否可以在后端优化后再进行 HBA？ | 可以；触发顺序上可严格「先后端再 HBA」，且建议 HBA 用后端的 `T_w_b_optimized` 做初始值。 |
| 对全局地图精度的影响？ | 约束不变则理论精度不变；实际中收敛更好，精度不变或略有提升。 |

---

## 4. 已实现行为

- **触发前**：`AutoMapSystem::ensureBackendCompletedAndFlushBeforeHBA()` 在每次 `triggerAsync` 前被调用（onSubmapFrozen、onGPSAligned、sensor_idle、finish_mapping、handleTriggerHBA），确保后端队列空且 pending 已 flush，释放 GTSAM 相关变量后再触发 HBA。
- **初始值**：`runGTSAMFallback()` 中按关键帧判断：若 `T_w_b_optimized` 与 `T_w_b` 不同（平移或旋转有差异），则用 `T_w_b_optimized` 填入 `initial` 并计算 Between 的 rel；否则用 `T_w_b`。
- **GPS**：HBA 中继续加入 GPS 约束（`task.enable_gps && gps_aligned_` 时），与 Prior、Between 一起优化。
