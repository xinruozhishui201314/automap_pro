# GPS 对齐前后位姿跳变根因分析

**分析深度**: L4 跨模块 | **日志**: full.log run_20260323_080656 | **状态**: 已修复 (v2)

---

## 1. 现象

GPS 对齐前后出现位姿跳变，RViz 中实时点云与优化轨迹不同步。

---

## 2. 证据链（5-Why）

### 2.1 Symptom（现象）

- **Log 8726**: `GPS alignment SUCCESS: rmse=0.142m matched=31 pts R_z_deg=-17.81 t=(0.14,-0.14,-0.03)`
- **Log 8720**: `[V3][VisualizationModule] Applied GPS alignment correction to real-time cloud`
- 用户反馈：GPS 对齐前后出现位姿跳变

### 2.2 Why-1: 实时点云与轨迹不同步

- **Log 8720** 显示 VisualizationModule 在 GPS 对齐成功时**立即**将 `current_correction_` 设为 (R,t)
- 下一帧 SyncedFrameEvent 时，实时点云会经 `T_map_odom = current_correction_` 变换到 map 系
- 优化轨迹 (optimized_path) 和关键帧位姿来自 MapRegistry，由 OptimizationResultEvent → onPoseOptimized 更新

### 2.3 Why-2: 坐标系转换时序错位

- **VisualizationModule** (visualization_module.h): 收到 GPSAlignedEvent 后**立即**设置 `current_correction_`（旧实现）
- **OptimizationResultEvent** 的发布时间：OptimizerModule 的 pose callback → MapRegistry.updatePoses → event_bus 发布
- 时序：GPSAlignedEvent 早于「ISAM2 完成 addBatchGPSFactors / forceUpdate」后的 OptimizationResultEvent

### 2.4 Why-3: 两路数据源使用不同触发

| 数据源           | 坐标系转换触发              | 实际更新时刻                     |
|------------------|-----------------------------|----------------------------------|
| 实时点云         | GPSAlignedEvent             | 对齐成功瞬间                     |
| 优化轨迹/关键帧  | OptimizationResultEvent     | ISAM2 forceUpdate 完成后         |

两路触发不同步 → 在 GPS 对齐瞬间，点云已进入 map 系，轨迹仍停留在 odom 系 → 视觉跳变。

### 2.5 Root Cause（根因）

**VisualizationModule 在 GPSAlignedEvent 上立即应用 `current_correction_`**，导致实时点云先于优化轨迹完成坐标系转换，产生 frame 不一致的位姿跳变。

---

## 3. 证据定位

| 层级 | 内容 | 文件:行 |
|------|------|---------|
| 根因 | VisualizationModule 在 GPSAlignedEvent 时立即设置 current_correction_（旧实现） | visualization_module.h |
| 中间 | SyncedFrameEvent 用 current_correction_ 变换点云 | visualization_module.h:37-41 |
| 中间 | 轨迹由 OptimizationResultEvent 触发刷新 | visualization_module.h:70-72 |
| 症状 | Log 显示 "Applied GPS alignment correction" 与 GPS alignment SUCCESS 同帧 | full.log:8720,8726 |

---

## 4. 修复方案（已实施）

**策略**: 延迟应用 GPS 修正，直到收到 OptimizationResultEvent 且轨迹已实质更新。

**实现** (visualization_module.h):

1. **GPSAlignedEvent**: 缓存到 `pending_correction_R_` / `pending_correction_t_`，置 `gps_correction_pending_ = true`
2. **OptimizationResultEvent**: 若 `gps_correction_pending_` 且 **trajectory_updated**，则应用 pending 到 `current_correction_`

**trajectory_updated 检查**（覆盖首子图 Prior(identity) 等边界）：
- 任一 submap `pose.translation().norm() > 0.05`，或
- 任一 keyframe `pose.translation().norm() > 0.5`，或
- `submap_poses.size() > 1`

---

## 5. 增强日志（grep GPS_ALIGN）

| 标签 | 含义 |
|------|------|
| `[V3][VizModule][GPS_ALIGN] GPS aligned rmse=...` | GPS 对齐成功，修正量已缓存 |
| `[V3][VizModule][GPS_ALIGN] Applied GPS correction` | 在 trajectory 更新后安全应用 |
| `[V3][VizModule][GPS_ALIGN] PENDING_STUCK` | 连续 ≥3 次 OptimizationResult 仍未应用 → 排查 trajectory_updated 条件 |

详见 `automap_pro/docs/LOGGING_AND_DIAGNOSIS.md` 10.6 节。

---

## 6. 验证

- 复现：使用同一 bag（M2DGR street_03_ros2）与配置
- 期望：GPS 对齐瞬间不再出现点云与轨迹的明显跳变
- 日志：`[V3][VizModule][GPS_ALIGN] Applied GPS correction` 应出现在 `GPS alignment SUCCESS` 之后

---

## 7. V2 根因（run_20260323_080656 仍跳变）

### 7.1 证据链

| 层级 | 内容 | 文件:行/日志 |
|------|------|--------------|
| 症状 | kf_count 59→60 时 last_pos 从 [12.95,-13.99] 跳到 [8.39,-17.73] | full.log:8735,9294 |
| Why-1 | 点云已用 current_correction_ 变换到 map，轨迹未变换 | visualization_module.h:36-45 |
| Why-2 | publishOptimizedPath 直接使用 kf->T_w_b_optimized（odom 系） | rviz_publisher.cpp:244 |
| Why-3 | V3 无 addBatchGPSFactors，ISAM2 未加 GPS 因子，轨迹始终 odom 系 | mapping_module.cpp:247-264 |
| 根因 | 轨迹发布时未应用 T_odom_to_map，点云与轨迹坐标系不一致 | rviz_publisher + visualization_module |

### 7.2 修复（v2）

- **RvizPublisher**：`publishOptimizedPath`/`publishKeyframePoses` 增加可选参数 `T_odom_to_map`，发布前将位姿变换到 map 系
- **VisualizationModule**：`publishEverything` 将 `current_correction_` 传入，使轨迹与点云同坐标系

---

## 8. V3 addBatchGPSFactors 缺失与子图冻结时序（根因分析）

**分析深度**: L4 跨模块 | **方法**: 5-Why + Root-Cause-Investigator

### 8.1 问题一：V3 缺少 addBatchGPSFactors

#### 证据链（5-Why）

| 层级 | 内容 | 文件/证据 |
|------|------|-----------|
| 症状 | 轨迹始终在 odom 系，即使应用 T_odom_to_map 于可视化，后端位姿仍为 odom | ROOT_CAUSE §7.1 |
| Why-1 | ISAM2 图内无历史关键帧 GPS 约束，优化结果不向 map 系收敛 | optimizer_module.cpp:110-131 |
| Why-2 | processKeyframeCreate 仅为**新创建**的关键帧添加 GPS 因子，历史 KF 无 | optimizer_module.cpp:127-129 |
| Why-3 | GPSAlignedEvent 触发时，MappingModule 仅 freezeSubmap + 设置 gps_aligned_，未向 Optimizer 投递批量 GPS 任务 | mapping_module.cpp:246-266 |
| 根因 | V3 未实现 addBatchGPSFactors 等价逻辑：GPS 对齐后未批量向 ISAM2 添加历史 KF 的 GPS 因子 | mapping_module + optimizer_module |

#### 最小修复（已实施）

1. **新增 OptTaskItem::Type::GPS_BATCH_KF**（opt_task_types.h）
2. **MappingModule.updateGPSAlignment**：在 freezeSubmap 之后，若 `gps.add_constraints_on_align=true`，发布 GraphTaskEvent(GPS_BATCH_KF, R_enu_to_map, t_enu_to_map)
3. **OptimizerModule.processGPSBatchKF**：从 MapRegistry 收集 `has_valid_gps` 的 KF，构建 `pos_map = R*position_enu + t`，调用 `addGPSFactorsForKeyFramesBatch`（内部已含 commitAndUpdate）
4. **processKeyframeCreate 修正**：`pos_map = task.gps_transform_R * kf->gps.position_enu + task.gps_transform_t`（原误用 position_enu 直接作 map 系）

### 8.2 问题二：子图冻结时序与 trajectory_updated 检查

#### 证据链

| 层级 | 内容 | 证据 |
|------|------|------|
| 现象 | GPS 对齐触发 freezeSubmap，先产生 submap 冻结相关 OptimizationResult | mapping_module.cpp:262-264 |
| Why-1 | freezeSubmap → onSubmapFrozen → 发布 SUBMAP_NODE + ODOM_FACTOR + FORCE_UPDATE | mapping_module.cpp:194-230 |
| Why-2 | Optimizer 串行处理，FORCE_UPDATE 触发 commitAndUpdate → OptimizationResultEvent | optimizer_module.cpp |
| 结论 | 时序为：GPSAlignedEvent → freezeSubmap（投递 SUBMAP/ODOM/FORCE）→ 随后投递 GPS_BATCH_KF。首轮 OptimizationResult 来自 freeze 的 FORCE_UPDATE，包含新冻结子图位姿 |

#### trajectory_updated 检查是否覆盖

- `submap_poses.size() > 1`：freeze 后至少 2 个子图（旧活跃 + 新冻结）→ **通过**
- `sm_trans > 0.05`：新冻结子图有非零位姿（来自 KF 累积）→ 通常通过
- **结论**：现有「延迟应用 + trajectory_updated 检查」逻辑可正确处理 GPS 对齐触发的 freeze 时序，无需修改
