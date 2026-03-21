# Minimal Mergeable PR: Pose Writeback Safety (Phase 1-5)

本方案是一个“可立即合并”的最小版本，目标是先建立**写回协议与并发安全护栏**，避免 GPS 对齐、回环、iSAM2 写回交叉时再次出现严重位姿错乱。

## 目标

- 单一写回协议（transaction）
- GPS 对齐期间写回屏障（barrier）
- 同轮写回互斥（子图增量 vs 关键帧绝对）
- 最小侵入实现（不改核心优化算法）
- 可回放验证（日志指标 + grep 脚本）

---

## Phase 1: 事务模型（已完成）

### 改动

- 新增 `automap_pro/include/automap_pro/system/pose_update_transaction.h`
  - `PoseWriteMode`:
    - `SUBMAP_DELTA_PROPAGATE`
    - `KEYFRAME_ABSOLUTE_SET`
    - `GLOBAL_RIGID_TRANSFORM`
  - `PoseFrameSemantics`:
    - `MAP_LOCAL`
    - `ENU_GLOBAL`
  - `PoseUpdateSource`:
    - `ISAM2_FORCE_UPDATE`
    - `GPS_ALIGN`
    - `HBA_WRITEBACK`
    - `LOOP_OPTIMIZATION`
  - `PoseUpdateTransaction`
- `automap_system.h` 接入事务头文件与版本计数器

### 验收

- 存在统一事务结构；
- 后续所有写回日志可带 `tx_version/mode/frame/source`。

---

## Phase 2: onPoseUpdated 统一入口化（已完成）

### 改动

- 在 `loop_optimization.cpp::onPoseUpdated` 增加：
  - 事务构建（version/mode/frame/source）
  - `POSE_TX` begin/end 结构化日志
  - barrier 激活时 defer 入队
- 新增 `flushDeferredPoseUpdates()` 用于屏障结束后回放 deferred 更新

### 验收

- 日志包含：
  - `[POSE_TX][APPLY_BEGIN]`
  - `[POSE_TX][APPLY_END]`
  - `[POSE_TX][DEFER]`
  - `[POSE_TX][FLUSH]`

---

## Phase 3: GPS 对齐屏障（已完成）

### 改动

- `worker_threads.cpp::gpsAlignWorkerLoop`
  - GPS 对齐开始时：
    - `pose_update_barrier_active_=true`
    - 日志 `[POSE_TX][BARRIER] activated`
  - 入队失败时：
    - 关闭 barrier + flush deferred
- `optWorkerLoop` 的 `GPS_ALIGN_COMPLETE` 分支：
  - 重建完成后关闭 barrier
  - flush deferred
  - 日志 `[POSE_TX][BARRIER] deactivated`

### 验收

- GPS 对齐期间普通 `onPoseUpdated` 被 defer，不再与重建并发写回。

---

## Phase 4: 写回互斥不变式（已完成）

### 改动

- `loop_optimization.cpp::onPoseUpdated`
  - 预先计算本轮 `sm_with_kf_updates`
  - 若某 `sm_id` 在同轮存在关键帧绝对写回，则**跳过** `updateSubmapPose(sm_id, ...)`
  - 日志：
    - `[POSE_TX][INVARIANT] skip submap delta update ... reason=same_tx_has_kf_absolute_set`

### 验收

- 同一轮不会再对同子图同时执行：
  - “关键帧绝对赋值”
  - “子图增量传播”

---

## Phase 5: 可回放验证（已完成）

### 验证要点（grep）

- 事务与屏障
  - `POSE_TX][APPLY_BEGIN`
  - `POSE_TX][APPLY_END`
  - `POSE_TX][DEFER`
  - `POSE_TX][FLUSH`
  - `POSE_TX][BARRIER`
- 不变式触发
  - `POSE_TX][INVARIANT`
- 旧风险信号
  - `POSE_JUMP`
  - `GPS_TRANSFORM`
  - `LOOP][POSE_DIAG`

### 通过标准

1. GPS 对齐窗口内存在 `BARRIER activated` 到 `deactivated` 成对日志；
2. 屏障期间 `onPoseUpdated` 进入 defer（有则合理）；
3. 屏障结束后 flush 成功并恢复 apply；
4. 同轮写回冲突时出现 invariant skip，而不是重复改写。

---

## 迁移与后续（非本 PR 必须）

- 已完成：`deferred_pose_updates_` 升级为 `std::deque<PoseUpdateTransaction>`，defer/flush 保留原始 `tx_version/source/mode/frame`；
- 已完成：HBA 写回封装为 `PoseUpdateTransaction(source=HBA_WRITEBACK)`，并统一经 `applyPoseTransaction()` 执行；
- 下一步：把更多写回路径（如纯 GPS 转换后的局部补偿路径）也逐步纳入同一入口；
- 下一步：把 frame 语义升级为更强类型（不仅日志标签）。

