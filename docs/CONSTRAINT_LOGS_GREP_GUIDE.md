# 约束日志与快速定位指南

## 0. 目的

保证**先验、里程计、子图内/间回环、GPS**等约束正确添加，并通过统一标签 **`[CONSTRAINT]`** 与 **`[BACKEND_STEP]`** 在出问题时**快速精准定位**到具体约束类型与结果（ok/skip/defer）。

---

## 1. 日志标签说明

| 标签 | 含义 | 典型内容 |
|------|------|----------|
| **`[CONSTRAINT]`** | 约束汇总，每条对应一次“尝试添加约束”的结果 | `step=odom from=1 to=2 result=ok` |
| **`[BACKEND_STEP]`** | 后端步骤流水，便于按时间线排查 | `step=addOdomFactor_enter from=1 to=2` |

- **step**：约束或 API 步骤名（见下表）。
- **result**：`ok` 已加入图；`skip` 未加入（原因见 reason）；`defer` 进入 pending 稍后 flush。
- **reason**：仅当 result=skip/defer 时出现，如 `node_not_exists`、`rel_non_finite`。

---

## 2. 约束类型与 step 一览

| 约束类型 | step 名称 | 调用位置 | 说明 |
|----------|-----------|----------|------|
| 子图节点 + 先验 | `submap_node` | onSubmapFrozen | 子图冻结时添加子图节点，首节点带 Prior |
| 子图间里程计 | `odom` | onSubmapFrozen | 相邻子图间 Between 因子 |
| 子图内回环 | `loop_intra` | 关键帧路径 / 冻结时 INTRA_LOOP | addLoopFactorDeferred，批量 commit |
| 子图间回环 | `loop_inter` | onLoopDetected → addLoopFactor | 跨子图回环，立即或异步 commit |
| 子图级 GPS | `gps_submap` | tryCreateKeyFrame / 对齐回调等 | addGPSFactor(sm_id) |
| 关键帧先验 | `prior_kf` | addKeyFrameNode | 首帧或新子图首帧 Prior |
| 关键帧间里程计 | `between_kf` | addKeyFrameNode | 相邻 KF 间 Between |
| 关键帧级 GPS | `gps_kf` | tryCreateKeyFrame | addGPSFactorForKeyFrame(kf_id) |

---

## 3. 常用 grep 命令（快速定位）

```bash
# 所有约束结果（ok/skip/defer）
grep '\[CONSTRAINT\]' full.log

# 只看未加入的（skip/defer）
grep '\[CONSTRAINT\]' full.log | grep -E 'result=skip|result=defer'

# 按约束类型
grep '\[CONSTRAINT\] step=odom' full.log
grep '\[CONSTRAINT\] step=loop_intra' full.log
grep '\[CONSTRAINT\] step=loop_inter' full.log
grep '\[CONSTRAINT\] step=gps_submap' full.log
grep '\[CONSTRAINT\] step=gps_kf' full.log
grep '\[CONSTRAINT\] step=prior_kf' full.log
grep '\[CONSTRAINT\] step=between_kf' full.log
grep '\[CONSTRAINT\] step=submap_node' full.log

# 后端步骤流水（精确到 API 入口/跳过/成功）
grep '\[BACKEND_STEP\]' full.log

# 某类约束的完整链路（例如子图间里程计）
grep -E '\[BACKEND_STEP\].*addOdomFactor|\[CONSTRAINT\].*step=odom' full.log
```

---

## 4. 典型问题与对应 grep

| 现象 | 建议 grep | 关注点 |
|------|------------|--------|
| 子图节点/先验没加上 | `CONSTRAINT.*submap_node`、`BACKEND_STEP.*addSubMapNode` | result=ok、has_prior=1 |
| 子图间没有里程计 | `CONSTRAINT.*step=odom`、`BACKEND_STEP.*addOdomFactor` | result=skip 时 reason |
| 子图内回环未生效 | `CONSTRAINT.*loop_intra`、`BACKEND_STEP.*addLoopFactorDeferred` | result=skip 与 reason |
| 子图间回环未生效 | `CONSTRAINT.*loop_inter`、`BACKEND_STEP.*addLoopFactor` | result=skip、node_not_in_graph |
| GPS 约束大量 defer | `CONSTRAINT.*gps_submap`、`CONSTRAINT.*gps_kf` | result=defer 与 reason |
| 关键帧链缺 Between | `CONSTRAINT.*between_kf`、`CONSTRAINT.*prior_kf` | result=ok 是否成对出现 |
| 崩溃/卡住前最后几步 | `BACKEND_STEP`、`CRASH_CONTEXT` | 最后几条 step= 确定卡在哪个 API |

---

## 5. 日志输出位置（代码）

- **IncrementalOptimizer**（`incremental_optimizer.cpp`）：所有 `addSubMapNode`、`addOdomFactor`、`addLoopFactor`、`addLoopFactorDeferred`、`addGPSFactor`、`addKeyFrameNode`、`addGPSFactorForKeyFrame` 内，通过 `CONSTRAINT_LOG` / `BACKEND_STEP` 输出。
- **AutoMapSystem**（`automap_system.cpp`）：`onSubmapFrozen`、`tryCreateKeyFrame`、`onLoopDetected` 及子图内回环路径中，在**调用**优化器前后打 `[CONSTRAINT] step=*_enter`，便于区分“调用意图”与“优化器结果”。

---

## 6. 校验清单（出问题时逐项看）

1. **子图节点**：每个冻结子图有一条 `step=submap_node result=ok`，且首子图对应 `has_prior=1`。
2. **子图间里程计**：从第二个冻结子图起，每条应有 `step=odom from=A to=B result=ok`；若 result=skip，查 reason。
3. **子图内回环**：有 INTRA_LOOP 时应有 `step=loop_intra result=ok` 或合理 skip（如 same_node）。
4. **子图间回环**：每次检测到回环应有 `step=loop_inter_enter`，随后优化器内 `step=loop_inter result=ok` 或 skip。
5. **GPS**：有 GPS 且对齐后，应有 `gps_submap` 或 `gps_kf` 的 result=ok 或 result=defer（defer 后需在 flush 后看到加入）。
6. **关键帧链**：首帧或新子图首帧应有 `step=prior_kf result=ok`；后续 KF 应有 `step=between_kf result=ok`（或合理 skip）。

按上述 grep 与清单即可在日志中快速定位“哪种约束在何时未加入或异常”。
