# 回环检测日志分析：run_20260317_171439

## 0. Executive Summary

| 维度 | 结论 | 说明 |
|------|------|------|
| **子图内回环 (Intra)** | **未产生约束，属预期** | 描述子准备与批量检测流程正常；本序列为单向前行、子图内无重访，故无满足 gap/overlap 的候选对。 |
| **子图间回环 (Inter)** | **未产生约束，主因配置+数据** | 检索与 gap 过滤逻辑正常；sm0 无历史、sm1/sm2 因 `min_submap_gap=2` 全部被过滤，仅 sm3 有 5 个有效候选进入 TEASER++，未在日志中看到约束加入后端（可能被 TEASER 拒绝或时序未到）。 |
| **整体** | **管道正常，约束数为 0 可解释** | 若需在 street_03 等“无真实闭环”数据上验证回环链路，建议：放宽 `min_submap_gap` 或使用有闭环的 bag。 |

---

## 1. 运行与配置摘要

- **日志**: `logs/run_20260317_171439/full.log`
- **数据**: 离线 bag `M2DGR/street_03_ros2`
- **回环配置**（从日志摘录）:
  - `intra_submap_enabled: true`，`min_keyframe_gap: 10`，`min_distance_gap_m: 3.0`，`min_temporal_gap_s: 3.0`
  - `inter`: ScanContext，`min_submap_gap: 2`（配置 1，实际用 2），`dist_threshold: 0.18`，TEASER 开启
- **运行结果**: 全程 `loop=0`（BACKEND 状态中回环约束数始终为 0），kf 约 326，子图 4 个（sm 0~3）。

---

## 2. 子图内回环 (Intra-Submap)

### 2.1 设计（与日志一致）

- **触发**: 每帧关键帧在 backend 做一次 intra 检查；子图冻结时对**该子图内所有关键帧**做一次批量检测。
- **描述子**: 子图冻结时 `prepareIntraSubmapDescriptors` 用 **ScanContext** 为该子图内所有 KF 生成描述子（无 OT 时仅用 ScanContext）。

### 2.2 日志证据

- **实时路径**（每 KF）: 大量 `[INTRA_LOOP][TRIGGER] submap_id=0 kf_idx=*`，随后 `intra_loop_exit duration_ms=0.0`，说明实时只做轻量检查（当时子图未冻结、无完整描述子，不产生约束属预期）。
- **冻结时**（以 sm0 为例）:
  - `[INTRA_LOOP][PREPARE_SUBMAP] submap_id=0 keyframes=78`
  - `[INTRA_LOOP][PREPARE] ====== START ====== ... use_scancontext=true`
  - 78 个 KF 的 ScanContext 均成功: `success=78 null_kf=0 null_cloud=0`，`sc_db_size=142`（存在重复/多线程日志，数值略有不一致，但流程完整）
  - `[INTRA_LOOP][PREPARE] ====== DONE ====== submap_id=0 total_kf=78 success=78 ...`
  - `[INTRA_LOOP][FROZEN_DETECT] submap_id=0 checking all 78 keyframes for intra-submap loops`
  - `[INTRA_LOOP][FROZEN_DONE] submap_id=0 intra-submap loop detection completed`
- **关键**: 整段日志中**从未出现** `[INTRA_LOOP][FROZEN_RESULT] ... detected=* loops`，即对每个 `query_idx` 调用 `detectIntraSubmapLoop` 均返回 0 条约束。

### 2.3 结论（子图内）

- **流程正常**: PREPARE → FROZEN_DETECT（遍历所有 KF）→ FROZEN_DONE 均按设计执行。
- **无约束原因**: street_03 为单向前行、子图内无轨迹重访；且满足 `min_keyframe_gap=10`、`min_distance_gap_m=3.0`、`min_temporal_gap_s=3.0` 以及 overlap 的候选对本就很少甚至没有，故未产生子图内回环约束属**预期**。

---

## 3. 子图间回环 (Inter-Submap)

### 3.1 流程概览

1. 子图冻结 → 入队 → desc worker 生成子图级描述子（本 run 为 ScanContext）→ `onDescriptorReady`
2. `addToDatabase` → 检索候选（ScanContext KD-Tree）→ gap 过滤（`min_submap_gap`）→ 几何预筛（若开启）→ 有效候选入 match 队列 → TEASER++ 匹配 → 发布 `loop_constraint` → 后端 `addLoopConstraint`，`loop` 计数增加。

### 3.2 各子图检索与 gap 过滤结果

| 子图 | 检索结果 | sc_size（检索时） | gap 过滤 | valid | 说明 |
|------|----------|-------------------|----------|-------|------|
| sm0 | NO_CAND | 0 | - | 0 | 首子图，ScanContext 历史为空，“Not enough history frames for search: available=0 < 1” |
| sm1 | OK, raw_candidates=5 | 1 | **ALL_FILTERED** (gap≤2) | 0 | CAND_STATS: same_session=5, diff_submap=0, filtered_by_gap=5 |
| sm2 | OK, raw_candidates=5 | 2 | **ALL_FILTERED** (gap≤2) | 0 | 同上，5 个候选均为相邻子图，gap 不满足 >2 |
| sm3 | OK, raw_candidates=5 | 3 | **PASS** | **5** | diff_submap=5, filtered_by_gap=0，5 个候选进入后续 TEASER++ |

- **sm0**:  
  `[LOOP_STEP] stage=retrieve_result NO_CAND ... Not enough history frames for search: available=0 < 1`  
  符合“第一个子图无历史”的设计。

- **sm1 / sm2**:  
  `[LoopDetector][CAND_STATS] query_id=1: candidates=5 ... filtered_by_gap=5 valid=0`  
  `[LOOP_STEP] stage=gap_filter ALL_FILTERED ... min_submap_gap=2`  
  配置中 `min_submap_gap=2` 表示要求 `|query_sm - cand_sm| > 2`，即至少间隔 2 个子图。sm1 的候选只有 sm0（gap=1），sm2 的候选为 sm0/sm1（gap=2,1），均被过滤，故**子图间约束为 0 主要来自此处**。

- **sm3**:  
  `[LoopDetector][CAND_STATS] query_id=3: ... diff_submap=5 filtered_by_gap=0 valid=5`  
  随后出现 `[LoopDetector][CAND] query_id=3 ... candidates=5 scores=[0.330, ...] → TEASER++`，说明 5 个有效候选已送入 TEASER++。

### 3.3 TEASER 之后为何仍为 loop=0

- 日志中**没有**出现“回环约束已加入后端”或 `loop=1` 的 BACKEND 状态；仅有 sm3 的“5 候选 → TEASER++”一条明确进入几何验证的记载。
- 合理推断二者至少其一成立：
  1. **TEASER 全部拒绝**: street_03 若实际无闭环，sm3 与历史子图的点云匹配可能 inlier_ratio/rmse 等不通过，导致未发布或未接受约束。
  2. **时序**: 约束在 match_worker 中异步处理，若 bag 结束或进程在 TEASER/发布/后端回调之前结束，则不会看到 `loop` 增加。

因此：**子图间管道（检索 → gap 过滤 → TEASER）在逻辑上正常**；本 run 中真正有机会产生 inter 约束的只有 sm3，且最终未在日志中体现为“已加入图”，与“无真实闭环或几何不通过”一致。

---

## 4. 配置与数据对“无回环约束”的影响

- **min_submap_gap=2**  
  - 相邻子图（0–1、1–2、2–3）一律不会成为 inter 候选，只有“隔两格”以上（如 sm3 对 sm0）才会通过 gap 过滤。  
  - 本 run 仅 4 个子图，故只有 sm3 有机会得到有效候选（对 sm0 等），与日志一致。

- **street_03 轨迹**  
  - 若为单程、无回到起点的闭环，则：  
    - 子图内几乎不会有满足时间/距离/overlap 的重复位置对；  
    - 子图间即使 ScanContext 给出若干候选（相似场景），TEASER 也可能因几何不一致而全部拒绝。

---

## 5. 结论与建议

### 5.1 结论

- **子图内回环**: 流程正常（PREPARE + FROZEN_DETECT + FROZEN_DONE），未产生约束是因为数据与阈值下无满足条件的候选，属预期。
- **子图间回环**: 检索与 gap 过滤按配置工作；sm0 无历史、sm1/sm2 被 `min_submap_gap=2` 全部过滤，sm3 有 5 个有效候选进入 TEASER++，但未在日志中看到约束加入图，与“无真实闭环或 TEASER 拒绝”相符。
- **整体**: 本 run 中**子图内与子图间回环逻辑均正常**，当前“0 条回环约束”可由配置（min_submap_gap）与数据（无闭环/几何不通过）解释。

### 5.2 建议

1. **若希望在本数据集上看到 inter 约束（仅作管道验证）**  
   - 可将 `loop_closure.min_submap_gap` 调为 `1`，使相邻子图也能成为候选（注意可能增加误匹配，仅建议短期验证）。

2. **若希望验证“真实闭环”效果**  
   - 使用有明确回到起点或重复路段的 bag，再观察 `[INTRA_LOOP][FROZEN_RESULT]` 与 inter 的 `retrieve_result OK` + `valid>0` 以及 BACKEND 的 `loop=N` 增加。

3. **可观测性增强（可选）**  
   - 在 TEASER 接受/拒绝时打 INFO（如 inlier_ratio、rmse、query/target submap id）；  
   - 在 backend 收到并加入回环约束时打一条含 `loop_count` 的日志，便于以后类似 run 的排查。

---

## 6. 关键日志位置速查

- 回环配置: `loop_closure.*`、`[LoopDetector][CONFIG]`、`[LOOP_DESIGN]`
- 子图内: `INTRA_LOOP][PREPARE]`、`INTRA_LOOP][FROZEN_DETECT]`、`INTRA_LOOP][FROZEN_DONE]`、`INTRA_LOOP][FROZEN_RESULT]`
- 子图间: `LOOP_STEP] stage=addSubmap`、`stage=retrieve_result`、`[LoopDetector][CAND_STATS]`、`stage=gap_filter`、`→ TEASER++`
- 后端状态: `[AutoMapSystem][BACKEND] state=MAPPING ... loop=*`
