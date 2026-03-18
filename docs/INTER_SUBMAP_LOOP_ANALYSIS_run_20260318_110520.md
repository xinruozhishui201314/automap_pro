# 子图间回环深度分析：run_20260318_110520

## 0. 根本原因（代码 Bug，已修复）

**TeaserMatcher 在 TEASER 成功路径中从未将求解结果 `solution` 写入 `result.T_tgt_src`**，导致返回的位姿始终为 **Identity（平移 0）**。因此：

- 子图间候选有被检测到（INTER_KEYFRAME_OT 入队 17+12+4 个任务），TEASER 内部也解出了非零平移（如 `trans_norm_m=1.746`）；
- 但返回给 LoopDetector 的 `res.T_tgt_src` 恒为 Identity → `lc->delta_T.translation()` 为 0 → 后端 `trans_norm < min_trans_m` 全部判为 trivial，**无一入图**。

**修复**：在 `teaser_matcher.cpp` 的 TEASER 成功分支中，在通过 inlier 检查后、计算 RMSE 前，增加：

```cpp
result.T_tgt_src.linear() = solution.rotation;
result.T_tgt_src.translation() = solution.translation;
```

修复后，子图间回环应能带着正确的相对位姿进入后端（仍会经过 odom–TEASER 一致性检查与 trivial 过滤，但不会因“恒零平移”被误杀）。

---

## 1. 结论：子图间回环**未真正入图**，根因是「几何不一致 + trivial 过滤」+ **上述 Bug**

| 现象 | 说明 |
|------|------|
| **LOOP_ACCEPTED 中 sm_i≠sm_j** | 仅 **1 条**：`sm_i=0 sm_j=1 score=0.998` |
| **该条是否加入后端图** | **否**：随后被 `reason=trivial_trans` 过滤，未执行 addLoopFactor |
| **实际入图的子图间回环数** | **0**（所有入图回环均为 sm_i=sm_j 子图内） |

---

## 2. 日志证据链

### 2.1 子图与检索

- `created new submap sm_id=0/1/2/3`：共 4 个子图。
- `stage=retrieve_enter` 仅对 **submap_id=0** 和 **submap_id=1** 出现：
  - **submap_id=0**：`db_size=1`（只有自己），无法产生子图间候选。
  - **submap_id=1**：`db_size=2`，`raw_candidates=2`，`diff_submap=1`，产生子图间候选 (1→0)。
- **submap_id=2、3** 在本 log 中**没有**出现 `retrieve_enter`，即未做子图间检索（或 run 在它们 descriptor 就绪前结束）。

因此，**仅有子图 1 对子图 0 做了子图间检索**；子图 2、3 未参与子图间检索。

### 2.2 高置信度绕过与 INTER_KEYFRAME_OT

- `query_id=1 target_id=0 score=0.999 geo_dist=58.1m BYPASS (score>=0.92)`：子图间候选 (1→0) 因 `geo_prefilter_skip_above_score=0.92` 绕过几何预筛，进入后续流程。
- `stage=match_enqueue INTER_KEYFRAME_OT query_id=1 tasks=17`：子图 1 共入队 17 个关键帧级子图间 TEASER 任务。

说明：**配置与 OT 关键帧级子图间分支均生效**，候选能进入 TEASER。

### 2.3 唯一一条“子图间”通过 TEASER 的约束

```
[INTER_KF][GEOM_DIAG] after TEASER sm_j=1 kf_j=65 sm_i=0 kf_i=66 odom_rel_trans=20.662 teaser_trans=0.000 trans_diff_m=20.662 inlier_ratio=0.0870
[LOOP_ACCEPTED] loop_constraint published sm_i=0 sm_j=1 score=0.998
[LOOP_STEP] stage=onLoopDetected_skip reason=trivial_trans sm_i=0 sm_j=1 trans_norm=0.000m min_trans_m=0.02m
[AutoMapSystem][LOOP] skip trivial loop sm_i=0 sm_j=1 trans_norm=0.000m < 0.02m (quality filter)
```

含义：

- **odom**：两关键帧 (sm_i=0 kf_i=66 与 sm_j=1 kf_j=65) 在世界系下相距 **20.66 m**。
- **TEASER**：估计相对位姿的平移为 **0.000 m**（等价于单位变换）。
- **trans_norm**：来自 `lc->delta_T.translation().norm()`，即 TEASER 的平移量，为 **0**。
- **trivial 过滤**：`loop_closure.teaser.min_relative_translation_m=0.02`，`trans_norm(0) < 0.02` → 判定为 trivial，**不加入图**。

因此：**子图间回环检测到 1 条并发布，但因其几何不合理（见下），被正确拒绝入图**。

---

## 3. 根本原因分析

### 3.1 为何 TEASER 给出 0 平移？

- 两帧在**里程计/世界系**下相距约 **20 m**，并非同一位置。
- 描述子相似度高 (0.998)，检索阶段把这两帧当成“可能闭环”送入 TEASER。
- 点云层面：要么是**重复结构**（如长走廊、相似建筑），要么 FPFH 对应关系错误，TEASER 在错误对应下解出**接近单位阵**的相对位姿（平移≈0）。
- 因此这是**描述子层面的假阳性** + **TEASER 的退化解**，不是“真闭环却被误杀”。

### 3.2 为何子图 2、3 没有子图间检索？

- 日志中未出现 `stage=retrieve_enter submap_id=2` 或 `submap_id=3`。
- 可能原因：  
  1）run 在子图 2/3 的 `onDescriptorReady` 触发前结束；  
  2）或 descriptor 计算/入队较晚，检索尚未执行到。  
- 结果是：**只有子图 1 对子图 0 做了子图间检索与 INTER_KEYFRAME_OT**，子图 2、3 未参与子图间闭环。

### 3.3 本 run 中子图间“机会”与“发布/过滤”统计

| 项目 | 数值 |
|------|------|
| 子图间检索机会 | query_id=1 时 db_size=2（1→0）；query_id=2 时 db_size=3（2→0,1）；query_id=4 时 db_size=4（4→0,1,2,3） |
| INTER_KEYFRAME_OT 入队任务数 | 17（query 1）+ 12（query 2）+ 4（query 4）= 33 |
| 到达 onLoopDetected 的子图间约束（published） | 至少 11 条（由 trivial_trans skip 条数统计：0↔1 共 1 条，1↔2 共 6 条，0↔2 共 2 条，1↔4 共 2 条） |
| 全部子图间约束的 trans_norm | **均为 0.000 m**（因 TeaserMatcher Bug 未把 solution 写入 result） |
| 实际入图子图间回环 | **0**（全部被 trivial 过滤） |

因此：**子图间存在大量回环“机会”和候选，没有产生大量入图回环的直接原因是 Bug 导致位姿恒为零，进而被 trivial 过滤；修复 Bug 后需结合 odom–TEASER 一致性检查与 inlier/rmse 阈值再观察**。

### 3.4 trivial 过滤在做什么？

- 代码位置：`automap_system.cpp` → `onLoopDetected()`，使用 `loop_closure.teaser.min_relative_translation_m`（当前 0.02 m）。
- 目的：**过滤“相对平移过小”的回环**，避免对图几乎没有约束的边加入后端。
- 本条约束 `trans_norm=0`，远小于 0.02，被判为 trivial 是**符合设计**的；若不加此过滤，会加入一条**平移为 0 的错误约束**，反而破坏图。

---

## 4. 根因归纳

| 层级 | 根因 |
|------|------|
| **现象** | 子图间回环“只看到 1 条 LOOP_ACCEPTED，且未入图” |
| **直接原因** | 该条约束 `trans_norm=0`，被 trivial 过滤拒绝 |
| **几何原因** | TEASER 估计平移≈0，而 odom 显示两关键帧相距 20 m → **几何不一致，属假阳性** |
| **检索原因** | 描述子将“非同一位置”的两帧判为相似（重复结构/外观相似），形成误匹配 |
| **机会原因** | 本 run 中仅子图 1 对子图 0 做了子图间检索；子图 2、3 未做子图间检索 |

---

## 5. 建议（配置 + 代码）

### 5.1 配置

- **保持** `min_relative_translation_m: 0.02`，用于过滤平移过小的错误约束。
- 若希望**更早拒绝**“odom 很远、TEASER 很近”的明显不一致，可依赖下面代码侧的**几何一致性检查**。

### 5.2 代码：子图间几何一致性检查（已实现见下）

在 **INTER_KF 分支**中，TEASER 通过后、`publishLoopConstraint(lc)` 之前：

- 若 **dist_world_m > 阈值（如 5 m）** 且 **teaser_trans_m < 阈值（如 1 m）**：
  - 认为 odom 与 TEASER 严重不一致，视为**假阳性**，**不发布**该约束（也不再触发 onLoopDetected）。
- 效果：减少“先发布再被 trivial 过滤”的无效回环，日志更干净，且避免后端收到明显错误的约束。

这样既保留 trivial 过滤作为最后一道防线，又在回环检测侧提前拦截几何不一致的子图间匹配。
