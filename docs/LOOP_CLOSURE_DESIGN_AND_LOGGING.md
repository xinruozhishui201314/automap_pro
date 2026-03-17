# 回环检测设计说明与增强日志

## 1. 您关心的几个问题（直接回答）

### 1.1 子图内的回环有检测么？

**有。** 子图内回环是 **关键帧级别** 的：

- **触发时机**：每来一个关键帧会触发一次 `detectIntraSubmapLoop(active_submap, query_keyframe_idx)`；子图冻结时会对该子图内**所有历史关键帧**再跑一遍子图内检测（弥补冻结前描述子未就绪的帧）。
- **粒度**：当前帧（query keyframe）与**同子图内**历史关键帧（cand keyframe）做 ScanContext 相似度 + TEASER 几何验证，通过则产生一条 **keyframe_i ↔ keyframe_j** 的约束（同 submap_id）。
- **约束入图**：子图内回环用 `addLoopFactorDeferred(node_i, node_j, ...)`，节点 ID = `submap_id * MAX_KF_PER_SUBMAP + keyframe_index`，即关键帧级节点。

### 1.2 子图间有检测「子图 0 与 2、3、4、5」的回环么？子图间如何用关键帧级？

**当前支持两种模式（由配置 `loop_closure.inter_keyframe_level` 控制）：**

1. **子图间关键帧级（inter_keyframe_level: true，默认）**  
   - 子图间做 **关键帧 ↔ 关键帧** 的检索与匹配：  
     - 每个子图在加入 DB 时会 **prepareIntraSubmapDescriptors**，得到每个关键帧的 ScanContext 与下采样点云。  
     - 当新子图就绪后，对其**采样关键帧**（每隔 `inter_keyframe_sample_step` 取 1 个）做检索：在**每个候选子图**内用 ScanContext 距离找 **top-K 关键帧**（`inter_keyframe_top_k_per_submap`），对每对 (query_kf, target_kf) 做 TEASER，通过则产生一条 **(submap_i, keyframe_i) ↔ (submap_j, keyframe_j)** 的约束。  
   - 约束入图：使用关键帧节点 ID（`node_id = submap_id * MAX_KF_PER_SUBMAP + keyframe_idx`），先 **addSubMapNode(node_i/node_j)** 再 **addLoopFactor(node_i, node_j, ...)**。  
   - 配置项：`inter_keyframe_level`、`inter_keyframe_sample_step`、`inter_keyframe_top_k_per_submap`（见 `system_config.yaml` / `system_config_M2DGR.yaml`）。

2. **子图级（inter_keyframe_level: false）**  
   - 每个子图 1 个描述子（`downsampled_cloud`），只产生 **子图对 (sm_i, sm_j)** 的一条约束；  
   - min_submap_gap 等过滤仍适用（例如子图 1、2 的候选可能被 gap 滤掉，只有 3、4、5 与 0 会进入 TEASER）。

### 1.3 回环检测是关键帧级别的么？

- **子图内**：是，关键帧 ↔ 关键帧（同子图内）。  
- **子图间**：否，是 **子图 ↔ 子图**（每个子图 1 个描述子、1 条约束 per 子图对）。  
  所以「理论上有很多回环」若指「很多关键帧对」，当前实现只有：  
  - 子图内：可能多对（关键帧级）；  
  - 子图间：每个 (query_submap, target_submap) 至多 1 条约束。

### 1.4 为什么理论上应该有特别多回环却都检测不出来？

可能原因归纳：

| 层面 | 说明 |
|------|------|
| **子图内** | 被 **时间/索引/距离/描述子** 过滤掉：`min_temporal_gap`、`min_keyframe_gap`、`min_distance_gap`、ScanContext 相似度阈值；或 TEASER 内点/rmse 不达标。 |
| **子图间** | ① 第一个子图无历史，检索直接 NO_CAND；② **min_submap_gap** 导致子图 1、2 的候选全部被滤掉；③ 子图 3、4、5 与 0 的匹配进入 TEASER 后，**inliers 过少**（如 3 < min_safe_inliers）、**inlier_ratio 低**、**rmse 大**，全部被拒。 |
| **几何不一致** | ScanContext 认为相似（overlap_score 尚可），但 FPFH+TEASER 几何一致性差（轨迹未必真闭环、重复结构、参数如 voxel/FPFH 导致对应点少），导致 TEASER 失败。 |

---

## 2. 增强日志：每个计算细节记录

为便于「为什么没检出回环」的深入分析，在以下环节增加了**逐候选/逐步骤**的 INFO 级日志。

### 2.1 初始化与设计说明（一次）

- **`[LOOP_DESIGN]`**  
  - 说明：inter-submap = 子图级（1 desc/子图）；intra-submap = 关键帧级（同子图内 kf↔kf）。  
  - 出现位置：`LoopDetector::init()`。  
  - Grep 示例：`grep LOOP_DESIGN full.log`

### 2.2 子图间（inter-submap）

| 阶段 | 日志 tag / 内容 | 说明 |
|------|------------------|------|
| 子图入队 | `[LOOP_STEP] stage=addSubmap` | sm_id, kf 数, desc_pts, desc_queue, db_size；注明「inter-submap: 1 desc per submap」。 |
| 检索结果 | `[LOOP_STEP] stage=retrieve_result` | NO_CAND 或 OK、raw_candidates 数。 |
| 检索明细 | `[LOOP_STEP] stage=retrieve_detail` | query_id、所有 raw 候选的 target_id 与 score 列表。 |
| 逐候选 gap 过滤 | `[LOOP_STEP] stage=gap_filter_cand` | 每个候选：query_id, cand_idx, target_id, score, **gap**, min_submap_gap, **PASS/FILTERED** 及原因（diff_session / same_submap / gap≤min）。 |
| 候选统计 | `[LoopDetector][CAND_STATS]` | candidates, same_session, same_submap, diff_submap, filtered_by_gap, **valid** 数量。 |
| 几何预筛 | `[LOOP_CAND]` | query_id, target_id, score, geo_dist。 |
| TEASER 入口 | `[LOOP_COMPUTE] query_id= target_id=` | 进入 TEASER 前的 query/target 子图及点数。 |
| TEASER 参数 | `[LOOP_STEP][TEASER] match_enter` | voxel_size, max_points, min_safe_inliers, min_inlier_ratio, max_rmse。 |
| TEASER 中间 | `[LOOP_COMPUTE][TEASER] fpfh_done` | corrs 数量。 |
| TEASER 结果 | `[LOOP_COMPUTE][TEASER] teaser_done` | inliers, corrs, ratio, valid。 |
| TEASER 诊断 | `[TEASER_DIAG]` | inliers, corrs, ratio, min_safe_inliers, min_ratio, valid。 |
| TEASER 汇总 | `[TEASER_RESULT]` | **PASS** 或 **FAIL**，带 inliers/ratio/rmse 及阈值，便于统计通过率。 |
| 拒绝原因 | `[LOOP_COMPUTE][TEASER] teaser_fail reason=` | teaser_extremely_few_inliers / inlier_ratio_low / rmse_too_high 等。 |
| 最终接受 | `[LOOP_ACCEPTED]` / `[LOOP_REJECTED]` | 子图间约束是否加入后端。 |

### 2.3 子图内（intra-submap）

| 阶段 | 日志 tag / 内容 | 说明 |
|------|------------------|------|
| 入口 | `[INTRA_LOOP][DEBUG] ====== detectIntraSubmapLoop START` | submap_id, query_kf_idx, kf_count, 各 min_gap / overlap 阈值。 |
| 检索开始 | `[INTRA_LOOP][INFO] START_SEARCH` | submap_id, query_kf_idx, query_ts, query_pos, history_kf_count。 |
| 时间过滤 | `[INTRA_LOOP][FILTER] TEMPORAL_GAP` | submap_id, query_idx, cand_idx, gap(s), 阈值；**INFO**。 |
| 索引过滤 | `[INTRA_LOOP][FILTER] INDEX_GAP` | submap_id, query_idx, cand_idx, gap, 阈值；**INFO**。 |
| 距离过滤 | `[INTRA_LOOP][FILTER] DISTANCE_GAP` | submap_id, query_idx, cand_idx, dist(m), 阈值；**INFO**。 |
| 描述子过滤 | `[INTRA_LOOP][FILTER] SC_DIST` / `SC_EMPTY` / `SIMILARITY` / `CAND_DESC_ZERO` | 未通过 ScanContext/相似度的原因；**INFO**。 |
| 候选通过 | `[INTRA_LOOP][CANDIDATE] FOUND` | submap_id, query_idx, cand_idx, sim, gap_idx, gap_time, cand_kf_id, cand_pos。 |
| TEASER 开始 | `[INTRA_LOOP][TEASER_START]` | query_idx, cand_idx, query_pts, cand_pts, invoked 次数。 |
| TEASER 拒绝 | `[INTRA_LOOP][TEASER] REJECT_INLIER` / `REJECT_RMSE` | submap_id, query_idx, cand_idx, inlier_ratio/inliers/corrs 或 rmse；**INFO**。 |
| 检测到回环 | `[INTRA_LOOP][DETECTED] ★★★ INTRA_SUBMAP_LOOP` | 成功时的 submap_id, kf_i, kf_j, overlap, inlier_ratio, rmse, delta_t/rpy。 |
| **汇总** | `[INTRA_LOOP][SUMMARY] ====== detectIntraSubmapLoop END` | submap_id, query_kf_idx, total_history_kf, **candidates_found**, **filtered: null/temporal/index/dist/desc/empty_cloud**, **teaser_invoked** / **teaser_failed** / **reject_inlier** / **reject_rmse**，**FINAL_detected**。 |

---

## 3. 建议的日志分析命令

```bash
# 设计说明（确认 inter=子图级 / intra=关键帧级）
grep "LOOP_DESIGN" full.log

# 子图间：谁被 gap 滤掉、谁进了 TEASER、谁被拒
grep -E "gap_filter_cand|retrieve_detail|CAND_STATS|LOOP_COMPUTE.*query_id|TEASER_RESULT|LOOP_ACCEPTED|LOOP_REJECTED" full.log

# 子图内：每个 query 的过滤与汇总
grep -E "INTRA_LOOP\]\[(FILTER|CANDIDATE|TEASER_START|TEASER\] REJECT|SUMMARY|DETECTED)" full.log

# TEASER 通过/失败统计
grep "TEASER_RESULT" full.log
```

---

## 4. 小结

- **子图内**：有检测，关键帧级；通过 `[INTRA_LOOP][FILTER/SUMMARY/DETECTED]` 等可看到每一步过滤和最终检测数。  
- **子图间**：有检测，子图级；子图 0 与 3、4、5 会尝试匹配，与 1、2 的候选会因 min_submap_gap 被滤掉；通过 `gap_filter_cand`、`retrieve_detail`、`TEASER_RESULT`、`LOOP_ACCEPTED/REJECTED` 可看到为何没形成约束。  
- **「理论上很多回环」**：若指关键帧对，子图间当前只做子图对约束；若指子图对，则可能被 gap 或 TEASER（inliers/ratio/rmse）全部拒绝，增强日志已覆盖这些细节，便于逐层排查。
