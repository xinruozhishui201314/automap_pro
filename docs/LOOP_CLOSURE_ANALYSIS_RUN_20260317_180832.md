# 回环检测深度分析报告（run_20260317_180832 / full.log）

## 0. Executive Summary

| 维度 | 结论 | 说明 |
|------|------|------|
| **子图内回环** | **逻辑正常，约束为 0** | 有 TRIGGER/SUMMARY/FILTER，778 次检测全部 `FINAL_detected=0`；候选被 **INDEX_GAP**（gap<10）与 **TEMPORAL_GAP**（<3s）过滤，未进入 TEASER，符合配置的保守策略。 |
| **子图间回环** | **检索正常，几何验证全部失败** | 子图 0：`NO_CAND`（仅 1 子图）；子图 1：`OK raw_candidates=10`。约 150 次 TEASER 全部 `teaser_fail reason=inlier_ratio_low`（ratio≈0.004~0.009 ≪ 0.12），**0 条约束入图**。 |
| **配置一致性** | **正常** | TEASER 使用 YAML 参数：`min_safe_inliers=4`、`min_inlier_ratio=0.12`（CONFIG + VERIFY + match_enter 一致）。 |
| **整体** | **回环流程正常，无有效闭合** | Backend 全程 `loop=0`；无 `LOOP_ACCEPTED`/`addLoopFactor`/`loop_intra`。根因是**几何一致性差**（inlier ratio 极低）或**轨迹无真实闭环**，非代码错误。 |

---

## 1. 日志与统计摘要

### 1.1 本 run 回环相关统计（从 full.log）

| 指标 | 数值 |
|------|------|
| INTRA_LOOP SUMMARY 总次数 | 778 |
| INTRA_LOOP FINAL_detected>0 | 0 |
| retrieve_result NO_CAND | 1（query_id=0, db_size=1） |
| retrieve_result OK | 1（query_id=1, raw_candidates=10） |
| TEASER 调用次数 | ~150 |
| TEASER valid=1 / LOOP_ACCEPTED | 0 |
| Backend loop 计数 | 全程 0 |

### 1.2 子图与检索

- **子图 0 冻结**（~18:13:16）：`retrieve_enter submap_id=0 db_size=1` → `retrieve_result NO_CAND`（仅 1 个子图，无跨子图候选，正常）。
- **子图 1 冻结**（~18:15:18）：`retrieve_enter submap_id=1 db_size=2` → `retrieve_result OK raw_candidates=10`，随后进入 **inter_keyframe** 级与子图 0 的 10 个候选对进行 TEASER。
- 日志中**仅出现子图 0、1 的 retrieve**；若 run 在子图 2 未再触发子图间检索或 run 提前结束，则子图 2 未产生子图间检索记录。

### 1.3 设计要点（与本 run 对应）

- **子图内**：关键帧级，`detectIntraSubmapLoop(active_submap, query_kf_idx)`；通过 index/temporal/dist/desc 过滤后 TEASER，通过则 `addLoopFactorDeferred`。
- **子图间**：inter_keyframe_level=true，子图冻结后 ScanContext 检索 → 关键帧对 → TEASER → 通过则 `onLoopDetected` → `addLoopFactor`。

---

## 2. 子图内回环分析

### 2.1 现象

- 每次新关键帧都有 `[INTRA_LOOP][TRIGGER] submap_id=0 kf_idx=*`。
- 778 条 `[INTRA_LOOP][SUMMARY]` 均为：`candidates_found=0` 或候选被过滤，`teaser_invoked=0`，`FINAL_detected=0`。
- 过滤统计（从代表性 SUMMARY）：`filtered: temporal=* index=* dist=0 desc=0`，即主要被 **INDEX_GAP**、**TEMPORAL_GAP** 过滤。

### 2.2 配置（与本 run 一致）

- `intra_submap_min_keyframe_gap: 10`
- `intra_submap_min_temporal_gap_s: 3.0`
- `intra_submap_min_distance_gap_m: 3.0`

### 2.3 结论

- **子图内回环逻辑正常**：TRIGGER → 候选枚举 → INDEX/TEMPORAL/DIST/DESC 过滤 → SUMMARY，流程完整。
- **未产生约束的原因**：  
  - 要么历史关键帧不足（query 与候选的 index gap < 10），  
  - 要么时间间隔 < 3s（TEMPORAL_GAP SKIP），  
  因此从未进入 TEASER（teaser_invoked=0）。
- 在**单子图、短时间、无回头轨迹**的片段内，不产生子图内回环是**符合预期的**；若希望更多子图内候选，可适当减小 `intra_submap_min_keyframe_gap` / `intra_submap_min_temporal_gap_s`（需权衡误匹配）。

---

## 3. 子图间回环分析

### 3.1 检索阶段

- **子图 0**：db_size=1 → NO_CAND，正常。
- **子图 1**：db_size=2，ScanContext 检索得到 10 个候选（对子图 0 的关键帧），进入子图间关键帧级匹配。

### 3.2 TEASER 几何验证

- 所有 TEASER 调用均使用：`min_safe_inliers=4 min_inlier_ratio=0.120 max_rmse=0.400m`（与 YAML 一致）。
- 约 150 次 TEASER 结果：
  - **inliers**：多为 4 或 5（≥ min_safe_inliers=4）。
  - **ratio**：0.004 ~ 0.009（远低于 min_inlier_ratio=0.12）。
  - **失败原因**：全部为 `teaser_fail reason=inlier_ratio_low`。
- 少数日志出现 `inliers=7 ratio=0.0085` 仍因 ratio < 0.12 被拒（`teaser_fail reason=inlier_ratio_low`）。

### 3.3 结论

- **子图间检索正常**：ScanContext 能给出 10 个候选，说明描述子层面“可能闭环”被检出。
- **几何验证全部不通过**：FPFH 对应点中 TEASER 内点比例极低（约 0.5%~1%），可能原因包括：
  1. **轨迹未真正回到子图 0 的同一位置**（无真实闭环）。
  2. **重复结构/不同视角**：ScanContext 相似但局部几何（FPFH）差异大。
  3. **点云质量/配准误差**：体素、噪声导致对应点少且内点少。
- **未放宽 min_inlier_ratio 的原因**：0.12 已是常用下限，再低误匹配风险高；当前瓶颈在**几何一致性**，而非参数是否生效。

---

## 4. 配置与一致性

- **CONFIG_READ_BACK**：`loop_closure.teaser: min_inlier_ratio=0.12 min_safe_inliers=4 max_rmse_m=0.40`。
- **LoopDetector CONFIG**：`TEASER applied from YAML: min_safe_inliers=4 min_inlier_ratio=0.12 max_rmse=0.40`。
- **TeaserMatcher VERIFY**：`first match params in use: min_safe_inliers=4 min_inlier_ratio=0.12 max_rmse=0.40`。
- **match_enter**：`min_safe_inliers=4 min_inlier_ratio=0.120 max_rmse=0.400m`。

与本 run 对应的前序分析（run_173943）中提到的“TeaserMatcher 使用默认 10/0.3”问题，在本 run 中**未出现**；本 run 中 TEASER 参数与 YAML 一致。

---

## 5. 总体结论与建议

### 5.1 结论汇总

| 项目 | 是否正常 | 说明 |
|------|----------|------|
| 子图内回环流程 | 是 | 触发、过滤、汇总完整；约束为 0 源于 index/temporal 过滤，符合配置。 |
| 子图间检索 | 是 | 子图 1 对 0 有 10 个候选，ScanContext 工作正常。 |
| 子图间几何验证 | 否（全部失败） | 全部 inlier_ratio_low，无约束入图。 |
| 配置加载 | 是 | TEASER 与 YAML 一致。 |
| 回环约束入图 | 否 | 全程 loop=0。 |

**整体**：子图内和子图间的**回环管线运行正常**，但本 run 中**未产生任何有效回环约束**，主要受限于**几何一致性极差**（inlier ratio 约 0.5%~1%）或**轨迹本身无真实闭环**。

### 5.2 建议

1. **数据与轨迹**  
   - 确认该 bag（M2DGR street_03_ros2）在子图 0 与 1 之间是否存在**真实回到同一区域的闭环**。  
   - 若为重复结构（如相似街道），可考虑更强描述子或几何约束，减少误候选。

2. **参数（谨慎放宽）**  
   - 若确认存在真实闭环且希望尝试更多约束：可**小幅**降低 `loop_closure.teaser.min_inlier_ratio`（如 0.08）并观察误匹配与优化稳定性。  
   - 可适当放宽 `loop_closure.teaser.max_rmse_m`（当前 0.4m），观察是否有因 RMSE 被拒的边界情况（本 run 中主要瓶颈为 ratio）。

3. **可观测性**  
   - 子图内已有 SUMMARY；子图间可增加“每子图检索/TEASER 统计”汇总（候选数、TEASER 调用数、失败原因分布），便于后续调参与排查。

4. **与 run_173943 对比**  
   - run_173943：TeaserMatcher 使用默认 10/0.3，子图间 inliers=5 被 safe_min=10 拒绝。  
   - run_180832：TeaserMatcher 使用 4/0.12，子图间 inliers 满足 4，但 ratio 远低于 0.12，故仍全部拒绝；**瓶颈从“inlier 数”转为“inlier 比例”**，说明本 run 配置已生效，问题在数据/几何一致性。

---

## 6. 附录：关键日志片段

### 6.1 子图内 SUMMARY 示例（FINAL_detected=0）

```
[INTRA_LOOP][SUMMARY] submap_id=0 query_kf_idx=1 history_kf=2 candidates_found=0 filtered: temporal=0 index=1 dist=0 desc=0 teaser_invoked=0 failed=0 reject_inlier=0 reject_rmse=0 FINAL_detected=0
[INTRA_LOOP][FILTER] INDEX_GAP: ... gap=%d < %d (SKIP)
[INTRA_LOOP][FILTER] TEMPORAL_GAP: ... gap=%.2fs < %.1fs (SKIP)
```

### 6.2 子图间 retrieve

```
stage=retrieve_result NO_CAND query_id=0 db_size=1
stage=retrieve_result OK query_id=1 raw_candidates=10
```

### 6.3 子图间 TEASER 失败（统一原因）

```
[LOOP_COMPUTE][TEASER] teaser_fail reason=inlier_ratio_low inliers=4 ratio=0.005 min_ratio=0.12
[LOOP_COMPUTE][TEASER] teaser_fail reason=inlier_ratio_low inliers=5 ratio=0.006 min_ratio=0.12
[TEASER_DIAG] inliers=7 corrs=821 ratio=0.0085 ... (valid=0)
[LOOP_COMPUTE][TEASER] teaser_fail reason=inlier_ratio_low inliers=7 ratio=0.009 min_ratio=0.12
```

### 6.4 Backend 状态（全程 loop=0）

```
[AutoMapSystem][BACKEND] state=MAPPING kf=* sm=1 loop=0 ...
[AutoMapSystem][BACKEND] state=MAPPING kf=* sm=2 loop=0 ...
```
