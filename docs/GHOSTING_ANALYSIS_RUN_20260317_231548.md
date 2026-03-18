# 重影问题深入分析 — run_20260317_231548

**运行**: run_20260317_231548 · **输出**: data/automap_output/20260317_2330  
**结论**: 本日志下重影主要来自 **① 无回环约束导致同地多姿态** 与 **② 建图过程中 odom_path 与 global_map 同屏**；写回顺序、PCD/RViz 同源已正确。

---

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **重影根因（优先级）** | 1）**无回环被接受** → 轨迹若有物理闭环，同地会出现在两处位姿 → 保存的 PCD 上表现为「同一场景两套点云」的严重重影；2）**建图全程发布 odom_path** → 与 global_map（iSAM2/HBA 系）同屏即双轨迹重影。 |
| **已排除的项** | 写回顺序错位（VERIFY_WRITEBACK match=1）、PCD 与 RViz 不同源（已用 cache 同写同发）、fallback_merged_cloud、T_w_b_optimized 未优化。 |
| **建议动作** | 优先：**放宽或调整回环接受条件**，使有效回环能加入 HBA；RViz 建图时**隐藏 odom_path** 或提前在首帧 HBA 后清空 odom_path。 |

---

## 1. 日志关键证据汇总

### 1.1 写回与数据源（无问题）

- **HBA 写回顺序**  
  - `[HBA_INPUT_ORDER] first kf_id=0 ... last kf_id=494 count=495` 与 `[WRITEBACK_ORDER] first kf_id=0 ... last kf_id=494 count=495` 一致。  
  - `[VERIFY_WRITEBACK] first kf_id=0 match=1 last kf_id=494 match=1` → 写回与 HBA 输出一一对应，**无位姿-关键帧错位**。
- **PCD 与 RViz 同源**  
  - `[PCD_GHOSTING_FIX] built global map once (pts=1178773), cache set for save + RViz` → `save using cached`、`publish using cached` → 保存的 `global_map.pcd` 与 RViz 最后一次发布为**同一份** HBA 后点云。
- **构建路径**  
  - 全部 `[GHOSTING_RISK] buildGlobalMap_async/sync path=snapshot|from_kf pose_source=T_w_b_optimized_only (无重影风险)`，无 fallback、无 UNOPT。

### 1.2 回环：整轮无一次接受（重影主因之一）

- **LOOP_ACCEPTED**  
  - 全日志 **无** `LOOP_ACCEPTED` / `addLoopFactor`，即**没有任何回环约束加入因子图**。
- **TEASER 全部失败**  
  - 大量 `[LOOP_COMPUTE][TEASER] teaser_done inliers=4 corrs=7xx ratio=0.004 valid=0`，拒绝原因：`inlier_ratio_low`（ratio≈0.004 &lt; min_ratio=0.08）。  
  - 伴随 `[FPFH_DIAG] dist_p10=9.52m p50=25.07m p90=57.46m`、`p90>>5m 表示存在大量误匹配`，几何一致性差。
- **含义**  
  - HBA 仅依赖 **里程计 + GPS**，无回环约束。若轨迹存在**物理闭环**（车回到同一区域），同一场景会对应**两个不同位姿**，全局图会呈现「同一建筑/道路出现两次」→ **保存的 global_map.pcd 上严重重影**。

### 1.3 odom_path 与 global_map 同屏（重影主因之二）

- **建图过程中**  
  - 持续有 `[GHOSTING_SOURCE] odom_path published pose_source=odom(T_w_b) count=...`，且出现 `[HBA_GHOSTING] odom_path 与 global_map 使用不同坐标系，同屏显示将产生严重重影`。  
  - `odom_path_stopped_after_hba_` 仅在 **onHBADone 且 gps_aligned_** 时置 true 并清空 odom_path；本 run 仅有一次 HBA（finish_mapping），故**建图全程** odom_path 一直在发布。
- **HBA 与 iSAM2 分离**  
  - `P2 HBA-iSAM2 separation: max_drift=3.803m, avg_drift=2.272m`；子图锚点 trans_diff 最大 3.8m（sm_id=2）。  
  - 即：**里程计轨迹（odom_path）与 HBA 优化轨迹（optimized_path / global_map）最大相差约 3.8m**，同屏即双轨迹重影。

### 1.4 其他与重影相关的日志

- **voxelDownsampleChunked**  
  - `skip_final_voxel (extent overflow risk)`：范围过大时跳过最终体素，可能带来重叠区未充分合并，通常为次要因素。
- **GHOSTING_CHEAT_SHEET**  
  - 建图过程中 `diff_odom_opt_m` 约 0.15～0.81m；结束时 `odom_count=0`（odom_path 已清空），`opt_last=[-0.58,-9.14,-0.08]`。

---

## 2. 根因归纳

```text
重影表现：
  A) 保存的 global_map.pcd 中「同一场景出现两次」→ 无回环约束，闭环处未对齐。
  B) RViz 建图时「两条轨迹/点云错位」→ odom_path（odom 系）与 global_map（iSAM2/HBA 系）同屏。
```

| 根因 | 说明 | 对应现象 |
|------|------|----------|
| **无回环接受** | 全 run 无 LOOP_ACCEPTED；TEASER 均 inlier_ratio_low（inliers=4, ratio≈0.004）。HBA 仅用 odom+GPS，不修正闭环。 | 轨迹若有闭环，PCD 上同地多姿态 → 严重重影。 |
| **odom_path 与 map 同屏** | 建图阶段一直发布 odom_path，仅在一次 HBA（finish_mapping）后才清空。 | RViz 中双轨迹/点云错位，diff 最大约 3.8m。 |

---

## 3. 建议措施（按优先级）

### 3.1 让回环能够被接受（解决 PCD 同地重影）

- **放宽 TEASER 接受条件（谨慎）**  
  - 当前：`min_inlier_ratio=0.08`、`min_safe_inliers=3`，实际 ratio≈0.004 被拒。  
  - 可尝试：略降 `min_inlier_ratio`（如 0.03～0.05）或配合 `min_safe_inliers` 做 A/B 测试；需同时监控误匹配（误回环会引入更大错位）。
- **改善回环候选质量**  
  - 日志中 FPFH `p90=25～57m` 表示对应点距离很大，多为误匹配。可考虑：  
    - 加强几何/距离过滤（如 `loop_closure.geo_prefilter_max_distance_m`、FPFH 后过滤）；  
    - 或优化 ScanContext 检索，减少明显错误候选再进 TEASER。
- **验证**  
  - 修改后重新跑同 bag，用 `grep LOOP_ACCEPTED full.log` 确认是否有回环加入；再检查保存的 PCD 同区域是否仍双影。

### 3.2 消除 RViz 同屏重影（odom_path）

- **使用侧**  
  - 建图时在 RViz 中**隐藏 / 取消勾选 odom_path**，仅显示 **optimized_path + global_map**。
- **代码侧（可选）**  
  - 若希望「第一次 HBA 完成后就清空 odom_path」（不等到 finish_mapping），可在 onHBADone 中在现有逻辑下确保 `gps_aligned_` 为 true 时即清空并发布空 Path（当前逻辑已满足「HBA 且 gps_aligned 时清空」，本 run 仅一次 HBA 在 finish 时，故表现为结束时才清空）。  
  - 若增加「周期 HBA」或「中间 HBA」，则第一次 HBA 完成后就会清空 odom_path，建图中期即可避免同屏双轨迹。

### 3.3 可选：大范围建图与体素

- 若 bbox 极大导致 `skip_final_voxel`，可评估分块下采样或调整体素参数，减少重叠区未合并带来的轻微重影感；优先级低于回环与 odom_path。

---

## 4. 验证清单（修改后建议执行）

| 检查项 | 命令/方法 |
|--------|-----------|
| 是否有回环被接受 | `grep LOOP_ACCEPTED full.log` |
| 写回是否仍正确 | `grep -E "VERIFY_WRITEBACK|writeback MISMATCH" full.log` |
| PCD 与 RViz 是否同源 | `grep PCD_GHOSTING_FIX full.log` |
| odom_path 是否仍与 map 同屏 | 建图时 RViz 仅勾选 optimized_path + global_map，或确认首帧 HBA 后 odom_count 清零 |

---

## 5. 小结

- **写回顺序、PCD/RViz 同源、构建路径**在本 run 中均正常，不是本次重影来源。  
- **严重重影**主要来自：  
  1）**全程无回环约束** → 闭环场景在 PCD 上呈「同地两套点云」；  
  2）**建图过程 odom_path 与 global_map 同屏** → RViz 双轨迹/点云错位（最大约 3.8m）。  
- 优先建议：**放宽或优化回环接受条件并验证 LOOP_ACCEPTED**；建图时 **RViz 隐藏 odom_path** 或依赖 HBA 后清空逻辑，避免同屏重影。
