# HBA 与回环设计说明

## 1) 为何必须先完成后端 GTSAM 优化再启动 HBA？

**结论：是的，必须后端 iSAM2 提交并释放 GTSAM 后，再启动 HBA 做一次最终全局优化，才能保证全局一致性。**

| 要点 | 说明 |
|------|------|
| **设计约束** | 调用 `ensureBackendCompletedAndFlushBeforeHBA()` 后才会 trigger HBA；该函数会等待后端空闲并 `forceUpdate()` 提交所有 pending 因子，释放 GTSAM 状态。 |
| **原因一** | 后端（IncrementalOptimizer/iSAM2）与 HBA（GTSAM batch PGO）共用 GTSAM 库，不能同时持有；否则易 double free 或状态错乱。 |
| **原因二** | HBA 需要基于**当前已提交的因子图**做 batch 优化（关键帧位姿 + 里程计/回环/GPS 约束）；若未先 flush，HBA 拿到的不是最新图。 |
| **原因三** | 先 flush 再 HBA，可保证「前端 + 增量 iSAM2 结果」与「HBA 最终 PGO」之间的顺序一致，便于回放与调试。 |

日志中可见：`[AutoMapSystem][HBA][DESIGN] 必须先完成后端 iSAM2 提交并释放 GTSAM，再启动 HBA 做最终全局优化以保证全局一致性`。

---

## 2) 为何没有有效回环？如何精准优化？

可能原因与对应日志关键字如下（**grep 这些关键字可精准定位**）：

| 阶段 | 可能原因 | 日志关键字 | 调优建议 |
|------|----------|------------|----------|
| 候选检索 | 无重叠候选、gap 过滤、几何预筛 | `[LOOP_PHASE] stage=candidates_retrieved`、`GAP_FILTER`、`GEO_PREFILTER` | 降低 `overlap_threshold`、减小 `min_submap_gap`、调大 `geo_prefilter_max_distance_m` 或置 0 |
| 描述子/点数 | 预处理后点太少 | `[LOOP_COMPUTE][TEASER] match_enter`、`teaser_fail reason=insufficient_pts` | 增大 `teaser.max_points` 或减小 `voxel_size` |
| FPFH 对应点 | 对应点过少 | `[LOOP_COMPUTE][TEASER] fpfh_done corrs=`、`teaser_fail reason=insufficient_corrs` | 改进场景重叠或 FPFH 参数 |
| TEASER 前检查 | corrs<20 跳过 TEASER | `teaser_fail reason=teaser_precheck_insufficient_corrs`、`insufficient_corrs_v3` | 增加可靠对应点（同上） |
| TEASER 求解后 | 内点过少（如 inliers=3&lt;10） | `[LOOP_COMPUTE][TEASER] teaser_done inliers=`、`teaser_fail reason=teaser_extremely_few_inliers` | 适当放宽 safe_min（代码中 kMinSafeInliers）或改进初值/重叠 |
| 内点率 | 内点率低于阈值 | `teaser_fail reason=inlier_ratio_low` | 调大 `min_inlier_ratio` 或改进匹配质量 |
| RMSE | 几何误差过大 | `[LOOP_REJECTED] reason=rmse_too_high` | 调大 `max_rmse` 或改进配准 |

**每条回环计算的全流程日志**（便于按 query_id/target_id 追踪）：

- `[LOOP_PHASE]`：阶段（descriptor_done → candidates_retrieved → match_enqueue → geom_verify_enter）
- `[LOOP_COMPUTE]`：单次匹配摘要（query_id、target_id、success、inliers、corrs、inlier_ratio、rmse、reason）
- `[LOOP_COMPUTE][TEASER]`：TEASER 内部（match_enter、fpfh_done、teaser_solve_enter、teaser_done、teaser_fail reason=...）

建议：跑完建图后 `grep -E "LOOP_PHASE|LOOP_COMPUTE" full.log` 即可看到完整回环流水线与每次失败原因，便于精准调参。
