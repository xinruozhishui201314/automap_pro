# 子图内无重影 vs 全局点云重影 + 子图保存环节

**结论**：子图内点云是「单段轨迹、统一位姿系」拼接，无闭环故无重影；全局图是「多子图 + 无回环约束」拼接，闭环处会重影。子图在 **saveMapToFiles() 末尾**、与 global_map.pcd 同一次保存流程中归档到 `session_<id>/submap_*`。

---

## 1. 为什么子图内没有重影、全局重影严重？

| 维度 | 子图内 | 全局图 |
|------|--------|--------|
| **点云来源** | 单子图内关键帧的 `cloud_body`，用**该子图内**的位姿（先 T_w_b，HBA 后 rebuild 用 T_w_b_optimized）变换到世界系并累加到 `merged_cloud`。 | 所有子图的所有关键帧，用各自的 **T_w_b_optimized** 变换到**同一**世界系后拼接成一张图。 |
| **轨迹形态** | 子图对应**一段连续轨迹**，不会在同一子图内「回到同一点」形成闭环。 | 整条轨迹可能**物理闭环**（车回到起点或重复区域）；若无回环约束，同一地点会对应两个不同位姿。 |
| **重影原因** | 单段 + 同一位姿系 → 无「同地两点」→ **无重影**。 | 多段拼接 + **无回环约束** → 闭环处同地在全局系下出现两次 → **严重重影**。 |

因此：**子图内没有重影**是预期行为（子图内本身无闭环）；**全局点云重影严重**来自「无回环约束下多子图在全局系拼接」，与 [GHOSTING_ANALYSIS_RUN_20260317_231548.md](./GHOSTING_ANALYSIS_RUN_20260317_231548.md) 中的结论一致。

---

## 2. 子图点云是怎么来的？（merged_cloud / downsampled_cloud）

- **建图过程中**：每来一帧关键帧，`mergeCloudToSubmap()` 用该帧的 **T_w_b（里程计）** 把 `cloud_body` 变换到世界系，累加到当前子图的 `merged_cloud`。注释写明：*merged_cloud 用 odom 系，HBA 后需 rebuild*。
- **HBA 完成后**：`rebuildMergedCloudFromOptimizedPoses()` 用每个关键帧的 **T_w_b_optimized** 重新把 `cloud_body` 变换到世界系，重算每个子图的 `merged_cloud`，使子图点云与 HBA 轨迹一致。
- **子图冻结时**：对 `merged_cloud` 做体素下采样得到 `downsampled_cloud`（用于回环匹配和归档）。归档时保存的就是这份 **downsampled_cloud**（已是 HBA 后的世界系）。

所以：**归档时的子图点云 = 单子图内、HBA 优化后位姿、同一世界系** → 子图内无闭环、无重影。

---

## 3. 子图在哪个环节保存？保存在哪？

- **环节**：在 **`saveMapToFiles(output_dir)`** 里，与保存 `global_map.pcd`、`trajectory_tum.txt`、`keyframe_poses.pcd`、`gps_positions_map.pcd` **同一次调用**；子图归档在**这些之后、函数末尾**执行。
- **调用链**：  
  `finish_mapping` → HBA 完成（含 `updateAllFromHBA` + `rebuildMergedCloudFromOptimizedPoses`）→ `saveMapToFiles(timestamped_dir)` → 写 global_map / 轨迹 / keyframe / GPS → **再** `archiveSubmap(sm, session_dir)` 对每个子图归档。
- **保存路径**：  
  `session_dir = output_dir + "/session_" + std::to_string(current_session_id_)`  
  例如：`data/automap_output/20260317_2330/session_1773760715896534342/`  
  每个子图一个目录：`session_1773760715896534342/submap_0/`、`submap_1/`、…
- **每个 submap_* 目录内容**（`SubMapManager::archiveSubmap()`）：
  - `submap_meta.json`：id、时间范围、锚点位姿（`pose_w_anchor_optimized`）、GPS、描述子等；
  - `downsampled_cloud.pcd`：该子图的 `merged_cloud` 下采样结果（**世界系，HBA 后**）。

因此：**子图不是在「建图过程中」零散保存的，而是在 finish_mapping 后的唯一一次 saveMapToFiles 末尾、按 session 一次性归档的**。

---

## 4. 小结

| 问题 | 答案 |
|------|------|
| 各个子图内没有任何重影？ | 是。子图内是单段轨迹、统一位姿系，无闭环，所以无重影。 |
| 全局点云重影严重？ | 是。多子图在全局系拼接且无回环约束，闭环处同地多姿态导致重影。 |
| 子图保存在哪个环节？ | 在 **saveMapToFiles()** 的**末尾**，与 global_map 同一次保存流程；先写 global_map / 轨迹 / keyframe / GPS，再对每个子图调用 **archiveSubmap** 写入 `output_dir/session_<session_id>/submap_<id>/`。 |

子图目录示例（本次 run）：  
`data/automap_output/20260317_2330/session_1773760715896534342/submap_0/` … `submap_5/`，每目录含 `submap_meta.json` 与 `downsampled_cloud.pcd`。
