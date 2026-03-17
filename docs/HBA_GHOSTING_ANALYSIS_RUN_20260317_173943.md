# HBA 优化后严重重影根因分析（run_20260317_173943）

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **现象** | HBA 优化后仍存在**严重重影**。 |
| **日志依据** | `full.log`：HBA 写回后 `last_pos=[-0.58,-9.14,-0.08]`，而 **odom_path** 持续发布 `last_pos` 约 [40,-40] 量级；rebuild/写回/快照时间线正确，无 save 与 rebuild 竞态。 |
| **根因** | **odom_path 与 global_map 使用不同坐标系**：global_map 与 optimized_path 使用 **T_w_b_optimized**（HBA/GPS 对齐后的世界系），odom_path 始终使用 **T_w_b（前端里程计）**。HBA 后整体轨迹被 GPS 对齐平移约 30–40m（首帧 [0,-0.01] 不变、末帧由约 [40,-40] 变为 [-0.58,-9.14]），同屏显示 odom_path + global_map = 两套几何叠加 = **严重重影**。 |
| **建议** | ① **RViz 中隐藏 odom_path**，仅显示 **optimized_path + global_map**；② 代码中在首次发布 odom_path 且已发布过 map/optimized_path 时打一次 WARN；③ HBA 完成后发布 optimized_path 时打日志标明“与 global_map 同源，请隐藏 odom_path”。 |
| **是否已修复** | **使用侧可消除重影**：隐藏 odom_path 即可。代码侧已加 WARN 与 HBA 后说明；若需从根本避免同屏错位可加配置「优化后停发 odom_path」（可选）。 |
| **精准定位** | 见下表「重影排查快速指南」：`grep GHOSTING_CHEAT_SHEET` / `GHOSTING_DIAG` / `HBA_GHOSTING` / `GHOSTING_SOURCE`。 |

### 重影排查快速指南（grep 关键词）

| 目的 | 命令 | 说明 |
|------|------|------|
| 一眼判断是否两系错位 | `grep GHOSTING_CHEAT_SHEET` | 同一行含 odom_last、opt_last、map_pts；若 odom_last 与 opt_last 相差大（>1m）则同屏必重影。 |
| 时间线：写回 vs 快照 vs 建图 | `grep GHOSTING_DIAG` | writeback_enter/done、pose_snapshot_taken、buildGlobalMapInternal_exit、map_published。 |
| HBA 末帧 odom→优化 平移量 | `grep HBA_GHOSTING` | last_kf odom=[...] -> optimized=[...] trans_diff=Ym；及「请隐藏 odom_path」提示。 |
| 位姿来源与 fallback | `grep GHOSTING_SOURCE` | odom_path / optimized_path / map 的 pose_source；snapshot_fallback_count>0 表示部分 KF 用 odom。 |

---

## 1. 日志与时间线证据

### 1.1 HBA 执行与写回（本次 run 已正确）

```
44972  [HBA][STATE] optimization start keyframes=498 gps=1 (running=1)
46046  [SubMapMgr][GHOSTING_DIAG] updateAllFromHBA enter ts=1628250231.411 optimized_poses=498 submaps=6
46047  [SubMapMgr][GHOSTING_DIAG] HBA writeback_enter ts=1628250231.411 pose_count=498
46048  [SubMapMgr][GHOSTING_DIAG] HBA writeback_done ts=1628250231.411 written=498 expected=498 first_pos=[0.00,-0.01,-0.01] last_pos=[-0.58,-9.14,-0.08]
46073  [SubMapMgr][HBA_DIAG] updateAllFromHBA done: submaps=6 max_trans_diff=4.230m max_rot_diff=5.29deg
46075  [SubMapMgr][HBA_GHOSTING] rebuildMergedCloudFromOptimizedPoses enter
46154  [SubMapMgr][REBUILD_MERGE] Done rebuilding merged_cloud for all submaps
46155  [SubMapMgr][GHOSTING_DIAG] rebuildMergedCloudFromOptimizedPoses done ts=1628250231.411
46163  [AutoMapSystem][HBA][DIAG] updateAllFromHBA completed, triggering global map refresh
```

- 写回 498 个关键帧，**last_pos 从 odom 系约 [40,-40] 变为优化系 [-0.58,-9.14,-0.08]**（约 30–40m 平移）。
- rebuild 在 writeback 之后完成，**无 save-before-rebuild 竞态**（与 HBA_GHOSTING_ROOT_CAUSE_20260317.md 中修复一致）。

### 1.2 位姿来源对比（重影根因）

| 数据源 | 位姿来源 | 典型 last_pos（HBA 后） | 与 global_map 同系？ |
|--------|----------|-------------------------|----------------------|
| **global_map** | T_w_b_optimized（快照） | [-0.58,-9.14,-0.08] | ✅ 是 |
| **optimized_path** | KF T_w_b_optimized | 同上 | ✅ 是 |
| **odom_path** | T_w_b（前端 odom） | 约 [40,-40] 量级 | ❌ **否** |

- 日志中持续出现：  
  `[GHOSTING_SOURCE] odom_path published pose_source=odom(T_w_b) ... (若与 global_map 同屏显示则轨迹与点云错位即重影)`  
- HBA 后若用户同时显示 **odom_path** 与 **global_map**，会看到两条轨迹/点云错位约 30–40m = **严重重影**。

### 1.3 buildGlobalMap 快照一致性

- build_id=10/11 的 `pose_snapshot_taken` 均在 **writeback_done 之后**，且 `last_pos=[-0.58,-9.14,-0.08]`，与 HBA 写回一致。
- `snapshot_fallback_count=0`，无 KF 使用 odom 回退，主路径位姿源正确。

---

## 2. 根因归纳（逐行对应）

### 2.1 数据流

1. **odom_path**：在 `automap_system.cpp` 的 `onOdom` 中，用当前帧 **T_w_b（odom）** 追加到 `odom_path_.poses` 并发布；**从不**改为 T_w_b_optimized。
2. **global_map**：由 `buildGlobalMap`（或 async 快照）用 **T_w_b_optimized** 将各 KF 的 cloud_body 变换到世界系；HBA 后通过 updateAllFromHBA + rebuild 已统一为优化系。
3. **optimized_path**：由 `publishOptimizedPath(all_sm)` 从各 KF 的 **T_w_b_optimized** 收集并发布；HBA 完成后在 `onHBADone` 中会再调用一次，与 global_map 同源。

### 2.2 为何“HBA 优化后”仍重影

- 重影**不是**“map 用错位姿”或“rebuild 未执行”：日志显示 map 与 rebuild 均使用 T_w_b_optimized。
- 重影**是**“**两条轨迹/点云同时显示且处于不同坐标系**”：  
  - 一条为 **global_map + optimized_path**（优化系，HBA/GPS 对齐后）；  
  - 一条为 **odom_path**（odom 系，未对齐）。  
- HBA 后优化系与 odom 系整体平移约 30–40m，同屏即表现为严重重影。

---

## 3. 建议措施

### 3.1 使用侧（立即生效）

- **RViz**：隐藏 **/automap/odom_path**，仅显示 **/automap/optimized_path** 与 **/automap/global_map**。
- 若需对比轨迹，可将 odom 轨迹导出到文件单独对比，不要与 global_map 同屏。

### 3.2 代码侧（可落地）

1. **增强日志**  
   - 在**首次**发布 odom_path 且已发布过 global_map 或 optimized_path 时，打**一次** WARN：  
     “odom_path 与 global_map 使用不同坐标系，同屏显示将产生重影；请仅显示 optimized_path + global_map”。  
   - 在 HBA 完成后 `publishOptimizedPath` 时打 INFO：  
     “optimized_path 已按 HBA 结果更新，与 global_map 同源；请在 RViz 中隐藏 odom_path 以避免重影”。

2. **可选**  
   - 配置项：如 `publish_odom_path_after_optimization: false`，在首次 HBA/优化后停止发布 odom_path，避免误用。  
   - 或保留发布但在 topic 说明/README 中明确“仅用于调试，与 global_map 不同系”。

### 3.3 验证

- 同一 bag、同一 run：在 RViz 中**仅**显示 optimized_path + global_map，应**无**重影。
- 再打开 odom_path：应出现约 30–40m 错位的“重影”；关闭 odom_path 后恢复一致。

---

## 4. 与本 run 无关的已修复点

- **Save 与 rebuild 竞态**：已通过“先执行 done_cbs_（含 rebuild），再置 hba_running_=false”修复（见 HBA_GHOSTING_ROOT_CAUSE_20260317.md）；本 run 日志也显示 rebuild 在 save 前完成。
- **快照与 writeback 竞态**：writeback 与 pose_snapshot 均在 SubMapManager::mutex_ 下串行，本 run 无 snapshot 落在 writeback 之间的异常。

---

## 5. 小结

| 问题 | 结论 |
|------|------|
| HBA 后是否仍用错位姿建图？ | 否；global_map 与 rebuild 均使用 T_w_b_optimized。 |
| 为何仍见严重重影？ | odom_path 与 global_map 不同坐标系（odom vs 优化系），同屏显示导致。 |
| 如何消除重影？ | 隐藏 odom_path，仅显示 optimized_path + global_map。 |
| 后续代码改进？ | 增加一次性 WARN 与 HBA 后 optimized_path 的说明日志；可选配置关闭 HBA 后 odom_path 发布。 |
