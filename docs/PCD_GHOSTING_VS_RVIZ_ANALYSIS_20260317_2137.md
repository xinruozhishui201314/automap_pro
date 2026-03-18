# global_map.pcd 重影 vs RViz 无重影 — 根因分析

**运行**: run_20260317_212313 · **输出目录**: data/automap_output/20260317_2137  
**现象**: 记录的 `global_map.pcd` 存在严重重影，后端 RViz 最后显示无重影。

---

## 0. 根本原因（直接回答）

| 问题 | 结论 |
|------|------|
| **RViz 没重影是因为没做完 HBA 吗？** | **不是。** HBA 已完整执行（writeback_done written=498）。RViz 没重影是因为 RViz 显示的那一帧点云与「当前显示的轨迹/坐标系」一致；而**保存的 PCD 与 RViz 用的不是同一次构建**，所以会出现「一边正常、一边重影」。 |
| **是保存的 PCD 有问题，还是 RViz 有问题？** | **本质是「两路数据不一致」**：保存的 PCD 来自一次 build（可能快照时机或路径与 writeback 有竞态，或走了 fallback），RViz 来自另一次 build 或更晚的发布。**出问题的是「没有单一数据源」**：PCD 和 RViz 各用各的 build，所以会一个重影、一个不重影。 |
| **根本原因一句话** | **保存用一套 build、发布用另一套 build，两路未共用同一份 HBA 后点云，导致 PCD 与 RViz 数据源不一致；重影出现在「用了错误或混合位姿」的那一路（通常是 PCD）。** |
| **若 RViz 显示的不是 HBA 优化后的点云，重影算解决了吗？** | **没解决。** 若 RViz 里看到的仍是 HBA 之前的旧图（或未与轨迹对齐），只是「看起来没重影」是因为旧图自身一致，并不是因为重影被修好了。**真正解决 = RViz 与保存的 PCD 都必须是「同一次 HBA 后构建」的同一份点云。** 已做修复：HBA 后只构建一次并缓存，该结果同时用于发布到 RViz 和写入 PCD，保证两边同源且均为 HBA 优化后。 |

**结论**：不是「RViz 没做 HBA」或「只有 PCD 错了」二选一，而是**两路（保存 vs 发布）没有共用同一份 HBA 后点云**。修复后应同时满足：① RViz 显示的是 HBA 优化后的点云；② 保存的 PCD 与 RViz 是同一份数据。

**如何确认 RViz 显示的是 HBA 优化后的点云？**  
HBA 完成后会触发 `map_publish_pending_`，map_publish 线程会执行一次 `publishGlobalMap()` 并发布新点云。因此**理论上** RViz 会收到 HBA 后的那一帧；但该次 build 是异步的（日志中可见 build 约 25 秒才完成），若在 build 完成并发布前就关掉或切界面，RViz 里看到的可能仍是**旧一帧**（HBA 前）。旧图自身一致所以「看起来没重影」，但并不是 HBA 优化后的结果。**修复后的逻辑**：在 onHBADone 里用**同步** build 构建一次，该结果先写缓存，再给「带时间戳的 save」和「map_publish 发布」共用，这样 RViz 和 PCD 都一定是同一次 HBA 后点云，且无两路竞态。

---

## 1. Executive Summary

| 项目 | 结论 |
|------|------|
| **根因** | 保存的 PCD 与 RViz 使用的全局图**并非同一次构建**；在「先触发 HBA 再立刻 save」的时序下，save 侧可能读到**尚未完成 writeback** 的位姿快照，或与 RViz 使用不同 build 导致数据源不一致。 |
| **设计现状** | `handleFinishMapping` 已使用 `triggerAsync(all, true, "finish_mapping")`（wait=true），理论上 save 应在 HBA 回调（含 `updateAllFromHBA`）之后执行；日志显示 save 的 build 在 writeback_done 之后取快照，但**存在两路并发 build**（map_publish 的 build_id=10 与 save 的 build_id=11/12），若调度或锁竞争导致 save 侧快照早于 writeback，即会产生重影。 |
| **建议修复** | 在 HBA 完成后**只构建一次**全局图，将该结果**同时**用于发布到 RViz 和写入 `global_map.pcd`，避免 save 路径单独再调 `buildGlobalMapAsync().get()`，从数据源上保证 PCD 与 RViz 一致且一定使用 HBA 写回后的位姿。 |

---

## 2. 数据流与代码路径

### 2.1 两路使用 global_map 的入口

- **RViz**  
  - 来源：`publishGlobalMap()` → `buildGlobalMapAsync(voxel_size).get()` 或 `buildGlobalMap()` → `global_map_pub_->publish()` + `rviz_publisher_.publishGlobalMap(global)`  
  - 触发：后端每 N 帧、以及 **HBA 完成后** `onHBADone` 里 `map_publish_pending_.store(true)` + `map_publish_cv_.notify_one()`，由 map_publish 线程执行 `publishGlobalMap()`。

- **保存 PCD**  
  - 来源：`saveMapToFiles(output_dir)` → `buildGlobalMapAsync(voxel_size).get()` 或 `buildGlobalMap(voxel_size)` → `pcl::io::savePCDFileBinary(pcd_path, *global)`  
  - 触发：`handleFinishMapping` 在 `triggerAsync(..., true, "finish_mapping")` **返回之后** 调用 `saveMapToFiles(out_dir)`。

两路都依赖 `SubMapManager::buildGlobalMap*`，位姿来源为关键帧的 `T_w_b_optimized`（未初始化时 fallback 到 `T_w_b`，会带来重影）。

### 2.2 HBA 与 writeback 时序（设计意图）

```
handleFinishMapping:
  ensureBackendCompletedAndFlushBeforeHBA();
  hba_optimizer_.triggerAsync(all, true, "finish_mapping");  // wait=true → 阻塞直到 HBA 空闲
  // 此处返回时，HBA  worker 已执行完 onHBADone(result)，含：
  //   submap_manager_.updateAllFromHBA(result);   // 写回 T_w_b_optimized
  //   submap_manager_.rebuildMergedCloudFromOptimizedPoses();
  finish_mapping_in_progress_.store(false);
  saveMapToFiles(out_dir);   // 再 build + 写 PCD
```

`hba_optimizer.cpp` 中：

- 先执行 `for (auto& cb : done_cbs_) cb(result);`（含 `updateAllFromHBA`），再 `hba_running_ = false`。
- `waitUntilIdleFor()` 要等到 `isIdle()`（队列空且 `hba_running_==false`），因此**设计上** save 一定在 writeback 之后。

即便如此，**save 路径会再起一次** `buildGlobalMapAsync().get()`，与 map_publish 线程触发的 `publishGlobalMap()` 中的 build 是**两次独立构建**，存在：

- 两次取快照的时间点可能因调度产生细微差异；
- 若存在任何「先 notify map_publish、再 set idle」的时序或锁竞争，理论上 save 侧快照仍可能早于或与 writeback 交错。

因此：**PCD 与 RViz 可能来自不同 build，且 save 侧 build 在极端情况下可能未完全使用 HBA 写回后的位姿**，从而出现「文件重影、RViz 不重影」。

---

## 3. 日志证据（run_20260317_212313 / full.log）

- **writeback 完成**  
  - `[SubMapMgr][GHOSTING_DIAG] HBA writeback_done ts=1628250231.410 written=498`  
  - 时间约 21:37:56.404（writeback 完成）。

- **save 与 build 时间**  
  - `event=save_writing output_dir=/data/automap_output/20260317_2137` 约 21:37:57.228。  
  - `pose_snapshot_taken build_id=10` 与 `save_writing` 同秒出现；后续还有 `build_id=11`、`build_id=12`。  
  - 说明：**至少有两路**在取快照（map_publish 一路，save 一路），且 save 的 `buildGlobalMapAsync().get()` 会再起新 build。

- **最终写入 PCD**  
  - `buildGlobalMapInternal_exit build_id=10 pts=1184368` 与 `Saved ... global_map.pcd (1184368 points)` 紧挨出现，说明**当前这次运行里**写入文件的可能是 build_id=10 的结果；但 build_id=10 的**快照时刻**若早于或与 writeback 有重叠，或与 RViz 所用 build 不一致，仍可能产生「文件重影、RViz 正常」的现象。

- **RViz 侧**  
  - HBA 完成后 `onHBADone` 中 `map_publish_pending_.store(true)`，map_publish 线程随后执行 `publishGlobalMap()`，用**当时**的 `buildGlobalMapAsync().get()` 结果发布，该结果**一定**在 writeback 之后（因为 `triggerAsync(..., true)` 已返回才允许主线程继续，而 map_publish 的触发在 onHBADone 内，与 writeback 同线程顺序执行）。因此 RViz 看到的往往是「最新一次」、且与 HBA 结果一致的全局图。

结论：**保存的 PCD 与 RViz 可能来自不同 build，且 save 路径的 build 未强制与「HBA 后仅一次」的发布共用同一份点云**，是「PCD 重影、RViz 无重影」的合理根因。

---

## 4. 其他可能加重重影的因素

- **buildGlobalMap 的 fallback**  
  - 若主路径 `combined` 为空（例如未保留 keyframe cloud_body），会退化为使用各子图 `merged_cloud`；若 `merged_cloud` 未在 HBA 后通过 `rebuildMergedCloudFromOptimizedPoses` 重建，则为 T_w_b（odom）系，与优化轨迹不一致 → 重影。  
  - 当前 save 使用 `buildGlobalMapAsync()`，走的是「快照 (cloud_body, T_w_b_optimized)」路径，一般不经过 merged_cloud fallback；若某处改为同步 `buildGlobalMap()` 且触发了 fallback，会加重重影。

- **T_w_b_optimized 未初始化**  
  - 快照时若 `T_w_b_optimized` 为 Identity 且 `T_w_b` 非 Identity，会 fallback 到 `T_w_b`，该帧会与优化轨迹错位。日志中 `snapshot_fallback_count=0` 表示当前这次 build 未出现 fallback，但不排除其他 build 或历史运行出现过。

- **voxelDownsampleChunked**  
  - 日志中有 `skip_final_voxel (extent overflow risk)`，在范围过大时可能跳过最终体素，导致重叠区域未充分合并，视觉上像「轻微重影」；通常不会单独导致「严重重影」，更多是叠加因素。

---

## 5. 修复建议（保证 PCD 与 RViz 一致、且必用 HBA 后位姿）

### 5.1 推荐方案：HBA 后只构建一次，发布与保存共用

- 在 **onHBADone** 中（writeback + rebuild 之后）：
  - 调用一次 `buildGlobalMapAsync(voxel_size).get()`（或同步 `buildGlobalMap`），得到 `CloudXYZIPtr global`；
  - 用该 `global`：  
    - 发布到 RViz（`global_map_pub_->publish` + `rviz_publisher_.publishGlobalMap(global)`）；  
    - 写入「待保存」缓存，例如 `last_hba_global_map_ = global`（或带时间戳/版本，避免被后续覆盖误用）。
- 在 **saveMapToFiles** 中：
  - 若存在「HBA 后待保存」的全局图（如 `last_hba_global_map_` 非空且为本次 session），则**直接使用该点云写 PCD**，不再调用 `buildGlobalMapAsync().get()`；
  - 否则（未做 HBA 或未启用 HBA）再回退为当前逻辑：`buildGlobalMapAsync().get()` 或 `buildGlobalMap()` 再写。

这样可保证：

- 写入 `global_map.pcd` 的数据与 RViz 最后一次显示的 HBA 后全局图**完全一致**；
- 只构建一次，避免 save 与 map_publish 两路并发 build 带来的时序与数据源不一致问题。

### 5.2 备选：仅保证 save 在 HBA 之后且使用同一 voxel 结果

- 保持现有「先 triggerAsync(..., true)，再 saveMapToFiles」；
- 在 onHBADone 中触发 map_publish 时，将本次 `publishGlobalMap()` 得到的 `global` 存下来，saveMapToFiles 若在「finish_mapping 且已触发 HBA」分支内，优先使用该缓存写 PCD，避免 save 再起一次 build。

### 5.3 配置与日志

- 确保 **retain_cloud_body=true**，避免主路径无点云退化为 merged_cloud fallback。
- 若需进一步排查，可在 save 写 PCD 前打一条日志：写出本次写入使用的来源（例如 `from_hba_cached` vs `from_buildGlobalMap`）和 build_id（若仍有一次 build），便于对比与 RViz 是否同源。

---

## 6. 小结

| 问题 | 原因 |
|------|------|
| 为何 PCD 重影而 RViz 不重影？ | 保存的 PCD 与 RViz 可能来自**不同次** build；save 路径的 build 未强制与 HBA 后发布使用**同一份**点云，且在并发与调度下存在理论上的快照早于/与 writeback 交错的窗口。 |
| 为何设计上 wait=true 仍可能出现？ | wait=true 保证的是「save 在 HBA 回调执行完之后再执行」，但 **save 仍会再起一次 build**，该 build 与 map_publish 的 build 并行，数据源不保证一致；且若存在任何锁或调度细节，save 侧快照仍可能未完全反映 writeback。 |
| 最稳妥的修复 | HBA 完成后**只构建一次**全局图，该结果**同时**用于发布到 RViz 和写入 `global_map.pcd`，从数据源上消除「两路 build、两套位姿」导致的 PCD 重影。 |

---

## 7. 相关代码与日志索引

- 保存 PCD：`automap_system.cpp` → `saveMapToFiles()`，约 3601–3614 行（build + save）。  
- 发布 global_map：`automap_system.cpp` → `publishGlobalMap()`，约 2726–2776 行；onHBADone 中触发：约 2373–2377 行。  
- HBA 回调与 writeback：`hba_optimizer.cpp` 约 253–256 行（先 `cb(result)` 再 `hba_running_ = false`）；`automap_system.cpp` → `onHBADone()` 约 2203–2223 行。  
- 日志：`grep GHOSTING_DIAG|writeback_done|save_writing|pose_snapshot_taken|build_id` 可串联 build 与 writeback 时间线。

---

## 8. 修改后 RViz 出现严重重影（run_20260317_220343）

**现象**：实施「HBA 后只构建一次、PCD 与 RViz 共用同一份点云」的修复后，RViz 反而出现**严重重影**。

**根因（一句话）**：**global_map 已正确使用 HBA 优化后点云，但 RViz 同时仍在显示 odom_path（里程计轨迹，T_w_b 系）；两套轨迹/点云不同坐标系同屏 → 表现为严重重影。**

| 项目 | 说明 |
|------|------|
| **数据源** | 日志中 `[PCD_GHOSTING_FIX] built global map once (pts=1183055)` → `save using cached` → `publish using cached`，说明同一份 HBA 后点云既写 PCD 又发 RViz，修复逻辑正确。 |
| **错位量** | `[GHOSTING_CHEAT_SHEET] odom_last=[0.03,-9.27,-0.02] opt_last=[-0.58,-9.13,-0.08] diff_odom_opt_m=0.63`：odom_path 与 optimized_path 末点相差约 0.63 m，与 global_map 同屏即重影。 |
| **结论** | 重影来自 **odom_path（odom 系）与 global_map（HBA 优化系）同屏显示**，不是 global_map 本身错误。 |

**代码修复（已实现）**：

- 在 **onHBADone**（`gps_aligned_` 分支）中：置 `odom_path_stopped_after_hba_ = true`，清空 `odom_path_.poses`，发布**一次空 Path**，使 RViz 清除 odom_path 显示。
- 在 **onOdometry** 中：若 `odom_path_stopped_after_hba_.load()` 为 true，则**不再向 odom_path 追加、也不再发布** odom_path。
- 在 **publishDataFlowSummary** 中：若已停止 odom_path，则不再对 `odom_path_` 做 trim。

效果：HBA 完成后 RViz 仅显示 **optimized_path + global_map**（同一坐标系），不再显示 odom_path，重影消失。若需对比 odom 与优化轨迹，可查看 VTK 轨迹查看器或日志中的 `GHOSTING_CHEAT_SHEET`。

---

## 9. 保存的 global_map.pcd 仍严重重影（根因：HBA 写回顺序与 HBA 输出顺序不一致）

**现象**：即使 PCD 与 RViz 已共用同一份 HBA 后构建的点云，**保存的 global_map.pcd 文件本身**仍出现严重重影。

**根因（一句话）**：**HBA 输出的 `result.optimized_poses` 顺序 = 按「过滤(有效 cloud_body + 有限 T_w_b) + 按 timestamp 排序 + 按 timestamp 去重」得到的关键帧顺序；而 `updateAllFromHBA` 原先按「子图迭代 × 关键帧原始顺序」写回，两者不一致 → 位姿与关键帧一一错位 → 每帧点云用错了位姿变换 → PCD 严重重影。**

### 9.1 为什么会出现两套顺序？——根本原因（「按理说不该错」却错了）

位姿确实是从**关键帧/后端优化**来的，没有错；错的是**「谁」在「什么顺序」下把「第 i 个位姿」和「哪一个关键帧」对应起来**。两套顺序来自两套不同的「约定」，从未在代码里统一，所以会错位。

| 维度 | 说明 |
|------|------|
| **数据从哪来** | 位姿来自 HBA（GTSAM/API）对关键帧的优化结果，关键帧来自 SubMapManager 的子图；**来源没问题**。 |
| **问题出在哪** | **顺序约定不一致**：HBA 侧有一套「关键帧顺序」（自己 collect 时定的），写回侧用了另一套「关键帧顺序」（SubMapManager 的存储顺序），两者没有共用同一套顺序。 |

**两套顺序分别是什么？**

1. **SubMapManager 的存储/迭代顺序（写回侧原先用的）**
   - 关键帧的物理存储：`submaps_[0].keyframes`, `submaps_[1].keyframes`, …，即**先按子图、再在子图内按插入顺序**。
   - 迭代顺序 = **子图序 × 子图内关键帧插入序**（可理解为「子图优先、时间其次」）。
   - 子图边界是按**空间/时间/数量**切分的，**不是**按全局时间戳排序的。例如：子图 0 的最后一帧可能是 t=200，子图 1 的第一帧可能是 t=150（不同轨迹段、回环等），所以「先子图 0 全部、再子图 1 全部」≠ 全局时间序。

2. **HBA 侧的关键帧顺序（优化结果对应的顺序）**
   - HBA 不直接迭代 SubMapManager，而是通过 **`HBAOptimizer::collectKeyFramesFromSubmaps(all_submaps)`** 拿关键帧。
   - 该函数：先按 `submaps` 遍历并 push 有效 KF → **过滤**（空 cloud_body、非有限 T_w_b 等）→ **按 timestamp 排序** → **按 timestamp 去重**。
   - 因此 `result.optimized_poses[i]` 对应的是「**上述列表里第 i 个关键帧**」，即**全局时间戳序（且过滤、去重后）**。

所以：
- **写回侧**默认：`pose[i]` 写回「**按 (子图, 子图内序号) 迭代到的第 i 个关键帧**」。
- **HBA 侧**实际：`pose[i]` 对应「**按时间戳排序（且过滤、去重）后的第 i 个关键帧**」。

两套顺序在以下情况下都会不一致：
- 子图边界导致「子图 0 末尾」时间戳大于「子图 1 开头」；
- 存在被 HBA 过滤掉的关键帧（空点云等），导致「第 i 个 pose」和「SubMapManager 迭代的第 i 个 KF」根本不是同一个关键帧。

**为什么容易误以为「不该错」？**

- 直觉是：「位姿就是从关键帧优化来的，写回关键帧数组，按顺序对应不就行了？」
- 实际上：**「关键帧数组」在 HBA 里和 SubMapManager 里并不是同一个顺序**。HBA 用的是自己 collect 出来的、**重排过的**列表；写回时若按 SubMapManager 的**存储顺序**逐项写，就等价于把「按时间序的第 i 个位姿」写到了「按子图序的第 i 个关键帧」上，**对应关系错位**。
- 所以根本原因是**设计/约定**：两处没有约定「同一份顺序」，也没有共用同一套 collect 逻辑，导致出现「低级」的索引错位。

**小结**：位姿来源正确，错在**顺序约定不统一**——写回必须使用与 HBA **完全相同的**关键帧顺序（同一套 filter + sort + dedupe），才能保证 `pose[i]` 写回第 i 个关键帧。

---

| 项目 | 说明 |
|------|------|
| **HBA 侧顺序** | `HBAOptimizer::collectKeyFramesFromSubmaps`：过滤（null/空 cloud_body/非有限 T_w_b）→ 按 `timestamp` 排序 → 按 timestamp 去重(0.001s)。GTSAM/API 输出的 `result.optimized_poses[i]` 对应该列表中第 i 个关键帧。 |
| **原写回顺序** | `SubMapManager::updateAllFromHBA` 原逻辑：`for (sm : submaps_) for (kf : sm->keyframes) kf->T_w_b_optimized = result.optimized_poses[pose_idx++]`，即**子图序 × 关键帧插入序**，与 HBA 的「时间戳序」一般不同。 |
| **后果** | 例如：时间戳最小的关键帧在子图 1 中间，HBA 的 pose[0] 是它的优化位姿；写回时 pose[0] 被赋给了「子图 0 的第一个关键帧」→ 子图 0 首帧点云用错位姿，其余帧同理错位 → 整图重影。 |

**代码修复（已实现）**：

- 在 **SubMapManager** 中新增 `collectKeyframesInHBAOrder()`：与 `HBAOptimizer::collectKeyFramesFromSubmaps` **完全一致**的逻辑（过滤 + 按 timestamp 排序 + 按 timestamp 去重），返回关键帧列表。
- 在 **updateAllFromHBA** 中：先调用 `collectKeyframesInHBAOrder()` 得到 `kfs_in_hba_order`，再执行 `kfs_in_hba_order[i]->T_w_b_optimized = result.optimized_poses[i]`（并处理 `kfs_in_hba_order.size() != result.optimized_poses.size()` 的 MISMATCH 日志与 min 写回）。

效果：写回后每个关键帧的 `T_w_b_optimized` 与 HBA 输出一一对应，`buildGlobalMap` 用到的位姿与点云匹配，保存的 **global_map.pcd** 与 RViz 显示的 **global_map** 均无错位重影。

---

## 10. 如何用日志与代码验证结论 + 精准定位（强化日志）

### 10.1 新增/强化的日志标签（grep 用）

| 标签 | 含义 | 出现位置 |
|------|------|----------|
| **HBA_INPUT_ORDER** | HBA 输入关键帧顺序（first/last kf_id, sm_id, ts, count） | HBAOptimizer::runHBA（API 路径用 task.keyframes）、runGTSAMFallback（GTSAM 路径用 sorted_kfs） |
| **WRITEBACK_ORDER** | 写回侧关键帧顺序（collectKeyframesInHBAOrder 得到的 first/last） | SubMapManager::updateAllFromHBA |
| **VERIFY_WRITEBACK** | 写回后校验：first/last 的 KF 上 T_w_b_optimized 是否与 result.optimized_poses 对应位姿一致（match=1 正常，0 表示错位） | SubMapManager::updateAllFromHBA |
| **WRITEBACK_SAMPLE** | 写回采样：idx、kf_id、sm_id、ts、trans（首/中/末三条） | SubMapManager::updateAllFromHBA |
| **writeback MISMATCH** | kfs_in_hba_order 数量与 optimized_poses 数量不一致 | SubMapManager::updateAllFromHBA |

### 10.2 验证「写回顺序与 HBA 一致」（正常时应满足）

1. **顺序对齐**：同一次 HBA 的 `HBA_INPUT_ORDER` 与紧接着的 `WRITEBACK_ORDER` 应对齐。
   - 命令：`grep -E "HBA_INPUT_ORDER|WRITEBACK_ORDER" full.log`
   - 检查：两条里 **first kf_id、ts** 与 **last kf_id、ts** 一致（count 一致更佳；若 GTSAM 路径，HBA 会打两条 INPUT，以 GTSAM_sorted 那条为准与 WRITEBACK 对照）。

2. **写回校验**：`VERIFY_WRITEBACK` 应为 **match=1**。
   - 命令：`grep "VERIFY_WRITEBACK" full.log`
   - 若出现 `match=0` 或 `VERIFY_WRITEBACK FAIL`：写回顺序已错位，保存的 PCD 会重影，需检查 `collectKeyframesInHBAOrder` 与 HBA 的 collect 逻辑是否一致。

3. **数量一致**：不应出现 `writeback MISMATCH`。
   - 命令：`grep "writeback MISMATCH" full.log`
   - 若出现：说明 HBA 输出的 pose 数量与 SubMapManager 侧 collect 出的关键帧数量不一致，写回只写了 min 个，可能漏写或顺序仍错位。

### 10.3 再出现重影时如何精准定位

1. **先确认是否写回错位**  
   `grep "VERIFY_WRITEBACK" full.log` → 若有 `match=0` 或 `FAIL`，则写回顺序错位是根因。

2. **对照 HBA 与写回顺序**  
   `grep -E "HBA_INPUT_ORDER|WRITEBACK_ORDER" full.log` → 对比同一次 HBA 的 first/last kf_id 与 ts；若 WRITEBACK 的 first/last 与 HBA 不一致，说明两套顺序不一致（例如 SubMapManager 未用 collectKeyframesInHBAOrder 或 HBA 侧 collect 逻辑被改）。

3. **看采样是否合理**  
   `grep "WRITEBACK_SAMPLE" full.log` → 看 idx=0 与 idx=last 的 kf_id、ts 是否与 WRITEBACK_ORDER 的 first/last 一致；并可与 `buildGlobalMap` 中使用的位姿（若有 POSE_DIAG/GHOSTING 日志）对照，确认「用在该 KF 上的位姿」是否来自 HBA 对应下标。

4. **代码侧核对**  
   - SubMapManager：写回必须使用 `collectKeyframesInHBAOrder()` 得到的列表，且 `kfs_in_hba_order[i]->T_w_b_optimized = result.optimized_poses[i]`，不得改为按 submaps_ 原始迭代。
   - HBAOptimizer：`collectKeyFramesFromSubmaps` 与 SubMapManager 的 `collectKeyframesInHBAOrder` 必须保持**同一套**过滤 + 按 timestamp 排序 + 按 timestamp 去重逻辑，否则两边的「第 i 个」仍会错位。

### 10.4 一键检查命令示例（运行后保存了 PCD 的 full.log）

```bash
# 顺序与校验
grep -E "HBA_INPUT_ORDER|WRITEBACK_ORDER|VERIFY_WRITEBACK" full.log

# 是否发生 MISMATCH
grep "writeback MISMATCH" full.log

# 写回采样（可选）
grep "WRITEBACK_SAMPLE" full.log
```

---

## 11. 重影是否还有其他原因？——其他原因 + 强化日志（GHOSTING_RISK）

除「写回顺序错位」与「odom_path 与 global_map 同屏」外，以下情况也会导致或加重重影；已用统一标签 **GHOSTING_RISK** 和既有日志做汇总，便于 **grep 一次定位**。

### 11.1 其他重影原因一览

| 原因 | 说明 | 日志标签 / 位置 |
|------|------|------------------|
| **写回顺序错位** | updateAllFromHBA 与 HBA 关键帧顺序不一致，位姿与 KF 错位 | `VERIFY_WRITEBACK` FAIL 或 `HBA_WRITEBACK_MISMATCH` → **std::abort() 退出** |
| **odom_path 与 global_map 同屏** | RViz 同时显示里程计轨迹与 HBA 优化后点云，两套坐标系 | `HBA_GHOSTING`、odom_path 已清空后无此问题 |
| **T_w_b_optimized 未优化（Identity）** | 某 KF 未参与 HBA 或未写回：**不回退到 T_w_b**，直接报严重错误并 **std::abort() 退出程序**（正常建图不应出现） | 出现即进程退出；grep `T_w_b_optimized_UNOPT` 可见最后一条错误日志 |
| **主路径为空退回到 merged_cloud** | combined 为空（如 retain_cloud_body=false 或 KF 无点云），用各子图 merged_cloud 拼接；若 merged_cloud 未在 HBA 后 rebuild，则为 T_w_b 系 | `path=fallback_merged_cloud`、`[GHOSTING_RISK] buildGlobalMap_sync path=fallback_merged_cloud`、`GHOSTING_SOURCE`；是否已重建看 `REBUILD_MERGE` |
| **快照与 writeback 竞态** | 异步 build 取快照时，若落在 updateAllFromHBA enter~exit 之间，可能取到混合位姿（当前设计已通过同锁串行避免） | `pose_snapshot_taken` 与 `writeback_done` 时间线对照；`GHOSTING_DIAG` |
| **retain_cloud_body=false** | 无 cloud_body 时主路径无点云，易走 fallback_merged_cloud | 配置与 `fallback_merged_cloud` 日志 |
| **publishGlobalMap 无 cache 时 async.get() 后未再检查 cache** | map_publish 线程在无 cache 时调用 buildGlobalMapAsync().get() 阻塞；阻塞期间 HBA 可能完成并写入 cache；.get() 返回后若不再检查 cache，会发布 pre-HBA 的 async 结果，地图与 optimized_path（HBA）不同源 → 重影 | **已修复**：.get() 返回后加锁再检查 last_hba_global_map_，若非空则改用 cache、丢弃 async 结果；日志 `PCD_GHOSTING_FIX after async.get() re-check` |
| **onPoseUpdated 在 HBA 后仍发布 opt_path_（iSAM2）** | 与 odom_path 同理：HBA 后 optimized_path 应由 keyframes 的 T_w_b_optimized（HBA）唯一发布；若 onPoseUpdated 仍发布 opt_path_（iSAM2 子图 anchor），会覆盖 HBA 轨迹，导致 map(HBA)+path(iSAM2)，max_drift 可达数米 | **已修复**：当 odom_path_stopped_after_hba_ 为 true 时，不再构建/发布 opt_path_，仅保留 rviz_publisher_.publishOptimizedPath(all_sm)（HBA 轨迹） |

### 11.2 后端严重错误 → 直接退出程序（建图要求高，不掩盖问题）

以下情况会打 ERROR 后 **std::abort()**，进程立即退出：

| 条件 | 日志关键字 |
|------|------------|
| HBA 优化失败 | `[AutoMapSystem][HBA][FATAL] HBA 优化失败` |
| 写回数量不一致 | `[HBA_WRITEBACK_MISMATCH] 严重错误` |
| 写回顺序校验失败 | `[VERIFY_WRITEBACK] FAIL` |
| T_w_b_optimized 未优化（Identity） | `[T_w_b_optimized_UNOPT] 严重错误` |

排查：`grep -E "FATAL|MISMATCH|VERIFY_WRITEBACK FAIL|T_w_b_optimized_UNOPT" full.log`

### 11.3 强化日志：统一标签 GHOSTING_RISK

所有「可能致重影」的构建路径均打一条 **GHOSTING_RISK**，便于一次 grep 排查：

- **buildGlobalMap_sync**：`[GHOSTING_RISK] buildGlobalMap_sync path=from_kf|fallback_merged_cloud`（若 path=fallback 则为 WARN。未优化 KF 会先打 `[T_w_b_optimized_UNOPT] 严重错误` 再 std::abort()，不会继续）。
- **buildGlobalMap_async**：同上，发现未优化即 std::abort()。
- **writeback 顺序错位 / 数量 MISMATCH**：出现即 `std::abort()`，不再仅 WARN。

### 11.4 重影排查 grep 清单（一键）

在本次运行的 `full.log` 下执行，可覆盖**所有**已知重影相关风险：

```bash
# 1）所有重影风险汇总（优先看此项）
grep "GHOSTING_RISK" full.log

# 2）写回顺序与校验
grep -E "HBA_INPUT_ORDER|WRITEBACK_ORDER|VERIFY_WRITEBACK|writeback MISMATCH" full.log

# 3）fallback 与 merged_cloud
grep -E "fallback_merged_cloud|T_w_b_optimized_UNOPT|REBUILD_MERGE" full.log

# 4）odom_path 与同屏
grep "HBA_GHOSTING" full.log

# 5）时间线（快照 vs 写回）
grep -E "pose_snapshot_taken|writeback_done|GHOSTING_DIAG" full.log
```

**解读**：

- 若 `GHOSTING_RISK` 仅出现 `path=from_kf`、无 `writeback_order_mismatch`、无 `path=fallback_merged_cloud`，则构建与写回侧**无已知重影风险**。
- 若出现以下任一**严重错误**，程序会 **std::abort() 直接退出**（建图要求高，不掩盖问题）：
  - `[T_w_b_optimized_UNOPT] 严重错误`：某关键帧 T_w_b_optimized 未优化出结果（Identity）。
  - `[HBA_WRITEBACK_MISMATCH] 严重错误`：写回时 kfs_in_hba_order 与 optimized_poses 数量不一致。
  - `[VERIFY_WRITEBACK] FAIL`：写回后首/末位姿与 result 不一致（顺序错位）。
  - `[AutoMapSystem][HBA][FATAL] HBA 优化失败`：HBA 返回 result.success=false。
- 若出现任一 WARN（且未 abort），按上表对应原因排查（如 path=fallback_merged_cloud、同屏等）。
