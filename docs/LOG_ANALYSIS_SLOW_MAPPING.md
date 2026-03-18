# 建图慢问题：日志与代码对照分析

## 1. 如何用日志精确定位卡住阶段

### 1.1 后端卡在哪一步

- **关键日志**：`[AutoMapSystem][BACKEND][STEP]`、`[SubMapMgr][STEP]`  
  每条 STEP 均带 **`file=... line=...`**，可直接对应到源码**具体行**（该 log 所在行；卡住时「下一行」即为可能阻塞的代码）。

- **定位方法**：在 full.log 中执行  
  `grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' full.log | tail -20`  
  看**最后一条**的 `step=` 与 **`file=`、`line=`**，即可定位到**文件和行号**（见下方「细粒度 STEP 与代码对应」）。

### 1.2 系统资源低但建图慢的典型原因

1. **后端单线程阻塞**  
   只有一条 backend 线程处理关键帧；若某一步（如子图合并体素、createKeyFrame 内体素、或同步回环）耗时很长，CPU 利用率会很低且建图变慢。

2. **子图合并体素无超时**  
   `SubMapManager::mergeCloudToSubmap` 在合并前后对 `merged_cloud` 做 `voxelDownsample`（>20 万点触发），无超时。点云很大时会调用 `voxelDownsampleChunked`，可能耗时数十秒，直接卡住 backend。

3. **intra_loop 队列满**  
   日志出现 `[STUCK_DIAG] intra_loop_task_queue full` 表示 intra_loop 任务队列已满（默认 max=8），本帧任务被丢弃。说明 **intra_loop_worker** 或下游 **opt_worker** 消费不过来（如 ISAM2 过慢），导致队列积压。

4. **背压与 ingress 超时**  
   `[FEEDER][TIMEOUT] backpressure`、`[INGRESS][TIMEOUT]` 表示 feeder 或 callback 在等队列空位/数据时发生等待超时，通常是因为 backend 处理太慢。

---

## 2. 细粒度 STEP 与代码/功能对应（精确定位到行或功能）

**用法**：卡住时在 full.log 中执行 `grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' full.log | tail -20`，看**最后一条**的 `step=` 与 **`file=`、`line=`**，即可对应到**源码具体行**（log 中的 line 即该条日志所在行；若卡住，实际阻塞的代码多为该行或下一行）。下表为 step 与功能/文件的对应。

| 最后一条 step= | 卡住位置（功能/代码） | 文件与函数 |
|----------------|----------------------|------------|
| `tryCreateKeyFrame_enter` | 即将进入 tryCreateKeyFrame，下一段为 createKeyFrame 或 addKeyFrame | automap_system.cpp backend 循环 |
| `createKeyFrame_enter` | 即将调用 kf_manager_.createKeyFrame | automap_system.cpp tryCreateKeyFrame() |
| `createKeyFrame_voxel_enter` | createKeyFrame 内体素下采样（无预降采样时） | automap_system.cpp createKeyFrame() → voxelDownsampleWithTimeout |
| `createKeyFrame_voxel_exit` | 体素已完成，下一段为 GPS 查询 | — |
| `createKeyFrame_gps_query_enter` | createKeyFrame 内 GPS 查询（queryByNearestPosition/queryByTimestampEnhanced） | automap_system.cpp createKeyFrame() → gps_manager_.query* |
| `createKeyFrame_gps_query_exit` | GPS 查询已完成 | — |
| `createKeyFrame_exit` | createKeyFrame 刚返回，下一段为 addKeyFrame | automap_system.cpp tryCreateKeyFrame() |
| `about_to_lock_keyframe_mutex` | **即将获取 keyframe_mutex_**；若此为最后一条则卡在锁等待（其他线程持锁） | automap_system.cpp tryCreateKeyFrame() 紧贴 std::lock_guard 前一行 |
| `addKeyFrame_enter` | 即将调用 submap_manager_.addKeyFrame | automap_system.cpp tryCreateKeyFrame() |
| `submap_addKeyFrame_enter` | 即将进入 SubMapManager::addKeyFrame | submap_manager.cpp addKeyFrame() 入口 |
| `addKeyFrame_enter` (SubMapMgr) | addKeyFrame 已持锁，准备写 kf_id / 创建子图 / 合并 | submap_manager.cpp addKeyFrame() |
| `mergeCloudToSubmap_enter` | 即将调用 mergeCloudToSubmap | submap_manager.cpp addKeyFrame() → mergeCloudToSubmap() |
| `merge_pre_voxel_enter` | mergeCloudToSubmap 内**合并前**体素下采样（merged_cloud>20 万点） | submap_manager.cpp mergeCloudToSubmap() → voxelDownsampleWithTimeout |
| `merge_pre_voxel_call_enter` | 下一行即 voxelDownsampleWithTimeout（卡住则在此调用） | submap_manager.cpp mergeCloudToSubmap() |
| `merge_pre_voxel_exit` | 合并前体素已完成 | — |
| `merge_transform_enter` | mergeCloudToSubmap 内 **pcl::transformPointCloud**（body→world） | submap_manager.cpp mergeCloudToSubmap() |
| `merge_transform_pcl_enter` | 下一行即 pcl::transformPointCloud（卡住则在此调用） | submap_manager.cpp mergeCloudToSubmap() |
| `merge_transform_exit` | 变换已完成 | — |
| `merge_append_enter` | mergeCloudToSubmap 内 **for 循环 push_back** 合并点云 | submap_manager.cpp mergeCloudToSubmap() |
| `merge_append_loop_enter` | 下一行即 for push_back 循环（卡住则在此循环） | submap_manager.cpp mergeCloudToSubmap() |
| `merge_append_loop_exit` | push_back 循环已结束 | — |
| `merge_append_exit` | 合并 append 已完成 | — |
| `merge_post_voxel_enter` | mergeCloudToSubmap 内**合并后**体素下采样（merged_cloud>20 万点） | submap_manager.cpp mergeCloudToSubmap() → voxelDownsampleWithTimeout |
| `merge_post_voxel_call_enter` | 下一行即 voxelDownsampleWithTimeout（卡住则在此调用） | submap_manager.cpp mergeCloudToSubmap() |
| `merge_post_voxel_exit` | 合并后体素已完成 | — |
| `mergeCloudToSubmap_exit` | mergeCloudToSubmap 已返回 | submap_manager.cpp addKeyFrame() |
| `addKeyFrame_before_freeze_check` | 合并完成，即将检查 isFull / freeze | submap_manager.cpp addKeyFrame() |
| `freeze_enter` | 即将调用 freezeActiveSubmap（子图已满） | submap_manager.cpp addKeyFrame() → freezeActiveSubmap() |
| `freeze_exit` | freezeActiveSubmap 已返回 | — |
| `addKeyFrame_exit` (SubMapMgr) | addKeyFrame 正常返回 | submap_manager.cpp addKeyFrame() |
| `submap_addKeyFrame_exit` | submap_manager_.addKeyFrame 已返回 | automap_system.cpp tryCreateKeyFrame() |
| `addKeyFrame_exit` (BACKEND) | 整段 addKeyFrame 完成，下一段为 intra_loop | automap_system.cpp tryCreateKeyFrame() |
| `intra_loop_enter` / `intra_loop_exit` | 子图内回环投递或同步执行 | automap_system.cpp tryCreateKeyFrame() |

**按行号定位**：每条 STEP 日志都带 `file=... line=...`，例如：
```text
[AutoMapSystem][BACKEND][STEP] step=tryCreateKeyFrame_enter processed_no=56 ts=123.456 file=/path/to/automap_system.cpp line=1171
```
则卡住时若最后一条是上述内容，说明尚未执行到 line 1171 之后的逻辑（例如下一行可能是 `about_to_lock_keyframe_mutex` 或锁等待）。用 `grep "line=" full.log` 可快速筛出所有带行号的 STEP。

**grep 示例**：
```bash
grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' full.log | tail -20
# 只看带 file/line 的 STEP（定位到具体行）
grep -E 'file=.*line=' full.log | grep -E 'STEP|SubMapMgr' | tail -20
```

---

## 3. 日志与代码对照表（卡住阶段，简表）

| 日志特征 | 卡住位置 | 代码位置 |
|----------|----------|----------|
| 最后 STEP=submap_addKeyFrame_enter，无 exit | SubMapManager::addKeyFrame 内部 | submap_manager.cpp addKeyFrame() |
| 最后 STEP=merge_pre_voxel_enter，无 exit | 合并前体素下采样 | submap_manager.cpp mergeCloudToSubmap() 合并前 voxelDownsampleWithTimeout |
| 最后 STEP=merge_post_voxel_enter，无 exit | 合并后体素下采样 | submap_manager.cpp mergeCloudToSubmap() 合并后 voxelDownsampleWithTimeout |
| 最后 STEP=freeze_enter，无 exit | 子图冻结 | submap_manager.cpp freezeActiveSubmap() |
| STUCK_DIAG intra_loop_task_queue full | 队列满，任务被丢 | automap_system.cpp，worker_threads.cpp intra_loop_worker |
| FEEDER backpressure / TIMEOUT | feeder 等 frame_queue 空位 | automap_system.cpp 约 840–869 |
| INGRESS TIMEOUT | 回调等 ingress 队列 | automap_system.cpp 约 665–708 |

---

## 4. 所有“计算超时”的日志（便于 grep）

**约定**：凡发生**计算超时**（含等待超时导致丢帧/降级），都会在日志中打出包含 **`[TIMEOUT]`** 或 **`[COMPUTE_TIMEOUT]`** 的行，便于统一检索。

建议在 full.log 中执行：

```bash
grep -E '\[TIMEOUT\]|\[COMPUTE_TIMEOUT\]' full.log
```

### 4.1 各模块超时日志位置

| 模块 | 日志 Tag | 含义 | 代码位置 |
|------|----------|------|----------|
| FEEDER | `[FEEDER][TIMEOUT]` | 体素下采样超时 / 背压等待超时 | automap_system.cpp 约 830、869 |
| INGRESS | `[INGRESS][TIMEOUT]` | 回调入队等待超时、队列满丢帧 | automap_system.cpp 约 679、698、704、708 |
| INTRA_LOOP | `[INTRA_LOOP][TIMEOUT]` | 子图内回环检测超时或上一帧未完成跳过 | automap_system.cpp 约 1623、1637 |
| SubMapMgr | `[SubMapMgr][TIMEOUT]` | 子图合并体素下采样超时 | submap_manager.cpp mergeCloudToSubmap（已增强） |
| BACKEND createKeyFrame | `[AutoMapSystem][BACKEND][TIMEOUT]` | createKeyFrame 内体素下采样超时 | automap_system.cpp createKeyFrame（已增强） |
| ISAM2 | `[ISAM2_DIAG][TIMEOUT]` / `[IncrementalOptimizer][BACKEND][TIMEOUT]` | 因 pending 超时强制 update / waitForPendingTasks 超时 | incremental_optimizer.cpp 约 1626、1668、2761 |
| LOOP | `[LOOP_STEP][TIMEOUT]` / `[AutoMapSystem][LOOP][TIMEOUT]` | 回环因子队列满丢约束 | automap_system.cpp 约 2067、2069 |
| HBA | `[HBAOptimizer][TIMEOUT]` | HBA 等待超时强制结束 | hba_optimizer.cpp 约 157 |
| FrameProcessor | `[COMPUTE_TIMEOUT]` | 体素下采样超时（ALOG） | frame_processor.cpp（已增强） |
| Utils | ALOG 含 "TIMEOUT" | voxelDownsampleWithTimeout 超时 | utils.cpp 约 244 |

若某次运行建图很慢，可先查是否有上述任意一条超时日志，再结合 `BACKEND STEP` 最后一步判断是哪一个环节阻塞。

---

## 5. 已做的增强（满足“计算超时必记日志”）

- **SubMapManager::mergeCloudToSubmap**：对合并前/后体素下采样改为带超时的 `voxelDownsampleWithTimeout`，超时时打 **`[SubMapMgr][TIMEOUT]`**。
- **createKeyFrame**：当未提供预降采样点云时，体素下采样改为带超时的 `voxelDownsampleWithTimeout`，超时时打 **`[AutoMapSystem][BACKEND][TIMEOUT]`**。
- **FrameProcessor**：体素超时时增加 **`[COMPUTE_TIMEOUT]`** 的 ALOG 行，便于 grep。

其他已有超时逻辑（FEEDER、INGRESS、INTRA_LOOP、ISAM2、LOOP、HBA）原本就有带 `[TIMEOUT]` 的日志，保持不变即可。

---

## 6. 建议排查顺序

1. `grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' full.log | tail -30` → 看**最后一条** step=，对照上文「细粒度 STEP 与代码/功能对应」表，精确定位到函数/代码段。
2. `grep -E '\[TIMEOUT\]|\[COMPUTE_TIMEOUT\]' full.log` → 看是否有计算/等待超时及发生频率。
3. `grep 'STUCK_DIAG' full.log` → 看队列满、单帧过慢等诊断。
4. 若最后 STEP 是 `merge_pre_voxel_enter` 或 `merge_post_voxel_enter` → 重点看是否有 `[SubMapMgr][TIMEOUT]`，确认是否为子图合并体素超时或阻塞。
