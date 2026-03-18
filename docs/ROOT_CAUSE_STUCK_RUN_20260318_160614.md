# 卡住根因分析：run_20260318_160614 full.log

## 1. 现象摘要

- **表现**：建图在 processed_no=56（frame_no=276）之后不再前进；fast_livo 持续发帧（#500、#1000、#1500…），automap 后端无新 STEP。
- **HEARTBEAT**：`[HEARTBEAT] CRITICAL: threads stuck: map_pub(55s) loop_opt(55s)`。

## 2. 日志定位结论

### 2.1 最后一条 BACKEND STEP

```text
16:10:23.430 [BACKEND][STEP] step=tryCreateKeyFrame_enter processed_no=56 ts=1628249905.209
```

之后**没有**出现：

- `step=createKeyFrame_enter`
- `step=submap_addKeyFrame_enter`
- `step=tryCreateKeyFrame_exit`（processed_no=56）

因此 **backend 线程卡在 tryCreateKeyFrame(56) 内部**，且卡在**尚未执行到 createKeyFrame_enter 日志**的位置。

### 2.2 两种可能

| 情况 | 含义 |
|------|------|
| **A. 阻塞在拿锁** | 卡在 `std::lock_guard<std::mutex> lk(keyframe_mutex_);`，即 **keyframe_mutex_** 被其他线程占用。 |
| **B. 已持锁、卡在 createKeyFrame/addKeyFrame** | 已进入临界区，但卡在 createKeyFrame 或 addKeyFrame 的某一步（体素/合并/子图等），**createKeyFrame_enter 等日志尚未刷出**。 |

代码中 **只有 backend 自己在 tryCreateKeyFrame 内会加 keyframe_mutex_**，没有其他线程加这把锁，因此：

- 若是 **A**，只能是同一 backend 线程在**未释放锁的情况下再次请求同一把锁**（例如错误地在 tryCreateKeyFrame 外又包了一层 keyframe_mutex_），会导致死锁；当前代码看只有一处加锁、且无外层锁，故 **A 概率较低**。
- 更符合日志的是 **B**：backend 已拿到锁，卡在 **createKeyFrame 或 addKeyFrame** 的某段耗时/阻塞逻辑里，且该段在打出 createKeyFrame_enter 之前或日志尚未刷新。

### 2.3 本 log 中 merge 体素的耗时

- kf_id=12：`merge_post_voxel_enter` → `merge_post_voxel_exit` **duration_ms=37.9**，pts=212914。
- kf_id=20：`merge_post_voxel_enter` → `merge_post_voxel_exit` **duration_ms=40.0**，pts=207588。

说明在 merged_cloud 约 20 万点时，**合并后体素**单次就约 40ms；点数再大或触发 pre+post 两次体素时，单帧在 merge 上的耗时可能到数百 ms 甚至更长，与「单帧卡死」现象一致。

## 3. 根因结论（优先解释）

**根本原因**：backend 在 **tryCreateKeyFrame(56)** 内持 **keyframe_mutex_** 期间，卡在以下某一（或组合）耗时路径上：

1. **createKeyFrame**  
   - 若本帧没有可用的预降采样点云（`cloud_ds`），会走 **createKeyFrame 内体素**（voxelDownsampleWithTimeout），点云大时可能很慢。  
   - 或 **kf_manager_.createKeyFrame()** 内部某步（拷贝/建 KF 结构）在特定数据下变慢。

2. **addKeyFrame → mergeCloudToSubmap**  
   - **merge_append**：merged_cloud 已达 14 万+ 时，再 push_back 约 1.6 万点，循环本身可能几十到上百 ms。  
   - **merge_pre_voxel / merge_post_voxel**：当 merged_cloud 超过 20 万（kDownsampleThreshold），会做带 15s 超时的体素；本 log 中 21 万点约 40ms，更大规模或异常情况下可能接近超时甚至阻塞感明显。

3. **HEARTBEAT 中 map_pub(55s) / loop_opt(55s)**  
   - **map_pub**：依赖 backend 每 N 帧设置 `map_publish_pending_`；backend 卡在 56 后不再前进，map_pub 一直 wait，**不会更新心跳** → 显示 55s 未更新。  
   - **loop_opt**：类似地，无新回环任务入队，线程在 wait，**心跳长时间不更新**。  
   因此二者是「因 backend 卡住而导致空闲等待」，并非独立死锁。

## 4. 建议措施

### 4.1 短期（配置/策略）

- **子图合并体素**  
  - 将 **kDownsampleThreshold**（当前 200000）适当**调低**（例如 120000～150000），使更早、更频繁地做 merge 体素，单次点数更小，避免单次 40ms+ 的峰值。  
  - 保持当前 **merge 体素 15s 超时** 与 **SubMapMgr STEP 日志**，便于再次卡住时确认是否仍为 merge 体素。

- **createKeyFrame 体素**  
  - 确保 feeder 预降采样稳定启用，减少「无 cloud_ds」时在 createKeyFrame 内做大点云体素的概率。  
  - 已有 8s 超时与 BACKEND TIMEOUT 日志，若出现 createKeyFrame 体素超时，会打在日志里。

### 4.2 中期（结构）

- **缩短 keyframe_mutex_ 持锁时间**  
  - 仅用 keyframe_mutex_ 保护「创建 KF + 加入子图」的最小序列（与现有设计一致）；确认没有在 tryCreateKeyFrame **外层**再包 keyframe_mutex_，避免同一线程重入死锁。  
  - 若后续将 merge 体素移到锁外（例如单独线程或异步任务），需保证与 submap 写入的线程安全与顺序。

- **map_pub / loop_opt 心跳**  
  - 若使用「无超时 wait」的 map_publish 实现，可在 wait 返回或超时分支里也更新心跳，避免 backend 卡住时被误报为「map_pub 线程 stuck」；或为 wait 增加超时，定期唤醒并更新心跳。

### 4.3 再次卡住时的排查命令

```bash
# 最后 30 条 STEP，精确定位卡在哪一步
grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' full.log | tail -30

# 是否有计算超时
grep -E '\[TIMEOUT\]|\[COMPUTE_TIMEOUT\]' full.log

# HEARTBEAT 与 STUCK 诊断
grep -E 'HEARTBEAT|STUCK_DIAG' full.log
```

若再次出现「仅有 tryCreateKeyFrame_enter(56)、无 createKeyFrame_enter / submap_addKeyFrame_enter」：

- 先确认是否为 **B**：在 createKeyFrame/addKeyFrame 内某步阻塞（体素/merge/子图），并对照 **SubMapMgr STEP** 与 **merge_*_voxel** 的 duration_ms。  
- 若长期无任何 STEP 且无 TIMEOUT 日志，再考虑 **A**（死锁），并检查是否有新代码路径持 keyframe_mutex_ 或与 backend 形成锁环。

---

**总结**：本次卡住是 **backend 在 tryCreateKeyFrame(56) 内持 keyframe_mutex_ 时，卡在 createKeyFrame 或 addKeyFrame 的耗时路径上**（以 merge 体素/合并或 createKeyFrame 体素为主因）；map_pub/loop_opt 的 55s 为随之而来的空闲等待，而非独立死锁。通过降低合并体素阈值、保证 feeder 预降采样、保持超时与 STEP 日志，可复现时更快定位并缓解。

**后续复现**：所有 BACKEND/SubMapMgr STEP 已增加 **`file=... line=...`**，复现时用 `grep -E '\[BACKEND\]\[STEP\]|\[SubMapMgr\]\[STEP\]' full.log | tail -5` 看最后一条的 **file 与 line** 即可定位到**源码具体行**（见 `docs/LOG_ANALYSIS_SLOW_MAPPING.md`）。
