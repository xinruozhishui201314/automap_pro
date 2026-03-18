# GPS 窗口快照：无锁读路径的并发正确性分析

## 1. 设计回顾

- **写路径**：`addGPSMeasurement`、`addKeyFramePose`、`try_align` 等持 `mutex_` 修改 `gps_window_`、`align_result_`、`state_` 等。
- **读路径（快照）**：`getSnapshot()` 持 `mutex_` **仅做一次拷贝**（`gps_window_` → `snap.window`，以及对齐 R/t、配置参数），然后**释放锁**；之后所有计算（插值、最近邻、外推等）均在**锁外**对局部变量 `snap` 进行只读访问。

## 2. 会不会产生冲突？

### 2.1 数据竞争（Data Race）

- **getSnapshot() 内**：持锁期间只有当前线程在读写共享状态；其他线程若调用 `addGPSMeasurement` 或 `try_align` 会阻塞在 `mutex_` 上，不会同时修改 `gps_window_`。因此拷贝过程不存在数据竞争。
- **getSnapshot() 返回后**：返回值 `snap` 是**线程局部的栈上对象**（或拷贝），其他线程无法访问。后续 `queryByTimestampOnSnapshot(ts, snap)`、`enu_to_map_from_snapshot` 等仅读取 `snap` 的只读成员，**没有任何线程会写 snap**。因此锁外读快照不会与任何写操作产生数据竞争。
- **结论**：**不会**出现“读到一个正在被改写的半成品状态”或未定义行为；C++ 内存模型下无数据竞争。

### 2.2 逻辑一致性（快照是否自洽）

- 快照在**同一时刻**拷贝：先拷贝 `gps_window_`，再拷贝 `is_aligned`、`R_gps_lidar`、`t_gps_lidar` 及各类参数。这些都在**同一把锁的一次持锁内**完成，因此得到的是**同一时刻**的窗口与对齐状态。
- 插值/最近邻/外推只用快照内的数据：`queryByTimestampOnSnapshot`、`estimateVelocityOnSnapshot` 仅使用 `snap.window` 和 `snap.*` 参数，不再访问 `gps_window_` 或 `align_result_`。因此一次查询内部**逻辑自洽**，不会出现“窗口是新的、对齐是旧的”等割裂状态。
- **结论**：单次 query 的输入来自**同一逻辑时刻**，计算逻辑正常。

### 2.3 时效性（Staleness）与业务正确性

- **可能现象**：取快照后、本次 query 完成前，其他线程可能执行了 `addGPSMeasurement`（新 GPS 入窗）或 `try_align`（对齐状态变化）。本次 query 结果基于**取快照那一刻**的窗口与对齐状态，即可能比“当前最新”落后一两次更新。
- **对关键帧绑定的影响**：createKeyFrame 时用当前（或略旧）窗口为**该关键帧**选 GPS。略旧的含义是：最多少看到“刚写入还未被本次 getSnapshot 拷贝进去”的那几条。对单帧而言，用“上一瞬间的窗口”做匹配仍然是**合法、一致**的绑定；下一帧会取到新快照。
- **对对齐状态的影响**：若取快照时尚未 ALIGNED，本次不会用按位置最近邻（queryByNearestPosition 返回 null，调用方回退 queryByTimestampEnhanced）；若取快照时已 ALIGNED，本次用到的 R/t 与窗口同属该时刻，一致。不会出现“用 ALIGNED 的 R/t 去配 NOT_ALIGNED 的窗口”等矛盾。
- **结论**：**不会**产生逻辑冲突；至多是**保守的滞后**，保证的是“用同一时刻的窗口+对齐做一次一致查询”，计算逻辑正常。

### 2.4 与“每步都持锁”的等价性

- 原先：query 过程中多次持锁（如 queryByNearestPosition 先锁找 nearest_ts，释放后再锁 queryByTimestamp），且存在持锁时调 `enu_to_map` 导致同一线程二次抢锁的**潜在死锁**。
- 现在：**一次** getSnapshot() 持锁拷贝，之后全在锁外。对**单次 query** 而言，等价于“在 getSnapshot 完成的那一时刻持锁做完了所有读”，再在锁外用这份只读拷贝算完。因此与“在那一时刻持锁做整段 query”的**语义等价**，且避免了长时间持锁与重入死锁风险。

## 3. 总结

| 问题 | 结论 |
|------|------|
| 不加锁（锁外读快照）会不会产生数据竞争？ | **不会**；快照为线程局部只读，无并发写。 |
| 快照是否自洽？ | **是**；同一持锁周期内拷贝，单次 query 输入一致。 |
| 略旧的快照会不会导致逻辑错误？ | **不会**；至多略保守，不会出现不一致或错误绑定。 |
| 计算逻辑是否正常？ | **是**；与“在取快照时刻持锁做整段 query”等价，并消除重入死锁风险。 |

因此，**在保证 getSnapshot() 内拷贝完整、锁外只读快照的前提下，不加锁不会产生冲突，计算逻辑正常。**

---

## 4. 日志与问题精准定位

所有快照/只读路径已加**逐步骤日志**，统一 tag 便于 grep：

| Tag | 含义 | 建议 grep |
|-----|------|-----------|
| `[GPS_SNAPSHOT]` | getSnapshot 持锁拷贝：enter → lock_held → exit（含 window_size, is_aligned, good_sample_count） | 看是否长时间停在 enter 后（等锁）或 exit 前（拷贝慢） |
| `[GPS_QUERY]` | 各 query 的 enter → got_snapshot → 分支（interp/nearest/not_found/extrapolate）→ exit | 看某次调用走到哪一步、以何种 mode 返回 |

**出问题时建议：**

1. 将 log level 设为 **DEBUG**（或至少对 GPSManager 开 DEBUG）。
2. 卡住时：
   - `grep '\[GPS_SNAPSHOT\]' full.log | tail -20`  
     若最后一条是 `getSnapshot enter` 且无后续 `lock_held`/`exit` → 说明**等锁**（其他线程持 mutex_）。
   - `grep '\[GPS_QUERY\]' full.log | tail -50`  
     看最后一次 query 的完整链路：enter → got_snapshot → … → exit；若缺 exit 则卡在该 query 的某一步（通常是 queryByTimestampOnSnapshot 或 estimateVelocityOnSnapshot 内）。
3. 结果异常时：对某 ts 或某 position_map，用 `grep '\[GPS_QUERY\].*ts=...'` 或 `position_map=` 过滤，对照 window_size、is_aligned、mode（interp/nearest/extrapolate）、has_result 判断是窗口不足、未对齐还是时间窗/外推导致。
