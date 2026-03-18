# 卡滞根因分析：run_20260318_172014 full.log

## 1. 现象摘要

- **HEARTBEAT**：`[HEARTBEAT] CRITICAL: threads stuck: map_pub(54s) loop_opt(54s)` 出现在 17:23:13。
- **Backend**：同一时间段 backend 在持续处理（processed_no=52→55，kf_id=19→22），且有 `createKeyFrame_gps_query_enter/exit`，**未**卡在 GPS 查询段（与 run_20260318_162915 不同）。
- **后续**：17:23:17 出现 QUEUE_DRAINED，之后 waiting_for_data；17:24:55 触发 HBA，commitAndUpdate 正常执行。

## 2. 根因结论

### 2.1 为何报 map_pub / loop_opt「stuck 54s」

- **map_pub** 与 **loop_opt** 使用 **无限 wait()**：只有被 backend/onLoopDetected **notify** 后才会唤醒并更新心跳。
- **map_pub** 仅在 `processed_no % map_interval == 0` 时被 backend 触发（map_interval 通常为 10）。若**前 54 秒内 backend 未处理到 10、20、30… 帧**（例如前几帧极慢、或首帧建图/首关键帧耗时很长），则 map_pub 从未被 notify，心跳一直停留在**线程启动时**，超过 30s 即被报 ERROR「stuck 54s」。
- **loop_opt** 同理：无回环时一直 wait()，心跳不更新，被误报为卡死。

因此本次日志中的「map_pub(54s) loop_opt(54s)」**不是**两线程死锁，而是**长时间等工、未收到任何一次 notify**，导致心跳未更新，被心跳检测误判为卡死。

### 2.2 与 run_20260318_162915 的差异

| 项目 | run_20260318_162915 | run_20260318_172014 |
|------|---------------------|----------------------|
| 卡点 | backend 卡在 `createKeyFrame_gps_query_enter` 内（等 GPSManager mutex） | backend 未卡；持续有 gps_query_exit 与后续 STEP |
| map_pub/loop_opt | 因 backend 卡住而无新进度，属连锁反应 | 因长时间未被 notify，心跳未更新，属**误报** |

## 3. 已做修改（精准定位 + 避免误报）

### 3.1 map_pub / loop_opt 使用 wait_for(5s)

- **mapPublishLoop**：`wait()` 改为 `wait_for(lock, 5s, predicate)`；每次唤醒（无论超时或 notify）后**立即更新心跳**。
- **loopOptThreadLoop**：同样改为 `wait_for(5s)` + 唤醒后更新心跳。
- **效果**：无工作时每 5s 超时唤醒并更新心跳，不再因「等工」被报 CRITICAL；若**仍**报 stuck，则说明 5s 内未唤醒，多半卡在 **publishGlobalMap()** 或 **addLoopFactor()** 内部，便于缩小范围。

### 3.2 增加「每行」级日志便于卡住时定位

- **publishGlobalMap**：  
  - `[MAP][LINE] step=enter file=... line=...`  
  - `step=before_last_hba_lock`、`step=buildGlobalMap_enter`、`step=buildGlobalMap_sync_enter/exit`、`step=buildGlobalMapAsync_get_enter/exit`，均带 `file`/`line`。  
  - 卡住时**最后一条 [MAP][LINE]** 即卡点所在行。
- **Backend 触发 map 发布**：  
  - `[BACKEND][MAP_PUB_REQ] processed_no=%d map_interval=%d step=about_to_notify_map_pub`，用于确认 backend 是否曾触发过 map 发布；若长期无此条而 map_pub 报 stuck，则多为 backend 未跑到 map_interval 条件（前段过慢）。

### 3.3 HEARTBEAT / STUCK_DIAG 说明

- 当报 threads stuck 时，额外打印：  
  `map_pub/loop_opt 使用 wait_for(5s)：若仍报 stuck 表示未在 5s 内唤醒，可能卡在 publishGlobalMap/addLoopFactor 内；grep MAP_PUB_REQ 看 backend 是否触发过 map 发布`。

## 4. 再次卡滞时的排查命令

```bash
# 最后几条 BACKEND STEP 与 MAP_PUB_REQ
grep -E '\[BACKEND\]\[STEP\]|\[BACKEND\]\[MAP_PUB_REQ\]' logs/run_*/full.log | tail -30

# 若怀疑 map_pub 卡在 publishGlobalMap 内：最后一条 [MAP][LINE]
grep '\[MAP\]\[LINE\]' logs/run_*/full.log | tail -10

# HEARTBEAT 与 STUCK_DIAG
grep -E 'HEARTBEAT|STUCK_DIAG' logs/run_*/full.log

# map_pub 是 timeout 唤醒还是 notify 唤醒（wait_for 后）
grep 'MAP_PUB.*wait_done' logs/run_*/full.log | tail -5
```

## 5. 总结

| 项目 | 结论 |
|------|------|
| **本次「卡滞」性质** | map_pub/loop_opt 为**误报**：长时间等工、未被 notify，心跳未更新。 |
| **根因** | 使用无限 wait()，且前 ~54s 内 backend 可能未达到 map_interval 的触发次数，导致两线程从未被唤醒。 |
| **代码修改** | wait_for(5s) + 唤醒即更新心跳；增加 [MAP][LINE] 与 [MAP_PUB_REQ] 日志；STUCK_DIAG 说明 wait_for 含义与 grep 建议。 |
| **与 162915 区别** | 162915：backend 卡在 GPS 查询（mutex）；172014：backend 正常，map_pub/loop_opt 为等工误报。 |

后续若再次出现 HEARTBEAT 报 map_pub/loop_opt stuck，可先查 **MAP_PUB_REQ** 与 **MAP][LINE**，区分「backend 未触发」与「卡在 publishGlobalMap 某行」。
