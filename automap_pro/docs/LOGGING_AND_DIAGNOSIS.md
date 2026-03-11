# 日志与精准诊断说明

本文档说明关键流水线日志标签、如何根据“最后一条日志”定位崩溃/异常，以及对应修复策略。

---

## 1. 日志标签与流水线步骤

### 1.1 全局地图发布（publishGlobalMap，易崩溃路径）

| 标签 / 关键字 | 含义 | 崩溃时“最后一条”说明 |
|---------------|------|------------------------|
| `[AutoMapSystem][BACKEND] frame_no=... step=publishGlobalMap enter` | 即将发布全局地图 | 崩溃在 publishGlobalMap 内部 |
| `[AutoMapSystem][MAP] publishGlobalMap step=enter` | 进入 publishGlobalMap | 崩溃在取配置或其后 |
| `[AutoMapSystem][MAP] publishGlobalMap step=using_cached_voxel` | 使用缓存的 voxel_size | 崩溃在 buildGlobalMap |
| `[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_enter` | 进入 buildGlobalMap | 崩溃在 SubMapManager::buildGlobalMap |
| `[AutoMapSystem][MAP] buildGlobalMap step=locked_enter` | 准备加锁 | 崩溃在加锁或其后 |
| `[AutoMapSystem][MAP] buildGlobalMap step=locked_done submaps=N` | 锁内，子图数 N | 崩溃在合并或 downsample |
| `[AutoMapSystem][MAP] buildGlobalMap step=sm_add idx=... sm_id=... pts=...` | 正在合并第 idx 个子图 | 崩溃在该子图拷贝或下一子图 |
| `[AutoMapSystem][MAP] buildGlobalMap step=merge_done combined=N` | 合并完成，点数 N | 崩溃在 voxelDownsampleChunked |
| `[AutoMapSystem][MAP] buildGlobalMap step=before_downsample` | 即将体素滤波 | 崩溃在 utils::voxelDownsampleChunked |
| `[Utils] [MAP] voxelDownsampleChunked step=enter` | 进入分块体素滤波 | 崩溃在 sanitize 或其后 |
| `[Utils] [MAP] voxelDownsampleChunked step=bypass_chunked` | 点数≤250k，走单次体素（安全路径） | 若仍崩溃则在 voxelDownsample 内部 |
| `[Utils] [MAP] voxelDownsampleChunked step=chunk_grid nx=...` | 分块网格已算 | 崩溃在 chunk 循环内 |
| `[Utils] [MAP] voxelDownsampleChunked step=chunk ix=... voxel_enter` | 正在对某 chunk 做体素 | 崩溃在该次 voxelDownsample(chunk) |
| `[Utils] [MAP] voxelDownsampleChunked step=chunk_merge_done` | 所有 chunk 已合并 | 崩溃在最终 voxelDownsample(merged) |
| `[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_done` | 全局点云已生成 | 崩溃在 toROSMsg / 发布 / rviz |

### 1.2 数据流与后端

| 标签 | 含义 |
|------|------|
| `[AutoMapSystem][DATA_FLOW]` | 接收/缓存/队列/发布计数汇总 |
| `[AutoMapSystem][BACKEND][RECV]` | 后端收到点云/入队 |
| `[AutoMapSystem][BACKEND][FRAME]` | 本帧处理结果（kf_created / skip_no_kf 等） |
| `[LivoBridge][RECV]` / `[LivoBridge][FRAME]` | 前端点云接收与转发 |

### 1.3 关闭与退出（SHUTDOWN / PIPELINE，便于定位 exit 时 SIGSEGV）

| 标签 / 关键字 | 含义 | 若崩溃在此之后 |
|---------------|------|----------------|
| `[PIPELINE] event=sensor_idle_timeout` | 传感器空闲超时，触发最终 HBA + 保存 + shutdown | 崩溃在保存或 rclcpp::shutdown 路径 |
| `[PIPELINE] event=sensor_idle_save_done` | 空闲触发的 saveMapToFiles 完成 | 随后会调用 rclcpp::shutdown |
| `[AutoMapSystem] Sensor idle: requesting context shutdown` | 请求 ROS 上下文关闭 | 主循环退出，节点析构开始 |
| `[PIPELINE] event=backend_worker_exited` | 后端 worker 线程已退出 | 析构中会 join 并执行 loop_detector_.stop() |
| `[AutoMapSystem][SHUTDOWN][step=1]` | 析构入口，请求 backend 退出 | 崩溃在 join 或之后 |
| `[SHUTDOWN][step=2]` | backend worker 已 join | 崩溃在 loop_detector_.stop() |
| `[LoopDetector][SHUTDOWN] stop() entered` | 回环检测正在停止（join workers） | 崩溃在 LoopDetector 或其后 |
| `[LoopDetector][SHUTDOWN] stop() done` | 回环检测已停止 | 崩溃在 saveMapToFiles 或 hba_optimizer_.stop() |
| `[SHUTDOWN][step=5b] saveMapToFiles done` | 关闭时保存地图完成 | 崩溃在 hba_optimizer_.stop() 或之后 |
| `[SHUTDOWN][step=7] Shutdown complete` | 析构即将结束（即将离开析构函数） | **若随后出现 SIGSEGV**：多为**静态/全局对象析构**（如 PCL FPFHEstimation），与 GDB 的 `exit()` → `~FPFHEstimation()` 栈对应 |

**FPFH 崩溃追踪**：`[CRASH_TRACE]` / `[FPFH_CRASH_TRACE]` 带 `ts_ms=`、`lwp=`，可与系统日志时间戳和 GDB `info threads` 的 LWP 对应，精确定位到 FPFH 计算哪一步。

---

## 2. 精准定位步骤（崩溃时）

### 2.1 运行中崩溃（如地图发布）

1. **保留完整终端输出**，从 `step=publishGlobalMap enter` 到进程退出（含 exit code -11 等）。
2. **找最后一条带 `[AutoMapSystem][MAP]` 或 `[Utils] [MAP]` 的日志**，对照 1.1 表确定崩溃阶段。
3. 若最后是 `step=chunk ix=... voxel_enter`：崩溃在该 chunk 的 PCL 体素滤波；可配合 `--gdb` 拿 backtrace 确认是否在 `pcl::VoxelGrid::applyFilter`。
4. 若最后是 `step=buildGlobalMap_enter` 且无 `locked_enter`：崩溃在 buildGlobalMap 入口（如 logger / 锁）。

### 2.2 退出时崩溃（SIGSEGV 在 exit/析构阶段）

1. **找最后一条带 `[SHUTDOWN]` 或 `[PIPELINE]` 的日志**，对照 1.3 表确定已执行到的步骤。
2. 若最后是 `[SHUTDOWN][step=7] Shutdown complete` 随后 SIGSEGV：崩溃在**静态析构**（如 `libautomap_loop_closure.so` 内 `~FPFHEstimation()` → `free()`），非业务逻辑；可依赖“heap 分配、永不 delete”的 FPFH 修复避免。
3. 用 **`[TRACE] step=... tid=... lwp=...`** 或 **`[FPFH_CRASH_TRACE] ts_ms=... lwp=...`** 与 GDB `info threads` 的 LWP 对应，确认崩溃线程。

---

## 3. 已实施的加固与策略

| 措施 | 说明 |
|------|------|
| **小点云绕过 chunk** | 点数 ≤ 250000 时直接单次 `voxelDownsample`，不走分块循环，避免 M2DGR 等场景在 chunk 循环内 SIGSEGV。 |
| **子图合并上限** | buildGlobalMap 中单子图/合并总点数上限，避免 PCL 或内存异常。 |
| **合并用 reserve+push_back** | 不用 PCL `operator+=`，避免大块重分配导致崩溃。 |
| **chunk 内 try-catch** | 单 chunk 的 `voxelDownsample` 抛异常时记录日志并用该 chunk 原样合并，不中断整体。 |
| **map_voxel_size 缓存** | publishGlobalMap 不再在回调中调用 ConfigManager，避免析构顺序问题。 |

---

## 4. GPS 无数据诊断（轨迹 CSV 无 gps_x/gps_y/gps_z）

当轨迹 CSV 中 GPS 列为空、日志出现 `TRAJ_LOG no GPS` 且 `gps_window_size=0` 时，按下列顺序排查。

### 4.1 根因判断（看日志）

| 日志关键字 | 含义 |
|------------|------|
| `[LivoBridge][GPS] First GPS message` | LivoBridge 已收到至少一条 NavSatFix（可能 status<0 无 fix） |
| `[LivoBridge][GPS] First valid GPS received` | 已收到有效 fix 并转发给 GPSManager |
| `[GPSManager][GPS_DIAG] First GPS measurement added` | GPSManager 已写入第一条测量；若**从未出现**则 LivoBridge 未转发有效 fix |
| `[LivoBridge][GPS_DIAG] Still 0 NavSatFix messages on topic=... after 45s` | 约 45s 内**未收到任何** NavSatFix → 订阅未收到数据 |

**结论**：若整段日志中**从未出现** `[LivoBridge][GPS] First GPS message`，说明 **LivoBridge::onGPS 从未被调用**，即 bag 未向当前节点发布该话题、或话题/类型/QoS 不匹配。

### 4.2 可能原因与对策

| 原因 | 对策 |
|------|------|
| Bag 中无该话题或 0 条消息 | `ros2 bag info <bag_dir>` 查看 topic 列表与 message count；确认存在 `sensor.gps.topic` 对应话题且类型为 `sensor_msgs/msg/NavSatFix` |
| Bag 仅有 ublox_msgs（如 /ublox/navsat） | rosbag2_player 会因缺少 `ublox_msgs` 包而 **Ignoring** 这些话题，不发布。需安装 ublox_msgs，或使用以 sensor_msgs/NavSatFix 录制的 bag，并配置 `sensor.gps.topic` 为该话题 |
| 配置 topic 与 bag 不一致 | M2DGR 常用 `/ublox/fix`；在 `system_config_M2DGR.yaml` 中 `sensor.gps.topic: "/ublox/fix"`，且 bag 中该话题类型为 NavSatFix |
| QoS 不匹配 | LivoBridge 使用 RELIABLE KeepLast(10)；若 bag 用 BEST_EFFORT 可能兼容，反之可尝试改订阅 QoS（一般 rosbag2 回放为 RELIABLE） |

### 4.3 推荐 grep 与验证步骤

```bash
# 是否收到过任意 GPS 消息（含无 fix）
grep -E 'LivoBridge\[GPS\]|GPS_DIAG|TRAJ_LOG no GPS' automap.log

# 45s 后若仍无数据会出现的诊断
grep 'LivoBridge\[GPS_DIAG\] Still 0' automap.log

# 确认配置与 bag 话题
grep -E 'sensor.gps.topic|gps_topic=|Subscribed to' automap.log
```

验证：在 bag 目录下执行 `ros2 bag info <bag_dir>`，确认存在 `sensor.gps.topic` 对应话题、类型为 `sensor_msgs/msg/NavSatFix`、且 message count > 0。

---

## 5. 强化日志规范与精准定位（避免每次无法定位）

### 5.1 统一标签约定

| 标签 | 用途 | 要求 |
|------|------|------|
| `[CRASH_CONTEXT]` | 崩溃时“最后一条”即故障点；每条带 `step=` 与关键上下文（frame/session_id/ts） | 关键路径入口/出口、易崩块前后必须打 |
| `[LIO][TRACE]` | fast_livo 每帧 LIO 步骤；`frame=N step=xxx` | 崩溃后看**最后一条** step 即知卡在哪一步 |
| `[TRACE]` | automap 后端/建图步骤；`step=xxx result=xxx frame_no= ts=` | 与 session_id 同现时便于一次 grep 定位 |
| `[DIAG]` | 诊断用（队列长度、首次收到、配置回显） | 问题复现时保留完整 DIAG 行 |
| `[SHUTDOWN]` | 析构与退出顺序 | 退出时崩溃必看 SHUTDOWN step=N |
| `[EXCEPTION]` | 异常类型与 what()、以及 step_where= | 必须带 step_where 或 frame_no/ts |

**原则**：任何可能崩溃或卡住的代码块，**块入口打一条 CRASH_CONTEXT/TRACE，块出口再打一条**；中间关键分支（如 laser_map_pub、voxel filter）再细分 step，保证“最后一条日志 = 崩溃前最后执行到的步骤”。

### 5.2 fast_livo 崩溃时精准定位

- **最后一条 LIO 步骤**：`grep -E 'fast_livo\\[LIO\\]\\[TRACE\\]|fast_livo\\[CRASH_CONTEXT\\]' logs/automap.log logs/full.log | tail -5`
- 最后一行中的 `frame=N step=xxx` 即崩溃发生前的执行点（若为 `step=after_laser_map_pub` 且下一行无 `exit_laser_map_block`，则崩溃在激光地图块析构等）。
- SIGSEGV 时 stderr 会打印 backtrace；同时建议：`ulimit -c unlimited` 后复现，用 `gdb -c core.<pid> $(which fastlivo_mapping)` 执行 `bt full`。

### 5.3 automap_system 崩溃时精准定位

- **后端 worker**：`grep -E 'CRASH_CONTEXT|TRACE.*step=backend|BACKEND.*DIAG.*frame_no|session_id' logs/automap.log | tail -20`
- **地图发布**：`grep -E '\[MAP\]|buildGlobalMap|voxelDownsampleChunked' logs/automap.log | tail -15`
- **关闭阶段**：`grep -E 'SHUTDOWN|PIPELINE.*event=' logs/automap.log | tail -15`

### 5.4 速查表：现象 → grep 命令

| 现象 | 建议 grep（在 logs/automap.log 或 full.log 中） |
|------|--------------------------------------------------|
| **fast_livo SIGSEGV** | `grep -E 'fast_livo\\[LIO\\]\\[TRACE\\]|fast_livo\\[CRASH_CONTEXT\\]' <log> \| tail -10` → 最后一行 step= 即崩溃前步骤 |
| **automap 节点崩溃** | `grep -E 'CRASH_CONTEXT|TRACE.*step=|session_id|SHUTDOWN' <log> \| tail -20` |
| **退出时 SIGSEGV** | `grep -E 'SHUTDOWN|PIPELINE|backend_worker_exited' <log> \| tail -15` |
| **卡住无输出** | `grep -E 'BACKEND.*popped|BACKEND.*processed|MAP.*step=' <log> \| tail -5` → 看最后处理的 frame_no / step |
| **无 GPS / 无轨迹** | `grep -E 'LivoBridge\\[GPS\\]|GPS_DIAG|TRAJ_LOG' <log>` |
| **无关键帧/无建图** | `grep -E 'kf_created|tryCreateKeyFrame|no_odom_in_cache' <log> \| tail -20` |
| **GPS 对齐后 iSAM2 崩溃** | `grep -E '\[GPS_BATCH\]\[DIAG\]|\[ISAM2_DIAG\]|\[ISAM2_QUEUE\]' <log> \| tail -40` → 见 §10 |

---

## 6. 常用 grep 示例

```bash
# 只看地图发布与体素相关
grep -E '\[MAP\]|publishGlobalMap|buildGlobalMap|voxelDownsampleChunked' 日志或终端输出

# 只看后端帧处理与地图触发
grep -E 'BACKEND.*FRAME|step=publishGlobalMap|DATA_FLOW' 日志或终端输出

# 关闭/退出顺序（精确定位 exit 时崩溃到哪一步）
grep -E '\[SHUTDOWN\]|\[PIPELINE\]|backend_worker_exited|Sensor idle' 日志或终端输出

# 精准追踪（step + tid + lwp，与 GDB info threads 对应）
grep -E '\[TRACE\]|\[CRASH_TRACE\]|\[FPFH_CRASH_TRACE\]' 日志或终端输出

# fast_livo 崩溃：最后 LIO 步骤（精准定位到 step）
grep -E 'fast_livo\[LIO\]\[TRACE\]|fast_livo\[CRASH_CONTEXT\]' logs/automap.log | tail -10

# 崩溃前最后 20 行（若已重定向到文件）
grep -E '\[MAP\]|publishGlobalMap|buildGlobalMap|voxelDownsample|SHUTDOWN|PIPELINE|CRASH_CONTEXT|LIO\]\[TRACE\]' log.txt | tail -20
```

---

## 7. GDB 与 Core Dump

崩溃精确定位请配合 GDB：使用 `--gdb` 启动或事后用 core 分析。详见 [DEBUG_WITH_GDB.md](DEBUG_WITH_GDB.md)。

---

## 8. 卡滞/阻塞与队列诊断（新增标签）

出现「计数不涨、HEARTBEAT 停在同一帧、无新日志」时，用下列标签精确定位卡在哪个环节。

### 8.1 标签与含义

| 标签 / 关键字 | 含义 | 排查方向 |
|---------------|------|----------|
| `[INGRESS] wait_start` | Executor 线程因 ingress 队列满开始等待 feeder | 若持续出现→feeder 或 backend 未消费，见 FEEDER/BACKEND |
| `[INGRESS] wait_timeout` | 单次 500ms 等待超时，consecutive_timeouts 递增 | 连续 3 次会丢弃最旧帧并 push 本帧；若反复出现→下游卡滞 |
| `[INGRESS] dropped oldest frame` | 已丢弃队首帧以解除 Executor 阻塞 | 正常保护；若频繁出现需查 feeder/backend 为何慢 |
| `[FEEDER] backpressure` | feeder 因 frame_queue 满在等待 backend | 若长期存在→backend 处理慢或卡在 runScheduledAlignment/waitForPendingTasks |
| `[FEEDER][HEARTBEAT]` | feeder 每 100 帧打一次，含 queue/ingress 大小 | 确认 feeder 是否存活、ingress 是否堆积 |
| `[BACKEND][HEARTBEAT]` | backend 每 30s 打一次（等数据时），含 livo_cloud/livo_odom/ingress | livo 不涨→回调未交付；ingress>0→feeder 可能卡住 |
| `[BACKEND] wait_done reason=timeout` | backend 每 2s 被唤醒但队列仍空 | 仅 DEBUG 级；若只有 HEARTBEAT 无 frame 处理→上游未入队 |
| `[GPS_ALIGN] runScheduledAlignment enter/exit` | 后台执行 try_align 的入口与耗时 | exit 后无 callbacks_done→可能卡在 align 回调（如 getHistoricalGPSBindings） |
| `[GPS_ALIGN] try_align callbacks_enter/callbacks_done` | 对齐成功后在锁外执行 onGPSAligned 等 | 若 enter 后无 done→回调内卡住（如 waitForPendingTasks） |
| `[GPS_BATCH] enter` | 开始批量添加 GPS 因子 | 随后会有 waitForPendingTasks；若长期无后续→卡在 Isam2 队列 |
| `[ISAM2_QUEUE] waitForPendingTasks enter/done/timeout` | 等待优化任务队列清空 | timeout 表示 5s 内未清空，backend 会继续；若频繁 timeout→opt 线程慢或任务堆积 |

### 8.2 卡滞时推荐 grep 顺序

```bash
# 1）最后几条 INGRESS/FEEDER/BACKEND，看卡在「等入队」还是「等消费」
grep -E '\[INGRESS\]|\[FEEDER\] backpressure|\[BACKEND\]\[HEARTBEAT\]' automap.log | tail -20

# 2）GPS 对齐与批量因子是否卡在回调或 Isam2
grep -E '\[GPS_ALIGN\]|\[GPS_BATCH\]|\[ISAM2_QUEUE\]' automap.log | tail -15

# 3）MAP_PUB / LOOP_OPT 是否在等（仅 DEBUG 级有 timeout 日志）
grep -E '\[MAP_PUB\]|\[LOOP_OPT\]' automap.log | tail -10
```

### 8.3 现象 → 可能原因速查

| 现象 | 可能原因 | 建议 grep |
|------|----------|-----------|
| livo_cloud/livo_odom 不涨 | ROS 回调未执行（Executor 阻塞或单线程卡在 onCloud 等） | `INGRESS wait_start`、`INGRESS wait_timeout` |
| HEARTBEAT 中 ingress>0 且持续增长 | feeder 未消费（feeder 卡在体素或 frame_queue 满） | `FEEDER backpressure`、`FEEDER voxel` |
| HEARTBEAT 中 queue=0 且 livo 涨 | backend 在等 frame_queue，feeder 未 push（或 push 极慢） | `FEEDER pushed`、`FEEDER HEARTBEAT` |
| 有 runScheduledAlignment enter 无 exit | try_align 内死锁或长时间持锁（已修复：回调在锁外） | `GPS_ALIGN` |
| 有 callbacks_enter 无 callbacks_done | onGPSAligned/addBatchGPSFactors 内卡住（如 waitForPendingTasks） | `GPS_BATCH`、`ISAM2_QUEUE` |

---

## 9. 修改阈值或日志量

- **放宽小点云绕过阈值**：在 `automap_pro/src/core/utils.cpp` 中修改 `kVoxelChunkedSizeThreshold`（当前 250000）。调大则更多场景走单次体素。
- **减少 chunk 步进日志**：将 `voxelDownsampleChunked` 内 `ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ...")` 改为 `ALOG_DEBUG`，可减少刷屏，需要时再开 DEBUG 级别。
- **卡滞诊断日志**：`[INGRESS] wait_start`、`[FEEDER] backpressure`、`[GPS_ALIGN] runScheduledAlignment`、`[ISAM2_QUEUE] waitForPendingTasks` 等为 INFO；backend/map_pub/loop_opt 的 wait_done timeout 为 DEBUG，需要时可将对应 RCLCPP_DEBUG 改为 RCLCPP_INFO。

---

## 10. iSAM2 / GPS 批量与崩溃根因分析

当崩溃发生在 **GPS 对齐成功之后**、或与 **iSAM2 优化线程** 相关时，用下列标签精确定位根因（参见 [FIX_GPS_BATCH_SIGSEGV_20260310.md](FIX_GPS_BATCH_SIGSEGV_20260310.md)）。

### 10.1 诊断标签与含义

| 标签 / 关键字 | 含义 | 用于根因分析 |
|---------------|------|----------------|
| `[GPS_BATCH][DIAG] phase=existing_async enqueue_batch` | 异步：已入队「已有 GPS 子图」批量任务，count= 本批因子数 | 随后应为 wait_enter → opt 线程处理 → wait_done |
| `[GPS_BATCH][DIAG] phase=existing_async wait_enter queue_depth=` | 回调线程开始等待优化队列，当前队列深度 | 若崩溃在 wait_enter 与 wait_done 之间，崩溃在 **opt 线程** |
| `[GPS_BATCH][DIAG] phase=existing_async wait_done` | 回调线程等到队列清空 | 若先出现 wait_done 再崩溃，可能是后续逻辑或另一线程 |
| `[GPS_BATCH][DIAG] phase=historical_async ...` | 同上，针对「历史绑定」批量 | 同样用 wait_enter / wait_done 界定 opt 线程是否在执行 |
| `[ISAM2_DIAG] optLoop pop type=LOOP_FACTOR|GPS_FACTOR|BATCH_UPDATE` | opt 线程刚取出任务类型与剩余队列深度 | **崩溃在 commitAndUpdate 时，最后一条 pop type= 即当时执行的任务类型** |
| `[ISAM2_DIAG] addGPSFactorsBatch enter count=` | opt 线程开始执行批量 GPS（BATCH_UPDATE 任务） | 随后会有 commitAndUpdate enter → 若崩溃则多在 isam2_.update 内部 |
| `[ISAM2_DIAG] addGPSFactorsBatch done added=` | 批量 GPS 添加并完成一次 commitAndUpdate | 有 enter 无 done → 崩溃在 commitAndUpdate / isam2_.update |
| `[ISAM2_DIAG] commitAndUpdate enter pending_factors= pending_values=` | 即将调用 isam2_.update(graph, values) | **若崩溃在此后且无 commitAndUpdate done → 崩溃在 GTSAM isam2_.update() 内部（如 addVariables/TBB internal_clear）** |
| `[ISAM2_DIAG] commitAndUpdate done elapsed_ms= success=1` | 单次 update 成功返回 | 有 enter 无 done → 崩溃在 update 或 calculateEstimate |
| `[ISAM2_DIAG] commitAndUpdate done success=0 exception=` | 单次 update 抛异常 | 根因为 GTSAM 异常，看 exception 内容 |
| `[ISAM2_QUEUE] waitForPendingTasks enter/done/timeout` | 等待队列清空 | timeout 表示 5s 内未清空，可能 opt 线程卡在 commitAndUpdate 或任务过多 |

### 10.2 GPS 对齐后崩溃：推荐 grep 与顺序

```bash
# 1）时间线：GPS 对齐 → 批量入队 → 等待 → opt 执行（用于确认崩溃发生在哪一阶段）
grep -E '\[GPS_ALIGN\]|\[GPS_BATCH\]|\[GPS_BATCH\]\[DIAG\]|\[ISAM2_DIAG\]|\[ISAM2_QUEUE\]' full.log | tail -60

# 2）仅 iSAM2 与队列（精确定位 commitAndUpdate 前后）
grep -E '\[ISAM2_DIAG\]|\[ISAM2_QUEUE\]' full.log | tail -30

# 3）最后一条 ISAM2_DIAG：若为 commitAndUpdate enter 且无后续 done → 崩溃在 isam2_.update 内部
grep '\[ISAM2_DIAG\]' full.log | tail -5
```

### 10.3 如何解读「最后一条日志」

| 最后一条（或最后几条） | 推断 |
|------------------------|------|
| `commitAndUpdate enter` 且无 `commitAndUpdate done` | 崩溃在 **isam2_.update(pending_graph_, pending_values_)** 内部（如 GTSAM addVariables、TBB internal_clear/free）。配合 GDB 栈：`free` ← `tbb::...::internal_clear` ← `gtsam::ISAM2::addVariables`。 |
| `optLoop pop type=BATCH_UPDATE` 后紧跟 `commitAndUpdate enter`，无 done | 同上，且可确认是 **批量 GPS 任务** 触发的单次 update。 |
| `addGPSFactorsBatch enter count=N` 后无 `addGPSFactorsBatch done` | 崩溃在 addGPSFactorsBatch 内，即 opt 线程在本次 batch 的 commitAndUpdate 中崩溃。 |
| `wait_enter` 有，`wait_done` 无，且中间有 `commitAndUpdate enter` 无 done | 回调线程在等待；opt 线程在 commitAndUpdate 内崩溃（典型 GPS 批量 SIGSEGV 场景）。 |
| `wait_done` 已有，随后崩溃 | 崩溃不在本次 GPS 批量路径，可能是后续 HBA、地图发布或其它线程。 |

### 10.4 速查：现象 → grep

| 现象 | 建议 grep |
|------|------------|
| **GPS 对齐后立即 SIGSEGV** | `grep -E '\[GPS_BATCH\]\[DIAG\]|\[ISAM2_DIAG\]|\[ISAM2_QUEUE\]' full.log \| tail -40` → 看最后是 commitAndUpdate enter 还是 wait_enter/wait_done |
| **opt 线程崩溃（GDB 显示 optLoop/commitAndUpdate）** | `grep '\[ISAM2_DIAG\]' full.log \| tail -10` → 确认 pop type= 与 commitAndUpdate enter/done |
| **队列长期不空 / 卡滞** | `grep -E '\[ISAM2_QUEUE\]|\[ISAM2_DIAG\] optLoop pop' full.log \| tail -20` |
| **多路 GTSAM 崩溃（iSAM2 / Optimizer / HBA 任一）** | `grep -E 'GTSAM_ENTRY|GTSAM_EXIT' full.log \| tail -30` → 最后一条 ENTRY 无 EXIT 即出事调用点，详见 [GTSAM_MULTI_USE_AND_LOGGING.md](GTSAM_MULTI_USE_AND_LOGGING.md) |
