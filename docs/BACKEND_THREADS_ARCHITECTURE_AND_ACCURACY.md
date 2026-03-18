# 后端多线程架构逐行分析与建图精度影响

本文档对后端相关线程做**逐行级**分析：异步程度、线程间交互复杂度、是否符合合格软件设计原则，以及对**最终建图精度**的影响（目标：影响最小或带来性能/精度提升）。

---

## 一、线程清单与数据流总览

### 1.1 后端相关线程（按数据流顺序）

| 线程名 | 入口函数 | 职责 | 数据来源 | 数据去向 |
|--------|----------|------|----------|----------|
| **ROS Executor** | 回调 | onOdometry / onCloud / onGPS / onKFInfo | 传感器/前端 | ingress_queue_, odom_cache_, kfinfo_cache_, gps_queue_ |
| **feeder** | feederLoop | ingress → frame_queue_，可选体素预计算 | ingress_queue_ | frame_queue_ |
| **backend_worker** | backendWorkerLoop | 取帧、建 KF、子图、子图内回环投递、GPS 因子、viz 投递 | frame_queue_, odom/kfinfo cache | keyframe_mutex_ 内 createKeyFrame+addKeyFrame；intra_loop_task_queue_ 或 sync intra_loop；isam2 直接调用；viz_cloud_queue_ |
| **intra_loop_worker** | intraLoopWorkerLoop | 子图内回环检测，结果投 opt | intra_loop_task_queue_ | opt_task_queue_ (INTRA_LOOP_BATCH) |
| **loop_trigger** | loopTriggerThreadLoop | 向 loop_detector 喂 KF（子图间回环） | loop_trigger_queue_ | loop_detector_.addKeyFrame |
| **opt_worker** | optWorkerLoop | 消费 opt_task_queue_，执行 ISAM2/子图/回环入图 | opt_task_queue_ | isam2_optimizer_, submap_manager_, kf_manager_, loop_trigger_queue_ |
| **IncrementalOptimizer::optLoop** | optLoop | 内部异步任务队列（若启用） | opt_queue_ (OptimTask) | commitAndUpdate |
| **gps_worker** | gpsWorkerLoop | GPS 测量入窗 + runScheduledAlignment | gps_queue_ | gps_manager_.addGPSMeasurement, runScheduledAlignment |
| **gps_align** | gpsAlignWorkerLoop | 完整 GPS 对齐流程（变换、重建） | gps_align_queue_ | transformAllPosesAfterGPSAlign, opt_task_queue_ (GPS_ALIGN_COMPLETE) |
| **map_publish** | mapPublishLoop | 按需 buildGlobalMap + 发布 | map_publish_cv_ | global_map_pub_ |
| **status_publisher** | statusPublisherLoop | 按需发布状态 | status_pub_cv_ | status_pub_ |
| **loop_opt** | loopOptThreadLoop | （若存在）消费 loop_factor_queue_ 执行 addLoopFactor | loop_opt_cv_ | isam2_optimizer_.addLoopFactor |
| **viz** | vizThreadLoop | 消费 viz_cloud_queue_ 发布点云 | viz_cv_ | 可视化 |
| **SubMapManager::freeze_post** | freezePostProcessLoop | 子图冻结后处理 | freeze_post_queue_ | 子图归档等 |
| **HBAOptimizer::worker** | workerLoop | 触发 HBA、写回位姿 | HBA 任务队列 | submap_manager_ 位姿更新 |
| **LoopDetector** | descWorkerLoop / matchWorkerLoop | 描述子计算、TEASER 匹配 | KeyFrame 队列、匹配队列 | 回环约束回调 → opt 或 onLoopDetected |

说明：实际链接的二进制可能是 **automap_system_component**（仅 automap_system.cpp），也可能是包含 **worker_threads.cpp / system_init.cpp / keyframe_submap.cpp** 等模块的构建；若为前者，则无 opt_worker、intra_loop_worker、gps_worker、gps_align、loop_trigger 等线程，后端在 automap_system.cpp 内完成 createKeyFrame + addKeyFrame + 本线程内 intra_loop（或 enqueue）+ 本线程 ISAM2 + viz 投递。

---

## 二、逐线程异步性与交互分析

### 2.1 ROS Executor（回调线程）

- **异步性**：与后端完全解耦。回调只做：写 ingress、写 odom/kfinfo cache、写 gps_queue_、更新 last_*，**不**调用任何重计算或 ISAM2。
- **交互**：仅通过**有界队列 + 非阻塞写**（队列满时策略见 feeder 背压），或**短临界区**（odom_cache_mutex_ 等）写入缓存。
- **设计**：符合「回调快速返回」原则；不持长锁、不阻塞。

### 2.2 feederLoop（automap_system.cpp 782–884）

- **逻辑**：  
  - `ingress_not_empty_cv_.wait_for` 取一帧 → 可选 `voxelDownsampleWithTimeout`（带 5s 超时）→ `frame_queue_not_full_cv_.wait_for` 背压（队列满时等待）→ push frame_queue_，notify frame_queue_cv_。
- **异步性**：与 backend 仅通过 **frame_queue_** 耦合；feeder 不读任何后端状态。
- **阻塞点**：  
  - 体素降采样可能最多 5s（有超时）；  
  - **frame_queue_ 满时**会 wait_for(backpressureWaitSec)，若 backend 极慢会拖住 feeder，但不会拖住 Executor。
- **交互复杂度**：低。两把锁：ingress_mutex_、frame_queue_mutex_，顺序固定（先 ingress 后 frame_queue），无环。
- **对精度影响**：背压或超时丢帧会减少参与建图的帧数，理论上略降轨迹密度；通常可接受。体素超时用 sanitized copy 可能略影响该帧质量，属降级保护。

### 2.3 backendWorkerLoop（核心后端线程）

- **逻辑概要**：  
  - 每轮：`runScheduledAlignment()` → `frame_queue_cv_.wait_for` 取一帧 → odom/kfinfo 缓存对齐 → (可选) process_every_n 跳帧 → **tryCreateKeyFrame**（见下）→ 按帧数触发 status/map 发布、notify map_publish_cv_/status_pub_cv_。
- **tryCreateKeyFrame 内**（automap_system.cpp 1402–1682）：  
  - shouldCreateKeyFrame → voxel（若无可选 cloud_ds）→ GPS 查询 → **keyframe_mutex_** 内 **createKeyFrame + addKeyFrame**（最小临界区）→ **释放 keyframe_mutex_** → 子图内回环（异步时仅 enqueue intra_loop_task_queue_，同步时 async+wait_for 或直接 detectIntraSubmapLoop）→ addKeyFrameNode / addGPSFactorForKeyFrame / forceUpdate（本线程调 ISAM2）→ viz_cloud_queue_.push。
- **异步性**：  
  - **与 feeder**：仅通过 frame_queue_ 消费，不反向依赖。  
  - **与子图内回环**：当 `intra_submap_async=true` 时，只 enqueue (submap, query_idx)，**不等待** detectIntraSubmapLoop，异步足够。  
  - **与 ISAM2**：本线程内直接调用 addKeyFrameNode、addGPSFactor、forceUpdate，与 IncrementalOptimizer 的 rw_mutex_ / 内部 optLoop（若启用）存在互斥，但**不**在 keyframe_mutex_ 下调用，临界区已缩小，符合设计。
- **阻塞点**：  
  - runScheduledAlignment() 内 try_align 可能持 gps_manager_ 锁并执行 SVD + 回调；若回调里调用了会等待 backend 的接口（如 waitForPendingTasks），存在**理论上的死锁/长等**；文档约定对齐回调不在持锁时等 backend，需保持。  
  - GPS 查询（queryByNearestPosition / queryByTimestampEnhanced）持 gps_manager_ 锁时间应很短。  
  - 本线程内 forceUpdate → commitAndUpdate 可能耗时较大，但**不持 keyframe_mutex_**，故不会阻塞「创建 KF + 加入子图」的并发入口（若有）。
- **交互复杂度**：中。涉及 keyframe_mutex_、frame_queue_mutex_、intra_loop_task_mutex_、viz_mutex_、odom/kfinfo cache、gps_manager_ 等；锁顺序需统一（见下节）。
- **对精度影响**：  
  - **KF 与子图顺序**：严格按帧时间戳顺序 createKeyFrame + addKeyFrame，顺序正确。  
  - **子图内回环**：异步时本帧只投任务，回环因子由 opt_worker 在后续执行，存在**帧间延迟**（通常一至数帧），对平滑与一致性影响极小；同步/超时模式则与本帧同序。  
  - **GPS 因子**：本帧即加，与 KF 同序，无额外延迟。

### 2.4 intra_loop_worker（intraLoopWorkerLoop）

- **逻辑**：wait_for(intra_loop_task_cv_) → 取 (submap, query_idx) → **detectIntraSubmapLoop** → 若 loops 非空，push **INTRA_LOOP_BATCH** 到 opt_task_queue_，notify opt_task_cv_。
- **异步性**：与 backend 完全解耦；backend 只投任务，不等待结果。
- **阻塞点**：detectIntraSubmapLoop 可能很慢（FPFH+TEASER），但只阻塞本线程，不阻塞 backend。
- **交互**：仅 intra_loop_task_mutex_ 与 opt_task_mutex_，无嵌套，顺序明确。
- **对精度影响**：  
  - 子图内回环**延迟执行**，可能比「同帧立即执行」晚几帧入图；对轨迹平滑和闭环一致性影响很小。  
  - 若队列满丢弃任务，会少若干子图内回环约束，可能略降局部一致性；可通过增大 kMaxIntraLoopTaskQueueSize 或保证 opt_worker 及时消费来缓解。

### 2.5 loop_trigger（loopTriggerThreadLoop）

- **逻辑**：wait_for(loop_trigger_cv_) → 取 KeyFrame → loop_detector_.addKeyFrame(kf)。
- **异步性**：与 backend 通过 loop_trigger_queue_ 解耦；backend（或 opt_worker 在 KEYFRAME_CREATE 分支）只 push KF，不等待 addKeyFrame 完成。
- **交互**：仅 loop_trigger_mutex_，简单。
- **对精度影响**：子图间回环检测与 KF 流异步，延迟可接受；不改变因子图语义，仅延迟发现回环的时间。

### 2.6 opt_worker（optWorkerLoop）

- **逻辑**：wait_for(opt_task_cv_) → 取 OptTaskItem → 按 type 执行：LOOP_FACTOR、GPS_FACTOR、SUBMAP_NODE、ODOM_FACTOR、REBUILD、GPS_ALIGN_COMPLETE、RESET、FORCE_UPDATE、**INTRA_LOOP_BATCH**、KEYFRAME_CREATE 等。
- **异步性**：所有 GTSAM/ISAM2 写操作集中在本线程，与 backend 通过 **opt_task_queue_** 串行化；backend 若也直接调 isam2_optimizer_（如 automap_system.cpp 路径），则存在**双写路径**（backend 与 opt_worker 都可能调 addKeyFrameNode、forceUpdate），需通过构建配置保证**只选一条路径**（要么全 backend 内同步 ISAM2，要么全经 opt_worker），否则需严格约定避免并发写 ISAM2。
- **阻塞点**：每类任务可能耗时不同（INTRA_LOOP_BATCH 多次 addSubMapNode + addLoopFactorDeferred + forceUpdate；KEYFRAME_CREATE 含 submap_manager_.addKeyFrame、addOdomFactor 等）；若单任务过久会延迟后续任务，但不阻塞 backend。
- **交互复杂度**：高。持 opt_task_mutex_ 取任务；执行时持 submap_update_mutex_（KEYFRAME_CREATE）、isam2 的 rw_mutex_（各类 add*）、以及可能 notify loop_trigger_cv_。锁顺序应为：opt_task_mutex_（取任务后释放）→ submap_update_mutex_ → 不持 keyframe_mutex_；与 IncrementalOptimizer 约定一致。
- **对精度影响**：  
  - **任务顺序**：FIFO 保证与投递顺序一致；若 backend 与 intra_loop_worker 同时投递，INTRA_LOOP_BATCH 与 KEYFRAME_CREATE 的相对顺序取决于入队顺序，一般 INTRA_LOOP 稍晚，对图一致性影响可接受。  
  - **KEYFRAME_CREATE**：在 opt_worker 内 addKeyFrame + addOdomFactor，与「backend 内 createKeyFrame+addKeyFrame+本线程 ISAM2」是两套实现；若混用需避免同一帧既在 backend 加 KF 又在 opt 加 KF。当前 CMake 仅链接 automap_system.cpp，则 KF 在 backend 内创建，opt_worker 的 KEYFRAME_CREATE 可能来自 keyframe_submap 的 tryCreateKeyFrame（若该模块被链接），需在构建与调用链上区分清楚。

### 2.7 gps_worker / gps_align

- **gps_worker**：从 gps_queue_ 取测量 → addGPSMeasurement；队列空时 **runScheduledAlignment()**。  
  - 与 backend 的交叉：backend 每轮也调 runScheduledAlignment()；两处都可能执行 try_align 与对齐回调，需保证**回调内不等待 backend**，且 gps_manager_ 内对齐状态更新线程安全。
- **gps_align**：从 gps_align_queue_ 取任务 → transformAllPosesAfterGPSAlign、waitForPendingTasks、getAllSubmapData 等 → 投 GPS_ALIGN_COMPLETE 到 opt_task_queue_。  
  - waitForPendingTasks 会等 opt_worker/backend 的 ISAM2 队列空且无 commitAndUpdate，若此时 backend 正持 keyframe_mutex_ 且某路径在等 gps_align，可能死锁；当前设计是「对齐完成后」才 wait 和投递，且不在持 keyframe_mutex_ 时调 gps 对齐，故风险可控。
- **对精度影响**：GPS 对齐与因子添加顺序由 runScheduledAlignment 与 addBatchGPSFactors / opt 任务顺序保证；延迟对齐仅带来因子晚几帧加入，对最终精度影响很小。

### 2.8 map_publish / status_publisher / viz / loop_opt

- **map_publish**：wait_for(map_publish_cv_) → buildGlobalMap + publish；**持 map_publish_mutex_ 仅用于 wait**，拿到 pending 后 **unlock 再** buildGlobalMap，不持 keyframe_mutex_，设计正确。
- **status_publisher**：按需触发，仅读原子/缓存状态，无重锁。
- **viz**：消费 viz_cloud_queue_，有界队列满则丢最旧；与 backend 单向解耦。
- **loop_opt**（若存在）：消费 loop_factor_queue_，调用 addLoopFactor；与子图间回环检测异步，不阻塞 backend。
- **对精度影响**：无。仅影响可视化与状态发布延迟，不改变优化结果。

### 2.9 IncrementalOptimizer 内部 optLoop（若启用）

- 内部 **opt_queue_** 消费 OptimTask（LOOP_FACTOR、GPS_FACTOR、BATCH_UPDATE 等），在 **opt_queue_mutex_** 下取任务，在 **rw_mutex_** 下执行 commitAndUpdate。
- 与 AutoMapSystem 的 **opt_worker** 关系：若两者并存，则存在「AutoMapSystem::opt_worker 调 isam2_optimizer_.add* / forceUpdate」与「IncrementalOptimizer::optLoop 调 commitAndUpdate」的并发；文档约定 GTSAM 调用串行化（如通过同一 opt_worker 或全局 GtsamCallScope），需在实现上保证只一条路径写 ISAM2，避免竞态。
- **对精度影响**：若串行化正确，无影响；若并发写，可能导致未定义行为或崩溃，必须避免。

---

## 三、锁顺序与设计原则

### 3.1 锁顺序约定（防死锁）

建议全局顺序（从外到内）：  
`keyframe_mutex_` → `submap_update_mutex_` → `opt_task_mutex_`（仅短时取任务）→ `IncrementalOptimizer::rw_mutex_`。  
**禁止**：持 rw_mutex_ 或 opt_task_mutex_ 时再去拿 keyframe_mutex_；持 keyframe_mutex_ 时不调用会阻塞或长时间持其他锁的路径（已通过缩小临界区与子图内回环异步满足）。

### 3.2 已满足的设计原则

- **最小临界区**：keyframe_mutex_ 仅包 createKeyFrame + addKeyFrame。  
- **回调不阻塞**：ROS 回调只写队列/缓存，快速返回。  
- **重计算不阻塞主路径**：子图内回环通过 intra_loop_worker 异步；子图间回环通过 loop_trigger + loop_detector 异步。  
- **单写 ISAM2**：同一时刻应只有 backend 或 opt_worker 之一在写 ISAM2（取决于构建与配置），需在代码与构建上明确。

### 3.3 可改进点

- **runScheduledAlignment 与 GPS 回调**：确保对齐回调（onGPSAligned、addBatchGPSFactors、waitForPendingTasks）**从不**在持 keyframe_mutex_ 的线程中调用，且回调内不等待 backend 持锁的路径。  
- **双路径 KF 写入**：若存在「backend 内 addKeyFrame」与「opt_worker KEYFRAME_CREATE 内 addKeyFrame」两路，需二选一或严格按 session/帧 ID 分工，避免重复或乱序。  
- **INTRA_LOOP_BATCH 与 KEYFRAME_CREATE 顺序**：opt_worker FIFO 自然保证先入队先执行；若希望「某 KF 的 INTRA_LOOP 一定在其 KEYFRAME_CREATE 之后」，当前异步下需依赖入队顺序（backend 先完成 addKeyFrame 再 enqueue intra_loop 任务，intra_loop_worker 再 enqueue INTRA_LOOP_BATCH），通常已满足。

---

## 四、对建图精度的影响（最小化与性能/精度提升）

### 4.1 顺序与一致性

- **关键帧与子图顺序**：由 backend 单线程按帧时间戳顺序 createKeyFrame + addKeyFrame，**顺序严格**，对精度无负面影响。  
- **子图内回环**：异步时晚 1～数帧入图，对轨迹平滑和闭环一致性影响很小；可能略增「回环闭合前」的短期漂移，闭合后与同步执行等价。  
- **GPS 因子**：与本帧 KF 同线程顺序添加（backend 路径）或经 opt 任务顺序（opt 路径），无乱序。

### 4.2 丢帧与降级

- **背压丢帧**：frame_queue_ 满且等待超时后强制 pop 一帧会减少建图帧数，轨迹密度略降；可通过增大 frame_queue 或提高 backend 吞吐（含子图内回环异步）减少发生。  
- **体素超时**：feeder 内 voxelDownsampleWithTimeout 超时后用 sanitized copy，该帧质量可能略差；属安全降级，对整体精度影响有限。  
- **子图内回环队列满**：丢弃 intra_loop 任务会少若干局部回环约束，可略微降低局部一致性；可通过队列大小与 opt_worker 处理速度缓解。

### 4.3 性能与精度提升点

- **子图内回环异步**：backend 不再被 FPFH/TEASER 拖慢，**帧率更稳**，在相同时间内可处理更多帧，**轨迹更密、约束更多**，有机会**提升精度**或至少不降。  
- **缩小 keyframe_mutex_**：减少锁竞争与阻塞，backend 吞吐提高，同上。  
- **单线程 ISAM2 写**：保证 GTSAM 状态一致，避免并发写导致的未定义行为或崩溃，**保护精度与稳定性**。

### 4.4 建议（保持精度并兼顾性能）

1. **明确唯一 KF 写入路径**：要么 backend 内 createKeyFrame+addKeyFrame（当前 automap_system.cpp），要么全走 opt_worker KEYFRAME_CREATE；避免两路同时写 submap_manager_/kf_manager_。  
2. **保证 GPS 对齐回调不持 keyframe_mutex_ 且不等待 backend**：避免死锁与长阻塞。  
3. **子图内回环**：默认 `intra_submap_async=true`，在保证不阻塞主线程的前提下，对精度影响最小，并可提升整体帧率与稳定性。  
4. **监控**：对 backpressure 丢帧、intra_loop 队列满、单帧耗时 STUCK_DIAG 做日志与告警，便于在保证精度的前提下调参（队列大小、process_every_n 等）。

**已落实**：2、3、4 已通过代码注释/契约、配置默认与新增告警落实；1 由当前构建（仅 automap_system.cpp）保证单一 KF 路径，见第七节。

---

## 五、结论表

| 维度 | 结论 |
|------|------|
| **异步程度** | 回调、feeder、backend、intra_loop_worker、loop_trigger、opt_worker、map_publish、viz、status 均通过队列或短临界区解耦；子图内回环在 async=true 时与主线程完全异步。 |
| **交互复杂度** | 中等：锁与队列较多，但顺序约定清晰；需避免双写 ISAM2 与 GPS 回调等 backend 互等。 |
| **设计原则** | 最小临界区、回调快速返回、重计算异步、单写 GTSAM 已落实；锁顺序与双路径 KF 需在实现与构建上统一。 |
| **对精度影响** | 顺序正确、无故意乱序；异步带来的 1～数帧延迟对精度影响很小；通过提升 backend 吞吐与稳定性，有机会净收益为正或持平。 |
| **建议** | 单一 KF 写入路径、GPS 回调不等待 backend、保持 intra_submap_async 默认 true、监控背压与队列满。 |

以上为后端多线程架构的逐行级分析与对建图精度的评估；按建议落实后，可在保证架构合格与稳定性的前提下，将精度影响最小化并争取性能与稳定性提升。

---

## 六、关键代码位置索引（便于逐行对照）

| 功能 | 文件:行 | 说明 |
|------|---------|------|
| feeder 主循环、背压 | automap_system.cpp ~782–884 | ingress_not_empty_cv_, frame_queue_not_full_cv_, voxelDownsampleWithTimeout |
| backend 主循环入口 | automap_system.cpp ~888 | runScheduledAlignment, frame_queue_cv_.wait_for |
| tryCreateKeyFrame 入口 | automap_system.cpp ~1402 | needNewKeyFrame, createKeyFrame, addKeyFrame |
| keyframe_mutex_ 临界区 | automap_system.cpp ~1512–1529 | 仅包 createKeyFrame + submap_manager_.addKeyFrame |
| 子图内回环异步 enqueue | automap_system.cpp ~1544–1558 | intra_loop_task_queue_.push_back, intra_loop_task_cv_.notify_one |
| 子图内回环同步/超时 | automap_system.cpp ~1561–1610 | detectIntraSubmapLoop 或 std::async + wait_for |
| addKeyFrameNode / forceUpdate | automap_system.cpp ~1660–1682 | 本线程 ISAM2 调用 |
| map_publish 持锁与释放后 build | automap_system.cpp ~1240–1246 | lock.unlock() 后再 publishGlobalMap |
| opt_worker 主循环 | worker_threads.cpp ~606–735 | opt_task_cv_.wait_for, INTRA_LOOP_BATCH, KEYFRAME_CREATE, submap_update_mutex_ |
| intra_loop_worker | worker_threads.cpp ~470–510 | intra_loop_task_queue_ → detectIntraSubmapLoop → opt_task_queue_ (INTRA_LOOP_BATCH) |
| KEYFRAME_CREATE 降级 addKeyFrame | keyframe_submap.cpp ~80–97 | opt_task_queue_ 满时 submap_update_mutex_ + addKeyFrame |
| IncrementalOptimizer optLoop | incremental_optimizer.cpp ~2451–2570 | opt_queue_mutex_, batch_tasks, rw_mutex_ 下 commitAndUpdate |
| 锁顺序约定注释 | incremental_optimizer.h ~256–257 | rw_mutex_ 与 keyframe_mutex_ 不嵌套 |
| 队列与 mutex 声明 | automap_system.h ~115–215 | ingress_queue_, frame_queue_, opt_task_queue_, intra_loop_task_queue_, 各 mutex/cv |

---

## 七、已落实的优化（基于结论）

在保证架构合格与稳定性的前提下，将精度影响最小化并争取性能与稳定性提升，已做如下实现：

| 建议 | 落实方式 |
|------|----------|
| **GPS 回调不持 keyframe_mutex_、不等待 backend** | 在 `onGPSAligned`、`addBatchGPSFactors` 顶部增加**契约注释**：仅由 runScheduledAlignment 在锁外回调触发；在 `runScheduledAlignment()` 调用处（backend 循环开头）注明仅在此处调用、不持 keyframe_mutex_，避免与 waitForPendingTasks 死锁。 |
| **子图内回环默认异步** | 配置默认 `intra_submap_async: true`（config_manager 与 yaml 已为 true），无需改代码。 |
| **监控：背压、intra_loop 队列满、单帧/连续慢帧** | ① 背压：已有 `[FEEDER] backpressure`、`[STUCK_DIAG] feeder blocked` 及强制丢帧 ERROR。② **intra_loop_task_queue 满**：当入队因队列满被跳过时，新增 `[STUCK_DIAG] intra_loop_task_queue full` 的 WARN_THROTTLE(5s)。③ **连续慢帧**：在 backend 中新增 `consecutive_slow_frames` 计数，单帧 >2s 时累加、否则清零；当连续 ≥3 帧慢时输出一次 `[STUCK_DIAG] consecutive N slow frames` 并清零，便于发现持续偏慢。 |
| **单一 KF 写入路径** | 当前构建仅链接 automap_system.cpp，KF 仅在 backend 内 createKeyFrame+addKeyFrame，无 opt_worker KEYFRAME_CREATE 双写；若将来链接 keyframe_submap 等模块，需在构建或配置上二选一。 |

---

## 八、进一步优化空间（含研究与开源参考）

在多线程与架构上仍有可挖掘的优化方向（无锁队列、iSAM2 选择性更新、任务优先级/批合并、GPS 线程隔离、可观测性增强等），已单独整理为 **[BACKEND_FURTHER_OPTIMIZATION_OPPORTUNITIES.md](BACKEND_FURTHER_OPTIMIZATION_OPPORTUNITIES.md)**，其中引用了 ROS2 realtime_tools、Boost.Lockfree、GTSAM ISAM2Params、FAST-LIO-SAM、信息引导 SLAM 与异步分布式 PGO 等研究与开源实现，便于后续选型与迭代。
