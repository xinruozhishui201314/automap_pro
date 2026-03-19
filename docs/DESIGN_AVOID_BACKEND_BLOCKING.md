# 从软件设计上避免后端阻塞

本文档基于 full.log 卡在 tryCreateKeyFrame(51) 的分析，从**锁粒度、临界区范围、耗时操作下放**等设计层面给出避免阻塞的方案，并给出可落地的代码级建议。

**根因与日志+代码对照**：见 [ROOT_CAUSE_BACKEND_STUCK_AND_BLOCKING.md](ROOT_CAUSE_BACKEND_STUCK_AND_BLOCKING.md)（为何卡住、锁顺序与 GPS/loop 互等、后端循环持锁修复）。

---

## 一、问题根因归纳

### 1.1 当前设计：一整段大临界区

后端 worker 在 **持 keyframe_mutex_ 的整段时间内** 执行整条 tryCreateKeyFrame 流水线：

```
worker_threads.cpp:
  keyframe_mutex_.lock()
  tryCreateKeyFrame(ts, pose, cov, cloud, ...)   // 全在锁内
  keyframe_mutex_.unlock()
```

tryCreateKeyFrame 内部顺序包括：

| 步骤 | 耗时/风险 | 是否必须与“关键帧状态”原子？ |
|------|-----------|------------------------------|
| shouldCreateKeyFrame | 低 | 是（读 kf_manager 状态） |
| voxel downsample | 中 | 否 |
| GPS query | 可能阻塞（GPSManager 锁） | 否 |
| createKeyFrame | 低 | 是（写 kf_manager） |
| submap_manager_.addKeyFrame(kf) | 低 | 是（写 submap 状态） |
| **detectIntraSubmapLoop** | **高（日志 ~3s）** | **否**（只读 active_sm） |
| isam2_optimizer_.addKeyFrameNode / addGPSFactorForKeyFrame | 可能阻塞（rw_mutex_） | 否（可延后） |
| viz 队列 push | 低 | 否 |

因此存在：

- **长时间持 keyframe_mutex_**：子图内回环（FPFH + TEASER）和 ISAM2 调用都在锁内，易导致单帧处理时间过长甚至卡死。
- **持锁做重计算**：detectIntraSubmapLoop 内部会拿 loop_detector 的 `intra_prepare_mutex_` 等，若与 loop_trigger 等线程存在锁序或等待关系，存在**死锁**或长时间阻塞风险。
- **持锁调第三方/ISAM2**：GTSAM/ISAM2 内部可能阻塞；在持 keyframe_mutex_ 时调用会放大影响并增加与其它线程死锁的可能。

### 1.2 为何会“卡住”

- **不是崩溃**：日志无 SIGSEGV，最后一条是 tryCreateKeyFrame_enter 51，无 exit。
- **可能情况**：  
  1）**死锁**：backend 持 keyframe_mutex_ → detectIntraSubmapLoop → 某锁 A；另一线程持锁 B → 等 keyframe_mutex_ 或锁 A。  
  2）**极慢或活锁**：FPFH/TEASER 在某种输入下极慢或循环，表现为“卡住”。  
  3）**ISAM2 内部阻塞**：addKeyFrameNode/forceUpdate 在 GTSAM 内长时间不返回。

设计上的共同点是：**把“必须与关键帧状态原子”的更新**和**重计算、I/O/第三方库**都放在同一把 keyframe_mutex_ 下，放大了上述任一情况的影响。

---

## 二、设计原则（避免阻塞）

1. **最小临界区**  
   只把“读/写关键帧与子图状态”的**最小必要序列**放在 keyframe_mutex_ 下，其余一律在锁外。

2. **持锁时不调用可能阻塞的代码**  
   在 keyframe_mutex_ 内不调用：  
   - 可能长时间计算的逻辑（FPFH、TEASER、大点云处理）；  
   - 可能等待其它线程或其它锁的接口（如 wait、等待 opt_worker）；  
   - 可能阻塞的第三方库（GTSAM 的 update、solver 等）。

3. **重活异步化或延后**  
   子图内回环、ISAM2 节点/因子添加等“重活”尽量：  
   - 放到独立任务队列（如 opt_worker），或  
   - 在**先释放 keyframe_mutex_** 后再在同一线程执行，避免与其它线程形成环等。

4. **锁顺序统一**  
   全局约定并遵守锁顺序（例如 keyframe_mutex_ → submap_update_mutex_ → opt_task_mutex_ → …），所有线程按同一顺序加锁，避免 AB-BA 死锁。

5. **超时与降级**  
   对“可能卡住”的步骤（如单次 detectIntraSubmapLoop、单次 forceUpdate）设超时或迭代上限，超时则跳过本帧回环或降级，保证后端线程不无限阻塞。

---

## 三、推荐方案（按优先级）

### 方案 A：缩小 keyframe_mutex_ 范围（优先、改动集中）

**思路**：keyframe_mutex_ 只保护“创建关键帧 + 加入子图”的原子性；**子图内回环、ISAM2、viz** 全部在**释放 keyframe_mutex_ 之后**执行。

**建议实现**：

1. **worker_threads.cpp**  
   不要在“整段 tryCreateKeyFrame”外持锁，改为只对“需要互斥的一小段”持锁（见下）。

2. **automap_system.cpp 的 tryCreateKeyFrame** 拆成两阶段：  
   - **阶段 1（可持 keyframe_mutex_）**：  
     - shouldCreateKeyFrame  
     - voxel downsample、GPS 查询、createKeyFrame  
     - submap_manager_.addKeyFrame(kf)  
     - 返回 `KeyFrame::Ptr`（及必要上下文，如 has_gps、pos_map、cov 等）。  
   - **阶段 2（在 keyframe_mutex_ 外）**：  
     - detectIntraSubmapLoop(active_sm, query_idx)  
     - isam2_optimizer_.addKeyFrameNode / addGPSFactorForKeyFrame  
     - forceUpdate（若本帧有回环）  
     - viz 队列 push  

3. **worker_threads.cpp 中**：  
   - 先持 keyframe_mutex_，只调用“阶段 1”（例如 `tryCreateKeyFramePhase1`），得到 kf（或 null）。  
   - 释放 keyframe_mutex_。  
   - 若 kf 非空，再调用“阶段 2”（例如 `tryCreateKeyFramePhase2(kf, ...)`），**不再持 keyframe_mutex_**。

这样即使 detectIntraSubmapLoop 或 ISAM2 卡住/极慢，也不会长时间占用 keyframe_mutex_，其它需要该锁的路径（若有）不会被拖死，且便于用超时/日志定位卡点。

### 方案 B：子图内回环异步化（进一步解耦）

**思路**：子图内回环不与“当前帧关键帧创建”强绑定在同一线程；当前帧只做“加入子图 + 入图节点/因子”，回环作为**异步任务**投递。

- 在 tryCreateKeyFrame 中（在**已释放 keyframe_mutex_** 之后）：  
  - 将“本子图 + 当前 query_idx”打包为任务，投递到 **opt_worker** 或专门的 **loop_worker** 队列。  
- 在 opt_worker/loop_worker 中：  
  - 调用 detectIntraSubmapLoop；  
  - 若有回环，再 addLoopFactorDeferred + forceUpdate。  

这样后端 worker 几乎不再执行 FPFH/TEASER，只做轻量入图与投递，阻塞风险进一步降低。

### 方案 C：ISAM2 调用不持 keyframe_mutex_

当前实现中，addKeyFrameNode / addGPSFactorForKeyFrame 已在 tryCreateKeyFrame 内、且与 detectIntraSubmapLoop 同处一大段逻辑；在方案 A 中它们会自然移到“阶段 2”，即**锁外**执行，无需再单独持 keyframe_mutex_。  
若后续有其它路径在持 keyframe_mutex_ 时调 ISAM2，应一律改为“先释放 keyframe_mutex_ 再调 ISAM2”。

### 方案 D：锁顺序文档化与检查

- 在文档或头文件中**固定锁顺序**，例如：  
  `keyframe_mutex_ → submap_update_mutex_ → opt_task_mutex_ → loop_trigger_mutex_`（或你们实际采用的顺序）。  
- 代码审查与静态检查：确保没有“先 opt_task_mutex_ 再 keyframe_mutex_”等反向顺序。  
- loop_detector 内部（desc_mutex_、intra_prepare_mutex_、db_mutex_ 等）与 AutoMapSystem 的锁之间若有交叉，建议在注释中标明“不得在持 keyframe_mutex_ 时调用会拿 loop_detector 某锁的接口”，当前方案 A 通过“先释放 keyframe_mutex_ 再做 detectIntraSubmapLoop”满足这一点。

### 方案 E：超时与降级（增强鲁棒性）

- 对 **detectIntraSubmapLoop**：  
  - 设 wall-clock 超时（如 5s）或最大迭代/点数上限；超时则本帧跳过子图内回环并打 WARN。  
- 对 **forceUpdate / commitAndUpdate**：  
  - 若已有或可加“带超时的 wait”（例如 try_wait_for 若干秒），超时则跳过本帧的 forceUpdate 并告警，避免单帧拖死整个后端。

---

## 四、落地步骤建议

1. **短期（方案 A）**  
   - 在 automap_system.cpp 中把 tryCreateKeyFrame 拆成 Phase1（持锁：创建 KF + addKeyFrame）和 Phase2（锁外：intra_loop + ISAM2 + viz）。  
   - 在 worker_threads.cpp 中改为：持 keyframe_mutex_ 只调 Phase1，释放锁后再调 Phase2。  
   - 回归测试（含复现 51 的 bag），确认不再卡住且轨迹/回环结果可接受。

2. **中期（方案 B + E）**  
   - 将子图内回环投递到 opt_worker 或 loop_worker，并加超时/降级。  
   - 统一锁顺序并做一次全代码路径检查。

3. **长期**  
   - 考虑“关键帧流水线”完全任务化：创建关键帧、描述子、回环、ISAM2 均为队列驱动，后端 worker 只负责出队、轻量决策和投递，进一步避免单点阻塞。

---

## 五、小结

- **根因**：后端在**持 keyframe_mutex_** 期间执行了整条 tryCreateKeyFrame，包含重计算（detectIntraSubmapLoop）和可能阻塞的 ISAM2 调用，易导致长时间阻塞或死锁。  
- **设计要点**：  
  - 缩小 keyframe_mutex_ 保护范围，只保护“创建 KF + 加入子图”；  
  - 持锁期间不调用重计算、不等待其它线程、不调可能阻塞的第三方；  
  - 子图内回环与 ISAM2 在锁外执行或异步化，并加超时/降级。  
- **优先实现**：方案 A（拆 Phase1/Phase2 + 锁只包 Phase1），改动集中、风险可控，且便于后续接方案 B、E。

按上述方式调整后，可以从设计上避免“后端在 tryCreateKeyFrame 内卡死”一类问题，即便 FPFH/TEASER 或 ISAM2 出现异常，也不会长时间占用 keyframe_mutex_，便于定位和降级。

---

## 六、已实现项（当前代码）

- **方案 A**：已实现并演进。当前实现已无 `keyframe_mutex_`：关键帧由 `tryCreateKeyFrame` 创建后投递 `KEYFRAME_CREATE` 到 opt 队列，由 **opt_worker** 执行 `addKeyFrame` 与 ISAM2；`detectIntraSubmapLoop`、ISAM2、viz 均在锁外或异步执行（见 `keyframe_submap.cpp`、`worker_threads.cpp`）。详见 BACKEND_MODULARITY_AND_STABILITY_ANALYSIS.md。
- **方案 E（部分）**：  
  - **子图内回环超时**：当 `intra_submap_async=false` 时，子图内回环用 `std::async` + `wait_for(max_duration)` 执行，超时则本帧跳过；若上一帧的 async 未完成则本帧跳过检测，避免堆积。  
  - **单帧耗时告警**：配置 `backend.single_frame_warn_duration_sec`（默认 15.0），单帧处理耗时超过该值时打 `[BACKEND][STUCK_DIAG]` WARN，便于定位卡点。
- **子图内回环与主线程完全异步（方案 B 落地）**：  
  - 配置 `loop_closure.intra_submap_async`（默认 **true**）：后端**仅投递** `(submap, query_idx)` 到 `intra_loop_task_queue_`，由 **intra_loop_worker** 线程执行 `detectIntraSubmapLoop`，结果以 **INTRA_LOOP_BATCH** 投递到 **opt_worker**，由 opt_worker 执行 addSubMapNode + addLoopFactorDeferred + forceUpdate。  
  - 主线程（后端）、回环线程（loop_trigger、intra_loop_worker）、GPS 线程、优化线程（opt_worker）彼此异步，**子图内回环再也不会卡住主线程**。
