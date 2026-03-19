# 根因分析：finish_mapping 后建图结果“未记录”与进程不退出

## 1. 现象与结论（先给结论）

- **建图结果实际上已经写入**：`finish_mapping` 被调用后，`saveMapToFiles()` 已执行，轨迹与地图已写到 `output_dir`（如 `/data/automap_output`），服务也返回了 `success=True, message='Map saved, requesting shutdown'`。
- **用户感知“未记录”的原因**：进程在保存完成后**没有退出**，一直空转（HEARTBEAT、回环检测等），容易让人误以为“没跑完”或“没保存”；若运行在容器内且未挂载 `output_dir`，宿主机上也看不到输出目录。
- **根本原因**：`rclcpp::shutdown()` 被调用后，**MultiThreadedExecutor 的 spin() 没有返回**，导致节点从未析构；析构里才设置的 `shutdown_requested_` 从未置位，所有 worker 线程一直 `while(!shutdown_requested_)` 空转，进程无法退出。而 spin() 不返回的直接原因是：**某个 executor 工作线程在 shutdown 时正卡在耗时回调（回环检测 TEASER/FPFH）里，该回调不主动检查 context 是否已 shutdown，导致该线程迟迟不退出，spin() 只能一直等待。**

---

## 2. 日志与代码证据

### 2.1 finish_mapping 确实执行且保存成功

日志片段（`full.log`）：

```
12:29:16 [bash-8] requester: making request: std_srvs.srv.Trigger_Request()
12:29:16 [run_under_gdb.sh-3] [AutoMapSystem][PIPELINE] event=finish_mapping_service (final HBA + save + shutdown)
12:29:16 [run_under_gdb.sh-3] [AutoMapSystem][HBA][TRACE] step=finish_mapping_ensureBackend_enter
...
12:29:22 [run_under_gdb.sh-3] [AutoMapSystem][TRAJ_LOG] wrote trajectory_odom ... to /data/automap_output/trajectory_odom_20260319_121724.csv and logs/...
12:29:22 [run_under_gdb.sh-3] [AutoMapSystem][PIPELINE] event=finish_mapping_save_done output_dir=/data/automap_output
12:29:22 [run_under_gdb.sh-3] [AutoMapSystem] finish_mapping: requesting context shutdown (end mapping)
12:29:22 [bash-8] response: std_srvs.srv.Trigger_Response(success=True, message='Map saved, requesting shutdown')
```

说明：服务回调执行完毕、保存完成、已调用 `rclcpp::shutdown()`，且客户端收到成功响应。

### 2.2 节点析构从未执行

全日志搜索：

```bash
grep '\[SHUTDOWN\]\|destructor entered' full.log
# 结果：No matches
```

`AutoMapSystem::~AutoMapSystem()` 中第一步就会打 `[SHUTDOWN][step=1] destructor entered`（见 `system_init.cpp`）。没有任何匹配说明**析构从未被调用**，即 `exec.spin()` 返回后执行的 `exec.remove_node(node)` 从未走到，或 spin() 根本没返回。

### 2.3 shutdown 后进程仍在打日志

12:29:22 之后仍有大量：

- `[MAP_PUB] start`、`publishGlobalMap`、`buildGlobalMapInternalFromSnapshot`
- `[INTRA_LOOP][REJECT]`、`FPFH_CRASH_TRACE`、`TeaserMatcher`、`findCorrespondences`
- `[New Thread ...]`、HEARTBEAT 等

说明：在“requesting context shutdown”之后，**其他 executor 线程或 worker 线程仍在跑**；节点未析构，故 `shutdown_requested_` 从未被置为 true，worker 的 `while (!shutdown_requested_.load())` 一直为真。

### 2.4 设计上的因果链

- **main()**（`automap_system_node.cpp`）：  
  `exec.add_node(node)` → `exec.spin()` → **只有 spin() 返回**才会执行 `exec.remove_node(node)`，node 引用计数减为 0 时才会析构。
- **~AutoMapSystem()**（`system_init.cpp`）：  
  析构里才 `shutdown_requested_.store(true)` 并 `notify_all`、然后逐个 `join` worker 线程；若析构不执行，这些都不会发生。
- **handleFinishMapping()**（`service_handlers.cpp`）：  
  只做了 `saveMapToFiles()` 和 `rclcpp::shutdown()`，**没有**设置 `shutdown_requested_`。因此“请求 ROS 关闭”与“请求本节点 worker 收尾”是解耦的，只有析构才能触发后者。
- **MultiThreadedExecutor**：  
  `rclcpp::shutdown()` 后，需要**所有** executor 工作线程都从当前可执行单元里退出，spin() 才会返回。若某一个线程正阻塞在**长时间、且不频繁检查 context 是否已 shutdown** 的回调里，该线程会拖住 spin()。

---

## 3. 根本原因归纳

1. **直接原因（为什么进程不退出）**  
   - `rclcpp::shutdown()` 被调用了，但 **exec.spin() 没有返回**。  
   - 因此 `remove_node(node)` 未执行，**~AutoMapSystem() 从未运行**，`shutdown_requested_` 从未被设为 true，所有 backend/loop_trigger/intra_loop_worker 等线程继续 `while(!shutdown_requested_)` 空转，进程无法退出。

2. **为什么 spin() 不返回**  
   - 使用 **MultiThreadedExecutor（4 线程）**，在 shutdown 时刻，至少有一个 executor 线程正在执行**耗时且不主动让出**的回调。  
   - 日志在 12:29:16–12:29:22 及之后仍有大量 **回环检测（TEASER/FPFH/findCorrespondences）** 等输出，说明这些回调在 shutdown 后仍在执行。  
   - 这些回调若内部没有在关键循环里检查 `rclcpp::ok()` 或 context 的 shutdown 状态，就会一直跑到结束，导致该 executor 线程迟迟不退出，**spin() 会一直等所有工作线程退出才返回**。

3. **为什么用户认为“建图结果没记录”**  
   - 从日志看，结果已写入 `output_dir`（如 `/data/automap_output`）和 `logs/`。  
   - 若运行在 Docker 且未把 `output_dir` 挂载到宿主机，用户在本机看不到文件；或用户以“进程是否正常退出”作为“是否记录完成”的判断，进程未退出就会误以为没记录。

---

## 4. 设计缺陷小结

| 点 | 说明 |
|----|------|
| finish_mapping 不设 shutdown_requested_ | 只调用了 `rclcpp::shutdown()`，没有 `shutdown_requested_.store(true)`，无法让 worker 在“保存完成”后立即开始收尾，必须等析构。 |
| 依赖析构触发收尾 | 只有 spin() 返回 → remove_node → 析构，才会 set shutdown 并 join；若 spin() 不返回，整条链断掉。 |
| 耗时回调不检查 shutdown | 回环检测等回调若在长循环/阻塞调用中不检查 `rclcpp::ok()`，会拖住 executor 线程，进而拖住 spin() 返回。 |

---

## 5. 建议修复方向（简要）

1. **在 handleFinishMapping 里显式设置 shutdown_requested_**  
   在调用 `rclcpp::shutdown()` 之前或之后立刻 `shutdown_requested_.store(true)` 并 `notify_all`，让所有 worker 在 spin() 还未返回时就能开始退出，减少“保存完成但进程永远不结束”的情况。
2. **耗时回调中定期检查 rclcpp::ok()**  
   在回环检测、TEASER/FPFH 等长耗时路径中，在合适循环或阶段检查 `rclcpp::ok()`，一旦 context 已 shutdown 则尽快返回，避免拖住 executor 线程。
3. **确认输出路径与挂载**  
   若在容器内运行，确保 `output_dir`（如 `/data/automap_output`）挂载到宿主机或用户可见目录，并在文档中说明“建图结果已写入该目录，与进程是否退出无关”。

---

## 6. 与本分析对应的日志与代码位置

- 日志：`logs/run_20260319_121518/full.log`  
  - 12:29:11 ros2-1 (bag) 结束，bash-8 启动；12:29:16 发起 finish_mapping；12:29:22 save_done、requesting shutdown、bash-8 收到 success；之后无 `[SHUTDOWN]`，持续 HEARTBEAT/回环日志。
- 代码：  
  - `automap_pro/src/system/modules/service_handlers.cpp`：`handleFinishMapping` 仅 `rclcpp::shutdown()`，未设 `shutdown_requested_`。  
  - `automap_pro/src/system/modules/system_init.cpp`：`~AutoMapSystem()` 中 `shutdown_requested_.store(true)` 与各线程 join。  
  - `automap_pro/src/nodes/automap_system_node.cpp`：`exec.spin()` 返回后才 `remove_node`。
