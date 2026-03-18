# P1 项具体改法（逐条）

本文档给出 [BACKEND_DEEP_ANALYSIS_LOGIC_AND_BLOCKING.md](BACKEND_DEEP_ANALYSIS_LOGIC_AND_BLOCKING.md) 中 P1 两项的**具体改法**：文件、行号、修改前后代码与说明。

---

## P1-1：子图内回环超时后长期不提交新任务

**问题**：`intra_submap_async=false` 且 `use_timeout=true` 时，超时后不赋新 `std::async`，下一帧仍看到 `intra_loop_future_.valid()==true` 且未 ready 则本帧跳过，导致后续多帧连续跳过、漏加回环约束。

**思路**：超时后（1）将当前未完成的 `intra_loop_future_` 移到后台线程中 `get()`，避免在本线程析构时阻塞；（2）立即为本帧启动新的 `std::async` 并赋给 `intra_loop_future_`，本帧不再等待（不添加回环）；下一帧会等待这个新 async，从而恢复提交。

**涉及文件**：`automap_pro/src/system/automap_system.cpp`（约 1624–1632 行）。

**修改前（超时分支）**：

```cpp
                        if (status == std::future_status::ready) {
                            loops = intra_loop_future_.get();
                        } else {
                            RCLCPP_WARN(get_logger(),
                                "[INTRA_LOOP] timeout after %.1fs (intra_submap_max_duration_sec=%.1f), skip adding loop factors this frame",
                                max_intra_sec, max_intra_sec);
                        }
```

**修改后**：

```cpp
                        if (status == std::future_status::ready) {
                            loops = intra_loop_future_.get();
                        } else {
                            RCLCPP_WARN(get_logger(),
                                "[INTRA_LOOP] timeout after %.1fs (intra_submap_max_duration_sec=%.1f), skip adding loop factors this frame",
                                max_intra_sec, max_intra_sec);
                            // 将未完成的 future 移到后台线程等待，避免析构阻塞；立即为本帧启动新 async，下一帧可复用
                            using FutureT = std::future<std::vector<LoopConstraint::Ptr>>;
                            FutureT old_future = std::move(intra_loop_future_);
                            std::thread([f = std::move(old_future)]() mutable {
                                try { if (f.valid()) f.get(); } catch (...) {}
                            }).detach();
                            intra_loop_future_ = std::async(std::launch::async,
                                [this](const SubMap::Ptr& sm, int qidx) { return loop_detector_.detectIntraSubmapLoop(sm, qidx); },
                                active_sm, query_idx);
                        }
```

**说明**：

- `std::thread` 已在当前文件中通过 `#include <thread>` 包含。
- `intra_loop_future_` 类型为 `std::future<std::vector<LoopConstraint::Ptr>>`，与 `old_future` 一致。
- 本帧超时后不再 `wait_for` 新 async，本帧不加回环；下一帧会进入 `if (intra_loop_future_.valid())` 分支并等待此次新启动的任务，从而避免长期跳过。

---

## P1-2：单帧 >60s 时 ISAM2 reset 与 SubMapManager 状态不一致

**问题**：单帧处理超过 60s 时调用 `isam2_optimizer_.reset()`，图与估计被清空，但 SubMapManager/KeyFrameManager 未同步清理，后续 `addKeyFrameNode` 等可能依赖已不存在的节点，产生图与子图不一致。

**思路**：  
（1）增加原子标志 `session_invalid_after_isam2_reset_`，在调用 `isam2_optimizer_.reset()` 时置为 `true`。  
（2）在后端循环中，在决定「本帧要 process」之后、进入 `tryCreateKeyFrame` 之前检查该标志；若为 `true` 则本帧跳过（不建 KF、不加入图），并打限频日志。  
（3）在「显式重置会话」的入口将该标志置回 `false`：例如在 `handleLoadSession` 成功加载后清除，以便新会话可继续建图；若未使用 load_session，需重启节点或后续增加「reset_after_stuck」类服务。

**涉及文件**：

- `automap_pro/include/automap_pro/system/automap_system.h`：新增成员。
- `automap_pro/src/system/automap_system.cpp`：三处——reset 处置位、backend 循环检查并跳过、handleLoadSession 成功处清除。

### 2.1 头文件新增成员

**文件**：`automap_pro/include/automap_pro/system/automap_system.h`  
**位置**：与 `last_backend_step_id_`、`intra_loop_future_` 等原子/后端状态成员放在一起（例如约 279 行附近）。

**新增**：

```cpp
    /** 单帧 >60s 触发 ISAM2 强制 reset 后置位；为 true 时拒绝新 KF 直至 load_session 或重启 */
    std::atomic<bool> session_invalid_after_isam2_reset_{false};
```

### 2.2 reset 时置位

**文件**：`automap_pro/src/system/automap_system.cpp`  
**位置**：约 1205–1210 行，`isam2_optimizer_.reset()` 调用附近。

**修改前**：

```cpp
        if (duration_ms > 60000.0) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][STUCK] processed_no=%d ts=%.3f duration_ms=%.1f (>60s) - forcing ISAM2 reset! (SubMapManager/前端未同步，轨迹可能断链；grep BACKEND STUCK)",
                        processed_no, f.ts, duration_ms);
            MetricsRegistry::instance().incrementCounter(metrics::ISAM2_FORCED_RESET, 1.0);
            isam2_optimizer_.reset();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STUCK] ISAM2 reset complete after stuck detection");
```

**修改后**：

```cpp
        if (duration_ms > 60000.0) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][STUCK] processed_no=%d ts=%.3f duration_ms=%.1f (>60s) - forcing ISAM2 reset! (SubMapManager/前端未同步，轨迹可能断链；grep BACKEND STUCK)",
                        processed_no, f.ts, duration_ms);
            MetricsRegistry::instance().incrementCounter(metrics::ISAM2_FORCED_RESET, 1.0);
            session_invalid_after_isam2_reset_.store(true, std::memory_order_release);
            isam2_optimizer_.reset();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STUCK] ISAM2 reset complete after stuck detection (new KF rejected until load_session or restart)");
```

### 2.3 后端循环：检查标志并跳过

**文件**：`automap_pro/src/system/automap_system.cpp`  
**位置**：在 `will_process` 为真且已 `fetch_add(processed_no)` 之后、在 `odomCacheGet` 之前检查（约 1108–1126 之间，建议紧接在「CRASH_CONTEXT」日志块之后、`// 按时间戳从缓存对齐` 之前）。

**插入**：

```cpp
        if (session_invalid_after_isam2_reset_.load(std::memory_order_acquire)) {
            static std::atomic<int> reject_log_count{0};
            if (reject_log_count.fetch_add(1) % 100 == 0) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND] rejecting KF (session_invalid_after_isam2_reset); call load_session or restart node to recover (processed_no=%d frame_no=%d)", processed_no, frame_no);
            }
            continue;
        }
```

这样在「显式 reset」之前都不会再建 KF，避免图与子图不一致。

### 2.4 清除标志：load_session 成功时

**文件**：`automap_pro/src/system/automap_system.cpp`  
**位置**：`handleLoadSession` 中，在 `res->success = (loaded > 0);` 及后续赋值、打日志之后（约 3381–3386 行），增加清除逻辑。

**在** `RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=session_loaded ...` **之后插入**：

```cpp
        session_invalid_after_isam2_reset_.store(false, std::memory_order_release);
```

这样加载新 session 后可以继续建图。若从不调用 load_session，只能通过重启节点恢复；后续可再增加例如 `/automap/reset_after_stuck` 服务仅清除该标志。

---

## 小结

| P1 项 | 修改文件 | 修改要点 |
|-------|----------|----------|
| P1-1 子图内回环超时 | automap_system.cpp | 超时分支：move 旧 future 到 detach 线程 get()，再为本帧启动新 async 赋给 `intra_loop_future_` |
| P1-2 ISAM2 reset 不一致 | automap_system.h | 新增 `session_invalid_after_isam2_reset_` |
| P1-2 | automap_system.cpp | reset 时 store(true)；backend 循环检查并 continue；handleLoadSession 成功时 store(false) |

按上述逐条修改即可落地 P1 两项；修改后建议跑一次后端单测或建图流程，确认无回归。
