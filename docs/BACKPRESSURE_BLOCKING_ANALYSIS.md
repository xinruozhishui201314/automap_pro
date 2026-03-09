# 背压实现：阻塞/卡死风险与性能分析

## 1. 当前实现简述

- **背压策略**：帧队列满时，`onCloud()` 在 `frame_queue_not_full_cv_` 上等待，不丢帧。
- **调用链**：ROS2 订阅回调 → `LivoBridge::onCloud` → `AutoMapSystem::onCloud`。
- **节点入口**：`automap_system_node` 使用 `rclcpp::spin(node)`（单线程 Executor）。

---

## 2. 阻塞与卡死风险

### 2.1 阻塞 Executor（主要问题）

| 现象 | 原因 |
|------|------|
| **订阅回调长时间不返回** | `onCloud()` 在队列满时 `wait_for(60s)`，占用执行回调的那条线程。 |
| **单线程 spin** | `rclcpp::spin(node)` 只有一条线程派发所有回调；该线程被 `onCloud` 占住后，**同一节点**的 odom、GPS、kfinfo、定时器等回调都无法执行。 |
| **后果** | odom/kfinfo 缓存得不到更新；worker 虽在另一线程消费，但若依赖最新 odom（按时间戳对齐），可能因缓存未更新而 skip；长时间“只进不出”会表现为界面/逻辑卡顿、像卡死。 |

因此：**当前实现会在队列满时长时间阻塞 Executor 线程**，存在“卡住”和性能未充分发挥的问题。

### 2.2 死锁可能性

- **worker ↔ onCloud**：worker 只做 `pop` + `notify_one(not_full)`，不依赖 Executor；onCloud 只等 `not_full` 或 shutdown。**同一把锁** `frame_queue_mutex_` 使用顺序一致（先等 cv 再操作队列），**无循环等待**，当前设计下**不会死锁**。
- **析构**：`shutdown_requested_=true` 且 `notify_all(not_full_cv)`，等待中的 `onCloud` 会退出，**不会在析构时卡死**。

结论：**无经典死锁**；主要风险是 **Executor 被长时间占用** 导致的“整节点卡顿”和缓存更新不足。

### 2.3 性能未充分发挥

- 只有 **1 个 backend worker** 消费帧队列；CPU 多核时，该线程可能跑满一核，其它核空闲。
- 若希望“充分发挥计算机性能”，可考虑：
  - **不阻塞 Executor**：背压放在独立 feeder 线程，回调只做“快速入队”。
  - **多线程 Executor**：同一节点内 odom/cloud/kfinfo 等回调并行执行，提高吞吐、减少互相阻塞。

---

## 3. 改进方案（已实现）

### 3.1 设计目标

- **数据不丢**：队列满时仍不丢帧（背压保留）。
- **尽量不阻塞 Executor**：订阅回调快速返回，长时间等待发生在**独立 feeder 线程**。
- **充分发挥多核**：节点使用 **MultiThreadedExecutor**，多个回调可并行。

### 3.2 架构：Ingress 缓冲 + Feeder 线程

```
  [ROS2 订阅] → onCloud() → 入队 ingress_queue_ (有界, 小) → 立即返回
                     ↑
                     |  若 ingress 满则短时等待（仅当 feeder 严重滞后）
                     |
  [Feeder 线程] 循环: 从 ingress 取 → 入队 frame_queue_（满则等 not_full）→ 通知 worker
```

- **onCloud()**：只负责把 (ts, cloud) 放入 **ingress 队列**（有界，如 16）；若 ingress 满才短时等待，等 feeder 取走一帧即返回。
- **Feeder 线程**：专门从 ingress 取数据并压入 **frame_queue_**；`frame_queue_` 满时在 feeder 里等 `frame_queue_not_full_cv_`，**不占用 Executor**。
- **效果**：Executor 上阻塞仅发生在“ingress 已满”时，且一次只等 feeder 移走一帧（约等于一次 worker 处理时间），阻塞时间短；背压仍由 feeder 对 `frame_queue_` 的等待实现，**数据不丢**。

### 3.3 MultiThreadedExecutor

- 节点入口改用 `rclcpp::executors::MultiThreadedExecutor`（例如 4 线程）。
- 即使某次 onCloud 在 ingress 满时短时阻塞，其它线程仍可执行 odom、GPS、kfinfo 等回调，**缓存持续更新**，worker 按时间戳对齐更可靠，整体吞吐和响应更好。

---

## 4. 配置与约束

- **ingress 队列大小**：默认 16；过小易触发 onCloud 短等，过大则内存略增（仍远小于 frame_queue_）。
- **frame_queue_max_size**：仍由配置控制；feeder 在该队列满时在**独立线程**中等待，不占用 Executor。
- **背压语义不变**：数据仍不丢；仅把“长时间等待”从订阅回调挪到 feeder 线程，从而避免 Executor 长时间阻塞、降低卡死感并更好利用多核。
