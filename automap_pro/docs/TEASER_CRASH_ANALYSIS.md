# TEASER++ 析构崩溃分析与精准定位

## Executive Summary

- **现象**：回环匹配线程在 `teaser_inlier_rejected` 后发生 SIGSEGV，崩溃于 `free()`，调用栈在 `teaser::RobustRegistrationSolver::~RobustRegistrationSolver()`。
- **根因**：TEASER++ 依赖 PMC（Parallel Maximum Clique）与 OpenMP；在**极少 inlier**（如 2/16）路径下，solver 析构时内部某处对 C 分配内存调用 `free()`，导致崩溃（多为 double-free 或 PMC/OpenMP 多线程析构顺序问题）。
- **已做**：① `teaser_matcher` 中 **`params.max_clique_num_threads = 1`**（强制 PMC 单线程，从根源减少析构时多线程 free）；② `[CRASH_TRACE]` 精准日志且带 **lwp**（与 GDB `info threads` 的 LWP 对应）；③ reset 前 **std::cerr 立即刷新**，便于崩溃瞬间仍能定位；④ match worker 内 `OMP_NUM_THREADS=1`。
- **建议**：若仍崩溃，根据“最后一条 CRASH_TRACE”的 step + lwp 判断崩溃点与线程，并尝试进程级 `OMP_NUM_THREADS=1` 或升级/修补 TEASER++/PMC。

---

## 1. 崩溃现象（来自终端日志）

```text
step=teaser_inlier_computed inliers=2/14 ratio=0.14 thresh=0.3
step=teaser_inlier_rejected inlier_ratio=0.14 < thresh=0.3

Thread 18 "automap_system_" received signal SIGSEGV, Segmentation fault.
#0  0x00007ffff799e3fe in free () from /lib/x86_64-linux-gnu/libc.so.6
#1  0x00007ffff56b283e in teaser::RobustRegistrationSolver::~RobustRegistrationSolver()
#2  0x00007ffff56af248 in automap_pro::TeaserMatcher::match(...)
#3  0x00007ffff56e7af5 in automap_pro::LoopDetector::processMatchTask(...)
#4  0x00007ffff56eac45 in automap_pro::LoopDetector::matchWorkerLoop()
```

- 崩溃发生在 **inlier 被拒绝** 之后，即我们执行 `solver.reset()` 时，`unique_ptr` 触发 `RobustRegistrationSolver` 析构，析构内部调用到 `free()` 导致 SIGSEGV。

---

## 2. 根因分析

| 层级 | 说明 |
|------|------|
| 直接位置 | `teaser::RobustRegistrationSolver` 的析构函数（编译器生成，依次析构成员） |
| 依赖链 | TEASER++ → PMC（max clique）→ OpenMP；PMC 或内部 BLAS/LAPACK 可能使用 C 的 malloc/free |
| 触发条件 | 少 inlier（如 2/14）、PMC_HEU 求解后，solver 内部状态处于“小 clique”或特殊图结构，析构时某块内存被重复释放或释放了非法指针 |
| 官方说明 | TEASER++ 文档 [known_issues.rst] 提到 PMC 可能产生 segmentation faults，建议设置 `OMP_NUM_THREADS` |

推断：**PMC/OpenMP 在多线程或特定图规模下，析构顺序或内存所有权不一致，导致在 `free()` 时崩溃。**

---

## 3. 精准日志（CRASH_TRACE）用法

在 `teaser_matcher.cpp` 中已加入 `[CRASH_TRACE]` 日志，用于精确定位崩溃发生在哪一步。每条 CRASH_TRACE 包含 **tid**（线程 hash）与 **lwp**（Linux 线程 ID，与 GDB `info threads` 中的 LWP 一致）。

| 日志 step | 含义 |
|-----------|------|
| `teaser_solver_created` | solver 刚创建，ptr 已打出来 |
| `teaser_solver_reset_before` (invalid path) | solution 无效，即将 `solver.reset()` |
| `teaser_solver_reset_after` (invalid path) | solution 无效路径上 reset 已返回 |
| `teaser_solver_reset_before` (inlier_rejected path) | inlier 不足，即将 `solver.reset()` |
| `teaser_solver_reset_after` (inlier_rejected path) | inlier 不足路径上 reset 已返回 |
| `teaser_solver_scope_exit success_path` | 成功路径上 solver 即将离开作用域（随后析构） |

**与 GDB 对应**：崩溃时 GDB 会显示 `Thread N "..." received signal SIGSEGV` 及 LWP。在日志中搜索 `lwp=<LWP>` 即可对应到具体 step（例如 `step=teaser_solver_reset_before ... inlier_rejected`）。reset 前会额外写一行到 **stderr**（`[CRASH_TRACE] lwp=... step=... about to destruct`），即使标准日志缓冲未刷新也能看到。

**用法**：崩溃后查看**最后一条**带 `[CRASH_TRACE]` 的日志（或 stderr 最后一行）：

- 最后一条是 `teaser_solver_reset_before (inlier_rejected path)` 且**没有**对应的 `teaser_solver_reset_after` → 崩溃发生在 **solver 析构内部**（与当前现象一致）。
- 若是 `teaser_solver_created` 之后、且没有任一 `reset_before` → 可能崩溃在 `solve()` 或 `getInlierMaxClique()` 等调用中。

---

## 4. 已做缓解

1. **强制 PMC 单线程 `max_clique_num_threads = 1`**（`teaser_matcher.cpp`）  
   - 在 TEASER `Params` 中设置 `params.max_clique_num_threads = 1`，使 PMC 最大团求解仅用 1 线程，从根源避免析构时多线程 free 导致的 SIGSEGV（优先于仅依赖 `OMP_NUM_THREADS`）。

2. **Match worker 内设 `OMP_NUM_THREADS=1`**（`loop_detector.cpp`）  
   - 在 `matchWorkerLoop()` 入口处对 Unix/Linux 调用 `setenv("OMP_NUM_THREADS", "1", 1)`，进一步限制 OpenMP 线程数。

3. **显式 `solver.reset()` 与 CRASH_TRACE（含 lwp + stderr 刷新）**  
   - 在 invalid 与 inlier_rejected 两处提前 `solver.reset()`，并打 `reset_before` / `reset_after`（带 tid、lwp）；reset 前写一行到 `std::cerr` 并 `endl` 刷新，便于崩溃瞬间仍能定位。

4. **最少对应点门槛**  
   - 至少 12 对对应点才进入 TEASER，减少“极少对应点 + 极少 inlier”的退化路径。

---

## 5. 若仍崩溃：排查与进一步缓解

1. **进程级限制 OpenMP**  
   启动前：
   ```bash
   export OMP_NUM_THREADS=1
   ./your_automap_node
   ```
   或 launch 里设置 `env.OMP_NUM_THREADS='1'`。

2. **确认崩溃点**  
   - 看最后一条 `[CRASH_TRACE]` 是否为 `teaser_solver_reset_before (inlier_rejected path)` 且无 `teaser_solver_reset_after`；用日志中的 **lwp** 与 GDB `info threads` 中收到 SIGSEGV 的线程 LWP 对照。  
   - GDB 断在 `free` 时打印传入的指针（如 `p ptr`），确认是否重复释放或非法指针。

3. **可选**  
   - 尝试 TEASER++ 官方建议：`OMP_NUM_THREADS=${MAX_THREADS}`（与当前“单线程”相反，仅作对照）。  
   - 升级或打补丁 TEASER++/PMC，或在小 clique 路径上避免调用 PMC（需改 TEASER++ 源码）。

---

## 6. 相关文件

- `automap_pro/src/loop_closure/teaser_matcher.cpp`：TEASER 调用与 CRASH_TRACE。
- `automap_pro/src/loop_closure/loop_detector.cpp`：match worker 与 `OMP_NUM_THREADS=1`。
- TEASER++：`teaser/include/teaser/registration.h`，`teaser/src/registration.cc`；PMC：`teaser/src/graph.cc`。
- 官方已知问题：TEASER++ 源码 `doc/known_issues.rst`。

---

## 7. 验证是否缓解

- 使用曾触发崩溃的 bag（如 M2DGR street_03）回放，观察：
  - 是否仍出现 `step=teaser_inlier_rejected` 后 SIGSEGV；
  - 若不再崩溃，且最后一条 CRASH_TRACE 为 `teaser_solver_reset_after (inlier_rejected path)`，可认为本次缓解有效。
- 若仍崩溃，收集：最后几条 CRASH_TRACE 日志 + GDB `free` 处指针值 + 对应栈，便于进一步定位或提交上游。
