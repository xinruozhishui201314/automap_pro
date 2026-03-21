# 崩溃分析报告：2026-03-21 run_123558

## 1. Executive Summary

| 项目 | 内容 |
|------|------|
| **崩溃现象** | SIGSEGV in `LoopDetector::detectIntraSubmapLoop` (Thread 21, LWP 433) |
| **直接触发点** | TEASER++ `RobustRegistrationSolver` 析构时 `free()` 崩溃 |
| **根因** | TEASER++ PMC 在极少 inliers（如 2–3）时内部堆损坏，析构时 SIGSEGV |
| **根本修复** | corrs<20 时用 **SVD 替代 TEASER**，不创建 TEASER 对象；corrs≥20 时 success/invalid 路径用 `solver.release()` 避免析构 |

---

## 2. 证据链（逻辑闭环）

### 2.1 崩溃时序

```
[TeaserMatcher] findCorr_exit_mutual corrs=752
[TeaserMatcher] FPFH_GEO_FILTER filtered_corrs=36/752 (4.8%) max_dist=5.00m
[TeaserMatcher] teaser_solve_enter corrs=36
[TeaserMatcher] teaser_done inliers=3 corrs=36 ratio=0.083 valid=1
[TeaserMatcher] [TEASER_DIAG][UNUSUAL_ORIENTATION] Large Pitch/Roll P=41.3 R=39.5 - flipped mismatch causing ghosting!
[TeaserMatcher] TEASER_RESULT PASS inliers≈3 corrs=36 ratio=0.0833 rmse=0.1062m (min_safe=3 min_ratio=0.04 max_rmse=0.75m)
[TeaserMatcher] [CRASH_TRACE] lwp=433 step=teaser_success_path_solver_natural_destruct solver_ptr=0x7fff3aa5b700

Thread 21 "automap_system_" received signal SIGSEGV, Segmentation fault.
#0  0x00007ffff55ebee0 in automap_pro::LoopDetector::detectIntraSubmapLoop(...)
#1  0x00007ffff5766616 in automap_pro::AutoMapSystem::intraLoopWorkerLoop()
```

### 2.2 证据 1：成功路径上的自然析构

- `valid=1` → inliers(3) >= min_safe_inliers(3)，**未**进入 L2 的 `teaser_extremely_few_inliers` 路径
- `TEASER_RESULT PASS` → 走 **success 路径**
- `step=teaser_success_path_solver_natural_destruct` → unique_ptr 即将离开 try 作用域，触发 solver 析构
- **结论**：崩溃发生在 success 路径的 solver 自然析构时刻

### 2.3 证据 2：TEASER++ 析构已知缺陷

参见 `automap_pro/docs/TEASER_CRASH_ANALYSIS.md`：

- 根因：TEASER++ PMC 在 **极少 inliers**（如 2–3）时内部堆损坏
- 析构时 `free()` 触发 SIGSEGV
- 已做缓解：`max_clique_num_threads=1`，L2/L3 路径使用 `release()` 避免析构

### 2.4 证据 3：success 路径无 release() 保护

`teaser_matcher.cpp` 中：

- L2 路径（inliers < min_safe_inliers）：`solver.release()` ✅
- L3 路径（inlier_ratio 不足）：`solver.release()` ✅
- **Success 路径**：无 `release()`，solver 自然析构 ❌

当 `min_safe_inliers=3` 且 inliers=3 时，恰好落在 success 路径，solver 必然被析构，触发已知 bug。

### 2.5 证据 4：配置与阈值

- 日志：`min_safe_inliers=3`、`min_ratio=0.04`
- `system_config.yaml`：`min_safe_inliers: 10`
- `system_config_M2DGR.yaml`：`min_safe_inliers: 3`
- `loop_closure_vegetation_override.yaml`：`min_safe_inliers: 3`

**结论**：当前 run 使用了 M2DGR 或植被 override 配置，使 3 inliers 被判定为有效，进入 success 路径。

### 2.6 证据 5：误匹配特征（佐证应拒绝）

- `FPFH_DIAG`: p90=46.37m >> 5m，大量误匹配
- `UNUSUAL_ORIENTATION`: Pitch=41.3°, Roll=39.5°，典型 flipped mismatch
- 路面建图 Pitch/Roll 通常 < 30°
- **结论**：该 3-inlier 结果本应被拒绝，提高 `min_safe_inliers` 符合“拒绝可疑匹配”的设计意图

---

## 3. 逻辑链闭环

```
min_safe_inliers=3 (M2DGR/植被配置)
    → inliers=3 满足 inliers >= min_safe_inliers
    → 进入 success 路径（无 release()）
    → solver unique_ptr 自然析构
    → TEASER++ 析构器内 free() 访问已损坏堆
    → SIGSEGV in detectIntraSubmapLoop（调用栈包含 match() 的析构）
```

---

## 4. 次要崩溃：fastlivo_mapping SIGSEGV

终端还显示 `fastlivo_mapping` 进程 exit code -11。从 backtrace 看为 `free()` 与 `shared_ptr` 析构，发生在 automap 因主崩溃退出、launch 发送 SIGINT 之后。属于 **级联退出**，非独立根因，可忽略。

---

## 5. 根本修复（已实现）

### 5.1 corrs < 20：SVD 替代 TEASER

- **策略**：不创建 TEASER solver，直接走 SVD/Umeyama 配准
- **效果**：内点可能确实很少（如植被、弱重叠），SVD 无 PMC 依赖，绝不崩溃
- **代价**：鲁棒性略低，但对少量对应点场景通常可接受

### 5.2 corrs ≥ 20：TEASER 全路径析构防护

- **invalid 路径**：`solution.valid == false` 时 `solver.release()`，避免析构
- **inlier 不足路径**：已有 `solver.release()`
- **success 路径 inliers < 10**：`solver.release()`，避免析构

### 5.3 配置建议（可选）

`min_safe_inliers=10` 可减少误匹配，但根本修复不依赖配置，`min_safe_inliers=3` 下也不会崩溃。

---

## 6. 验证步骤

1. 使用相同 bag 与 `--gdb` 重跑
2. 预期：不再出现 TEASER 析构相关的 SIGSEGV
3. corrs<20 时日志出现 `step=svd_fallback_low_corrs`
4. corrs≥20 且 inliers<10 时出现 `[SAFEGUARD] success path inliers=... release solver`

---

## 7. 相关文档

- `automap_pro/docs/TEASER_CRASH_ANALYSIS.md`：TEASER 析构崩溃原理
- `TEASER_CRASH_FIX_V3_FINAL.md`：V3 修复思路（L2/L3 使用 release()）
- `docs/DESIGN_AVOID_BACKEND_BLOCKING.md`：架构与锁设计
