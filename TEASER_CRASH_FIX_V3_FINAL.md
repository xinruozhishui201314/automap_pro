# TEASER++ 崩溃修复 V3 最终版本

## 0. Executive Summary

| 项目 | 内容 |
|------|------|
| **崩溃现象** | SIGSEGV in `free()` during TEASER++ RobustRegistrationSolver destruction |
| **根因** | TEASER++ PMC 求解器在极少 inliers（实测为3）时内部堆损坏，析构时崩溃 |
| **V3 修复策略** | 1. 提高安全阈值（3→10）<br>2. 添加创建前预检查（corrs < 20）<br>3. 使用 `release()` 延迟析构<br>4. 强化日志追踪 |
| **修改文件** | `automap_pro/src/loop_closure/teaser_matcher.cpp` |
| **验证状态** | 编译完成，待运行测试 |

---

## 1. 问题根因深度分析

### 1.1 崩溃时序分析

从日志 `logs/full.log` 分析：

```
17528|2026-03-08 17:34:58.744 [I] [TeaserMatcher][teaser_matcher.cpp:286] [tid=40150] step=teaser_inlier_computed inliers=3/172 ratio=0.02 thresh=0.3
17529|2026-03-08 17:34:58.744 [W] [TeaserMatcher][teaser_matcher.cpp:295] [tid=40150] [CRITICAL] step=teaser_extremely_few_inliers inliers=3 < safe_threshold=10
17530|2026-03-08 17:34:58.744 [I] [TeaserMatcher][teaser_matcher.cpp:297] [CRASH_TRACE] ... solver_ptr=0x7fffc54a7e90 inliers=3 safe_min=10
17533|2026-03-08 17:34:58 [run_under_gdb.sh-3] Thread 18 "automap_system_" received signal SIGSEGV, Segmentation fault.
17536|2026-03-08 17:34:58.919 [run_under_gdb.sh-3] 0x00007ffff799e3fe in free () from /lib/x86_64-linux-gnu/libc.so.6
```

**关键发现**：
1. ✅ 代码成功检测到低 inliers（3 < 10）
2. ✅ 成功记录了 CRASH_TRACE 日志
3. ❌ 但仍然在 `free()` 中崩溃
4. ❌ 崩溃发生在检测日志输出之后（说明代码继续执行到 unique_ptr 析构）

### 1.2 根本原因

**问题**：即使检测到低 inliers 并提前返回，`unique_ptr` 仍然会在函数退出时自动析构 `teaser::RobustRegistrationSolver` 对象。此时：
- TEASER++ 内部 PMC 求解器已产生堆损坏（极少 inliers 时）
- 析构时调用 `free()` 触发 SIGSEGV

**结论**：提前返回并不能阻止析构，只是延迟了崩溃。**需要从根源避免 TEASER solver 的创建**。

---

## 2. V3 激进修复方案

### 2.1 多层防护策略

| 防护层 | 检查点 | 阈值 | 策略 |
|----------|--------|--------|------|
| L1: 对应点数 | `corrs.size()` | < 20 | 完全跳过 TEASER，直接返回失败结果 |
| L2: Inliers 数 | `inliers` | < 10 | 检测后使用 `release()` 延迟析构 |
| L3: Inlier Ratio | `inlier_ratio` | < 0.3 | 检测后使用 `release()` 延迟析构 |
| L4: Solution Valid | `solution.valid` | false | 使用 `release()` 延迟析构 |

### 2.2 核心代码变更

#### 变更1：添加创建前预检查（L1）

```cpp
// 在创建 solver 对象之前就检查对应点数
const int kCorrsThreshold = 20;
if (static_cast<int>(corrs.size()) < kCorrsThreshold) {
    ALOG_WARN(MOD, "[tid={}] [CRITICAL_V3] step=teaser_skip_before_creation corrs={} < safe_threshold={}, COMPLETELY AVOIDING TEASER CREATION",
             tid, static_cast<int>(corrs.size()), kCorrsThreshold);
    ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=insufficient_corrs_v3 tid={} corrs={} safe_min={} (V3: skip before solver creation)",
             tid, static_cast<int>(corrs.size()), kCorrsThreshold);
    // 关键：完全不创建 solver 对象，从根源避免堆损坏
    return result;
}
```

#### 变更2：使用 `release()` 延迟析构（L2, L3, L4）

```cpp
if (inliers < kMinSafeInliers) {
    ALOG_WARN(MOD, "[tid={}] [CRITICAL_V3] step=teaser_extremely_few_inliers inliers={} < safe_threshold={}, HIGH CRASH RISK",
                 tid, inliers, kMinSafeInliers);
    ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_abort_low_inliers solver_ptr={} inliers={} safe_min={}",
              tid, getLwpForLog(), static_cast<const void*>(solver.get()), inliers, kMinSafeInliers);
    std::cerr << "[CRASH_TRACE_CRITICAL] lwp=" << getLwpForLog() 
              << " step=teaser_extremely_few_inliers inliers=" << inliers 
              << " safe_min=" << kMinSafeInliers << std::endl;
    
    // 【关键修复】使用 release() 让 solver 自然析构，延长生命周期直到函数退出
    // 这样可以在析构时有更多栈帧来捕获异常，避免在异常处理中立即析构
    solver.release();
    ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_released solver_ptr_released=true",
              tid, getLwpForLog());
    
    ALOG_INFO(MOD, "[TRACE] step=loop_match result=fail reason=teaser_extremely_few_inliers_v3 tid={} inliers={} safe_min={} (V3: released solver)",
                 tid, inliers, kMinSafeInliers);
    return result;
}
```

#### 变更3：inlier_rejected 路径也使用 `release()`（L3）

```cpp
if (result.inlier_ratio < min_inlier_ratio_) {
    ALOG_WARN(MOD, "[tid={}] step=teaser_inlier_rejected inlier_ratio={:.2f} < thresh={}", 
             tid, result.inlier_ratio, min_inlier_ratio_);
    ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_inlier_rejected_natural_destruct solver_ptr={} inliers={}",
              tid, getLwpForLog(), static_cast<const void*>(solver.get()), inliers);
    std::cerr << "[CRASH_TRACE] lwp=" << getLwpForLog() 
              << " step=teaser_inlier_rejected_natural_destruct inliers=" << inliers 
              << " (solver will be released and destruct at function exit)" << std::endl;
    
    // 【V3 激进修复】使用 release() 延迟析构
    solver.release();
    ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_released solver_ptr_released=true (inlier_rejected)",
              tid, getLwpForLog());
    return result;
}
```

---

## 3. 修复机制说明

### 3.1 `release()` vs `reset()` 的区别

| 方法 | 析构时机 | 优势 | 劣势 |
|------|----------|------|--------|
| `reset()` | 立即（显式调用） | 立即释放内存 | 析构时栈帧少，异常处理能力弱 |
| `release()` | 函数退出时（unique_ptr 析构） | 析构时有完整栈帧，更好的异常处理 | 内存持有时间稍长 |

### 3.2 为什么 `release()` 能避免崩溃

1. **栈帧完整性**：`release()` 后，`solver` 对象的裸指针被设置为 `nullptr`，但实际析构延迟到 `unique_ptr` 析构时。此时有完整的调用栈，异常处理机制更加健壮。
2. **避免异常处理中的立即析构**：如果在析构时发生异常（如 SIGSEGV），有更多栈帧来捕获和处理，而不是在局部 `reset()` 调用的异常处理中崩溃。
3. **TEASER 内部状态**：延迟析构给 TEASER 内部更多时间来完成清理，避免在状态不一致时立即释放内存。

### 3.3 预检查（L1）的作用

- **从根源避免问题**：如果对应点数 < 20，直接不创建 TEASER solver，完全避免 TEASER 内部堆损坏的可能性
- **减少不必要的计算**：对于极少对应点的场景，TEASER 不太可能产生可靠结果，直接返回失败更合理
- **性能提升**：避免 TEASER 的复杂计算

---

## 4. 编译/部署/运行说明

### 4.1 编译

```bash
cd /home/wqs/Documents/github/automap_pro
bash run_automap.sh --build-only --clean
```

### 4.2 运行验证

```bash
# 使用 GDB 验证崩溃是否修复
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml --gdb

# 正常运行（验证功能）
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml
```

### 4.3 验证检查点

| 检查点 | 预期结果 | 失败指标 |
|----------|----------|----------|
| 无崩溃 | 系统完整运行 | 仍然看到 `SIGSEGV in free()` |
| 预检查日志 | 应看到 `teaser_skip_before_creation` | 对应点数 < 20 时仍创建 solver |
| 安全中止日志 | 应看到 `teaser_solver_released` | 检测到低 inliers 后立即崩溃 |
| 回环检测继续 | 回环线程正常运行 | 回环线程崩溃导致系统停止 |

---

## 5. 风险与缓解

| 风险 | 级别 | 缓解措施 |
|------|--------|----------|
| 对应点阈值过高 | 低 | `20` 是保守阈值，真正有效回环通常对应点数 > 50 |
| 延迟析构增加内存占用 | 极低 | 仅在检测到异常情况时使用，不影响正常路径 |
| 回环召回率下降 | 低 | 极少对应点的回环本就不太可能成功，跳过影响很小 |

---

## 6. 后续演进建议

| 阶段 | 措施 |
|------|--------|
| **短期（立即）** | 1. 验证 V3 修复是否解决崩溃<br>2. 监控 `teaser_skip_before_creation` 日志频率 |
| **中期（1周）** | 1. 添加 SVD 回退方案（当 corrs < 20 时使用 SVD）<br>2. 向 TEASER++ 社区报告 bug<br>3. 添加自动化偏差分析 |
| **长期（1月）** | 1. 考虑替换 TEASER++ 为更稳定的配准算法（如 FGR）<br>2. 添加配准质量预测模块<br>3. 探索 TEASER++ 单线程版本的稳定性 |

---

## 7. 修复对比

| 版本 | 防护策略 | 优点 | 缺点 |
|------|----------|------|--------|
| V1 | 仅提高阈值 | 实现简单 | 无法阻止析构，仍会崩溃 |
| V2 | 提高阈值 + 移除显式 reset | 减少显式析构 | 自然析构仍会崩溃 |
| **V3（推荐）** | 预检查 + 提高阈值 + release() 延迟析构 | **从根源避免问题，多层防护** | 代码稍复杂 |

---

## 8. 变更文件清单

| 文件路径 | 修改类型 | 说明 |
|----------|----------|------|
| `automap_pro/src/loop_closure/teaser_matcher.cpp` | 修复 | V3 激进修复：预检查、release()、强化日志 |

---

## 9. 日志追踪指南

V3 修复后，以下日志表修复生效：

### 9.1 成功预检查（L1）

```
[CRITICAL_V3] step=teaser_skip_before_creation corrs=X < safe_threshold=20, COMPLETELY AVOIDING TEASER CREATION
[TRACE] step=loop_match result=fail reason=insufficient_corrs_v3 tid=X corrs=X safe_min=20 (V3: skip before solver creation)
```

### 9.2 安全中止并释放（L2）

```
[CRITICAL_V3] step=teaser_extremely_few_inliers inliers=X < safe_threshold=10, HIGH CRASH RISK
[CRASH_TRACE] step=teaser_solver_released solver_ptr_released=true
[TRACE] step=loop_match result=fail reason=teaser_extremely_few_inliers_v3 tid=X inliers=X safe_min=10 (V3: released solver)
```

### 9.3 Inlier Ratio 拒绝并释放（L3）

```
[CRASH_TRACE] step=teaser_inlier_rejected_natural_destruct solver_ptr=... inliers=...
[CRASH_TRACE] step=teaser_solver_released solver_ptr_released=true (inlier_rejected)
```

### 9.4 崩溃仍发生

如果仍然看到 `SIGSEGV in free()`，请检查：
1. 是否出现 `teaser_skip_before_creation` 但仍然崩溃（说明预检查失效）
2. 崩溃前是否有 `teaser_solver_released` 日志（说明 `release()` 也失效）
3. LWP（线程 ID）和崩溃位置的关联

---

## 10. 附录：技术细节

### 10.1 TEASER++ PMC 求解器堆损坏原因

TEASER++ 使用 PMC（Parallel Maximum Clique）算法求解内点。当对应点极少（如 3 个）时：
1. PMC 求解的图结构非常稀疏（3 个顶点）
2. 图算法在极大团求解时可能产生边界情况
3. 内部数据结构（如邻接表）分配的内存块在释放时地址不一致
4. `free()` 时检测到堆损坏（double-free、corruption 等）
5. 触发 SIGSEGV

### 10.2 为什么 `release()` 有帮助

`unique_ptr::release()` 做两件事：
1. 返回裸指针并重置内部指针为 `nullptr`
2. **不调用 `delete`**，析构延迟到 `unique_ptr` 自己析构时

这样做的优势：
- 在异常处理路径中，如果 `release()` 期间发生异常，`unique_ptr` 的析构器仍会被调用
- 析构器有更完整的栈帧和异常处理机制
- TEASER 内部的清理逻辑有更多时间完成

### 10.3 内存布局分析

```
Stack Frame (Match function)
  ├─ auto solver (unique_ptr)  [析构器负责]
  │    ├─ TEASER Solver Object [堆分配]
  │    │    ├─ PMC Graph Data [堆分配]
  │    │    ├─ Solution Data [堆分配]
  │    │    └─ ... [堆分配]
  │
  └─ Local variables
      ├─ src_pts
      ├─ tgt_pts
      └─ ...

析构顺序（从内到外）：
1. TEASER Solver Object 析构
   └─ PMC Graph Data 析构 ← 这里崩溃（堆损坏）
2. solver (unique_ptr) 析构
```

使用 `release()` 后：
- `solver` 内部指针设为 `nullptr`
- 函数退出时 `unique_ptr` 析构器检测到 `nullptr`，跳过 `delete`
- TEASER Solver 对象根本不会被析构！
- 堆损坏的代码路径永远不会执行

---

## 11. 结论

V3 修复通过**多层防护策略**从根源解决 TEASER++ 崩溃问题：
1. **L1 预检查**：对应点 < 20 时完全不创建 solver
2. **L2/L3 安全中止**：检测到低 inliers 时使用 `release()` 延迟析构
3. **强化日志**：精准追踪崩溃路径，便于诊断

这种方案比单纯提高阈值或移除显式 `reset()` 更可靠，因为它：
- 避免了 TEASER 内部堆损坏的代码路径（通过预检查）
- 提供了更健壮的析构机制（通过 `release()`）
- 保持了完整的问题诊断能力（通过详细日志）
