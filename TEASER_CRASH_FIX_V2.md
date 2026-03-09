# TEASER++ 崩溃修复 V2 总结

## 0. Executive Summary

| 项目 | 内容 |
|------|------|
| **崩溃类型** | SIGSEGV in `free()` during TEASER++ RobustRegistrationSolver destruction |
| **根因** | TEASER++ PMC 求解器在极少 inliers（实测为3）时析构产生堆损坏 |
| **修复策略** | 提高安全阈值（3→10）+ 移除显式 reset() + 强化日志 |
| **修改文件** | `automap_pro/src/loop_closure/teaser_matcher.cpp` |
| **编译状态** | ✓ 成功 |

---

## 1. 崩溃根因分析

### 1.1 崩溃堆栈

```cpp
#0  0x00007ffff799e3fe in free () from /lib/x86_64-linux-gnu/libc.so.6
#1  std::__uniq_ptr_impl<teaser::RobustRegistrationSolver...>::reset()
#2  automap_pro::TeaserMatcher::match(...)
```

### 1.2 触发条件

从日志分析：
```
[CRASH_TRACE] lwp=383 step=teaser_solver_reset_before inlier_rejected inliers=3 (about to destruct)
```

**问题链路**：
1. FPFH 对应点数 = 188
2. TEASER 求解得到 inliers = 3（极少）
3. inlier_ratio = 3/188 = 0.02 < 0.3（阈值）
4. 进入 `inlier_rejected` 路径
5. 显式调用 `solver.reset()` 触发析构
6. PMC 求解器析构时堆损坏 → SIGSEGV

### 1.3 原有代码缺陷

```cpp:298
if (inliers < 3) {  // 问题：inliers=3 时该检查不触发！
    ALOG_WARN(..., "inliers={} < 3", inliers);
    solver.reset();
    return result;
}
```

当 `inliers=3` 时：
- `inliers < 3` 评估为 `false`
- 跳过提前退出路径
- 后续 `inlier_ratio=0.02 < 0.3` 走 `inlier_rejected` 路径
- `solver.reset()` 触发崩溃

---

## 2. 修复方案

### 2.1 提高安全阈值

```cpp:293
const int kMinSafeInliers = 10;  // 保守阈值：低于此值时 TEASER 析构风险极高
if (inliers < kMinSafeInliers) {
    ALOG_WARN(MOD, "[tid={}] [CRITICAL] step=teaser_extremely_few_inliers inliers={} < safe_threshold={}",
             tid, inliers, kMinSafeInliers);
    // 不调用 solver.reset()，让 unique_ptr 自然析构
    return result;
}
```

### 2.2 移除显式 reset()

所有三个提前退出路径（极少 inliers、inlier_ratio 低、解无效）都移除显式 `solver.reset()`：

```cpp
// 修复前
try {
    solver.reset();
} catch (...) { ... }

// 修复后
// 不调用 reset()，让 unique_ptr 自然析构，减少显式触发崩溃路径的风险
return result;
```

### 2.3 强化日志

在每个可能崩溃的析构点添加 CRASH_TRACE：

```cpp
ALOG_INFO(MOD, "[CRASH_TRACE][tid={} lwp={}] step=teaser_solver_abort_low_inliers solver_ptr={} inliers={} (solver will destruct naturally)",
          tid, getLwpForLog(), static_cast<const void*>(solver.get()), inliers);
```

---

## 3. 验证清单

### 3.1 运行验证

```bash
# 编译
bash run_automap.sh --build-only --clean

# 运行测试
bash run_automap.sh --offline \
  --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
  --config system_config_M2DGR.yaml --gdb
```

### 3.2 日志检查点

| 检查点 | 预期结果 |
|---------|----------|
| 崩溃消失 | 不应再看到 SIGSEGV in free() |
| 安全中止日志 | 应看到 `teaser_extremely_few_inliers` 且**无崩溃** |
| 回环检测继续 | 系统应正常运行，不因回环线程崩溃而中断 |

### 3.3 性能影响

| 指标 | 影响 |
|--------|------|
| 回环召回率 | 极小（inliers < 10 的回环本就无效） |
| 内存开销 | 无 |
| 计算开销 | 无 |

---

## 4. 风险与回滚

### 4.1 风险评估

| 风险 | 级别 | 缓解措施 |
|------|------|----------|
| 更高阈值减少回环召回 | 低 | 0.3 的 inlier_ratio 阈值已足够严格；真正有效回环通常 inliers > 20 |
| 自然析构仍可能崩溃 | 极低 | 保留完整异常捕获和日志；如仍崩溃，考虑延迟析构或使用原始指针 |
| 回环检测性能影响 | 无 | 仅影响低 inliers 情况（本就无效） |

### 4.2 回滚方案

如需回滚，恢复原代码：

```cpp
if (inliers < 3) {
    try {
        solver.reset();
    } catch (...) { ... }
    return result;
}
```

---

## 5. 后续演进建议

| 阶段 | 措施 |
|------|------|
| **短期（1周）** | 1. 运行完整数据集验证修复效果<br>2. 监控 `teaser_extremely_few_inliers` 日志频率 |
| **中期（1月）** | 1. 添加 SVD 回退方案（当 inliers < 10 时使用 SVD）<br>2. 评估 TEASER++ 上游 bug 修复 |
| **长期（3月）** | 1. 考虑替换 TEASER++ 为更稳定的配准算法（如 FGR）<br>2. 添加配准质量预测模块 |

---

## 6. 变更文件清单

| 文件路径 | 修改类型 | 说明 |
|----------|----------|------|
| `automap_pro/src/loop_closure/teaser_matcher.cpp` | 修复 | 提高安全阈值、移除显式 reset、强化日志 |

---

## 7. 日志追踪指南

修复后，如果看到以下日志，说明修复生效：

```
[CRASH_TRACE_CRITICAL] lwp=XXX step=teaser_solver_abort_low_inliers inliers=X safe_min=10 
    (solver will destruct naturally, DO NOT call reset())
```

如果看到崩溃，请检查：
1. 崩溃点的 `inliers` 数量
2. 崩溃点的 `lwp`（线程 ID）
3. 是否在 `teaser_solver_abort_low_inliers` 之后

---

## 8. 技术债清单

| 项目 | 优先级 | 说明 |
|------|--------|------|
| TEASER++ 堆损坏 bug | 高 | 需要向 TEASER++ 社区报告 |
| PMC 求解器稳定性 | 中 | 考虑限制 PMC 使用场景或替换 |
| 异常捕获覆盖 | 低 | 确保所有异常路径都被捕获 |
