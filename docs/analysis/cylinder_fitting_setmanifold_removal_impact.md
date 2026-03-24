# 根因分析：移除 Cylinder 拟合中 SetManifold 对建图精度的影响

**分析日期**: 2026-03-24  
**分析方法**: Root Cause Analyst + 5-Why 证据链  
**分析范围**: `semantic_processor.cpp` fitCylinder 中删除 `problem.SetManifold(ray, new ceres::SphereManifold<3>())` 的影响

---

## 1. 证据链与根因追溯

### 症状 (Symptom)
- 编译报错：`ceres/manifold.h: No such file or directory`（Ceres 1.x 无此头文件）
- `SetManifold` / `SphereManifold` 为 Ceres 2.0+ 专属 API

### 5-Why 追溯

| Level | 问题 | 证据 (文件:行) |
|-------|------|----------------|
| Why 1 | 为何会编译失败？ | 当前 Ceres 环境为 1.x，无 `ceres/manifold.h` 和 `SphereManifold` |
| Why 2 | 为何需要 SphereManifold？ | 约束 `ray`（圆柱轴向）为单位向量，防止优化过程中 `\|v\|` 漂移 |
| Why 3 | 为何 ray 需单位化？ | 成本函数 `d = \|(p-r)×v\|/\|v\|` 在数学上对 v 尺度不变，但数值上 \|v\|→0 会触发 `v_norm < 1e-6` 提前返回 |
| Why 4 | 移除后输出是否仍正确？ | `landmark->ray = Eigen::Vector3d(...).normalized()` (229 行) 与 GTSAM `Unit3` 均会归一化，下游仅使用方向 |
| Why 5 | 方向估计会受何影响？ | 成本对 v 尺度不变 ⇒ 梯度沿 v 方向分量为 0 ⇒ 理论上方向不变；实际受 LM 迭代、数值误差影响 |

### 根因归纳
- **[根因]** 成本函数对 `ray` 具有尺度不变性，优化器在无 manifold 约束时理论上仅优化方向。
- **[工程根因]** Ceres 1.x 无法使用 SphereManifold，当前修复采用“移除约束 + 后置归一化”作为最小化工作区。

---

## 2. 数学与实现一致性验证

### 2.1 成本函数尺度不变性

```
residual = |(p - r) × v| / |v| - radius
```

- 对任意 λ>0：`|(p-r)×(λv)|/|λv| = |(p-r)×v|/|v|` ⇒ 成本对 v 尺度不变
- 梯度：对零齐次函数 f(v)=g(v/|v|)，有 v·∇f=0 ⇒ 梯度不含径向分量，梯度下降不改变 |v|

**代码证据**: `semantic_processor.cpp:26-32`

```cpp
T v_norm = v.norm();
if (v_norm < T(1e-6)) return false;  // 数值保护
T dist_to_axis = cross.norm() / v_norm;
```

### 2.2 下游一致性

| 环节 | 文件:行 | 处理 |
|------|---------|------|
| 拟合输出 | semantic_processor.cpp:229 | `landmark->ray = ... .normalized()` |
| 因子构建 | incremental_optimizer.cpp:1463 | `gtsam::Unit3 ray_submap(...)` 内部归一化 |
| 因子误差 | isam2_factor_types.h:77 | `dist_to_axis = cross_prod.norm()`，假设 ray 为单位向量 |

**结论**: 下游均假设 ray 为单位向量，当前修改通过后置归一化保持契约。

---

## 3. 对建图精度的影响评估

### 3.1 理论影响（低）

- 在理想、无舍入误差情况下，尺度不变性保证优化方向不变，仅输出时归一化即可。
- 与使用 SphereManifold 相比，数学上等价于在同一流形（单位球面）上优化，差异来自优化路径与数值实现。

### 3.2 实际风险

| 风险 | 场景 | 影响 | 概率 |
|------|------|------|------|
| \|v\| 漂移至 < 1e-6 | 退化点云、坏初值 | `return false` → 该簇无 landmark | 低 |
| LM 更新导致 \|v\| 振荡 | 迭代中 v 幅度变化 | 梯度方向可能偏离，收敛变慢或到不同局部极小 | 低 |
| 初值 \|ray\| 异常 | PCL 线系数未归一化 | 初值尺度偏离 1，但成本尺度不变，通常可收敛 | 中 |

### 3.3 与 SphereManifold 的差异

| 维度 | 有 SphereManifold | 无 SphereManifold（当前） |
|------|-------------------|---------------------------|
| 参数空间 | S²（单位球面） | ℝ³ |
| 更新方式 | 流形上的 plus/minus | 欧式空间梯度更新 |
| 数值稳定性 | 始终在单位球上，更稳 | 依赖尺度不变性，极端情况可能退化 |
| 收敛性 | 流形优化，通常更平滑 | 在良好初值下通常无显著差异 |

---

## 4. 产品化与工程建议

### 4.1 当前修改的适用性

- **可接受场景**: 标准森林/城市树木场景、点云质量正常、RANSAC 初值可靠
- **需关注场景**: 稀疏/退化点云、极端遮挡、初值易错的边缘情况

### 4.2 推荐加固措施（按优先级）

1. **运行时监控（低成本）**
   - 在 `fitCylinder` 成功后检查 `Eigen::Vector3d(ray[0],ray[1],ray[2]).norm()`，若严重偏离 1（如 < 0.1 或 > 10）则打 WARN 日志
   - 统计 Ceres 非收敛比例，用于质量评估

2. **Ceres 1.x 等价实现（中成本）**
   - 使用 `LocalParameterization` 将 ray 约束到单位球面，替代 SphereManifold
   - 参考 `sloam/src/core/sloam.cpp` 中 `EigenQuaternionParameterization` 的用法，实现 `Unit3Parameterization`

3. **构建时 Ceres 版本检测（推荐）**
   - 在 CMake 中检测 Ceres 版本，Ceres ≥ 2.0 时启用 SphereManifold
   - 避免条件编译包含不存在的头文件，改为在 CMake 中传递 `HAS_CERES_MANIFOLD` 定义

### 4.3 回归与验证建议

- 在典型树木场景上对比：有/无 SphereManifold（Ceres 2 环境）的拟合结果
- 比较 `landmark->ray` 方向差异、半径与根点差异
- 检查后端优化中圆柱因子的残差与收敛情况

---

## 5. 结论

| 维度 | 结论 |
|------|------|
| **建图精度** | 在常规场景下，影响预计较小；尺度不变性与后置归一化保证输出语义正确 |
| **数值稳定性** | 略逊于使用 SphereManifold，极端退化情形下可能丢 landmark |
| **工程可行性** | 作为 Ceres 1.x 的权宜方案可接受，建议增加监控并在 Ceres 2 环境中恢复 SphereManifold |
| **产品化** | 建议纳入技术债务，在具备 Ceres 2 或自定义 Unit3 参数化后补齐流形约束 |
