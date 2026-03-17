# 后端「孤立节点」根本原因分析（2026-03-17）

## 0. Executive Summary

| 层级 | 根因 | 后果 |
|------|------|------|
| **表象** | 日志出现 `Removed stale node_exists_ entry: sm_id=1/2`，随后 `addOdomFactor_skip from_exists=0` | 子图链断裂，Odom 无法添加 |
| **直接原因** | 验证失败分支中「凡不在 current_estimate_ 就从 node_exists_ 删除」过于激进 | 本批新加、因 commit 失败而未进 estimate 的节点被删，下次 Odom 的 from 不存在 |
| **状态不一致** | commit 异常后只清 pending，**未**从 node_exists_ 移除「未入图的子图节点」 | node_exists_ 与 ISAM2 图不一致，下次 commit 时 values_for_validation 缺 key → 验证失败 → 触发上述删除 |
| **根本原因** | **V5 首次 update 只注入 values（空 graph），随后清空 pending，导致 Prior/Between 因子从未进入 ISAM2**；第二次 update 仅加入 55 个 keyframe GPS → 56 变量 + 55 个仅位置约束 → 奇异 → IndeterminantLinearSystemException，后续子图 commit 也因图无结构而继续失败 |

**结论**：要彻底解决需同时做两点：(1) **修正 V5 首次更新逻辑**，保证结构因子（Prior + Between）在首次或紧接着的第二次 update 中进入 ISAM2；(2) **收敛失败时的 node_exists_ 同步策略**（方案 A：只移除真正无因子引用的孤立节点），避免误删「本批未入图但仍有 pending 因子引用」的节点。

---

## 1. 因果链（从表象到根因）

### 1.1 表象：Removed stale / addOdomFactor skip

- 日志：`[VALIDATION] Removed stale node_exists_ entry: sm_id=2`、`sm_id=1`；之后 `addOdomFactor_skip from=2 to=3 reason=node_not_exists (from_exists=0)`。
- 含义：`node_exists_` 中曾有的 s1、s2 被删，后续以 s1/s2 为 `from` 的 Odom 无法添加。

### 1.2 直接原因：验证失败时的删除策略

- 代码位置：`incremental_optimizer.cpp` 验证失败分支（约 1816–1827 行）。
- 逻辑：若 `!validation_ok`，则遍历 `node_exists_`，**凡 `!current_estimate_.exists(SM(id))` 就加入 `nodes_to_remove` 并 erase**。
- 问题：未区分「真正孤立的节点」与「本批新加、因 commit 失败而未进 estimate 的节点」。后者仍可能被 pending 中的因子引用（如 Between(s1,s2)），删除后导致后续 Odom 的 from 不存在，子图链断裂。

### 1.3 为何会走到验证失败？

- 典型序列：
  1. 子图 1 冻结 → addSubMapNode(1)、addOdomFactor(0,1) → pending 含 s1 + Between(s0,s1)。
  2. forceUpdate → commit 时 **isam2_.update() 抛异常**（如 IndeterminantLinearSystemException）。
  3. 异常路径：清空 `pending_graph_`/`pending_values_`，**不**修改 `node_exists_` → s1 仍留在 `node_exists_`，但 **ISAM2 图中没有 s1**（commit 未成功）。
  4. 子图 2 冻结 → addSubMapNode(2)、addOdomFactor(1,2) → pending 含 s2 + Between(s1,s2)。
  5. 构建 `values_for_validation = current_estimate_ + pending_values_`：current_estimate_ 只有 s0 + 55 KF（无 s1），pending_values_ 只有 s2 → **没有 s1**。
  6. `graph_copy` 含 Between(s1,s2)，需要 key s1 和 s2；values_for_validation 缺 s1 → **all_keys_exist = false** → 验证失败。
  7. 进入验证失败分支，删除「不在 current_estimate_ 的节点」→ 删除 s1、s2 → 子图链断。

因此：**验证失败是由「node_exists_ 与 ISAM2 图已不一致」触发的**；不一致来自上一步 commit 异常后只清 pending、未同步 node_exists_。

### 1.4 为何 commit 会抛 IndeterminantLinearSystemException？

日志（run_20260317_122530）：

```text
pending_values=0 pending_factors=55 current_estimate_=56
exception='Indeterminant linear system detected while working near variable'
```

- 场景：**第二次 forceUpdate**（子图 0 冻结后，首次 commit 成功，flush 55 个 keyframe GPS 后第二次 forceUpdate）。
- 此时：
  - **第一次 commit** 走 V5 首次路径：`isam2_.update(empty_graph, values_for_update)` → 只注入 56 个 values，**未加入任何因子**（Prior/Between 全在 pending 里）。
  - 随后 **pending 被清空**（见 1886–1890 行），Prior/Between 被丢弃，**从未进入 ISAM2**。
  - 第二次 forceUpdate 时，pending 仅有 flush 进来的 **55 个 keyframe GPS 因子**，pending_values_ 为空。
- 因此：ISAM2 内是 **56 个变量 + 0 个因子**；本次 update 加入 **55 个仅位置约束的 GPS 因子** → 56 变量、55 条约束且无 Prior/Between 拓扑 → **线性化后系统奇异** → IndeterminantLinearSystemException。

再往后（子图 1、2… 冻结）：每次 commit 仍是在「图内几乎无结构因子」的基础上加一个 Between(s_i, s_{i+1}) 和少量 value，系统继续奇异或病态，容易再次抛异常，形成恶性循环。

---

## 2. 根因归纳

| 层级 | 根因 | 代码/逻辑位置 |
|------|------|----------------|
| **设计/实现错误** | V5 首次 update 为避免 NoiseModelFactor::error() 只做 `update(empty_graph, values)`，随后清空 pending，**结构因子（Prior、Between）从未加入 ISAM2** | 1721–1790、1886–1890 行；注释「因子在本次一并添加到 ISAM2」与实现不符 |
| **触发** | 第二次 update 仅 55 个 GPS 因子 + 56 变量 → 奇异 → 异常 | 第二次 forceUpdate → commitAndUpdate 增量路径 |
| **状态不一致** | 异常路径只清 pending，**不**从 node_exists_ 移除「未入图的子图节点」 | 1921–1948 行；node_exists_ 与 ISAM2 图分叉 |
| **连锁** | 下次 commit 的 graph 引用 s1，但 values_for_validation 无 s1 → 验证失败 → 激进删除 node_exists_ → 断链 | 1816–1827 行 |

---

## 3. 数据流与 V5 首次路径（简要）

```mermaid
sequenceDiagram
  participant F as forceUpdate
  participant C as commitAndUpdate
  participant I as ISAM2

  Note over F,I: 子图0冻结，首次 commit
  F->>C: pending = Prior(s0)+Between(x23..x77)+56 values
  C->>C: is_first_update=true
  C->>I: update(empty_graph, 56 values)
  Note over I: 56 variables, 0 factors
  C->>C: current_estimate_ = values; pending.clear()
  Note over C: Prior/Between 被清掉，未进 ISAM2

  Note over F,I: flush 55 KF GPS，第二次 forceUpdate
  F->>C: pending = 55 GPS factors, 0 new values
  C->>C: graph_copy=55 factors, values_for_update=empty
  C->>I: update(55 factors, empty values)
  I-->>C: IndeterminantLinearSystemException
  C->>C: clear pending, node_exists_ 不变
  Note over C: node_exists_={0}, 图仍 56 vars + 0 factors

  Note over F,I: 子图1冻结
  F->>C: addSubMapNode(1), addOdom(0,1), pending=s1+Between(s0,s1)
  C->>I: update(...)
  I-->>C: 再次异常（图无结构）
  C->>C: clear pending, node_exists_ 仍含 1 → 不一致

  Note over F,I: 子图2冻结
  F->>C: addSubMapNode(2), addOdom(1,2), pending=s2+Between(s1,s2)
  C->>C: values_for_validation = estimate + s2（无 s1）
  C->>C: 校验: graph 含 key s1, values 无 s1 → 验证失败
  C->>C: 删除不在 estimate 的 node_exists_ → 删 s1,s2
  Note over C: 子图链断，后续 addOdom(2,3) from_exists=0
```

---

## 4. 修复方向（与优先级）

### 4.1 根本修复（必须）：V5 首次更新让结构因子入图

- **目标**：首次 update 后，ISAM2 中除 56 个 variables 外，还应有 Prior(s0) 及 Keyframe 间 Between 等结构因子。
- **约束**：需继续规避 NoiseModelFactor::error() 的首次调用问题（如 borglab/gtsam#1189）。
- **可选做法**：
  - **A**：首次 update 仍只做 `update(empty_graph, values)`；**在同一 commitAndUpdate 内**紧接着再做一次 `update(graph_copy, Values())`，只加因子、不加新 value，再清 pending。若第二次 update 仍触发 error()，需在 GTSAM 侧或因子使用方式上再规避。
  - **B**：首次不清 pending，下一次 forceUpdate 时 pending 仍为「Prior+Between」（即不在首次后立刻 flush KF GPS），让第二次 commit 把「结构因子 + 56 values 已存在」一起加入。需保证 onSubmapFrozen 的调用顺序与 flush 时机不破坏该顺序。
  - **C**：换用不触发该 bug 的因子或线性化路径，使首次即可安全注入「values + 结构因子」。

建议先实现 A 并在同场景回放验证；若仍崩溃再考虑 B/C 或 GTSAM 升级/补丁。

### 4.2 收敛策略（必须）：方案 A——只删真正孤立节点

- **目标**：验证失败或异常后，**仅移除「图中无任何因子引用」的 node_exists_ 条目**，不删「本批新加、仅因 commit 失败未进 estimate、但仍被 pending 因子引用」的节点。
- **实现要点**（见 BACKEND_CONSTRAINT_ANALYSIS_20260317.md 方案 A）：
  - 在决定 `nodes_to_remove` 时，先收集 **pending_graph_ 中所有出现的 key**（以及若可低成本拿到，ISAM2 已有因子中的 key）。
  - 仅当 `id` 满足：`!current_estimate_.exists(SM(id))` **且** SM(id) 不在「任一因子引用的 key 集合」中时，才将 `id` 加入 `nodes_to_remove`。
  - 这样，因 commit 失败而未进 estimate 的 s1、s2 若仍被 Between(s1,s2) 等引用，就不会被删，避免断链。

### 4.3 可选：异常路径同步 node_exists_

- 在 commit 抛异常且判定为 unrecoverable 时，除清 pending 与 rollback keyframe 外，**从 node_exists_ 中移除本批 pending_values_ 里出现的子图节点**（仅 submap 符号 ‘s’），使 node_exists_ 与「实际未入图的节点」一致，减少后续「缺 key 导致验证失败」的概率。  
- 此为收敛策略，不能替代 4.1（不修复根因的话，后续 commit 仍会持续失败）。

---

## 5. 验证清单

- [ ] **V5 结构因子**：同 bag 下，首次 commit 后通过日志或调试确认 ISAM2 中至少含 Prior(s0) 及若干 Between，再执行 flush 55 KF GPS 的第二次 update 不再抛 IndeterminantLinearSystemException。
- [ ] **方案 A**：人为构造一次验证失败（或复用现有失败 run），确认不再出现 `Removed stale node_exists_ entry: sm_id=1/2`，且后续 `addOdomFactor(2,3)` 不为 `from_exists=0`。
- [ ] **端到端**：同配置、同 bag 回放，子图 2→3、4→5 的 Odom 均为 `addOdomFactor_added`，无 `reason=node_not_exists`，且无 Removed stale 对「本批未入图子图节点」的误删。

---

## 附录：根本修复实现与验证（2026-03-17）

### 已实现修改（incremental_optimizer.cpp）

- **V5 首次路径**：在 Step1 `update(empty_graph, values_for_update)` 之后增加 **Step2**：
  - 若 `graph_copy.size() > 0`，执行 `isam2_.update(graph_copy, no_new_values)`，将 Prior + Between 结构因子加入 ISAM2；
  - 随后 `current_estimate_ = isam2_.calculateEstimate()`，再清空 pending。
- 这样首次 commit 后 ISAM2 内为「56 变量 + 结构因子」，后续 flush 的 55 个 KF GPS 在已有拓扑上加入，避免奇异。

### 验证步骤（请在本地执行）

1. **编译**（容器内或本机 colcon）  
   ```bash
   # 若用 run_automap.sh：带 --clean 会触发重新编译
   bash run_automap.sh --offline --bag-file "$(pwd)/data/automap_input/M2DGR/street_03_ros2" \
     --config system_config_M2DGR.yaml --clean
   ```

2. **跑通同场景**  
   使用与之前相同的 bag 与 config，跑至至少 2～3 个子图冻结（或跑完）。

3. **日志检查**（在新生成的 `logs/run_YYYYMMDD_HHMMSS/full.log` 下）  
   - **应出现**（V5 两阶段成功）：  
     `first_update_v5_post_inject_factors`、`first update: injected %zu structural factors`  
   - **不应出现**：  
     `Indeterminant linear system detected`、`commitAndUpdate done success=0 exception=`（在第二次 forceUpdate 处）  
   - **不应出现**（断链修复后）：  
     `Removed stale node_exists_ entry: sm_id=1` / `sm_id=2`  
   - **应出现**（子图链正常）：  
     `addOdomFactor_added from=1 to=2`、`addOdomFactor_added from=2 to=3` 等，无 `addOdomFactor_skip ... from_exists=0`

4. **快速 grep 命令**  
   ```bash
   LOG="logs/run_YYYYMMDD_HHMMSS/full.log"  # 替换为本次 run 的 log
   grep -E "first_update_v5_post_inject_factors|structural factors" "$LOG" | head -5
   grep -c "Indeterminant linear system" "$LOG" || true   # 应为 0
   grep -c "Removed stale node_exists_" "$LOG" || true  # 应为 0
   grep "addOdomFactor_skip.*from_exists=0" "$LOG" || true  # 应无输出
   ```

若上述均通过，可认为根本修复已生效。

---

*基于 `logs/run_20260317_122530/full.log` 与 `incremental_optimizer.cpp` 分析。*
