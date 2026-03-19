# 回环分析：为什么子图间只是「节点级」回环，没有真正的 keyframe 级回环

基于 `logs/run_20260318_184144/full.log` 与代码的深入分析结论。

---

## 1. 现象与配置

- 配置中 **`loop_closure.inter_keyframe_level: true`**，设计上期望子图间做 **关键帧 ↔ 关键帧** 的检索与匹配，约束入图时使用关键帧级节点。
- 日志中确实出现了 **子图间关键帧级** 的检测与入图尝试，例如：
  - `[CONSTRAINT] step=loop_inter_keyframe node_i=78 node_j=100000 (子图间关键帧级回环)`
  - `[LOOP_ACCEPTED] loop_constraint published sm_i=0 sm_j=1`
- 但随后大量出现 **addLoopFactor failed**、**commit_success=0**、**isolated node(s) detected**、**node not in graph** 等，说明这些「关键帧级」回环约束**几乎没有真正成功进入并生效于优化图**。

因此：**子图间在实现上表现为「只是节点级的尝试」，没有形成稳定的、可用的 keyframe 级回环约束**。下面说明根因。

---

## 2. 根因一：后端存在两套节点 ID 与两种 Symbol，子图间回环用错了空间

### 2.1 图中实际有两类节点

| 类型 | Symbol | 含义 | 如何加入图 | 存在集合 |
|------|--------|------|------------|----------|
| 子图节点 | `SM(id)` = `Symbol('s', id)` | 子图位姿，id = 0,1,2,... | `addSubMapNode(sm_id, ...)` | `node_exists_` |
| 关键帧节点 | `KF(id)` = `Symbol('x', id)` | 关键帧位姿，id = kf->id 全局递增 | `addKeyFrameNode(kf_id, ...)` | `keyframe_node_exists_` |

- 子图链：`SM(0) —Odom— SM(1) —Odom— SM(2)`，且有 Prior(SM(0))。
- 关键帧链：`KF(0) —Odom— KF(1) — … — KF(149)`，有 Prior(KF(0)) 和相邻 Odom。

### 2.2 子图间「关键帧级」回环当前是怎么入图的

在 `automap_system.cpp` 的 `onLoopDetected` / loop_opt_thread 中，当 `lc->keyframe_i >= 0 && lc->keyframe_j >= 0` 时：

- 计算的是 **编码 ID**：  
  `from_id = submap_i * MAX_KF_PER_SUBMAP + keyframe_i`  
  `to_id  = submap_j * MAX_KF_PER_SUBMAP + keyframe_j`  
  例如：78 = 0*100000+78，100000 = 1*100000+0。
- 然后用的是 **子图节点接口**：  
  `isam2_optimizer_.addSubMapNode(from_id, ...)`  
  `isam2_optimizer_.addSubMapNode(to_id, ...)`  
  `isam2_optimizer_.addLoopFactor(from_id, to_id, ...)`  
  即：**把 78、100000 当作 submap 节点 ID，加入的是 SM(78)、SM(100000)，回环是 Between(SM(78), SM(100000))**。

也就是说：**子图间回环虽然语义上是「关键帧↔关键帧」，但在后端被实现成了「再建两个 SM 节点 + 一条 Between」**，并没有使用图上已有的关键帧节点 `KF(kf_id)`。

### 2.3 后果：SM(78) / SM(100000) 与 KF 链完全脱节

- 图中已有的关键帧节点是 **KF(0), KF(1), …, KF(149)**，其 id 是 **全局 keyframe id**（`kf->id`），来自 `addKeyFrameNode(static_cast<int>(kf->id), ...)`。
- 子图间回环引入的是 **SM(78), SM(100000)** 等，它们：
  - 不在子图链上（没有 SM(78)—SM(0) 或 SM(100000)—SM(1) 的 Odom/Prior）；
  - 也不在关键帧链上（不是 KF(78)、KF(100000) 等，且 100000 根本不在 KF 的 id 空间里）。
- 因此 **SM(78) 与 SM(100000) 只在彼此之间有一条 Between**，形成一个与主图（SM 链 + KF 链）**不连通**的子图。

所以从「谁在图上、谁被约束」的角度看：**子图间回环只是多加了两个「孤立的子图型节点」和它们之间的一条边，并没有形成对已有 keyframe 图的约束**，这就是「只是节点级（且是错误节点空间）回环」的含义。

---

## 3. 根因二：不连通 + 无 Prior → commit 失败或孤立节点中止

### 3.1 日志中的典型失败

- `[IncrementalOptimizer][BACKEND][CONSTRAINT] ABORT commitAndUpdate: isolated node(s) detected`
- `[IncrementalOptimizer][BACKEND][VALIDATION] rollbackKeyframeStateForPendingKeys: removed 1 KF ids`
- `[BACKEND_STEP] step=addLoopFactor_done from=78 to=100000 commit_success=0 nodes_updated=0`
- 有时：`skip addLoopFactor from=39 to=100060 (node not in graph)`

### 3.2 为何会「isolated」或 commit 失败

- **SM(78)、SM(100000)** 只通过 **Between(SM(78), SM(100000))** 相连，与 SM(0),SM(1),SM(2) 和整条 KF 链都没有边。
- 这个 **{SM(78), SM(100000)}** 连通分量：
  - 没有 Prior，整体具有 6 自由度 gauge 自由度；
  - 在 iSAM2 中会导致系统欠定或数值问题，容易触发异常或我们的「孤立节点」保护逻辑。
- `addLoopFactor` 里只检查 **node_exists_**（子图节点集合）。  
  当使用编码 id（如 78、100000）时，我们通过 `addSubMapNode` 把 78、100000 放进 `node_exists_`，所以「node not in graph」通常发生在 **恢复/回滚** 之后：例如某次 commit 失败后清理了 pending，但 `node_exists_` 与 `keyframe_node_exists_` 的同步或后续逻辑导致下次用到的 id 不在当前图中，从而被拒绝。

综合：**子图间回环用 SM(78)/SM(100000) 这种「新子图节点」且不与主图相连，要么被判定为孤立节点而中止，要么在 commit 时因不连通/欠定而失败**，所以看起来「只是节点级尝试，没有 keyframe 级生效」。

---

## 4. 根因三：子图间回环的节点 ID 与图上 keyframe 节点 ID 不一致

- **图上关键帧节点**：id = **kf->id**（全局递增），例如 0,1,…,149。  
  对应关系：某个 keyframe 属于 submap_i，在 submap 内是第 keyframe_index 帧，但它在优化图里的 key 是 **KF(kf->id)**，不是 KF(submap_id*100000 + keyframe_index)。
- **子图间回环当前用的 ID**：  
  `node_id = submap_id * MAX_KF_PER_SUBMAP + keyframe_index`  
  例如 78、100000、100005。  
  这些 id 被当作 **submap 节点** 加入，得到的是 **SM(78), SM(100000)**，不是 **KF(78), KF(100000)**；且 100000 等远大于当前 kf->id 范围，根本不对应任何 KF 节点。

因此：**即使用户认为「子图间关键帧级回环」在逻辑上存在，后端也没有把这些约束加在「真正的 keyframe 节点」上，而是加在了一组与 KF 链无关的 SM 节点上，且这组节点与主图不连通**。

---

## 5. 小结：为什么是「子图间只是节点级回环，不是 keyframe 级」？

| 层面 | 说明 |
|------|------|
| **设计意图** | `inter_keyframe_level=true` 时，子图间应做 keyframe↔keyframe 匹配并加入 keyframe 级约束。 |
| **实际实现** | 子图间回环用 `submap_id*MAX_KF_PER_SUBMAP + keyframe_index` 作为 id，通过 **addSubMapNode** 加入，得到的是 **SM(78), SM(100000)** 等，即「新的子图型节点」，不是图上已有的 **KF(kf->id)**。 |
| **图结构** | 这些 SM 节点只与彼此有一条 Between，与 SM 链、KF 链都不连通，形成孤立分量，无 Prior → 易触发 isolated 检测或 iSAM2 失败。 |
| **结果** | 大量 addLoopFactor 报 commit_success=0、isolated、node not in graph，子图间回环约束几乎无法稳定入图并生效，因此表现为「只是节点级（且错误节点空间）的尝试，没有可用的 keyframe 级回环」。 |

---

## 6. 建议的修复方向（简要）

要让子图间回环真正成为 **keyframe 级** 并在现有图上生效，需要让回环约束加在 **已有的关键帧节点** 上，并保持与主图连通：

1. **统一使用关键帧全局 id**  
   - 回环约束应使用 **kf_i->id**、**kf_j->id**（即当前图中已有的 KF 节点 id），而不是 `submap_id*MAX_KF_PER_SUBMAP + keyframe_index`。
2. **用关键帧节点接口加回环**  
   - 不再对 78、100000 调用 `addSubMapNode`；  
   - 改为在 **KF 空间** 添加 Between 约束：  
     `addLoopFactorBetweenKeyframes(kf_i->id, kf_j->id, delta_T, information)`，内部使用 **KF(kf_i_id)**、**KF(kf_j_id)**，并只允许在 `keyframe_node_exists_` 已存在的 id 上添加。
3. **LoopConstraint 传递 keyframe 全局 id**  
   - 在 `LoopConstraint` 或等价结构中增加 `keyframe_global_id_i`、`keyframe_global_id_j`（或从 submap/keyframe_index 反查 kf->id），loop_detector 在发布回环时填这两个 id，后端只根据这两个 id 在 KF 图上加 Between。
4. **可选：子图级回环作为补充**  
   - 若希望保留「子图对」一级的约束，可以单独保留一条 **SM(submap_i) — Between — SM(submap_j)** 的路径，并确保 SM 链有 Prior；不要与「关键帧级」混用同一套编码 id 的 SM 节点。

按上述方向修改后，子图间回环才会真正落在 keyframe 图上，成为「子图间 keyframe 级回环」，而不再是「仅节点级且错误节点空间的失败尝试」。

---

*文档基于 full.log 与 automap_system.cpp / incremental_optimizer.cpp / loop_detector 相关逻辑整理。*
