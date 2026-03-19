# 回环位姿一致性（pose_consistency）配置说明

## 1. 检查在做什么

回环检测用 TEASER 估计「当前帧相对候选帧」的相对位姿；同时用里程计/图上的位姿得到「当前帧相对候选帧」的** odom 相对位姿**。  
**pose_consistency** 比较这两者的差异：

- **平移差** `trans_diff_m = ||t_teaser - t_odom||`（米）
- **旋转差** `rot_diff_deg = angle(R_teaser * R_odom^T)`（度）

若 `trans_diff_m > pose_consistency_max_trans_diff_m` 或 `rot_diff_deg > pose_consistency_max_rot_diff_deg`，则视为异常，**拒绝该回环**。

---

## 2. 当前默认 5m / 25° 是否合理？

**取决于你对回环的假设：**

| 场景假设 | 是否合理 | 说明 |
|----------|----------|------|
| **回环 = 微调**：odom 已经较准，回环只是小修正 | ✅ 合理 | 同向再次经过同一点时，odom 与 TEASER 应接近，5m/25° 足够过滤明显错误。 |
| **同向 + 反向 + 两次位移较大**：希望只要进入同一环境就尽量检出回环 | ❌ 偏严 | 反向或大位移时，odom 相对位姿会很大或方向相反，与 TEASER「同一点」结果差很大，容易全部被拒。 |

因此：**若需求是「只要进入环境、无论同向还是反向、两次可能位移较大都要能检测到回环」**，仅用 5m/25° 会**不合理地拒绝**很多正确回环，需要放宽或关闭该检查。

---

## 3. 为何会拒绝「反向」和「位移大」？

- **同向、小位移**：odom 相对 ≈ 几米内，TEASER 相对 ≈ 0（同一点）→ 差在几米内 → 5m 可能通过。
- **反向**：odom 相对 = 从候选到当前沿轨迹的位移（例如 30m 向前），TEASER 相对 ≈ 0 → `trans_diff ≈ 30m > 5m` → 拒绝。
- **同向但两次经过位移大**：odom 累积了 20m，TEASER 仍认为同一点 → `trans_diff ≈ 20m > 5m` → 拒绝。
- **反向 + 旋转**：odom 相对旋转可能接近 180°，`rot_diff_deg ≈ 180° > 25°` → 拒绝。

代码注释也写明：*「反向运动时两向量相反，差向量模约 2*|t|，会超阈值」*，即**当前设计会主动拒绝「反向 + 大位移」类型的回环**。要支持这类回环，就必须放宽或关闭 pose_consistency。

---

## 4. 如何配置才能「同向+反向+位移大」都尽量检测到回环？

目标：**只要进入同一环境就尽量检测回环**，不因「反向」或「两次位移大」而被 pose_consistency 误杀。

### 方案 A：关闭 pose_consistency（最直接）

在 `system_config_*.yaml` 中：

```yaml
loop_closure:
  pose_consistency_max_trans_diff_m: 0    # 0=关闭平移检查
  pose_consistency_max_rot_diff_deg: 0   # 0=关闭旋转检查
```

- **优点**：同向、反向、大位移都能依赖 TEASER 的 inlier_ratio / rmse 通过，不再被 pose 差误拒。
- **注意**：误匹配若 TEASER 也给出「看起来合理」的结果，就无法用 pose_consistency 挡掉，需依赖 TEASER 质量与后续优化鲁棒性。

### 方案 B：大幅放宽，只挡明显错误

例如只拒绝「差得离谱」的匹配：

```yaml
loop_closure:
  pose_consistency_max_trans_diff_m: 20.0   # 例如 15~25，仅挡极大平移错误
  pose_consistency_max_rot_diff_deg: 90.0    # 反向约 180°，要接受反向可 90 或更大
```

- 反向、大位移时，只要 TEASER 与 odom 差在 20m / 90° 以内仍会接受；超过才拒绝。
- 若仍大量 reject_pose_anomaly，可再放宽或改为 0。

### 方案 C：保持 5m/25°（仅「微调」场景）

若你的场景**没有反向、且回环只是对已有轨迹的小修正**，保持默认即可：

```yaml
loop_closure:
  pose_consistency_max_trans_diff_m: 5.0
  pose_consistency_max_rot_diff_deg: 25.0
```

---

## 5. 如何判断当前配置是否合适？

1. **看日志**  
   - `grep "reject_pose_anomaly" full.log`：若大量拒绝且 trans_diff/rot_diff 只略超 5/25，说明阈值偏严。  
   - `grep "INTRA_LOOP.*DETECTED" full.log`：若子图内几乎没有 DETECTED 而多是 pose_anomaly，可尝试放宽或关闭。

2. **看场景**  
   - 有**反向重访、来回扫**或**两次经过间隔远、位移大** → 倾向方案 A 或 B。  
   - **仅同向、短闭环、odom 较准** → 方案 C 可保留。

3. **调参顺序建议**  
   - 先试 **0/0**，确认子图内/子图间回环数量与轨迹是否合理。  
   - 若误匹配增多，再改为**放宽数值**（如 15~20m, 90°）折中，而不是回到 5/25。

---

## 6. 小结

| 需求 | 建议配置 |
|------|----------|
| 回环只是微调，同向、小位移 | 保持 `5.0` / `25.0` |
| **同向+反向+位移大都要能检测回环** | `0` / `0` 关闭，或放宽到 `15~20` / `90` |

当前配置 5m/25° 在「微调」前提下是合理的；若要求「只要进入环境、无论同向反向、位移较大也要检测回环」，应**关闭或大幅放宽** pose_consistency，并主要依靠 TEASER 的 inlier_ratio、rmse 以及后端优化来保证质量。
