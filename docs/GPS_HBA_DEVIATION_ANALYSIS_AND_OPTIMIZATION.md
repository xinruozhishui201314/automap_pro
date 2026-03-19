# GPS 与 HBA 优化后关键帧偏差分析及优化建议

## 一、数据与现象

### 1.1 accuracy_trajectories.csv 与 accuracy_summary.txt

- **keyframe_count**: 497  
- **mean_deviation_m**: 1.3353 m  
- **max_deviation_m**: 4.6969 m  
- **cum_dist_total_m**: 411.9 m  

CSV 中 `hba_*` 为 HBA 优化后的关键帧位置，`gps_*` 为通过 `enu_to_map` 转换到 map 系的 GPS 位置，`deviation_m` 为二者欧氏距离。

### 1.2 偏差分布特征（从 CSV 可见）

1. **起步段 (index 0~30)**  
   - deviation 约 0.2~0.6 m，相对较小。  
   - 对应日志中 `gps_aligned rmse_m=0.144`，对齐后前期轨迹与 GPS 较一致。

2. **中段 (index 76~92, cum_dist ~61~72 m)**  
   - **hba 轨迹出现明显“折返”**：hba_x/hba_y 先增后减（如 41.x → 40.x → 35.x），而 cum_dist 单调增加。  
   - 说明 **回环/BA 把轨迹拉回**，与“单调前进”的 GPS 产生 2~3 m 级偏差（deviation 最高约 3.3 m）。  
   - 同一时段 GPS 点存在重复（多帧共用同一 gps_x/y/z），说明 GPS 时间匹配或更新频率导致部分关键帧共用同一 GPS 观测。

3. **后半段 (index 429~496)**  
   - 存在多段“回头”的 hba 轨迹（cum_dist 继续增加但 hba 位置往回），与 GPS 偏差再次拉大到 1.5~2.2 m。  
   - **末尾 (index 494~496)**：gps_z_m 突变为约 -4.43，而 hba_z 约 0，deviation 约 4.5~4.7 m。  
   - 可能原因：  
     - 时间匹配到异常 GPS 点（如 `queryByTimestampForLog` 匹配到跳变或不同时段）；  
     - 高度通道噪声/多路径导致 GPS 高度异常。

4. **整体**  
   - 凡是 **HBA 轨迹被回环“拉回去”的区段**，与“单调前进”的 GPS 相比都会出现 1.5~3+ m 的偏差；  
   - 首帧被强 prior 固定，若存在系统误差，会带来整体平移，进一步放大与 GPS 的偏差。

---

## 二、原因分析

### 2.1 约束强度严重失衡：里程计/回环远强于 GPS

- **HBA GTSAM fallback**（`hba_optimizer.cpp`）：  
  - `prior_var = 1e-8`（首帧几乎固定）；  
  - `between_var = 0.01`（相邻关键帧相对位姿方差 0.01，即 std≈0.1 m，非常紧）。

- **GPS 方差**（`gps_manager.cpp`）：  
  - `sigma_h = hdop * 0.5`, `sigma_v = hdop * 1.0`；  
  - 典型 hdop≈10 → sigma_h=5, sigma_v=10 → 方差 25、100；  
  - 再除以 `factor_weight * quality_scale`（如 0.8×0.4=0.32）→ 水平方差约 78（std≈8.8 m）。

- **量级对比**：  
  - Between 因子方差 0.01 vs GPS 水平方差 ~78 → **里程计约束相对强度约为 GPS 的数千倍**。  
  - 优化器会优先满足 odom + loop，GPS 只能做“软牵引”，一旦回环或 odom 与 GPS 冲突，轨迹会跟 odom/回环走，从而产生 1~4 m 级偏差。

### 2.2 回环与 GPS 的冲突

- 日志显示存在回环检测与 HBA；  
- CSV 中 hba 轨迹在多处“折返”（cum_dist 增加但位置往回），与真实路径应单调前进的假设不符，说明 **回环约束把轨迹拉回**；  
- GPS 是单调、无回环的绝对位置，二者在“折返”区段必然产生大偏差。

### 2.3 首帧强 prior 的影响

- `prior_variance=1e-8` 使首帧几乎固定在初始位姿；  
- GPS 对齐是在“当前 odom 轨迹”上做的（align 时用 odom+GPS 拟合），若对齐时已有小偏差，或 HBA 中回环/odom 占主导，整条轨迹会相对 GPS 产生整体平移，难以通过后续 GPS 因子完全拉回。

### 2.4 GPS 使用方式

- GPS 通过 `enu_to_map` 转到 map 系（对齐后的 R/t）；  
- accuracy 中“真值”是同一套对齐后的 GPS；HBA 结果若被 odom/回环拉偏，与这套 GPS 的偏差就会变大。  
- 部分关键帧无绑定 GPS 时用 `queryByTimestampForLog` 插值/最近邻；时间或插值异常会引入异常点（如末尾 gps_z 突变）。

---

## 三、优化建议（按优先级）

### 3.1 提高 GPS 约束权重（推荐优先做）

**目标**：让优化器在冲突时更多考虑 GPS，减小 mean/max deviation。

- **配置**（`system_config_M2DGR.yaml`）：  
  - 将 `gps.factor_weight` 从 0.8 提高到 **1.2~1.5**（或更高，视轨迹平滑度再微调）；  
  - 适当提高 `gps.factor_quality_scale_medium`（如 0.4→0.6），让中等质量 GPS 也更有话语权。

- **代码层（可选）**：  
  - 在 `gps_manager.cpp` 中给 GPS 协方差再除以一个 **额外权重**（如 0.5），等价于把 GPS 方差减半、约束加倍；  
  - 或在 `incremental_optimizer.cpp` / HBA 里对 GPS 因子使用更小的方差（例如在现有 cov 上再乘 0.3~0.5）。

- **注意**：权重过大会导致轨迹在 GPS 噪声处抖动，建议配合鲁棒核或 outlier 检测（见下）。

### 3.2 放宽首帧 prior，避免过度锁定

- 将 `backend.isam2.prior_variance` 从 `1e-8` 改为 **1e-4~1e-3**（或 1e-5），让首帧可被 GPS/回环轻微调整，减少整体平移偏差。  
- HBA 的 `prior_var`（`hba_optimizer.cpp` 中 1e-8）也可同步改为 1e-4，与 backend 一致。

### 3.3 加强回环验证，减少错误回环对 GPS 的压制

- 已有 `loop_closure.teaser.*`、`pose_consistency_*` 等；可考虑：  
  - 对“回环候选”做 **与 GPS 的一致性检查**：若回环带来的位姿与 GPS 偏差超过阈值（如 3 m），则降权或拒绝该回环；  
  - 或提高 `loop_closure.teaser.min_inlier_ratio` / 收紧 `pose_consistency_max_trans_diff_m`，减少错误闭环。

### 3.4 对 GPS 因子使用鲁棒核（代码层）

- 在添加 GPS 因子的地方，用 GTSAM 的 **Huber 或 Cauchy 鲁棒核** 包裹 GPS 因子，使大残差（如错误匹配、高度跳变）不被过度放大，避免单点把整条轨迹拉偏。  
- 同时可保留或加强现有的 **GPS 异常值检测**（`incremental_optimizer.cpp` 中 `residual_baseline`、`outlier_scale`），对残差过大的 GPS 放大协方差或跳过。

### 3.5 时间对齐与 GPS 匹配

- 检查无绑定 GPS 的关键帧：`queryByTimestampForLog` 的 `kGpsMaxDt`（1.0 s）是否导致匹配到错误时刻；  
- 对 **高度通道** 单独处理：若仅用于评估可考虑只比平面偏差（xy），或对 gps_z 使用更大方差/鲁棒核，减轻末尾 z 突变对 deviation 的影响。

### 3.6 分阶段或分层优化（中长期）

- 先做 **odom + GPS** 的轻量优化（或仅用 GPS 做轨迹对齐），得到更贴近 GPS 的轨迹；  
- 再在此基础上加回环做 HBA，并适当降低 between 因子权重或提高 GPS 权重，使回环在“不严重违背 GPS”的前提下修正漂移。

---

## 四、建议的配置修改示例（可直接尝试）

在 `automap_pro/config/system_config_M2DGR.yaml` 中：

```yaml
# 3.2 GPS
gps:
  factor_weight:           1.2    # 从 0.8 提高，增强 GPS 约束
  factor_quality_scale_medium: 0.6   # 从 0.4 提高
  # 其余可暂不改

# backend
backend:
  isam2:
    prior_variance:         1e-4   # 从 1e-8 放宽，避免首帧过约束
```

并在 `automap_pro/src/backend/hba_optimizer.cpp` 中（GTSAM fallback 分支）：

- 将 `const double prior_var = 1e-8;` 改为 `const double prior_var = 1e-4;`（与 backend 一致）。

重新跑同一条 bag，对比 `accuracy_summary.txt` 的 mean_deviation_m、max_deviation_m 和轨迹是否仍出现不合理折返。若偏差明显下降，可再微调 `factor_weight` 与 `prior_variance` 以平衡平滑度与 GPS 贴合度。

---

## 五、小结

| 现象 | 主要原因 |
|------|----------|
| 平均/最大偏差 1.3 m / 4.7 m | GPS 约束相对 odom/回环过弱；回环拉回轨迹与 GPS 单调性冲突 |
| 中段 2~3 m 偏差 | 回环导致 hba 轨迹“折返”，与 GPS 不一致 |
| 末尾 4.5+ m（含 z 突变） | 时间匹配或高度异常点；prior 过强放大整体偏移 |

**优先实施**：提高 `factor_weight`、放宽 `prior_variance`，并在 HBA 中同步 prior；再视情况加鲁棒核与回环–GPS 一致性检查。
