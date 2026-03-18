# 回环为何没有 + 树木为主场景的优化手段

**目标**：说清当前 run 无回环被接受的原因，并针对「大部分是树、结构化建筑很少」的场景给出可落地的优化手段，提高回环数量。

---

## 树多场景如何检测到足够多的回环（速查）

按下面顺序做，一般能在树木为主场景下明显增加回环数量。

### 第一步：启用树木场景回环参数（必做）

**方式 A：用现成覆盖配置**

- 若 launch 支持多 config：在 launch 里增加一份 `loop_closure_vegetation_override.yaml` 的加载（与主配置合并或后加载覆盖）。
- 若只有单 config 文件：把 `automap_pro/config/loop_closure_vegetation_override.yaml` 里的 **`loop_closure` 整段**复制进你当前使用的 `system_config_*.yaml`，覆盖其中的 `loop_closure` 段。

**方式 B：只改关键几项**

在主配置的 `loop_closure` 下至少改这几项：

```yaml
loop_closure:
  teaser:
    fpfh_corr_max_distance_m: 5.0   # 关键：只保留距离<5m 的 FPFH 对应点送 TEASER
    min_inlier_ratio: 0.04          # 树木场景放宽（配合上面过滤）
    voxel_size:       0.5           # 略大体素，减树叶噪声
    max_rmse_m:       0.60
  scancontext:
    dist_threshold:   0.20          # 略收紧，提高候选质量
    num_candidates:   10
```

其中 **`fpfh_corr_max_distance_m: 5.0`** 对树木场景影响最大（过滤掉大量误对应，让 TEASER 内点率上来）。

### 第二步：跑一遍并看日志

- 跑完后：`grep -E "LOOP_ACCEPTED|FPFH_GEO_FILTER|TEASER_done|TEASER_SUMMARY" full.log`
- 期望看到：
  - `[FPFH_GEO_FILTER] filtered_corrs=...` 且过滤后数量明显小于原始 corrs（说明 5m 过滤在生效）；
  - `[LOOP_ACCEPTED]` 或 `[TEASER_SUMMARY] accepted=...` 中 accepted≥1。

若仍无 `LOOP_ACCEPTED`：看 `TEASER_done` 的 `inlier_ratio` 是否比之前提高；若仍很低，可把 `fpfh_corr_max_distance_m` 再降到 **3.0** 试一次。

### 第三步：按需微调

| 现象 | 可调项 |
|------|--------|
| 仍无回环、inlier_ratio 仍很低 | `fpfh_corr_max_distance_m`: 5 → **3**；或 `min_inlier_ratio`: 0.04 → **0.03** |
| 回环有了但轨迹错位/撕裂（误回环） | 收紧：`min_inlier_ratio` 提到 0.05～0.06，或 `fpfh_corr_max_distance_m` 降到 3 |
| 候选太少（retrieve 很少） | 略放宽 ScanContext：`dist_threshold` 0.20→0.22，`num_candidates` 10→12 |

### 第四步：长期可选

- 启用 **Overlap Transformer**（若环境有 LibTorch）：学习描述子在重复结构场景往往比 ScanContext 更稳。
- 将 **FPFH 半径**做成可配置并适当增大（如 normal_r=0.6、fpfh_r=1.2），提高树木点云上的区分度（需改代码）。
- 评估 **子图级匹配**（用子图 merged_cloud 做 FPFH+TEASER），点更多、几何更稳（需新增逻辑）。

---

## 0. Executive Summary

| 项目 | 结论 |
|------|------|
| **为何没有回环** | 流水线是 ScanContext 检索 → FPFH 对应点 → TEASER 几何验证。当前日志中 **TEASER 全部拒绝**：inliers=4、inlier_ratio≈0.004，低于 min_inlier_ratio=0.08；根因是 **FPFH 对应点质量极差**（p90=25～57m），绝大多数是误匹配，TEASER 只能筛出极少内点。 |
| **树木场景为何更难** | 树木为主时：① **ScanContext** 环状距离分布相似处多，易检索到「看起来像」但非同一地点的候选；② **FPFH** 依赖局部几何（法向+曲率），树木重复结构多、缺乏平面/棱角，易产生大量「相似但错位」的对应点；③ 二者叠加 → 误候选 + 误对应 → TEASER 内点率极低。 |
| **优化方向** | 分三层：**参数与策略**（放宽 TEASER 条件 + 更严 FPFH 几何过滤、体素/半径）、**检索与描述子**（ScanContext 收紧/子图级、FPFH 半径可配置、有条件时用 Overlap Transformer）、**场景专用**（树木场景专用配置 + 可选子图级匹配）。 |

---

## 1. 回环流水线（当前实现）

```
子图冻结/关键帧就绪
    → ScanContext 描述子检索（overlap / 距离阈值）
    → 得到候选 (query_submap, query_kf, target_submap, target_kf)
    → 几何预筛（可选 geo_prefilter_max_distance_m）
    → 对每个候选：取两帧点云 → 体素下采样(0.4) → FPFH(normal_r=0.5, fpfh_r=1.0) → 最近邻对应
    → FPFH 几何过滤（p90>10m 时按距离过滤，当前阈值偏松）
    → TEASER++ 鲁棒求解 → 检查 inlier_ratio >= 0.08 && inliers >= 3 && rmse <= 0.55
    → 可选 ICP 精化 → 通过则 LOOP_ACCEPTED
```

回环被接受的条件（`loop_detector.cpp` / `teaser_matcher.cpp`）：

- `res.success && res.inlier_ratio >= min_inlier_ratio_ && res.rmse <= max_rmse_`（INTER_KF 路径）；
- 并行 TEASER 路径同理，再叠加 ICP 后 RMSE 检查。

当前配置（M2DGR）：`min_inlier_ratio=0.08`，`min_safe_inliers=3`，`max_rmse_m=0.55`。

---

## 2. 为何没有回环？——日志证据与根因

### 2.1 现象

- 全日志 **无** `LOOP_ACCEPTED`。
- 大量：`[LOOP_COMPUTE][TEASER] teaser_done inliers=4 corrs=7xx ratio=0.004 valid=0`，拒绝原因 `inlier_ratio_low`（ratio 远低于 0.08）。

### 2.2 根因链

| 环节 | 日志/现象 | 含义 |
|------|-----------|------|
| **ScanContext 检索** | 有 `retrieve_result` 且进入 TEASER，说明有候选。 | 检索不是瓶颈；但树木场景下候选可能多为「相似环状分布」的误检索。 |
| **FPFH 对应点** | `corrs=780～964`，`[FPFH_DIAG] dist_p10=9.52m p50=25.07m p90=57.46m`，`p90>>5m 表示存在大量误匹配`。 | 对应点数量够，但**空间距离极大**，绝大多数是错误匹配（不同位置被 FPFH 判成相似）。 |
| **FPFH 几何过滤** | `[FPFH_GEO_FILTER] p90=57.46m > threshold=10.00m, applying geometric filter`，`filtered_corrs=918/944 (97.2%) median_dist=25.07m threshold=75.21m`。 | 当前逻辑：p90>10m 时用 `max(median*2+median, 10m)` 做阈值；median≈25m 时阈值≈75m，**几乎保留全部错误对应**，过滤效果很弱。 |
| **TEASER** | 输入 ~900 对对应点，几乎全为误匹配 → 鲁棒求解后 **inliers=4**，ratio=4/900≈0.004。 | 远低于 min_inlier_ratio=0.08 → **全部被拒**。 |

**结论**：没有回环的直接原因是 **TEASER 内点率不达标**；内点率低的根本原因是 **FPFH 在树木/弱结构场景下产生大量误对应，且当前 FPFH 后几何过滤过松**，误对应被原样送入 TEASER。

---

## 3. 树木为主、少结构化建筑为何更难？

| 因素 | 说明 |
|------|------|
| **ScanContext** | 基于距离环的 2D 直方图，树木场景中「环状距离分布」在不同地点容易相似 → 检索到的 top‑k 中**假阳性**多，真正闭环帧可能被淹没或根本没进候选。 |
| **FPFH** | 依赖局部法向与曲率；树木、灌木多为**重复的枝干/叶簇**，缺乏稳定平面与棱角，不同位置容易产生相似描述子 → **对应点数量多但空间错位大**（p50/p90 很大）。 |
| **真闭环几何** | 真回环时两帧相对位姿应在一个较小范围（几米内）；而当前 FPFH 对应点 p50≈25m、p90≈57m，说明匹配到的多是「语义/外观相似、空间不对」的点对。 |

因此：**树木多、建筑少**会同时恶化「候选质量」和「对应点质量」，需要从检索、描述子、几何过滤、TEASER 阈值多端一起调，而不是单点放宽 TEASER。

---

## 4. 优化手段（提高回环数量）

### 4.1 参数与策略（优先做）

- **收紧 FPFH 后几何过滤（关键）**  
  - 现状：`teaser_matcher.cpp` 里 `kGeoFilterThreshold = 10.0`，且用 `median + 2*median` 得到约 75m 的阈值，误对应几乎全保留。  
  - **建议**：  
    - 将「FPFH 对应点距离阈值」做成可配置（如 `loop_closure.teaser.fpfh_corr_max_distance_m`），**树木场景先用 3～5m**（真闭环相对位移通常不大），只保留距离 < 该值的对应点再送 TEASER。  
    - 若过滤后对应点 < 20，可保持「本候选跳过 TEASER」的既有逻辑，避免崩溃；若 20～50 之间可观察 inlier_ratio 是否明显提升。  
  - **效果**：减少误对应输入，TEASER 内点率有望明显提高，更容易达到 0.08 或适度放宽后的阈值。

- **适度放宽 TEASER 接受条件（需配合 RMSE）**  
  - 在**已加强 FPFH 几何过滤**的前提下，可尝试：  
    - `min_inlier_ratio`: 0.08 → **0.03～0.05**（树木场景可先试 0.04）；  
    - `min_safe_inliers`: 保持 3 或略提高到 4；  
    - `max_rmse_m`: 保持 0.55 或略放宽到 **0.6～0.7**，作为安全网，避免误回环残差过大。  
  - **注意**：仅建议在「几何过滤已收紧」或「同时启用 ICP 精化 + RMSE 严格」时放宽 ratio，否则易引入误回环。

- **增大回环用体素、减少噪声**  
  - 当前 `loop_closure.teaser.voxel_size: 0.4`。树木点云噪声大，可尝试 **0.5～0.6**，减少树叶/细枝带来的抖动，让 FPFH 更稳定（会略减少点数，通常仍够用）。

- **检索阶段减少误候选（ScanContext）**  
  - 树木场景可**适当收紧** ScanContext，让进入 TEASER 的候选更「像真闭环」：  
    - `dist_threshold`: 0.22 → **0.18～0.20**（减少明显不像的候选）；  
    - `num_candidates`: 15 → **10**（减少无效 TEASER 调用）。  
  - 若发现「真闭环根本检索不到」，再考虑略放宽；优先保证「检索到的里真闭环比例」提高。

### 4.2 描述子与 FPFH（中期）

- **FPFH 半径可配置**  
  - 当前 `normal_radius=0.5`、`fpfh_radius=1.0` 写死在 `teaser_matcher.cpp`。  
  - **建议**：从配置读取（如 `loop_closure.teaser.fpfh_normal_radius` / `fpfh_radius`）。树木场景可试 **normal_radius=0.6～0.8、fpfh_radius=1.2～1.5**，用更大邻域获得更稳定、略更区分性的描述子（计算量会略增）。

- **Overlap Transformer（若可用）**  
  - 配置中已有 `overlap_transformer.model_path`；若当前用 ScanContext 是因为 LibTorch 未用上，**启用 Overlap Transformer** 可换用学习描述子，在重复结构场景往往比 ScanContext 更稳，有利于减少误候选、提高真闭环排名。

### 4.3 场景与策略（树木专用）

- **子图级匹配（可选）**  
  - 当前 INTER_KEYFRAME 是关键帧对关键帧；树木场景单帧区分度差。可评估**子图对子图**：用子图 `merged_cloud`/`downsampled_cloud` 做 FPFH+TEASER，点更多、更平滑，几何更稳，但实现与触发逻辑需单独设计。

- **几何预筛（检索后、TEASER 前）**  
  - `geo_prefilter_max_distance_m` 当前为 0（关闭）。若轨迹有粗略位姿（如 GPS/odom），可设为 **30～80m**：只对「锚点距离 < 该值」的候选做 TEASER，避免明显不可能是同一地点的子图对浪费算力并拉低统计。

- **专用配置片段（树木场景）**  
  - 下面给出一份「树木为主」的 YAML 片段，集中上述参数，便于 A/B 对比。

---

## 5. 树木场景推荐配置片段（YAML）

可单独建 `system_config_vegetation.yaml` 或在现有配置中增加对应 override / 环境/launch 选择：

```yaml
# 树木/植被为主、少结构化建筑 — 回环优化
loop_closure:
  overlap_threshold:   0.18      # 略收紧，减少误候选
  top_k:               10
  min_temporal_gap_s:  15.0
  min_submap_gap:      0

  intra_submap_enabled: true
  intra_submap_overlap_threshold: 0.18
  inter_keyframe_sample_step: 5
  inter_keyframe_top_k_per_submap: 5

  geo_prefilter_max_distance_m: 50.0   # 可选：只对锚点距离<50m 的候选做 TEASER

  teaser:
    noise_bound:      0.15      # 略放宽，适应树木点云噪声
    voxel_size:       0.5       # 略大，减树叶噪声
    max_points:       8000
    min_inlier_ratio: 0.04      # 树木场景先放宽（配合下面 FPFH 几何过滤收紧）
    min_safe_inliers: 3
    max_rmse_m:       0.60      # 略放宽，仍可挡误回环
    min_relative_translation_m: 0.02
    icp_refine:       true

  scancontext:
    enabled:           true
    dist_threshold:    0.20     # 略收紧，提高候选质量
    num_candidates:   10
    exclude_recent:   3
    tree_making_period: 30
    min_history_for_search: 3
```

**已实现**：

- `loop_closure.teaser.fpfh_corr_max_distance_m` 已可配置（ConfigManager + teaser_matcher）；默认 10.0，树木场景在 `loop_closure_vegetation_override.yaml` 中设为 **5.0**。仅保留距离小于该值的 FPFH 对应点再送 TEASER；若过滤后不足 10 对则回退到宽松过滤逻辑。

---

## 6. 验证与迭代

| 步骤 | 做法 |
|------|------|
| 1）确认瓶颈 | `grep -E "LOOP_STEP|TEASER_done|FPFH_DIAG|FPFH_GEO_FILTER|LOOP_ACCEPTED" full.log`，看是「无候选」「对应点少」还是「TEASER 全拒」。 |
| 2）改后对比 | 改 FPFH 几何阈值 + 上述 YAML 后重跑同 bag，再 `grep LOOP_ACCEPTED full.log`，看是否出现回环。 |
| 3）防误回环 | 观察 HBA 后轨迹/全局图是否出现错位、撕裂；若出现，优先收紧 `min_inlier_ratio` 或 `max_rmse_m`，或再收紧 FPFH 几何过滤。 |

---

## 7. 小结

- **没有回环的原因**：TEASER 内点率极低（inliers=4、ratio≈0.004），因 FPFH 在树木/弱结构场景产生大量误对应，且当前 FPFH 后几何过滤过松（约 75m 阈值），误对应几乎全进 TEASER。  
- **树木场景**：ScanContext 易检索到相似环状分布的误候选，FPFH 在重复植被结构上误匹配多，需要「收紧检索 + 收紧 FPFH 几何过滤 + 适度放宽 TEASER 比例 + 体素/半径」组合优化。  
- **优先落地**：① 把 FPFH 对应点「最大距离」改为可配置并设为 3～5m（树木）；② 使用上述树木专用 YAML；③ 视情况放宽 `min_inlier_ratio` 到 0.04 并保持 `max_rmse_m`/ICP 作为安全网。  
- **中长期**：FPFH 半径可配置、启用 Overlap Transformer、可选子图级匹配，进一步稳定树木场景回环数量与质量。
