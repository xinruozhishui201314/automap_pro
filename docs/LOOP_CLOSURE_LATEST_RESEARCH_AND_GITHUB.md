# 基于最新研究与开源的回环数量提升方法

基于 2023–2025 年 LiDAR 回环/位置识别研究与 GitHub 开源，整理可提高回环数量的方法与可落地方案。

---

## 0. 结论速览

| 优先级 | 方法 | 与本工程关系 | 预期效果 |
|--------|------|--------------|----------|
| **P0 立即可做** | 启用 OverlapTransformer（LibTorch） | 本工程已集成，编译时启用 USE_TORCH 即可 | 检索质量优于 ScanContext，尤其重复结构/树木 |
| **P0 已做** | FPFH 几何过滤 + 树多参数（fpfh_corr_max_distance_m=5, min_inlier_ratio=0.04） | 已写入 system_config_M2DGR.yaml | 过滤误对应，提高 TEASER 通过率 |
| **P1 短期** | 多描述子融合（ScanContext + OverlapTransformer 双路检索取并集） | 需改 loop_detector 检索逻辑 | 候选更全，真闭环更易进 top-k |
| **P2 中期** | 接入 LCR-Net 或 SALSA 做检索/粗位姿 | 需接入 PyTorch 推理或 C++ 导出 | 检索与粗位姿更稳，减少对 FPFH 依赖 |
| **P3 树木专用** | ForestLPR / TreeLoc 思路（多 BEV 密度图、树干几何） | 需新模块或替换描述子 | 针对森林/树多场景显著提升 |

---

## 1. 本工程已具备：OverlapTransformer

- **论文/代码**：OverlapTransformer (RAL/IROS 2022)，GitHub: [haomo-ai/OverlapTransformer](https://github.com/haomo-ai/OverlapTransformer)，C++ 单帧约 2ms。
- **本工程**：`overlap_transformer_infer.cpp` 已集成；若编译时 **LibTorch 可用**（USE_TORCH），则用 OverlapTransformer 做描述子与检索；否则回退到 ScanContext。日志中若出现 `[OT] LibTorch not available; use loop_closure.scancontext.enabled=true` 表示当前未用 OT。
- **建议**：在树多/重复结构场景下，**优先启用 LibTorch 编译并加载 OverlapTransformer 模型**（配置中已有 `loop_closure.overlap_transformer.model_path`），用 OT 替代或与 ScanContext 并联做检索，通常能提高回环候选质量与数量。

---

## 2. 近年开源算法与 GitHub（可参考集成）

### 2.1 端到端检索 + 位姿（适合替代/增强当前「检索 + FPFH+TEASER」）

| 名称 | 发表 | GitHub | 特点 | 对回环数量的帮助 |
|------|------|--------|------|------------------|
| **LCR-Net** | TRO | [nubot-nudt/LCR-Net](https://github.com/nubot-nudt/LCR-Net) | 相似度 + 6-DoF 粗位姿一体，已接入 SLAM | 检索更准、粗位姿可直接用于约束或初值，减少对 FPFH 的依赖，有利于通过几何验证 |
| **SALSA** | RA-L 2024 | [raktimgg/SALSA](https://github.com/raktimgg/salsa) | SphereFormer + 自注意力池化，实时 SOTA | 提升检索召回与精度，候选里真闭环更多 → 后续 TEASER 通过更多 |
| **CFPR** | ICRA 2024 | [fcchit/CFPR](https://github.com/fcchit/CFPR) | 粗到精 + 注意力描述子 + 重叠估计 | 粗筛选更多真闭环，精化阶段更稳 |

### 2.2 检索/描述子（适合与 ScanContext/OT 并联或替换）

| 名称 | 发表 | 说明 | 对回环数量的帮助 |
|------|------|------|------------------|
| **P-GAT** | RA-L 2024 | [csiro-robotics/P-GAT](https://github.com/csiro-robotics/P-GAT)，姿态图注意力网络 | 利用位姿图结构做位置识别，适合已有子图/轨迹的 SLAM |
| **OverlapTransformer 系列** | RAL 2022 + 后续 | [haomo-ai/OverlapTransformer](https://github.com/haomo-ai/OverlapTransformer)，CVTNet/SeqOT 等扩展 | 与本工程已集成一致；启用即可提升检索质量 |

### 2.3 树木/森林场景（直接针对树多）

| 名称 | 说明 | 对回环数量的帮助 |
|------|------|------------------|
| **ForestLPR** | 多高度 BEV 密度图 + ViT，面向森林（高自相似、植被变化） | 针对树多场景设计，可显著提高森林/树多场景的闭环检测数量与鲁棒性 |
| **TreeLoc** | 树干/DBH 几何表示 + 分布直方图粗匹配 + 2D 三角形描述子精匹配 | 适合强树木结构场景，与当前「点云+FPFH」形成互补思路 |
| **密度图方法** | BEV 密度图 + ORB 等特征 + 二叉检索 | 对扫描模式/分辨率变化更鲁棒，便于多传感器与实时 |

### 2.4 ScanContext 增强与几何策略

| 名称 | 说明 | 对回环数量的帮助 |
|------|------|------------------|
| **When-to-Loop** | 基于 ScanContext 的「何时触发回环」策略（城市） | 减少无效触发、提高有效回环比例 |
| **SGLC** | 语义图引导的粗-精-精化闭环，语义图 + 背景几何 | 语义+几何联合，有利于复杂/重复场景的稳定闭环 |
| **Weighted ScanContext / LiDAR Iris** | 加权或稀疏高度特征的 ScanContext 变体 | 在保留 ScanContext 接口前提下提升区分度 |

---

## 3. 按本工程落地的建议顺序

### 3.1 不改大架构、先做即效

1. **启用 OverlapTransformer（LibTorch）**  
   - 编译选项打开 LibTorch，配置 `loop_closure.overlap_transformer.model_path` 指向正确 `.pth.tar`，确保日志中不再出现「LibTorch not available」。
2. **保持当前树多参数**  
   - `fpfh_corr_max_distance_m: 5.0`、`min_inlier_ratio: 0.04`、`teaser.voxel_size: 0.5` 等（已写在 system_config_M2DGR.yaml）。
3. **可选：双路检索**  
   - 同时跑 ScanContext 与 OverlapTransformer 检索，对两路候选取并集（或按分数融合）后再进 TEASER，在不改几何验证的前提下提高真闭环进入候选的概率。

### 3.2 短期：引入一个深度学习检索模块

- **优先考虑 LCR-Net 或 SALSA**：  
  - LCR-Net：可直接输出相似度 + 6-DoF，便于替代或辅助「ScanContext/OT + FPFH+TEASER」中的检索与粗位姿。  
  - SALSA：仅做全局描述子检索时，接口与 OverlapTransformer 类似，可做成「OT 失败时回退」或「OT + SALSA 双路」。
- 集成方式：PyTorch 转 ONNX/LibTorch 在 C++ 中推理，或单独 Python 服务通过 ROS2 srv 提供描述子/相似度，与本工程现有 OverlapTransformer 服务模式一致。

### 3.3 树多场景专项

- **ForestLPR**：若作者开源或复现可得，适合作为「树多/森林」场景的默认检索描述子或与 OT 并联，对提高树多场景回环数量最有针对性。  
- **TreeLoc**：若场景中树干清晰、可提取树干几何，可考虑用其粗匹配结果作为「额外候选源」，与现有点云回环并行。

### 3.4 几何验证侧（保持现有管线时）

- 继续依赖 **TEASER + ICP**，已具备鲁棒性。  
- 若引入 LCR-Net 等提供粗 6-DoF，可将该位姿作为 TEASER/ICP 的初值，进一步减少误拒与提高收敛率，间接提高回环数量。

---

## 4. 参考文献与链接汇总

| 项目 | 链接 |
|------|------|
| LCR-Net | https://github.com/nubot-nudt/LCR-Net |
| SALSA | https://github.com/raktimgg/salsa |
| CFPR | https://github.com/fcchit/CFPR |
| P-GAT | https://github.com/csiro-robotics/P-GAT |
| OverlapTransformer | https://github.com/haomo-ai/OverlapTransformer |
| ForestLPR | arXiv:2503.04475（关注是否开源） |
| TreeLoc | arXiv:2602.01501 |

---

## 5. 代码层面已实现优化（参考 OverlapTransformer / SeqOT）

以下优化已在本工程中实现，对应最新研究与开源实践。

### 5.1 高置信度绕过几何预筛（High-Score Bypass）

- **依据**：OverlapTransformer / SeqOT 等工作中，**高 overlap 分数**通常对应真实闭环，而闭环时两子图锚点在**未校正位姿**下距离往往很大（>50m），若仅按锚点距离预筛会误杀真实回环。
- **实现**：新增配置 `loop_closure.geo_prefilter_skip_above_score`（默认 0，建议 0.90~0.95）。当候选描述子相似度 **≥ 该阈值**时，**跳过**几何距离预筛，直接进入 TEASER。
- **代码**：`ConfigManager::loopGeoPrefilterSkipAboveScore()`，`loop_detector.cpp` 中在构建 `geo_filtered_candidates` 时先判断 `cand.score >= geo_prefilter_skip_above_score_`，满足则直接加入候选并 continue。
- **配置示例**：`geo_prefilter_skip_above_score: 0.92`（与 `system_config_M2DGR.yaml` 中已添加一致）。

### 5.2 子图间关键帧级匹配（OverlapTransformer 模式）

- **依据**：SeqOT（TIE 2022）、EINet（IROS 2024）等采用**序列/关键帧级**匹配提升召回；原逻辑仅在 `scancontext.enabled=true` 时做子图间关键帧级，OT 模式下只有子图级，子图间回环机会少。
- **实现**：当 `inter_keyframe_level=true` 且 **未使用 ScanContext**（即使用 OverlapTransformer）时，增加**子图间关键帧级**分支：
  - 对当前子图与候选子图调用 `prepareIntraSubmapDescriptors`，补齐 **OT 关键帧描述子**；
  - 按 `inter_keyframe_sample_step` 采样 query 关键帧，对每个候选子图在其关键帧描述子中做余弦相似度检索，取 **top_k_per_submap** 构成 `InterKfCandidate`；
  - 入队 `MatchTask`（`query_kf_idx`、`candidates_kf`），由现有 `processMatchTask` 走 TEASER 几何验证。
- **代码**：`loop_detector.cpp` 中在 ScanContext 关键帧级分支之后、子图级入队之前，新增 `INTER_KEYFRAME_OT` 分支；日志关键字 `stage=match_enqueue INTER_KEYFRAME_OT`。
- **效果**：在 OT 模式下也能产生大量「子图间关键帧↔关键帧」候选，与 ScanContext 模式对称，提高子图间回环数量。

### 5.3 参考文献与仓库

| 项目 | 说明 |
|------|------|
| [haomo-ai/OverlapTransformer](https://github.com/haomo-ai/OverlapTransformer) | RAL/IROS 2022，范围图 + 全局描述子，高 overlap 即可靠 |
| [BIT-MJY/SeqOT](https://github.com/BIT-MJY/SeqOT) | TIE 2022，序列 LiDAR 时空 Transformer，关键帧/序列级匹配 |
| EINet (IROS 2024) | 以 OverlapTransformer 为 backbone 的序列位置识别 |

---

## 6. 小结

- **立刻可做**：启用本工程已集成的 OverlapTransformer（LibTorch）+ 保持树多参数（fpfh_corr_max_distance_m=5、min_inlier_ratio=0.04 等），并可选双路检索（ScanContext + OT）。  
- **短期**：参考 LCR-Net、SALSA 等 2024 年开源，接入一个深度学习检索（或检索+粗位姿）模块，与现有 FPFH+TEASER 管线配合，提高候选质量与通过率。  
- **树多专项**：关注 ForestLPR、TreeLoc 等面向森林/树木的开源或复现，作为树多场景下提高回环数量的重点方向。  
- **代码已实现**：高置信度绕过几何预筛（`geo_prefilter_skip_above_score`）、OT 子图间关键帧级匹配（INTER_KEYFRAME_OT），见上文 §5。

上述方法均可与当前「ScanContext/OT → FPFH → TEASER → ICP」流程兼容：要么提升检索（更多真闭环进候选），要么提供更好初值（提高 TEASER/ICP 通过率），从而在树多场景下提高回环数量。
