# system_config_M2DGR.yaml 点云模糊优化参数分析

## 0. Executive Summary

| 目标 | 针对 M2DGR 配置，标出优化后能**改善全局点云模糊**的参数，并给出取值建议与取舍。 |
|------|-----------------------------------------------------------------------------|
| **高优先级** | `map.voxel_size`、`keyframe.retain_cloud_body`、`fast_livo.preprocess.filter_size_surf` |
| **中优先级** | `submap.merge_resolution`、`submap.match_resolution`、LIO `voxel_size`、`blind` |
| **间接影响** | 回环/HBA/GPS 相关参数 → 位姿更准 → 减少重影 |

---

## 1. 模糊类型与参数映射

| 模糊表现 | 主要成因 | 本配置中可调参数 |
|----------|----------|------------------|
| **块状/分辨率低** | 体素下采样过粗、前端点云已稀疏 | `map.voxel_size`、`fast_livo.preprocess.*`、`submap.merge_resolution` |
| **重影/厚度** | 位姿误差、回退路径、未优化 KF | `keyframe.retain_cloud_body`、HBA/回环/GPS、`submap.match_resolution` |
| **噪点/抖动** | 前端配准噪声、运动畸变、盲区 | `fast_livo.lio.*`、`fast_livo.preprocess.blind`、时间同步 |

---

## 2. 参数逐项分析（按配置节）

### 2.1 地图输出（直接决定全局图观感）

| 参数 | 当前值 | 建议范围（改善模糊） | 说明与取舍 |
|------|--------|----------------------|------------|
| **map.voxel_size** | **0.35** | **0.20～0.25** | 全局图发布前体素滤波。当前 0.35 易导致“块状/糊”；降到 0.2～0.25 可明显提升清晰度。代码下限 0.2（ConfigManager），设更小也会被 clamp。**代价**：点数增多，buildGlobalMap/传输/显示更吃资源。 |
| map.statistical_filter | true | 保持 true | 统计滤波去离群点，有利于观感；关闭可能增加噪点。 |

**结论**：优先将 `map.voxel_size` 从 0.35 改为 **0.2** 或 **0.25**，是改善“块状模糊”最直接的一步。

---

### 2.2 关键帧（避免回退、保证主路径）

| 参数 | 当前值 | 建议（改善模糊） | 说明与取舍 |
|------|--------|------------------|------------|
| **keyframe.retain_cloud_body** | **true** | **必须 true** | 为 false 时无关键帧点云会走回退路径（merged_cloud 旧世界系），全局图与优化轨迹错位 → 重影。当前已 true，勿改。 |
| keyframe.allow_cloud_archival | false | 保持 false | 为 true 时可能因归档丢弃点云触发回退，易重影。 |
| keyframe.max_memory_mb | 4096 | 可保持或略增 | 内存不足时可能被迫清空 cloud_body，触发回退；若机器内存充足可保持 4096 或适当增大。 |
| keyframe.min_translation | 1.0 | 0.8～1.0 | 略减小可增加关键帧密度，全局图更密；过小会增多 KF、内存与算力上升。 |
| keyframe.max_interval | 2.0 | 1.5～2.0 | 略减小可避免长时间无 KF 导致局部稀疏。 |

**结论**：保持 `retain_cloud_body: true` 与 `allow_cloud_archival: false`；若仍出现回退，再查内存与 `max_memory_mb`。

---

### 2.3 前端 Fast-LIVO：预处理（点云密度与盲区）

| 参数 | 当前值 | 建议范围（改善模糊） | 说明与取舍 |
|------|--------|----------------------|------------|
| **fast_livo.preprocess.filter_size_surf** | **0.25** | **0.15～0.20** | 面点体素滤波，越小点越密。当前 0.25 已较粗；降到 0.2 或 0.15 可减轻“输入就稀疏”导致的糊。**代价**：点更多，LIO 与后端耗时略增。 |
| fast_livo.preprocess.point_filter_num | 3 | 2～3 | 抽帧数，越小保留点越多；过小会增加计算量。 |
| **fast_livo.preprocess.blind** | **1.0** | **0.5～1.0** | 最小量程(m)，近于 blind 的点会被滤掉。若场景近处结构重要，可适当减小（如 0.5）；过小可能引入近处噪点。 |

**结论**：在 LIO 仍稳定的前提下，优先尝试 `filter_size_surf: 0.20`；若单帧点数仍少（见 `[GLOBAL_MAP_BLUR] sparse_keyframe`），再试 0.15 或略减 `blind`。

---

### 2.4 前端 Fast-LIVO：LIO（配准质量 → 位姿与噪点）

| 参数 | 当前值 | 建议范围（改善模糊） | 说明与取舍 |
|------|--------|----------------------|------------|
| fast_livo.lio.voxel_size | 0.25 | 0.2～0.25 | 前端体素地图分辨率；略小可提高配准精度、减轻漂移与重影，但计算更重。 |
| fast_livo.lio.max_iterations | 5 | 5～8 | 略增可提高收敛质量，位姿更稳。 |
| fast_livo.lio.dept_err / beam_err | 0.02, 0.05 | 保持或略紧 | 残差阈值，过松会接受差匹配；过紧可能不收敛。 |
| fast_livo.lio.min_eigen_value | 0.0025 | 保持 | 与退化相关，一般不动。 |

**结论**：LIO 参数以稳定为主；若轨迹漂移明显、重影多，可尝试 `voxel_size: 0.2` 或 `max_iterations: 6～8`。

---

### 2.5 子图（合并分辨率与匹配）

| 参数 | 当前值 | 建议范围（改善模糊） | 说明与取舍 |
|------|--------|----------------------|------------|
| **submap.merge_resolution** | **0.2** | **0.15～0.2** | 子图内 merged_cloud 下采样分辨率；仅影响回退路径与子图展示。主路径用关键帧 cloud_body，不直接受此约束；若希望回退时子图更细，可试 0.15。 |
| **submap.match_resolution** | **0.4** | **0.3～0.4** | 关键帧下采样用于回环匹配；不影响 buildGlobalMap 主路径的点云分辨率，但影响回环匹配质量 → 位姿更准 → 少重影。可试 0.3。 |

**结论**：改善模糊时，优先保证位姿（见回环/HBA）；子图分辨率可次要地试 `merge_resolution: 0.15`、`match_resolution: 0.3`。

---

### 2.6 回环与后端优化（位姿一致性 → 少重影）

| 参数 | 当前值 | 建议（改善模糊） | 说明与取舍 |
|------|--------|------------------|------------|
| backend.hba.trigger_on_loop | false | 可试 true | 回环时触发 HBA，更快纠正漂移，减少重影；代价是算力与触发频率。 |
| backend.hba.trigger_every_n_submaps | 10 | 8～10 | 更频繁优化可减少累积误差。 |
| backend.hba.enable_gps_factor | true | 保持 true | GPS 约束有助于全局一致，减少重影。 |
| loop_closure.teaser.max_rmse_m | 0.3 | 0.2～0.3 | 回环约束更紧，位姿更准；过紧可能漏回环。 |
| loop_closure.teaser.icp_refine | true | 保持 true | 精化相对位姿，有利于对齐。 |

**结论**：若日志中 `kf_fallback_unopt` 多或轨迹明显漂移，优先保证 HBA 正常触发（含 `trigger_on_finish: true`），并可试 `trigger_on_loop: true`。

---

### 2.7 其他相关项

| 参数 | 当前值 | 说明 |
|------|--------|------|
| fast_livo.common.ros_driver_bug_fix | true | 时间对齐，有利于 LIO 与建图一致；保持。 |
| frontend.cloud_frame | "world" | 与 fast_livo 输出一致；保持。 |
| gps.add_constraints_on_align | true | GPS 约束参与优化；保持。 |

---

## 3. 推荐调整优先级（改善模糊）

1. **必调（直接缓解块状/糊）**  
   - `map.voxel_size`: **0.35 → 0.2**（或 0.25）

2. **建议调（提升输入密度）**  
   - `fast_livo.preprocess.filter_size_surf`: **0.25 → 0.20**（若稳定可试 0.15）

3. **保持不动（防重影）**  
   - `keyframe.retain_cloud_body: true`  
   - `keyframe.allow_cloud_archival: false`

4. **按需调（位姿更好 → 少重影）**  
   - `submap.match_resolution`: 0.4 → 0.3  
   - `backend.hba.trigger_on_loop`: false → true  
   - `fast_livo.lio.voxel_size`: 0.25 → 0.2（若 LIO 稳定）

5. **可选（回退路径与子图更细）**  
   - `submap.merge_resolution`: 0.2 → 0.15  
   - `fast_livo.preprocess.blind`: 1.0 → 0.5（仅当近处结构重要时）

---

## 4. 可直接粘贴的优化片段（M2DGR 模糊优化）

在 `system_config_M2DGR.yaml` 中仅修改以下片段即可做 A/B 对比（先备份原配置）：

```yaml
# ── 2.3 关键帧（保持主路径、防回退）──
keyframe:
  min_translation:    1.0
  min_rotation_deg:  10.0
  max_interval:       2.0
  max_esikf_cov_norm: 1000.0
  retain_cloud_body: true      # 必须 true
  allow_cloud_archival: false
  max_memory_mb: 4096

# ── 2.4 前端预处理（提高点云密度）──
  preprocess:
    lidar_type:       2
    scan_line:        32
    point_filter_num: 3
    filter_size_surf: 0.20     # 原 0.25，改小以减轻稀疏导致的糊
    blind:            1.0

# ── 3.1 子图（可选：匹配与合并更细）──
submap:
  max_keyframes:    100
  max_spatial_m:    100.0
  max_temporal_s:   60.0
  match_resolution: 0.35      # 原 0.4，略细有利于回环位姿
  merge_resolution: 0.2       # 可试 0.15

# ── 3.5 地图输出（最关键：减轻块状模糊）──
map:
  voxel_size:        0.2      # 原 0.35，显著提升清晰度
  statistical_filter: true
```

**说明**：若机器内存或算力紧张，可先将 `map.voxel_size` 改为 **0.25**，再视效果降到 0.2。

---

## 5. 验证与回滚

- **验证**：跑同一段 bag，对比修改前后：  
  - 日志中 `[GLOBAL_MAP_BLUR]` 的 `blur_risk`、`comp_pct`、`voxel`；  
  - RViz 中全局图观感（块状是否减轻、重影是否减少）。  
- **回滚**：若 LIO 发散或 buildGlobalMap 过慢，优先恢复 `map.voxel_size` 与 `filter_size_surf`，再逐项还原其余参数。

---

## 6. 与 GLOBAL_MAP_BLUR_ANALYSIS 的关系

- **GLOBAL_MAP_BLUR_ANALYSIS.md**：模糊根因、数据流、诊断方法。  
- **本文**：针对 **system_config_M2DGR.yaml** 的逐参数说明与优化建议，便于直接改配置并验证效果。
