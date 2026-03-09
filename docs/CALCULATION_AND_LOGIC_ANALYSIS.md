# 计算与逻辑分析报告

## 0. Executive Summary

| 结论 | 说明 |
|------|------|
| **已核对** | 回环 delta_T（T_tgt_src）、里程计 rel、子图位姿更新 delta、buildGlobalMap 位姿选择、iSAM2 BetweenFactor 约定均与设计一致。 |
| **已修复** | ① odom/kfinfo 缓存在“无 ts≤cloud_ts”时不再使用 future 数据，改为返回 false；② MapExporter 轨迹与元数据边界统一使用优化位姿（与 buildGlobalMap/saveMapToFiles 一致）。 |
| **未改** | 核心数学（rel/computeOdomInfoMatrix、infoToNoise、GPS 协方差、ENU→WGS84）逻辑正确，仅做防御性加固时已覆盖。 |

---

## 1. 计算链路核对结果

### 1.1 位姿与因子图约定

- **GTSAM BetweenFactor(from, to, rel)**：约束为 `pose_to = pose_from * rel`，即 `rel = pose_from^{-1} * pose_to`（to 在 from 系下的位姿）。
- **里程计因子**：`rel = prev->pose_w_anchor_optimized.inverse() * submap->pose_w_anchor`，即前一子图锚定到当前子图锚定的相对位姿，与 BetweenFactor(prev, curr, rel) 一致。✅
- **回环因子**：TEASER 输出 `T_tgt_src`（将 source=query 点变换到 target 系），即 target 系下 query 的位姿；`submap_i=target, submap_j=query`，BetweenFactor(i, j, T_i_j) 需要 `rel = pose_i^{-1} * pose_j = T_tgt_query`，与 `T_tgt_src` 一致。✅
- **子图位姿更新**：`delta = new_pose * old_anchor.inverse()`，`kf->T_w_b_optimized = delta * kf->T_w_b`，保持锚定系内相对关系，与文档一致。✅

### 1.2 时间戳对齐

- **odomCacheGet(ts)**：原逻辑在“所有缓存 ts > cloud_ts”时回退到 `front()`，会用到**未来**里程计对齐当前帧，存在逻辑错误。
- **修复**：仅当存在 `it->ts <= ts` 的条目时才返回 true 并赋值；否则返回 false，后端 worker 按现有逻辑 skip 该帧。✅
- **kfinfoCacheGet(ts)**：同样在无 `ts<=cloud_ts` 时不再使用 front()，返回 false，worker 使用 `last_livo_info_`。✅

### 1.3 导出与显示位姿一致性

- **buildGlobalMap**：已使用 `T_w_b_optimized`，Identity 时回退到 `T_w_b`。✅
- **saveMapToFiles**：轨迹 TUM 与 keyframe_poses PCD 使用 `T_w_b_optimized`。✅
- **MapExporter**：原先轨迹 txt/KML/CSV 与 buildMetadata 边界均使用 `T_w_b`，与全局图/保存轨迹不一致。
- **修复**：exportTrajectory、exportTrajectoryKML、exportTrajectoryCSV、buildMetadata 中位姿统一为“优先 T_w_b_optimized，若为 Identity 则用 T_w_b”，与 buildGlobalMap 逻辑一致。✅

---

## 2. 涉及文件与变更

| 文件 | 变更 |
|------|------|
| `automap_pro/src/system/automap_system.cpp` | odomCacheGet：无 ts≤cloud_ts 时 return false；kfinfoCacheGet：同上。 |
| `automap_pro/src/map/map_exporter.cpp` | exportTrajectory / exportTrajectoryKML / exportTrajectoryCSV / buildMetadata：位姿选用 T_w_b_optimized（Identity 回退 T_w_b）。 |

---

## 3. 验证建议

- **时间戳**：回放 bag，观察后端在 odom 晚于点云到达时是否正确 skip（no odom in cache），且无错误对齐。
- **导出**：保存地图后对比 trajectory.txt 与 RViz 中 optimized_path、global_map 是否一致（均应为优化后位姿）。

---

## 4. 未改动的正确逻辑（简要）

- **ICP 初值**：loop_detector 传入 TEASER 的 `T_tgt_src` 作为 ICP 的 initial，ICP 内部使用 T_src_tgt（src→tgt），与 T_tgt_src 等价（同一变换）。✅
- **infoToNoise**：信息矩阵求逆前做秩与条件数检查、正则化，GTSAM 使用 Covariance。✅
- **computeOdomInfoMatrix**：prev/curr 空检查与 kf 空遍历已在前序异常分析中加固。✅
- **首子图 pose_w_anchor_optimized**：addKeyFrame 首帧时已设为 kf->T_w_b，与 pose_w_anchor 一致，odom 因子 rel 计算正确。✅
