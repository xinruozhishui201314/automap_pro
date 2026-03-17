# 重影根因精确定位与 GHOSTING_SOURCE 日志使用指南

**目的**：通过新增的 `[GHOSTING_SOURCE]` 与既有 `[GHOSTING_DIAG]` 日志，精确定位重影来自哪一环节，并据此修复。

---

## 0. Executive Summary

| 可能原因 | 含义 | 如何用日志确认 |
|----------|------|----------------|
| **odom_path 与 global_map 同屏** | 轨迹用 T_w_b(odom)，点云用 T_w_b_optimized，二者不同源 → 视觉上轨迹与点云错位 | 若见 `GHOSTING_SOURCE odom_path published` 且 RViz 同时显示 odom_path 与 global_map → 关闭 odom_path 或仅显示 optimized_path |
| **部分 KF 用 T_w_b 参与 build** | T_w_b_optimized=Identity 时 fallback 到 T_w_b，该帧点云在 odom 系 → 与其余帧错位 | grep `snapshot_kf_fallback` 或 `buildGlobalMap_sync.*T_w_b_fallback`，若有则重影来自这些 kf_id |
| **fallback_merged_cloud 且未 rebuild** | 主路径 combined 为空时用 merged_cloud；若 HBA 后未执行 rebuild，merged_cloud 仍为 T_w_b 系 | grep `path=fallback_merged_cloud` 与 `merged_cloud rebuild_done` 时间顺序：若 fallback 在 rebuild_done 之前则重影 |
| **build 与 HBA 写回重叠（已缓解）** | 一次 build 内读到混合 pre/post HBA 位姿 | grep `buildGlobalMap_enter` / `updateAllFromHBA enter` 时间线；当前实现已用快照且 GHOSTING_FIX 避免重叠 |

---

## 1. 重影相关数据源（谁用哪套位姿）

| 显示/发布物 | 位姿来源 | 日志标记 |
|-------------|----------|----------|
| **odom_path** | T_w_b（里程计） | `[GHOSTING_SOURCE] odom_path published pose_source=odom(T_w_b)` |
| **optimized_path**（最终发布） | 各 KF 的 T_w_b_optimized | `[GHOSTING_SOURCE] optimized_path publishing pose_source=KF_T_w_b_optimized` |
| **global_map**（async） | 快照时的 T_w_b_optimized（或 fallback T_w_b） | `[GHOSTING_SOURCE] map_published pose_source=async_snapshot(...)` + `snapshot_kf_fallback` |
| **global_map**（sync） | 现场读 kf->T_w_b_optimized（或 fallback T_w_b） | `[GHOSTING_SOURCE] map_published pose_source=sync_live(...)` + `buildGlobalMap_sync.*T_w_b_fallback` |
| **fallback merged_cloud** | 各子图 merged_cloud：若已 rebuild 则为 T_w_b_optimized，否则为 T_w_b | `[GHOSTING_SOURCE] buildGlobalMap path=fallback_merged_cloud` + `merged_cloud rebuild_done` |

**结论**：只要 **odom_path 与 global_map 同时显示**，就会看到“轨迹与点云错位”（重影）。optimized_path 与 global_map 应同源（均为 T_w_b_optimized），若仍重影则必为 fallback 或 fallback_merged_cloud 未 rebuild。

---

## 2. 新增日志清单与 grep 用法

### 2.1 轨迹与地图发布（位姿来源）

```bash
# 每约 5 秒一条：odom_path 使用 odom，与 global_map 不同源
grep "GHOSTING_SOURCE.*odom_path" full.log

# 每次 onPoseUpdated：optimized_path 使用 KF T_w_b_optimized
grep "GHOSTING_SOURCE.*optimized_path publishing" full.log

# 每次 map 发布：global_map 使用的位姿来源（async_snapshot / sync_live）
grep "GHOSTING_SOURCE.*map_published" full.log
```

### 2.2 快照与 fallback（精确到 KF）

```bash
# 快照中哪些 KF 因 T_w_b_optimized=Identity 用了 T_w_b（每出现一条即一帧可能重影）
grep "GHOSTING_SOURCE.*snapshot_kf_fallback" full.log

# 快照汇总：总 KF 数、fallback 数量
grep "GHOSTING_SOURCE.*buildGlobalMap snapshot" full.log

# sync 路径下 fallback 的 KF（非 async 时）
grep "GHOSTING_SOURCE.*buildGlobalMap_sync.*T_w_b_fallback" full.log
```

### 2.3 merged_cloud 与 fallback 路径

```bash
# 何时进入 rebuild、何时完成（HBA 后必须有一次 done）
grep "GHOSTING_SOURCE.*merged_cloud rebuild" full.log

# 是否走了 fallback_merged_cloud（主路径为空时）
grep "GHOSTING_SOURCE.*path=fallback_merged_cloud" full.log
```

### 2.4 时间线串联（与既有 GHOSTING_DIAG 一起用）

```bash
# 一次完整时间线：build 开始 → 快照 → build 结束 → map 发布
grep -E "buildGlobalMap_enter|pose_snapshot_taken|buildGlobalMapInternal_exit|map_published|GHOSTING_SOURCE.*map_published" full.log

# HBA 写回与 rebuild 顺序（应为 enter → done → 再 build）
grep -E "updateAllFromHBA enter|rebuildMergedCloud.*enter|rebuildMergedCloud.*done|GHOSTING_SOURCE.*merged_cloud rebuild" full.log
```

---

## 3. 精确定位流程（按优先级）

1. **先看 RViz 显示**  
   - 若同时显示 **odom_path** 与 **global_map** → 先关闭 odom_path，仅保留 **optimized_path** + **global_map**，重影多半消失。  
   - 对应根因：odom 与优化后点云不同源，属“显示源不一致”而非建图错误。

2. **再查 fallback**  
   - `grep "GHOSTING_SOURCE.*snapshot_kf_fallback\|buildGlobalMap_sync.*T_w_b_fallback" full.log`  
   - 若有输出：这些 kf_id 的点云用了 T_w_b，与其余帧（T_w_b_optimized）错位 → 需排查为何这些 KF 的 T_w_b_optimized 为 Identity（未参与优化或未写回）。

3. **再查 fallback_merged_cloud**  
   - `grep "GHOSTING_SOURCE.*fallback_merged_cloud\|merged_cloud rebuild_done" full.log`  
   - 若存在 `path=fallback_merged_cloud`，且该次 build 早于最后一次 `merged_cloud rebuild_done`，则当时用的 merged_cloud 可能仍为 T_w_b 系 → 重影来自 fallback 路径未与 HBA 同步。

4. **最后查时序（混合位姿）**  
   - 若已确认无 odom 同屏、无 fallback、无未 rebuild 的 fallback，再查 `buildGlobalMap_enter` 与 `updateAllFromHBA enter` 是否重叠（当前实现应已避免）。

---

## 4. 代码位置速查

| 日志内容 | 文件 | 说明 |
|----------|------|------|
| odom_path published | automap_system.cpp | onOdometry 中节流发布后 |
| optimized_path publishing | automap_system.cpp | onPoseUpdated 中 publishOptimizedPath 前 |
| map_published pose_source= | automap_system.cpp | publishGlobalMap 中 build 完成后 |
| snapshot_kf_fallback / buildGlobalMap snapshot | submap_manager.cpp | buildGlobalMapAsync 快照循环内/后 |
| buildGlobalMap_sync T_w_b_fallback | submap_manager.cpp | buildGlobalMap 主路径 T_w_b_optimized=Identity 分支 |
| path=fallback_merged_cloud | submap_manager.cpp | buildGlobalMap 主路径 combined 为空时 |
| merged_cloud rebuild_enter / rebuild_done | submap_manager.cpp | rebuildMergedCloudFromOptimizedPoses 入口/出口 |

---

## 5. 验证建议

- 同 bag 回放后：  
  - 仅显示 **optimized_path** + **global_map**，隐藏 odom_path；  
  - 再执行上述 grep，确认无 `snapshot_kf_fallback`、无 `path=fallback_merged_cloud`（或 fallback 仅在 rebuild_done 之后）；  
  - 若重影仍存在，把含 `[GHOSTING_SOURCE]` 与 `[GHOSTING_DIAG]` 的片段贴出，可进一步精确到某次 build、某 kf_id。
