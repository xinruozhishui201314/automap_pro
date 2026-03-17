# 重影排查日志索引（GHOSTING_LOG_INDEX）

重影问题难以定位时，按本索引 **grep 对应 TAG** 即可串联时间线、位姿来源与回环入图情况。所有能辅助定位重影的日志均带有下表所列 TAG 之一。

---

## 0. 快速排查（先跑这几条）

```bash
# 首次建图时的重影排查入口（文档与主要 grep）
grep GHOSTING_QUICK_REF full.log

# 一眼判断是否两系错位（odom vs 优化系）
grep GHOSTING_CHEAT_SHEET full.log

# 时间线：写回 vs 快照 vs 建图
grep GHOSTING_DIAG full.log

# 回环是否入图（0 条 = 点云结构重影）
grep LOOP_ACCEPTED full.log

# 位姿来源与 fallback
grep GHOSTING_SOURCE full.log

# HBA 相关（写回、rebuild、末帧 odom→opt 平移）
grep HBA_GHOSTING full.log
grep REBUILD_MERGE full.log
```

---

## 1. 按 TAG 分类索引

| TAG | 含义 | 典型 grep | 文件位置 |
|-----|------|-----------|----------|
| **GHOSTING_CHEAT_SHEET** | 一行汇总：odom_last / opt_last / map_pts / diff_odom_opt_m；diff>1m 则同屏必轨迹重影 | `grep GHOSTING_CHEAT_SHEET` | automap_system.cpp (publishGlobalMap) |
| **GHOSTING_DIAG** | 重影时间线：build_enter/exit、writeback_enter/done、pose_snapshot_taken、rebuild_done、map_published、onPoseUpdated_enter/exit | `grep GHOSTING_DIAG` | automap_system, submap_manager, incremental_optimizer |
| **GHOSTING_SOURCE** | 位姿来源：odom_path=odom(T_w_b)；optimized_path=KF_T_w_b_optimized；map=async_snapshot(T_w_b_optimized)；fallback 时标明 | `grep GHOSTING_SOURCE` | automap_system, submap_manager |
| **GHOSTING_QUICK_REF** | 每次 publishGlobalMap 时打印的重影排查入口：文档路径与主要 grep 关键词 | `grep GHOSTING_QUICK_REF` | automap_system (publishGlobalMap) |
| **GHOSTING_FIX** | 为避免重影做的逻辑：defer map_publish、skip 并发 build、finish 时 HBA 只触发一次等 | `grep GHOSTING_FIX` | automap_system |
| **HBA_GHOSTING** | HBA 写回/rebuild 与 odom_path 不同系提示；last_kf odom→optimized trans_diff；请隐藏 odom_path | `grep HBA_GHOSTING` | automap_system, submap_manager, hba_optimizer |
| **LOOP_ACCEPTED** | 回环被检测发布或成功加入因子图；无此类日志 = 零回环 = 点云结构重影 | `grep LOOP_ACCEPTED` | loop_detector (publishLoopConstraint + TEASER 通过处), loop_optimization (onLoopDetected enqueue), incremental_optimizer (addLoopFactor/addLoopFactorDeferred 成功) |
| **REBUILD_MERGE** | merged_cloud 用 T_w_b_optimized 重建；Done rebuilding 后 buildGlobalMap 与 map 同系 | `grep REBUILD_MERGE` | submap_manager |
| **GLOBAL_MAP_DIAG** | buildGlobalMap 入口/路径/成功/异常；fallback 时提示可能重影 | `grep GLOBAL_MAP_DIAG` | submap_manager |
| **POSE_DIAG** | 单帧/锚点 T_w_b vs T_w_b_optimized、trans_diff、yaw；写回与 rebuild 抽样 | `grep POSE_DIAG` | submap_manager |
| **HBA_DIAG** | updateAllFromHBA 的 max_trans_diff/max_rot_diff、子图锚点变化 | `grep HBA_DIAG` | submap_manager, hba_optimizer |
| **ISAM2_GHOSTING_DIAG** | notifyPoseUpdate_enter/exit；与 map_publish/buildGlobalMap 时序对照 | `grep ISAM2_GHOSTING_DIAG` | incremental_optimizer |
| **BACKEND_ISAM2_GHOSTING_DIAG** | onPoseUpdated 子图/KF 写回摘要；build 若并发读可能混合旧/新位姿 | `grep BACKEND_ISAM2_GHOSTING_DIAG` | automap_system |
| **INTRA_LOOP** | 子图内回环 SUMMARY/DETECTED；FINAL_detected=0 表示无子图内回环 | `grep INTRA_LOOP` | loop_detector |
| **LOOP_STEP** | 回环流水线：addSubmap、onLoopDetected_enter/enqueue/done、CONSTRAINT 等 | `grep LOOP_STEP` | automap_system, loop_detector |
| **CONSTRAINT** / **BACKEND.*LOOP** | 约束入图/跳过原因（same_node、node_not_in_graph、validation 等） | `grep CONSTRAINT` / `grep BACKEND LOOP` | incremental_optimizer |

---

## 2. 按排查目标选 grep

| 排查目标 | 推荐 grep |
|----------|------------|
| 轨迹重影（两条线错位） | `GHOSTING_CHEAT_SHEET`、`GHOSTING_SOURCE`、`HBA_GHOSTING` |
| 点云结构重影（双墙/双建筑） | `LOOP_ACCEPTED`、`INTRA_LOOP`、`CONSTRAINT`、`BACKEND LOOP` |
| 写回与建图是否竞态 | `GHOSTING_DIAG`（writeback_enter/done 与 pose_snapshot_taken、build_id 时间线） |
| map 是否用了 fallback 位姿 | `GHOSTING_SOURCE`（snapshot_fallback_count、fallback_merged_cloud）、`GLOBAL_MAP_DIAG` |
| HBA 后 odom 与优化系差多少 | `HBA_GHOSTING`（last_kf odom->optimized trans_diff）、`GHOSTING_CHEAT_SHEET` |

---

## 3. 日志输出位置汇总（代码文件）

| 文件 | TAG 出现位置 |
|------|----------------|
| automap_system.cpp | GHOSTING_SOURCE, GHOSTING_DIAG, GHOSTING_CHEAT_SHEET, GHOSTING_FIX, HBA_GHOSTING, BACKEND_ISAM2_GHOSTING_DIAG |
| submap_manager.cpp | GHOSTING_DIAG, GHOSTING_SOURCE, HBA_GHOSTING, REBUILD_MERGE, GLOBAL_MAP_DIAG, POSE_DIAG, HBA_DIAG |
| incremental_optimizer.cpp | ISAM2_GHOSTING_DIAG, CONSTRAINT_LOG/BACKEND LOOP, LOOP_ACCEPTED |
| loop_detector.cpp | INTRA_LOOP, LOOP_STEP, LOOP_ACCEPTED (publishLoopConstraint + TEASER 通过) |
| loop_optimization.cpp (或 automap_system 内 onLoopDetected) | LOOP_ACCEPTED (enqueue 入后端队列) |
| hba_optimizer.cpp | HBA_GHOSTING, GHOSTING_DIAG |
| rviz_publisher.cpp | GHOSTING_DIAG (optimized_path published) |

---

## 4. 重影根因与 TAG 对应关系

- **轨迹重影**：odom_path 与 global_map 不同系 → 查 `GHOSTING_SOURCE`、`GHOSTING_CHEAT_SHEET`、`HBA_GHOSTING`；解决：RViz 隐藏 odom_path。
- **点云结构重影**：零回环导致同地多点云未对齐 → 查 `LOOP_ACCEPTED`、`INTRA_LOOP`、`CONSTRAINT`；解决：让回环检测/几何校验/后端入图生效。
- **写回与 build 竞态**：pose_snapshot 落在 writeback enter~exit 之间 → 查 `GHOSTING_DIAG` 时间线；解决：已通过同锁串行修复，日志用于验证。
- **map 用了 odom 系**：fallback 或 rebuild 前用 merged_cloud → 查 `GHOSTING_SOURCE`、`GLOBAL_MAP_DIAG`、`REBUILD_MERGE`。

本文档与代码中 TAG 保持同步；新增可定位重影的日志时请在此索引中补充对应 TAG 与 grep 说明。
