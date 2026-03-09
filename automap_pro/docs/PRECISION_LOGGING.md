# 建图精度日志说明（PRECISION Logging）

## 1. 概述

工程中已增加统一前缀 `[PRECISION]` 的精度相关日志，用于分析各模块计算精度并据此判断建图质量。所有精度日志均带 `[PRECISION][模块]` 标签，便于 grep/脚本解析。

## 2. 日志标签与含义（大规模覆盖）

| 标签 | 模块 | 含义 | 关键字段 | 输出频率 |
|------|------|------|----------|----------|
| `[PRECISION][LIO]` | 前端 LIO | 位姿协方差(1σ) + 点面残差 | pos_std_xyz, rot_std_rad, mean_residual_m, effct_feat | 每 100 帧 + 每 3000 帧汇总 |
| `[PRECISION][ODOM]` | 后端收 odom | 收到的里程计位姿不确定性 | pos_std_xyz, pos | 每 500 条 odom |
| `[PRECISION][KF]` | 关键帧创建 | 每个新关键帧的位姿协方差 | pos_std_xyz, rot_std_norm | 每创建一个 KF |
| `[PRECISION][SUBMAP]` | 子图创建/冻结 | 新子图锚定帧协方差；冻结时子图几何与锚定不确定性 | pos_std_xyz, kf_count, extent_m | 创建新子图 + 每次子图冻结 |
| `[PRECISION][LOOP]` | 回环检测 | 匹配 RMSE、内点比、信息矩阵尺度 | rmse_m, inlier, score, info_scale | 每次接受回环 |
| `[PRECISION][LOOP]` | 回环优化 | iSAM2 优化后节点数、耗时、final_rmse | nodes_updated, elapsed_ms, final_rmse | 每次添加回环因子后 |
| `[PRECISION][GPS]` | GPS 对齐 | 轨迹-GPS 对齐残差与平移 | rmse_m, matched, t_xyz | 对齐成功时 |
| `[PRECISION][HBA]` | 后端 HBA | 图优化后的平均误差 | final_mme, poses, elapsed_ms | 每次 HBA 完成 |
| `[PRECISION][OPT]` | iSAM2 优化器 | 每次 update 的节点数、因子数、耗时 | nodes_updated, factor_count, elapsed_ms | 每次 commitAndUpdate |

## 3. 如何根据数据判断建图精度

- **前端质量**：看 `[PRECISION][LIO]` 的 `mean_residual_m`（点面残差均值，单位 m）和 `pos_std_xyz`。残差小、位置标准差小表示前端匹配稳定。
- **关键帧/子图质量**：看 `[PRECISION][KF]`、`[PRECISION][SUBMAP]` 的 `pos_std_xyz`。数值持续偏大可能表示前端不确定度高或环境退化。
- **回环质量**：看 `[PRECISION][LOOP]` 的 `rmse_m`（匹配 RMSE）和 `final_rmse`（优化后）。RMSE 过大或 inlier 过低需警惕误回环。
- **全局一致性**：看 `[PRECISION][GPS]` 的 `rmse_m`（对齐残差）和 `[PRECISION][HBA]` 的 `final_mme`。二者越小，全局轨迹与地图越一致。

## 4. 提取与解析

```bash
# 仅看精度相关日志
ros2 run ... 2>&1 | grep '\[PRECISION\]'

# 按模块分别提取（示例：LIO）
ros2 run ... 2>&1 | grep '\[PRECISION\]\[LIO\]'

# 离线 bag 回放后从终端重定向中提取
./run_automap.sh --offline --bag-file ... 2>&1 | tee run.log
grep '\[PRECISION\]' run.log > precision.log
```

可根据 `precision.log` 用 Python/Excel 做时序图或统计（如 LIO mean_residual_m 随帧变化、回环 rmse 分布等）。

**轨迹对比文件**：系统还会将每帧位姿与每条 GPS 写入 CSV（见 [TRAJECTORY_LOG_AND_PLOT.md](TRAJECTORY_LOG_AND_PLOT.md)），可用 `scripts/plot_trajectory_compare.py` 绘制 odom 与 GPS 曲线对比，直观反映建图精度。

## 5. 输出频率与配置

- **LIO**：每 **100** 帧输出一次 `[PRECISION][LIO]`（`kPrecisionLogEveryNFrames`），另每 **3000** 帧输出时间表 + 同格式精度一行（`kLioLogIntervalFrames`）。便于做时序曲线与建图精度分析。
- **ODOM**：后端每收到 500 条 odom 输出一次 `[PRECISION][ODOM]`（与 `[BACKEND][RECV]` 节流同步）。
- **KF**：每创建一个关键帧输出一次。
- **SUBMAP**：创建新子图时输出一次；**子图冻结**时再输出一次（含 kf_count、extent_m、锚定帧 pos_std）。
- **LOOP/OPT**：每次检测到并接受回环输出 `[PRECISION][LOOP]`；iSAM2 每次 update 输出 `[PRECISION][OPT]`。
- **GPS/HBA**：对齐或 HBA 完成时各输出一次。

无需额外配置；若需更密或更疏的 LIO 精度日志，可修改 `kPrecisionLogEveryNFrames`（100）或 `kLioLogIntervalFrames`（3000）后重新编译。

## 6. 编译与验证

- **编译**：与工程一致，使用现有 colcon/catkin 构建即可，无新增依赖。
- **验证**：运行建图（在线或 `--offline` 回放），确认终端或日志中出现 `[PRECISION][LIO]`、`[PRECISION][KF]` 等；用 `grep '\[PRECISION\]' run.log` 应有对应行。
- **Runbook**：若看不到 `[PRECISION][LIO]`，检查是否已处理满 100 帧（默认每 100 帧一条）；若看不到 `[PRECISION][KF]`，确认关键帧已创建（运动/时间满足 KF 条件）。编译使用与工程一致的源码（如 run_automap.sh 会挂载 `automap_pro/src/modular/fast-livo2-humble` 为容器内 `src/fast_livo`，确保修改在该路径下）。

## 7. 精准定位问题原因（[TRACE]）

所有关键决策与失败分支均输出 **`[TRACE]`** 日志，便于从日志中直接定位“为什么没产生关键帧/回环/HBA 成功”。

### 7.1 标签与含义

| step | result | reason 示例 | 含义 |
|------|--------|-------------|------|
| kf_decision | skip | cloud_empty | 点云为空，未做 KF 决策 |
| kf_decision | skip | shouldCreateKeyFrame_false | 距离/旋转/间隔未达阈值 |
| kf_decision | ok | kf_created | 关键帧已创建 |
| backend_worker | skip | no_odom_in_cache | 缓存无对应位姿，本帧被跳过 |
| backend_worker | fail | exception / unknown_exception | tryCreateKeyFrame 抛异常（见 what=） |
| loop_factor_add | ok / fail | isam2_node_missing_or_exception | 回环因子加入 iSAM2 成功/失败 |
| hba_done | fail | optimization_failed | HBA 优化失败（详见 [HBA][BACKEND]） |
| loop_cand | skip | **no_overlap_candidates** | 描述子检索无候选（db 内无超过 threshold 的相似子图） |
| loop_cand | skip | **all_filtered_by_submap_gap** | 候选均被 min_submap_gap 过滤（同 session 且 id 过近） |
| loop_match | skip | query_cloud_empty | 无查询点云，未做几何验证 |
| loop_match | fail | teaser_exception / teaser_unknown_exception | TEASER/FPFH 抛异常 |
| loop_match | fail | **insufficient_pts** | 预处理后 src 或 tgt 点数 <30 |
| loop_match | fail | **fpfh_src_empty** / **fpfh_tgt_empty** | 源/目标 FPFH 特征为空 |
| loop_match | fail | **insufficient_corrs** | FPFH 对应点数 <10 |
| loop_match | fail | **insufficient_corrs_for_teaser** | 对应点数 <12（TEASER 要求） |
| loop_match | fail | **teaser_solution_invalid** | TEASER 解无效 |
| loop_match | fail | **inlier_ratio_low** | 内点率 < min_inlier_ratio |
| loop_match | fail | **rmse_too_high** | 匹配 RMSE > max_rmse_m |
| loop_cand | skip | target_cloud_empty / teaser_fail_or_inlier_low / rmse_too_high | 某候选被跳过原因 |
| loop_match | ok | loop_accepted | 回环约束已发布 |

### 7.2 如何用 TRACE 定位原因

```bash
# 只看所有决策/失败（精准定位）
grep '\[TRACE\]' run.log

# 只看失败与跳过（定位“为什么没有…”）
grep '\[TRACE\].*result=fail\|result=skip' run.log

# 关键帧：为什么没建 KF
grep '\[TRACE\].*step=kf_decision' run.log

# 回环：为什么没回环 / 为什么回环因子失败
grep '\[TRACE\].*step=loop_match\|step=loop_factor_add' run.log

# Backend：为什么某帧被跳过或抛异常
grep '\[TRACE\].*step=backend_worker' run.log

# 回环：按 reason 统计（定位“帧太少”瓶颈）
grep '\[TRACE\].*step=loop_cand\|step=loop_match' run.log | grep -oE 'reason=[a-z_]+' | sort | uniq -c
```

### 7.3 回环“帧太少”排查与调参

**数据流（漏斗）**：子图冻结 → 描述子计算 → 候选检索 → 过滤 gap → 入队几何验证 → TEASER/FPFH → 接受回环 → 后端因子。

- 若 **reason=no_overlap_candidates** 多：描述子相似度不够。可尝试 **降低** `loop_closure.overlap_threshold`（如 0.25）、**增大** `top_k`（如 8～10）、或检查环境/光照是否变化大。
- 若 **reason=all_filtered_by_submap_gap** 多：候选子图与当前子图 id 太近。可适当 **减小** `loop_closure.min_submap_gap`（如 2），注意过小可能增加误回环。
- 若 **reason=insufficient_pts** / **fpfh_src_empty** / **fpfh_tgt_empty**：预处理后点数不足或特征为空。可 **增大** `loop_closure.teaser.max_points`（默认 8000）、或 **减小** `loop_closure.teaser.voxel_size`（如 0.35）以保留更多点。
- 若 **reason=insufficient_corrs** / **insufficient_corrs_for_teaser**：FPFH 对应点少。同上，增大点云有效点数；或检查场景是否缺乏几何纹理。
- 若 **reason=inlier_ratio_low**：可适当 **降低** `loop_closure.teaser.min_inlier_ratio`（如 0.25），会提高召回但可能增加误匹配。
- 若 **reason=rmse_too_high**：可适当 **增大** `loop_closure.teaser.max_rmse_m`（如 0.4），或启用/加强 ICP 精化。

**增加“参与回环的子图数量”**：在配置中减小 `submap.max_keyframes`（如 50）、`submap.max_spatial_m`（如 50），子图更密，更多子图参与描述子检索与几何验证，后端收到的回环约束会增多。

---

## 8. 回环 FPFH 崩溃追踪日志（CRASH_TRACE）

回环检测中的 FPFH 特征计算使用 `[CRASH_TRACE]` / `[FPFH_CRASH_TRACE]` 精准步骤日志，便于在 SIGSEGV 时定位到具体步骤：

| 步骤 | 含义 |
|------|------|
| STEP=0.0–0.4 | 入参校验、点云深拷贝、feat 预分配 |
| STEP=1.0–1.9 | 加锁、法向量估计（KdTree、setInputCloud、compute） |
| STEP=2.0–2.7 | FPFH 计算（setInput、compute） |
| STEP=3.0–3.1 | 成功返回、清空静态 estimator 引用 |

- 环境变量 `AUTOMAP_FPFH_BACKTRACE=1` 时，在关键步骤额外打印 backtrace。
- 若在 FPFH 计算内发生 SIGSEGV，会打印 demangle 后的栈并 `_exit(139)`。

## 9. 变更文件清单（建图精度日志）

- `automap_pro/include/automap_pro/core/precision_logger.h`：**新增** 精度标签常量（可选引用）
- `automap_pro/include/automap_pro/core/config_manager.h`：回环 `teaserMaxPoints()`（对应 `loop_closure.teaser.max_points`，默认 8000，用于增加参与匹配的点数、缓解“帧太少”）
- `automap_pro/src/modular/fast-livo2-humble/include/voxel_map.h`：新增 `last_mean_residual_`
- `automap_pro/src/modular/fast-livo2-humble/src/voxel_map.cpp`：在 StateEstimation 中更新 `last_mean_residual_`
- `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp`：每 **100** 帧 + 每 3000 帧输出 `[PRECISION][LIO]`
- `automap_pro/src/system/automap_system.cpp`：`[PRECISION][ODOM]`（onOdometry）、`[PRECISION][KF]`、`[PRECISION][SUBMAP]`（onSubmapFrozen）、`[PRECISION][LOOP]`、`[PRECISION][GPS]`、`[PRECISION][HBA]`
- `automap_pro/src/submap/submap_manager.cpp`：创建新子图时输出 `[PRECISION][SUBMAP]`
- `automap_pro/src/loop_closure/loop_detector.cpp`：接受回环时输出 `[PRECISION][LOOP]`（detected）；候选检索无结果/全被 gap 过滤时输出 `[TRACE]`（reason=no_overlap_candidates/all_filtered_by_submap_gap）；match 任务与候选跳过/接受增加 `[TRACE]`（query_cloud_empty、teaser_exception、target_cloud_empty、teaser_fail_or_inlier_low、rmse_too_high、loop_accepted 等）
- `automap_pro/src/loop_closure/teaser_matcher.cpp`：几何验证各失败分支统一输出 `[TRACE]`（reason=insufficient_pts、fpfh_src_empty、fpfh_tgt_empty、insufficient_corrs、insufficient_corrs_for_teaser、teaser_solution_invalid、inlier_ratio_low、rmse_too_high），便于精准定位“后端回环帧太少”的环节
- `automap_pro/src/backend/incremental_optimizer.cpp`：每次 iSAM2 update 输出 `[PRECISION][OPT]`
- `automap_pro/src/system/automap_system.cpp`：KF/backend/loop/HBA 关键分支增加 `[TRACE]`（step/result/reason），便于精准定位问题原因
