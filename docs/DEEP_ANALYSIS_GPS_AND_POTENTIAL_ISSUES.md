# 深度分析：GPS/坐标与潜在问题

本文档汇总对 GPS 坐标语义、后端/HBA、线程安全与一致性所做的排查结论，以及已修复与待关注项。

---

## 一、已修复的严重问题

### 1. position_enu 双重变换（导致 accuracy 曲线偏差 ~10m）

- **现象**：`accuracy_curves.png` 中 GPS 与 HBA 优化轨迹偏差很大（mean ~10.67 m）。
- **根因**：对齐后 `queryByTimestamp` / `queryByTimestampForLog` 把 **map 系** 写入 `GPSMeasurement.position_enu`，而 HBA 与精度计算处仍按 ENU 再做一次 `enu_to_map`，导致 **双重变换**。
- **修复**：
  - `gps_manager.cpp`：query 接口 **始终** 在 `position_enu` 中返回 **ENU**（插值/最近邻/ForLog/外推均统一）。
  - 外推路径 `queryByTimestampEnhanced` 中，map 系外推结果经 `map_to_enu()` 转回 ENU 再写入 `position_enu`。
  - 轨迹 GPS 写文件处改为使用 `enu_to_map_with_frame(m.position_enu)` 写盘，保证写出坐标与 frame 标签一致。

### 2. align_result_ 并发读写竞态

- **问题**：`onlineCalibrate()` 无锁写 `align_result_.t_gps_lidar`，与 `enu_to_map` / `map_to_enu` / `enu_to_map_with_frame` 的无锁读可能产生数据竞争。
- **修复**：
  - 在 `enu_to_map`、`map_to_enu`、`enu_to_map_with_frame` 中，当 `state_` 为 ALIGNED/DEGRADED 时，**先加锁再读** `align_result_`。
  - 在 `onlineCalibrate()` 中更新 `align_result_` 时 **持 mutex_** 再写。

---

## 二、潜在问题与建议

### 1. HBA 中 GPS 协方差未随坐标系旋转（中等）

- **位置**：`hba_optimizer.cpp` 使用 `kf->gps.covariance` 作为对角方差直接传给 GTSAM GPS 因子。
- **说明**：该协方差在 keyframe 绑定时按 ENU 语义构造（如 HDOP 等），而 HBA 中位置已转换到 map 系。严格做法应对协方差做 **C_map = R * C_enu * R^T**。
- **影响**：旋转以绕 Z 为主且角度不大时，对角近似误差有限；大旋转或需更严不确定性时建议在 HBA 内对协方差做旋转变换。

### 2. GPSFusion::processForFrontend 的坐标系假设（低～中）

- **位置**：`gps_fusion.cpp` 中 `delta = meas.position_enu - current_pose.translation()`，用于 Mahalanobis 检验。
- **说明**：要求 `meas.position_enu` 与 `current_pose.translation()` **同系**（均为 ENU 或均为 map）。当前 `position_enu` 已统一为 ENU；若前端传入的 `current_pose` 为 map 系（例如对齐后），则 delta 混系，检验失效。
- **建议**：若启用 GPS 融合前端，需明确约定 `current_pose` 为 ENU 或在对齐后先 `map_to_enu` 再传入，并在注释/文档中写明。

### 3. onlineCalibrate 的 API 与当前无人调用（低）

- **位置**：`gps_manager.cpp::onlineCalibrate(gps_ts, gps_enu, odom_ts, odom_pose)`。
- **说明**：当前代码库中 **无调用点**。实现已改为要求 `gps_enu` 与 `odom_pose.translation()` 均为 **ENU**，同系求差；并已对 `align_result_` 的写加锁。
- **建议**：若将来接入在线校准，调用方必须保证两路均为 ENU（或文档明确约定 odom 为 map 时先转 ENU 再传）。

### 4. 子图 gps_center 的持久化与向后兼容（低）

- **说明**：`gps_center` 现为 keyframe `position_enu`（ENU）的平均，即 **ENU**。保存/加载子图或 session 时会持久化 `gps_center`。
- **风险**：在修复「position_enu 恒为 ENU」之前保存的 session，其 `gps_center` 可能是“旧语义”（曾混入 map 或混合），加载后若直接当 ENU 用（如 `enu_to_map(sm->gps_center)`）会错。
- **建议**：若需兼容旧数据，可在 meta 中增加版本或 frame 标记，加载时按版本决定是否对 `gps_center` 做一次转换或忽略。

### 5. R_lidar_gps() / t_lidar_gps() 返回 const 引用（低）

- **位置**：`gps_manager.h` 中返回 `align_result_.R_gps_lidar` / `t_gps_lidar` 的 const 引用。
- **说明**：若在 `onlineCalibrate` 持锁写 `align_result_` 的同时，其他线程通过这两接口读，仍可能读到不一致状态（读未加锁）。
- **建议**：对对齐结果需要“快照”的调用方，可改为在 GPSManager 内提供“在锁内拷贝一份 align_result_ 并返回”的接口，避免长时间持锁但保证一致性。

---

## 三、已确认无问题的路径

- **submap gps_center**：由 keyframe 的 `position_enu`（ENU）求平均，使用处 `enu_to_map(sm->gps_center)` 只做一次转换，正确。
- **闭环检索**：`query_gps_pos` 与 `sm->gps_center` 均为子图 gps_center，同系（ENU）求距离，一致。
- **gps_processing 对齐后加因子**：`pos_enu = gps.position_enu`（ENU），`pos_map = R*pos_enu + t`，正确。
- **delayed_gps_compensator**：`enuToMap(gps_meas->position_enu)` 期望 ENU 输入，与当前 query 语义一致。
- **getGpsPositionsInMapFrame()**：在锁内遍历窗口并对 `r.pos_enu` 做 `enu_to_map`，返回 map 系，正确。
- **DelayedGPSCompensator::enuToMap**：内部用 mutex 拷贝 `align_result_` 再计算，无竞态。

---

## 四、小结

- **严重问题**：position_enu 双重变换、align_result_ 并发读写已按上述方式修复。
- **潜在问题**：HBA 协方差未旋转、GPSFusion 同系假设、onlineCalibrate 无人调用与 API 约定、gps_center 持久化兼容性、R/t  getter 无锁读，已记录为改进项，可按优先级逐步处理。
- 建议用同一 bag 与配置 **重新跑建图并生成 accuracy 曲线**，验证偏差是否降至合理范围（如亚米级）。
