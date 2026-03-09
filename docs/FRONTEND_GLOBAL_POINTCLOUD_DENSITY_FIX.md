# 前端全局点云过稠/不清晰：分析与修复

## Executive Summary

- **现象**：`automap_frontend.rviz` 中前端全局点云（/Laser_map 与 /automap/global_map）显得过稠、不够清晰。
- **根因**：  
  1. **/Laser_map**：fast_livo 每帧将当前帧世界系点云**无体素去重**地累加到 `pcl_laser_map_accum_` 并发布，且 `dense_map_en=true` 时使用未下采样的 `feats_undistort`，导致点数爆炸、同一区域多重叠加，视觉上“糊”。  
  2. **RViz**：点尺寸偏小、叠加过密时难以分辨结构。
- **修复**：  
  1. 为 /Laser_map 增加**发布前体素下采样**（可配置 `publish.laser_map_voxel_size`），发布后并用下采样结果替换内存中的累积云，控制密度与内存。  
  2. 优化 RViz 中点云显示（点大小、透明度），使结构更易辨认。

---

## 1. 背景与目标

- **前端 RViz**（`automap_frontend.rviz`）主要显示：
  - **frontend map**：话题 `/Laser_map`，由 fast_livo 发布的**前端累积地图**；
  - **front LIO cloud**：`/cloud_registered`，当前帧 LIO 点云；
  - **backend global map**：`/automap/global_map`，后端全局图（已有体素下采样）。
- 用户反馈：**前端的全局点云**（尤指 /Laser_map）过稠、点云不够清晰。

---

## 2. 逐行分析

### 2.1 前端累积地图 /Laser_map 的数据流（LIVMapper.cpp）

| 行号/位置 | 代码/逻辑 | 说明 |
|----------|-----------|------|
| 641 | `PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);` | `dense_map_en=true` 时使用**未下采样**的 `feats_undistort`，单帧点数极大。 |
| 643-649 | 将 `laserCloudFullRes` 转到世界系 → `laserCloudWorld`，再 `*pcl_w_wait_pub = *laserCloudWorld` | 每帧世界系点云进入待发布/累积缓冲。 |
| 662-663 | `*pcl_laser_map_accum_ += *pcl_w_wait_pub;` | **无任何体素去重**，直接累加；随运行时间累积点数线性甚至超线性增长。 |
| 664-669 | 每 `laser_map_pub_interval_` 帧发布一次：`pcl::toROSMsg(*pcl_laser_map_accum_, ...)` | 发布的是**原始累积云**，无下采样 → RViz 收到的是极稠点云。 |

**结论**：/Laser_map 的“过稠、不清晰”来自：  
① 累积无体素去重；② `dense_map_en=true` 时单帧即为稠密点云；③ 发布前未做降采样。

### 2.2 后端全局图 /automap/global_map

- 由 `SubMapManager::buildGlobalMap(voxel_size)` 生成，内部使用 `utils::voxelDownsampleChunked(combined, vs, 50.0f)` 做体素下采样，密度由 `map.voxel_size`（如 0.2）控制，**本身较清晰**。  
- 若仍觉糊，多为 RViz 点尺寸或多图层叠加导致，本次通过调整 RViz 显示参数改善。

### 2.3 RViz 配置（automap_frontend.rviz）

| 显示项 | 原 Size (m) | 原 Alpha | 说明 |
|--------|-------------|----------|------|
| frontend map | 0.04 | 0.9 | 点过小且叠加过密时难以分辨结构。 |
| backend global map | 0.04 | 0.85 | 同上。 |

---

## 3. 方案与取舍

- **方案 A**：仅在 RViz 中调大点尺寸、调透明度。  
  - 优点：不改代码。  
  - 缺点：/Laser_map 数据量不变，仍极稠，大点会严重重叠，仍糊。
- **方案 B**：在 fast_livo 发布 /Laser_map 前对累积云做体素下采样，并可选地用下采样结果替换内存中的累积（控制内存与密度）。  
  - 优点：从数据源降低密度，清晰度与性能兼得；可配置、可回退（`laser_map_voxel_size=0`）。  
  - 缺点：需改 LIVMapper 与配置。  
- **采用**：**B + RViz 微调**（适度增大点尺寸、略调 Alpha），兼顾数据侧与显示侧。

---

## 4. 变更清单

| 文件 | 变更概要 |
|------|----------|
| `automap_pro/src/modular/fast-livo2-humble/include/LIVMapper.h` | 新增成员 `laser_map_voxel_size_`，默认 0.2。 |
| `automap_pro/src/modular/fast-livo2-humble/src/LIVMapper.cpp` | 声明并读取参数 `publish.laser_map_voxel_size`；在发布 /Laser_map 前对 `pcl_laser_map_accum_` 做体素下采样，发布后用下采样结果替换累积；异常时回退为发布原累积云。 |
| `automap_pro/rviz/automap_frontend.rviz` | frontend map / backend global map：Size (m) 0.04→0.06，Alpha 微调，注释更新。 |
| `automap_pro/config/system_config_M2DGR.yaml` | 在 `fast_livo.publish` 下增加 `laser_map_interval`、`laser_map_voxel_size`。 |
| `automap_pro/config/system_config.yaml` | 同上。 |

---

## 5. 编译 / 部署 / 运行说明

- **编译**：在工程内执行原有编译流程（如 `run_automap.sh --build-only` 或容器内 `colcon build --packages-select automap_pro`）。
- **配置**：  
  - `fast_livo.publish.laser_map_voxel_size`：发布前体素边长(m)。**0.2** 为推荐默认；设为 **0** 则关闭下采样（恢复原稠密行为）。  
  - `fast_livo.publish.laser_map_interval`：每 N 帧发布一次 /Laser_map（已有则保持即可）。
- **验证**：  
  1. 启动前端 + RViz（`automap_frontend.rviz`），订阅 `/Laser_map`。  
  2. 观察累积地图：应明显比修复前稀疏、结构更清晰；若仍偏密可适当增大 `laser_map_voxel_size`（如 0.25）。  
  3. 将 `laser_map_voxel_size` 设为 0，确认行为与修复前一致（更稠）。

---

## 6. 风险与回滚

- **风险**：体素过大可能略损细节；PCL VoxelGrid 在极大范围点云下可能溢出（已加 try-catch，失败时回退为发布原累积云）。  
- **回滚**：  
  - 代码：还原 LIVMapper 发布段与头文件、配置中的新增参数。  
  - 行为：配置 `laser_map_voxel_size: 0` 即关闭发布前下采样，恢复原逻辑。

---

## 7. 后续可选

- 若需与后端 `map.voxel_size` 统一观感，可将 `laser_map_voxel_size` 与 `map.voxel_size` 同源配置或文档说明对齐。  
- 可增加 ROS2 动态参数，运行时调节 `laser_map_voxel_size` 与 `laser_map_interval`，便于调试。
