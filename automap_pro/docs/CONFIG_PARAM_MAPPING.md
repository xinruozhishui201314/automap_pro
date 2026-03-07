# 配置文件与读取代码映射表

本文档保证 `system_config*.yaml` 与 C++/Python 读取代码**完全匹配**，避免键名或路径不一致导致读取错误。

---

## 1. 读取方与配置文件

| 读取方 | 配置文件 | 说明 |
|--------|----------|------|
| **ConfigManager** (C++) | `config_file` 参数指向的 YAML（通常为 system_config.yaml / system_config_M2DGR.yaml） | 点号路径，如 `sensor.lidar.topic` |
| **params_from_system_config.py** | 同上，由 launch 传入路径 | 字典 `config.get("sensor")` 等 |
| **fastlivo_mapping** 节点 | 不直接读 YAML，由 launch 将 `get_fast_livo2_params(config)` 生成的字典转为 ROS2 params-file | 键来自 sensor + fast_livo 合并 |

---

## 2. ConfigManager（C++）读取的键与 YAML 路径

以下键均在 `include/automap_pro/core/config_manager.h` 与 `src/core/config_manager.cpp` 中使用，**必须与 YAML 顶层节与子键一致**。

### 2.1 system

| 代码键（点号路径） | 类型 | 默认值 | YAML 位置 | 备注 |
|-------------------|------|--------|-----------|------|
| `system.name` | string | "AutoMap-Pro" | `system.name` | |
| `system.output_dir` | string | "/data/automap_output" | `system.output_dir` | |
| `system.num_threads` | int | 8 | `system.num_threads` | |

### 2.2 sensor

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `sensor.lidar.topic` | string | "/os1_cloud_node1/points" | `sensor.lidar.topic` |
| `sensor.imu.topic` | string | "/imu/imu" | `sensor.imu.topic` |
| `sensor.gps.topic` | string | "/gps/fix" | `sensor.gps.topic` |
| `sensor.gps.enabled` | bool | false | `sensor.gps.enabled` |
| `sensor.camera.enabled` | bool | false | `sensor.camera.enabled` |
| `sensor.camera.topic` | string | "/camera/image_raw" | `sensor.camera.topic` |

### 2.3 frontend

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `frontend.odom_topic` | string | "/aft_mapped_to_init" | `frontend.odom_topic` |
| `frontend.cloud_topic` | string | "/cloud_registered" | `frontend.cloud_topic` |
| `frontend.kf_info_topic` | string | "/fast_livo/keyframe_info" | `frontend.kf_info_topic` |
| `frontend.use_composable_node` | bool | true | `frontend.use_composable_node` |

### 2.4 keyframe

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `keyframe.min_translation` | double | 1.0 | `keyframe.min_translation` |
| `keyframe.min_rotation_deg` | double | 10.0 | `keyframe.min_rotation_deg` |
| `keyframe.max_interval` | double | 2.0 | `keyframe.max_interval` |
| `keyframe.max_esikf_cov_norm` | double | 1000.0 | `keyframe.max_esikf_cov_norm` |

### 2.5 submap

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `submap.max_keyframes` | int | 100 | `submap.max_keyframes` |
| `submap.max_spatial_m` | double | 100.0 | `submap.max_spatial_m` |
| `submap.max_temporal_s` | double | 60.0 | `submap.max_temporal_s` |
| `submap.match_resolution` | double | 0.4 | `submap.match_resolution` |
| `submap.merge_resolution` | double | 0.1 | `submap.merge_resolution` |

### 2.6 gps（延迟对齐 + 可选质量/协方差）

| 代码键 | 类型 | 默认值 | YAML 位置 | 备注 |
|--------|------|--------|-----------|------|
| `gps.align_min_points` | int | 50 | `gps.align_min_points` | |
| `gps.align_min_distance_m` | double | 30.0 | `gps.align_min_distance_m` | |
| `gps.quality_threshold_hdop` | double | 2.0 | `gps.quality_threshold_hdop` | |
| `gps.align_rmse_threshold_m` | double | 1.5 | `gps.align_rmse_threshold_m` | |
| `gps.good_samples_needed` | int | 30 | `gps.good_samples_needed` | |
| `gps.add_constraints_on_align` | bool | true | `gps.add_constraints_on_align` | |
| `gps.factor_interval_m` | double | 5.0 | `gps.factor_interval_m` | |
| `gps.quality_excellent_hdop` | double | 0.8 | 可选 | 缺省用默认值 |
| `gps.quality_high_hdop` | double | 1.5 | 可选 | |
| `gps.quality_medium_hdop` | double | 3.0 | 可选 | |
| `gps.max_jump_m` | double | 10.0 | 可选 | |
| `gps.max_velocity_ms` | double | 30.0 | 可选 | |
| `gps.chi2_threshold` | double | 7.815 | 可选 | |
| `gps.consecutive_valid_required` | int | 3 | 可选 | |
| `gps.jump_detection_enabled` | bool | true | 可选 | |
| `gps.consistency_check_enabled` | bool | true | 可选 | |
| `gps.cov_excellent` | [x,y,z] | [0.5,0.5,1.0] | 可选，3 元素数组 | config_manager.cpp readVector3d |
| `gps.cov_high` | [x,y,z] | [1,1,2] | 可选 | |
| `gps.cov_medium` | [x,y,z] | [2,2,4] | 可选 | |
| `gps.cov_low` | [x,y,z] | [4,4,8] | 可选 | |

### 2.7 session

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `session.multi_session` | bool | false | `session.multi_session` |
| `session.session_dir` | string | "" | `session.session_dir` |
| `session.previous_session_dirs` | 字符串数组 | [] | `session.previous_session_dirs` |

### 2.8 loop_closure

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `loop_closure.overlap_threshold` | double | 0.3 | `loop_closure.overlap_threshold` |
| `loop_closure.top_k` | int | 5 | `loop_closure.top_k` |
| `loop_closure.min_temporal_gap_s` | double | 30.0 | `loop_closure.min_temporal_gap_s` |
| `loop_closure.min_submap_gap` | int | 3 | `loop_closure.min_submap_gap` |
| `loop_closure.gps_search_radius_m` | double | 200.0 | `loop_closure.gps_search_radius_m` |
| `loop_closure.worker_threads` | int | 2 | `loop_closure.worker_threads` |
| `loop_closure.overlap_transformer.model_path` | string | "" | `loop_closure.overlap_transformer.model_path` |
| `loop_closure.overlap_transformer.proj_H` | int | 64 | `loop_closure.overlap_transformer.proj_H` |
| `loop_closure.overlap_transformer.proj_W` | int | 900 | `loop_closure.overlap_transformer.proj_W` |
| `loop_closure.overlap_transformer.fov_up` | float | 3.0 | `loop_closure.overlap_transformer.fov_up` |
| `loop_closure.overlap_transformer.fov_down` | float | -25.0 | `loop_closure.overlap_transformer.fov_down` |
| `loop_closure.overlap_transformer.max_range` | float | 50.0 | `loop_closure.overlap_transformer.max_range` |
| `loop_closure.overlap_transformer.descriptor_dim` | int | 256 | `loop_closure.overlap_transformer.descriptor_dim` |
| `loop_closure.teaser.noise_bound` | double | 0.1 | `loop_closure.teaser.noise_bound` |
| `loop_closure.teaser.cbar2` | double | 1.0 | `loop_closure.teaser.cbar2` |
| `loop_closure.teaser.voxel_size` | double | 0.4 | `loop_closure.teaser.voxel_size` |
| `loop_closure.teaser.min_inlier_ratio` | double | 0.30 | `loop_closure.teaser.min_inlier_ratio` |
| `loop_closure.teaser.max_rmse_m` | double | 0.3 | `loop_closure.teaser.max_rmse_m` |
| `loop_closure.teaser.icp_refine` | bool | true | `loop_closure.teaser.icp_refine` |

### 2.9 backend

| 代码键 | 类型 | 默认值 | YAML 位置 |
|--------|------|--------|------------|
| `backend.isam2.relinearize_threshold` | double | 0.01 | `backend.isam2.relinearize_threshold` |
| `backend.isam2.relinearize_skip` | int | 1 | `backend.isam2.relinearize_skip` |
| `backend.isam2.enable_relinearization` | bool | true | `backend.isam2.enable_relinearization` |
| `backend.hba.total_layer_num` | int | 3 | `backend.hba.total_layer_num` |
| `backend.hba.thread_num` | int | 8 | `backend.hba.thread_num` |
| `backend.hba.trigger_every_n_submaps` | int | 10 | `backend.hba.trigger_every_n_submaps` |
| `backend.hba.trigger_on_loop` | bool | false | `backend.hba.trigger_on_loop` |
| `backend.hba.trigger_on_finish` | bool | true | `backend.hba.trigger_on_finish` |
| `backend.hba.data_path` | string | "/tmp/hba_data" | `backend.hba.data_path` |

### 2.10 map

| 代码键 | 类型 | 默认值 | YAML 位置 | 备注 |
|--------|------|--------|-----------|------|
| `map.voxel_size` | double | 0.1 | `map.voxel_size` | |
| `map.statistical_filter` | bool | true | `map.statistical_filter` | |
| `map.statistical_filter_mean_k` | int | 50 | 可选 | 缺省 50 |
| `map.statistical_filter_std_mul` | double | 1.0 | 可选 | 缺省 1.0 |

---

## 3. params_from_system_config.py 读取的键

### 3.1 get_overlap_transformer_params

从 `config["loop_closure"]["overlap_transformer"]` 与可选的 `range_image` 读取：

| Python 键 | 默认值 | YAML 路径 |
|-----------|--------|-----------|
| model_path | "" | loop_closure.overlap_transformer.model_path |
| proj_H | 64 或 range_image.height | loop_closure.overlap_transformer.proj_H 或 .range_image.height |
| proj_W | 900 或 range_image.width | .proj_W 或 .range_image.width |
| fov_up, fov_down, max_range | 3.0, -25.0, 80.0 | .fov_up, .fov_down, .max_range |

### 3.2 get_hba_params

从 `config["backend"]["hba"]` 读取，**必须在 YAML 中提供或使用默认值**：

| Python 键 | 默认值 | YAML 路径 |
|-----------|--------|-----------|
| data_path | "/data/automap_output/hba_export" | backend.hba.data_path |
| total_layer_num | 3 | backend.hba.total_layer_num |
| **pcd_name_fill_num** | **0** | **backend.hba.pcd_name_fill_num**（原 YAML 缺失，需补） |
| thread_num | 16 | backend.hba.thread_num |
| **enable_gps_factor** | **True** | **backend.hba.enable_gps_factor**（原 YAML 缺失，需补） |

### 3.3 get_hba_cal_mme_params

从 `config["backend"]["hba_cal_mme"]` 读取：

| Python 键 | 默认值 | YAML 路径 |
|-----------|--------|-----------|
| file_path | "/data/automap_output/hba_export" | backend.hba_cal_mme.file_path |
| THR_NUM (来自 thr_num) | 16 | backend.hba_cal_mme.thr_num |

若 YAML 无 `backend.hba_cal_mme`，则全部用默认值。

### 3.4 get_hba_visualize_params

从 `config["backend"]["hba_visualize"]` 读取：

| Python 键 | 默认值 | YAML 路径 |
|-----------|--------|-----------|
| file_path | "/data/automap_output/hba_export" | backend.hba_visualize.file_path |
| downsample_size | 0.1 | backend.hba_visualize.downsample_size |
| pcd_name_fill_num | 0 | backend.hba_visualize.pcd_name_fill_num |
| marker_size | 0.5 | backend.hba_visualize.marker_size |

若 YAML 无 `backend.hba_visualize`，则全部用默认值。

### 3.5 get_fast_livo2_params

- **话题**：`lid_topic` / `imu_topic` / `img_topic` 唯一来自 `config["sensor"]["lidar|imu|camera"]["topic"]`，不由 `fast_livo.common` 定义。
- **fast_livo 节**：优先 `config["frontend"]["fast_livo2"]`，若无则 `config["fast_livo"]`。从 fl2 拷贝的 section：`common`（仅行为开关）、`extrin_calib`、`time_offset`、`preprocess`、`vio`、`imu`、`lio`、`local_map`、`uav`、`publish`、`evo`、`pcd_save`、`image_save`（可选）。相机内参来自 `fl2.camera_calib` 或 `sensor.camera_left` 生成 `parameter_blackboard`。

---

## 4. 必须补全的 YAML 键（与代码完全匹配）

为避免 Python 使用与 C++/文档不一致的默认值，建议在 **system_config_M2DGR.yaml** 与 **system_config.yaml** 中补全：

1. **backend.hba**
   - `pcd_name_fill_num: 0`
   - `enable_gps_factor: true`（或按场景设为 false）

2. **backend.hba_cal_mme**（可选，launch 启用 cal_MME 时有用）
   - `file_path: "/tmp/hba_data"`（或与 hba.data_path 一致）
   - `thr_num: 16`

3. **backend.hba_visualize**（可选，launch 启用 visualize 时有用）
   - `file_path: "/tmp/hba_data"`
   - `downsample_size: 0.1`
   - `pcd_name_fill_num: 0`
   - `marker_size: 0.5`

补全后，Python 与 C++ 从同一份 YAML 读到的值一致，便于排查与文档化。
