# AutoMap-Pro: High-Precision Automated Point Cloud Mapping System

> **仓库根目录推荐入口**：在工程根目录执行 `bash automap_start.sh` 可一键完成 Docker 内编译与建图运行。本文件描述 automap_pro 包本身的结构与本地构建/运行方式。

## System Overview

AutoMap-Pro is an end-to-end automated 3D point cloud mapping system designed for urban roads, campuses, tunnels, and mines. It achieves:

- **Global accuracy** < 0.3% (relative to trajectory length)
- **Frontend** ≥ 10 Hz real-time LiDAR-IMU-(Visual) state estimation
- **Incremental** multi-session mapping with cross-session loop closure
- **Robustness** to intermittent GPS, tunnels, and degenerate scenes

## Architecture

```
LiDAR + IMU + GPS + Camera
          │
    [Layer 1] Sensor Preprocessing (time sync, undistortion, WGS84→ENU)
          │
    [Layer 2] Fast-LIVO2 ESIKF Frontend (Thread 1)
          │
    [Layer 3] MS-Mapping Submap Management (Thread 2)
          │
    ┌─────┴───────┐
    │             │
[Layer 4a]    [Layer 4b]
OverlapTransf  Pose Graph
+ TEASER++
(Thread 3)
    │
    └─────┬───────┘
          │
    [Layer 6] HBA Global Optimization (Thread 4)
          │
    [Layer 7] Map Generation & Export (Thread 5)
```

## Quick Start

### Prerequisites
- Ubuntu 22.04 + **ROS2 Humble**
- CUDA ≥ 11.3 (for GPU features)
- GTSAM ≥ 4.1, PCL ≥ 1.10, OpenCV ≥ 4.2, Eigen ≥ 3.3

### Build
```bash
# Setup workspace (creates ~/automap_ws, links this package, rosdep install)
make setup

# Build (colcon)
make build-release

# Run tests
make test
```

### Run Online Mapping
```bash
# Source workspace then start AutoMap-Pro (ROS2 launch)
make run-online

# Offline with rosbag2 (path to bag directory or .db3)
make run-offline BAG_FILE=/path/to/mapping
```

### Service Commands
```bash
# Check system status
make status

# Trigger global optimization
make trigger-opt

# Save map to disk
make save-map OUTPUT_DIR=/data/output
```

### Docker
```bash
# Build image
make docker-build

# Run in container
make docker-run
```

## Configuration

All parameters are in `config/system_config.yaml`. Key sections:

| Section | Description |
|---------|-------------|
| `sensor` | LiDAR/IMU/GPS/camera topics and rates |
| `gps_fusion` | Quality thresholds and covariance settings |
| `frontend` | Keyframe policy, ESIKF parameters |
| `submap` | Submap split policy (max keyframes, spatial/temporal extent) |
| `loop_closure` | OverlapTransformer + TEASER++ parameters |
| `backend` | HBA optimization settings |
| `map_output` | Voxel size, tiling, export formats |

## Output Structure

```
output_dir/
├── trajectory/
│   ├── optimized_trajectory_tum.txt
│   ├── optimized_trajectory_kitti.txt
│   └── keyframe_poses.json
├── map/
│   ├── global_map.pcd
│   ├── global_map.ply
│   └── tiles/
├── submaps/
├── loop_closures/loop_report.json
├── pose_graph/pose_graph.g2o
└── descriptor_db.json
```

## Evaluation
```bash
# Trajectory evaluation (requires evo)
make eval-traj EST=output/trajectory/optimized_trajectory_tum.txt REF=groundtruth.txt

# Map quality evaluation
make eval-map MAP=output/map/global_map.pcd

# Visualize results
make visualize DIR=output/
```

## Module Summary

| Module | File | Role |
|--------|------|------|
| ESIKF Frontend | `src/frontend/fast_livo2_wrapper.cpp` | LiDAR-IMU tightly-coupled odometry |
| GPS Fusion | `src/frontend/gps_fusion.cpp` | Adaptive GPS observation injection |
| Keyframe Manager | `src/frontend/keyframe_manager.cpp` | Keyframe selection |
| Submap Manager | `src/submap/submap_manager.cpp` | Submap lifecycle management |
| Session Manager | `src/submap/session_manager.cpp` | Multi-session data |
| OverlapTransformer | `src/loop_closure/overlap_transformer.cpp` | Loop place recognition |
| FPFH Extractor | `src/loop_closure/fpfh_extractor.cpp` | Feature extraction |
| TEASER Matcher | `src/loop_closure/teaser_matcher.cpp` | Robust registration |
| ICP Refiner | `src/loop_closure/icp_refiner.cpp` | Fine alignment |
| Loop Detector | `src/loop_closure/loop_detector.cpp` | Pipeline orchestration |
| HBA Wrapper | `src/backend/hba_wrapper.cpp` | Hierarchical optimization |
| Pose Graph | `src/backend/pose_graph.cpp` | Factor graph |
| Optimizer | `src/backend/optimizer.cpp` | GTSAM / GN solver |
| Map Builder | `src/map/map_builder.cpp` | Point cloud assembly |
| Map Filter | `src/map/map_filter.cpp` | Voxel + statistical filter |
| Map Exporter | `src/map/map_exporter.cpp` | PCD/PLY/LAS/tiles export |
| RViz Publisher | `src/visualization/rviz_publisher.cpp` | Visualization |
