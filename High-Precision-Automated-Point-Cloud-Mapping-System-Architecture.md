# High-Precision High-Performance Automated Point Cloud Mapping System Architecture

## Version Information

| Item           | Content                                                |
| -------------- | ------------------------------------------------------ |
| Document Ver.  | v1.0                                                   |
| Created        | 2025-01-XX                                             |
| System Name    | AutoMap-Pro Automated High-Precision Point Cloud Mapping System |
| Target Platform| Ubuntu 20.04 / ROS Noetic                              |
| Hardware       | x86_64, ≥32GB RAM, NVIDIA GPU (≥RTX 3060)              |

---

## Table of Contents

- [1. System Overview](#1-system-overview)
  - [1.1 System Objectives](#11-system-objectives)
  - [1.2 Core Modules Overview](#12-core-modules-overview)
  - [1.3 Inputs and Outputs](#13-inputs-and-outputs)
- [2. System Architecture](#2-system-architecture)
  - [2.1 Architecture Overview](#21-architecture-overview)
  - [2.2 Data Flow Overview](#22-data-flow-overview)
  - [2.3 Threading Model](#23-threading-model)
- [3. Module Design](#3-module-design)
  - [3.1 Sensor Ingestion and Preprocessing](#31-sensor-ingestion-and-preprocessing-module)
  - [3.2 Frontend Odometry — Fast-LIVO2](#32-frontend-odometry-module--fast-livo2)
  - [3.3 GPS Fusion Module](#33-gps-fusion-module)
  - [3.4 Incremental Submap Management — MS-Mapping](#34-incremental-submap-management--ms-mapping)
  - [3.5 Loop Closure Coarse Matching — OverlapTransformer](#35-loop-closure-coarse-matching--overlaptransformer)
  - [3.6 Loop Closure Fine Matching — TEASER++](#36-loop-closure-fine-matching--teaser)
  - [3.7 Global Consistency Optimization — HBA](#37-global-consistency-optimization--hba)
  - [3.8 Global Map Output Module](#38-global-map-output-module)
- [4. Core Data Structures](#4-core-data-structures)
  - [4.1 Keyframe](#41-keyframe)
  - [4.2 Submap](#42-submap)
  - [4.3 Pose Graph](#43-pose-graph)
  - [4.4 Loop Closure Constraints](#44-loop-closure-constraints)
- [5. System Runtime Flow](#5-system-runtime-flow)
  - [5.1 Online Mapping Main Flow](#51-online-mapping-main-flow)
  - [5.2 Loop Detection and Optimization Flow](#52-loop-detection-and-optimization-flow)
  - [5.3 Incremental Mapping Flow](#53-incremental-mapping-flow)
- [6. GPS Processing Strategy](#6-gps-processing-strategy)
- [7. Project Directory Structure](#7-project-directory-structure)
- [8. ROS Nodes and Topics](#8-ros-nodes-and-topics-design)
- [9. Performance Metrics and Optimization](#9-performance-metrics-and-optimization-strategy)
- [10. Build and Deployment](#10-build-and-deployment)
- [11. Testing and Validation](#11-testing-and-validation)
- [12. Future Extensions](#12-future-extensions)
- [Appendix A: Dependencies](#appendix-a-dependencies)
- [Appendix B: Configuration Examples](#appendix-b-configuration-examples)

---

## 1. System Overview

### 1.1 System Objectives

Build an **automated high-precision 3D point cloud mapping system** for urban roads, campuses, tunnels, mines, and other complex scenes, meeting these core requirements:

- **High precision**: Global consistency error < 0.3% (relative to trajectory length)
- **High robustness**: Handles intermittent GPS and degenerate scenes (long corridors, tunnels)
- **High performance**: Frontend real-time ≥ 10 Hz, full pipeline runnable online
- **Incremental**: Supports incremental stitching of multiple acquisition sessions
- **Automation**: End-to-end mapping without manual intervention

### 1.2 Core Modules Overview

| Module                     | Technology              | Role                                              |
| -------------------------- | ----------------------- | ------------------------------------------------- |
| Frontend Odometry          | **Fast-LIVO2**          | LiDAR-IMU-(Visual) tightly-coupled state estimation |
| GPS Fusion                 | Adaptive factor-graph fusion | Adaptive fusion for weak/strong GPS              |
| Incremental Submap Mgmt    | **MS-Mapping**          | Multi-session incremental mapping, submap management |
| Loop Coarse Matching       | **OverlapTransformer**  | Deep-learning-based place recognition            |
| Loop Fine Matching         | **TEASER++**            | Robust point cloud registration                   |
| Global Consistency Opt.    | **HBA**                 | Hierarchical pose graph + BA                      |
| Map Output                 | In-house                | Multi-format high-precision map export            |

### 1.3 Inputs and Outputs

#### Inputs

| Source   | Rate          | Required/Optional | Description                          |
| -------- | ------------- | ----------------- | ------------------------------------ |
| LiDAR    | 10–20 Hz      | **Required**      | Primary geometry source              |
| IMU      | 200–400 Hz    | **Required**      | High-rate pose propagation           |
| GPS/GNSS | 1–10 Hz       | **Required**      | Global coordinate constraint (unstable quality) |
| Camera   | 20–30 Hz      | Optional          | Visual cues (Fast-LIVO2 supports)    |

#### Outputs

| Output            | Format              | Description                        |
| ----------------- | ------------------- | ---------------------------------- |
| Global point cloud| PCD / PLY / LAS     | High-precision 3D point cloud       |
| Optimized trajectory | TUM / KITTI      | 6DoF poses with timestamps         |
| Submap set       | Tiled PCD + pose JSON | Supports incremental update     |
| Pose graph       | g2o / JSON          | Visualization and re-optimization  |
| Loop closure report | JSON / CSV       | Loop pairs and confidence          |

---

## 2. System Architecture

### 2.1 Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        AutoMap-Pro System Architecture                   │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                    Layer 0: Sensor Ingestion                       │   │
│  │   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────────────┐  │   │
│  │   │  LiDAR  │  │   IMU   │  │ Camera  │  │   GPS/GNSS       │  │   │
│  │   │ 10-20Hz │  │ 200Hz+  │  │ 20-30Hz │  │ 1-10Hz (unstable)│  │   │
│  │   └────┬────┘  └────┬────┘  └────┬────┘  └────────┬─────────┘  │   │
│  └────────┼─────────────┼───────────┼─────────────────┼────────────┘   │
│           │             │           │                 │                 │
│           ▼             ▼           ▼                 ▼                 │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │             Layer 1: Preprocessing and Time Sync                  │   │
│  │  • Hardware/software time synchronization                         │   │
│  │  • LiDAR motion compensation (undistortion)                      │   │
│  │  • GPS coordinate transform (WGS84 → ENU)                          │   │
│  │  • GPS quality assessment and tagging                             │   │
│  └────────────────────────┬─────────────────────────────────────────┘   │
│                           │                                             │
│                           ▼                                             │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │              Layer 2: Frontend State Est. (Fast-LIVO2)             │   │
│  │  • ESIKF tightly-coupled LiDAR-IMU-(Visual)                       │   │
│  │  • IMU preintegration + LiDAR point-to-plane + (visual)          │   │
│  │  • Real-time pose T_w_b @ 10Hz+                                   │   │
│  │  • Keyframe selection and local map maintenance                   │   │
│  │  • Adaptive GPS factor injection                                  │   │
│  └────────────────────────┬─────────────────────────────────────────┘   │
│                           │                                             │
│            Keyframes + poses + local point cloud                         │
│                           │                                             │
│                           ▼                                             │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │        Layer 3: Incremental Submap Mgmt (MS-Mapping)              │   │
│  │  • Submap create / switch / freeze                                │   │
│  │  • Multi-session data association                                 │   │
│  │  • Inter-submap relative pose maintenance                         │   │
│  │  • Submap serialize/deserialize                                   │   │
│  └──────┬────────────────────────────────────┬──────────────────────┘   │
│         │                                    │                          │
│         ▼                                    ▼                          │
│  ┌──────────────────┐              ┌──────────────────────────────┐     │
│  │   Layer 4a:       │              │   Layer 4b:                  │     │
│  │   Loop Coarse     │              │   Pose Graph                 │     │
│  │ (OverlapTransf.)  │              │   (odom + GPS edges)         │     │
│  │                   │              │                              │     │
│  │ • Range Image     │              │                              │     │
│  │ • Descriptor (GPU)│              │                              │     │
│  │ • KNN TopK        │              │                              │     │
│  └────────┬──────────┘              └──────────────┬───────────────┘     │
│           │                                        │                    │
│           ▼                                        │                    │
│  ┌──────────────────┐                              │                    │
│  │   Layer 5:        │                              │                    │
│  │   Loop Fine       │                              │                    │
│  │   (TEASER++)      │                              │                    │
│  │                   │                              │                    │
│  │ • FPFH            │                              │                    │
│  │ • Correspondences │                              │                    │
│  │ • Robust pose     │                              │                    │
│  │ • Inlier check    │                              │                    │
│  └────────┬──────────┘                              │                    │
│           │                                        │                    │
│           │ Loop constraints (ΔT, Σ)               │                    │
│           │                                        │                    │
│           ▼                                        ▼                    │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │           Layer 6: Global Consistency Opt. (HBA)                  │   │
│  │  • Hierarchical Bundle Adjustment                                 │   │
│  │  • Level 0: Frame-level                                           │   │
│  │  • Level 1: Submap-level                                          │   │
│  │  • Level 2: Global pose graph                                     │   │
│  │  • Factors: odometry + loop + GPS                                  │   │
│  └────────────────────────┬─────────────────────────────────────────┘   │
│                           │                                             │
│                           ▼                                             │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │           Layer 7: Global Map Generation and Output                │   │
│  │  • Point cloud reprojection after pose update                      │   │
│  │  • Voxel downsample / statistical filter                          │   │
│  │  • Multi-format export (PCD/PLY/LAS)                              │   │
│  │  • Tiling and LOD                                                 │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Data Flow Overview

```
LiDAR ─┐
IMU ───┤──► Time Sync ──► Fast-LIVO2 ──► Keyframe stream ──► MS-Mapping ─┐
Camera ┤                     ▲                           │             │
GPS ───┘                     │                           │             │
       GPS factors (adaptive) ──────┘                     │             │
                                                          ▼             │
                                                     Submap mgmt        │
                                                     ┌──────┐          │
                                                     │SubMap│          │
                                                     │  0   │          │
                                                     ├──────┤          │
                                                     │SubMap│          │
                                                     │  1   │          │
                                                     ├──────┤          │
                                                     │ ...  │          │
                                                     └──┬───┘          │
                                                        │             │
                                         ┌──────────────┴───┐         │
                                         ▼                  ▼         │
                                  OverlapTransf.      Pose graph      │
                                    (coarse)            │             │
                                         │              │             │
                                         ▼              │             │
                                    TEASER++            │             │
                                   (fine match)         │             │
                                         │              │             │
                                         ▼              ▼             │
                                  Loop constraints ──► HBA ◄──────────┘
                                                    │
                                                    ▼
                                              Update global poses
                                                    │
                                                    ▼
                                              Global map output
```

### 2.3 Threading Model

```
┌─────────────────────────────────────────────────────────────────┐
│                          Main Process                            │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Thread 1 (real-time, highest priority)                        │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Fast-LIVO2 frontend                                        │ │
│  │  • IMU callback (400Hz)                                      │ │
│  │  • LiDAR callback (10-20Hz)                                  │ │
│  │  • Camera callback (20-30Hz, optional)                        │ │
│  │  • GPS callback + quality check (1-10Hz)                     │ │
│  │  • State estimation + keyframe generation                     │ │
│  └────────────────────────┬───────────────────────────────────┘ │
│                           │ KeyFrame (Lock-Free Queue)           │
│                           ▼                                     │
│  Thread 2 (high priority)                                        │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  MS-Mapping submap management                               │ │
│  │  • Keyframe ingestion                                       │ │
│  │  • Submap split and switch                                  │ │
│  │  • Odometry factor insertion                                │ │
│  │  • GPS factor insertion                                     │ │
│  │  • Submap serialization                                     │ │
│  └────────────────────────┬───────────────────────────────────┘ │
│                           │ SubMap Descriptor                   │
│                           ▼                                     │
│  Thread 3 (medium priority, GPU)                                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Loop closure pipeline                                      │ │
│  │  ┌──────────────────────────────────────────────────────┐  │ │
│  │  │  Stage 1: OverlapTransformer (GPU)                    │  │ │
│  │  │  • Range Image generation                              │  │ │
│  │  │  • Network inference → global descriptor               │  │ │
│  │  │  • Candidate retrieval (TopK)                          │  │ │
│  │  └──────────────────────────┬─────────────────────────────┘  │ │
│  │                             │ Loop candidates              │ │
│  │  ┌──────────────────────────▼─────────────────────────────┐  │ │
│  │  │  Stage 2: TEASER++ (CPU)                              │  │ │
│  │  │  • FPFH computation                                    │  │ │
│  │  │  • Correspondence building                             │  │ │
│  │  │  • Robust estimation                                    │  │ │
│  │  │  • Validation and threshold filter                     │  │ │
│  │  └──────────────────────────┬─────────────────────────────┘  │ │
│  └─────────────────────────────┼──────────────────────────────┘ │
│                                │ LoopConstraint                │
│                                ▼                               │
│  Thread 4 (low priority, compute-heavy)                         │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  HBA global optimization                                    │ │
│  │  • Incremental factor graph update                          │ │
│  │  • Hierarchical Bundle Adjustment                           │ │
│  │  • Pose update and propagation                              │ │
│  └────────────────────────┬───────────────────────────────────┘ │
│                           │                                     │
│                           ▼                                     │
│  Thread 5 (background)                                           │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Global map generation                                      │ │
│  │  • Point cloud reprojection                                 │ │
│  │  • Downsample and filter                                    │ │
│  │  • Tiled storage                                            │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Module Design

### 3.1 Sensor Ingestion and Preprocessing Module

#### 3.1.1 Responsibilities

- Multi-sensor data reception and buffering
- Hardware/software time synchronization
- LiDAR motion distortion compensation
- GPS coordinate transform and quality assessment
- Data validity checks

#### 3.1.2 Time Synchronization

**Software time alignment**

- Use LiDAR timestamps as reference
- IMU: linear interpolation
- Camera: nearest-neighbor
- GPS: nearest-neighbor + delay compensation

#### 3.1.3 LiDAR Motion Compensation

- **Input**: Raw point cloud P_raw, IMU queue
- **Process**:
  1. Frame time range [t_start, t_end]
  2. IMU integration for in-frame motion
  3. Per-point timestamp interpolation for compensation
- **Output**: Undistorted point cloud P_undistorted

#### 3.1.4 GPS Coordinate Transform

```
WGS84 (lat, lon, alt)
    │
    ▼ GeographicLib / proj4
ENU (East, North, Up)
    │
    ▼ Origin alignment
Local Frame (x, y, z) — aligned with odometry frame
```

#### 3.1.5 GPS Quality Assessment

```cpp
enum class GPSQuality {
    INVALID    = 0,   // Invalid
    LOW        = 1,   // Single-point, HDOP > 5.0
    MEDIUM     = 2,   // DGPS, 2.0 < HDOP ≤ 5.0
    HIGH       = 3,   // RTK Float, 1.0 < HDOP ≤ 2.0
    EXCELLENT  = 4    // RTK Fixed, HDOP ≤ 1.0
};

struct GPSMeasurement {
    double timestamp;
    Eigen::Vector3d position_enu;
    GPSQuality quality;
    double hdop;
    int num_satellites;
    Eigen::Matrix3d covariance;
};
```

### 3.2 Frontend Odometry Module — Fast-LIVO2

#### 3.2.1 Overview

Fast-LIVO2 is a tightly-coupled LiDAR-Inertial-(Visual) odometry system based on Error-State Iterated Kalman Filter (ESIKF).

#### 3.2.2 State Vector

```
x = [R, p, v, bg, ba, g]^T
```

- R ∈ SO(3): rotation  
- p ∈ R³: position  
- v ∈ R³: velocity  
- bg ∈ R³: gyro bias  
- ba ∈ R³: accel bias  
- g ∈ R³: gravity

#### 3.2.3 Processing Flow

```
IMU ──► Predict (forward propagation)
             │
LiDAR ──► Point-to-plane residual ──► ESIKF update
             │
Camera ──► Photometric residual ──► ESIKF update (optional)
             │
             ▼
        Posterior state
             │
             ├──► Publish real-time pose
             │
             ├──► Keyframe decision
             │       ├── Translation > 1.0 m
             │       ├── Rotation > 10°
             │       └── Time > 2.0 s
             │
             └──► Local map update (ikd-Tree)
```

#### 3.2.4 Keyframe Output Format

```cpp
struct KeyFrame {
    uint64_t id;
    double timestamp;
    Eigen::Isometry3d pose_w_b;
    pcl::PointCloud<PointXYZI>::Ptr cloud;
    cv::Mat image;                      // optional
    GPSMeasurement gps;
    Eigen::Matrix<double, 6, 6> covariance;
    int submap_id;
};
```

#### 3.2.5 GPS Factor in Fast-LIVO2

In ESIKF update, add GPS observation:

- **EXCELLENT/HIGH** → Use as position observation (z_gps = p_enu, small covariance)
- **MEDIUM** → Inflate covariance then inject
- **LOW** → Soft constraint or skip (large covariance)
- **INVALID** → Do not use

#### 3.2.6 Adaptation Notes

| Change              | Description                                              |
| ------------------- | -------------------------------------------------------- |
| Keyframe callback   | Expose keyframe output to MS-Mapping                     |
| GPS fusion          | Add GPS position observation in ESIKF                   |
| Point cloud output  | Downsampled keyframe cloud                               |
| Covariance output   | Output pose covariance for backend                      |

### 3.3 GPS Fusion Module

#### 3.3.1 Design Principles

GPS quality varies; use **graded adaptive fusion**:

- **Quality assessment**: fix_type, HDOP, satellite count, consistency with odometry
- **Jump detection**: |Δp| > threshold → mark INVALID
- **Covariance adaptation**: σ² = f(quality, hdop, consistency)
- **Output**: (position, covariance, is_valid)

#### 3.3.2 GPS–Odometry Consistency

For each GPS measurement:

1. Δp = p_gps - p_odom  
2. Mahalanobis: d_M = sqrt(Δp^T (Σ_gps + Σ_odom)^{-1} Δp)  
3. If d_M > χ²(3, 0.99): mark outlier, do not use; else use with quality-based covariance.

#### 3.3.3 GPS Usage by Module

| Module       | GPS usage                                      |
| ------------ | ----------------------------------------------- |
| Fast-LIVO2   | ESIKF position observation (adaptive covariance)|
| MS-Mapping   | Submap init pose reference                      |
| HBA          | Unary factor in pose graph                      |
| Loop closure | Limit search range when GPS available           |

### 3.4 Incremental Submap Management — MS-Mapping

#### 3.4.1 Overview

MS-Mapping (Multi-Session Mapping) provides:

- Submap partitioning within a session
- Incremental stitching across sessions
- Inter-submap pose maintenance
- Submap-level optimization and update

#### 3.4.2 Submap Split Policy

Switch submap when **any** of:

- Keyframe count in current submap ≥ N_max (default 100)
- Spatial extent ≥ D_max (default 100 m)
- Temporal extent ≥ T_max (default 60 s)
- Significant scene change (feature density/distribution)

#### 3.4.3 Submap Lifecycle

```
  Create (Active)
    │
    ▼  Receive keyframes, update local map
  Mature
    │
    ▼  Split condition met
  Frozen
    │  No new keyframes; used for loop closure and global opt.
    ▼
  Updated
    │  Pose update after global opt.; reproject point cloud
    ▼
  Archived
       Serialize to disk; load on demand
```

#### 3.4.4 Submap Data Structure

```cpp
struct SubMap {
    int id;
    int session_id;
    std::vector<KeyFrame::Ptr> keyframes;
    int anchor_keyframe_id;
    Eigen::Isometry3d pose_w_anchor;
    std::vector<Eigen::Isometry3d> relative_poses;
    pcl::PointCloud<PointXYZI>::Ptr merged_cloud;
    pcl::PointCloud<PointXYZI>::Ptr downsampled_cloud;
    Eigen::VectorXf overlap_descriptor;
    std::vector<GPSMeasurement> gps_measurements;
    Eigen::Vector3d gps_center;
    bool has_valid_gps;
    SubMapState state;  // ACTIVE, MATURE, FROZEN, UPDATED, ARCHIVED
    // methods: addKeyFrame, freeze, updatePoses, reproject, serialize, deserialize
};
```

#### 3.4.5 Incremental Mapping

- **Session 1**: Sensor → Fast-LIVO2 → keyframes → SubMap_0..N → loop + HBA → archive.
- **Session 2**: New data → new keyframes → SubMap_N+1.. → load Session 1 descriptors → cross-session loop (OverlapTransformer + TEASER++) → HBA (Session 1 + 2) → unified map.

### 3.5 Loop Closure Coarse Matching — OverlapTransformer

#### 3.5.1 Overview

OverlapTransformer uses LiDAR range images to produce global descriptors for efficient loop candidate retrieval.

#### 3.5.2 Pipeline

- Input: submap point cloud (downsampled)
- Range image (e.g. 64×900, depth + intensity)
- Network (Encoder + Transformer + NetVLAD) → 256-d descriptor
- Retrieval (FAISS/KD-Tree, TopK=5, time/space filter)
- Output: [(submap_i, submap_j, overlap_score), ...]

#### 3.5.3 GPS-Assisted Search

If both current and candidate have valid GPS and `gps_distance > GPS_SEARCH_RADIUS` (e.g. 200 m), skip candidate; otherwise use descriptor matching.

#### 3.5.4 Key Parameters

| Parameter           | Default | Description              |
| ------------------- | ------- | ------------------------ |
| range_image_height  | 64      | Range image height       |
| range_image_width   | 900     | Range image width        |
| descriptor_dim      | 256     | Descriptor dimension     |
| top_k               | 5       | Number of candidates     |
| overlap_threshold   | 0.3     | Min overlap score        |
| min_temporal_gap    | 30 s    | Min time gap             |
| gps_search_radius   | 200 m   | GPS search radius        |

### 3.6 Loop Closure Fine Matching — TEASER++

#### 3.6.1 Overview

TEASER++ is a certifiably optimal robust point cloud registration method, tolerant to very high outlier ratios (e.g. 99%).

#### 3.6.2 Pipeline

1. **Preprocess**: Voxel downsample (e.g. 0.5 m), outlier removal, cap points (e.g. ≤ 5000).
2. **Features**: FPFH (e.g. k=30, radius=2.5 m) → 33-d per point.
3. **Correspondences**: Feature-space NN + mutual matching → {(s_i, t_i)}.
4. **TEASER++**: Scale (TLS) → rotation (GNC-TLS) → translation (TLS) → R*, t*, inliers.
5. **Validation**: Inlier ratio > threshold (e.g. 30%), RMSE < max_rmse (e.g. 0.3 m); optional ICP refine → emit LoopConstraint or discard.

#### 3.6.3 Optional ICP Refinement

After TEASER++: Point-to-plane ICP (e.g. max iter 30, conv 1e-6, max dist 1.0 m) → T_refined.

#### 3.6.4 Key Parameters

| Parameter           | Default | Description          |
| ------------------- | ------- | -------------------- |
| voxel_size          | 0.5 m   | Downsample voxel     |
| fpfh_radius         | 2.5 m   | FPFH radius          |
| noise_bound         | 0.1 m   | TEASER++ noise       |
| inlier_threshold    | 0.30    | Min inlier ratio     |
| max_rmse            | 0.3 m   | Max RMSE             |
| use_icp_refine      | true    | Use ICP refinement   |

#### 3.6.5 Loop Constraint Structure

```cpp
struct LoopConstraint {
    int submap_i, submap_j;
    int keyframe_i, keyframe_j;
    Eigen::Isometry3d delta_T;           // T_j_i
    Eigen::Matrix<double, 6, 6> information;
    double inlier_ratio, fitness_score, overlap_score;
    bool is_inter_session;
};
```

### 3.7 Global Consistency Optimization — HBA

#### 3.7.1 Overview

HBA (Hierarchical Bundle Adjustment) uses a hierarchical pose graph and point cloud BA for large-scale scenes.

#### 3.7.2 Levels

- **Level 2**: Global coarse — nodes: submap anchor poses; edges: inter-submap odometry, loop (TEASER++), GPS unary; optimizer: LM/GN → optimized submap poses.
- **Level 1**: Per-submap — nodes: keyframe poses; edges: odometry, GPS; anchor fixed from Level 2 → optimize keyframes.
- **Level 0**: Point cloud BA — voxelized point-to-plane, joint pose + plane optimization.

After optimization: update all keyframe poses → trigger reprojection and map update.

#### 3.7.3 Factor Types

1. **Odometry (binary)**: KeyFrame_i — KeyFrame_{i+1}, measurement ΔT_odom, covariance from Fast-LIVO2.
2. **Loop (binary)**: KeyFrame_i — KeyFrame_j, ΔT_loop, covariance from inlier ratio and RMSE.
3. **GPS (unary)**: KeyFrame_k, p_gps_enu, covariance by quality (e.g. EXCELLENT σ≈0.05 m, HIGH 0.1 m, MEDIUM 1 m, LOW 10 m, INVALID not added).
4. **Submap (Level 2 binary)**: SubMap_i — SubMap_{i+1}, relative transform of anchors.

#### 3.7.4 Trigger Policy

- **On new loop** → global optimization  
- **Every M new submaps** → incremental optimization  
- **On mapping end** → final global optimization  
- **Manual trigger**

### 3.8 Global Map Output Module

#### 3.8.1 Flow

After HBA: propagate pose updates → reproject (cloud_world = T_w_b_optimized × cloud_body) → merge global cloud → voxel downsample, statistical outlier removal, optional ground/outlier removal → tiling (e.g. 100 m × 100 m) → export PCD/PLY/LAS and tile index (JSON).

#### 3.8.2 Output Layout

```
output/
├── trajectory/
│   ├── optimized_trajectory_tum.txt
│   ├── optimized_trajectory_kitti.txt
│   └── keyframe_poses.json
├── map/
│   ├── global_map.pcd
│   ├── global_map.ply
│   └── tiles/
├── submaps/
│   ├── submap_000/
│   └── ...
├── loop_closures/
│   └── loop_report.json
├── pose_graph/
│   └── pose_graph.g2o
└── config/
    └── system_config.yaml
```

---

## 4. Core Data Structures

### 4.1 Keyframe

```cpp
struct KeyFrame {
    uint64_t id, session_id;
    int submap_id;
    double timestamp;
    Eigen::Isometry3d T_w_b, T_w_b_optimized;
    Eigen::Matrix<double, 6, 6> covariance;
    pcl::PointCloud<PointXYZI>::Ptr cloud_body, cloud_ds_body;
    cv::Mat image;
    GPSMeasurement gps;
    bool has_valid_gps, is_anchor;
};
```

### 4.2 Submap

(As in §3.4.4; includes id, session_id, state, keyframes, anchor, poses, clouds, descriptor, GPS, adjacency.)

### 4.3 Pose Graph

```cpp
struct PoseGraph {
    std::map<int, PoseNode> nodes;   // id, type, pose, pose_optimized
    std::vector<PoseEdge> edges;      // from, to, type, measurement, information
    void addNode(...), addEdge(...), optimize(), getOptimizedPoses(...);
};
```

### 4.4 Loop Constraint

(As in §3.6.5; submap/keyframe ids, T_j_i, information, scores, is_inter_session, status.)

---

## 5. System Runtime Flow

### 5.1 Online Mapping Main Flow

1. **Start**: Init Fast-LIVO2, MS-Mapping (load historical descriptors if any), OverlapTransformer, TEASER++, HBA, GPS; set ENU origin.
2. **Loop**:  
   - Feed IMU/LiDAR/Camera/GPS to Fast-LIVO2.  
   - On keyframe: build KeyFrame, attach GPS, downsample, push to MS-Mapping.  
   - MS-Mapping: add to current submap, add odom/GPS factors; on split: freeze submap, compute descriptor (Thread 3), create new submap.  
   - Thread 3: OverlapTransformer → TEASER++ → loop constraints.  
   - Thread 4: On new loop → HBA → pose update → Thread 5 map update.
3. **End**: Final HBA, reproject, post-process, export, serialize submaps.

### 5.2 Loop Detection and Optimization Flow

Submap frozen → Range image → OverlapTransformer (~10 ms) → descriptor DB + query → TopK → GPS filter → for each candidate: FPFH (~50 ms), TEASER++ (~100 ms), optional ICP (~30 ms) → validate (inlier ratio, RMSE, fitness) → LoopConstraint to HBA → Level 2 (~200 ms) → Level 1 (~500 ms) → Level 0 (~2 s optional) → pose broadcast → reproject and map update.

### 5.3 Incremental Mapping Flow (Session N)

1. Load historical descriptor DB, pose graph (compact), GPS anchors.  
2. Run Fast-LIVO2 on new data → new submaps (session_id = N), compute descriptors.  
3. Cross-session loop: OverlapTransformer (new vs history), GPS pruning, TEASER++ → cross-session constraints.  
4. HBA: new + history submaps, GPS factors → global poses.  
5. Update affected point clouds, merge maps, archive updated submaps.

---

## 6. GPS Processing Strategy

### 6.1 GPS State Machine

- **INIT**: Wait for first valid GPS.  
- **TRACKING**: Normal GPS; on loss/degradation → **DEGRADED** (odometry only); on recovery → back to TRACKING.  
- **DEGRADED**: Degraded GPS / odometry only.  
- **LOST**: Long loss → rely on loop + odometry; on recovery → back to TRACKING.

### 6.2 Covariance by Stage

| State     | Covariance  | Frontend      | Backend           |
| --------- | ----------- | ------------- | ------------------ |
| EXCELLENT | σ≈0.05 m    | Strong        | Strong unary       |
| HIGH      | σ≈0.1 m     | Strong        | Strong unary       |
| MEDIUM    | σ≈1 m       | Weak          | Weak unary         |
| LOW       | σ≈10 m      | Almost none   | Very weak / none   |
| INVALID   | Not used    | Skip          | Not added          |
| DEGRADED  | Increasing σ| Gradually weak| Increase over time |
| LOST      | Not used    | LIO only      | Rely on loops      |

### 6.3 Jump Handling

- **Detect**: |p_gps(t) - p_gps(t-1)| > v_max×Δt + 3σ.  
- **Action**: Mark outlier; 3 consecutive → DEGRADED; 5 consecutive good → TRACKING.  
- **Recovery from LOST**: Wait for 5 consistent GPS, compare with odometry; if offset OK → gradually inject GPS factors; else keep waiting.

---

## 7. Project Directory Structure

```
automap_pro/
├── CMakeLists.txt, package.xml, README.md, LICENSE
├── config/
│   ├── system_config.yaml, fast_livo2_config.yaml
│   ├── overlap_transformer_config.yaml, teaser_config.yaml
│   ├── hba_config.yaml, ms_mapping_config.yaml, gps_config.yaml
│   └── sensor_config/ (lidar_imu_extrinsic, camera_imu_extrinsic, gps_imu_extrinsic)
├── launch/
│   ├── automap_online.launch, automap_offline.launch
│   ├── automap_incremental.launch, visualization.launch
├── src/
│   ├── core/          (system, config_manager, data_types, utils)
│   ├── sensor/        (sensor_manager, lidar/imu/camera/gps_processor, time_sync)
│   ├── frontend/      (fast_livo2_wrapper, keyframe_manager, gps_fusion)
│   ├── submap/        (ms_mapping_wrapper, submap, submap_manager, session_manager)
│   ├── loop_closure/  (loop_detector, overlap_transformer, teaser_matcher, fpfh_extractor, icp_refiner, loop_validator)
│   ├── backend/       (hba_wrapper, pose_graph, factor_types, optimizer)
│   ├── map/           (global_map, map_builder, map_filter, map_exporter)
│   └── visualization/ (rviz_publisher, web_visualizer)
├── scripts/           (evaluate_trajectory, evaluate_map, merge_sessions, visualize_results)
├── thirdparty/        (fast_livo2, overlap_transformer, teaser_plusplus, hba, ms_mapping)
├── models/            (overlap_transformer/pretrained.pth)
├── test/
├── docker/
└── docs/
```

---

## 8. ROS Nodes and Topics Design

### 8.1 Nodes

- `/automap/sensor_sync`, `/automap/fast_livo2`, `/automap/gps_processor`
- `/automap/submap_manager`, `/automap/loop_detector`, `/automap/optimizer`
- `/automap/map_builder`, `/automap/visualizer`

### 8.2 Topics

**Input**: `/livox/lidar` (PointCloud2), `/livox/imu` (Imu), `/camera/image_raw` (Image), `/gps/fix` (NavSatFix), `/gps/status` (NavSatStatus).

**Internal**: `/automap/synced_lidar`, `/automap/synced_imu`, `/automap/gps_enu`, `/automap/gps_quality`, `/automap/odometry`, `/automap/keyframe`, `/automap/submap_event`, `/automap/loop_constraint`, `/automap/optimization_trigger`.

**Output**: `/automap/odom_path`, `/automap/optimized_path`, `/automap/current_cloud`, `/automap/submap_cloud`, `/automap/global_map`, `/automap/loop_markers`, `/automap/gps_markers`.

**Services**: `/automap/save_map`, `/automap/trigger_optimize`, `/automap/load_session`, `/automap/get_status`.

### 8.3 Custom Messages

- **KeyFrame**: header, id, session_id, submap_id, pose, cloud, image, gps, has_valid_gps.  
- **GPSMeasurement**: header, position_enu[3], quality, hdop, num_satellites, covariance[9].  
- **LoopConstraint**: header, submap_i/j, kf_i/j, delta_pose, information[36], overlap_score, inlier_ratio, fitness_score, rmse, is_inter_session.  
- **SubMapEvent**: header, submap_id, session_id, event_type ("CREATED","FROZEN","UPDATED","ARCHIVED").

---

## 9. Performance Metrics and Optimization Strategy

### 9.1 Target Metrics

| Metric              | Target    | Note                    |
| ------------------- | --------- | ----------------------- |
| Frontend rate       | ≥ 10 Hz   | Fast-LIVO2              |
| Keyframe rate       | 1–5 Hz    | Motion-dependent        |
| Submap descriptor   | < 50 ms   | OverlapTransformer GPU  |
| Loop candidate      | < 10 ms   | FAISS/KD-Tree           |
| TEASER++ fine       | < 200 ms  | incl. FPFH              |
| HBA incremental     | < 1 s     | On new loop             |
| HBA global          | < 30 s    | ~1000 submaps           |
| End-to-end delay    | < 3 s     | Loop to map update      |
| Global accuracy     | < 0.3%    | Relative trajectory     |
| Trajectory length   | > 50 km   | Incremental             |
| Memory              | < 16 GB   | Online                  |

### 9.2 Optimization

- **Compute**: GPU for OverlapTransformer (TensorRT), point cloud downsample, range image; multi-thread (frontend/loop/opt/map), lock-free queues; incremental HBA and descriptor DB; ikd-Tree, FAISS, tiled hash; load submaps/point clouds on demand.
- **Memory**: Keyframe cloud in body frame, downsampled (e.g. 0.2 m); submap matching cloud 0.5 m; archived submaps: release cloud, keep descriptor; global map: stream to disk by tiles; release range images after use.

---

## 10. Build and Deployment

### 10.1 Dependencies

- **OS**: Ubuntu 20.04 LTS  
- **ROS**: ROS2 Humble (or as per project)  
- **Build**: CMake ≥ 3.16, GCC ≥ 9.0, CUDA ≥ 11.3 (for GPU)  
- **Libraries**: Eigen ≥ 3.3, PCL ≥ 1.10, OpenCV ≥ 4.2, GTSAM ≥ 4.1 (or Ceres), GeographicLib ≥ 1.50, LibTorch ≥ 1.10 (or Python API), Open3D ≥ 0.13  

### 10.2 Build

```bash
mkdir -p ~/automap_ws/src
cd ~/automap_ws/src
# Clone or link modules (ensure ROS2 versions if required)
cd ~/automap_ws
colcon build
```

### 10.3 Docker

Use project `docker/Dockerfile` and `docker-compose.yml` for containerized deployment (see repo).

---

## 11. Testing and Validation

### 11.1 Datasets

- Custom urban (10–50 km, intermittent GPS)  
- KITTI (~40 km, GPS)  
- MulRan (~10 km, GPS)  
- Custom tunnel (1–5 km, no GPS)  
- Custom campus (2–10 km, partial GPS)  

### 11.2 Metrics

- **Trajectory**: APE, RPE, ATE (with ground truth or RTK).  
- **Map**: Consistency (overlap alignment), flatness, sharpness.  
- **System**: Frontend rate, loop latency, optimization time, memory, CPU/GPU.  
- **Loop**: Precision, Recall, F1.  

### 11.3 Tools

```bash
evo_ape tum groundtruth.txt optimized_trajectory.txt -p --plot_mode=xz
evo_rpe tum groundtruth.txt optimized_trajectory.txt -p
# Map: CloudCompare C2C distance
# Loops: scripts/evaluate_loop_detection.py
```

---

## 12. Future Extensions

### 12.1 Short Term

- Semantic point cloud (semantic segmentation)
- Dynamic object removal (motion consistency)
- Multi-resolution LOD map
- Web visualization/monitoring

### 12.2 Medium Term

- Multi-robot collaborative mapping
- ROS2 migration
- Real-time map service (gRPC)
- Map change detection

### 12.3 Long Term

- HD map element extraction
- BEV map generation
- Automated QA pipeline
- Cloud distributed mapping

---

# Appendix A: Dependencies

(See §10.1.)

---

# Appendix B: Configuration Examples

```yaml
# ============================================================
# AutoMap-Pro System Global Configuration
# ============================================================

system:
  name: "AutoMap-Pro"
  mode: "online"          # online / offline / incremental
  log_level: "info"      # debug / info / warn / error
  output_dir: "/data/automap_output"
  num_threads: 8
  use_gpu: true
  gpu_device_id: 0

sensor:
  lidar:
    type: "livox_avia"
    topic: "/livox/lidar"
    frequency: 10
    max_range: 150.0
    min_range: 0.5
    undistort: true
  imu:
    topic: "/livox/imu"
    frequency: 200
    gravity: 9.81
  camera:
    enabled: false
    topic: "/camera/image_raw"
    frequency: 20
  gps:
    enabled: true
    topic: "/gps/fix"
    frequency: 10
    enu_origin:
      auto_init: true

gps_fusion:
  quality_thresholds:
    excellent_hdop: 1.0
    high_hdop: 2.0
    medium_hdop: 5.0
  covariance:
    excellent: [0.05, 0.05, 0.10]
    high:      [0.10, 0.10, 0.20]
    medium:    [1.00, 1.00, 2.00]
    low:       [10.0, 10.0, 20.0]
  consistency_check:
    enabled: true
    chi2_threshold: 11.345
    max_velocity: 30.0
  jump_detection:
    enabled: true
    max_jump: 5.0
    consecutive_valid: 5

frontend:
  fast_livo2:
    config_file: "config/fast_livo2_config.yaml"
    keyframe_policy:
      min_translation: 1.0
      min_rotation: 10.0
      max_interval: 2.0
    cloud_downsample_resolution: 0.2

submap:
  ms_mapping:
    config_file: "config/ms_mapping_config.yaml"
  split_policy:
    max_keyframes: 100
    max_spatial_extent: 100.0
    max_temporal_extent: 60.0
  cloud_for_matching_resolution: 0.5

loop_closure:
  overlap_transformer:
    model_path: "models/overlap_transformer/pretrained.pth"
    range_image:
      height: 64
      width: 900
    descriptor_dim: 256
    top_k: 5
    overlap_threshold: 0.3
    min_temporal_gap: 30.0
    min_submap_gap: 3
    gps_search_radius: 200.0
  teaser:
    voxel_size: 0.5
    fpfh:
      normal_radius: 1.0
      feature_radius: 2.5
      max_nn_normal: 30
      max_nn_feature: 100
    solver:
      noise_bound: 0.1
      cbar2: 1.0
      rotation_gnc_factor: 1.4
      rotation_max_iterations: 100
      rotation_cost_threshold: 1e-6
    validation:
      min_inlier_ratio: 0.30
      max_rmse: 0.3
      min_fitness: 0.5
    icp_refine:
      enabled: true
      max_iterations: 30
      max_correspondence_distance: 1.0
      convergence_threshold: 1e-6

backend:
  hba:
    config_file: "config/hba_config.yaml"
    trigger_policy:
      on_loop: true
      periodic_submaps: 10
      on_finish: true
    optimization:
      max_iterations: 100
      convergence_threshold: 1e-4
      use_robust_kernel: true
      robust_kernel_delta: 1.0

map_output:
  global_map:
    voxel_size: 0.1
    statistical_filter:
      enabled: true
      mean_k: 50
      std_dev_mul: 2.0
  tiling:
    enabled: true
    tile_size: 100.0
  formats:
    pcd: true
    ply: true
    las: false

visualization:
  publish_rate: 1.0
  global_map_publish: true
  global_map_downsample: 0.5
  show_loop_closures: true
  show_gps_trajectory: true
  show_submap_boundaries: true
```

---

**End of document**

This document defines the full architecture of the AutoMap-Pro system, from sensor ingestion to final map output. The system uses Fast-LIVO2 for high-precision frontend estimation, OverlapTransformer and TEASER++ for robust loop closure, HBA for global consistency, MS-Mapping for incremental mapping, and adaptive GPS fusion for unstable GPS conditions.
