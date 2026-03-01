#pragma once

#include <memory>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// Type aliases
// ──────────────────────────────────────────────────────────
using PointXYZI   = pcl::PointXYZI;
using CloudXYZI   = pcl::PointCloud<PointXYZI>;
using CloudXYZIPtr = CloudXYZI::Ptr;
using Pose3d      = Eigen::Isometry3d;
using Mat66d      = Eigen::Matrix<double, 6, 6>;
using Vec3d       = Eigen::Vector3d;
using Vec6d       = Eigen::Matrix<double, 6, 1>;
using Quat        = Eigen::Quaterniond;

// ──────────────────────────────────────────────────────────
// GPS Quality
// ──────────────────────────────────────────────────────────
enum class GPSQuality : int {
    INVALID   = 0,
    LOW       = 1,   // Single-point, HDOP > 5.0
    MEDIUM    = 2,   // DGPS,  2.0 < HDOP ≤ 5.0
    HIGH      = 3,   // RTK Float, 1.0 < HDOP ≤ 2.0
    EXCELLENT = 4    // RTK Fixed, HDOP ≤ 1.0
};

std::string gpsQualityToString(GPSQuality q);

// ──────────────────────────────────────────────────────────
// GPS State Machine
// ──────────────────────────────────────────────────────────
enum class GPSState : int {
    INIT     = 0,
    TRACKING = 1,
    DEGRADED = 2,
    LOST     = 3
};

std::string gpsStateToString(GPSState s);

// ──────────────────────────────────────────────────────────
// GPS Measurement
// ──────────────────────────────────────────────────────────
struct GPSMeasurement {
    double timestamp = 0.0;
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero();
    GPSQuality quality = GPSQuality::INVALID;
    double hdop = 99.0;
    int num_satellites = 0;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 1e6;
    bool is_valid = false;
    bool is_outlier = false;

    // Raw GNSS
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    using Ptr = std::shared_ptr<GPSMeasurement>;
};

// ──────────────────────────────────────────────────────────
// IMU Data
// ──────────────────────────────────────────────────────────
struct ImuData {
    double timestamp = 0.0;
    Eigen::Vector3d angular_velocity  = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();

    using Ptr = std::shared_ptr<ImuData>;
};

// ──────────────────────────────────────────────────────────
// KeyFrame
// ──────────────────────────────────────────────────────────
struct KeyFrame {
    uint64_t id          = 0;
    uint64_t session_id  = 0;
    int submap_id        = -1;
    double timestamp     = 0.0;

    Pose3d T_w_b;                        // World ← Body (raw odometry)
    Pose3d T_w_b_optimized;              // World ← Body (after HBA)
    Mat66d covariance = Mat66d::Identity() * 1e-3;

    CloudXYZIPtr cloud_body;             // Point cloud in body frame
    CloudXYZIPtr cloud_ds_body;          // Downsampled cloud in body frame

    cv::Mat image;                       // Optional camera image
    GPSMeasurement gps;
    bool has_valid_gps = false;
    bool is_anchor     = false;

    using Ptr = std::shared_ptr<KeyFrame>;
};

// ──────────────────────────────────────────────────────────
// SubMap lifecycle state
// ──────────────────────────────────────────────────────────
enum class SubMapState : int {
    ACTIVE   = 0,
    MATURE   = 1,
    FROZEN   = 2,
    UPDATED  = 3,
    ARCHIVED = 4
};

std::string subMapStateToString(SubMapState s);

// ──────────────────────────────────────────────────────────
// SubMap
// ──────────────────────────────────────────────────────────
struct SubMap {
    int id         = -1;
    int session_id = 0;

    std::vector<KeyFrame::Ptr> keyframes;
    int anchor_keyframe_id = -1;

    Pose3d pose_w_anchor;
    Pose3d pose_w_anchor_optimized;
    std::vector<Pose3d> relative_poses;  // T_anchor_kfi for each keyframe

    CloudXYZIPtr merged_cloud;           // All points in world frame
    CloudXYZIPtr downsampled_cloud;      // Matching cloud (0.5 m voxel)

    Eigen::VectorXf overlap_descriptor;  // 256-d OverlapTransformer descriptor
    bool has_descriptor = false;

    std::vector<GPSMeasurement> gps_measurements;
    Eigen::Vector3d gps_center = Eigen::Vector3d::Zero();
    bool has_valid_gps = false;

    SubMapState state = SubMapState::ACTIVE;

    double t_start = 0.0;
    double t_end   = 0.0;
    double spatial_extent = 0.0;

    using Ptr = std::shared_ptr<SubMap>;

    bool isFull(int max_kf, double max_spatial, double max_temporal) const;
    void addKeyFrame(const KeyFrame::Ptr& kf);
    void freeze();
    void updateAnchorPose(const Pose3d& new_pose);
    void reproject();
};

// ──────────────────────────────────────────────────────────
// Loop Constraint Status
// ──────────────────────────────────────────────────────────
enum class LoopStatus : int {
    CANDIDATE  = 0,
    ACCEPTED   = 1,
    REJECTED   = 2,
    INTEGRATED = 3
};

// ──────────────────────────────────────────────────────────
// Loop Constraint
// ──────────────────────────────────────────────────────────
struct LoopConstraint {
    int submap_i    = -1;
    int submap_j    = -1;
    int keyframe_i  = -1;
    int keyframe_j  = -1;

    Pose3d delta_T;               // T_j_i  (transform from i to j)
    Mat66d information = Mat66d::Identity();

    double inlier_ratio   = 0.0;
    double fitness_score  = 0.0;
    double overlap_score  = 0.0;
    double rmse           = 1e9;

    bool is_inter_session = false;
    LoopStatus status = LoopStatus::CANDIDATE;

    using Ptr = std::shared_ptr<LoopConstraint>;
};

// ──────────────────────────────────────────────────────────
// Pose Graph Node types
// ──────────────────────────────────────────────────────────
enum class NodeType : int {
    KEYFRAME = 0,
    SUBMAP   = 1
};

enum class EdgeType : int {
    ODOMETRY    = 0,
    LOOP        = 1,
    GPS         = 2,
    SUBMAP_ODOM = 3
};

struct PoseNode {
    int id = -1;
    NodeType type = NodeType::KEYFRAME;
    Pose3d pose;
    Pose3d pose_optimized;
    bool fixed = false;

    using Ptr = std::shared_ptr<PoseNode>;
};

struct PoseEdge {
    int from = -1;
    int to   = -1;
    EdgeType type = EdgeType::ODOMETRY;
    Pose3d measurement;                      // Relative transform (binary) or absolute (unary GPS)
    Mat66d information = Mat66d::Identity();

    using Ptr = std::shared_ptr<PoseEdge>;
};

// ──────────────────────────────────────────────────────────
// System State
// ──────────────────────────────────────────────────────────
enum class SystemState : int {
    IDLE        = 0,
    INITIALIZING = 1,
    MAPPING     = 2,
    OPTIMIZING  = 3,
    SAVING      = 4,
    ERROR       = 5
};

// ──────────────────────────────────────────────────────────
// Mapping Mode
// ──────────────────────────────────────────────────────────
enum class MappingMode : int {
    ONLINE      = 0,
    OFFLINE     = 1,
    INCREMENTAL = 2
};

// ──────────────────────────────────────────────────────────
// ENU Origin
// ──────────────────────────────────────────────────────────
struct ENUOrigin {
    double latitude  = 0.0;
    double longitude = 0.0;
    double altitude  = 0.0;
    bool initialized = false;
};

}  // namespace automap_pro
