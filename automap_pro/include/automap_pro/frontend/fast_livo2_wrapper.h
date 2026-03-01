#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <functional>

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/sensor/lidar_processor.h"
#include "automap_pro/sensor/imu_processor.h"
#include "automap_pro/sensor/time_sync.h"
#include "automap_pro/frontend/keyframe_manager.h"
#include "automap_pro/frontend/gps_fusion.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// ESIKF State
// ──────────────────────────────────────────────────────────
struct ESIKFState {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // rotation
    Eigen::Vector3d p = Eigen::Vector3d::Zero();       // position
    Eigen::Vector3d v = Eigen::Vector3d::Zero();       // velocity
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();      // gyro bias
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();      // accel bias
    Eigen::Vector3d g  = {0, 0, -9.81};               // gravity

    // 18×18 covariance (R(9), p(3), v(3), bg(3), ba(3) — in tangent space)
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(18, 18) * 1e-3;

    Pose3d toPose() const {
        Pose3d T;
        T.linear()      = R;
        T.translation() = p;
        return T;
    }
};

// ──────────────────────────────────────────────────────────
// Fast-LIVO2 Wrapper
// Wraps the actual Fast-LIVO2 library (or provides a self-contained
// ESIKF-based LiDAR-IMU odometry when the library is not present).
// ──────────────────────────────────────────────────────────
class FastLIVO2Wrapper {
public:
    using PoseCallback    = std::function<void(double, const Pose3d&, const Mat66d&)>;
    using KeyFrameCallback = std::function<void(const KeyFrame::Ptr&)>;

    FastLIVO2Wrapper();
    ~FastLIVO2Wrapper();

    void init(rclcpp::Node::SharedPtr node,
              TimedBuffer<ImuData>& imu_buffer,
              TimedBuffer<GPSMeasurement>& gps_buffer);

    void registerPoseCallback   (PoseCallback cb);
    void registerKeyFrameCallback(KeyFrameCallback cb);

    // Feed data
    void feedLidar(const LidarFrame::Ptr& frame);
    void feedImu  (const ImuData& imu);
    void feedGPS  (const GPSMeasurement& gps);

    const ESIKFState& currentState() const;
    bool isInitialized() const;

    void reset();

private:
    // ESIKF prediction step
    void imuPredict(const ImuData& imu);

    // ESIKF update step with point-to-plane residuals
    void lidarUpdate(const CloudXYZIPtr& cloud);

    // ESIKF update with GPS position observation
    void gpsUpdate(const GPSMeasurement& gps);

    // ikd-Tree local map update (simplified flat hash map version)
    void updateLocalMap(const CloudXYZIPtr& cloud_world);

    // Find nearest points in local map (KNN)
    bool findNearestPlane(const Eigen::Vector3d& query,
                           Eigen::Vector3d& normal,
                           Eigen::Vector3d& centroid,
                           double& residual) const;

    // Publish real-time pose to ROS
    void publishPose(double timestamp, const Pose3d& pose, const Mat66d& cov);

    mutable std::mutex state_mutex_;
    ESIKFState state_;
    bool       initialized_ = false;
    double     last_imu_time_ = -1.0;

    // Local map: voxelized point storage
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& o) const { return x==o.x && y==o.y && z==o.z; }
    };
    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 16) ^ (std::hash<int>()(k.z) << 32);
        }
    };
    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3f>, VoxelKeyHash> local_map_;
    double local_map_voxel_ = 0.2;
    size_t local_map_max_pts_per_voxel_ = 20;
    mutable std::mutex map_mutex_;

    // ESIKF parameters
    double gyro_noise_  = 1e-3;
    double accel_noise_ = 1e-2;
    double gyro_bias_noise_  = 1e-5;
    double accel_bias_noise_ = 1e-4;
    int    esikf_max_iter_   = 5;
    double esikf_tolerance_  = 1e-6;

    // Keyframe
    std::unique_ptr<KeyFrameManager> kf_manager_;
    std::unique_ptr<GPSFusion>       gps_fusion_;
    uint64_t session_id_ = 0;
    GPSMeasurement latest_gps_;
    bool has_latest_gps_ = false;

    std::vector<PoseCallback>     pose_callbacks_;
    std::vector<KeyFrameCallback> kf_callbacks_;

    // ROS2 publishers
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    TimedBuffer<ImuData>*        imu_buffer_ = nullptr;
    TimedBuffer<GPSMeasurement>* gps_buffer_ = nullptr;

    // Cloud downsample resolution
    double cloud_ds_res_ = 0.2;
};

}  // namespace automap_pro
