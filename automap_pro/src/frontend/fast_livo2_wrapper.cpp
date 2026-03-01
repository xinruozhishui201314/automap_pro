#include "automap_pro/frontend/fast_livo2_wrapper.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace automap_pro {

FastLIVO2Wrapper::FastLIVO2Wrapper()
    : kf_manager_(std::make_unique<KeyFrameManager>())
    , gps_fusion_(std::make_unique<GPSFusion>())
{
    const auto& cfg = ConfigManager::instance();
    cloud_ds_res_ = cfg.cloudDownsampleResolution();
    local_map_voxel_ = 0.2;
}

FastLIVO2Wrapper::~FastLIVO2Wrapper() = default;

void FastLIVO2Wrapper::init(rclcpp::Node::SharedPtr node,
                              TimedBuffer<ImuData>& imu_buffer,
                              TimedBuffer<GPSMeasurement>& gps_buffer) {
    node_ = node;
    imu_buffer_ = &imu_buffer;
    gps_buffer_ = &gps_buffer;

    pose_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/automap/odometry", 10);
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/automap/odom_path", 1);

    RCLCPP_INFO(node->get_logger(), "[FastLIVO2] Initialized (ESIKF LiDAR-IMU odometry).");
}

void FastLIVO2Wrapper::registerPoseCallback(PoseCallback cb) {
    pose_callbacks_.push_back(std::move(cb));
}

void FastLIVO2Wrapper::registerKeyFrameCallback(KeyFrameCallback cb) {
    kf_manager_->registerCallback(std::move(cb));
}

void FastLIVO2Wrapper::feedImu(const ImuData& imu) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (!initialized_) {
        last_imu_time_ = imu.timestamp;
        return;
    }
    if (last_imu_time_ > 0) {
        imuPredict(imu);
    }
    last_imu_time_ = imu.timestamp;
}

void FastLIVO2Wrapper::feedGPS(const GPSMeasurement& gps) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_gps_    = gps;
    has_latest_gps_ = gps.is_valid;
    if (initialized_) {
        gpsUpdate(gps);
    }
}

void FastLIVO2Wrapper::feedLidar(const LidarFrame::Ptr& frame) {
    if (!frame || frame->cloud_undistorted->empty()) return;

    // First valid LiDAR frame initializes the system
    if (!initialized_) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        state_.g = Eigen::Vector3d(0, 0, -9.81);
        initialized_ = true;
        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[FastLIVO2] Initialized with first LiDAR frame at t=%.3f",
                 frame->timestamp_start);
    }

    CloudXYZIPtr cloud_ds = utils::voxelDownsample(
        frame->cloud_undistorted, local_map_voxel_);

    // ESIKF LiDAR update
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        lidarUpdate(cloud_ds);
    }

    // Build world-frame cloud for local map update
    Pose3d current_pose;
    Mat66d current_cov;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        current_pose = state_.toPose();
        current_cov  = Mat66d::Identity() * 1e-4;
    }

    CloudXYZIPtr cloud_world = utils::transformCloud(cloud_ds, current_pose);
    {
        std::lock_guard<std::mutex> lk(map_mutex_);
        updateLocalMap(cloud_world);
    }

    // Publish pose
    publishPose(frame->timestamp_start, current_pose, current_cov);

    // Keyframe decision
    if (kf_manager_->shouldCreateKeyFrame(current_pose, frame->timestamp_start)) {
        auto cloud_kf_ds = utils::voxelDownsample(frame->cloud_undistorted, cloud_ds_res_);

        GPSMeasurement gps_copy;
        bool has_gps = false;
        {
            std::lock_guard<std::mutex> lk(state_mutex_);
            gps_copy = latest_gps_;
            has_gps  = has_latest_gps_;
        }

        kf_manager_->createKeyFrame(
            current_pose, current_pose, current_cov,
            frame->timestamp_start,
            frame->cloud_undistorted, cloud_kf_ds,
            gps_copy, has_gps, session_id_);
    }
}

void FastLIVO2Wrapper::imuPredict(const ImuData& imu) {
    double dt = imu.timestamp - last_imu_time_;
    if (dt <= 0 || dt > 0.1) return;

    auto& s = state_;
    Eigen::Vector3d omega = imu.angular_velocity   - s.bg;
    Eigen::Vector3d acc   = imu.linear_acceleration - s.ba;

    // Rotation update
    double angle = omega.norm() * dt;
    if (angle > 1e-10) {
        Eigen::AngleAxisd aa(angle, omega.normalized());
        s.R = s.R * aa.toRotationMatrix();
    }

    // Position and velocity update
    Eigen::Vector3d acc_world = s.R * acc + s.g;
    s.p += s.v * dt + 0.5 * acc_world * dt * dt;
    s.v += acc_world * dt;

    // Covariance propagation (simplified)
    // F = identity + F_c * dt, Q_d = Q_c * dt
    double gn = gyro_noise_  * gyro_noise_;
    double an = accel_noise_ * accel_noise_;
    double gbn = gyro_bias_noise_  * gyro_bias_noise_;
    double abn = accel_bias_noise_ * accel_bias_noise_;

    // Process noise (diagonal approx)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(18, 18);
    Q.block<3,3>(0,0)   = Eigen::Matrix3d::Identity() * gn * dt;
    Q.block<3,3>(6,6)   = Eigen::Matrix3d::Identity() * an * dt;
    Q.block<3,3>(9,9)   = Eigen::Matrix3d::Identity() * gbn * dt;
    Q.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * abn * dt;
    s.P += Q;
}

void FastLIVO2Wrapper::lidarUpdate(const CloudXYZIPtr& cloud) {
    if (!cloud || cloud->empty()) return;

    // Build residuals: for each point find nearest plane in local map
    std::vector<Eigen::Vector3d> normals, residuals_v;
    std::vector<Eigen::Vector3d> points_body;

    for (const auto& pt : cloud->points) {
        Eigen::Vector3d pb(pt.x, pt.y, pt.z);
        Eigen::Vector3d pw = state_.R * pb + state_.p;

        Eigen::Vector3d normal, centroid;
        double residual;
        std::lock_guard<std::mutex> lk(map_mutex_);
        if (findNearestPlane(pw, normal, centroid, residual)) {
            normals.push_back(normal);
            points_body.push_back(pb);
            residuals_v.push_back(Eigen::Vector3d(residual, 0, 0));
        }
    }

    if (normals.empty()) return;

    // ESIKF iterative update
    const int max_iter = esikf_max_iter_;
    for (int iter = 0; iter < max_iter; ++iter) {
        // Build H and z
        int n_obs = static_cast<int>(normals.size());
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_obs, 18);
        Eigen::VectorXd z = Eigen::VectorXd::Zero(n_obs);
        Eigen::MatrixXd R_noise = Eigen::MatrixXd::Identity(n_obs, n_obs) * 0.01;

        for (int i = 0; i < n_obs; ++i) {
            const auto& n = normals[i];
            const auto& pb = points_body[i];
            // Measurement Jacobian w.r.t. rotation and position
            // H_pos = n^T
            H.block<1,3>(i, 3) = n.transpose();
            // H_rot = -n^T * R * [pb]_x
            Eigen::Matrix3d pb_skew = utils::skewSymmetric(pb);
            H.block<1,3>(i, 0)     = -n.transpose() * state_.R * pb_skew;

            // Residual: n^T * (R*pb + p - centroid)
            Eigen::Vector3d pw = state_.R * pb + state_.p;
            z(i) = n.dot(pw - Eigen::Vector3d(0,0,0));  // using residual directly
            z(i) = residuals_v[i](0);
        }

        // Kalman gain
        Eigen::MatrixXd S = H * state_.P * H.transpose() + R_noise;
        Eigen::MatrixXd K = state_.P * H.transpose() * S.inverse();

        // State update (in error space)
        Eigen::VectorXd delta_x = -K * z;

        // Apply correction
        Eigen::Vector3d delta_r = delta_x.segment<3>(0);
        state_.p  += delta_x.segment<3>(3);
        state_.v  += delta_x.segment<3>(6);
        state_.bg += delta_x.segment<3>(9);
        state_.ba += delta_x.segment<3>(12);

        double angle = delta_r.norm();
        if (angle > 1e-10) {
            Eigen::AngleAxisd aa(angle, delta_r.normalized());
            state_.R = state_.R * aa.toRotationMatrix();
        }

        // Covariance update
        Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(18,18) - K * H;
        state_.P = I_KH * state_.P * I_KH.transpose()
                 + K * R_noise * K.transpose();

        if (delta_x.norm() < esikf_tolerance_) break;
    }
}

void FastLIVO2Wrapper::gpsUpdate(const GPSMeasurement& gps) {
    if (!gps.is_valid || gps.is_outlier) return;

    auto obs = gps_fusion_->processForFrontend(
        gps, state_.toPose(),
        state_.P.block<3,3>(3,3));
    if (!obs.should_use) return;

    // GPS observation: z = p_enu, H = [0 I 0 ...] (position)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 18);
    H.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd R_gps = obs.covariance;
    Eigen::MatrixXd S     = H * state_.P * H.transpose() + R_gps;
    Eigen::MatrixXd K     = state_.P * H.transpose() * S.inverse();

    Eigen::Vector3d innovation = obs.position_enu - state_.p;
    Eigen::VectorXd delta_x   = K * innovation;

    state_.p  += delta_x.segment<3>(3);
    state_.v  += delta_x.segment<3>(6);
    state_.bg += delta_x.segment<3>(9);
    state_.ba += delta_x.segment<3>(12);

    Eigen::Vector3d delta_r = delta_x.segment<3>(0);
    double angle = delta_r.norm();
    if (angle > 1e-10) {
        Eigen::AngleAxisd aa(angle, delta_r.normalized());
        state_.R = state_.R * aa.toRotationMatrix();
    }

    Eigen::MatrixXd I_KH = Eigen::MatrixXd::Identity(18,18) - K * H;
    state_.P = I_KH * state_.P;
}

void FastLIVO2Wrapper::updateLocalMap(const CloudXYZIPtr& cloud_world) {
    for (const auto& pt : cloud_world->points) {
        VoxelKey key{
            static_cast<int>(std::floor(pt.x / local_map_voxel_)),
            static_cast<int>(std::floor(pt.y / local_map_voxel_)),
            static_cast<int>(std::floor(pt.z / local_map_voxel_))
        };
        auto& cell = local_map_[key];
        if (cell.size() < local_map_max_pts_per_voxel_) {
            cell.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
        }
    }
    // Remove old voxels if map grows too large
    if (local_map_.size() > 200000) {
        auto it = local_map_.begin();
        size_t to_remove = local_map_.size() / 4;
        for (size_t i = 0; i < to_remove && it != local_map_.end(); ++i, ++it) {
            it = local_map_.erase(it);
        }
    }
}

bool FastLIVO2Wrapper::findNearestPlane(const Eigen::Vector3d& query,
                                          Eigen::Vector3d& normal,
                                          Eigen::Vector3d& centroid,
                                          double& residual) const {
    // Search 3x3x3 neighborhood
    int ix = static_cast<int>(std::floor(query.x() / local_map_voxel_));
    int iy = static_cast<int>(std::floor(query.y() / local_map_voxel_));
    int iz = static_cast<int>(std::floor(query.z() / local_map_voxel_));

    std::vector<Eigen::Vector3f> neighbors;
    for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
    for (int dz = -1; dz <= 1; ++dz) {
        VoxelKey k{ix+dx, iy+dy, iz+dz};
        auto it = local_map_.find(k);
        if (it != local_map_.end()) {
            for (const auto& p : it->second) neighbors.push_back(p);
        }
    }

    if (neighbors.size() < 5) return false;

    // Compute centroid and covariance → PCA for normal
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    for (const auto& p : neighbors) c += p.cast<double>();
    c /= neighbors.size();

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
        Eigen::Vector3d d = p.cast<double>() - c;
        cov += d * d.transpose();
    }
    cov /= neighbors.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    normal   = es.eigenvectors().col(0);  // eigenvector of smallest eigenvalue
    centroid = c;
    residual = normal.dot(query - c);

    // Check planarity: ratio of smallest to second eigenvalue
    double planarity = es.eigenvalues()(0) / (es.eigenvalues()(1) + 1e-10);
    return planarity < 0.1;
}

void FastLIVO2Wrapper::publishPose(double timestamp, const Pose3d& pose, const Mat66d& /*cov*/) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = static_cast<int32_t>(std::floor(timestamp));
    odom.header.stamp.nanosec = static_cast<uint32_t>(std::round((timestamp - std::floor(timestamp)) * 1e9));
    odom.header.frame_id = "world";
    odom.child_frame_id  = "body";

    const auto& p = pose.translation();
    Eigen::Quaterniond q(pose.rotation());
    odom.pose.pose.position.x    = p.x();
    odom.pose.pose.position.y    = p.y();
    odom.pose.pose.position.z    = p.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    pose_pub_->publish(odom);

    // Path
    geometry_msgs::msg::PoseStamped ps;
    ps.header = odom.header;
    ps.pose   = odom.pose.pose;
    path_msg_.header = odom.header;
    path_msg_.poses.push_back(ps);
    if (path_msg_.poses.size() > 10000) path_msg_.poses.erase(path_msg_.poses.begin());
    path_pub_->publish(path_msg_);

    for (auto& cb : pose_callbacks_) {
        cb(timestamp, pose, Mat66d::Identity() * 1e-4);
    }
}

const ESIKFState& FastLIVO2Wrapper::currentState() const {
    return state_;
}

bool FastLIVO2Wrapper::isInitialized() const {
    return initialized_;
}

void FastLIVO2Wrapper::reset() {
    std::lock_guard<std::mutex> lk(state_mutex_);
    state_      = ESIKFState();
    initialized_ = false;
    last_imu_time_ = -1.0;
    {
        std::lock_guard<std::mutex> ml(map_mutex_);
        local_map_.clear();
    }
    kf_manager_->reset();
}

}  // namespace automap_pro
