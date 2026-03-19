// 模块2: 数据流回调
// 包含: onOdometry, onCloud, onKFInfo, onGPS, odomCacheAdd, odomCacheGet, kfinfoCacheAdd, kfinfoCacheGet

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/logger.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 数据流：里程计 → 关键帧 → 子图
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    static std::atomic<int> backend_odom_recv_count{0};
    const int seq = backend_odom_recv_count.fetch_add(1) + 1;

    ALOG_TRACE_STEP("AutoMapSystem", "onOdometry_enter");

    if (seq <= 5 || seq % 500 == 0) {
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][BACKEND][RECV] odom #%d ts=%.3f pos=[%.2f,%.2f,%.2f] (backend entry)",
            seq, ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
        double pos_std_x = std::isfinite(cov(3, 3)) ? std::sqrt(std::max(0.0, cov(3, 3))) : 0.0;
        double pos_std_y = std::isfinite(cov(4, 4)) ? std::sqrt(std::max(0.0, cov(4, 4))) : 0.0;
        double pos_std_z = std::isfinite(cov(5, 5)) ? std::sqrt(std::max(0.0, cov(5, 5))) : 0.0;
        RCLCPP_INFO(get_logger(),
            "[PRECISION][ODOM] seq=%d ts=%.4f pos_std_xyz=[%.4f,%.4f,%.4f] pos=[%.3f,%.3f,%.3f]",
            seq, ts, pos_std_x, pos_std_y, pos_std_z,
            pose.translation().x(), pose.translation().y(), pose.translation().z());
    }
    if (!first_odom_logged_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA] First odometry received ts=%.3f pos=[%.2f,%.2f,%.2f]", ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=first_odom ts=%.3f", ts);
    }
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_odom_pose_ = pose;
        last_odom_ts_   = ts;
        last_cov_       = cov;
    }
    odomCacheAdd(ts, pose, cov);

    gps_manager_.addKeyFramePose(ts, pose);

    // 🔧 V2 修复：如果 HBA 已完成，停止发布里程计轨迹，避免与优化后的地图/轨迹冲突导致重影
    if (odom_path_stopped_after_hba_.load(std::memory_order_acquire)) {
        return;
    }

    // 🔧 V2 修复：使用 odom_path_mutex_ 保护轨迹追加与裁剪，防止 MultiThreadedExecutor 下的竞态
    std::lock_guard<std::mutex> lk(odom_path_mutex_);
    
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = now();
    ps.header.frame_id = "map";
    auto& pp           = ps.pose.position;
    pp.x = pose.translation().x();
    pp.y = pose.translation().y();
    pp.z = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    ps.pose.orientation.w = q.w(); ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z();
    
    odom_path_.poses.push_back(ps);
    odom_path_.header = ps.header;

    // 🔧 V2 修复：限制轨迹长度，避免长期运行内存爆炸
    if (odom_path_.poses.size() > 10000) {
        odom_path_.poses.erase(odom_path_.poses.begin(), odom_path_.poses.begin() + 1000);
    }

    if (seq <= 20 || seq % 100 == 0) {
        rviz_publisher_.publishOdometryPath(odom_path_);
        pub_odom_path_count_++;
    }

    if (trajectory_log_enabled_ && !trajectory_log_after_mapping_only_) writeTrajectoryOdom(ts, pose, cov);

    if (attitude_estimator_) {
        const Eigen::Matrix3d& R = pose.linear();
        double yaw = std::atan2(R(1, 0), R(0, 0));
        attitude_estimator_->addOdometryYaw(ts, yaw);
    }
}

void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud) {
    ALOG_TRACE_STEP("AutoMapSystem", "onCloud_enter");

    const size_t pts = cloud ? cloud->size() : 0u;
    (void)pts;
    if (!first_cloud_logged_.exchange(true)) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][DATA][CLOUD] First cloud received ts=%.3f pts=%zu", ts, pts);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=first_cloud ts=%.3f pts=%zu", ts, pts);
    }
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_cloud_    = cloud;
        last_cloud_ts_ = ts;
    }
    last_sensor_data_wall_time_ = std::chrono::steady_clock::now();

    // 使用 FrameProcessor 处理帧
    const size_t queue_before = frame_processor_.frameQueueSize();
    const bool push_success = frame_processor_.pushFrame(ts, cloud);
    const size_t queue_after = frame_processor_.frameQueueSize();

    if (!push_success) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][DATA][CLOUD][DROP] pushFrame failed ts=%.3f pts=%zu queue_before=%zu", ts, pts, queue_before);
    } else if (queue_after > queue_before) {
        RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][DATA][CLOUD] pushed ts=%.3f pts=%zu queue=%zu->%zu", ts, pts, queue_before, queue_after);
    }
}

void AutoMapSystem::onKFInfo(const LivoKeyFrameInfo& info) {
    static std::atomic<int> backend_kfinfo_recv_count{0};
    const int seq = backend_kfinfo_recv_count.fetch_add(1) + 1;
    if (seq <= 5 || seq % 20 == 0) {
    }
    { std::lock_guard<std::mutex> lk(data_mutex_); last_livo_info_ = info; }
    kfinfoCacheAdd(info.timestamp, info);
}

void AutoMapSystem::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats) {
    std::lock_guard<std::mutex> lk(gps_queue_mutex_);
    if (gps_queue_.size() < kMaxGPSQueueSize) {
        gps_queue_.push_back({ts, lat, lon, alt, hdop, sats});
        gps_queue_cv_.notify_one();
    } else {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][GPS] queue full, dropping GPS data");
    }
}

void AutoMapSystem::odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    const bool was_empty = odom_cache_.empty();
    odom_cache_.push_back({ts, pose, cov});
    while (odom_cache_.size() > kMaxOdomCacheSize) odom_cache_.pop_front();
    if (was_empty) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] first odom cached ts=%.3f (worker can now match cloud by ts)", ts);
    }
}

bool AutoMapSystem::odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    if (odom_cache_.empty()) return false;
    const OdomCacheEntry* best = nullptr;
    for (auto it = odom_cache_.rbegin(); it != odom_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) return false;
    out_pose = best->pose;
    out_cov  = best->cov;
    return true;
}

void AutoMapSystem::kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    kfinfo_cache_.push_back({ts, info});
    while (kfinfo_cache_.size() > kMaxKFinfoCacheSize) kfinfo_cache_.pop_front();
}

bool AutoMapSystem::kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    if (kfinfo_cache_.empty()) return false;
    const KFinfoCacheEntry* best = nullptr;
    for (auto it = kfinfo_cache_.rbegin(); it != kfinfo_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) return false;
    out_info = best->info;
    return true;
}

}  // namespace automap_pro
