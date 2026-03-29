#pragma once

#include <memory>
#include <mutex>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/frontend/keyframe_manager.h"

namespace automap_pro {

/**
 * 订阅 fast-livo2-humble 发布的 Odometry 与点云，转换为 automap_pro 内部
 * KeyFrame 与位姿回调，使下游 SubmapManager / LoopDetector / HBA 无需改动。
 * 关键帧策略与 KeyFrameManager 一致（平移/旋转/间隔阈值）。
 */
class FastLIVO2Adapter {
public:
    using PoseCallback     = std::function<void(double, const Pose3d&, const Mat66d&)>;
    using KeyFrameCallback = std::function<void(const KeyFrame::Ptr&)>;

    FastLIVO2Adapter();
    ~FastLIVO2Adapter();

    void init(rclcpp::Node::SharedPtr node);
    void registerPoseCallback(PoseCallback cb);
    void registerKeyFrameCallback(KeyFrameCallback cb);

    void setSessionId(uint64_t session_id) { session_id_ = session_id; }
    void setCloudDownsampleResolution(double res) { cloud_ds_res_ = res; }
    void setKeyFramePolicy(double min_translation, double min_rotation_deg, double max_interval);

    bool isInitialized() const { return initialized_; }

private:
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onCloudRegistered(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    Pose3d odometryToPose(const nav_msgs::msg::Odometry::SharedPtr msg) const;
    CloudXYZIPtr cloudMsgToPcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

    /** 🏛️ [无损增密方案] 缓存每一帧原始雷达扫描及其对应的位姿，用于关键帧触发时生成稠密局部地图 */
    struct RawSweep {
        double timestamp;
        CloudXYZIPtr cloud;
        Pose3d T_odom_b;
    };
    void addSweepToBuffer(double ts, const CloudXYZIPtr& cloud, const Pose3d& pose);
    CloudXYZIPtr accumulateSweeps(const Pose3d& T_curr_kf);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    std::unique_ptr<KeyFrameManager> kf_manager_;
    std::vector<PoseCallback> pose_callbacks_;
    std::vector<KeyFrameCallback> kf_callbacks_;

    mutable std::mutex state_mutex_;
    bool initialized_ = false;
    Pose3d last_odom_pose_;
    double last_odom_time_ = -1e9;
    
    std::deque<RawSweep> sweep_buffer_; // 高频扫描缓冲区
    int max_sweep_buffer_size_ = 20;    // 默认缓存最近 20 帧高频扫描（约 1-2 秒数据）

    CloudXYZIPtr last_cloud_;   // 与 last_odom_time_ 对齐，用于关键帧
    double cloud_ds_res_ = 0.2;
    uint64_t session_id_ = 0;

    uint64_t odom_count_ = 0;
    uint64_t cloud_count_ = 0;
    double last_log_time_ = -1e9;
    static constexpr double kDataFlowLogInterval = 2.0;
};

}  // namespace automap_pro
