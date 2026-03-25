#pragma once

#include "automap_pro/core/data_types.h"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <automap_pro/msg/key_frame_info_msg.hpp>

#include <mutex>
#include <atomic>

namespace automap_pro {

/**
 * FAST-LIVO2 数据桥接器
 *
 * 支持两种接入模式：
 *
 *   Mode A（Composable Node，推荐）：
 *     fast_livo 和 automap_system 在同一 Component Container 中运行。
 *     rclcpp 的 Intra-Process Communication 自动启用零拷贝。
 *     话题订阅使用 TRANSIENT_LOCAL QoS。
 *
 *   Mode B（独立进程，兼容）：
 *     fast_livo 作为独立进程运行，通过 ROS2 DDS 话题传递数据。
 *     与 Mode A 代码完全相同，仅 QoS 不同。
 *
 * 新增：KeyFrameInfoMsg 订阅
 *   fast_livo 发布扩展关键帧信息（ESIKF 协方差、IMU bias）。
 *   automap_pro 据此自适应调整关键帧选取策略：
 *   - 当 esikf_cov_norm 高时（定位不稳定）→ 降低关键帧阈值，强制插入
 *   - 当 is_degenerate=true 时 → 暂停回环检测
 */
class LivoBridge {
public:
    LivoBridge();
    ~LivoBridge() = default;

    /** 仅用 node 初始化，GPS 使能/话题从 ConfigManager 读取（可能与 loadConfigAndInit 时序不一致） */
    void init(rclcpp::Node::SharedPtr node);
    /** 推荐：由 AutoMapSystem 传入已加载的 GPS 配置，避免与 ConfigManager 读值不一致导致 GPS 被误关 */
    void init(rclcpp::Node::SharedPtr node, bool gps_enabled, const std::string& gps_topic);

    // ── 回调注册 ──────────────────────────────────────────────────────────

    /** 里程计回调（高频，10-20Hz） */
    using OdomCallback = std::function<void(double ts, const Pose3d&, const Mat66d& cov)>;
    void registerOdomCallback(OdomCallback cb) { odom_cbs_.push_back(std::move(cb)); }

    /** 点云回调（与里程计同步，10Hz） */
    using CloudCallback = std::function<void(double ts, const CloudXYZIPtr&)>;
    void registerCloudCallback(CloudCallback cb) { cloud_cbs_.push_back(std::move(cb)); }

    /** ESIKF 扩展信息回调（可选，10Hz） */
    using KFInfoCallback = std::function<void(const LivoKeyFrameInfo&)>;
    void registerKFInfoCallback(KFInfoCallback cb) { kfinfo_cbs_.push_back(std::move(cb)); }

    /** GPS 测量回调 */
    using GPSCallback = std::function<void(double ts, double lat, double lon, double alt,
                                            double hdop, int sats)>;
    void registerGPSCallback(GPSCallback cb) { gps_cbs_.push_back(std::move(cb)); }

    // ── 状态查询 ──────────────────────────────────────────────────────────
    bool isConnected()   const { return connected_.load(); }
    int  odomCount()     const { return odom_count_.load(); }
    int  cloudCount()    const { return cloud_count_.load(); }
    int  emptyCloudCount() const { return empty_cloud_count_.load(); }
    double lastOdomTs()  const { std::lock_guard<std::mutex> lk(mutex_); return last_odom_ts_; }
    double lastCloudTs() const { std::lock_guard<std::mutex> lk(mutex_); return last_cloud_ts_; }

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_sub_;
    rclcpp::Subscription<automap_pro::msg::KeyFrameInfoMsg>::SharedPtr kfinfo_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr     gps_sub_;

    mutable std::mutex mutex_;
    std::atomic<bool>  connected_{false};
    std::atomic<int>   odom_count_{0};
    std::atomic<int>   cloud_count_{0};
    std::atomic<int>   empty_cloud_count_{0};
    std::atomic<int>   pcl_conversion_error_count_{0};
    std::atomic<int>   invalid_point_cloud_count_{0};
    std::atomic<uint64_t> kfinfo_count_{0};
    std::atomic<uint64_t> kfinfo_invalid_ts_count_{0};
    std::atomic<uint64_t> kfinfo_degenerate_count_{0};
    std::atomic<double> kfinfo_last_valid_ts_{-1.0};
    std::atomic<int>   gps_msg_count_{0};   // 收到的 NavSatFix 消息总数（含无 fix），用于诊断“无 GPS 数据”
    double             last_odom_ts_ = 0.0;
    double             last_cloud_ts_ = 0.0;
    std::string        gps_topic_;         // 当前订阅的 GPS 话题名（诊断用）
    rclcpp::TimerBase::SharedPtr gps_diag_timer_;  // 延迟诊断：若超时仍 0 条则打 WARN
    rclcpp::TimerBase::SharedPtr kfinfo_diag_timer_;  // 延迟诊断：若长期 0 条，提示上游未发布

    std::vector<OdomCallback>   odom_cbs_;
    std::vector<CloudCallback>  cloud_cbs_;
    std::vector<KFInfoCallback> kfinfo_cbs_;
    std::vector<GPSCallback>    gps_cbs_;

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onKFInfo(const automap_pro::msg::KeyFrameInfoMsg::SharedPtr msg);
    void onGPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    static Pose3d poseFromOdom(const nav_msgs::msg::Odometry& msg);
    static Mat66d covFromOdom(const nav_msgs::msg::Odometry& msg);
};

} // namespace automap_pro
