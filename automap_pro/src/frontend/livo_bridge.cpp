#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "LivoBridge"

#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

namespace automap_pro {

LivoBridge::LivoBridge() = default;

void LivoBridge::init(rclcpp::Node::SharedPtr node) {
    if (!node) {
        throw std::invalid_argument("LivoBridge::init: node is null");
    }
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    // 订阅端缓存足够大：接收到即放入 buffer，避免 executor 繁忙时 DDS 丢包（SensorDataQoS 默认 depth=5 易丢）
    constexpr int kOdomKfinfoDepth = 200000;  // 里程计/关键帧信息与点云同频，缓冲与 frame_queue 量级一致
    constexpr int kCloudDepth = 200000;       // 点云：与 frame_queue_max_size(1500) 对齐并留余量
    constexpr int kGpsDepth = 50000;          // GPS 低频，适度缓冲即可
    auto odom_kfinfo_qos = rclcpp::QoS(rclcpp::KeepLast(kOdomKfinfoDepth)).best_effort();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(kCloudDepth)).reliable();
    auto gps_qos = rclcpp::QoS(rclcpp::KeepLast(kGpsDepth)).reliable();

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        cfg.fastLivoOdomTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onOdometry, this, std::placeholders::_1));

    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.fastLivoCloudTopic(), cloud_qos,
        std::bind(&LivoBridge::onCloud, this, std::placeholders::_1));

    kfinfo_sub_ = node->create_subscription<automap_pro::msg::KeyFrameInfoMsg>(
        cfg.fastLivoKFInfoTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onKFInfo, this, std::placeholders::_1));

    gps_topic_ = cfg.gpsTopic();
    gps_msg_count_.store(0);
    if (cfg.gpsEnabled()) {
        gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
            cfg.gpsTopic(), gps_qos,
            std::bind(&LivoBridge::onGPS, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(),
            "[LivoBridge][GPS] Subscription created: topic=%s QoS=RELIABLE KeepLast(%d)",
            cfg.gpsTopic().c_str(), kGpsDepth);
        gps_diag_timer_ = node->create_wall_timer(
            std::chrono::seconds(45),
            [this]() {
                if (gps_msg_count_.load() == 0 && gps_sub_) {
                    RCLCPP_WARN(node_->get_logger(),
                        "[LivoBridge][GPS_DIAG] Still 0 NavSatFix messages on topic=%s after 45s. Check: ros2 bag info <bag_dir>; topic type must be sensor_msgs/msg/NavSatFix.",
                        gps_topic_.c_str());
                }
            });
    } else {
        RCLCPP_WARN(node->get_logger(),
            "[LivoBridge][GPS] GPS disabled (sensor.gps.enabled=false). Trajectory CSV will have no GPS columns filled.");
    }

    connected_ = true;
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][TOPIC] subscribe: odom=%s (KeepLast(%d) best_effort) cloud=%s (KeepLast(%d) reliable) kfinfo=%s gps=%s",
        cfg.fastLivoOdomTopic().c_str(), kOdomKfinfoDepth,
        cfg.fastLivoCloudTopic().c_str(), kCloudDepth,
        cfg.fastLivoKFInfoTopic().c_str(),
        cfg.gpsEnabled() ? cfg.gpsTopic().c_str() : "disabled");
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][FLOW] fast_livo publishes odom then cloud per frame; backend triggers KF on cloud (pose=last_odom)");
}

void LivoBridge::init(rclcpp::Node::SharedPtr node, bool gps_enabled, const std::string& gps_topic) {
    if (!node) {
        throw std::invalid_argument("LivoBridge::init: node is null");
    }
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    // 与 init(node) 一致：前端接收传感器数据大幅度增加 buff，来一帧进一帧
    constexpr int kOdomKfinfoDepth = 200000;
    constexpr int kCloudDepth = 200000;
    constexpr int kGpsDepth = 50000;
    auto odom_kfinfo_qos = rclcpp::QoS(rclcpp::KeepLast(kOdomKfinfoDepth)).best_effort();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(kCloudDepth)).reliable();
    auto gps_qos = rclcpp::QoS(rclcpp::KeepLast(kGpsDepth)).reliable();

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        cfg.fastLivoOdomTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onOdometry, this, std::placeholders::_1));

    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.fastLivoCloudTopic(), cloud_qos,
        std::bind(&LivoBridge::onCloud, this, std::placeholders::_1));

    kfinfo_sub_ = node->create_subscription<automap_pro::msg::KeyFrameInfoMsg>(
        cfg.fastLivoKFInfoTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onKFInfo, this, std::placeholders::_1));

    std::string topic = gps_topic.empty() ? cfg.gpsTopic() : gps_topic;
    gps_topic_ = topic;
    gps_msg_count_.store(0);
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][GPS] init received gps_enabled=%s topic=%s (from AutoMapSystem/ConfigManager)",
        gps_enabled ? "true" : "false", topic.c_str());
    if (gps_enabled) {
        gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
            topic, gps_qos,
            std::bind(&LivoBridge::onGPS, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(),
            "[LivoBridge][GPS] Subscription created: topic=%s QoS=RELIABLE KeepLast(%d) (config passed from AutoMapSystem)",
            topic.c_str(), kGpsDepth);
        RCLCPP_INFO(node->get_logger(),
            "[LivoBridge][GPS_DIAG] Subscribed to %s (NavSatFix). If no \"[LivoBridge][GPS] First GPS message\" within ~60s, check: ros2 bag info <bag_dir> has this topic with type sensor_msgs/msg/NavSatFix; M2DGR bag uses /ublox/fix.",
            topic.c_str());
        // 延迟诊断：约 45s 后若仍无任何消息，打一次 WARN 便于排查“无 GPS 数据”
        gps_diag_timer_ = node->create_wall_timer(
            std::chrono::seconds(45),
            [this]() {
                if (gps_msg_count_.load() == 0 && gps_sub_) {
                    RCLCPP_WARN(node_->get_logger(),
                        "[LivoBridge][GPS_DIAG] Still 0 NavSatFix messages on topic=%s after 45s. Possible causes: (1) Bag has no such topic or 0 messages - run 'ros2 bag info <bag_dir>' and check topic list; (2) Bag has ublox_msgs (e.g. /ublox/navsat) which rosbag2 ignores - need sensor_msgs/NavSatFix on %s; (3) QoS mismatch (we use RELIABLE). Trajectory CSV will have no GPS columns.",
                        gps_topic_.c_str(), gps_topic_.c_str());
                }
            });
    } else {
        RCLCPP_WARN(node->get_logger(),
            "[LivoBridge][GPS] GPS disabled (sensor.gps.enabled=false from config). Trajectory CSV will have no GPS columns filled.");
    }

    connected_ = true;
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][TOPIC] subscribe: odom=%s (KeepLast(%d) best_effort) cloud=%s (KeepLast(%d) reliable) kfinfo=%s gps=%s",
        cfg.fastLivoOdomTopic().c_str(), kOdomKfinfoDepth,
        cfg.fastLivoCloudTopic().c_str(), kCloudDepth,
        cfg.fastLivoKFInfoTopic().c_str(),
        gps_enabled ? topic.c_str() : "disabled");
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][FLOW] fast_livo publishes odom then cloud per frame; backend triggers KF on cloud (pose=last_odom)");
}

void LivoBridge::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_count_++;
    double ts = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;

    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_odom_ts_ = ts;
    }

    Pose3d pose = poseFromOdom(*msg);
    Mat66d cov  = covFromOdom(*msg);

    const int o = odom_count_.load();
    if (o <= 5) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] odom #%d ts=%.3f pos=[%.2f,%.2f,%.2f]",
            o, ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
    } else if (o % 500 == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] odom #%d ts=%.3f pos=[%.2f,%.2f,%.2f]",
            o, ts, pose.translation().x(), pose.translation().y(), pose.translation().z());
    }
    if (odom_count_ % 100 == 0) {
        ALOG_DEBUG(MOD, "Odom#{}: ts={:.3f} t=[{:.2f},{:.2f},{:.2f}]",
                   odom_count_, ts,
                   pose.translation().x(), pose.translation().y(), pose.translation().z());
    }
    for (auto& cb : odom_cbs_) {
        try {
            cb(ts, pose, cov);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] odom callback ts=%.3f: %s (next odom will still be delivered)", ts, e.what());
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] odom callback ts=%.3f: unknown exception", ts);
        }
    }
}

void LivoBridge::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    using Clock = std::chrono::steady_clock;
    const auto recv_wall = Clock::now();
    cloud_count_++;
    double ts = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;

    CloudXYZIPtr cloud(new CloudXYZI);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
        empty_cloud_count_++;
        const int ec = empty_cloud_count_.load();
        if (ec <= 10 || ec % 50 == 0) {
            RCLCPP_WARN(node_->get_logger(), "[LivoBridge][DATA] empty cloud discarded #%d total_cloud_msg=%d ts=%.3f (backend will not see this frame)",
                ec, cloud_count_.load(), ts);
        }
        ALOG_WARN(MOD, "Empty cloud at ts={:.3f} count={}", ts, cloud_count_.load());
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_cloud_ts_ = ts;
    }
    const int c = cloud_count_.load();
    // 每帧都打一条，便于与 fast_livo [PUB] 对照，排查「前端发了后端没收到」
    static auto s_last_recv_wall = recv_wall;
    double delta_ms = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(recv_wall - s_last_recv_wall).count();
    s_last_recv_wall = recv_wall;
    RCLCPP_DEBUG(node_->get_logger(),
        "[LivoBridge][RECV] cloud #%d ts=%.3f pts=%zu delta_recv_ms=%.0f",
        c, ts, cloud->size(), c > 1 ? delta_ms : 0.0);
    if (c <= 3 || c % 500 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][DATA] cloud #%d ts=%.3f pts=%zu", c, ts, cloud->size());
    }
    RCLCPP_DEBUG(node_->get_logger(), "[LivoBridge][FRAME] #%d ts=%.3f pts=%zu → backend", c, ts, cloud->size());
    if (cloud_count_ % 50 == 0) {
        ALOG_DEBUG(MOD, "Cloud#{}: pts={} ts={:.3f}", cloud_count_, cloud->size(), ts);
    }
    for (auto& cb : cloud_cbs_) {
        try {
            cb(ts, cloud);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] cloud callback #%d ts=%.3f: %s (next frames will still be delivered)", c, ts, e.what());
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] cloud callback #%d ts=%.3f: unknown exception", c, ts);
        }
    }
}

void LivoBridge::onKFInfo(const automap_pro::msg::KeyFrameInfoMsg::SharedPtr msg) {
    static std::atomic<uint32_t> kfinfo_count{0};
    kfinfo_count++;

    LivoKeyFrameInfo info;
    info.timestamp        = msg->timestamp;
    info.esikf_cov_norm   = msg->esikf_covariance_norm;
    info.is_degenerate    = msg->is_degenerate;
    info.cloud_point_count = msg->cloud_point_count;
    info.cloud_valid       = msg->cloud_valid;
    info.gyro_bias  = Eigen::Vector3d(msg->gyro_bias[0], msg->gyro_bias[1], msg->gyro_bias[2]);
    info.accel_bias = Eigen::Vector3d(msg->accel_bias[0], msg->accel_bias[1], msg->accel_bias[2]);

    if (kfinfo_count == 1 || (kfinfo_count % 200 == 0)) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] kfinfo count=%u ts=%.3f cov_norm=%.4f degen=%d",
            kfinfo_count.load(), info.timestamp, info.esikf_cov_norm, info.is_degenerate ? 1 : 0);
    }
    for (auto& cb : kfinfo_cbs_) {
        try {
            cb(info);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] kfinfo callback ts=%.3f: %s", info.timestamp, e.what());
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] kfinfo callback ts=%.3f: unknown exception", info.timestamp);
        }
    }
}

void LivoBridge::onGPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    gps_msg_count_++;
    // GPS 时有时无均正常接收：仅 status<0（无 fix）丢弃；有 fix 则转发，下游 GPSManager 按质量门控使用
    static std::atomic<uint32_t> gps_first_seen{0};
    if (gps_first_seen++ == 0) {
        double ts = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][GPS] First GPS message (any status): status=%d ts=%.3f (valid fix requires status>=0; intermittent GPS is normal)",
            static_cast<int>(msg->status.status), ts);
    }

    if (msg->status.status < 0) {
        // 无 fix 时不转发（不影响建图）；首条与每 50 条打印，避免刷屏
        static std::atomic<uint32_t> gps_dropped{0};
        uint32_t d = gps_dropped++;
        if (d == 0) {
            RCLCPP_INFO(node_->get_logger(),
                "[LivoBridge][GPS] No fix (status=%d); skipping. Valid fixes will be used when available.",
                static_cast<int>(msg->status.status));
        } else if (d % 50 == 0) {
            RCLCPP_DEBUG(node_->get_logger(),
                "[LivoBridge][GPS] no-fix count=%u (status=%d)", d + 1, static_cast<int>(msg->status.status));
        }
        return;
    }

    static std::atomic<uint32_t> gps_count{0};
    uint32_t c = gps_count++;

    double ts  = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    double sigma_h = std::sqrt(std::max(0.0, static_cast<double>(msg->position_covariance[0])));
    double hdop    = std::max(0.5, sigma_h / 0.3);
    int sats = 0;

    if (c == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][GPS] First valid GPS received: ts=%.3f lat=%.6f lon=%.6f alt=%.1f hdop=%.2f → forwarding to GPSManager (trajectory_gps_*.csv will be created)",
            ts, lat, lon, alt, hdop);
    } else if (c % 50 == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] gps count=%u ts=%.3f lat=%.6f lon=%.6f alt=%.1f hdop=%.2f",
            c + 1, ts, lat, lon, alt, hdop);
    }
    for (auto& cb : gps_cbs_) {
        try {
            cb(ts, lat, lon, alt, hdop, sats);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] gps callback ts=%.3f: %s", ts, e.what());
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][EXCEPTION] gps callback ts=%.3f: unknown exception", ts);
        }
    }
}

Pose3d LivoBridge::poseFromOdom(const nav_msgs::msg::Odometry& msg) {
    const auto& p = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    Pose3d T = Pose3d::Identity();
    T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    T.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    return T;
}

Mat66d LivoBridge::covFromOdom(const nav_msgs::msg::Odometry& msg) {
    Mat66d cov = Mat66d::Identity() * 1e-4;
    const auto& c = msg.pose.covariance;
    if (c.size() >= 36) {
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                cov(i,j) = c[i*6+j];
    }
    return cov;
}

} // namespace automap_pro
