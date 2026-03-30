/**
 * @file frontend/livo_bridge.cpp
 * @brief 前端与里程计适配实现。
 */
#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "LivoBridge"

#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cmath>

namespace automap_pro {

LivoBridge::LivoBridge() = default;

void LivoBridge::init(rclcpp::Node::SharedPtr node) {
    if (!node) {
        throw std::invalid_argument("LivoBridge::init: node is null");
    }
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    // 订阅端 depth 由配置控制，资源受限时可减小（system.subscription_odom_cloud_depth / subscription_gps_depth）
    const int odom_cloud_depth = cfg.subscriptionOdomCloudDepth();
    const int gps_depth = cfg.subscriptionGpsDepth();
    // V3: odom/kfinfo 与 cloud 同为 RELIABLE，避免 BEST_EFFORT 丢包导致 cloud 时刻在 odom 缓存中无法插值 → SyncedFrame 缺失、kf=0
    auto odom_kfinfo_qos = rclcpp::QoS(rclcpp::KeepLast(odom_cloud_depth)).reliable();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(odom_cloud_depth)).reliable();
    auto gps_qos = rclcpp::QoS(rclcpp::KeepLast(gps_depth)).reliable();

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        cfg.fastLivoOdomTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onOdometry, this, std::placeholders::_1));

    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.fastLivoCloudTopic(), cloud_qos,
        std::bind(&LivoBridge::onCloud, this, std::placeholders::_1));

    kfinfo_sub_ = node->create_subscription<automap_pro::msg::KeyFrameInfoMsg>(
        cfg.fastLivoKFInfoTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onKFInfo, this, std::placeholders::_1));
    kfinfo_diag_timer_ = node->create_wall_timer(
        std::chrono::seconds(30),
        [this]() {
            if (kfinfo_count_.load(std::memory_order_relaxed) == 0 && kfinfo_sub_) {
                RCLCPP_WARN(node_->get_logger(),
                    "[CHAIN][B1 LIVO->FE] kfinfo still 0 after 30s topic=%s. If this dataset/fast_livo has no keyframe_info publisher, set semantic.keyframes_only=false.",
                    ConfigManager::instance().fastLivoKFInfoTopic().c_str());
            }
        });

    gps_topic_ = cfg.gpsTopic();
    gps_msg_count_.store(0);
    if (cfg.gpsEnabled()) {
        gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
            cfg.gpsTopic(), gps_qos,
            std::bind(&LivoBridge::onGPS, this, std::placeholders::_1));
        RCLCPP_INFO(node->get_logger(),
            "[LivoBridge][GPS] Subscription created: topic=%s QoS=RELIABLE KeepLast(%d)",
            cfg.gpsTopic().c_str(), gps_depth);
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
        "[LivoBridge][TOPIC] subscribe: odom=%s (KeepLast(%d) reliable) cloud=%s (KeepLast(%d) reliable) kfinfo=%s gps=%s",
        cfg.fastLivoOdomTopic().c_str(), odom_cloud_depth,
        cfg.fastLivoCloudTopic().c_str(), odom_cloud_depth,
        cfg.fastLivoKFInfoTopic().c_str(),
        cfg.gpsEnabled() ? cfg.gpsTopic().c_str() : "disabled");
    RCLCPP_INFO(node->get_logger(),
        "[CHAIN][B1 CONFIG] odom_topic=%s cloud_topic=%s kfinfo_topic=%s",
        cfg.fastLivoOdomTopic().c_str(),
        cfg.fastLivoCloudTopic().c_str(),
        cfg.fastLivoKFInfoTopic().c_str());
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][FLOW] fast_livo publishes odom then cloud per frame; backend triggers KF on cloud (pose=last_odom)");
}

void LivoBridge::init(rclcpp::Node::SharedPtr node, bool gps_enabled, const std::string& gps_topic) {
    if (!node) {
        throw std::invalid_argument("LivoBridge::init: node is null");
    }
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    // 与 init(node) 一致：订阅 depth 由配置 system.subscription_odom_cloud_depth / subscription_gps_depth 控制
    const int odom_cloud_depth = cfg.subscriptionOdomCloudDepth();
    const int gps_depth = cfg.subscriptionGpsDepth();
    // V3: odom/kfinfo 与 cloud 同为 RELIABLE，避免 BEST_EFFORT 丢包导致 cloud 时刻在 odom 缓存中无法插值 → SyncedFrame 缺失、kf=0
    auto odom_kfinfo_qos = rclcpp::QoS(rclcpp::KeepLast(odom_cloud_depth)).reliable();
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(odom_cloud_depth)).reliable();
    auto gps_qos = rclcpp::QoS(rclcpp::KeepLast(gps_depth)).reliable();

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        cfg.fastLivoOdomTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onOdometry, this, std::placeholders::_1));

    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.fastLivoCloudTopic(), cloud_qos,
        std::bind(&LivoBridge::onCloud, this, std::placeholders::_1));

    kfinfo_sub_ = node->create_subscription<automap_pro::msg::KeyFrameInfoMsg>(
        cfg.fastLivoKFInfoTopic(), odom_kfinfo_qos,
        std::bind(&LivoBridge::onKFInfo, this, std::placeholders::_1));
    kfinfo_diag_timer_ = node->create_wall_timer(
        std::chrono::seconds(30),
        [this]() {
            if (kfinfo_count_.load(std::memory_order_relaxed) == 0 && kfinfo_sub_) {
                RCLCPP_WARN(node_->get_logger(),
                    "[CHAIN][B1 LIVO->FE] kfinfo still 0 after 30s topic=%s. If this dataset/fast_livo has no keyframe_info publisher, set semantic.keyframes_only=false.",
                    ConfigManager::instance().fastLivoKFInfoTopic().c_str());
            }
        });

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
            topic.c_str(), gps_depth);
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
        "[LivoBridge][TOPIC] subscribe: odom=%s (KeepLast(%d) reliable) cloud=%s (KeepLast(%d) reliable) kfinfo=%s gps=%s",
        cfg.fastLivoOdomTopic().c_str(), odom_cloud_depth,
        cfg.fastLivoCloudTopic().c_str(), odom_cloud_depth,
        cfg.fastLivoKFInfoTopic().c_str(),
        gps_enabled ? topic.c_str() : "disabled");
    RCLCPP_INFO(node->get_logger(),
        "[CHAIN][B1 CONFIG] odom_topic=%s cloud_topic=%s kfinfo_topic=%s",
        cfg.fastLivoOdomTopic().c_str(),
        cfg.fastLivoCloudTopic().c_str(),
        cfg.fastLivoKFInfoTopic().c_str());
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
    if (o <= 10 || o % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] odom #%d ts=%.3f pos=[%.2f,%.2f,%.2f] (grep LivoBridge DATA 可对照后端 RECV)",
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
    const int c_enter = cloud_count_.load();
    double ts = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;

    // 诊断：前 20 帧打 ENTER/EXIT，便于确认「回调是否被调度」（若 cloud #5 无 EXIT 则卡在回调内）
    if (c_enter <= 20) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][CALLBACK] onCloud ENTER #%d ts=%.3f (grep CALLBACK 可确认 Executor 是否交付)", c_enter, ts);
    }

    CloudXYZIPtr cloud(new CloudXYZI);
    try {
        pcl::fromROSMsg(*msg, *cloud);
    } catch (const std::exception& e) {
        pcl_conversion_error_count_++;
        RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][DATA] PCL conversion failed #%d ts=%.3f: %s",
            pcl_conversion_error_count_.load(), ts, e.what());
        if (c_enter <= 20) {
            RCLCPP_INFO(node_->get_logger(), "[LivoBridge][CALLBACK] onCloud EXIT #%d (pcl conversion failed)", c_enter);
        }
        return;
    }

    if (cloud->empty()) {
        empty_cloud_count_++;
        const int ec = empty_cloud_count_.load();
        if (ec <= 10 || ec % 50 == 0) {
            RCLCPP_WARN(node_->get_logger(), "[LivoBridge][DATA] empty cloud discarded #%d total_cloud_msg=%d ts=%.3f (backend will not see this frame)",
                ec, cloud_count_.load(), ts);
        }
        if (c_enter <= 20) {
            RCLCPP_INFO(node_->get_logger(), "[LivoBridge][CALLBACK] onCloud EXIT #%d (empty, discarded)", c_enter);
        }
        ALOG_WARN(MOD, "Empty cloud at ts={:.3f} count={}", ts, cloud_count_.load());
        return;
    }

    // 检查点云数据有效性：是否有NaN或Inf
    bool has_invalid_points = false;
    size_t nan_count = 0;
    for (const auto& pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            nan_count++;
            if (nan_count > 100) {  // 统计过多则跳过
                has_invalid_points = true;
                break;
            }
        }
    }
    if (has_invalid_points || nan_count > 0) {
        invalid_point_cloud_count_++;
        RCLCPP_WARN(node_->get_logger(), "[LivoBridge][DATA] cloud #%d ts=%.3f has %zu NaN/Inf points, filtering...",
            cloud_count_.load(), ts, nan_count);
        // 过滤掉无效点
        CloudXYZIPtr filtered(new CloudXYZI);
        filtered->reserve(cloud->size());
        for (const auto& pt : cloud->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                filtered->push_back(pt);
            }
        }
        if (filtered->empty()) {
            RCLCPP_ERROR(node_->get_logger(), "[LivoBridge][DATA] cloud #%d: all points invalid after filtering", cloud_count_.load());
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][DATA] cloud #%d filtered: %zu -> %zu points",
            cloud_count_.load(), cloud->size(), filtered->size());
        cloud = filtered;
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
    if (c <= 10 || c % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][DATA] cloud #%d ts=%.3f pts=%zu (grep LivoBridge DATA 可对照 fast_livo PUB)", c, ts, cloud->size());
    }
    if (c <= 20) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][CALLBACK] onCloud EXIT #%d ts=%.3f pts=%zu (grep CALLBACK 可确认回调返回)", c, ts, cloud->size());
    }
    if (c % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][HEARTBEAT] cloud_count=%d odom_count=%d last_cloud_ts=%.3f (grep LivoBridge HEARTBEAT 可确认收包存活)",
                    c, odom_count_.load(), ts);
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
    const auto kfinfo_count = kfinfo_count_.fetch_add(1, std::memory_order_relaxed) + 1;

    LivoKeyFrameInfo info;
    info.timestamp        = msg->timestamp;
    info.esikf_cov_norm   = msg->esikf_covariance_norm;
    info.is_degenerate    = msg->is_degenerate;
    info.cloud_point_count = msg->cloud_point_count;
    info.cloud_valid       = msg->cloud_valid;
    info.gyro_bias  = Eigen::Vector3d(msg->gyro_bias[0], msg->gyro_bias[1], msg->gyro_bias[2]);
    info.accel_bias = Eigen::Vector3d(msg->accel_bias[0], msg->accel_bias[1], msg->accel_bias[2]);

    const bool valid_ts = std::isfinite(info.timestamp) && info.timestamp > 0.0;
    if (!valid_ts) {
        kfinfo_invalid_ts_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[LivoBridge][KFINFO_DIAG] invalid_ts count=%lu ts=%.6f cloud_points=%u cloud_valid=%d",
            static_cast<unsigned long>(kfinfo_count), info.timestamp,
            static_cast<unsigned int>(info.cloud_point_count), info.cloud_valid ? 1 : 0);
    } else {
        kfinfo_last_valid_ts_.store(info.timestamp, std::memory_order_relaxed);
    }
    if (info.is_degenerate) {
        kfinfo_degenerate_count_.fetch_add(1, std::memory_order_relaxed);
    }

    if (kfinfo_count == 1 || (kfinfo_count % 200 == 0)) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] kfinfo count=%lu ts=%.3f cov_norm=%.4f degen=%d",
            static_cast<unsigned long>(kfinfo_count), info.timestamp, info.esikf_cov_norm, info.is_degenerate ? 1 : 0);
    }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "[LivoBridge][KFINFO_DIAG] total=%lu invalid_ts=%lu degenerate=%lu invalid_ratio=%.3f last_valid_ts=%.3f",
        static_cast<unsigned long>(kfinfo_count_.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(kfinfo_invalid_ts_count_.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(kfinfo_degenerate_count_.load(std::memory_order_relaxed)),
        static_cast<double>(kfinfo_invalid_ts_count_.load(std::memory_order_relaxed)) /
            static_cast<double>(std::max<uint64_t>(1, kfinfo_count_.load(std::memory_order_relaxed))),
        kfinfo_last_valid_ts_.load(std::memory_order_relaxed));

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

    double sigma_h = std::isfinite(msg->position_covariance[0]) ? std::sqrt(std::max(0.0, static_cast<double>(msg->position_covariance[0]))) : 1.0;
    double hdop    = std::max(0.5, sigma_h / 0.3);
    // NavSatFix 不携带卫星数；用 -1 表示 unknown，避免上游将其当成“卫星数不足”。
    int sats = -1;

    if (c == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][CALLBACK] onGPS ENTER first valid fix ts=%.3f (grep CALLBACK 可确认首条 GPS 回调是否阻塞)",
            ts);
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
    if (c == 0) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][CALLBACK] onGPS EXIT first valid fix ts=%.3f (若此后无 cloud #5 等，疑 Executor 未再调度)", ts);
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
