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

    // QoS：SensorData = BEST_EFFORT | VOLATILE，适合高频传感器数据
    // 当 Composable Node 同进程时，rclcpp 自动启用 Intra-Process Communication（零拷贝）
    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // 里程计（高频，BEST_EFFORT 降低延迟）
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        cfg.fastLivoOdomTopic(), sensor_qos,
        std::bind(&LivoBridge::onOdometry, this, std::placeholders::_1));

    // 点云：与 fast_livo 的 publish(..., 100) 一致，RELIABLE + KeepLast(100)，避免队列满丢帧
    auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.fastLivoCloudTopic(), cloud_qos,
        std::bind(&LivoBridge::onCloud, this, std::placeholders::_1));

    // ESIKF 扩展信息（与里程计同频）
    kfinfo_sub_ = node->create_subscription<automap_pro::msg::KeyFrameInfoMsg>(
        cfg.fastLivoKFInfoTopic(), sensor_qos,
        std::bind(&LivoBridge::onKFInfo, this, std::placeholders::_1));

    // GPS（低频，RELIABLE）
    if (cfg.gpsEnabled()) {
        gps_sub_ = node->create_subscription<sensor_msgs::msg::NavSatFix>(
            cfg.gpsTopic(), reliable_qos,
            std::bind(&LivoBridge::onGPS, this, std::placeholders::_1));
    }

    connected_ = true;
    RCLCPP_INFO(node->get_logger(),
        "[LivoBridge][TOPIC] subscribe: odom=%s (SensorData) cloud=%s (RELIABLE KeepLast(100), match frontend) kfinfo=%s gps=%s",
        cfg.fastLivoOdomTopic().c_str(),
        cfg.fastLivoCloudTopic().c_str(),
        cfg.fastLivoKFInfoTopic().c_str(),
        cfg.gpsEnabled() ? cfg.gpsTopic().c_str() : "disabled");
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
    RCLCPP_INFO(node_->get_logger(),
        "[LivoBridge][RECV] cloud #%d ts=%.3f pts=%zu delta_recv_ms=%.0f (if delta_recv_ms>>200 and frontend published more, executor may be busy or QoS mismatch)",
        c, ts, cloud->size(), c > 1 ? delta_ms : 0.0);
    if (c <= 5) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][DATA] cloud #%d ts=%.3f pts=%zu → trigger KF", c, ts, cloud->size());
    } else if (c % 100 == 0) {
        RCLCPP_INFO(node_->get_logger(), "[LivoBridge][DATA] cloud #%d ts=%.3f pts=%zu", c, ts, cloud->size());
    }
    RCLCPP_INFO(node_->get_logger(), "[LivoBridge][FRAME] #%d ts=%.3f pts=%zu → backend",
                c, ts, cloud->size());
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
    if (msg->status.status < 0) return;  // GPS fix 无效

    static std::atomic<uint32_t> gps_count{0};
    gps_count++;

    double ts  = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    double sigma_h = std::sqrt(msg->position_covariance[0]);
    double hdop    = std::max(0.5, sigma_h / 0.3);
    int sats = 0;

    if (gps_count == 1 || (gps_count % 50 == 0)) {
        RCLCPP_INFO(node_->get_logger(),
            "[LivoBridge][DATA] gps count=%u ts=%.3f lat=%.6f lon=%.6f alt=%.1f hdop=%.2f",
            gps_count.load(), ts, lat, lon, alt, hdop);
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
