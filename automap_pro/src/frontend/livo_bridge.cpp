#include "automap_pro/frontend/livo_bridge.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "LivoBridge"

#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

namespace automap_pro {

LivoBridge::LivoBridge() = default;

void LivoBridge::init(rclcpp::Node::SharedPtr node) {
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

    // 点云（中频，RELIABLE 保证不丢帧）
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.fastLivoCloudTopic(), reliable_qos,
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
        "[LivoBridge] Initialized (odom=%s, cloud=%s, kfinfo=%s, gps=%s)",
        cfg.fastLivoOdomTopic().c_str(),
        cfg.fastLivoCloudTopic().c_str(),
        cfg.fastLivoKFInfoTopic().c_str(),
        cfg.gpsEnabled() ? cfg.gpsTopic().c_str() : "disabled");
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

    if (odom_count_ % 100 == 0) {
        ALOG_DEBUG(MOD, "Odom#{}: ts={:.3f} t=[{:.2f},{:.2f},{:.2f}]",
                   odom_count_, ts,
                   pose.translation().x(), pose.translation().y(), pose.translation().z());
    }
    for (auto& cb : odom_cbs_) cb(ts, pose, cov);
}

void LivoBridge::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_count_++;
    double ts = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;

    CloudXYZIPtr cloud(new CloudXYZI);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
        ALOG_WARN(MOD, "Empty cloud at ts={:.3f}", ts);
        return;
    }
    if (cloud_count_ % 50 == 0) {
        ALOG_DEBUG(MOD, "Cloud#{}: pts={} ts={:.3f}", cloud_count_, cloud->size(), ts);
    }
    for (auto& cb : cloud_cbs_) cb(ts, cloud);
}

void LivoBridge::onKFInfo(const automap_pro::msg::KeyFrameInfoMsg::SharedPtr msg) {
    LivoKeyFrameInfo info;
    info.timestamp        = msg->timestamp;
    info.esikf_cov_norm   = msg->esikf_covariance_norm;
    info.is_degenerate    = msg->is_degenerate;
    info.cloud_point_count = msg->cloud_point_count;
    info.cloud_valid       = msg->cloud_valid;
    info.gyro_bias  = Eigen::Vector3d(msg->gyro_bias[0], msg->gyro_bias[1], msg->gyro_bias[2]);
    info.accel_bias = Eigen::Vector3d(msg->accel_bias[0], msg->accel_bias[1], msg->accel_bias[2]);

    for (auto& cb : kfinfo_cbs_) cb(info);
}

void LivoBridge::onGPS(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status < 0) return;  // GPS fix 无效

    double ts  = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    double lat = msg->latitude;
    double lon = msg->longitude;
    double alt = msg->altitude;

    // 从协方差矩阵估算 HDOP
    double sigma_h = std::sqrt(msg->position_covariance[0]);  // east variance
    double hdop    = std::max(0.5, sigma_h / 0.3);  // 0.3m/HDOP 估算

    int sats = 0;  // NavSatFix 不包含卫星数，使用默认值

    for (auto& cb : gps_cbs_) cb(ts, lat, lon, alt, hdop, sats);
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
