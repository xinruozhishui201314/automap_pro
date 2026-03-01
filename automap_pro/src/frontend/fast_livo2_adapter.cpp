#include "automap_pro/frontend/fast_livo2_adapter.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

FastLIVO2Adapter::FastLIVO2Adapter()
    : kf_manager_(std::make_unique<KeyFrameManager>()) {
}

FastLIVO2Adapter::~FastLIVO2Adapter() = default;

void FastLIVO2Adapter::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    std::string odom_topic  = cfg.externalFastLivoOdomTopic();
    std::string cloud_topic = cfg.externalFastLivoCloudTopic();
    cloud_ds_res_ = cfg.cloudDownsampleResolution();

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&FastLIVO2Adapter::onOdometry, this, std::placeholders::_1));
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, 5, std::bind(&FastLIVO2Adapter::onCloudRegistered, this, std::placeholders::_1));

    pose_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/automap/odometry", 10);
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/automap/odom_path", 1);

    initialized_ = true;
    RCLCPP_INFO(node->get_logger(), "[FastLIVO2Adapter] Subscribed to %s, %s", odom_topic.c_str(), cloud_topic.c_str());
}

void FastLIVO2Adapter::registerPoseCallback(PoseCallback cb) {
    pose_callbacks_.push_back(std::move(cb));
}

void FastLIVO2Adapter::registerKeyFrameCallback(KeyFrameCallback cb) {
    kf_manager_->registerCallback(std::move(cb));
}

void FastLIVO2Adapter::setKeyFramePolicy(double min_translation, double min_rotation_deg, double max_interval) {
    kf_manager_->setMinTranslation(min_translation);
    kf_manager_->setMinRotationDeg(min_rotation_deg);
    kf_manager_->setMaxInterval(max_interval);
}

void FastLIVO2Adapter::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double t = static_cast<double>(msg->header.stamp.sec) + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    Pose3d pose = odometryToPose(msg);

    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        last_odom_pose_ = pose;
        last_odom_time_ = t;
    }

    Mat66d cov = Mat66d::Identity() * 1e-4;
    for (size_t i = 0; i < 36 && i < msg->pose.covariance.size(); ++i)
        cov(i / 6, i % 6) = msg->pose.covariance[i];

    nav_msgs::msg::Odometry odom_out;
    odom_out.header = msg->header;
    odom_out.child_frame_id = "body";
    odom_out.pose.pose = msg->pose.pose;
    odom_out.pose.covariance = msg->pose.covariance;
    pose_pub_->publish(odom_out);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = msg->header;
    ps.pose   = msg->pose.pose;
    path_msg_.header = msg->header;
    path_msg_.poses.push_back(ps);
    if (path_msg_.poses.size() > 10000) path_msg_.poses.erase(path_msg_.poses.begin());
    path_pub_->publish(path_msg_);

    for (auto& cb : pose_callbacks_) cb(t, pose, cov);

    if (!kf_manager_->shouldCreateKeyFrame(pose, t)) return;

    CloudXYZIPtr cloud_body = nullptr;
    CloudXYZIPtr cloud_ds   = nullptr;
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (last_cloud_ && !last_cloud_->empty()) {
            cloud_body = last_cloud_;
            cloud_ds   = utils::voxelDownsample(last_cloud_, cloud_ds_res_);
        }
    }
    if (!cloud_body || !cloud_ds) return;

    GPSMeasurement gps;
    KeyFrame::Ptr kf = kf_manager_->createKeyFrame(
        pose, pose, cov, t, cloud_body, cloud_ds, gps, false, session_id_);
    (void)kf;
}

void FastLIVO2Adapter::onCloudRegistered(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudXYZIPtr cloud = cloudMsgToPcl(msg);
    if (!cloud || cloud->empty()) return;
    std::lock_guard<std::mutex> lk(state_mutex_);
    last_cloud_ = cloud;
}

Pose3d FastLIVO2Adapter::odometryToPose(const nav_msgs::msg::Odometry::SharedPtr msg) const {
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    Pose3d T;
    T.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    T.linear()      = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    return T;
}

CloudXYZIPtr FastLIVO2Adapter::cloudMsgToPcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
    CloudXYZIPtr cloud(new CloudXYZI);
    pcl::fromROSMsg(*msg, *cloud);
    return cloud;
}

}  // namespace automap_pro
