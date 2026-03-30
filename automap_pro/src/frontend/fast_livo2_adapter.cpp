/**
 * @file frontend/fast_livo2_adapter.cpp
 * @brief 前端与里程计适配实现。
 */
#include "automap_pro/frontend/fast_livo2_adapter.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace automap_pro {

FastLIVO2Adapter::FastLIVO2Adapter()
    : kf_manager_(std::make_unique<KeyFrameManager>()) {
}

FastLIVO2Adapter::~FastLIVO2Adapter() = default;

void FastLIVO2Adapter::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    std::string odom_topic  = cfg.fastLivoOdomTopic();
    std::string cloud_topic = cfg.fastLivoCloudTopic();
    // 🔍 修复: 使用 submapMatchRes 作为默认降采样分辨率，或直接设为 0.0
    cloud_ds_res_ = 0.1; 
    
    max_sweep_buffer_size_ = cfg.frontendSweepAccumulationFrames();

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&FastLIVO2Adapter::onOdometry, this, std::placeholders::_1));
    cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, 5, std::bind(&FastLIVO2Adapter::onCloudRegistered, this, std::placeholders::_1));

    pose_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/automap/odometry", 10);
    path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/automap/odom_path", 1);

    initialized_ = true;
    RCLCPP_INFO(node->get_logger(),
        "[FastLIVO2Adapter][TOPIC] subscribe: %s, %s | publish: /automap/odometry, /automap/odom_path",
        odom_topic.c_str(), cloud_topic.c_str());
    RCLCPP_INFO(node->get_logger(),
        "[FastLIVO2Adapter][COORD] frontend.cloud_frame=%s (world: KF cloud_body = T_odom_b^-1 * registered; body: pass-through)",
        cfg.frontendCloudFrame().c_str());
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
    odom_count_++;
    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now - last_log_time_ >= kDataFlowLogInterval) {
        RCLCPP_INFO(node_->get_logger(),
                    "[DataFlow] FastLIVO2Adapter Odom | count=%lu | cloud_count=%lu | last_ts=%.3f",
                    static_cast<unsigned long>(odom_count_), static_cast<unsigned long>(cloud_count_), t);
        odom_count_ = 0;
        cloud_count_ = 0;
        last_log_time_ = now;
    }
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
        // 🏛️ [无损增密] 优先使用累加器生成的稠密局部点云
        CloudXYZIPtr dense_cloud = accumulateSweeps(pose);
        if (dense_cloud && !dense_cloud->empty()) {
            cloud_body = dense_cloud;
            // 🏛️ [逻辑加固] 创建关键帧后清空缓冲区，防止同一帧点云被重复计入多个关键帧
            sweep_buffer_.clear(); 
            RCLCPP_DEBUG(node_->get_logger(), "[FastLIVO2Adapter] Created KF with DENSE cloud: pts=%zu", cloud_body->size());
        } else if (last_cloud_ && !last_cloud_->empty()) {
            // 回退逻辑：如果累加器为空，使用最后一帧原始扫描
            const std::string& cf = ConfigManager::instance().frontendCloudFrame();
            CloudXYZIPtr src = last_cloud_;
            if (cf == "world") {
                CloudXYZIPtr body_from_world(new CloudXYZI());
                pcl::transformPointCloud(*last_cloud_, *body_from_world, pose.inverse().matrix().cast<float>());
                body_from_world->header = last_cloud_->header;
                body_from_world->width = static_cast<uint32_t>(body_from_world->size());
                body_from_world->height = 1;
                src = body_from_world;
            }
            cloud_body = src;
        }
        
        if (cloud_body) {
            cloud_ds = utils::voxelDownsample(cloud_body, cloud_ds_res_);
        }
    }
    if (!cloud_body || !cloud_ds) return;

    GPSMeasurement gps;
    KeyFrame::Ptr kf = kf_manager_->createKeyFrame(
        pose, cov, t, cloud_body, cloud_ds, gps, false, session_id_);
    (void)kf;
}

void FastLIVO2Adapter::onCloudRegistered(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    cloud_count_++;
    CloudXYZIPtr cloud = cloudMsgToPcl(msg);
    if (!cloud || cloud->empty()) return;
    
    std::lock_guard<std::mutex> lk(state_mutex_);
    last_cloud_ = cloud;
    
    // 🏛️ [无损增密] 将每一帧原始扫描存入缓冲区，并关联当前最接近的里程计位姿
    if (last_odom_time_ > 0) {
        addSweepToBuffer(last_odom_time_, cloud, last_odom_pose_);
    }
}

void FastLIVO2Adapter::addSweepToBuffer(double ts, const CloudXYZIPtr& cloud, const Pose3d& pose) {
    // 缓冲区管理：FIFO
    sweep_buffer_.push_back({ts, cloud, pose});
    while (sweep_buffer_.size() > static_cast<size_t>(max_sweep_buffer_size_)) {
        sweep_buffer_.pop_front();
    }
}

CloudXYZIPtr FastLIVO2Adapter::accumulateSweeps(const Pose3d& T_curr_kf) {
    if (sweep_buffer_.empty()) return nullptr;

    CloudXYZIPtr accumulated(new CloudXYZI());
    const Pose3d T_curr_inv = T_curr_kf.inverse();
    const std::string& cf = ConfigManager::instance().frontendCloudFrame();

    for (const auto& sweep : sweep_buffer_) {
        CloudXYZIPtr transformed(new CloudXYZI());
        
        // 坐标系转换逻辑：
        // 1. 如果源云是 world 系 (fast-livo 默认)，则需要 T_curr_kf^-1 * p_world 转到当前 body
        // 2. 如果源云是 body 系，则需要 T_curr_kf^-1 * T_sweep_odom * p_body 转到当前 body
        if (cf == "world") {
            pcl::transformPointCloud(*(sweep.cloud), *transformed, T_curr_inv.matrix().cast<float>());
        } else {
            Pose3d T_rel = T_curr_inv * sweep.T_odom_b;
            pcl::transformPointCloud(*(sweep.cloud), *transformed, T_rel.matrix().cast<float>());
        }
        
        *accumulated += *transformed;
    }
    
    // 保持 header 一致，以满足下游对时间戳的需求
    if (!accumulated->empty()) {
        accumulated->header = sweep_buffer_.back().cloud->header;
        accumulated->width = static_cast<uint32_t>(accumulated->size());
        accumulated->height = 1;
    }
    
    return accumulated;
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
