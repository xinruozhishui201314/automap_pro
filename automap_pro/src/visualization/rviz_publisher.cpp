#include "automap_pro/visualization/rviz_publisher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace automap_pro {

void RvizPublisher::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();
    global_map_ds_res_  = cfg.visGlobalMapDownsample();
    publish_global_map_ = cfg.visPublishRate() > 0.0;

    global_map_pub_    = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/global_map", 1);
    current_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/current_cloud", 1);
    submap_cloud_pub_  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/submap_cloud", 1);
    loop_marker_pub_   = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/loop_markers", 10);
    gps_marker_pub_    = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/gps_markers", 10);
    submap_boundary_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/submap_boundaries", 10);
    odom_path_pub_     = node->create_publisher<nav_msgs::msg::Path>("/automap/odom_path", 1);
    opt_path_pub_      = node->create_publisher<nav_msgs::msg::Path>("/automap/optimized_path", 1);

    RCLCPP_INFO(node->get_logger(), "[RvizPublisher] Initialized.");
}

void RvizPublisher::publishGlobalMap(const CloudXYZIPtr& cloud) {
    if (!publish_global_map_ || !cloud || cloud->empty()) return;
    auto ds = global_map_ds_res_ > 0 ?
              utils::voxelDownsample(cloud, global_map_ds_res_) : cloud;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*ds, msg);
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "world";
    global_map_pub_->publish(msg);
}

void RvizPublisher::publishCurrentCloud(const CloudXYZIPtr& cloud) {
    if (!cloud || cloud->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "world";
    current_cloud_pub_->publish(msg);
}

void RvizPublisher::publishSubmapCloud(const SubMap::Ptr& sm) {
    if (!sm || !sm->downsampled_cloud || sm->downsampled_cloud->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*sm->downsampled_cloud, msg);
    msg.header.stamp    = node_->now();
    msg.header.frame_id = "world";
    submap_cloud_pub_->publish(msg);
}

void RvizPublisher::publishLoopMarkers(
        const std::vector<LoopConstraint::Ptr>& loops,
        const std::vector<SubMap::Ptr>& submaps) {

    std::map<int, Eigen::Vector3d> sm_pos;
    for (const auto& sm : submaps) {
        sm_pos[sm->id] = sm->pose_w_anchor_optimized.translation();
    }

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& lc : loops) {
        auto it_i = sm_pos.find(lc->submap_i);
        auto it_j = sm_pos.find(lc->submap_j);
        if (it_i == sm_pos.end() || it_j == sm_pos.end()) continue;

        visualization_msgs::msg::Marker line;
        line.header.frame_id = "world";
        line.header.stamp    = node_->now();
        line.ns    = "loops";
        line.id    = id++;
        line.type  = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action= visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.1;
        line.color.r = lc->is_inter_session ? 1.0f : 0.0f;
        line.color.g = lc->is_inter_session ? 0.5f : 1.0f;
        line.color.b = 0.0f;
        line.color.a = 0.8f;

        geometry_msgs::msg::Point p1, p2;
        p1.x = it_i->second.x(); p1.y = it_i->second.y(); p1.z = it_i->second.z();
        p2.x = it_j->second.x(); p2.y = it_j->second.y(); p2.z = it_j->second.z();
        line.points.push_back(p1);
        line.points.push_back(p2);
        markers.markers.push_back(line);
    }
    loop_marker_pub_->publish(markers);
}

void RvizPublisher::publishGPSMarkers(const std::vector<SubMap::Ptr>& submaps) {
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& sm : submaps) {
        if (!sm->has_valid_gps) continue;
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp    = node_->now();
        sphere.ns    = "gps";
        sphere.id    = id++;
        sphere.type  = visualization_msgs::msg::Marker::SPHERE;
        sphere.action= visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = sm->gps_center.x();
        sphere.pose.position.y = sm->gps_center.y();
        sphere.pose.position.z = sm->gps_center.z();
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 2.0;
        sphere.color.r = 0.0f;
        sphere.color.g = 0.5f;
        sphere.color.b = 1.0f;
        sphere.color.a = 0.7f;
        markers.markers.push_back(sphere);
    }
    gps_marker_pub_->publish(markers);
}

void RvizPublisher::publishSubmapBoundaries(const std::vector<SubMap::Ptr>& submaps) {
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& sm : submaps) {
        visualization_msgs::msg::Marker text;
        text.header.frame_id = "world";
        text.header.stamp    = node_->now();
        text.ns    = "submap_labels";
        text.id    = id++;
        text.type  = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action= visualization_msgs::msg::Marker::ADD;
        const auto& p = sm->pose_w_anchor_optimized.translation();
        text.pose.position.x = p.x();
        text.pose.position.y = p.y();
        text.pose.position.z = p.z() + 2.0;
        text.scale.z  = 1.5;
        text.color.r  = 1.0f;
        text.color.g  = 1.0f;
        text.color.b  = 0.0f;
        text.color.a  = 1.0f;
        text.text = "SM" + std::to_string(sm->id);
        markers.markers.push_back(text);
    }
    submap_boundary_pub_->publish(markers);
}

void RvizPublisher::publishOptimizedPath(const std::vector<SubMap::Ptr>& submaps) {
    nav_msgs::msg::Path path;
    path.header.stamp    = node_->now();
    path.header.frame_id = "world";

    std::vector<std::pair<double, Pose3d>> sorted_kfs;
    for (const auto& sm : submaps) {
        for (const auto& kf : sm->keyframes) {
            sorted_kfs.push_back({kf->timestamp, kf->T_w_b_optimized});
        }
    }
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const std::pair<double, Pose3d>& a, const std::pair<double, Pose3d>& b) {
                  return a.first < b.first;
              });

    for (const auto& [ts, pose] : sorted_kfs) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.header.stamp.sec = static_cast<int32_t>(std::floor(ts));
        ps.header.stamp.nanosec = static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
        const auto& p = pose.translation();
        Eigen::Quaterniond q(pose.rotation());
        ps.pose.position.x    = p.x();
        ps.pose.position.y    = p.y();
        ps.pose.position.z    = p.z();
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        path.poses.push_back(ps);
    }
    opt_path_pub_->publish(path);
}

}  // namespace automap_pro
