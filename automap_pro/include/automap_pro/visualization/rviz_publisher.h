#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

class RvizPublisher {
public:
    RvizPublisher() = default;
    ~RvizPublisher() = default;

    void init(rclcpp::Node::SharedPtr node);

    void publishGlobalMap(const CloudXYZIPtr& cloud);
    void publishCurrentCloud(const CloudXYZIPtr& cloud);
    void publishSubmapCloud(const SubMap::Ptr& sm);

    void publishLoopMarkers(const std::vector<LoopConstraint::Ptr>& loops,
                             const std::vector<SubMap::Ptr>& submaps);

    void publishGPSMarkers(const std::vector<SubMap::Ptr>& submaps);
    void publishSubmapBoundaries(const std::vector<SubMap::Ptr>& submaps);

    void publishOdometryPath(const std::vector<std::pair<double, Pose3d>>& path);
    void publishOptimizedPath(const std::vector<SubMap::Ptr>& submaps);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gps_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submap_boundary_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;

    double global_map_ds_res_ = 0.5;
    bool   publish_global_map_ = true;

    CloudXYZIPtr toROSCloud(const CloudXYZIPtr& cloud, double ds_res = 0.0) const;
};

}  // namespace automap_pro
