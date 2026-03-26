#include "automap_pro/visualization/rviz_publisher.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cmath>
#include <algorithm>

namespace automap_pro {

namespace {
// RViz 球体为直径 scale=2*radius；GPS 约束用小球，回环连线略加粗便于辨认
constexpr float kGpsMarkerSphereRadius   = 0.06f;   // ~12cm 直径
constexpr float kGpsAlignedSphereRadius   = 0.08f;
constexpr float kLoopConstraintLineWidth  = 0.12f;
constexpr float kLoopEdgeLineWidthM       = 0.28f;
constexpr float kLoopEndpointSphereRadius = 0.10f;
}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// init
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();
    global_map_ds_res_  = static_cast<float>(cfg.mapVoxelSize());
    publish_global_map_ = true;

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto path_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

    // 点云（全局地图用 Reliable，便于 RViz 完整显示）
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    global_map_pub_     = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/global_map", map_qos);
    current_cloud_pub_  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/current_cloud", sensor_qos);
    submap_cloud_pub_   = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/submap_cloud", sensor_qos);
    colored_cloud_pub_  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/colored_cloud", sensor_qos);
    density_heatmap_pub_= node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/density_heatmap", sensor_qos);
    loop_candidate_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/loop_candidate_cloud", sensor_qos);

    // Marker
    loop_marker_pub_      = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/loop_markers", reliable_qos);
    gps_marker_pub_       = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/gps_markers", reliable_qos);
    submap_boundary_pub_  = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/submap_boundaries", reliable_qos);
    submap_bbox_pub_      = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/submap_bboxes", reliable_qos);
    submap_graph_pub_     = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/submap_graph", reliable_qos);
    covariance_pub_      = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/covariance_ellipses", reliable_qos);
    factor_graph_pub_     = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/factor_graph", reliable_qos);
    gps_quality_pub_     = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/gps_quality", reliable_qos);
    gps_positions_map_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/gps_positions_map", reliable_qos);
    module_status_pub_    = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/module_status", reliable_qos);
    frame_marker_pub_     = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/coordinate_frames", reliable_qos);
    active_region_pub_    = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/active_region", reliable_qos);
    degen_region_pub_     = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/degeneration_regions", reliable_qos);
    hba_result_pub_       = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/hba_result", reliable_qos);
    convergence_pub_      = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/convergence", reliable_qos);
    convergence_residual_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("/automap/convergence_residual", reliable_qos);
    semantic_landmark_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/semantic_landmarks", reliable_qos);

    // Path（optimized_path / gps_keyframe_path 使用 TransientLocal，供 HBA 后自动启动的 VTK 查看器收最后一次发布）
    auto path_qos_latched = rclcpp::QoS(rclcpp::KeepLast(100)).reliable()
        .durability(rclcpp::DurabilityPolicy::TransientLocal);
    odom_path_pub_        = node->create_publisher<nav_msgs::msg::Path>("/automap/odom_path", path_qos);
    opt_path_pub_         = node->create_publisher<nav_msgs::msg::Path>("/automap/optimized_path", path_qos_latched);
    gps_raw_path_pub_     = node->create_publisher<nav_msgs::msg::Path>("/automap/gps_raw_path", path_qos);
    gps_aligned_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/automap/gps_aligned_path", path_qos);
    gps_keyframe_path_pub_= node->create_publisher<nav_msgs::msg::Path>("/automap/gps_keyframe_path", path_qos_latched);
    hba_gps_deviation_pub_= node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/hba_gps_deviation", reliable_qos);
    hba_trajectory_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/hba_trajectory_points", map_qos);
    gps_trajectory_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/automap/gps_trajectory_points", map_qos);

    kf_pose_array_pub_    = node->create_publisher<geometry_msgs::msg::PoseArray>("/automap/keyframe_poses", reliable_qos);

    RCLCPP_INFO(node->get_logger(),
        "[RvizPublisher][TOPIC] publish: /automap/global_map, /automap/current_cloud, /automap/submap_cloud, /automap/colored_cloud, "
        "/automap/density_heatmap, /automap/loop_candidate_cloud, /automap/loop_markers, /automap/gps_markers, /automap/submap_boundaries, "
        "/automap/submap_bboxes, /automap/submap_graph, /automap/covariance_ellipses, /automap/factor_graph, /automap/gps_quality, /automap/gps_positions_map, "
        "/automap/module_status, /automap/coordinate_frames, /automap/active_region, /automap/degeneration_regions, /automap/hba_result, "
        "/automap/convergence, /automap/convergence_residual, /automap/odom_path, /automap/optimized_path, /automap/gps_raw_path, "
        "/automap/gps_aligned_path, /automap/gps_keyframe_path, /automap/hba_gps_deviation, /automap/hba_trajectory_points, /automap/gps_trajectory_points, /automap/keyframe_poses, /automap/semantic_landmarks");
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. 点云可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishGlobalMap(const CloudXYZIPtr& cloud) {
    if (!publish_global_map_ || !node() || !cloud || cloud->empty()) return;
    try {
        // 调用方（AutoMapSystem::publishGlobalMap）传入的已是 buildGlobalMap 下采样后的点云，
        // 不再在此处二次体素滤波，避免 PCL VoxelGrid 在部分 leaf/范围组合下写越界导致析构时 SIGSEGV。
        RCLCPP_DEBUG(node()->get_logger(), "[GLOBAL_MAP_DIAG] RvizPublisher publishGlobalMap frame_id=%s pts=%zu",
            frame_id_.c_str(), cloud->size());
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp    = node()->now();
        msg.header.frame_id = frame_id_;
        global_map_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishGlobalMap failed: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishGlobalMap failed: unknown exception (SIGSEGV possible)");
    }
}

void RvizPublisher::publishCurrentCloud(const CloudXYZIPtr& cloud) {
    if (!node() || !cloud || cloud->empty()) return;
    try {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp    = node()->now();
        msg.header.frame_id = frame_id_;
        current_cloud_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishCurrentCloud failed: %s", e.what());
    }
}

void RvizPublisher::publishCurrentCloud(const CloudXYZIConstPtr& cloud) {
    if (!node() || !cloud || cloud->empty()) return;
    try {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp    = node()->now();
        msg.header.frame_id = frame_id_;
        current_cloud_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishCurrentCloud failed: %s", e.what());
    }
}

void RvizPublisher::publishSubmapCloud(const SubMap::Ptr& sm) {
    if (!node() || !sm || !sm->downsampled_cloud || sm->downsampled_cloud->empty()) return;
    try {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*sm->downsampled_cloud, msg);
        msg.header.stamp    = node()->now();
        msg.header.frame_id = frame_id_;
        submap_cloud_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishSubmapCloud failed: %s", e.what());
    }
}

void RvizPublisher::publishColoredCloud(const CloudXYZIPtr& cloud,
                                         const std::string& color_mode,
                                         const std::string& /* topic_name */) {
    if (!node() || !colored_cloud_pub_ || !cloud || cloud->empty()) return;
    try {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        rgb_cloud->resize(cloud->size());
        float v_min = 1e9f, v_max = -1e9f;
        for (const auto& p : cloud->points) {
            float v = (color_mode == "height") ? static_cast<float>(p.z) : p.intensity;
            v_min = std::min(v_min, v); v_max = std::max(v_max, v);
        }
        if (v_max <= v_min) v_max = v_min + 1.0f;
        for (size_t i = 0; i < cloud->size(); ++i) {
            rgb_cloud->points[i].x = cloud->points[i].x;
            rgb_cloud->points[i].y = cloud->points[i].y;
            rgb_cloud->points[i].z = cloud->points[i].z;
            float t = (cloud->points[i].intensity - v_min) / (v_max - v_min);
            if (color_mode == "height")
                t = (cloud->points[i].z - v_min) / (v_max - v_min);
            std_msgs::msg::ColorRGBA c = makeColorMap(t);
            rgb_cloud->points[i].r = static_cast<uint8_t>(c.r * 255);
            rgb_cloud->points[i].g = static_cast<uint8_t>(c.g * 255);
            rgb_cloud->points[i].b = static_cast<uint8_t>(c.b * 255);
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*rgb_cloud, msg);
        msg.header.stamp = node()->now();
        msg.header.frame_id = frame_id_;
        colored_cloud_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishColoredCloud failed: %s", e.what());
    }
}

void RvizPublisher::publishDensityHeatmap(const CloudXYZIPtr& cloud, float grid_size) {
    if (!node() || !density_heatmap_pub_ || !cloud || cloud->empty() || grid_size <= 0.f) return;
    try {
        std::map<std::tuple<int,int,int>, int> grid_count;
        for (const auto& p : cloud->points) {
            int gx = static_cast<int>(std::floor(p.x / grid_size));
            int gy = static_cast<int>(std::floor(p.y / grid_size));
            int gz = static_cast<int>(std::floor(p.z / grid_size));
            grid_count[{gx, gy, gz}]++;
        }
        int c_max = 0;
        for (const auto& kv : grid_count) c_max = std::max(c_max, kv.second);
        if (c_max == 0) c_max = 1;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& kv : grid_count) {
            pcl::PointXYZRGB pt;
            pt.x = (std::get<0>(kv.first) + 0.5f) * grid_size;
            pt.y = (std::get<1>(kv.first) + 0.5f) * grid_size;
            pt.z = (std::get<2>(kv.first) + 0.5f) * grid_size;
            std_msgs::msg::ColorRGBA c = makeColorMap(static_cast<float>(kv.second) / c_max);
            pt.r = static_cast<uint8_t>(c.r * 255); pt.g = static_cast<uint8_t>(c.g * 255); pt.b = static_cast<uint8_t>(c.b * 255);
            out->push_back(pt);
        }
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*out, msg);
        msg.header.stamp = node()->now();
        msg.header.frame_id = frame_id_;
        density_heatmap_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishDensityHeatmap failed: %s", e.what());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. 轨迹可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishOdometryPath(const std::vector<std::pair<double, Pose3d>>& path) {
    if (!node() || !odom_path_pub_) return;
    nav_msgs::msg::Path msg;
    msg.header.stamp = node()->now();
    msg.header.frame_id = frame_id_;
    for (const auto& [ts, pose] : path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header;
        ps.header.stamp.sec = static_cast<int32_t>(std::floor(ts));
        ps.header.stamp.nanosec = static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
        const auto& p = pose.translation();
        Eigen::Quaterniond q(pose.rotation());
        ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        msg.poses.push_back(ps);
    }
    odom_path_pub_->publish(msg);
}

void RvizPublisher::publishOdometryPath(const nav_msgs::msg::Path& path) {
    if (!node() || !odom_path_pub_) return;
    odom_path_pub_->publish(path);
}

void RvizPublisher::publishOptimizedPath(const std::vector<SubMap::Ptr>& submaps,
                                         const PoseSnapshot::Ptr& snapshot) {
    if (!node() || !opt_path_pub_) return;
    nav_msgs::msg::Path path;
    path.header.stamp    = node()->now();
    path.header.frame_id = frame_id_;

    std::vector<std::pair<double, Pose3d>> sorted_kfs;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            
            Pose3d pose_opt = kf->T_map_b_optimized;
            if (snapshot) {
                auto it = snapshot->keyframe_poses.find(kf->id);
                if (it != snapshot->keyframe_poses.end()) {
                    pose_opt = it->second;
                }
            }
            
            // 与 buildGlobalMap 一致：使用 kf->T_map_b_optimized（及快照），不在此再乘 R_enu_to_map（否则 Path 在 ENU/map 而 global_map 仍在 SLAM 世界系，RViz 中轨迹与点云错位）
            Pose3d pose_map = pose_opt;
            sorted_kfs.push_back({kf->timestamp, pose_map});
        }
    }
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const std::pair<double, Pose3d>& a, const std::pair<double, Pose3d>& b) {
                  return a.first < b.first;
              });

    if (!sorted_kfs.empty()) {
        const auto& first_t = sorted_kfs.front().second.translation();
        const auto& last_t = sorted_kfs.back().second.translation();
        RCLCPP_INFO(node()->get_logger(),
            "[GHOSTING_DIAG] optimized_path published version=%lu kf_count=%zu first_pos=[%.2f,%.2f,%.2f] last_pos=[%.2f,%.2f,%.2f] frame=slam_world_same_as_global_map",
            snapshot ? snapshot->version : 0, sorted_kfs.size(), first_t.x(), first_t.y(), first_t.z(), last_t.x(), last_t.y(), last_t.z());
    }

    for (const auto& [ts, pose] : sorted_kfs) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.header.stamp.sec = static_cast<int32_t>(std::floor(ts));
        ps.header.stamp.nanosec = static_cast<uint32_t>(std::round((ts - std::floor(ts)) * 1e9));
        const auto& p = pose.translation();
        Eigen::Quaterniond q(pose.rotation());
        ps.pose.position.x    = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        path.poses.push_back(ps);
    }
    opt_path_pub_->publish(path);
}

void RvizPublisher::publishKeyframePoses(const std::vector<KeyFrame::Ptr>& keyframes,
                                         const PoseSnapshot::Ptr& snapshot) {
    if (!node() || !kf_pose_array_pub_) return;
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = node()->now();
    msg.header.frame_id = frame_id_;

    Eigen::Vector3d first_t = Eigen::Vector3d::Zero(), last_t = Eigen::Vector3d::Zero();
    size_t valid_count = 0;
    for (const auto& kf : keyframes) {
        if (!kf) continue;
        
        Pose3d pose_opt = kf->T_map_b_optimized;
        if (snapshot) {
            auto it = snapshot->keyframe_poses.find(kf->id);
            if (it != snapshot->keyframe_poses.end()) {
                pose_opt = it->second;
            }
        }

        Pose3d pose_map = pose_opt;
        const auto& p = pose_map.translation();
        if (valid_count == 0) first_t = p;
        last_t = p;
        valid_count++;
        geometry_msgs::msg::Pose pose;
        Eigen::Quaterniond q(pose_map.rotation());
        pose.position.x = p.x(); pose.position.y = p.y(); pose.position.z = p.z();
        pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
        msg.poses.push_back(pose);
    }
    if (valid_count > 0) {
        RCLCPP_DEBUG(node()->get_logger(),
            "[GHOSTING_DIAG] keyframe_poses published version=%lu kf_count=%zu first_pos=[%.2f,%.2f,%.2f] last_pos=[%.2f,%.2f,%.2f] frame=slam_world_same_as_global_map",
            snapshot ? snapshot->version : 0, valid_count, first_t.x(), first_t.y(), first_t.z(), last_t.x(), last_t.y(), last_t.z());
    }
    kf_pose_array_pub_->publish(msg);
}

void RvizPublisher::publishGPSTrajectory(const std::vector<SubMap::Ptr>& submaps, bool show_aligned) {
    // 无 map 系位置时（未对齐）raw 使用 gps_center（ENU），标为 "enu" 避免与 map 混系
    publishGPSTrajectory(submaps, std::vector<Eigen::Vector3d>{}, show_aligned, "enu");
}

void RvizPublisher::publishGPSTrajectory(const std::vector<SubMap::Ptr>& submaps,
                                          const std::vector<Eigen::Vector3d>& gps_positions_map,
                                          bool show_aligned,
                                          const std::string& raw_path_frame_id) {
    if (!node() || !gps_raw_path_pub_) return;
    nav_msgs::msg::Path raw_path, aligned_path;
    raw_path.header.stamp = aligned_path.header.stamp = node()->now();
    raw_path.header.frame_id = raw_path_frame_id;  // raw：map 系或 enu 系由调用方指定
    aligned_path.header.frame_id = frame_id_;      // 对齐轨迹始终为 map
    size_t map_idx = 0;
    const bool use_map_frame = !gps_positions_map.empty();
    for (const auto& sm : submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        geometry_msgs::msg::PoseStamped ps;
        ps.header = raw_path.header;
        ps.pose.orientation.w = 1.0;
        if (use_map_frame && map_idx < gps_positions_map.size()) {
            const auto& p = gps_positions_map[map_idx++];
            ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        } else {
            ps.pose.position.x = sm->gps_center.x();
            ps.pose.position.y = sm->gps_center.y();
            ps.pose.position.z = sm->gps_center.z();
        }
        raw_path.poses.push_back(ps);
        ps.header = aligned_path.header;
        ps.pose.position.x = sm->pose_map_anchor_optimized.translation().x();
        ps.pose.position.y = sm->pose_map_anchor_optimized.translation().y();
        ps.pose.position.z = sm->pose_map_anchor_optimized.translation().z();
        aligned_path.poses.push_back(ps);
    }
    gps_raw_path_pub_->publish(raw_path);
    if (show_aligned && gps_aligned_path_pub_) gps_aligned_path_pub_->publish(aligned_path);
}

void RvizPublisher::publishTrajectoryComparison(
    const std::vector<std::pair<double, Pose3d>>& odom_path,
    const std::vector<std::pair<double, Pose3d>>& opt_path) {
    publishOdometryPath(odom_path);
    nav_msgs::msg::Path path;
    path.header.stamp = node()->now();
    path.header.frame_id = frame_id_;
    for (const auto& [ts, pose] : opt_path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        const auto& p = pose.translation();
        Eigen::Quaterniond q(pose.rotation());
        ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        path.poses.push_back(ps);
    }
    opt_path_pub_->publish(path);
}

void RvizPublisher::publishGpsKeyframePath(const std::vector<Eigen::Vector3d>& gps_positions_map) {
    if (!node() || !gps_keyframe_path_pub_ || gps_positions_map.empty()) return;
    nav_msgs::msg::Path path;
    path.header.stamp = node()->now();
    path.header.frame_id = frame_id_;
    for (const auto& p : gps_positions_map) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
    }
    gps_keyframe_path_pub_->publish(path);
}

void RvizPublisher::publishHbaGpsDeviationMarkers(const std::vector<Eigen::Vector3d>& hba_positions,
                                                  const std::vector<Eigen::Vector3d>& gps_positions_map) {
    if (!node() || !hba_gps_deviation_pub_ || hba_positions.size() != gps_positions_map.size() || hba_positions.empty()) return;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = node()->now();
    line.ns = "hba_gps_deviation";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.08;
    line.color = makeColor(1.0f, 0.2f, 0.2f, 0.9f);
    for (size_t i = 0; i < hba_positions.size(); ++i) {
        geometry_msgs::msg::Point pa, pb;
        pa.x = hba_positions[i].x(); pa.y = hba_positions[i].y(); pa.z = hba_positions[i].z();
        pb.x = gps_positions_map[i].x(); pb.y = gps_positions_map[i].y(); pb.z = gps_positions_map[i].z();
        line.points.push_back(pa);
        line.points.push_back(pb);
    }
    markers.markers.push_back(line);
    hba_gps_deviation_pub_->publish(markers);
}

void RvizPublisher::publishHbaGpsTrajectoryClouds(const std::vector<Eigen::Vector3d>& hba_positions,
                                                  const std::vector<Eigen::Vector3d>& gps_positions_map) {
    if (!node() || !hba_trajectory_cloud_pub_ || !gps_trajectory_cloud_pub_) return;
    if (hba_positions.empty() && gps_positions_map.empty()) return;

    const auto stamp = node()->now();
    const std::string frame_id = frame_id_;

    // HBA 轨迹点云（绿色）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hba_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    hba_cloud->header.frame_id = frame_id;
    hba_cloud->reserve(hba_positions.size());
    for (const auto& p : hba_positions) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        pt.r = 0;
        pt.g = 255;
        pt.b = 0;
        hba_cloud->push_back(pt);
    }
    if (!hba_cloud->empty()) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*hba_cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        hba_trajectory_cloud_pub_->publish(msg);
    }

    // GPS 轨迹点云（蓝色）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr gps_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    gps_cloud->header.frame_id = frame_id;
    gps_cloud->reserve(gps_positions_map.size());
    for (const auto& p : gps_positions_map) {
        pcl::PointXYZRGB pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;
        gps_cloud->push_back(pt);
    }
    if (!gps_cloud->empty()) {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*gps_cloud, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        gps_trajectory_cloud_pub_->publish(msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. 回环可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishLoopMarkers(
        const std::vector<LoopConstraint::Ptr>& loops,
        const std::vector<SubMap::Ptr>& submaps) {

    if (!node() || !loop_marker_pub_) return;

    std::map<int, Eigen::Vector3d> sm_pos;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        sm_pos[sm->id] = sm->pose_map_anchor_optimized.translation();
    }

    visualization_msgs::msg::MarkerArray markers;
    // 先清除该命名空间下旧的回环线，避免数量减少时残留
    auto delete_all = makeDeleteAllMarkers("loops");
    markers.markers.push_back(delete_all.markers.front());
    markers.markers.push_back(makeDeleteAllMarkers("loop_ends").markers.front());

    int id = 0;
    int end_id = 0;
    for (const auto& lc : loops) {
        auto it_i = sm_pos.find(lc->submap_i);
        auto it_j = sm_pos.find(lc->submap_j);
        if (it_i == sm_pos.end() || it_j == sm_pos.end()) continue;

        visualization_msgs::msg::Marker line;
        line.header.frame_id = frame_id_;
        line.header.stamp    = node()->now();
        line.ns    = "loops";
        line.id    = id++;
        line.type  = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action= visualization_msgs::msg::Marker::ADD;
        line.scale.x = kLoopEdgeLineWidthM;
        line.color.r = lc->is_inter_session ? 1.0f : 0.0f;
        line.color.g = lc->is_inter_session ? 0.5f : 1.0f;
        line.color.b = 0.0f;
        line.color.a = 1.0f;

        geometry_msgs::msg::Point p1, p2;
        p1.x = it_i->second.x(); p1.y = it_i->second.y(); p1.z = it_i->second.z();
        p2.x = it_j->second.x(); p2.y = it_j->second.y(); p2.z = it_j->second.z();
        line.points.push_back(p1);
        line.points.push_back(p2);
        markers.markers.push_back(line);

        std_msgs::msg::ColorRGBA end_c = line.color;
        end_c.a = 1.0f;
        markers.markers.push_back(
            makeSphereMarker("loop_ends", end_id++, it_i->second, kLoopEndpointSphereRadius, end_c));
        markers.markers.push_back(
            makeSphereMarker("loop_ends", end_id++, it_j->second, kLoopEndpointSphereRadius, end_c));
    }
    loop_marker_pub_->publish(markers);
}

void RvizPublisher::publishLoopCandidateClouds(const SubMap::Ptr& query,
                                                const SubMap::Ptr& candidate,
                                                int /* loop_id */) {
    if (!node() || !loop_candidate_pub_ || !query || !candidate) return;
    if ((!query->downsampled_cloud || query->downsampled_cloud->empty()) &&
        (!candidate->downsampled_cloud || candidate->downsampled_cloud->empty())) return;
    try {
        CloudXYZIPtr merged(new CloudXYZI);
        if (query->downsampled_cloud && !query->downsampled_cloud->empty())
            *merged += *query->downsampled_cloud;
        if (candidate->downsampled_cloud && !candidate->downsampled_cloud->empty()) {
            CloudXYZI cand_world;
            pcl::transformPointCloud(*candidate->downsampled_cloud, cand_world, candidate->pose_map_anchor_optimized.matrix());
            *merged += cand_world;
        }
        if (merged->empty()) return;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*merged, msg);
        msg.header.stamp = node()->now();
        msg.header.frame_id = frame_id_;
        loop_candidate_pub_->publish(msg);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node()->get_logger(), "[RvizPublisher] publishLoopCandidateClouds failed: %s", e.what());
    }
}

void RvizPublisher::publishLoopDetectionStatus(const SubMap::Ptr& query,
                                                const std::vector<std::pair<int, float>>& scores) {
    if (!node() || !loop_marker_pub_ || scores.empty()) return;
    visualization_msgs::msg::MarkerArray markers;
    // 以查询子图中心为基准，沿 X 轴偏移显示各候选得分（便于在场景中定位）
    Eigen::Vector3d base = query ? query->pose_map_anchor_optimized.translation() : Eigen::Vector3d(0, 0, 0);
    int id = 0;
    float max_s = 0.f;
    for (const auto& p : scores) max_s = std::max(max_s, p.second);
    if (max_s <= 0.f) max_s = 1.f;
    for (size_t i = 0; i < scores.size(); ++i) {
        Eigen::Vector3d pos = base + Eigen::Vector3d(static_cast<double>(i) * 3.0, 0.0, scores[i].second * 2.0);
        auto m = makeSphereMarker("loop_scores", id++, pos, 0.4f, makeColorMap(scores[i].second / max_s));
        markers.markers.push_back(m);
    }
    loop_marker_pub_->publish(markers);
}

void RvizPublisher::clearLoopCandidateClouds() {
    if (!loop_candidate_pub_) return;
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = node()->now();
    msg.header.frame_id = frame_id_;
    msg.width = 0;
    msg.height = 1;
    msg.is_dense = true;
    loop_candidate_pub_->publish(msg);
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. 子图可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishSubmapBoundaries(const std::vector<SubMap::Ptr>& submaps) {
    if (!node()) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        std_msgs::msg::ColorRGBA yellow;
        yellow.r = 1.0f; yellow.g = 1.0f; yellow.b = 0.0f; yellow.a = 1.0f;
        auto text = makeTextMarker("submap_labels", id++, sm->pose_map_anchor_optimized.translation(),
            "SM" + std::to_string(sm->id), 1.5, &yellow);
        markers.markers.push_back(text);
    }
    submap_boundary_pub_->publish(markers);
}

void RvizPublisher::publishSubmapBoundingBoxes(const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !submap_bbox_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& sm : submaps) {
        if (!sm || !sm->merged_cloud || sm->merged_cloud->empty()) continue;
        Eigen::Vector3d min_pt(1e6, 1e6, 1e6), max_pt(-1e6, -1e6, -1e6);
        for (const auto& p : sm->merged_cloud->points) {
            min_pt.x() = std::min(min_pt.x(), static_cast<double>(p.x));
            min_pt.y() = std::min(min_pt.y(), static_cast<double>(p.y));
            min_pt.z() = std::min(min_pt.z(), static_cast<double>(p.z));
            max_pt.x() = std::max(max_pt.x(), static_cast<double>(p.x));
            max_pt.y() = std::max(max_pt.y(), static_cast<double>(p.y));
            max_pt.z() = std::max(max_pt.z(), static_cast<double>(p.z));
        }
        Eigen::Vector3d center = (min_pt + max_pt) / 2.0;
        Eigen::Vector3d scale = max_pt - min_pt;
        Pose3d pose = Pose3d::Identity();
        pose.translation() = center;
        std_msgs::msg::ColorRGBA c;
        c.r = 0.2f; c.g = 0.8f; c.b = 0.2f; c.a = 0.3f;
        auto box = makeBoundingBoxMarker("submap_bbox", id++, pose, scale, c);
        markers.markers.push_back(box);
    }
    submap_bbox_pub_->publish(markers);
}

void RvizPublisher::publishSubmapGraph(const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !submap_graph_pub_) return;
    // 按子图 ID 排序后连接相邻子图（过滤 null 避免 sort 比较器解引用空指针）
    std::vector<SubMap::Ptr> sorted;
    for (const auto& s : submaps) if (s) sorted.push_back(s);
    std::sort(sorted.begin(), sorted.end(),
        [](const SubMap::Ptr& a, const SubMap::Ptr& b) { return a->id < b->id; });

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    std_msgs::msg::ColorRGBA gray = makeColor(0.5f, 0.5f, 0.5f, 0.6f);
    for (size_t i = 0; i + 1 < sorted.size(); ++i) {
        Eigen::Vector3d a = sorted[i]->pose_map_anchor_optimized.translation();
        Eigen::Vector3d b = sorted[i+1]->pose_map_anchor_optimized.translation();
        markers.markers.push_back(makeLineMarker("submap_graph", id++, a, b, gray, 0.03));
    }
    submap_graph_pub_->publish(markers);
}

void RvizPublisher::publishSubmapDetail(const SubMap::Ptr& submap) {
    if (!submap) return;
    publishSubmapCloud(submap);
    publishSubmapBoundingBoxes({submap});
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. GPS 可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishGPSMarkers(const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !gps_marker_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    // 先清除旧 GPS 约束标记，避免数量变化时残留
    auto delete_all = makeDeleteAllMarkers("gps");
    markers.markers.push_back(delete_all.markers.front());

    int id = 0;
    for (const auto& sm : submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        // 使用优化后位姿，与轨迹一致（子图中心即 GPS 约束节点）
        Eigen::Vector3d pos = sm->pose_map_anchor_optimized.translation();
        auto sphere = makeSphereMarker("gps", id++, pos, kGpsMarkerSphereRadius, makeColor(0.0f, 0.6f, 1.0f, 0.9f));
        markers.markers.push_back(sphere);
    }
    gps_marker_pub_->publish(markers);
}

void RvizPublisher::publishGPSMarkersWithConstraintLines(
    const std::vector<SubMap::Ptr>& submaps,
    const std::vector<Eigen::Vector3d>& gps_positions_map) {
    if (!node() || !gps_marker_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarkers("gps").markers.front());
    markers.markers.push_back(makeDeleteAllMarkers("gps_constraint_lines").markers.front());

    size_t gps_idx = 0;
    int sphere_id = 0;
    std_msgs::msg::ColorRGBA line_c = makeColor(0.2f, 0.8f, 0.4f, 0.9f);
    for (const auto& sm : submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        if (gps_idx >= gps_positions_map.size()) break;
        Eigen::Vector3d pos = sm->pose_map_anchor_optimized.translation();
        markers.markers.push_back(
            makeSphereMarker("gps", sphere_id++, pos, kGpsMarkerSphereRadius, makeColor(0.0f, 0.6f, 1.0f, 0.9f)));
        markers.markers.push_back(
            makeLineMarker("gps_constraint_lines", static_cast<int>(gps_idx),
                           pos, gps_positions_map[gps_idx], line_c, kLoopConstraintLineWidth));
        gps_idx++;
    }
    gps_marker_pub_->publish(markers);
}

void RvizPublisher::publishGPSAlignment(const GPSAlignResult& result,
                                        const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !gps_marker_pub_ || !result.success) return;
    visualization_msgs::msg::MarkerArray markers;
    // 先清除旧命名空间，避免重复触发对齐时 marker 堆积导致显示异常
    markers.markers.push_back(makeDeleteAllMarkers("gps_align").markers.front());
    markers.markers.push_back(makeDeleteAllMarkers("gps_aligned").markers.front());
    Pose3d T_align = Pose3d(result.R_gps_lidar) * Eigen::Translation3d(result.t_gps_lidar);
    auto axes = makeAxisMarkers("gps_align", 0, T_align, 3.0);
    for (auto& m : axes.markers) markers.markers.push_back(std::move(m));
    std_msgs::msg::ColorRGBA c = makeColor(0.0f, 1.0f, 0.0f, 0.8f);
    for (const auto& sm : submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        markers.markers.push_back(makeSphereMarker("gps_aligned", static_cast<int>(markers.markers.size()),
            sm->pose_map_anchor_optimized.translation(), kGpsAlignedSphereRadius, c));
    }
    gps_marker_pub_->publish(markers);
}

void RvizPublisher::publishGPSQualityMarkers(const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !gps_quality_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& sm : submaps) {
        if (!sm || !sm->has_valid_gps) continue;
        for (const auto& kf : sm->keyframes) {
            if (!kf || !kf->has_valid_gps) continue;
            float q = 1.0f;
            if (kf->gps.hdop > 5.0) q = 0.2f;
            else if (kf->gps.hdop > 2.0) q = 0.5f;
            else if (kf->gps.hdop > 1.0) q = 0.8f;
            std_msgs::msg::ColorRGBA c = makeColorMap(q);
            c.a = 0.8f;
            Eigen::Vector3d p = kf->T_map_b_optimized.translation();
            markers.markers.push_back(makeSphereMarker("gps_quality", id++, p, 0.5, c));
        }
    }
    gps_quality_pub_->publish(markers);
}

void RvizPublisher::publishGPSPositionsInMap(const std::vector<Eigen::Vector3d>& positions_map) {
    if (!node() || !gps_positions_map_pub_) return;
    if (positions_map.empty()) {
        gps_positions_map_pub_->publish(makeDeleteAllMarkers("gps_positions_map"));
        return;
    }
    visualization_msgs::msg::MarkerArray markers;
    std_msgs::msg::ColorRGBA c = makeColor(0.0f, 0.8f, 0.4f, 0.85f);  // 青绿色，表示真实 GPS 位置（地图系）
    for (size_t i = 0; i < positions_map.size(); ++i) {
        markers.markers.push_back(
            makeSphereMarker("gps_positions_map", static_cast<int>(i), positions_map[i], kGpsMarkerSphereRadius, c));
    }
    gps_positions_map_pub_->publish(markers);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. 后端优化可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishCovarianceEllipses(const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !covariance_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto& sm : submaps) {
        if (!sm || sm->keyframes.empty()) continue;
        auto* kf = sm->keyframes.front().get();
        if (!kf) continue;
        Eigen::Matrix3d cov_xy = kf->covariance.block<3,3>(0,0);
        auto ell = makeCovarianceEllipse("cov", id++, kf->T_map_b_optimized, cov_xy, 2.0);
        markers.markers.push_back(ell);
    }
    covariance_pub_->publish(markers);
}

void RvizPublisher::publishFactorGraph(const PoseGraph& graph) {
    if (!node() || !factor_graph_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    auto nodes = graph.allNodes();
    auto edges = graph.allEdges();
    std::map<int, Eigen::Vector3d> pos;
    int id = 0;
    for (const auto& n : nodes) {
        pos[n.id] = n.pose_opt.translation();
        markers.markers.push_back(makeSphereMarker("factor_nodes", id++, n.pose_opt.translation(), 0.4, makeColor(0.2f, 0.6f, 1.0f, 0.9f)));
    }
    std_msgs::msg::ColorRGBA odom_c = makeColor(0.5f, 0.5f, 0.5f, 0.6f);
    std_msgs::msg::ColorRGBA loop_c = makeColor(1.0f, 0.3f, 0.0f, 0.8f);
    std_msgs::msg::ColorRGBA gps_c  = makeColor(0.0f, 0.8f, 0.0f, 0.8f);
    for (const auto& e : edges) {
        auto it_from = pos.find(e.from);
        if (it_from == pos.end()) continue;
        // GPS 一元因子：to < 0，在节点上方画绿色小球表示约束
        if (e.type == EdgeType::GPS && e.to < 0) {
            Eigen::Vector3d at = it_from->second + Eigen::Vector3d(0, 0, 1.5);
            markers.markers.push_back(makeSphereMarker("factor_gps", id++, at, 0.25, gps_c));
            markers.markers.push_back(makeLineMarker("factor_gps", id++, it_from->second, at, gps_c, 0.03));
            continue;
        }
        auto it_to = pos.find(e.to);
        if (it_to == pos.end()) continue;
        std_msgs::msg::ColorRGBA* c = &odom_c;
        if (e.type == EdgeType::LOOP) c = &loop_c;
        else if (e.type == EdgeType::GPS) c = &gps_c;
        markers.markers.push_back(makeLineMarker("factor_edges", id++, it_from->second, it_to->second, *c, 0.02));
    }
    factor_graph_pub_->publish(markers);
}

void RvizPublisher::publishHBAResult(const HBAResult& result) {
    if (!node() || !hba_result_pub_ || !result.success) return;
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < result.optimized_poses.size(); ++i) {
        auto m = makeSphereMarker("hba_poses", static_cast<int>(i),
            result.optimized_poses[i].translation(), 0.3, makeColor(0.0f, 0.9f, 0.4f, 0.8f));
        markers.markers.push_back(m);
    }
    hba_result_pub_->publish(markers);
}

void RvizPublisher::publishOptimizationConvergence(const std::vector<double>& residuals) {
    if (!node() || !convergence_pub_ || residuals.empty()) return;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker line;
    line.header.frame_id = frame_id_;
    line.header.stamp = node()->now();
    line.ns = "convergence";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.15;
    line.color = makeColor(1.0f, 0.5f, 0.0f, 0.9f);
    double max_r = *std::max_element(residuals.begin(), residuals.end());
    if (max_r < 1e-9) max_r = 1.0;
    // 缩放至场景可见：X 方向 1m/迭代，Y 方向 10*m 放大残差（便于在 RViz 原点附近查看）
    const double x_scale = 1.0;
    const double y_scale = 10.0;
    for (size_t i = 0; i < residuals.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = static_cast<double>(i) * x_scale;
        p.y = residuals[i] * y_scale;
        p.z = 0.0;
        line.points.push_back(p);
    }
    markers.markers.push_back(line);
    convergence_pub_->publish(markers);

    // 发布残差序列供 PlotJuggler 等绘制（话题 /automap/convergence_residual）
    if (convergence_residual_pub_) {
        std_msgs::msg::Float64MultiArray arr;
        arr.data.resize(residuals.size());
        for (size_t i = 0; i < residuals.size(); ++i) arr.data[i] = residuals[i];
        convergence_residual_pub_->publish(arr);
    }
}

void RvizPublisher::publishPoseUpdateAnimation(const std::vector<Pose3d>& before,
                                               const std::vector<Pose3d>& after) {
    if (!node() || !opt_path_pub_ || before.size() != after.size()) return;
    nav_msgs::msg::Path path;
    path.header.stamp = node()->now();
    path.header.frame_id = frame_id_;
    for (size_t i = 0; i < after.size(); ++i) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        const auto& p = after[i].translation();
        Eigen::Quaterniond q(after[i].rotation());
        ps.pose.position.x = p.x(); ps.pose.position.y = p.y(); ps.pose.position.z = p.z();
        ps.pose.orientation.x = q.x(); ps.pose.orientation.y = q.y(); ps.pose.orientation.z = q.z(); ps.pose.orientation.w = q.w();
        path.poses.push_back(ps);
    }
    opt_path_pub_->publish(path);
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. 系统状态可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishCoordinateFrames(const Pose3d& base_pose, const Pose3d& sensor_pose) {
    if (!node() || !frame_marker_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    auto addFrame = [&](const std::string& ns, const Pose3d& pose, double scale) {
        Eigen::Vector3d o = pose.translation();
        Eigen::Vector3d x = pose.rotation() * Eigen::Vector3d::UnitX() * scale + o;
        Eigen::Vector3d y = pose.rotation() * Eigen::Vector3d::UnitY() * scale + o;
        Eigen::Vector3d z = pose.rotation() * Eigen::Vector3d::UnitZ() * scale + o;
        markers.markers.push_back(makeArrowMarker(ns, id++, o, x, makeColor(1,0,0,1), 0.03));
        markers.markers.push_back(makeArrowMarker(ns, id++, o, y, makeColor(0,1,0,1), 0.03));
        markers.markers.push_back(makeArrowMarker(ns, id++, o, z, makeColor(0,0,1,1), 0.03));
    };
    addFrame("world", Pose3d::Identity(), 2.0);
    addFrame("base", base_pose, 1.0);
    addFrame("sensor", sensor_pose, 0.5);
    frame_marker_pub_->publish(markers);
}

void RvizPublisher::publishModuleStatus(const std::map<std::string, int>& module_status) {
    if (!node() || !module_status_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    double y = 0.0;
    for (const auto& [name, status] : module_status) {
        std::string display_name = moduleDisplayName(name);
        std::string label = display_name + (status ? ": OK" : ": FAIL");
        std_msgs::msg::ColorRGBA c = (status == 1) ? makeColor(0.0f, 1.0f, 0.0f, 1.0f) : makeColor(1.0f, 0.0f, 0.0f, 1.0f);
        auto text = makeTextMarker("module_status", id++, Eigen::Vector3d(0.0, y, 0.0), label, 0.6, &c);
        markers.markers.push_back(text);
        y += 1.0;
    }
    module_status_pub_->publish(markers);
}

void RvizPublisher::publishActiveRegion(const SubMap::Ptr& active_submap) {
    if (!node() || !active_region_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    if (active_submap && active_submap->merged_cloud && !active_submap->merged_cloud->empty()) {
        Eigen::Vector3d c = active_submap->pose_map_anchor_optimized.translation();
        auto sphere = makeSphereMarker("active", 0, c, active_submap->spatial_extent_m > 0 ? active_submap->spatial_extent_m : 10.0,
            makeColor(0.2f, 0.8f, 0.2f, 0.25f));
        markers.markers.push_back(sphere);
    }
    active_region_pub_->publish(markers);
}

void RvizPublisher::publishDegenerationRegions(const std::vector<std::pair<double, Pose3d>>& degraded_poses) {
    if (!node() || !degen_region_pub_) return;
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < degraded_poses.size(); ++i) {
        auto m = makeSphereMarker("degen", static_cast<int>(i), degraded_poses[i].second.translation(), 0.5,
            makeColor(1.0f, 0.0f, 0.0f, 0.6f));
        markers.markers.push_back(m);
    }
    degen_region_pub_->publish(markers);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. 语义信息可视化
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::publishSemanticLandmarks(const std::vector<SubMap::Ptr>& submaps) {
    if (!node() || !semantic_landmark_pub_) return;

    visualization_msgs::msg::MarkerArray markers;
    // 首先删除所有旧的地标 Marker，防止残留
    markers.markers.push_back(makeDeleteAllMarkers("semantic_landmarks").markers.front());
    markers.markers.push_back(makeDeleteAllMarkers("semantic_labels").markers.front());

    int marker_id = 0;
    int label_id = 0;
    for (const auto& sm : submaps) {
        if (!sm) continue;
        
        // 每个子图都有自己的 landmarks (相对于子图锚点系)
        for (const auto& lm : sm->landmarks) {
            if (!lm || !lm->isValid()) continue;

            // 将地标 root 变换到 map 系
            Eigen::Vector3d root_map = sm->pose_map_anchor_optimized * lm->root;
            Eigen::Vector3d ray_map = sm->pose_map_anchor_optimized.rotation() * lm->ray;

            // 创建圆柱体 Marker (CYLINDER 类型)
            visualization_msgs::msg::Marker cylinder;
            cylinder.header.frame_id = frame_id_;
            cylinder.header.stamp = node()->now();
            cylinder.ns = "semantic_landmarks";
            cylinder.id = marker_id++;
            cylinder.type = visualization_msgs::msg::Marker::CYLINDER;
            cylinder.action = visualization_msgs::msg::Marker::ADD;

            // 设置圆柱体位姿（中心点位置）
            // 注意：Marker::CYLINDER 的中心点在圆柱体几何中心。lm->root 通常是底部中心。
            // 假设圆柱体高度为 5.0m (可视需求调整)
            const double cylinder_height = 5.0;
            Eigen::Vector3d center_map = root_map + ray_map * (cylinder_height * 0.5);
            cylinder.pose.position.x = center_map.x();
            cylinder.pose.position.y = center_map.y();
            cylinder.pose.position.z = center_map.z();

            // 设置方向（朝向 ray_map）
            // 默认 CYLINDER 沿 Z 轴。我们需要旋转到 ray_map 方向。
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), ray_map);
            cylinder.pose.orientation.x = q.x();
            cylinder.pose.orientation.y = q.y();
            cylinder.pose.orientation.z = q.z();
            cylinder.pose.orientation.w = q.w();

            // 设置尺寸
            cylinder.scale.x = lm->radius * 2.0; // 直径
            cylinder.scale.y = lm->radius * 2.0; // 直径
            cylinder.scale.z = cylinder_height;   // 高度
            
            // 设置颜色（棕色/木头色）
            cylinder.color = makeColor(0.54f, 0.27f, 0.07f, 0.8f);
            markers.markers.push_back(cylinder);

            // 添加置信度标签 (可选)
            std::string label_text = "Tree_" + std::to_string(lm->id) + " (conf:" + std::to_string(static_cast<int>(lm->confidence * 100)) + "%)";
            markers.markers.push_back(makeTextMarker("semantic_labels", label_id++, root_map + Eigen::Vector3d(0, 0, 2.0), label_text, 0.8));
        }
    }
    semantic_landmark_pub_->publish(markers);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. 工具函数
// ─────────────────────────────────────────────────────────────────────────────

void RvizPublisher::clearAllMarkers() {
    auto arr = makeDeleteAllMarkers("");
    for (auto* pub : {loop_marker_pub_.get(), gps_marker_pub_.get(), gps_positions_map_pub_.get(),
            submap_boundary_pub_.get(), submap_bbox_pub_.get(), submap_graph_pub_.get(), covariance_pub_.get(),
            factor_graph_pub_.get(), gps_quality_pub_.get(), module_status_pub_.get(), frame_marker_pub_.get(),
            active_region_pub_.get(), degen_region_pub_.get(), hba_result_pub_.get(), convergence_pub_.get()}) {
        if (pub) pub->publish(arr);
    }
}

void RvizPublisher::clearMarkers(const std::string& ns) {
    auto arr = makeDeleteAllMarkers(ns);
    for (auto* pub : {loop_marker_pub_.get(), gps_marker_pub_.get(), gps_positions_map_pub_.get(),
            submap_boundary_pub_.get(), submap_bbox_pub_.get(), submap_graph_pub_.get(), covariance_pub_.get(),
            factor_graph_pub_.get(), gps_quality_pub_.get(), module_status_pub_.get(), frame_marker_pub_.get(),
            active_region_pub_.get(), degen_region_pub_.get(), hba_result_pub_.get(), convergence_pub_.get()}) {
        if (pub) pub->publish(arr);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 私有辅助
// ─────────────────────────────────────────────────────────────────────────────

CloudXYZIPtr RvizPublisher::toROSCloud(const CloudXYZIPtr& cloud, double ds_res) const {
    if (!cloud || cloud->empty()) return nullptr;
    if (ds_res <= 0) return cloud;
    return utils::voxelDownsample(cloud, ds_res);
}

std_msgs::msg::ColorRGBA RvizPublisher::makeColor(float r, float g, float b, float a) const {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

std_msgs::msg::ColorRGBA RvizPublisher::makeColorMap(float value, const std::string& cmap) const {
    value = std::max(0.0f, std::min(1.0f, value));
    float r = 0, g = 0, b = 0;
    if (cmap == "jet") {
        if (value < 0.25f) { r = 0; g = value * 4.0f; b = 1.0f; }
        else if (value < 0.5f) { r = 0; g = 1.0f; b = 2.0f - value * 4.0f; }
        else if (value < 0.75f) { r = value * 4.0f - 2.0f; g = 1.0f; b = 0; }
        else { r = 1.0f; g = 4.0f - value * 4.0f; b = 0; }
    } else {
        r = value; g = 1.0f - value; b = 0.5f;
    }
    return makeColor(r, g, b, 1.0f);
}

visualization_msgs::msg::MarkerArray RvizPublisher::makeAxisMarkers(
    const std::string& ns, int base_id, const Pose3d& pose, double scale) const {
    visualization_msgs::msg::MarkerArray arr;
    Eigen::Vector3d o = pose.translation();
    auto addArrow = [&](int offset, const Eigen::Vector3d& dir, const std_msgs::msg::ColorRGBA& c) {
        Eigen::Vector3d end = pose.rotation() * dir * scale + o;
        arr.markers.push_back(makeArrowMarker(ns, base_id + offset, o, end, c, scale * 0.05));
    };
    addArrow(0, Eigen::Vector3d::UnitX(), makeColor(1, 0, 0, 1)); // X 红
    addArrow(1, Eigen::Vector3d::UnitY(), makeColor(0, 1, 0, 1)); // Y 绿
    addArrow(2, Eigen::Vector3d::UnitZ(), makeColor(0, 0, 1, 1)); // Z 蓝
    return arr;
}

visualization_msgs::msg::Marker RvizPublisher::makeArrowMarker(
    const std::string& ns, int id, const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const std_msgs::msg::ColorRGBA& color, double shaft_diameter) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = shaft_diameter;
    m.scale.y = shaft_diameter * 2.0;
    m.scale.z = 0.0;
    m.points.resize(2);
    m.points[0].x = start.x(); m.points[0].y = start.y(); m.points[0].z = start.z();
    m.points[1].x = end.x(); m.points[1].y = end.y(); m.points[1].z = end.z();
    m.color = color;
    return m;
}

visualization_msgs::msg::Marker RvizPublisher::makeLineMarker(
    const std::string& ns, int id, const Eigen::Vector3d& start, const Eigen::Vector3d& end,
    const std_msgs::msg::ColorRGBA& color, double width) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = width;
    m.color = color;
    geometry_msgs::msg::Point p1, p2;
    p1.x = start.x(); p1.y = start.y(); p1.z = start.z();
    p2.x = end.x(); p2.y = end.y(); p2.z = end.z();
    m.points.push_back(p1);
    m.points.push_back(p2);
    return m;
}

visualization_msgs::msg::Marker RvizPublisher::makeSphereMarker(
    const std::string& ns, int id, const Eigen::Vector3d& center,
    double radius, const std_msgs::msg::ColorRGBA& color) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = center.x(); m.pose.position.y = center.y(); m.pose.position.z = center.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = radius * 2.0;
    m.color = color;
    return m;
}

std::string RvizPublisher::toAsciiDisplayText(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char c : text) {
        if (c >= 0x20 && c < 0x7F) out += c;
        else if (c != 0) out += ' ';
    }
    return out.empty() ? " " : out;
}

std::string RvizPublisher::moduleDisplayName(const std::string& name) {
    if (name == "前端" || name == "Frontend") return "Frontend";
    if (name == "后端" || name == "Backend") return "Backend";
    if (name == "回环" || name == "Loop") return "Loop";
    if (name == "GPS") return "GPS";
    if (name == "HBA" || name == "HBAOptimizer") return "HBA";
    if (name == "子图" || name == "Submap") return "Submap";
    if (name == "建图" || name == "Mapping") return "Mapping";
    return toAsciiDisplayText(name);
}

visualization_msgs::msg::Marker RvizPublisher::makeTextMarker(
    const std::string& ns, int id, const Eigen::Vector3d& position,
    const std::string& text, double size, const std_msgs::msg::ColorRGBA* color) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = position.x(); m.pose.position.y = position.y(); m.pose.position.z = position.z();
    m.pose.orientation.w = 1.0;
    m.scale.z = size;
    m.text = toAsciiDisplayText(text);
    m.color = color ? *color : makeColor(1.0f, 1.0f, 0.0f, 1.0f);
    return m;
}

visualization_msgs::msg::Marker RvizPublisher::makeBoundingBoxMarker(
    const std::string& ns, int id, const Pose3d& pose,
    const Eigen::Vector3d& scale, const std_msgs::msg::ColorRGBA& color) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pose.translation().x();
    m.pose.position.y = pose.translation().y();
    m.pose.position.z = pose.translation().z();
    Eigen::Quaterniond q(pose.rotation());
    m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z(); m.pose.orientation.w = q.w();
    m.scale.x = scale.x(); m.scale.y = scale.y(); m.scale.z = scale.z();
    m.color = color;
    return m;
}

visualization_msgs::msg::Marker RvizPublisher::makeCovarianceEllipse(
    const std::string& ns, int id, const Pose3d& pose,
    const Eigen::Matrix3d& cov_xy, double scale) const {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov_xy);
    Eigen::Vector3d ev = es.eigenvalues().cwiseMax(1e-9).cwiseSqrt() * scale;
    Eigen::Matrix3d R = es.eigenvectors();
    Pose3d T = Pose3d::Identity();
    T.translation() = pose.translation();
    T.linear() = pose.rotation() * R;
    std_msgs::msg::ColorRGBA c = makeColor(0.5f, 0.5f, 1.0f, 0.25f);
    return makeBoundingBoxMarker(ns, id, T, ev * 2.0, c);
}

visualization_msgs::msg::Marker RvizPublisher::makeDeleteMarker(const std::string& ns, int id) const {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::msg::Marker::DELETE;
    return m;
}

visualization_msgs::msg::MarkerArray RvizPublisher::makeDeleteAllMarkers(const std::string& ns) const {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = node() ? node()->now() : rclcpp::Time(0);
    m.ns = ns;
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(m);
    return arr;
}

}  // namespace automap_pro
