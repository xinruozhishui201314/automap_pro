#pragma once

#include <memory>
#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/backend/pose_graph.h"

namespace automap_pro {

/**
 * RvizPublisher - 综合可视化发布器
 *
 * 功能模块：
 *   1. 点云可视化：全局地图、当前帧、子图点云
 *   2. 轨迹可视化：里程计轨迹、优化后轨迹、GPS轨迹
 *   3. 回环可视化：回环约束连线、候选点云叠加
 *   4. 子图可视化：子图边界框、标签、连接图
 *   5. GPS可视化：GPS标记、对齐变换、质量指示
 *   6. 优化可视化：协方差椭圆、因子图、收敛曲线
 *   7. 系统状态可视化：模块状态、坐标轴、性能指标
 */
class RvizPublisher {
public:
    RvizPublisher() = default;
    ~RvizPublisher() = default;

    void init(rclcpp::Node::SharedPtr node);

    // ═══════════════════════════════════════════════════════════════════════
    // 1. 点云可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布全局地图点云（自动降采样） */
    void publishGlobalMap(const CloudXYZIPtr& cloud);

    /** 发布当前帧点云 */
    void publishCurrentCloud(const CloudXYZIPtr& cloud);

    /** 发布子图点云 */
    void publishSubmapCloud(const SubMap::Ptr& sm);

    /** 发布带颜色编码的点云（按高度/强度/子图ID） */
    void publishColoredCloud(const CloudXYZIPtr& cloud,
                             const std::string& color_mode,
                             const std::string& topic_name = "colored_cloud");

    /** 发布点云密度热力图 */
    void publishDensityHeatmap(const CloudXYZIPtr& cloud, float grid_size = 5.0f);

    // ═══════════════════════════════════════════════════════════════════════
    // 2. 轨迹可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布里程计轨迹 */
    void publishOdometryPath(const std::vector<std::pair<double, Pose3d>>& path);

    /** 发布优化后轨迹 */
    void publishOptimizedPath(const std::vector<SubMap::Ptr>& submaps);

    /** 发布关键帧位姿数组 */
    void publishKeyframePoses(const std::vector<KeyFrame::Ptr>& keyframes);

    /**
     * 发布 GPS 轨迹 Path：/automap/gps_raw_path（原始）、/automap/gps_aligned_path（对齐后）。
     * 若传入 gps_positions_map 非空，raw path 使用地图系坐标，frame_id=raw_path_frame_id（默认 "map"）；
     * 否则 raw 使用子图 gps_center（ENU），frame_id=raw_path_frame_id，未对齐时应传 "enu" 避免 ENU 标成 map。
     */
    void publishGPSTrajectory(const std::vector<SubMap::Ptr>& submaps,
                              bool show_aligned = true);
    void publishGPSTrajectory(const std::vector<SubMap::Ptr>& submaps,
                              const std::vector<Eigen::Vector3d>& gps_positions_map,
                              bool show_aligned = true,
                              const std::string& raw_path_frame_id = "map");

    /** 发布轨迹对比（优化前后） */
    void publishTrajectoryComparison(
        const std::vector<std::pair<double, Pose3d>>& odom_path,
        const std::vector<std::pair<double, Pose3d>>& opt_path);

    // ═══════════════════════════════════════════════════════════════════════
    // 3. 回环检测可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布回环约束连线 */
    void publishLoopMarkers(const std::vector<LoopConstraint::Ptr>& loops,
                            const std::vector<SubMap::Ptr>& submaps);

    /** 发布回环候选点云叠加（用于调试） */
    void publishLoopCandidateClouds(const SubMap::Ptr& query,
                                    const SubMap::Ptr& candidate,
                                    int loop_id);

    /** 发布回环检测状态（描述子相似度热力图） */
    void publishLoopDetectionStatus(const SubMap::Ptr& query,
                                    const std::vector<std::pair<int, float>>& scores);

    /** 清除回环候选可视化 */
    void clearLoopCandidateClouds();

    // ═══════════════════════════════════════════════════════════════════════
    // 4. 子图可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布子图边界标签 */
    void publishSubmapBoundaries(const std::vector<SubMap::Ptr>& submaps);

    /** 发布子图3D边界框 */
    void publishSubmapBoundingBoxes(const std::vector<SubMap::Ptr>& submaps);

    /** 发布子图连接图（里程计约束） */
    void publishSubmapGraph(const std::vector<SubMap::Ptr>& submaps);

    /** 发布单个子图详情（选中时） */
    void publishSubmapDetail(const SubMap::Ptr& submap);

    // ═══════════════════════════════════════════════════════════════════════
    // 5. GPS可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布GPS标记点 */
    void publishGPSMarkers(const std::vector<SubMap::Ptr>& submaps);

    /** 发布GPS对齐结果可视化 */
    void publishGPSAlignment(const GPSAlignResult& result,
                            const std::vector<SubMap::Ptr>& submaps);

    /** 发布GPS质量指示（按HDOP/卫星数着色） */
    void publishGPSQualityMarkers(const std::vector<SubMap::Ptr>& submaps);

    /** 发布真实GPS位置（已转换到全局地图坐标系），供 RViz 显示 */
    void publishGPSPositionsInMap(const std::vector<Eigen::Vector3d>& positions_map);

    /**
     * 发布 GPS 约束标记与约束线（同 topic 一次发布，避免覆盖）。
     * gps_positions_map 与有 has_valid_gps 的子图一一对应（顺序一致），表示各地图系下 GPS 位置。
     */
    void publishGPSMarkersWithConstraintLines(
        const std::vector<SubMap::Ptr>& submaps,
        const std::vector<Eigen::Vector3d>& gps_positions_map);

    // ═══════════════════════════════════════════════════════════════════════
    // 6. 后端优化可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布位姿协方差椭圆 */
    void publishCovarianceEllipses(const std::vector<SubMap::Ptr>& submaps);

    /** 发布因子图结构（节点+边） */
    void publishFactorGraph(const PoseGraph& graph);

    /** 发布HBA优化结果 */
    void publishHBAResult(const HBAResult& result);

    /** 发布优化残差收敛曲线 */
    void publishOptimizationConvergence(const std::vector<double>& residuals);

    /** 发布位姿更新动画（优化过程） */
    void publishPoseUpdateAnimation(const std::vector<Pose3d>& before,
                                   const std::vector<Pose3d>& after);

    // ═══════════════════════════════════════════════════════════════════════
    // 7. 系统状态可视化
    // ═══════════════════════════════════════════════════════════════════════

    /** 发布坐标系（world, base_link, sensor） */
    void publishCoordinateFrames(const Pose3d& base_pose = Pose3d::Identity(),
                                const Pose3d& sensor_pose = Pose3d::Identity());

    /** 发布模块状态指示灯 */
    void publishModuleStatus(const std::map<std::string, int>& module_status);

    /** 发布当前子图活跃区域 */
    void publishActiveRegion(const SubMap::Ptr& active_submap);

    /** 发布ESIKF退化区域标注 */
    void publishDegenerationRegions(const std::vector<std::pair<double, Pose3d>>& degraded_poses);

    // ═══════════════════════════════════════════════════════════════════════
    // 8. 工具函数
    // ═══════════════════════════════════════════════════════════════════════

    /** 清除所有Marker */
    void clearAllMarkers();

    /** 清除指定命名空间的Marker */
    void clearMarkers(const std::string& ns);

    /** 设置全局帧ID */
    void setFrameId(const std::string& frame_id) { frame_id_ = frame_id; }

    /** 获取全局帧ID */
    std::string frameId() const { return frame_id_; }

private:
    rclcpp::Node::SharedPtr node_;
    std::string frame_id_ = "world";

    // ── 发布者 ─────────────────────────────────────────────────────────────

    // 点云发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr density_heatmap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_candidate_pub_;

    // Marker发布者
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gps_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submap_boundary_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submap_bbox_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submap_graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr covariance_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr factor_graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gps_quality_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gps_positions_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr module_status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frame_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr active_region_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr degen_region_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hba_result_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr convergence_pub_;

    /** 收敛曲线数据（供 PlotJuggler 等绘制迭代-残差） */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr convergence_residual_pub_;

    // 路径发布者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_raw_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr gps_aligned_path_pub_;

    // 位姿数组发布者
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr kf_pose_array_pub_;

    // ── 配置参数 ───────────────────────────────────────────────────────────
    double global_map_ds_res_ = 0.5;
    bool   publish_global_map_ = true;
    int    marker_id_counter_ = 0;

    // ── 私有辅助方法 ───────────────────────────────────────────────────────

    /** 点云转换到ROS消息 */
    CloudXYZIPtr toROSCloud(const CloudXYZIPtr& cloud, double ds_res = 0.0) const;

    /** 创建颜色 */
    std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a = 1.0f) const;

    /** 创建颜色（按值映射） */
    std_msgs::msg::ColorRGBA makeColorMap(float value, const std::string& cmap = "jet") const;

    /** 创建坐标轴Marker（三轴：红X、绿Y、蓝Z） */
    visualization_msgs::msg::MarkerArray makeAxisMarkers(
        const std::string& ns, int base_id, const Pose3d& pose, double scale = 1.0) const;

    /** 创建箭头Marker */
    visualization_msgs::msg::Marker makeArrowMarker(
        const std::string& ns, int id,
        const Eigen::Vector3d& start, const Eigen::Vector3d& end,
        const std_msgs::msg::ColorRGBA& color, double shaft_diameter = 0.05) const;

    /** 创建线段Marker */
    visualization_msgs::msg::Marker makeLineMarker(
        const std::string& ns, int id,
        const Eigen::Vector3d& start, const Eigen::Vector3d& end,
        const std_msgs::msg::ColorRGBA& color, double width = 0.05) const;

    /** 创建球体Marker */
    visualization_msgs::msg::Marker makeSphereMarker(
        const std::string& ns, int id, const Eigen::Vector3d& center,
        double radius, const std_msgs::msg::ColorRGBA& color) const;

    /** 创建文本Marker（默认黄色） */
    visualization_msgs::msg::Marker makeTextMarker(
        const std::string& ns, int id, const Eigen::Vector3d& position,
        const std::string& text, double size = 0.5,
        const std_msgs::msg::ColorRGBA* color = nullptr) const;

    /** 创建边界框Marker */
    visualization_msgs::msg::Marker makeBoundingBoxMarker(
        const std::string& ns, int id, const Pose3d& pose,
        const Eigen::Vector3d& scale, const std_msgs::msg::ColorRGBA& color) const;

    /** 创建协方差椭圆Marker（3D椭球） */
    visualization_msgs::msg::Marker makeCovarianceEllipse(
        const std::string& ns, int id, const Pose3d& pose,
        const Eigen::Matrix3d& cov_xy, double scale = 3.0) const;

    /** 删除Marker */
    visualization_msgs::msg::Marker makeDeleteMarker(const std::string& ns, int id) const;

    /** 删除命名空间所有Marker */
    visualization_msgs::msg::MarkerArray makeDeleteAllMarkers(const std::string& ns) const;

    /** 将文本转为 RViz 可正确显示的 ASCII（非 ASCII 替换为空格，避免显示为方框） */
    static std::string toAsciiDisplayText(const std::string& text);

    /** 模块名中文→英文，用于 RViz 状态文本 */
    static std::string moduleDisplayName(const std::string& name);
};

}  // namespace automap_pro
