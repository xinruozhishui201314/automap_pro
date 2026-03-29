#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <memory>
#include <vector>
#include <deque>
#include <mutex>
#include <string>
#include "automap_pro/core/data_types.h"

namespace automap_pro::v3 {

struct GeometricProcessorConfig {
    struct {
        float sensor_height = 1.73f;
        int num_iter = 3;
        float th_dist = 0.2f;
        float max_range = 80.0f;
        /// 每帧从点云估计 Patchwork++ sensor_height（车体/雷达系，地面通常在 z<0）；false 时仅用 sensor_height 常量。
        bool auto_sensor_height = true;
        /// 参与估计的水平距离环 [min_xy, max_xy]（米），避开近距盲区与极远噪声。
        float auto_height_min_xy_m = 4.0f;
        float auto_height_max_xy_m = 50.0f;
        /// 估计用最小样本数；不足时自动放宽环带再试。
        int auto_height_min_samples = 400;
        /// 取 z 的该分位数作为地面参考（0~1，建议 0.05~0.12）。
        float auto_height_percentile = 0.08f;
        /// 估计值夹紧范围（米）；失败时回退 sensor_height。
        float auto_height_clamp_min_m = 0.25f;
        float auto_height_clamp_max_m = 3.8f;
        /// EMA 平滑系数（0=每帧直接用原始估计；建议 0.08~0.2）。
        float auto_height_ema_alpha = 0.12f;
        /// 仅保留 |z|/r 小于该值的点参与分位数（抑制立面/上空点污染地面高度估计）。
        float auto_height_max_z_over_r = 0.55f;
        /// 用车体在里程计系位姿估计「竖直向上」在 body 中的方向（R^T * e_up_odom），用于墙面法向与树干倾角；false 时等价于 body +Z。
        bool use_odom_gravity = true;
        /// 里程计/地图系竖直轴：0=x,1=y,2=z（FAST-LIVO 类 world 通常 z 向上）。
        int odom_up_axis = 2;
        /// 在 Patchwork 前将点云绕原点旋转，使 odom 竖直对齐到 body +Z，再分割后旋回 body；大倾角场景可改善地面/非地面划分（默认关）。
        bool level_cloud_for_patchwork = false;
    } patchwork;
    struct {
        bool enabled = true;
        float distance_threshold = 0.15f;
        int min_inliers = 100;
        float max_normal_tilt_deg = 15.0f;
        // Split thresholds for density-varying scenes. Defaults preserve legacy behavior.
        float line_distance_threshold = 0.10f;
        int line_min_inliers = 100;
        float plane_distance_threshold = 0.15f;
        int plane_min_inliers = 100;
    } wall_ransac;
    struct {
        bool enabled = true;
        int max_frames = 24;
        /// If true, merged multi-frame cloud overwrites PointXYZI::intensity with GeometricProcessor scan index
        /// (monotonic, same as [GEOMETRIC][FRAME] idx) so each point is traceable to its source sweep.
        bool tag_intensity_with_scan_seq = true;
        /// Master: false = never write debug PCDs below (ignores save_merged_cloud_dir; avoids disk flood).
        bool save_debug_pcd = true;
        /// Non-empty directory for debug PCDs when save_debug_pcd is true.
        std::string save_merged_cloud_dir;
        /// Save every N geometric frames (1 = each time the save branch runs).
        int save_merged_cloud_every_n = 1;
        /// Save accumulate() merged body cloud (tag accum_body) when save_debug_pcd && dir non-empty.
        bool save_accum_body_pcd = true;
        /// Save cloud passed to classifyPrimitives (after 2nd Patchwork + optional ROI; tag prim_input); same dir/every_n.
        bool save_primitive_input_cloud = false;
    } accumulator;
    struct {
        bool enabled = true;
        float linearity_threshold = 0.7f;
        float planarity_threshold = 0.6f;
    } primitive_classifier;
    /// Euclidean clustering of nonground points before primitive classification (PCL).
    struct {
        float tolerance_m = 0.5f;
        int min_points = 20;
        int max_points = 5000;
    } euclidean_cluster;
    /// Patchwork 之后、欧氏聚类 / wall·line RANSAC 之前：在车体系 xy 上做局部环带裁剪（大关键帧稀疏云时提高局部密度）。
    /// 点云已在当前帧 body（与 T_odom_b 一致）；不修改 result.nonground_cloud，仅缩小送入 classifyPrimitives 的点集。
    struct PrimitiveRoi {
        bool enabled = false;
        /// 水平圆柱外半径（米）：保留 sqrt(x^2+y^2) <= R；0 表示不设外边界（仍可用 ring_max_xy_m）。
        float body_xy_radius_m = 50.0f;
        /// 内半径（米）：剔除近距盲区/车体；0 表示不裁内侧。
        float ring_min_xy_m = 3.0f;
        /// 外半径上界（米）：0 表示仅用 body_xy_radius_m；>0 时与 body_xy_radius_m 取较小者为实际上界。
        float ring_max_xy_m = 0.0f;
        /// 体素叶子边长（米）；0 关闭。用于控 CPU / 归一化密度（在 xy 裁剪之后执行）。
        float voxel_leaf_m = 0.0f;
    } primitive_roi;
    /// 低分辨率 range view：梯度+连通域（V1）或 OpenCV DNN 读 ONNX（V2），仅在候选 3D patch 上跑聚类/RANSAC；可显著减全云欧氏聚类成本。
    struct RangeView {
        bool enabled = false;
        /// none | gradient | onnx — onnx 依赖 OpenCV dnn + 有效模型路径；失败时回退 gradient（若仍失败且 fallback_full_cloud 则整云聚类）。
        std::string mode = "gradient";
        int image_width = 1024;
        int image_height = 64;
        float min_range_m = 0.5f;
        float max_range_m = 80.0f;
        float elev_min_deg = -24.0f;
        float elev_max_deg = 24.0f;
        /// 梯度候选
        float grad_mag_norm_thresh = 0.07f;
        int dilate_iterations = 1;
        int min_cc_pixels = 6;
        int max_cc_pixels = 12000;
        int wall_min_width_u = 10;
        float wall_max_aspect_h_over_w = 8.0f;
        int trunk_max_width_u = 10;
        float trunk_min_aspect_h_over_w = 1.2f;
        int bbox_margin_u = 2;
        int bbox_margin_v = 2;
        int max_patches_per_frame = 64;
        /// 单 patch 点数上限（防止单连通域过大拖慢）；超出则随机下采样到该值（顺序遍历抽稀）。
        int max_patch_points = 14000;
        bool fallback_full_cloud = true;
        /// ONNX（OpenCV readNetFromONNX）；空路径则跳过
        std::string onnx_model_path;
        int onnx_input_width = 512;
        int onnx_input_height = 64;
        int onnx_n_classes = 4;
        int onnx_wall_class_id = 2;
        int onnx_trunk_class_id = 3;
        /// 供下游 Landmark 置信度：residual *= (1 + fusion_rv_boost_scale * range_view_score)
        float fusion_rv_boost_scale = 0.35f;
    } range_view;
    /// After a linear cluster yields one RANSAC line, optionally peel inliers and fit again (sparse LiDAR:
    /// several trunks often share one 0.5m cluster). 1 = legacy single line per cluster.
    int max_lines_per_cluster = 2;
    std::string log_level = "info"; // off | info | debug
    bool log_detail = false;
};

/**
 * @brief 几何语义辨识结果，包含地面、墙面、杆状物标签
 */
struct GeometricResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud;
    
    // 辨识出的几何基元（Zhou22ral）
    struct Primitive {
        enum class Type { PLANE, LINE, CYLINDER, UNKNOWN };
        Type type;
        pcl::PointCloud<pcl::PointXYZI>::Ptr points;
        Eigen::Matrix<double, 6, 1> model_coeffs; // 平面使用前4维；线使用6维
        double linearity;
        double planarity;
        double residual;
        /// Range-view 候选置信度 [0,1]；0 表示未走 RV 或未命中。
        float range_view_score = 0.f;
        /// 1=梯度立面启发，2=梯度树干启发，3=ONNX 墙，4=ONNX 干，0=未知/整云聚类。
        uint8_t range_view_label = 0;
        bool range_view_from_nn = false;
    };
    std::vector<Primitive> primitives;
    /// 当前帧 body 系下「竖直向上」单位向量（来自里程计姿态或默认 +Z），供语义树干倾角等与几何一致。
    Eigen::Vector3d gravity_up_body{0.0, 0.0, 1.0};
};

/**
 * @brief 帧累加器，用于应对低线数 LiDAR 的稀疏性
 */
class FrameAccumulator {
public:
    struct Frame {
        double timestamp = 0.0;
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
        Eigen::Isometry3d T_odom_b = Eigen::Isometry3d::Identity();
        /// Monotonic index when the sweep was ingested (matches GeometricProcessor process log idx).
        uint64_t scan_seq = 0;
    };

    explicit FrameAccumulator(int max_frames = 24, bool tag_intensity_with_scan_seq = true);

    void addFrame(double ts,
                  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud,
                  const Eigen::Isometry3d& T_odom_b,
                  uint64_t scan_seq);
    
    /** 将缓存帧补偿到当前帧 T_curr 下并合并 */
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulate(const Eigen::Isometry3d& T_curr);

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
    }

private:
    int max_frames_;
    bool tag_intensity_with_scan_seq_;
    std::deque<Frame> frames_;
    std::mutex mutex_;
};

/**
 * @brief 几何语义处理器：集成 Patchwork++ 与 Zhou22ral 辨识逻辑
 */
class GeometricProcessor {
public:
    using Ptr = std::shared_ptr<GeometricProcessor>;

    /// 不透明实现类型；完整定义仅在 geometric_processor.cpp，供 TU 内辅助函数签名使用。
    struct Impl;

    explicit GeometricProcessor(const GeometricProcessorConfig& config);
    ~GeometricProcessor();

    /** 处理点云，提取几何语义 */
    GeometricResult process(double ts, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud, const Eigen::Isometry3d& T_odom_b);

private:
    GeometricProcessorConfig config_;
    std::unique_ptr<FrameAccumulator> accumulator_;

    // 第三方算法实例 (需在 .cpp 中包含头文件以避免依赖污染)
    std::unique_ptr<Impl> impl_;

    /** 基元预分类 (Zhou22ral eigenvalue analysis)；可选 range-view 候选驱动 patch。 */
    void classifyPrimitives(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                            std::vector<GeometricResult::Primitive>& out_primitives,
                            const Eigen::Vector3d& gravity_up_body,
                            uint64_t frame_idx);

    /** 根据当前帧点云更新 Patchwork sensor_height（可重复调用；内部 EMA）。 */
    void applyPatchworkAutoSensorHeight_(uint64_t frame_idx, double ts, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);

    double patchwork_height_ema_{0.0};
    bool patchwork_height_ema_inited_{false};
    /// Last `sensor_height` passed to Patchwork++ (meters), for closed-loop log evidence.
    double last_patchwork_sensor_height_m_{0.0};
};

} // namespace automap_pro::v3
