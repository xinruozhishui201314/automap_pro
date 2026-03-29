#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/v3/semantic_segmentor.h"
#include "automap_pro/v3/geometric_processor.h"
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>

class Instance;
struct FeatureModelParams;

namespace automap_pro::v3 {

/**
 * @brief SemanticProcessor: Wraps semantic segmentation and instance clustering
 * to extract tree landmarks from point clouds.
 */
class SemanticProcessor {
public:
    using Ptr = std::shared_ptr<SemanticProcessor>;

    struct Config {
        std::string model_type = "lsk3dnet";  // sloam | lsk3dnet | lsk3dnet_hybrid
        std::string model_path;
        std::string lsk3dnet_model_path;
        std::string lsk3dnet_device = "cpu";
        std::string lsk3dnet_repo_root;
        std::string lsk3dnet_config_yaml;
        std::string lsk3dnet_checkpoint;
        std::string lsk3dnet_classifier_torchscript;
        std::string lsk3dnet_python_exe = "python3";
        std::string lsk3dnet_worker_script;
        std::string lsk3dnet_hybrid_normal_mode = "range";
        float lsk3dnet_normal_fov_up_deg = 3.0f;
        float lsk3dnet_normal_fov_down_deg = -25.0f;
        int lsk3dnet_normal_proj_h = 64;
        int lsk3dnet_normal_proj_w = 900;
        float fov_up = 22.5f;
        float fov_down = -22.5f;
        int img_w = 2048;
        int img_h = 64;
        int lsk_num_vote = 1;
        bool lsk_training_volume_crop = false;
        bool lsk_volume_bounds_valid = false;
        float lsk_vol_min_x = 0.f;
        float lsk_vol_min_y = 0.f;
        float lsk_vol_min_z = 0.f;
        float lsk_vol_max_x = 0.f;
        float lsk_vol_max_y = 0.f;
        float lsk_vol_max_z = 0.f;
        int input_channels = 0;  // 0 means auto from model
        int num_classes = 0;     // 0 means auto from model
        int tree_class_id = -1;  // -1 means fallback policy in inferencer
        std::vector<float> input_mean;
        std::vector<float> input_std;
        bool do_destagger = true;
        
        // Clustering params
        float beam_cluster_threshold = 0.1f;
        float max_dist_to_centroid = 0.2f;
        int min_vertex_size = 2;
        int min_landmark_size = 4;
        float min_landmark_height = 1.0f;

        // Fitting params
        float default_tree_radius = 0.1f;
        float max_tree_radius = 0.5f;
        float max_axis_theta = 15.0f; // degrees from vertical
        float tree_truncation_height = 0.0f; // 0.0 means no truncation (e.g. 1.2 to keep bottom 1.2m)
        // Cylinder fitting height window above local ground (see ground_cloud + vertical_ref), not body z=0.
        // Only points with relative height along up in [min, max] participate in fit.
        float cylinder_fit_rel_z_min = 0.4f;
        float cylinder_fit_rel_z_max = 1.2f;
        /// Horizontal search radius (m) in the plane orthogonal to vertical_ref for sampling Patchwork ground under the cluster.
        float cylinder_fit_ground_search_radius_m = 5.0f;
        /// Minimum ground points in that neighborhood to trust ground-based level; else fallback to cluster min along up.
        int cylinder_fit_ground_min_samples = 30;
        /// Low percentile of ground samples' dot(up) as local ground level (0~1).
        float cylinder_fit_ground_percentile = 0.10f;
        /// Learning-label id written into the range mask for Patchwork ground (hybrid / geometric_only paint).
        /// SemanticKITTI-sub learning_map_inv: 9=road, 12=other-ground, 17=terrain; 13=building (must not use for ground).
        int geometric_ground_paint_class_id = 9;

        /// geometric_only：在 RANSAC LINE 之外，用水平栅格粗搜「地面–树冠」竖直柱，再走同一套圆柱拟合链，提高稀疏树干召回。
        bool geometric_only_shaft_enable = true;
        float geometric_only_shaft_xy_cell_m = 0.42f;
        float geometric_only_shaft_min_extent_m = 1.62f;
        int geometric_only_shaft_min_points = 14;
        float geometric_only_shaft_max_xy_spread_m = 0.95f;
        float geometric_only_shaft_refine_radius_m = 0.58f;
        float geometric_only_shaft_rel_z_max_m = 22.0f;
        int geometric_only_shaft_max_candidates = 64;
        float geometric_only_shaft_skip_near_line_xy_m = 0.52f;
        float geometric_only_shaft_skip_near_tree_xy_m = 0.55f;
        /// Max std/mean of radial distances to axis after Ceres; <=0 disables (person-like blobs tend higher).
        float cylinder_radial_cv_max = 0.42f;
        /// Min span along fitted axis (m); <=0 disables.
        float cylinder_min_axial_extent_m = 0.28f;
        /// line_ceres: RANSAC line + Ceres; pcl_sac: PCL SACMODEL_CYLINDER + normals; pcl_sac_ceres: PCL init + Ceres refine.
        std::string cylinder_fit_method = "line_ceres";
        /// Inlier distance threshold for PCL cylinder SAC (m).
        float cylinder_pcl_sac_distance = 0.08f;
        /// Normal estimation radius for PCL cylinder path (m); ignored when using k-search on small clouds.
        float cylinder_pcl_normal_radius = 0.22f;
        /// Min fit confidence (inlier ratio from line/PCL RANSAC vs point count) to keep a tree landmark [0,1].
        /// 过高会误杀真实树干；默认偏宽松，可按日志再收紧。
        float min_tree_confidence = 0.75f;
        /// 车体坐标系下树干参考点 root 到自车（原点）的水平距离 d_xy=||root.xy|| 若过小，真实路旁树干通常不应与车体重叠。
        /// >0：低于该值打 [SEMANTIC][TRUNK_EGO_CLEARANCE] 告警便于排查；0 表示关闭告警。
        float trunk_ego_xy_clearance_warn_m = 1.8f;
        /// 低于该水平距离则丢弃该树干；0 表示不按此项剔除（仅告警仍可由 warn_m 触发）。
        float trunk_ego_xy_clearance_reject_m = 0.0f;

        /// geometric_only：稀疏雷达下线状树干——在几何 LINE 轴线周围从非地面点云中收集「立柱」点，
        /// 若相对地面向上延伸足够高且上半段点数足够，则与窄带 trunk 点合并后重试圆柱拟合。
        bool sparse_trunk_column_enable = true;
        float sparse_trunk_column_radius_m = 0.40f;
        float sparse_trunk_min_vertical_extent_m = 1.0f;
        float sparse_trunk_upper_rel_z_min_m = 1.0f;
        float sparse_trunk_upper_rel_z_max_m = 14.0f;
        int sparse_trunk_min_upper_points = 12;
        int sparse_trunk_min_column_points = 20;
        int sparse_trunk_max_fit_points = 800;
        /// 标准柱门控（柱内点数/上半段点数）过严时：用语义结构证据——相对地面有竖向延伸、柱内下段较上段更粗（锥度）、
        /// 轴线外更大半径窗口内有足够「冠层」点（树叶团簇），避免仅依赖严重降采样后的离散树干点。
        bool sparse_trunk_structural_enable = true;
        float sparse_trunk_structural_min_extent_m = 0.68f;
        int sparse_trunk_structural_min_column_points = 5;
        int sparse_trunk_structural_min_foliage_points = 12;
        float sparse_trunk_structural_foliage_radius_m = 0.58f;
        float sparse_trunk_structural_foliage_rel_z_min_m = 1.0f;
        float sparse_trunk_structural_foliage_rel_z_max_m = 18.0f;
        /// 柱内点按高度分半后，下半径向中位数 / 上半径向中位数下限（略 <1 允许噪声；仅当柱内点数≥ taper_min_column_pts 时检查）。
        float sparse_trunk_structural_taper_min_ratio = 0.78f;
        int sparse_trunk_structural_taper_min_column_pts = 10;
        /// 参与锥度比较的点相对地面的最大高度（米以上），削弱高处冠层混入窄柱对锥度的破坏。
        float sparse_trunk_structural_taper_max_rel_z_m = 3.8f;
        int sparse_trunk_structural_merge_foliage_max_points = 320;
        /// 结构化门控（锥度+冠层）通过时：直接由合并点构造圆柱地标，跳过 band_plus_column 拟合与 fallback（避免稀疏真树被圆柱质检误杀）。
        bool sparse_trunk_structural_direct_cylinder = true;
        /// 结构化树干：在轴线周围环形区域内（柱半径外、外半径内）与柱同高度带内的非地面点数不应过大，否则多为墙/密集物误检。
        bool sparse_trunk_structural_ambient_check_enable = true;
        /// 环形区内径 = sparse_trunk_column_radius_m + 本值（米），避免把柱面点算进环境。
        float sparse_trunk_structural_ambient_inner_margin_m = 0.10f;
        /// 环形区外半径（米），相对轴线的水平距离上限。
        float sparse_trunk_structural_ambient_outer_radius_m = 1.35f;
        /// 上述环形区 + 柱高度带（相对地面）允许的最大点数；少量离散噪点可容忍。
        int sparse_trunk_structural_ambient_max_points = 45;
        /// 高度带在柱顶/底的扩展（米），避免边界切片漏计。
        float sparse_trunk_structural_ambient_rel_z_pad_m = 0.12f;
        /// 稀疏柱门控通过但圆柱拟合仍失败时，用 LINE 轴向 + 合并点估计半径/根部，构造退化树干地标。
        bool sparse_trunk_fallback_enable = true;
        /// 退化路径最低置信度（可低于 min_tree_confidence，避免稀疏场景全被拒）。
        float sparse_trunk_fallback_min_confidence = 0.50f;
        /// band_plus_column 圆柱仍失败时，对合并点做 PCA 主方向（近竖直则采用，否则退回重力轴）再退化拟合。
        bool sparse_trunk_fallback_pca_after_merge = true;
        /// 退化半径判定时 r_med 相对 max_tree_radius 的放宽倍数（仅 PCA/显式轴路径；默认 1.12 过严时常杀稀疏树干）。
        float sparse_trunk_fallback_r_med_slack = 1.32f;
        /// PCA 主轴与重力夹角余弦低于此则视为水平主导，直接采用竖直参考轴。
        float sparse_trunk_fallback_pca_min_up_cos = 0.45f;
        /// 使用 PCA/显式轴退化时，在 max_axis_theta 基础上额外允许的倾角（度）。
        float sparse_trunk_fallback_pca_extra_tilt_deg = 12.0f;

        // Diagnostic toggles (default off to preserve behavior)
        bool diag_enable_detailed_stats = false;
        bool diag_log_class_histogram = false;
        int diag_class_hist_top_k = 8;
        int diag_class_hist_interval_frames = 50;
        int diag_override_tree_class_id = -2;  // -2: no override; -1: auto; >=0: force class id
        std::string diag_cluster_profile = "default";  // default | relaxed
        std::string diag_cluster_input_mode = "sparse";  // sparse | dense_for_clustering
        bool diag_dump_all_classes = false;  // dump per-class pixel/point stats
        /// true：每根几何 LINE 输出 [SEMANTIC][TRUNK_CHAIN]；并打开圆柱子步骤 [SEMANTIC][TRUNK_FIT]、稀疏柱 [TRUNK_COLUMN]、退化 [TRUNK_FALLBACK]（量大，仅调参时开）
        bool diag_trunk_chain_log = false;
        int diag_dump_points_per_class_limit = 0;  // 0 disables point samples
        int diag_trellis_min_cluster_points = 80;  // keep legacy default
        int diag_trellis_min_tree_vertices = 16;   // keep legacy default

        // GeoSemantic-Fusion (Zhou22ral + Patchwork++)
        std::string mode = "geometric_only"; // model_only | geometric_only | hybrid
        bool geometric_enabled = true;
        std::string geometric_log_level = "info"; // off | info | debug
        bool geometric_log_detail = false;
        struct {
            float sensor_height = 1.73f;
            int num_iter = 3;
            float th_dist = 0.2f;
            float max_range = 80.0f;
            bool auto_sensor_height = true;
            float auto_height_min_xy_m = 4.0f;
            float auto_height_max_xy_m = 50.0f;
            int auto_height_min_samples = 400;
            float auto_height_percentile = 0.08f;
            float auto_height_clamp_min_m = 0.25f;
            float auto_height_clamp_max_m = 3.8f;
            float auto_height_ema_alpha = 0.12f;
            float auto_height_max_z_over_r = 0.55f;
            bool use_odom_gravity = true;
            int odom_up_axis = 2;
            bool level_cloud_for_patchwork = false;
        } patchwork;
        struct {
            bool enabled = true;
            float distance_threshold = 0.15f;
            int min_inliers = 100;
            float max_normal_tilt_deg = 15.0f;
            float line_distance_threshold = 0.10f;
            int line_min_inliers = 100;
            float plane_distance_threshold = 0.15f;
            int plane_min_inliers = 100;
        } wall_ransac;
        struct {
            bool enabled = true;
            int max_frames = 24;
            /// 多帧累加点云：将 intensity 写为几何处理扫描序号（与日志 idx 一致），用于追溯点到哪帧
            bool tag_intensity_with_scan_seq = true;
            /// false：不写几何调试用 PCD（总开关，忽略 save_merged_cloud_dir）
            bool save_debug_pcd = true;
            /// 调试输出目录（仅当 save_debug_pcd=true 且非空时生效）
            std::string save_merged_cloud_dir;
            int save_merged_cloud_every_n = 1;
            /// true：写多帧叠加 accum_body PCD
            bool save_accum_body_pcd = true;
            /// true：写进入 classifyPrimitives 前的 prim_input PCD
            bool save_primitive_input_cloud = false;
        } accumulator;
        struct {
            bool enabled = true;
            float linearity_threshold = 0.7f;
            float planarity_threshold = 0.6f;
        } primitive_classifier;
        struct {
            float tolerance_m = 0.5f;
            int min_points = 20;
            int max_points = 5000;
        } euclidean_cluster;
        struct {
            bool enabled = false;
            float body_xy_radius_m = 50.0f;
            float ring_min_xy_m = 3.0f;
            float ring_max_xy_m = 0.0f;
            float voxel_leaf_m = 0.0f;
        } primitive_roi;
        struct {
            bool enabled = false;
            std::string mode = "gradient";
            int image_width = 1024;
            int image_height = 64;
            float min_range_m = 0.5f;
            float max_range_m = 80.0f;
            float elev_min_deg = -24.0f;
            float elev_max_deg = 24.0f;
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
            int max_patch_points = 14000;
            bool fallback_full_cloud = true;
            std::string onnx_model_path;
            int onnx_input_width = 512;
            int onnx_input_height = 64;
            int onnx_n_classes = 4;
            int onnx_wall_class_id = 2;
            int onnx_trunk_class_id = 3;
            float fusion_rv_boost_scale = 0.35f;
        } range_view;
        int max_lines_per_cluster = 2;
        /// geometric_only：range_view_label 为立面启发（1=梯度墙，3=ONNX 墙）的 LINE 不进入树干圆柱链，减少墙误成树。整云回退（label=0）仍走树链。
        bool trunk_chain_skip_rv_wall_label = true;
        /// geometric_only：针对极稀疏场景，启用“地-干-冠”三层强连通性召回，降低对点数和锥度的要求。
        bool sparse_trunk_connectivity_recall_enable = true;
        int sparse_trunk_connectivity_min_per_layer = 2;
        float sparse_trunk_connectivity_canopy_radius_m = 2.5f;
        /// geometric_only: merge nearby trunk fits in the same frame (body frame). 0 = auto legacy formula.
        struct {
            float max_xy_m = 0.f;
            float max_z_m = 0.f;
            float max_axis_angle_deg = 0.f;
        } geometric_only_frame_merge;
    };

    explicit SemanticProcessor(const Config& config);
    ~SemanticProcessor();

    struct ProcessResult {
        std::vector<CylinderLandmark::Ptr> tree_landmarks;
        std::vector<PlaneLandmark::Ptr> plane_landmarks;
    };

    /**
     * @brief Extracts cylindrical and planar landmarks from a point cloud
     * @param cloud The input point cloud
     * @param ts Timestamp
     * @param T_odom_b Body pose in odom frame
     * @param out_labeled_cloud Optional output point cloud where intensity contains semantic label
     * @param out_trunk_pre_cluster_body maskCloud(tree_label) 后有效树干点，body 系（与 cloud 同坐标约定）
     * @param out_trunk_post_cluster_body Trellis 聚类后点云，body 系，intensity=簇序号 1..N
     * @return ProcessResult containing extracted landmarks
     */
    ProcessResult process(const CloudXYZIConstPtr& cloud,
                          double ts,
                          const Eigen::Isometry3d& T_odom_b,
                          CloudXYZIPtr* out_labeled_cloud = nullptr,
                          CloudXYZIPtr* out_trunk_pre_cluster_body = nullptr,
                          CloudXYZIPtr* out_trunk_post_cluster_body = nullptr);
    bool hasRuntimeCapability() const;

private:
    enum class FitRejectReason {
        kSuccess = 0,
        kRansacReject,
        kCeresFail,
        kRejectRadius,
        kRejectTilt,
        kRejectHeightWindow,
        kRejectCylinderRadialCv,
        kRejectCylinderAxialSpan,
    };

    Config config_;
    std::unique_ptr<ISemanticSegmentor> segmentor_;
    std::unique_ptr<GeometricProcessor> geometric_processor_;
    std::unique_ptr<Instance> instance_detector_;
    std::unique_ptr<FeatureModelParams> fm_params_;
    mutable std::recursive_mutex resource_mutex_;
    std::atomic<size_t> consecutive_failures_{0};
    std::atomic<bool> runtime_disabled_{false};
    static constexpr size_t kMaxConsecutiveFailures = 10;
    std::atomic<uint64_t> processed_frames_{0};
    std::atomic<uint64_t> empty_tree_frames_{0};
    std::atomic<uint64_t> empty_cluster_frames_{0};
    std::atomic<uint64_t> empty_fit_frames_{0};
    std::atomic<uint64_t> fit_input_clusters_total_{0};
    std::atomic<uint64_t> fit_success_total_{0};
    std::atomic<uint64_t> fit_reject_ransac_total_{0};
    std::atomic<uint64_t> fit_reject_ceres_total_{0};
    std::atomic<uint64_t> fit_reject_radius_total_{0};
    std::atomic<uint64_t> fit_reject_tilt_total_{0};
    std::atomic<uint64_t> fit_reject_trunk_near_ego_total_{0};
    std::chrono::steady_clock::time_point last_stats_log_tp_{std::chrono::steady_clock::now()};
    /// Body-frame vertical reference for trunk tilt / axis canonicalization (from GeometricProcessor odom gravity or +Z).
    Eigen::Vector3d vertical_ref_body_{0.0, 0.0, 1.0};
    /// Single-frame body frame: reject/warn when horizontal distance from trunk root to ego is too small.
    bool trunkPassesEgoClearance_(const Eigen::Vector3d& root_body, uint64_t landmark_id, const char* stage_tag) const;

    /// geometric_only：对单条 LINE 或 shaft 伪 LINE 执行窄带/柱合并/圆柱拟合链；失败返回 nullptr。
    CylinderLandmark::Ptr geometricOnlyTryTrunkChain_(
        const GeometricResult::Primitive& prim,
        const std::vector<pcl::PointXYZI>& raw_line_points,
        float band_rel_h_lo,
        float band_rel_h_hi,
        const GeometricResult& geo_result,
        const CloudXYZIConstPtr& cloud,
        double ts,
        int trunk_line_idx,
        size_t& rejected_line_cylinder_fit,
        size_t& rejected_line_tilt,
        size_t& rejected_line_low_confidence,
        size_t& rejected_line_trunk_near_ego,
        size_t& sparse_trunk_column_gate_pass,
        size_t& sparse_trunk_structural_gate_pass,
        size_t& sparse_trunk_structural_direct_accepted,
        size_t& sparse_trunk_fit_recovered,
        size_t& sparse_trunk_fallback_accepted);

    /// 圆柱拟合分阶段日志关联：`diag_trunk_chain_log` 或 `geometric_log_level`/`geometric_log_detail` 开启且指针非空时，
    /// `fitCylinder` 输出 `[SEMANTIC][TRUNK_FIT]` 各子步骤（便于定位 RANSAC/Ceres/CV/轴向跨度等失败点）。
    struct CylinderFitDiagFrame {
        double ts = 0.0;
        /// geometric_only：几何 LINE 序号；模型聚类路径：簇序号（从 0 递增）。
        int context_idx = -1;
        const char* pass_tag = "";
    };

    CylinderLandmark::Ptr fitCylinder(const std::vector<pcl::PointXYZI>& points,
                                      FitRejectReason* reason = nullptr,
                                      double* out_tilt_deg = nullptr,
                                      const CylinderFitDiagFrame* diag_frame = nullptr);

    /// PCL Sample Consensus cylinder (SACMODEL_CYLINDER + normals). See pcl::SampleConsensusModelCylinder.
    bool fitCylinderTryPclSac(const CloudXYZIConstPtr& cloud,
                              double root[3],
                              double ray[3],
                              double* radius,
                              size_t* inlier_count,
                              FitRejectReason* reason,
                              bool log_stages = false);
};

} // namespace automap_pro::v3
