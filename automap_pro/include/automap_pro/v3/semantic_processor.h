#pragma once

#include "automap_pro/core/data_types.h"
#include "automap_pro/v3/semantic_segmentor.h"
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
        std::string model_type = "sloam";  // sloam | lsk3dnet | lsk3dnet_hybrid
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

        // Diagnostic toggles (default off to preserve behavior)
        bool diag_enable_detailed_stats = false;
        bool diag_log_class_histogram = false;
        int diag_class_hist_top_k = 8;
        int diag_class_hist_interval_frames = 50;
        int diag_override_tree_class_id = -2;  // -2: no override; -1: auto; >=0: force class id
        std::string diag_cluster_profile = "default";  // default | relaxed
        std::string diag_cluster_input_mode = "sparse";  // sparse | dense_for_clustering
        bool diag_dump_all_classes = false;  // dump per-class pixel/point stats
        int diag_dump_points_per_class_limit = 0;  // 0 disables point samples
        int diag_trellis_min_cluster_points = 80;  // keep legacy default
        int diag_trellis_min_tree_vertices = 16;   // keep legacy default
    };

    explicit SemanticProcessor(const Config& config);
    ~SemanticProcessor();

    /**
     * @brief Extracts cylindrical landmarks from a point cloud
     * @param cloud The input point cloud
     * @return A vector of extracted CylinderLandmarks
     */
    std::vector<CylinderLandmark::Ptr> process(const CloudXYZIConstPtr& cloud);
    bool hasRuntimeCapability() const;

private:
    Config config_;
    std::unique_ptr<ISemanticSegmentor> segmentor_;
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
    std::chrono::steady_clock::time_point last_stats_log_tp_{std::chrono::steady_clock::now()};
    CylinderLandmark::Ptr fitCylinder(const std::vector<pcl::PointXYZI>& points);
};

} // namespace automap_pro::v3
