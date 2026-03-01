#pragma once

#include <vector>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// OverlapTransformer: deep-learning loop place recognition
// Produces 256-d global descriptors from LiDAR range images.
// When LibTorch is not available, uses a hand-crafted fallback
// descriptor (M2DP-like: projection histogram).
// ──────────────────────────────────────────────────────────
class OverlapTransformer {
public:
    OverlapTransformer();
    ~OverlapTransformer() = default;

    bool loadModel(const std::string& model_path);
    bool isModelLoaded() const;

    // Generate descriptor for a submap point cloud
    Eigen::VectorXf computeDescriptor(const CloudXYZIPtr& cloud) const;

    // Generate range image from point cloud
    cv::Mat generateRangeImage(const CloudXYZIPtr& cloud) const;

    // Retrieve top-K candidates from descriptor DB
    struct Candidate {
        int submap_id   = -1;
        int session_id  = 0;
        float score     = 0.0f;
    };

    std::vector<Candidate> retrieve(
        const Eigen::VectorXf& query_desc,
        const std::vector<SubMap::Ptr>& db_submaps,
        int top_k = 5,
        double min_score = 0.3,
        int min_submap_gap = 3,
        double min_temporal_gap = 30.0,
        double gps_search_radius = 200.0,
        const Eigen::Vector3d& query_gps_pos = Eigen::Vector3d::Zero(),
        bool query_has_gps = false) const;

private:
    int range_image_h_;
    int range_image_w_;
    int descriptor_dim_;
    bool model_loaded_ = false;

    // Fallback: projection-based descriptor (M2DP-inspired)
    Eigen::VectorXf computeFallbackDescriptor(const CloudXYZIPtr& cloud) const;

    // Compute cosine similarity
    static float cosineSimilarity(const Eigen::VectorXf& a, const Eigen::VectorXf& b);

#ifdef USE_TORCH
    // LibTorch model placeholder
    std::shared_ptr<void> model_;  // torch::jit::Module
#endif
};

}  // namespace automap_pro
