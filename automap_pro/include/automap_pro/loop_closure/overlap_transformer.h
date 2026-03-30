#pragma once
/**
 * @file overlap_transformer.h
 * @brief 深度学习地点识别描述子封装（与 infer 分工：此处可为高层 API）；无模型时用类 M2DP 投影直方图 fallback。
 */
#include <vector>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

/** @brief LiDAR 全局描述子 + Top-K 检索（LibTorch 可选）。 */
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
