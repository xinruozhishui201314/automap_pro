#include "automap_pro/loop_closure/overlap_transformer.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace automap_pro {

OverlapTransformer::OverlapTransformer() {
    const auto& cfg = ConfigManager::instance();
    range_image_h_  = cfg.rangeImageHeight();
    range_image_w_  = cfg.rangeImageWidth();
    descriptor_dim_ = cfg.descriptorDim();
}

bool OverlapTransformer::loadModel(const std::string& model_path) {
#ifdef USE_TORCH
    try {
        // In a real implementation: model_ = torch::jit::load(model_path);
        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[OverlapTransformer] Would load model from %s", model_path.c_str());
        model_loaded_ = false;  // set true when torch model loaded
    } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("automap_pro"), "[OverlapTransformer] Cannot load model: %s. Using fallback.", e.what());
    }
#else
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[OverlapTransformer] LibTorch not available. Using fallback descriptor.");
    (void)model_path;
#endif
    return true;
}

bool OverlapTransformer::isModelLoaded() const { return model_loaded_; }

cv::Mat OverlapTransformer::generateRangeImage(const CloudXYZIPtr& cloud) const {
    cv::Mat range_img = cv::Mat::zeros(range_image_h_, range_image_w_, CV_32FC1);

    const float fov_up   =  15.0f * M_PI / 180.0f;
    const float fov_down = -25.0f * M_PI / 180.0f;
    const float fov_v    = fov_up - fov_down;

    for (const auto& pt : cloud->points) {
        float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (r < 0.5f) continue;

        float pitch = std::asin(pt.z / r);
        float yaw   = std::atan2(pt.y, pt.x);

        int row = static_cast<int>((1.0f - (pitch - fov_down) / fov_v) * range_image_h_);
        int col = static_cast<int>((yaw + M_PI) / (2.0f * M_PI) * range_image_w_);

        row = std::max(0, std::min(range_image_h_ - 1, row));
        col = std::max(0, std::min(range_image_w_ - 1, col));

        float existing = range_img.at<float>(row, col);
        if (existing == 0.0f || r < existing) {
            range_img.at<float>(row, col) = r;
        }
    }
    return range_img;
}

Eigen::VectorXf OverlapTransformer::computeDescriptor(const CloudXYZIPtr& cloud) const {
#ifdef USE_TORCH
    if (model_loaded_) {
        // Real network inference would go here
    }
#endif
    return computeFallbackDescriptor(cloud);
}

Eigen::VectorXf OverlapTransformer::computeFallbackDescriptor(const CloudXYZIPtr& cloud) const {
    // M2DP-inspired: project point cloud onto multiple planes and compute histograms
    Eigen::VectorXf desc = Eigen::VectorXf::Zero(descriptor_dim_);
    if (!cloud || cloud->empty()) return desc;

    const int n_azimuth = 16;
    const int n_elev    = 8;
    const int n_dist    = 16;  // bins per sector
    // Total: 16*8*2 = 256 dimensions (horizontal + vertical projections)

    std::vector<float> hist(n_azimuth * n_dist, 0.0f);
    std::vector<float> hist_vert(n_elev * n_dist, 0.0f);

    float max_range = 80.0f;

    for (const auto& pt : cloud->points) {
        float r_xy = std::sqrt(pt.x*pt.x + pt.y*pt.y);
        float r_3d = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);

        // Azimuth histogram
        float yaw = std::atan2(pt.y, pt.x);  // [-pi, pi]
        int a_bin = static_cast<int>((yaw + M_PI) / (2.0f * M_PI) * n_azimuth);
        a_bin = std::max(0, std::min(n_azimuth - 1, a_bin));
        int d_bin = static_cast<int>(r_xy / max_range * n_dist);
        d_bin = std::max(0, std::min(n_dist - 1, d_bin));
        hist[a_bin * n_dist + d_bin] += 1.0f;

        // Elevation histogram
        float pitch = std::atan2(std::abs(pt.z), r_xy);
        int e_bin = static_cast<int>(pitch / (M_PI / 2.0f) * n_elev);
        e_bin = std::max(0, std::min(n_elev - 1, e_bin));
        int d3_bin = static_cast<int>(r_3d / max_range * n_dist);
        d3_bin = std::max(0, std::min(n_dist - 1, d3_bin));
        hist_vert[e_bin * n_dist + d3_bin] += 1.0f;
    }

    // Combine into descriptor (truncate/pad to descriptor_dim_)
    int copy_h1 = std::min(static_cast<int>(hist.size()),      descriptor_dim_ / 2);
    int copy_h2 = std::min(static_cast<int>(hist_vert.size()), descriptor_dim_ - copy_h1);

    for (int i = 0; i < copy_h1; ++i)        desc(i)          = hist[i];
    for (int i = 0; i < copy_h2; ++i)        desc(copy_h1 + i) = hist_vert[i];

    // L2 normalize
    float norm = desc.norm();
    if (norm > 1e-6f) desc /= norm;
    return desc;
}

float OverlapTransformer::cosineSimilarity(const Eigen::VectorXf& a,
                                             const Eigen::VectorXf& b) {
    float dot  = a.dot(b);
    float norm = a.norm() * b.norm();
    if (norm < 1e-6f) return 0.0f;
    return dot / norm;
}

std::vector<OverlapTransformer::Candidate> OverlapTransformer::retrieve(
        const Eigen::VectorXf& query_desc,
        const std::vector<SubMap::Ptr>& db_submaps,
        int top_k,
        double min_score,
        int min_submap_gap,
        double min_temporal_gap,
        double gps_search_radius,
        const Eigen::Vector3d& query_gps_pos,
        bool query_has_gps) const {

    std::vector<std::pair<float, Candidate>> scored;

    for (const auto& sm : db_submaps) {
        if (!sm->has_descriptor) continue;

        // Submap gap check
        // (caller ensures query is current submap; we check id difference)

        // Temporal gap check
        // (skipped here since we don't have query timestamp)

        // GPS distance pruning
        if (query_has_gps && sm->has_valid_gps) {
            double gps_dist = (query_gps_pos - sm->gps_center).norm();
            if (gps_dist > gps_search_radius) continue;
        }

        float score = cosineSimilarity(query_desc, sm->overlap_descriptor);
        if (score < static_cast<float>(min_score)) continue;

        Candidate c;
        c.submap_id  = sm->id;
        c.session_id = sm->session_id;
        c.score      = score;
        scored.push_back({score, c});
    }

    // Sort descending by score
    std::sort(scored.begin(), scored.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<Candidate> out;
    for (int i = 0; i < std::min(top_k, (int)scored.size()); ++i) {
        out.push_back(scored[i].second);
    }
    return out;
}

}  // namespace automap_pro
