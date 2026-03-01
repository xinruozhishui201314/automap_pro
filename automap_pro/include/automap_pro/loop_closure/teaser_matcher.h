#pragma once

#include <optional>
#include <memory>

#include "automap_pro/core/data_types.h"
#include "automap_pro/loop_closure/fpfh_extractor.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// TeaserMatcher: FPFH + TEASER++ point cloud registration
// Falls back to RANSAC-based SVD when TEASER++ library
// is not available.
// ──────────────────────────────────────────────────────────
class TeaserMatcher {
public:
    struct Result {
        Pose3d T_tgt_src;           // Transform from source to target
        double inlier_ratio  = 0.0;
        double rmse          = 1e9;
        bool   success       = false;
        int    num_inliers   = 0;
        int    num_correspondences = 0;
    };

    TeaserMatcher();
    ~TeaserMatcher() = default;

    Result match(const CloudXYZIPtr& src_cloud,
                 const CloudXYZIPtr& tgt_cloud,
                 const Pose3d& initial_guess = Pose3d::Identity()) const;

private:
    // Preprocessing: voxel downsample + crop to max_points
    CloudXYZIPtr preprocess(const CloudXYZIPtr& cloud) const;

    // RANSAC-based SVD fallback (when TEASER++ not available)
    Result matchRANSAC(
        const CloudXYZIPtr& src, const CloudXYZIPtr& tgt,
        const std::vector<std::pair<int,int>>& correspondences) const;

    // Compute RMSE after applying transform
    double computeRMSE(const CloudXYZIPtr& src,
                        const CloudXYZIPtr& tgt,
                        const Pose3d& T) const;

    FPFHExtractor fpfh_extractor_;

    double voxel_size_;
    double noise_bound_;
    double min_inlier_ratio_;
    double max_rmse_;
    int    max_points_;

    // RANSAC params (fallback)
    int    ransac_max_iter_     = 50000;
    double ransac_inlier_dist_  = 0.3;
};

}  // namespace automap_pro
