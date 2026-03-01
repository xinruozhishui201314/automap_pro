#pragma once

#include <memory>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

using FPFHCloud    = pcl::PointCloud<pcl::FPFHSignature33>;
using FPFHCloudPtr = FPFHCloud::Ptr;
using NormalCloud  = pcl::PointCloud<pcl::Normal>;

// ──────────────────────────────────────────────────────────
// FPFHExtractor: computes Fast Point Feature Histograms
// ──────────────────────────────────────────────────────────
class FPFHExtractor {
public:
    FPFHExtractor();
    ~FPFHExtractor() = default;

    FPFHCloudPtr compute(const CloudXYZIPtr& cloud) const;

    NormalCloud::Ptr computeNormals(const CloudXYZIPtr& cloud) const;

    // Build correspondences from two FPFH feature sets
    // Returns pairs of matching point indices (src_idx, tgt_idx)
    std::vector<std::pair<int,int>> findCorrespondences(
        const FPFHCloudPtr& src_feat,
        const FPFHCloudPtr& tgt_feat,
        bool mutual_check = true) const;

private:
    double normal_radius_;
    double feature_radius_;
    int    max_nn_normal_;
    int    max_nn_feature_;
};

}  // namespace automap_pro
