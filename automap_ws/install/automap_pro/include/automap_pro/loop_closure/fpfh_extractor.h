#pragma once
#include "automap_pro/core/data_types.h"
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>

namespace automap_pro {

using FPFHCloud    = pcl::PointCloud<pcl::FPFHSignature33>;
using FPFHCloudPtr = FPFHCloud::Ptr;

class FpfhExtractor {
public:
    FPFHCloudPtr compute(const CloudXYZIPtr& cloud,
                         float normal_radius = 0.5f,
                         float fpfh_radius   = 1.0f) const;

    std::vector<std::pair<int,int>> findCorrespondences(
        const FPFHCloudPtr& src_feat,
        const FPFHCloudPtr& tgt_feat,
        bool mutual = true) const;
};

} // namespace automap_pro
