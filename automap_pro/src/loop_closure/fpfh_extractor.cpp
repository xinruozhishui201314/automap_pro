#include "automap_pro/loop_closure/fpfh_extractor.h"
#include "automap_pro/core/config_manager.h"

#include <pcl/search/kdtree.h>

namespace automap_pro {

FPFHExtractor::FPFHExtractor() {
    const auto& cfg = ConfigManager::instance();
    normal_radius_  = cfg.fpfhNormalRadius();
    feature_radius_ = cfg.fpfhFeatureRadius();
    max_nn_normal_  = cfg.fpfhMaxNNNormal();
    max_nn_feature_ = cfg.fpfhMaxNNFeature();
}

NormalCloud::Ptr FPFHExtractor::computeNormals(const CloudXYZIPtr& cloud) const {
    auto normals = std::make_shared<NormalCloud>();
    pcl::NormalEstimation<PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    auto tree = std::make_shared<pcl::search::KdTree<PointXYZI>>();
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(normal_radius_);
    ne.setKSearch(max_nn_normal_);
    ne.compute(*normals);
    return normals;
}

FPFHCloudPtr FPFHExtractor::compute(const CloudXYZIPtr& cloud) const {
    auto normals = computeNormals(cloud);

    pcl::FPFHEstimation<PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    auto tree = std::make_shared<pcl::search::KdTree<PointXYZI>>();
    fpfh.setSearchMethod(tree);
    fpfh.setRadiusSearch(feature_radius_);
    fpfh.setKSearch(max_nn_feature_);

    auto features = std::make_shared<FPFHCloud>();
    fpfh.compute(*features);
    return features;
}

std::vector<std::pair<int,int>> FPFHExtractor::findCorrespondences(
        const FPFHCloudPtr& src_feat,
        const FPFHCloudPtr& tgt_feat,
        bool mutual_check) const {
    if (!src_feat || !tgt_feat) return {};

    int ns = static_cast<int>(src_feat->size());
    int nt = static_cast<int>(tgt_feat->size());

    auto l2 = [](const pcl::FPFHSignature33& a,
                  const pcl::FPFHSignature33& b) {
        float d = 0.0f;
        for (int i = 0; i < 33; ++i) d += (a.histogram[i] - b.histogram[i]) *
                                           (a.histogram[i] - b.histogram[i]);
        return d;
    };

    // Forward: for each src, find best match in tgt
    std::vector<int> fwd(ns, -1);
    for (int i = 0; i < ns; ++i) {
        float best = std::numeric_limits<float>::max();
        for (int j = 0; j < nt; ++j) {
            float d = l2(src_feat->points[i], tgt_feat->points[j]);
            if (d < best) { best = d; fwd[i] = j; }
        }
    }

    if (!mutual_check) {
        std::vector<std::pair<int,int>> out;
        for (int i = 0; i < ns; ++i) {
            if (fwd[i] >= 0) out.push_back({i, fwd[i]});
        }
        return out;
    }

    // Backward: for each tgt, find best match in src
    std::vector<int> bwd(nt, -1);
    for (int j = 0; j < nt; ++j) {
        float best = std::numeric_limits<float>::max();
        for (int i = 0; i < ns; ++i) {
            float d = l2(tgt_feat->points[j], src_feat->points[i]);
            if (d < best) { best = d; bwd[j] = i; }
        }
    }

    // Mutual matches
    std::vector<std::pair<int,int>> out;
    for (int i = 0; i < ns; ++i) {
        int j = fwd[i];
        if (j >= 0 && bwd[j] == i) out.push_back({i, j});
    }
    return out;
}

}  // namespace automap_pro
