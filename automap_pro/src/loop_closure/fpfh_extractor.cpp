#include "automap_pro/loop_closure/fpfh_extractor.h"
#include "automap_pro/core/logger.h"
#include <pcl/search/kdtree.h>
#include <limits>

#define MOD "FPFHExtractor"

namespace automap_pro {

FPFHCloudPtr FpfhExtractor::compute(
    const CloudXYZIPtr& cloud,
    float normal_radius,
    float fpfh_radius) const
{
    try {
        if (!cloud || cloud->empty()) {
            ALOG_DEBUG(MOD, "compute: empty or null cloud");
            return std::make_shared<FPFHCloud>();
        }

        if (normal_radius <= 0 || fpfh_radius <= 0) {
            ALOG_WARN(MOD, "compute: invalid radii (normal={:.3f}, fpfh={:.3f}), using defaults",
                      normal_radius, fpfh_radius);
            normal_radius = std::max(normal_radius, 0.1f);
            fpfh_radius = std::max(fpfh_radius, 0.1f);
        }

        auto feat = std::make_shared<FPFHCloud>();

        // 法向量估计
        auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
        pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        auto tree_n = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        ne.setSearchMethod(tree_n);
        ne.setRadiusSearch(normal_radius);
        ne.setNumberOfThreads(4);
        
        try {
            ne.compute(*normals);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "Normal estimation failed: {}", e.what());
            return feat;
        }

        // FPFH 计算
        pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals);
        auto tree_f = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        fpfh.setSearchMethod(tree_f);
        fpfh.setRadiusSearch(fpfh_radius);
        fpfh.setNumberOfThreads(4);
        
        try {
            fpfh.compute(*feat);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "FPFH computation failed: {}", e.what());
            return feat;
        }

        ALOG_DEBUG(MOD, "FPFH computed: {} features", feat->size());
        return feat;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "compute exception: {}", e.what());
        return std::make_shared<FPFHCloud>();
    } catch (...) {
        ALOG_ERROR(MOD, "compute unknown exception");
        return std::make_shared<FPFHCloud>();
    }
}

static float fpfhDist(const pcl::FPFHSignature33& a, const pcl::FPFHSignature33& b) {
    float d = 0;
    for (int i = 0; i < 33; ++i) {
        float diff = a.histogram[i] - b.histogram[i];
        d += diff * diff;
    }
    return d;
}

std::vector<std::pair<int,int>> FpfhExtractor::findCorrespondences(
    const FPFHCloudPtr& src, const FPFHCloudPtr& tgt, bool mutual) const
{
    try {
        std::vector<std::pair<int,int>> corrs;
        if (!src || !tgt || src->empty() || tgt->empty()) {
            ALOG_DEBUG(MOD, "findCorrespondences: empty clouds");
            return corrs;
        }

        ALOG_DEBUG(MOD, "findCorrespondences: src={} tgt={} mutual={}",
                   src->size(), tgt->size(), mutual);

        // src → tgt 最近邻
        std::vector<int> src2tgt(src->size(), -1);
        std::vector<float> src_dists(src->size(), std::numeric_limits<float>::max());

        for (size_t i = 0; i < src->size(); ++i) {
            float best = std::numeric_limits<float>::max();
            for (size_t j = 0; j < tgt->size(); ++j) {
                float d = fpfhDist(src->points[i], tgt->points[j]);
                if (d < best) {
                    best = d;
                    src2tgt[i] = static_cast<int>(j);
                }
            }
            src_dists[i] = best;
        }

        if (!mutual) {
            for (size_t i = 0; i < src->size(); ++i)
                if (src2tgt[i] >= 0) corrs.push_back({static_cast<int>(i), src2tgt[i]});
            return corrs;
        }

        // tgt → src 最近邻 (双向匹配)
        std::vector<int> tgt2src(tgt->size(), -1);
        std::vector<float> tgt_dists(tgt->size(), std::numeric_limits<float>::max());

        for (size_t j = 0; j < tgt->size(); ++j) {
            float best = std::numeric_limits<float>::max();
            for (size_t i = 0; i < src->size(); ++i) {
                float d = fpfhDist(src->points[i], tgt->points[j]);
                if (d < best) {
                    best = d;
                    tgt2src[j] = static_cast<int>(i);
                }
            }
            tgt_dists[j] = best;
        }

        // 验证互惠性
        for (size_t i = 0; i < src->size(); ++i) {
            int j = src2tgt[i];
            if (j >= 0 && tgt2src[j] == static_cast<int>(i))
                corrs.push_back({static_cast<int>(i), j});
        }

        ALOG_DEBUG(MOD, "findCorrespondences: {} mutual correspondences found", corrs.size());
        return corrs;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "findCorrespondences exception: {}", e.what());
        return {};
    } catch (...) {
        ALOG_ERROR(MOD, "findCorrespondences unknown exception");
        return {};
    }
}

} // namespace automap_pro
