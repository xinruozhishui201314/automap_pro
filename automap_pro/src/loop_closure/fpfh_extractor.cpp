#include "automap_pro/loop_closure/fpfh_extractor.h"
#include <pcl/search/kdtree.h>
#include <limits>

namespace automap_pro {

FPFHCloudPtr FpfhExtractor::compute(
    const CloudXYZIPtr& cloud,
    float normal_radius,
    float fpfh_radius) const
{
    auto feat = std::make_shared<FPFHCloud>();
    if (!cloud || cloud->empty()) return feat;

    // 法向量估计（OpenMP 并行）
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    auto tree_n = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    ne.setSearchMethod(tree_n);
    ne.setRadiusSearch(normal_radius);
    ne.setNumberOfThreads(4);
    ne.compute(*normals);

    // FPFH 计算（OpenMP 并行）
    pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    auto tree_f = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    fpfh.setSearchMethod(tree_f);
    fpfh.setRadiusSearch(fpfh_radius);
    fpfh.setNumberOfThreads(4);
    fpfh.compute(*feat);
    return feat;
}

static float fpfhDist(const pcl::FPFHSignature33& a, const pcl::FPFHSignature33& b) {
    float d = 0;
    for (int i = 0; i < 33; ++i) { float diff = a.histogram[i] - b.histogram[i]; d += diff*diff; }
    return d;
}

std::vector<std::pair<int,int>> FpfhExtractor::findCorrespondences(
    const FPFHCloudPtr& src, const FPFHCloudPtr& tgt, bool mutual) const
{
    std::vector<std::pair<int,int>> corrs;
    if (!src || !tgt || src->empty() || tgt->empty()) return corrs;

    // src → tgt 最近邻
    std::vector<int> src2tgt(src->size(), -1);
    for (size_t i = 0; i < src->size(); ++i) {
        float best = std::numeric_limits<float>::max();
        for (size_t j = 0; j < tgt->size(); ++j) {
            float d = fpfhDist(src->points[i], tgt->points[j]);
            if (d < best) { best = d; src2tgt[i] = (int)j; }
        }
    }

    if (!mutual) {
        for (size_t i = 0; i < src->size(); ++i)
            if (src2tgt[i] >= 0) corrs.push_back({(int)i, src2tgt[i]});
        return corrs;
    }

    // ✅ 修复：使用 KD-Tree 加速最近邻搜索（O(n log n)）
    pcl::KdTreeFLANN<pcl::FPFHSignature33> kdtree_tgt;
    kdtree_tgt.setInputCloud(tgt);
    
    std::vector<int> tgt2src(tgt->size(), -1);
    for (size_t j = 0; j < tgt->size(); ++j) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        if (kdtree_tgt.nearestKSearch(tgt->points[j], 1, indices, distances) > 0) {
            tgt2src[j] = indices[0];
        }
    }

    for (size_t i = 0; i < src->size(); ++i) {
        int j = src2tgt[i];
        if (j >= 0 && tgt2src[j] == (int)i) corrs.push_back({(int)i, j});
    }
    return corrs;
}

} // namespace automap_pro
