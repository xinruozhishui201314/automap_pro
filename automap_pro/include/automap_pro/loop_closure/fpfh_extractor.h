#pragma once
/**
 * @file fpfh_extractor.h
 * @brief FPFH（Fast Point Feature Histograms）提取与互最近邻匹配；PCL 单线程路径 + 全局互斥规避 OMP 析构问题。
 *
 * @details
 * 对点 @f$\mathbf{p}@f$，SPFH 统计 @f$r@f$ 邻域内相对几何直方图，FPFH 为邻域 SPFH 加权平均（Rusu et al.）；
 * 输出 33 维直方图用于粗对应检索。
 */
#include "automap_pro/core/data_types.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <mutex>
#include <memory>

namespace automap_pro {

using FPFHCloud    = pcl::PointCloud<pcl::FPFHSignature33>;
using FPFHCloudPtr = FPFHCloud::Ptr;

/**
 * @class FpfhExtractor
 * @brief FPFH 特征提取器（进程内串行化保证线程安全）。
 *
 * @details
 * PCL 1.11–1.12 FPFHEstimationOMP 存在多线程泄漏；此处用单线程 FPFHEstimation + 静态互斥。
 * 性能量级约单帧 80–120 ms（~4k 点），适用于离线/回环。
 */
class FpfhExtractor {
public:
    FpfhExtractor();
    ~FpfhExtractor() = default;

    FPFHCloudPtr compute(const CloudXYZIPtr& cloud,
                         float normal_radius = 0.5f,
                         float fpfh_radius   = 1.0f) const;

    std::vector<std::pair<int,int>> findCorrespondences(
        const FPFHCloudPtr& src_feat,
        const FPFHCloudPtr& tgt_feat,
        bool mutual = true) const;

private:
    // 全局锁保护PCL对象（PCL的OMP版本线程不安全，单线程版本也有析构缺陷）
    static std::mutex compute_mutex_;
};

} // namespace automap_pro
