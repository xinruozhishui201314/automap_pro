#pragma once
#include "automap_pro/core/data_types.h"
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <mutex>
#include <memory>

namespace automap_pro {

using FPFHCloud    = pcl::PointCloud<pcl::FPFHSignature33>;
using FPFHCloudPtr = FPFHCloud::Ptr;

/**
 * FPFH特征提取器（线程安全版本）
 * 
 * 修复PCL 1.11-1.12版本中FPFHEstimationOMP的多线程内存泄漏问题：
 * - 使用单线程版本（pcl::FPFHEstimation）替代OMP
 * - 全局互斥锁保护单个Extractor实例
 * - 预生成对象以避免频繁创建销毁触发double-free
 * 
 * 性能：单帧~80-120ms (4000pts) → acceptable for offline/loop closure
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
