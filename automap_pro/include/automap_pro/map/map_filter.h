#pragma once
/**
 * @file map/map_filter.h
 * @brief 全局地图：增量构建、体素/KD、导出与滤波。
 */


#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// MapFilter: voxel downsample, statistical outlier removal,
// ground/outlier removal
// ──────────────────────────────────────────────────────────
class MapFilter {
public:
    MapFilter();
    ~MapFilter() = default;

    CloudXYZIPtr voxelFilter(const CloudXYZIPtr& cloud,
                              double voxel_size = 0.0) const;

    CloudXYZIPtr statisticalOutlierFilter(const CloudXYZIPtr& cloud,
                                           int mean_k = 0,
                                           double std_mul = 0.0) const;

    CloudXYZIPtr applyAll(const CloudXYZIPtr& cloud) const;

private:
    double voxel_size_;
    bool   stat_filter_enabled_;
    int    stat_mean_k_;
    double stat_std_mul_;
};

}  // namespace automap_pro
