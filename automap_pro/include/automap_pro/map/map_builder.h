#pragma once

#include <vector>
#include <mutex>
#include <functional>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// MapBuilder: reprojects submap clouds after pose updates,
// merges into global map, manages incremental updates
// ──────────────────────────────────────────────────────────
class MapBuilder {
public:
    MapBuilder();
    ~MapBuilder() = default;

    // Reproject all submaps using optimized poses
    void reprojectAllSubmaps(const std::vector<SubMap::Ptr>& submaps,
                              const std::map<int, Pose3d>& optimized_poses);

    // Merge all submap clouds into global cloud
    CloudXYZIPtr buildGlobalMap(const std::vector<SubMap::Ptr>& submaps);

    // Incremental update for changed submaps
    void updateSubmaps(const std::vector<SubMap::Ptr>& changed_submaps);

    // Getters
    CloudXYZIPtr globalCloud() const;
    size_t numPoints() const;

    void clear();

private:
    CloudXYZIPtr global_cloud_;
    mutable std::mutex mutex_;
    double voxel_size_;
};

}  // namespace automap_pro
