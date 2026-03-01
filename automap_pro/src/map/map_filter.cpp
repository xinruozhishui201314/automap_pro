#include "automap_pro/map/map_filter.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

namespace automap_pro {

MapFilter::MapFilter() {
    const auto& cfg = ConfigManager::instance();
    voxel_size_          = cfg.mapVoxelSize();
    stat_filter_enabled_ = cfg.mapStatFilter();
    stat_mean_k_         = cfg.mapStatFilterMeanK();
    stat_std_mul_        = cfg.mapStatFilterStdMul();
}

CloudXYZIPtr MapFilter::voxelFilter(const CloudXYZIPtr& cloud, double vs) const {
    double v = (vs > 0.0) ? vs : voxel_size_;
    return utils::voxelDownsample(cloud, v);
}

CloudXYZIPtr MapFilter::statisticalOutlierFilter(const CloudXYZIPtr& cloud,
                                                    int mk, double sm) const {
    int    mean_k  = (mk  > 0)    ? mk  : stat_mean_k_;
    double std_mul = (sm  > 0.0)  ? sm  : stat_std_mul_;
    return utils::statisticalOutlierRemoval(cloud, mean_k, std_mul);
}

CloudXYZIPtr MapFilter::applyAll(const CloudXYZIPtr& cloud) const {
    auto out = voxelFilter(cloud);
    if (stat_filter_enabled_) {
        out = statisticalOutlierFilter(out);
    }
    return out;
}

}  // namespace automap_pro
