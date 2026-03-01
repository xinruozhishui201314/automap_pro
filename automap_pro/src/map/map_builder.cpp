#include "automap_pro/map/map_builder.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

MapBuilder::MapBuilder() {
    voxel_size_   = ConfigManager::instance().mapVoxelSize();
    global_cloud_ = std::make_shared<CloudXYZI>();
}

void MapBuilder::reprojectAllSubmaps(const std::vector<SubMap::Ptr>& submaps,
                                      const std::map<int, Pose3d>& optimized_poses) {
    for (const auto& sm : submaps) {
        auto it = optimized_poses.find(sm->id);
        if (it != optimized_poses.end()) {
            sm->updateAnchorPose(it->second);
        }
        sm->reproject();
    }
}

CloudXYZIPtr MapBuilder::buildGlobalMap(const std::vector<SubMap::Ptr>& submaps) {
    std::lock_guard<std::mutex> lk(mutex_);
    global_cloud_ = std::make_shared<CloudXYZI>();

    for (const auto& sm : submaps) {
        if (sm->merged_cloud && !sm->merged_cloud->empty()) {
            *global_cloud_ += *sm->merged_cloud;
        }
    }

    // Voxel downsample
    global_cloud_ = utils::voxelDownsample(global_cloud_, voxel_size_);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[MapBuilder] Global map: %zu points (voxel=%.2f m)",
             global_cloud_->size(), voxel_size_);
    return global_cloud_;
}

void MapBuilder::updateSubmaps(const std::vector<SubMap::Ptr>& changed_submaps) {
    // For incremental updates, rebuild global map from scratch is simplest approach
    // In production: track which tiles are affected and rebuild only those
    for (const auto& sm : changed_submaps) {
        sm->reproject();
    }
}

CloudXYZIPtr MapBuilder::globalCloud() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return global_cloud_;
}

size_t MapBuilder::numPoints() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return global_cloud_ ? global_cloud_->size() : 0;
}

void MapBuilder::clear() {
    std::lock_guard<std::mutex> lk(mutex_);
    global_cloud_ = std::make_shared<CloudXYZI>();
}

}  // namespace automap_pro
