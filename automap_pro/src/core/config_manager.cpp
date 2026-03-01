#include "automap_pro/core/config_manager.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>

namespace automap_pro {

ConfigManager& ConfigManager::instance() {
    static ConfigManager inst;
    return inst;
}

bool ConfigManager::loadFromFile(const std::string& path) {
    std::lock_guard<std::mutex> lk(mutex_);
    try {
        root_ = YAML::LoadFile(path);
        RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[ConfigManager] Loaded config: %s", path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_pro"), "[ConfigManager] Failed to load %s: %s", path.c_str(), e.what());
        return false;
    }
}

template<typename T>
T ConfigManager::get(const std::string& path, const T& default_val) const {
    std::lock_guard<std::mutex> lk(mutex_);
    try {
        YAML::Node node = root_;
        std::istringstream ss(path);
        std::string token;
        while (std::getline(ss, token, '.')) {
            if (!node.IsMap() || !node[token]) return default_val;
            node = node[token];
        }
        return node.as<T>();
    } catch (...) {
        return default_val;
    }
}

YAML::Node ConfigManager::getNode(const std::string& key) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return root_[key];
}

// ── System ──────────────────────────────────────────────
std::string ConfigManager::systemName()   const { return get<std::string>("system.name",      "AutoMap-Pro"); }
std::string ConfigManager::mode()         const { return get<std::string>("system.mode",      "online"); }
std::string ConfigManager::logLevel()     const { return get<std::string>("system.log_level", "info"); }
std::string ConfigManager::outputDir()    const { return get<std::string>("system.output_dir", "/tmp/automap_output"); }
int         ConfigManager::numThreads()   const { return get<int>("system.num_threads", 8); }
bool        ConfigManager::useGPU()       const { return get<bool>("system.use_gpu",    true); }
int         ConfigManager::gpuDeviceId()  const { return get<int>("system.gpu_device_id", 0); }

// ── Sensor ──────────────────────────────────────────────
std::string ConfigManager::lidarTopic()    const { return get<std::string>("sensor.lidar.topic",     "/livox/lidar"); }
double      ConfigManager::lidarFrequency() const { return get<double>("sensor.lidar.frequency",    10.0); }
double      ConfigManager::lidarMaxRange()  const { return get<double>("sensor.lidar.max_range",   150.0); }
double      ConfigManager::lidarMinRange()  const { return get<double>("sensor.lidar.min_range",     0.5); }
bool        ConfigManager::lidarUndistort() const { return get<bool>("sensor.lidar.undistort",      true); }

std::string ConfigManager::imuTopic()      const { return get<std::string>("sensor.imu.topic",   "/livox/imu"); }
double      ConfigManager::imuFrequency()  const { return get<double>("sensor.imu.frequency",    200.0); }
double      ConfigManager::imuGravity()    const { return get<double>("sensor.imu.gravity",        9.81); }

bool        ConfigManager::cameraEnabled() const { return get<bool>("sensor.camera.enabled",      false); }
std::string ConfigManager::cameraTopic()   const { return get<std::string>("sensor.camera.topic", "/camera/image_raw"); }

bool        ConfigManager::gpsEnabled()    const { return get<bool>("sensor.gps.enabled",         true); }
std::string ConfigManager::gpsTopic()      const { return get<std::string>("sensor.gps.topic",    "/gps/fix"); }
bool        ConfigManager::gpsAutoInitENU() const { return get<bool>("sensor.gps.enu_origin.auto_init", true); }

// ── GPS Fusion ──────────────────────────────────────────
double ConfigManager::gpsExcellentHDOP() const { return get<double>("gps_fusion.quality_thresholds.excellent_hdop", 1.0); }
double ConfigManager::gpsHighHDOP()      const { return get<double>("gps_fusion.quality_thresholds.high_hdop",      2.0); }
double ConfigManager::gpsMediumHDOP()    const { return get<double>("gps_fusion.quality_thresholds.medium_hdop",    5.0); }

Eigen::Vector3d ConfigManager::gpsCovExcellent() const {
    auto v = get<std::vector<double>>("gps_fusion.covariance.excellent", {0.05, 0.05, 0.10});
    return Eigen::Vector3d(v[0], v[1], v[2]);
}
Eigen::Vector3d ConfigManager::gpsCovHigh() const {
    auto v = get<std::vector<double>>("gps_fusion.covariance.high", {0.10, 0.10, 0.20});
    return Eigen::Vector3d(v[0], v[1], v[2]);
}
Eigen::Vector3d ConfigManager::gpsCovMedium() const {
    auto v = get<std::vector<double>>("gps_fusion.covariance.medium", {1.0, 1.0, 2.0});
    return Eigen::Vector3d(v[0], v[1], v[2]);
}
Eigen::Vector3d ConfigManager::gpsCovLow() const {
    auto v = get<std::vector<double>>("gps_fusion.covariance.low", {10.0, 10.0, 20.0});
    return Eigen::Vector3d(v[0], v[1], v[2]);
}

bool   ConfigManager::gpsConsistencyCheck()  const { return get<bool>("gps_fusion.consistency_check.enabled",      true); }
double ConfigManager::gpsChi2Threshold()     const { return get<double>("gps_fusion.consistency_check.chi2_threshold", 11.345); }
double ConfigManager::gpsMaxVelocity()       const { return get<double>("gps_fusion.consistency_check.max_velocity",   30.0); }
bool   ConfigManager::gpsJumpDetection()     const { return get<bool>("gps_fusion.jump_detection.enabled",          true); }
double ConfigManager::gpsMaxJump()           const { return get<double>("gps_fusion.jump_detection.max_jump",          5.0); }
int    ConfigManager::gpsConsecutiveValid()  const { return get<int>("gps_fusion.jump_detection.consecutive_valid",     5); }

// ── Frontend ────────────────────────────────────────────
std::string ConfigManager::frontendMode()            const { return get<std::string>("frontend.mode", "internal"); }
std::string ConfigManager::externalFastLivoOdomTopic()  const { return get<std::string>("frontend.external_fast_livo.odom_topic", "/aft_mapped_to_init"); }
std::string ConfigManager::externalFastLivoCloudTopic() const { return get<std::string>("frontend.external_fast_livo.cloud_topic", "/cloud_registered"); }
std::string ConfigManager::fastLivo2Config()         const { return get<std::string>("frontend.fast_livo2.config_file", "config/fast_livo2_config.yaml"); }
double ConfigManager::kfMinTranslation()             const { return get<double>("frontend.fast_livo2.keyframe_policy.min_translation",  1.0); }
double ConfigManager::kfMinRotationDeg()             const { return get<double>("frontend.fast_livo2.keyframe_policy.min_rotation",     10.0); }
double ConfigManager::kfMaxInterval()                const { return get<double>("frontend.fast_livo2.keyframe_policy.max_interval",      2.0); }
double ConfigManager::cloudDownsampleResolution()    const { return get<double>("frontend.fast_livo2.cloud_downsample_resolution",       0.2); }

// ── Submap ──────────────────────────────────────────────
std::string ConfigManager::msMappingConfig()         const { return get<std::string>("submap.ms_mapping.config_file", "config/ms_mapping_config.yaml"); }
int    ConfigManager::submapMaxKeyframes()           const { return get<int>("submap.split_policy.max_keyframes",           100); }
double ConfigManager::submapMaxSpatialExtent()       const { return get<double>("submap.split_policy.max_spatial_extent",  100.0); }
double ConfigManager::submapMaxTemporalExtent()      const { return get<double>("submap.split_policy.max_temporal_extent",  60.0); }
double ConfigManager::submapMatchResolution()        const { return get<double>("submap.cloud_for_matching_resolution",      0.5); }

// ── Loop Closure ────────────────────────────────────────
std::string ConfigManager::overlapTransformerMode() const { return get<std::string>("loop_closure.overlap_transformer.mode", "internal"); }
std::string ConfigManager::overlapDescriptorServiceName() const { return get<std::string>("loop_closure.overlap_transformer.descriptor_service", "/automap/compute_descriptor"); }
std::string ConfigManager::overlapModelPath() const { return get<std::string>("loop_closure.overlap_transformer.model_path", "models/overlap_transformer/pretrained.pth"); }
int    ConfigManager::rangeImageHeight()      const { return get<int>("loop_closure.overlap_transformer.range_image.height",  64); }
int    ConfigManager::rangeImageWidth()       const { return get<int>("loop_closure.overlap_transformer.range_image.width",  900); }
int    ConfigManager::descriptorDim()         const { return get<int>("loop_closure.overlap_transformer.descriptor_dim",     256); }
int    ConfigManager::loopTopK()              const { return get<int>("loop_closure.overlap_transformer.top_k",               5); }
double ConfigManager::overlapThreshold()      const { return get<double>("loop_closure.overlap_transformer.overlap_threshold",  0.3); }
double ConfigManager::loopMinTemporalGap()    const { return get<double>("loop_closure.overlap_transformer.min_temporal_gap",  30.0); }
int    ConfigManager::loopMinSubmapGap()      const { return get<int>("loop_closure.overlap_transformer.min_submap_gap",         3); }
double ConfigManager::gpsSearchRadius()       const { return get<double>("loop_closure.overlap_transformer.gps_search_radius", 200.0); }

double ConfigManager::teaserVoxelSize()       const { return get<double>("loop_closure.teaser.voxel_size",                0.5); }
double ConfigManager::fpfhNormalRadius()      const { return get<double>("loop_closure.teaser.fpfh.normal_radius",        1.0); }
double ConfigManager::fpfhFeatureRadius()     const { return get<double>("loop_closure.teaser.fpfh.feature_radius",       2.5); }
int    ConfigManager::fpfhMaxNNNormal()       const { return get<int>("loop_closure.teaser.fpfh.max_nn_normal",           30); }
int    ConfigManager::fpfhMaxNNFeature()      const { return get<int>("loop_closure.teaser.fpfh.max_nn_feature",         100); }
double ConfigManager::teaserNoiseBound()      const { return get<double>("loop_closure.teaser.solver.noise_bound",        0.1); }
double ConfigManager::teaserCbar2()           const { return get<double>("loop_closure.teaser.solver.cbar2",              1.0); }
double ConfigManager::teaserRotGNCFactor()    const { return get<double>("loop_closure.teaser.solver.rotation_gnc_factor", 1.4); }
int    ConfigManager::teaserRotMaxIter()      const { return get<int>("loop_closure.teaser.solver.rotation_max_iterations", 100); }
double ConfigManager::teaserRotCostThresh()   const { return get<double>("loop_closure.teaser.solver.rotation_cost_threshold", 1e-6); }
double ConfigManager::teaserMinInlierRatio()  const { return get<double>("loop_closure.teaser.validation.min_inlier_ratio",  0.30); }
double ConfigManager::teaserMaxRMSE()         const { return get<double>("loop_closure.teaser.validation.max_rmse",           0.3); }
double ConfigManager::teaserMinFitness()      const { return get<double>("loop_closure.teaser.validation.min_fitness",        0.5); }
bool   ConfigManager::teaserICPRefine()       const { return get<bool>("loop_closure.teaser.icp_refine.enabled",            true); }
int    ConfigManager::icpMaxIterations()      const { return get<int>("loop_closure.teaser.icp_refine.max_iterations",       30); }
double ConfigManager::icpMaxCorrDist()        const { return get<double>("loop_closure.teaser.icp_refine.max_correspondence_distance", 1.0); }
double ConfigManager::icpConvThresh()         const { return get<double>("loop_closure.teaser.icp_refine.convergence_threshold",     1e-6); }

// ── Backend (HBA) ───────────────────────────────────────
std::string ConfigManager::hbaConfig()             const { return get<std::string>("backend.hba.config_file", "config/hba_config.yaml"); }
bool   ConfigManager::hbaTriggerOnLoop()           const { return get<bool>("backend.hba.trigger_policy.on_loop",         true); }
int    ConfigManager::hbaTriggerPeriodicSubmaps()  const { return get<int>("backend.hba.trigger_policy.periodic_submaps",  10); }
bool   ConfigManager::hbaTriggerOnFinish()         const { return get<bool>("backend.hba.trigger_policy.on_finish",       true); }
int    ConfigManager::hbaMaxIterations()           const { return get<int>("backend.hba.optimization.max_iterations",     100); }
double ConfigManager::hbaConvergenceThreshold()    const { return get<double>("backend.hba.optimization.convergence_threshold", 1e-4); }
bool   ConfigManager::hbaRobustKernel()            const { return get<bool>("backend.hba.optimization.use_robust_kernel",  true); }
double ConfigManager::hbaRobustKernelDelta()       const { return get<double>("backend.hba.optimization.robust_kernel_delta", 1.0); }
std::string ConfigManager::hbaBridgeExportPath()   const { return get<std::string>("backend.hba_bridge.export_path", "/data/automap_output/hba_export"); }
bool   ConfigManager::hbaBridgeRunAfterExport()    const { return get<bool>("backend.hba_bridge.run_after_export", false); }
std::string ConfigManager::hbaBridgeRunCommandTemplate() const { return get<std::string>("backend.hba_bridge.run_command", "ros2 run hba hba --ros-args -p data_path:=%s"); }

// ── Map Output ──────────────────────────────────────────
double ConfigManager::mapVoxelSize()          const { return get<double>("map_output.global_map.voxel_size",                     0.1); }
bool   ConfigManager::mapStatFilter()         const { return get<bool>("map_output.global_map.statistical_filter.enabled",      true); }
int    ConfigManager::mapStatFilterMeanK()    const { return get<int>("map_output.global_map.statistical_filter.mean_k",         50); }
double ConfigManager::mapStatFilterStdMul()   const { return get<double>("map_output.global_map.statistical_filter.std_dev_mul", 2.0); }
bool   ConfigManager::mapTilingEnabled()      const { return get<bool>("map_output.tiling.enabled",                             true); }
double ConfigManager::mapTileSize()           const { return get<double>("map_output.tiling.tile_size",                        100.0); }
bool   ConfigManager::saveFormatPCD()         const { return get<bool>("map_output.formats.pcd",                               true); }
bool   ConfigManager::saveFormatPLY()         const { return get<bool>("map_output.formats.ply",                               true); }
bool   ConfigManager::saveFormatLAS()         const { return get<bool>("map_output.formats.las",                              false); }

// ── Visualization ───────────────────────────────────────
double ConfigManager::visPublishRate()         const { return get<double>("visualization.publish_rate",        1.0); }
bool   ConfigManager::visShowLoopClosures()    const { return get<bool>("visualization.show_loop_closures",   true); }
bool   ConfigManager::visShowGPSTrajectory()   const { return get<bool>("visualization.show_gps_trajectory", true); }
bool   ConfigManager::visShowSubmapBoundaries() const { return get<bool>("visualization.show_submap_boundaries", true); }
double ConfigManager::visGlobalMapDownsample() const { return get<double>("visualization.global_map_downsample", 0.5); }

}  // namespace automap_pro
