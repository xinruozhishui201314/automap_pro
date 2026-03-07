#pragma once
#include <algorithm>
#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace automap_pro {

class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager inst;
        return inst;
    }

    void load(const std::string& yaml_path);
    void loadFromFile(const std::string& yaml_path) { load(yaml_path); }  // 别名，nodes 使用

    // ── 系统 ──────────────────────────────────────────────
    std::string systemName()    const { return get<std::string>("system.name",    "AutoMap-Pro"); }
    std::string outputDir()     const { return get<std::string>("system.output_dir", "/data/automap_output"); }
    int         numThreads()    const { return get<int>("system.num_threads", 8); }
    /** 传感器数据空闲超过该秒数且队列已空时，触发最终处理并结束建图（需同时开启 auto_finish_on_sensor_idle） */
    double      sensorIdleTimeoutSec() const { return std::max(1.0, get<double>("system.sensor_idle_timeout_sec", 10.0)); }
    /** 是否在传感器空闲超时后自动执行最终 HBA、导出地图并结束建图 */
    bool        autoFinishOnSensorIdle() const { return get<bool>("system.auto_finish_on_sensor_idle", true); }

    // ── 传感器（ROS2 话题默认从配置文件 sensor: 节读取）────────────────────────
    std::string lidarTopic()    const { return get<std::string>("sensor.lidar.topic", "/os1_cloud_node1/points"); }
    std::string imuTopic()      const { return get<std::string>("sensor.imu.topic",   "/imu/imu"); }
    std::string gpsTopic()      const { return get<std::string>("sensor.gps.topic",   "/gps/fix"); }
    bool        gpsEnabled()    const { return get<bool>("sensor.gps.enabled", false); }
    bool        cameraEnabled() const { return get<bool>("sensor.camera.enabled", false); }
    std::string cameraTopic()   const { return get<std::string>("sensor.camera.topic", "/camera/image_raw"); }

    // ── 前端 FAST-LIVO2 ───────────────────────────────────
    std::string fastLivoOdomTopic()  const { return get<std::string>("frontend.odom_topic",  "/aft_mapped_to_init"); }
    std::string fastLivoCloudTopic() const { return get<std::string>("frontend.cloud_topic", "/cloud_registered"); }
    std::string fastLivoKFInfoTopic()const { return get<std::string>("frontend.kf_info_topic", "/fast_livo/keyframe_info"); }
    bool        useComposableNode()  const { return get<bool>("frontend.use_composable_node", true); }
    /** 前端发布的点云坐标系：body=雷达/body 系，需用 T_w_b 变换到世界；world=camera_init/世界系（fast_livo 默认），需先转 body 再建图，否则会双重变换导致全局图错乱 */
    std::string frontendCloudFrame() const { return get<std::string>("frontend.cloud_frame", "world"); }

    // ── 关键帧 ────────────────────────────────────────────
    double kfMinTranslation()   const { return get<double>("keyframe.min_translation", 1.0); }
    double kfMinRotationDeg()   const { return get<double>("keyframe.min_rotation_deg", 10.0); }
    double kfMaxInterval()      const { return get<double>("keyframe.max_interval", 2.0); }
    double kfMaxEsikfCovNorm()  const { return get<double>("keyframe.max_esikf_cov_norm", 1e3); }
    /** P1 修复：是否保留关键帧点云（若 false 则定期删除以节省内存，但 buildGlobalMap 会回退） */
    bool   retainCloudBody()    const { return get<bool>("keyframe.retain_cloud_body", true); }
    /** 是否允许子图归档时删除关键帧点云（不推荐，会导致回退到 merged_cloud） */
    bool   allowCloudArchival() const { return get<bool>("keyframe.allow_cloud_archival", false); }
    /** 关键帧内存上限（MB，诊断参考） */
    int    maxKeyframeMemoryMb()const { return get<int>("keyframe.max_memory_mb", 4096); }

    // ── 子图 ──────────────────────────────────────────────
    int    submapMaxKF()        const { return get<int>("submap.max_keyframes", 100); }
    double submapMaxSpatial()   const { return get<double>("submap.max_spatial_m", 100.0); }
    double submapMaxTemporal()  const { return get<double>("submap.max_temporal_s", 60.0); }
    double submapMatchRes()     const { return std::max(0.2, get<double>("submap.match_resolution", 0.4)); }
    double submapMergeRes()     const { return std::max(0.2, get<double>("submap.merge_resolution", 0.2)); }

    // ── GPS 延迟对齐 ──────────────────────────────────────
    int    gpsAlignMinPoints()  const { return get<int>("gps.align_min_points", 50); }
    double gpsAlignMinDist()    const { return get<double>("gps.align_min_distance_m", 30.0); }
    double gpsQualityThreshold()const { return get<double>("gps.quality_threshold_hdop", 2.0); }
    double gpsAlignRmseThresh() const { return get<double>("gps.align_rmse_threshold_m", 1.5); }
    int    gpsGoodSamplesNeeded()const { return get<int>("gps.good_samples_needed", 30); }
    bool   gpsAddConstraintsOnAlign() const { return get<bool>("gps.add_constraints_on_align", true); }
    double gpsFactorIntervalM() const { return get<double>("gps.factor_interval_m", 5.0); }
    // GPS 质量/滤波（gps_processor / gps_fusion 使用；可选，缺省用默认值）
    double gpsExcellentHDOP()   const { return get<double>("gps.quality_excellent_hdop", 0.8); }
    double gpsHighHDOP()        const { return get<double>("gps.quality_high_hdop", 1.5); }
    double gpsMediumHDOP()      const { return get<double>("gps.quality_medium_hdop", 3.0); }
    double gpsMaxJump()         const { return get<double>("gps.max_jump_m", 10.0); }
    double gpsMaxVelocity()     const { return get<double>("gps.max_velocity_ms", 30.0); }
    double gpsChi2Threshold()   const { return get<double>("gps.chi2_threshold", 7.815); }
    int    gpsConsecutiveValid()const { return get<int>("gps.consecutive_valid_required", 3); }
    bool   gpsJumpDetection()   const { return get<bool>("gps.jump_detection_enabled", true); }
    bool   gpsConsistencyCheck()const { return get<bool>("gps.consistency_check_enabled", true); }
    Eigen::Vector3d gpsCovExcellent() const;
    Eigen::Vector3d gpsCovHigh()     const;
    Eigen::Vector3d gpsCovMedium()   const;
    Eigen::Vector3d gpsCovLow()      const;

    // ── 回环检测 ──────────────────────────────────────────
    double overlapThreshold()   const { return get<double>("loop_closure.overlap_threshold", 0.3); }
    int    loopTopK()           const { return get<int>("loop_closure.top_k", 5); }
    double loopMinTemporalGap() const { return get<double>("loop_closure.min_temporal_gap_s", 30.0); }
    int    loopMinSubmapGap()   const { return get<int>("loop_closure.min_submap_gap", 3); }
    double gpsSearchRadius()    const { return get<double>("loop_closure.gps_search_radius_m", 200.0); }
    int    loopWorkerThreads()  const { return get<int>("loop_closure.worker_threads", 2); }

    // ── OverlapTransformer ────────────────────────────────
    std::string overlapModelPath()   const { return get<std::string>("loop_closure.overlap_transformer.model_path", ""); }
    int    rangeImageH()             const { return get<int>("loop_closure.overlap_transformer.proj_H", 64); }
    int    rangeImageW()             const { return get<int>("loop_closure.overlap_transformer.proj_W", 900); }
    float  fovUp()                   const { return get<float>("loop_closure.overlap_transformer.fov_up",   3.0f); }
    float  fovDown()                 const { return get<float>("loop_closure.overlap_transformer.fov_down", -25.0f); }
    float  maxRange()                const { return get<float>("loop_closure.overlap_transformer.max_range", 50.0f); }
    int    descriptorDim()           const { return get<int>("loop_closure.overlap_transformer.descriptor_dim", 256); }

    // ── TEASER++ ──────────────────────────────────────────
    double teaserNoiseBound()   const { return get<double>("loop_closure.teaser.noise_bound", 0.1); }
    double teaserCbar2()        const { return get<double>("loop_closure.teaser.cbar2", 1.0); }
    double teaserVoxelSize()    const { return std::max(0.2, get<double>("loop_closure.teaser.voxel_size", 0.4)); }
    double teaserMinInlierRatio()const{ return get<double>("loop_closure.teaser.min_inlier_ratio", 0.30); }
    double teaserMaxRMSE()      const { return get<double>("loop_closure.teaser.max_rmse_m", 0.3); }
    bool   teaserICPRefine()    const { return get<bool>("loop_closure.teaser.icp_refine", true); }

    // ── iSAM2 ─────────────────────────────────────────────
    double isam2RelinThresh()   const { return get<double>("backend.isam2.relinearize_threshold", 0.01); }
    int    isam2RelinSkip()     const { return get<int>("backend.isam2.relinearize_skip", 1); }
    bool   isam2EnableRelin()   const { return get<bool>("backend.isam2.enable_relinearization", true); }

    // ── HBA ───────────────────────────────────────────────
    int    hbaTotalLayers()     const { return get<int>("backend.hba.total_layer_num", 3); }
    int    hbaThreadNum()       const { return get<int>("backend.hba.thread_num", 8); }
    int    hbaTriggerSubmaps()  const { return get<int>("backend.hba.trigger_every_n_submaps", 10); }
    bool   hbaOnLoop()          const { return get<bool>("backend.hba.trigger_on_loop", false); }
    bool   hbaOnFinish()        const { return get<bool>("backend.hba.trigger_on_finish", true); }
    std::string hbaDataPath()   const { return get<std::string>("backend.hba.data_path", "/tmp/hba_data"); }

    // ── 地图（东北天 ENU 统一坐标系）──────────────────────────────────────
    double mapVoxelSize()       const { return std::max(0.2, get<double>("map.voxel_size", 0.2)); }
    /** 地图坐标系原点经纬度配置文件路径（.cfg），先创建再写入；后端统一使用该 ENU 原点 */
    std::string mapFrameConfigPath() const {
        std::string p = get<std::string>("map.frame_config_path", "");
        return p.empty() ? (outputDir() + "/map_frame.cfg") : p;
    }
    bool   mapStatisticalFilter()const{ return get<bool>("map.statistical_filter", true); }
    bool   mapStatFilter()        const { return mapStatisticalFilter(); }  // 别名，map_filter 使用
    int    mapStatFilterMeanK() const { return get<int>("map.statistical_filter_mean_k", 50); }
    double mapStatFilterStdMul()const { return get<double>("map.statistical_filter_std_mul", 1.0); }

    // ── 会话 ──────────────────────────────────────────────
    bool   multiSessionEnabled()const { return get<bool>("session.multi_session", false); }
    std::string sessionDir()    const { return get<std::string>("session.session_dir", ""); }
    std::vector<std::string> previousSessionDirs() const;

private:
    YAML::Node cfg_;

    template<typename T>
    T get(const std::string& key, const T& default_val) const {
        try {
            YAML::Node node = cfg_;
            std::istringstream ss(key);
            std::string token;
            while (std::getline(ss, token, '.')) {
                if (!node.IsMap() || !node[token]) return default_val;
                node = node[token];
            }
            return node.as<T>();
        } catch (...) { return default_val; }
    }

    ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
};

} // namespace automap_pro
