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
    /** 传感器数据空闲超过该秒数且队列已空时，触发最终处理并结束建图（需同时开启 auto_finish_on_sensor_idle）；离线/在线由 mode 段区分，load() 时写入 */
    double      sensorIdleTimeoutSec() const { return std::max(1.0, sensor_idle_timeout_sec_); }
    /** 是否在传感器空闲超时后自动执行最终 HBA、导出地图并结束建图 */
    bool        autoFinishOnSensorIdle() const { return get<bool>("system.auto_finish_on_sensor_idle", true); }
    /** 离线模式“播完再结束”：为 true 时不依赖传感器空闲自动结束，仅由 /automap/finish_mapping 服务触发（launch 在 bag 播完后调用） */
    bool        offlineFinishAfterBag() const { return get<bool>("system.offline_finish_after_bag", false); }
    /** 后端帧队列最大长度；计算跟不上时可适当增大，允许“算慢一点”缓冲更多帧（内存占用增加） */
    size_t      frameQueueMaxSize() const {
        int v = get<int>("system.frame_queue_max_size", 500);
        if (v < 100) return 100;
        if (v > 10000) return 10000;
        return static_cast<size_t>(v);
    }
    /** 入口缓冲队列长度；回调只写入此队列后快速返回，背压在 feeder 线程，避免阻塞 Executor；默认 16 */
    size_t      ingressQueueMaxSize() const {
        int v = get<int>("system.ingress_queue_max_size", 16);
        if (v < 2) return 2;
        if (v > 256) return 256;
        return static_cast<size_t>(v);
    }
    /** ROS2 订阅 depth：里程计/点云/KF 话题 KeepLast，过大会占内存；资源受限时可减小。默认 1000 */
    int subscriptionOdomCloudDepth() const {
        int v = get<int>("system.subscription_odom_cloud_depth", 1000);
        return std::max(100, std::min(10000, v));
    }
    /** ROS2 订阅 depth：GPS 话题 KeepLast。默认 1000 */
    int subscriptionGpsDepth() const {
        int v = get<int>("system.subscription_gps_depth", 1000);
        return std::max(100, std::min(20000, v));
    }
    /** 背压：frame_queue 满时每次等待秒数，资源受限时可缩短以更快丢帧保活。默认 60 */
    int backpressureWaitSec() const {
        int v = get<int>("system.backpressure_wait_sec", 60);
        return std::max(1, std::min(120, v));
    }
    /** 背压：最多等待次数，超限后强制丢弃最旧帧。默认 3（即 60*3=180s 后丢帧） */
    int backpressureMaxWaits() const {
        int v = get<int>("system.backpressure_max_waits", 3);
        return std::max(1, std::min(10, v));
    }

    // ── 传感器（ROS2 话题默认从配置文件 sensor: 节读取）────────────────────────
    std::string lidarTopic()    const { return get<std::string>("sensor.lidar.topic", "/os1_cloud_node1/points"); }
    std::string imuTopic()      const { return get<std::string>("sensor.imu.topic",   "/imu/imu"); }
    /** 从 sensor.gps.topic 读取；若 get() 因键路径问题返回默认值，则从 YAML 直接读取兜底（避免 LivoBridge 误用 /gps/fix） */
    std::string gpsTopic() const;
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
    /** 预处理雷达类型：1=Livox Avia 2=Velodyne 3=Ouster 4=RealSense L515，用于元数据导出 */
    int    keyframePreprocessLidarType() const { return get<int>("keyframe.preprocess.lidar_type", 0); }

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
    /** 关键帧匹配 GPS 的最大时间差(秒)。1Hz GPS 时建议 0.5~1.0，原 0.1 易导致 has_gps=0。默认 0.5 */
    double gpsKeyframeMatchWindowS() const { return std::max(0.1, get<double>("gps.keyframe_match_window_s", 0.5)); }
    /** 关键帧绑定 GPS 时允许的最大 HDOP（放宽 M2DGR 等弱 GPS 场景）。默认 12.0 */
    double gpsKeyframeMaxHdop() const {
        double v = get<double>("gps.keyframe_max_hdop", 12.0);
        return std::max(0.1, std::min(99.0, v));
    }
    /** 双样本线性插值时允许的最大时间间隔(秒)，超过则退化为最近邻。默认 2.0 */
    double gpsMaxInterpGapS() const { return std::max(0.2, get<double>("gps.max_interp_gap_s", 2.0)); }
    /** 外推边界(秒)：ts 在 GPS 窗口边缘 ± margin 内时启用速度外推。默认 0.5 */
    double gpsExtrapolationMarginS() const { return std::max(0.1, get<double>("gps.extrapolation_margin_s", 0.5)); }
    /** 外推协方差放大倍数。默认 2.0 */
    double gpsExtrapolationUncertaintyScale() const { return std::max(1.0, get<double>("gps.extrapolation_uncertainty_scale", 2.0)); }
    /** 速度估计窗口(秒)。默认 2.0 */
    double gpsVelocityEstimationWindowS() const { return std::max(0.5, get<double>("gps.velocity_estimation_window_s", 2.0)); }
    /** GPS 滑动窗口最大条数（gps_window_/kf_window_），超限 FIFO 淘汰，防长时间运行 OOM。默认 5000 */
    size_t gpsMaxWindowSize() const {
        int v = get<int>("gps.max_window_size", 5000);
        return static_cast<size_t>(std::max(1000, std::min(50000, v)));
    }
    /** GPS 因子权重：>1 加强约束（协方差缩小），<1 减弱约束；默认 1.0 */
    double gpsFactorWeight() const { return std::max(0.01, get<double>("gps.factor_weight", 1.0)); }
    /** 按质量自动缩放：EXCELLENT/HIGH/MEDIUM 对应因子强度倍数（>1 则该质量约束更强），与 factor_weight 相乘 */
    double gpsFactorQualityScaleExcellent() const { return std::max(0.1, get<double>("gps.factor_quality_scale_excellent", 2.0)); }
    double gpsFactorQualityScaleHigh()      const { return std::max(0.1, get<double>("gps.factor_quality_scale_high", 1.0)); }
    double gpsFactorQualityScaleMedium()    const { return std::max(0.1, get<double>("gps.factor_quality_scale_medium", 0.5)); }
    double gpsFactorQualityScaleLow()       const { return std::max(0.1, get<double>("gps.factor_quality_scale_low", 0.25)); }
    // ── GPS 延迟补偿（对齐后历史帧/回环帧补偿）────────────────────────────────────
    bool   gpsDelayedCompensationEnabled() const { return get<bool>("gps.delayed_compensation.enabled", true); }
    bool   gpsCompensateOnLoop()           const { return get<bool>("gps.delayed_compensation.compensate_on_loop", true); }
    bool   gpsCompensateOnAlign()          const { return get<bool>("gps.delayed_compensation.compensate_on_align", true); }
    int    gpsCompensationBatchSize()      const { return std::max(1, std::min(100, get<int>("gps.delayed_compensation.batch_size", 10))); }
    int    gpsMaxPendingSubmaps()          const { return std::max(100, std::min(10000, get<int>("gps.delayed_compensation.max_pending_submaps", 1000))); }
    /** 补偿时 GPS 协方差基础标准差（米），水平/垂直 */
    double gpsBaseSigmaH()                 const { return std::max(0.01, get<double>("gps.delayed_compensation.base_sigma_h", 0.5)); }
    double gpsBaseSigmaV()                 const { return std::max(0.01, get<double>("gps.delayed_compensation.base_sigma_v", 1.0)); }
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
    /** IMU 重力加速度 (m/s²)，用于预积分与姿态估计。默认 9.81 */
    double imuGravity() const {
        double v = get<double>("fast_livo.imu.gravity", 9.81);
        if (!std::isfinite(v) || v <= 0.0) return 9.81;
        return std::max(8.0, std::min(11.0, v));
    }
    // ── GPS 姿态（是否使用接收机姿态）────────────────────────────────────────
    /** 为 true 时：若通过 addGPSAttitude 注入了接收机姿态（双天线/INS），则直接使用；为 false 时：始终使用估计姿态（IMU pitch/roll + 航迹角/里程计 yaw） */
    bool   gpsHasAttitude() const { return get<bool>("gps.has_attitude", false); }
    // ── GPS 姿态估计（IMU pitch/roll + GPS 航迹角 yaw）────────────────────────
    bool   gpsAttitudeEstimationEnable() const { return get<bool>("gps.attitude_estimation.enable", true); }
    double gpsAttitudeImuLowpassHz() const { return std::max(0.1, get<double>("gps.attitude_estimation.imu_lowpass_cutoff_hz", 0.5)); }
    double gpsAttitudeMinVelocityForYaw() const { return std::max(0.1, get<double>("gps.attitude_estimation.min_velocity_for_yaw", 1.0)); }
    int    gpsAttitudeYawSmoothingWindow() const { return std::max(1, std::min(21, get<int>("gps.attitude_estimation.yaw_smoothing_window", 5))); }
    double gpsAttitudeYawMaxChangeRad() const { return std::max(0.01, get<double>("gps.attitude_estimation.yaw_max_change_rad", 0.5)); }
    double gpsAttitudePitchRollBaseVar() const { return std::max(1e-6, get<double>("gps.attitude_estimation.pitch_roll_base_var", 0.01)); }
    double gpsAttitudeYawVarVelocityScale() const { return std::max(1e-6, get<double>("gps.attitude_estimation.yaw_var_velocity_scale", 0.1)); }
    Eigen::Vector3d gpsCovExcellent() const;
    Eigen::Vector3d gpsCovHigh()     const;
    Eigen::Vector3d gpsCovMedium()   const;
    Eigen::Vector3d gpsCovLow()      const;
    /** 是否在配置中显式指定 ENU 原点 (lat, lon, alt)。存在则 GPSManager 不再用首条 GPS 覆盖原点。 */
    bool gpsEnuOriginConfigured() const;
    /** 若已配置 ENU 原点，返回 [lat, lon, alt]；否则返回 [0,0,0]。调用前可用 gpsEnuOriginConfigured() 判断。 */
    Eigen::Vector3d gpsEnuOrigin() const;

    // ── 【优化】GPS动态协方差和异常值检测参数 ─────────────────────────────
    bool   gpsEnableDynamicCov() const { return get<bool>("gps.enable_dynamic_cov", true); }
    int    gpsMinSatellites()      const { return get<int>("gps.min_satellites", 4); }
    double gpsHighAltitudeThreshold() const { return get<double>("gps.high_altitude_threshold", 100.0); }
    double gpsHighAltitudeScale()    const { return get<double>("gps.high_altitude_scale", 2.0); }
    
    bool   gpsEnableOutlierDetection() const { return get<bool>("gps.enable_outlier_detection", true); }
    double gpsOutlierZScore()        const { return get<double>("gps.outlier_z_score", 3.0); }
    double gpsOutlierCovScale()      const { return get<double>("gps.outlier_cov_scale", 100.0); }
    double gpsResidualBaseline()     const { return get<double>("gps.residual_baseline", 2.0); }

    // ── 回环检测 ──────────────────────────────────────────
    double overlapThreshold()   const { return get<double>("loop_closure.overlap_threshold", 0.3); }
    int    loopTopK()           const { return get<int>("loop_closure.top_k", 5); }
    double loopMinTemporalGap() const { return get<double>("loop_closure.min_temporal_gap_s", 30.0); }
    /** 最小子图间隔：候选与当前子图索引差至少为此值才作回环。2=更易在短轨迹(少量子图)下产生回环，3=更保守 */
    int    loopMinSubmapGap()   const { return std::max(1, get<int>("loop_closure.min_submap_gap", 2)); }
    double gpsSearchRadius()    const { return get<double>("loop_closure.gps_search_radius_m", 200.0); }
    /** 回环几何距离预筛：两子图锚定位姿距离超过此值(米)的候选将被过滤，抑制重复结构误检。0=关闭。默认 0 */
    double loopGeoPrefilterMaxDistanceM() const {
        double v = get<double>("loop_closure.geo_prefilter_max_distance_m", 0.0);
        return std::max(0.0, v);
    }
    int    loopWorkerThreads()  const { return get<int>("loop_closure.worker_threads", 2); }
    /** 回环描述子队列最大长度，超限丢弃最低优先级，防无界堆积。默认 128 */
    size_t loopMaxDescQueueSize() const {
        int v = get<int>("loop_closure.max_desc_queue_size", 128);
        return static_cast<size_t>(std::max(32, std::min(512, v)));
    }
    /** 回环匹配队列最大长度，超限丢弃最旧任务。默认 128 */
    size_t loopMaxMatchQueueSize() const {
        int v = get<int>("loop_closure.max_match_queue_size", 128);
        return static_cast<size_t>(std::max(32, std::min(512, v)));
    }

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
    int    teaserMaxPoints()    const { return std::max(500, get<int>("loop_closure.teaser.max_points", 8000)); }
    bool   teaserICPRefine()    const { return get<bool>("loop_closure.teaser.icp_refine", true); }

    // ── 后端帧率控制（前端每帧都发，后端可每隔 N 帧处理一帧以减轻负载）────────────────
    /** 每隔多少帧处理一帧（1=每帧都处理，5=默认跳过4帧处理1帧）；仅影响后端 tryCreateKeyFrame，队列仍每帧弹出不阻塞 */
    int backendProcessEveryNFrames() const {
        int v = get<int>("backend.process_every_n_frames", 5);
        return std::max(1, std::min(100, v));
    }
    /** 每处理多少帧发布一次全局图（buildGlobalMap+voxel 下采样，耗时随地图增大；增大可减轻后端峰值阻塞，默认 100） */
    int backendPublishGlobalMapEveryNProcessed() const {
        int v = get<int>("backend.publish_global_map_every_n_processed", 100);
        return std::max(50, std::min(1000, v));
    }

    // ── 性能优化（performance.*）────────────────────────────────────────────
    bool asyncGlobalMapBuild() const { return get<bool>("performance.async_global_map_build", true); }
    bool asyncIsam2Update() const { return get<bool>("performance.async_isam2_update", false); }
    bool parallelVoxelDownsample() const { return get<bool>("performance.parallel_voxel_downsample", true); }
    bool parallelTeaserMatch() const { return get<bool>("performance.parallel_teaser_match", true); }
    int maxOptimizationQueueSize() const {
        int v = get<int>("performance.max_optimization_queue_size", 64);
        return std::max(4, std::min(256, v));
    }

    // ── iSAM2 ─────────────────────────────────────────────
    double isam2RelinThresh()   const { return get<double>("backend.isam2.relinearize_threshold", 0.01); }
    int    isam2RelinSkip()     const { return get<int>("backend.isam2.relinearize_skip", 1); }
    bool   isam2EnableRelin()   const { return get<bool>("backend.isam2.enable_relinearization", true); }
    /** 先验因子方差（6 维共用）：仅作用于第一个/固定子图节点，过小=过强约束可能加重数值刚度，过大=原点易漂移。默认 1e-8 平衡清晰度与稳定性。见 docs/CONFIG_M2DGR_BLUR_OPTIMIZATION.md */
    double isam2PriorVariance() const {
        double v = get<double>("backend.isam2.prior_variance", 1e-8);
        return std::max(1e-10, std::min(1e-4, v));
    }

    // ── HBA ───────────────────────────────────────────────
    /** 是否启用 HBA（周期/对齐/结束时的触发）。false 时仅跑 ISAM2+GPS，用于隔离双路 GTSAM 崩溃。默认 true */
    bool   hbaEnabled()         const { return get<bool>("backend.hba.enabled", true); }
    int    hbaTotalLayers()     const { return get<int>("backend.hba.total_layer_num", 3); }
    int    hbaThreadNum()       const { return get<int>("backend.hba.thread_num", 8); }
    int    hbaTriggerSubmaps()  const { return get<int>("backend.hba.trigger_every_n_submaps", 10); }
    bool   hbaOnLoop()          const { return get<bool>("backend.hba.trigger_on_loop", false); }
    bool   hbaOnFinish()        const { return get<bool>("backend.hba.trigger_on_finish", true); }
    std::string hbaDataPath()   const { return get<std::string>("backend.hba.data_path", "/tmp/hba_data"); }
    /** 是否允许使用 GTSAM fallback 做 HBA（无 hba_api 时）。生产建议 false，避免 double free；见 docs/HBA_GTSAM_FALLBACK_DOUBLE_FREE_FIX.md */
    bool   hbaGtsamFallbackEnabled() const { return get<bool>("backend.hba.enable_gtsam_fallback", false); }

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
    /** 传感器空闲超时（秒），load() 中根据 mode.type 与 mode.online/offline 设置；无 mode 时用 system.sensor_idle_timeout_sec */
    double sensor_idle_timeout_sec_{10.0};
    /** 直接从 cfg_ 读取 sensor.gps.topic，用于 gpsTopic() 兜底 */
    std::string getSensorGpsTopicRaw() const;

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
