#pragma once
#include <algorithm>
#include <cstdlib>
#include <map>
#include <sstream>
#include <type_traits>
#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

class ConfigManager {
public:
    static ConfigManager& instance() {
        static ConfigManager inst;
        return inst;
    }

    /**
     * 全工程唯一配置入口：整个进程应只调用一次 load（由 AutoMapSystem::loadConfigAndInit 或主节点在启动时调用）。
     * 前端、后端、回环等均通过 ConfigManager::instance() 读参，不再单独读 YAML。
     * 重复调用：若路径与已加载相同则忽略；若路径不同则抛异常，避免多源不一致。
     */
    void load(const std::string& yaml_path);
    void loadFromFile(const std::string& yaml_path) { load(yaml_path); }  // 别名，standalone 节点使用

    /** 是否已加载过配置（任意组件可通过此判断是否可安全使用 getter） */
    bool isLoaded() const { return !config_file_path_.empty(); }
    /** 当前加载的配置文件路径（唯一源；未加载时为空） */
    const std::string& configFilePath() const { return config_file_path_; }

    // ── 系统 ──────────────────────────────────────────────
    std::string systemName()    const { return get<std::string>("system.name",    "AutoMap-Pro"); }
    std::string outputDir()     const { return get<std::string>("system.output_dir", "/data/automap_output"); }
    int         numThreads()    const { return get<int>("system.num_threads", 8); }
    /** 传感器数据空闲超过该秒数且队列已空时，触发最终处理并结束建图（需同时开启 auto_finish_on_sensor_idle）；离线/在线由 mode 段区分，load() 时写入 */
    double      sensorIdleTimeoutSec() const { return std::max(1.0, sensor_idle_timeout_sec_); }
    /** 是否在传感器空闲超时后自动执行最终 HBA、导出地图并结束建图 */
    bool        autoFinishOnSensorIdle() const { return get<bool>("system.auto_finish_on_sensor_idle", true); }
    /** 离线模式“播完再结束”；load() 时从 system 节缓存 */
    bool        offlineFinishAfterBag() const {
        if (system_queue_cached_) return system_offline_finish_after_bag_;
        return get<bool>("system.offline_finish_after_bag", false);
    }
    /** 后端帧队列最大长度；load() 时从 system 节缓存，避免 get() 路径解析与文件不一致 */
    size_t      frameQueueMaxSize() const {
        if (system_queue_cached_) {
            size_t v = static_cast<size_t>(system_frame_queue_max_size_);
            if (v < 100) return 100;
            if (v > 10000) return 10000;
            return v;
        }
        int v = get<int>("system.frame_queue_max_size", 500);
        if (v < 100) return 100;
        if (v > 10000) return 10000;
        return static_cast<size_t>(v);
    }
    /** 入口缓冲队列长度；load() 时从 system 节缓存 */
    size_t      ingressQueueMaxSize() const {
        if (system_queue_cached_) {
            size_t v = static_cast<size_t>(system_ingress_queue_max_size_);
            if (v < 2) return 2;
            if (v > 256) return 256;
            return v;
        }
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
    /** 优先用 load() 时缓存的 sensor.lidar.topic，再 get()/Raw，保证与文件一致（见 LOG_ANALYSIS 配置修复） */
    std::string lidarTopic() const;
    /** 优先用 load() 时缓存的 sensor.imu.topic，再 get()/Raw */
    std::string imuTopic() const;
    /** 从 sensor.gps.topic 读取；若 get() 因键路径问题返回默认值，则从 YAML 直接读取兜底（避免 LivoBridge 误用 /gps/fix） */
    std::string gpsTopic() const;
    /** 从 sensor.gps.enabled 读取；优先从 YAML 直接读 cfg_["sensor"]["gps"]["enabled"]，与 gpsTopic() 一致，避免 get() 路径解析问题导致始终为 false */
    bool gpsEnabled() const;
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
    double kfMinTranslation()   const { return get<double>("keyframe.min_translation", 0.5); }
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

    // ── GPS 延迟对齐（load() 时一次性读入缓存，getter 仅返回缓存值，保证与日志一致）──────────────────────
    int    gpsAlignMinPoints()  const { return gps_cached_ ? gps_align_min_points_  : get<int>("gps.align_min_points", 50); }
    double gpsAlignMinDist()    const { return gps_cached_ ? gps_align_min_distance_m_ : get<double>("gps.align_min_distance_m", 30.0); }
    double gpsQualityThreshold()const { return gps_cached_ ? gps_quality_threshold_hdop_ : get<double>("gps.quality_threshold_hdop", 2.0); }
    double gpsAlignRmseThresh() const { return gps_cached_ ? gps_align_rmse_threshold_m_ : get<double>("gps.align_rmse_threshold_m", 1.5); }
    int    gpsGoodSamplesNeeded()const { return gps_cached_ ? gps_good_samples_needed_ : get<int>("gps.good_samples_needed", 30); }
    bool   gpsAddConstraintsOnAlign() const { return gps_cached_ ? gps_add_constraints_on_align_ : get<bool>("gps.add_constraints_on_align", true); }
    double gpsFactorIntervalM() const { return gps_cached_ ? gps_factor_interval_m_ : get<double>("gps.factor_interval_m", 5.0); }
    /** 关键帧匹配 GPS 的最大时间差(秒)。1Hz GPS 时建议 0.5~1.0，原 0.1 易导致 has_gps=0。默认 0.5 */
    double gpsKeyframeMatchWindowS() const {
        if (gps_cached_) return gps_keyframe_match_window_s_;
        return std::max(0.1, get<double>("gps.keyframe_match_window_s", 0.5));
    }
    /** 关键帧绑定 GPS 时允许的最大 HDOP（放宽 M2DGR 等弱 GPS 场景）。默认 12.0 */
    double gpsKeyframeMaxHdop() const {
        double v = gps_cached_ ? gps_keyframe_max_hdop_ : get<double>("gps.keyframe_max_hdop", 12.0);
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
    double gpsFactorWeight() const {
        double v = gps_cached_ ? gps_factor_weight_ : get<double>("gps.factor_weight", 1.0);
        return std::max(0.01, v);
    }
    /** 按质量自动缩放：EXCELLENT/HIGH/MEDIUM 对应因子强度倍数（>1 则该质量约束更强），与 factor_weight 相乘 */
    double gpsFactorQualityScaleExcellent() const {
        double v = gps_cached_ ? gps_factor_quality_scale_excellent_ : get<double>("gps.factor_quality_scale_excellent", 2.0);
        return std::max(0.1, v);
    }
    double gpsFactorQualityScaleHigh() const {
        double v = gps_cached_ ? gps_factor_quality_scale_high_ : get<double>("gps.factor_quality_scale_high", 1.0);
        return std::max(0.1, v);
    }
    double gpsFactorQualityScaleMedium() const {
        double v = gps_cached_ ? gps_factor_quality_scale_medium_ : get<double>("gps.factor_quality_scale_medium", 0.5);
        return std::max(0.1, v);
    }
    double gpsFactorQualityScaleLow() const {
        double v = gps_cached_ ? gps_factor_quality_scale_low_ : get<double>("gps.factor_quality_scale_low", 0.25);
        return std::max(0.1, v);
    }
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
    /**
     * GPS 天线相对 IMU 原点的杆臂（米），在 **IMU/body 系** 下表达（与 HBA 中 pos_imu = pos_gps - R*T_w_b * lever_arm 一致）。
     * 来自标定：t_gnss^L - t_imu^L（两外参均为 [to LIDAR] 且 R=I 时与 body 系轴对齐）。
     * 未配置或全零时 HBA 可回退 legacy `sensor_config/gps_imu_extrinsic.yaml`。
     */
    Eigen::Vector3d gpsLeverArmImu() const;

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
    /** 最小子图间隔：0=不按子图间隔过滤(允许相邻子图回环)；>0=候选与当前子图索引差须大于该值。默认 0 */
    int    loopMinSubmapGap()   const { return std::max(0, get<int>("loop_closure.min_submap_gap", 0)); }
    double gpsSearchRadius()    const { return get<double>("loop_closure.gps_search_radius_m", 200.0); }
    /** 回环几何距离预筛：两子图锚定位姿距离超过此值(米)的候选将被过滤，抑制重复结构误检。0=关闭。默认 0 */
    double loopGeoPrefilterMaxDistanceM() const {
        double v = get<double>("loop_closure.geo_prefilter_max_distance_m", 0.0);
        return std::max(0.0, v);
    }
    /** 高置信度绕过几何预筛：描述子相似度≥此值时不做距离过滤，直接进入 TEASER（参考 OverlapTransformer/SeqOT 高 overlap 即可靠）。0=关闭。建议 0.90~0.95 */
    double loopGeoPrefilterSkipAboveScore() const {
        double v = get<double>("loop_closure.geo_prefilter_skip_above_score", 0.0);
        return std::max(0.0, std::min(1.0, v));
    }
    int    loopWorkerThreads()  const { return get<int>("loop_closure.worker_threads", 2); }

    // ── 子图内回环检测配置 ─────────────────────────────────────────────────
    /** 子图内回环检测：关键帧之间最小时间间隔（秒） */
    double intraSubmapLoopMinTemporalGap() const {
        return get<double>("loop_closure.intra_submap_min_temporal_gap_s", 5.0);
    }
    /** 子图内回环检测：是否启用 */
    bool intraSubmapLoopEnabled() const {
        return get<bool>("loop_closure.intra_submap_enabled", false);
    }
    /** 子图内回环检测：关键帧对之间最小索引间隔（避免相邻帧假回环），可配 20 或更大 */
    int intraSubmapLoopMinKeyframeGap() const {
        return get<int>("loop_closure.intra_submap_min_keyframe_gap", 20);
    }
    /** 子图内回环检测：关键帧之间最小距离间隔(米)，避免过密假回环 */
    double intraSubmapLoopMinDistanceGap() const {
        return get<double>("loop_closure.intra_submap_min_distance_gap_m", 3.0);
    }
    /** 子图内回环检测：描述子相似度阈值 */
    double intraSubmapLoopOverlapThreshold() const {
        return get<double>("loop_closure.intra_submap_overlap_threshold", 0.3);
    }
    /** 子图内回环：每帧最多对多少候选做 TEASER 配准（避免候选过多导致 10s+ 卡住），≤0 表示不限制 */
    int intraSubmapLoopMaxTeaserCandidates() const {
        return get<int>("loop_closure.intra_submap_max_teaser_candidates", 5);
    }
    /** 子图内回环单次检测最大耗时(秒)，超时则本帧跳过添加回环因子，避免后端阻塞；≤0 表示不限制（同步执行） */
    double intraSubmapLoopMaxDurationSec() const {
        return get<double>("loop_closure.intra_submap_max_duration_sec", 8.0);
    }
    /** 子图内回环是否与主线程异步：true=仅投递任务到 intra_loop_worker，主线程不等待；false=在当前线程执行（可配合 max_duration_sec 超时） */
    bool intraSubmapLoopAsync() const {
        return get<bool>("loop_closure.intra_submap_async", true);
    }

    /** 子图间回环是否使用关键帧级（true=关键帧↔关键帧跨子图，false=子图↔子图） */
    bool interKeyframeLevelEnabled() const {
        return get<bool>("loop_closure.inter_keyframe_level", true);
    }
    /** 子图间关键帧级：query 子图内每隔多少关键帧取一个做检索（1=每帧，5=每5帧，控制计算量） */
    int interKeyframeSampleStep() const {
        return std::max(1, get<int>("loop_closure.inter_keyframe_sample_step", 5));
    }
    /** 子图间关键帧级：每个候选子图内取 top-K 个关键帧做 TEASER（控制每 query 关键帧的匹配对数） */
    int interKeyframeTopKPerSubmap() const {
        return std::max(1, std::min(20, get<int>("loop_closure.inter_keyframe_top_k_per_submap", 3)));
    }
    /** 子图间回环：两关键帧对之间最小全局关键帧间隔（帧数），0=不限制 */
    int interSubmapMinKeyframeGap() const {
        return std::max(0, get<int>("loop_closure.inter_submap_min_keyframe_gap", 20));
    }
    /** 回环检测节流：成功检测到回环后，隔多少关键帧再触发下一次检测；0=不节流；仅成功检测后生效，未成功则下一帧继续检测 */
    int loopDetectionMinKeyframeIntervalAfterSuccess() const {
        return std::max(0, get<int>("loop_closure.loop_detection_min_keyframe_interval_after_success", 20));
    }

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
    /** 返回 model_path，${CMAKE_CURRENT_SOURCE_DIR} 已在 load() 中展开为配置所在包根目录；未加载时走 get() */
    std::string overlapModelPath()   const;
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

    // ── ScanContext ────────────────────────────────────────────────────────
    bool   scancontextEnabled()        const { return get<bool>("loop_closure.scancontext.enabled", true); }
    double scancontextDistThreshold()  const { return get<double>("loop_closure.scancontext.dist_threshold", 0.13); }
    int    scancontextNumCandidates()  const { return get<int>("loop_closure.scancontext.num_candidates", 5); }
    int    scancontextExcludeRecent()  const { return get<int>("loop_closure.scancontext.exclude_recent", 50); }
    int    scancontextTreeMakingPeriod() const { return get<int>("loop_closure.scancontext.tree_making_period", 50); }

    /** 最小安全内点数：TEASER 内点数低于此值会拒绝（防崩溃/误匹配）。弱重叠场景可放宽至 6，需关注误匹配。 */
    int    teaserMinSafeInliers() const {
        int v = get<int>("loop_closure.teaser.min_safe_inliers", 10);
        return std::max(1, std::min(50, v));
    }
    /** FPFH 对应点几何过滤：仅保留距离小于此值(米)的对应点再送 TEASER。树木/植被场景建议 3～5，结构化场景可用 10。0=禁用过滤。 */
    double teaserFpfhCorrMaxDistanceM() const {
        double v = get<double>("loop_closure.teaser.fpfh_corr_max_distance_m", 10.0);
        return std::max(0.0, std::min(100.0, v));
    }
    /** 回环最小相对平移(米)：低于此值的回环视为 trivial 不加入图，提高回环质量。默认 0.05 */
    double loopMinRelativeTranslationM() const {
        double v = get<double>("loop_closure.teaser.min_relative_translation_m", 0.05);
        return std::max(0.0, std::min(10.0, v));
    }
    /** 回环位姿一致性：TEASER 相对位姿与里程计相对位姿的平移差异超过此值(米)视为异常，拒绝该回环（理论上回环应是微调）。默认 2.0，0=关闭检查 */
    double loopPoseConsistencyMaxTransDiffM() const {
        double v = get<double>("loop_closure.pose_consistency_max_trans_diff_m", 2.0);
        return std::max(0.0, std::min(50.0, v));
    }
    /** 回环位姿一致性：TEASER 与 odom 相对旋转差异超过此值(度)视为异常，拒绝。默认 25.0，0=关闭检查 */
    double loopPoseConsistencyMaxRotDiffDeg() const {
        double v = get<double>("loop_closure.pose_consistency_max_rot_diff_deg", 25.0);
        return std::max(0.0, std::min(180.0, v));
    }

    // ── 后端帧率控制（前端每帧都发，后端可每隔 N 帧处理一帧以减轻负载）────────────────
    /** 每隔多少帧处理一帧（1=每帧都处理，5=默认跳过4帧处理1帧）；仅影响后端 tryCreateKeyFrame，队列仍每帧弹出不阻塞 */
    int backendProcessEveryNFrames() const {
        int v = get<int>("backend.process_every_n_frames", 5);
        return std::max(1, std::min(100, v));
    }
    /** 单帧处理耗时超过此值(秒)时打 WARN 便于诊断卡点；≤0 表示不告警。默认 15.0 */
    double backendSingleFrameWarnDurationSec() const {
        return get<double>("backend.single_frame_warn_duration_sec", 15.0);
    }
    /** 每处理多少帧发布一次全局图（buildGlobalMap+voxel 下采样，耗时随地图增大；增大可减轻后端峰值阻塞，默认 100） */
    int backendPublishGlobalMapEveryNProcessed() const {
        int v = get<int>("backend.publish_global_map_every_n_processed", 100);
        return std::max(50, std::min(1000, v));
    }
    /** 子图冻结时合并 forceUpdate：先 flush 所有 pending GPS 再统一一次 forceUpdate，减少 ISAM2 调用次数（默认 false 保持原 3 次）。见 BACKEND_FURTHER_OPTIMIZATION_OPPORTUNITIES.md */
    bool backendForceUpdateCoalesceOnSubmapFreeze() const {
        return get<bool>("backend.force_update_coalesce_on_submap_freeze", false);
    }
    /** Keyframe 级 pending GPS 因子队列上限，超过时丢弃最旧（FIFO），防长时间失败导致 OOM。默认 1000，范围 100～5000。见 BACKEND_POTENTIAL_ISSUES 1.2.1 */
    int backendMaxPendingGpsKeyframeFactors() const {
        int v = get<int>("backend.max_pending_gps_keyframe_factors", 1000);
        return std::max(100, std::min(5000, v));
    }
    /** 后端详细追踪日志：true=每个计算环节打 BACKEND_TRACE，便于证据链闭环定位问题；发布阶段置 false 关闭 */
    bool backendVerboseTrace() const {
        return get<bool>("backend.verbose_trace", false);
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
    /** 前端数据（点云/里程计）超过此秒数未到达且后端队列已空时，触发一次 HBA。≤0 表示不按空闲触发，仅依赖 trigger_on_finish/周期。默认 10.0 */
    double hbaFrontendIdleTriggerSec() const {
        double v = get<double>("backend.hba.frontend_idle_trigger_sec", 10.0);
        return v < 0.0 ? 0.0 : v;
    }
    /** frontend_idle 触发 HBA 时至少需要的子图数，避免「第一个子图刚建完就 HBA」。≤0 表示仅要求 >0；默认 2。若期望共 N 个子图再优化可设为 N（如 5）。 */
    int hbaFrontendIdleMinSubmaps() const {
        return get<int>("backend.hba.frontend_idle_min_submaps", 2);
    }

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
    /** 加载过的配置文件路径；全工程唯一源，用于“只加载一次”守卫与 configFilePath() */
    std::string config_file_path_;
    /** 传感器空闲超时（秒），load() 中根据 mode.type 与 mode.online/offline 设置；无 mode 时用 system.sensor_idle_timeout_sec */
    double sensor_idle_timeout_sec_{10.0};

    /** load() 时一次性读取的 GPS 配置缓存，避免重复解析 YAML 与默认值不一致 */
    bool gps_cached_{false};
    int    gps_align_min_points_{50};
    double gps_align_min_distance_m_{30.0};
    double gps_quality_threshold_hdop_{2.0};
    double gps_align_rmse_threshold_m_{1.5};
    int    gps_good_samples_needed_{30};
    bool   gps_add_constraints_on_align_{true};
    double gps_factor_interval_m_{5.0};
    double gps_keyframe_match_window_s_{0.5};
    double gps_keyframe_max_hdop_{12.0};
    double gps_factor_weight_{1.0};
    double gps_factor_quality_scale_excellent_{2.0};
    double gps_factor_quality_scale_high_{1.0};
    double gps_factor_quality_scale_medium_{0.5};
    double gps_factor_quality_scale_low_{0.25};
    /** 直接从 cfg_ 读取 sensor.gps/lidar/imu.topic，用于 getter 兜底，保证与 YAML 一致 */
    std::string getSensorGpsTopicRaw() const;
    std::string getSensorLidarTopicRaw() const;
    std::string getSensorImuTopicRaw() const;
    /** load() 时从 flatten 结果填充：当 get()/Raw 路径解析失败时由此回退，保证与 CONFIG_FILE_PARAMS 一致（见 LOG_ANALYSIS 配置读取修复） */
    std::map<std::string, std::string> flat_params_cache_;
    /** load() 时从 flat 或 YAML 填写的 sensor 话题缓存，避免 get() 返回默认值 */
    bool sensor_lidar_topic_cached_{false};
    bool sensor_imu_topic_cached_{false};
    std::string sensor_lidar_topic_value_;
    std::string sensor_imu_topic_value_;
    /** load() 时从 flat 或 YAML 填写的 model_path 展开后缓存（${CMAKE_CURRENT_SOURCE_DIR} 已替换为包根目录） */
    std::string overlap_model_path_expanded_;
    /** load() 时从 system 节读入并缓存，保证 CONFIG_READ_BACK 与文件一致 */
    bool system_queue_cached_{false};
    int system_frame_queue_max_size_{500};
    int system_ingress_queue_max_size_{16};
    bool system_offline_finish_after_bag_{false};
    /** load() 时从 sensor.gps.enabled 读入并缓存，避免 gpsEnabled() 与 load() 内诊断不一致（见 LOG_ANALYSIS） */
    bool sensor_gps_enabled_cached_{false};
    bool sensor_gps_enabled_value_{false};

    /** 从 flat_params_cache_ 解析字符串为 T，get() 在 node 路径失败时回退使用 */
    template<typename T>
    T parseFlatValue(const std::string& s, const T& default_val) const {
        try {
            if constexpr (std::is_same_v<T, std::string>) return s;
            if constexpr (std::is_same_v<T, int>) return static_cast<int>(std::stoi(s));
            if constexpr (std::is_same_v<T, double>) return std::stod(s);
            if constexpr (std::is_same_v<T, float>) return std::stof(s);
            if constexpr (std::is_same_v<T, bool>) {
                if (s == "true" || s == "1" || s == "yes") return true;
                if (s == "false" || s == "0" || s == "no") return false;
                return default_val;
            }
        } catch (...) {}
        return default_val;
    }

    template<typename T>
    T get(const std::string& key, const T& default_val) const {
        try {
            YAML::Node node = cfg_;
            std::istringstream ss(key);
            std::string token;
            while (std::getline(ss, token, '.')) {
                if (!node.IsMap() || !node[token]) break;
                node = node[token];
            }
            if (!ss.eof()) { /* 未消费完 token，路径中断，走 flat 回退 */ }
            else {
                try {
                    T v = node.as<T>();
                    return v;
                } catch (...) {}
            }
        } catch (...) {}
        auto it = flat_params_cache_.find(key);
        if (it != flat_params_cache_.end()) {
            try {
                return parseFlatValue<T>(it->second, default_val);
            } catch (...) {}
        }
        return default_val;
    }

    ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
};

} // namespace automap_pro
