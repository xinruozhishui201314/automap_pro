#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/map_frame_config.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/SVD>
#include <chrono>
#include <limits>
#include <numeric>
#include <cmath>

#define MOD "GPSManager"

namespace automap_pro {

namespace {
    // 取两者中较差的 GPS 质量（用于插值结果的质量与 is_valid 判定）
    GPSQuality worseQuality(GPSQuality a, GPSQuality b) {
        return static_cast<int>(a) <= static_cast<int>(b) ? a : b;
    }

    // 快照内 enu→map 变换（仅用快照的 R/t，不持锁）
    Eigen::Vector3d enu_to_map_from_snapshot(const Eigen::Vector3d& enu,
                                             const GPSManager::GpsWindowSnapshot& snap) {
        if (!snap.is_aligned) return enu;
        return snap.R_gps_lidar * enu + snap.t_gps_lidar;
    }
    Eigen::Vector3d map_to_enu_from_snapshot(const Eigen::Vector3d& map_pos,
                                             const GPSManager::GpsWindowSnapshot& snap) {
        if (!snap.is_aligned) return map_pos;
        return snap.R_gps_lidar.transpose() * (map_pos - snap.t_gps_lidar);
    }

    std::optional<GPSMeasurement> queryByTimestampOnSnapshot(double ts,
                                                             const GPSManager::GpsWindowSnapshot& snap) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampOnSnapshot enter ts={:.3f} window_size={}", ts, snap.window.size());
        const double max_dt = snap.keyframe_match_window_s;
        const double max_interp_gap_s = snap.max_interp_gap_s;
        const auto& w = snap.window;
        const GPSManager::GpsWindowSnapshot::Record* r_before = nullptr;
        const GPSManager::GpsWindowSnapshot::Record* r_after = nullptr;
        for (const auto& r : w) {
            if (r.timestamp <= ts) r_before = &r;
            if (r.timestamp >= ts && r_after == nullptr) r_after = &r;
        }
        if (r_before && r_after && r_before != r_after) {
            const double t0 = r_before->timestamp;
            const double t1 = r_after->timestamp;
            if (t1 > t0 && (t1 - t0) <= max_interp_gap_s) {
                const double alpha = (ts - t0) / (t1 - t0);
                GPSMeasurement m;
                m.timestamp = ts;
                m.position_enu = (1.0 - alpha) * r_before->pos_enu + alpha * r_after->pos_enu;
                m.hdop = (1.0 - alpha) * r_before->hdop + alpha * r_after->hdop;
                m.quality = worseQuality(r_before->quality, r_after->quality);
                m.is_valid = (m.hdop <= snap.keyframe_max_hdop);
                ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampOnSnapshot exit mode=interp t0={:.3f} t1={:.3f} is_valid={}", t0, t1, m.is_valid ? 1 : 0);
                return m;
            }
        }
        double best_dt = max_dt + 1.0;
        const GPSManager::GpsWindowSnapshot::Record* best = nullptr;
        for (const auto& r : w) {
            double dt = std::abs(r.timestamp - ts);
            if (dt <= max_dt && (best == nullptr || dt < best_dt)) {
                best_dt = dt;
                best = &r;
            }
        }
        if (!best) {
            ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampOnSnapshot exit mode=not_found ts={:.3f} max_dt={:.3f}", ts, max_dt);
            return std::nullopt;
        }
        GPSMeasurement m;
        m.timestamp = best->timestamp;
        m.position_enu = best->pos_enu;
        m.quality = best->quality;
        m.hdop = best->hdop;
        m.is_valid = (best->hdop <= snap.keyframe_max_hdop);
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampOnSnapshot exit mode=nearest best_dt={:.3f} gps_ts={:.3f} is_valid={}", best_dt, best->timestamp, m.is_valid ? 1 : 0);
        return m;
    }

    std::optional<Eigen::Vector3d> estimateVelocityOnSnapshot(double ts,
                                                             const GPSManager::GpsWindowSnapshot& snap) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] estimateVelocityOnSnapshot enter ts={:.3f} window_size={} half_win={:.3f}", ts, snap.window.size(), snap.velocity_estimation_window_s * 0.5);
        const double half = snap.velocity_estimation_window_s * 0.5;
        std::vector<std::pair<double, Eigen::Vector3d>> pts;
        for (const auto& r : snap.window) {
            if (r.timestamp >= ts - half && r.timestamp <= ts + half && r.quality >= GPSQuality::MEDIUM) {
                pts.push_back({r.timestamp, snap.is_aligned ? enu_to_map_from_snapshot(r.pos_enu, snap) : r.pos_enu});
            }
        }
        if (pts.size() < 2) {
            ALOG_DEBUG(MOD, "[GPS_QUERY] estimateVelocityOnSnapshot exit pts_size={} (need>=2)", pts.size());
            return std::nullopt;
        }
        std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
        Eigen::Vector3d sum_vel = Eigen::Vector3d::Zero();
        double sum_dt = 0.0;
        for (size_t i = 1; i < pts.size(); ++i) {
            double dt = pts[i].first - pts[i - 1].first;
            if (dt > 0.01 && dt < snap.velocity_estimation_window_s) {
                sum_vel += (pts[i].second - pts[i - 1].second) / dt;
                sum_dt += 1.0;
            }
        }
        if (sum_dt < 0.5) {
            ALOG_DEBUG(MOD, "[GPS_QUERY] estimateVelocityOnSnapshot exit sum_dt={:.3f} (need>=0.5)", sum_dt);
            return std::nullopt;
        }
        ALOG_DEBUG(MOD, "[GPS_QUERY] estimateVelocityOnSnapshot exit ok pts={} sum_dt={:.3f}", pts.size(), sum_dt);
        return sum_vel / sum_dt;
    }
} // namespace

GPSManager::GPSManager() {
    const auto& cfg = ConfigManager::instance();
    min_align_points_       = cfg.gpsAlignMinPoints();
    min_align_dist_m_       = cfg.gpsAlignMinDist();
    keyframe_match_window_s_ = cfg.gpsKeyframeMatchWindowS();
    keyframe_max_hdop_       = cfg.gpsKeyframeMaxHdop();
    max_interp_gap_s_        = cfg.gpsMaxInterpGapS();
    extrapolation_margin_s_  = cfg.gpsExtrapolationMarginS();
    extrapolation_uncertainty_scale_ = cfg.gpsExtrapolationUncertaintyScale();
    velocity_estimation_window_s_   = cfg.gpsVelocityEstimationWindowS();
    quality_hdop_thresh_    = cfg.gpsQualityThreshold();
    rmse_accept_thresh_     = cfg.gpsAlignRmseThresh();
    good_samples_needed_    = cfg.gpsGoodSamplesNeeded();

    // 【修复】初始化精準对齐增强参数
    online_calib_min_dist_    = 5.0;   // 最小累积距离触发在线校正（米）
    online_calib_max_rmse_    = 2.0;   // 在线校正RMSE阈值（米）
    online_calib_min_samples_ = 5;     // 在线校正最小样本数
    lever_arm_imu_            = cfg.gpsLeverArmImu();

    // 若配置中显式给出了 ENU 原点，经纬高以配置为准，避免首条 GPS 改写原点
    if (cfg.gpsEnuOriginConfigured()) {
        Eigen::Vector3d origin = cfg.gpsEnuOrigin();  // [lat, lon, alt]
        enu_origin_lat_ = origin.x();
        enu_origin_lon_ = origin.y();
        enu_origin_alt_ = origin.z();
        enu_origin_set_.store(true, std::memory_order_release);
        ALOG_INFO(MOD,
                  "[ENU_ORIGIN] Using configured ENU origin: lat={:.6f} lon={:.6f} alt={:.2f} "
                  "(gps.enu_origin from YAML, will be written to map_frame.cfg)",
                  enu_origin_lat_, enu_origin_lon_, enu_origin_alt_);
        std::string cfg_path = cfg.mapFrameConfigPath();
        MapFrameConfig::write(cfg_path, enu_origin_lat_, enu_origin_lon_, enu_origin_alt_);
    }
}

void GPSManager::applyConfig() {
    const auto& cfg = ConfigManager::instance();
    min_align_points_       = cfg.gpsAlignMinPoints();
    min_align_dist_m_      = cfg.gpsAlignMinDist();
    keyframe_match_window_s_ = cfg.gpsKeyframeMatchWindowS();
    keyframe_max_hdop_     = cfg.gpsKeyframeMaxHdop();
    max_interp_gap_s_      = cfg.gpsMaxInterpGapS();
    extrapolation_margin_s_ = cfg.gpsExtrapolationMarginS();
    extrapolation_uncertainty_scale_ = cfg.gpsExtrapolationUncertaintyScale();
    velocity_estimation_window_s_   = cfg.gpsVelocityEstimationWindowS();
    quality_hdop_thresh_   = cfg.gpsQualityThreshold();
    rmse_accept_thresh_   = cfg.gpsAlignRmseThresh();
    good_samples_needed_  = cfg.gpsGoodSamplesNeeded();
    lever_arm_imu_        = cfg.gpsLeverArmImu();
    ALOG_INFO(MOD, "[applyConfig] min_align_points={} min_align_dist_m={:.1f} keyframe_match_window_s={:.2f} good_samples_needed={} rmse_thresh={:.2f}m",
              min_align_points_, min_align_dist_m_, keyframe_match_window_s_, good_samples_needed_, rmse_accept_thresh_);
}

void GPSManager::addGPSMeasurement(
    double timestamp,
    double latitude, double longitude, double altitude,
    double hdop, int num_sats)
{
    // ENU 原点仅设置一次，避免多线程竞态；并写入 map_frame.cfg 供后端统一使用
    // 若已通过配置指定 ENU 原点，则此处仅负责写回配置文件，不再用首条 GPS 覆盖。
    std::call_once(enu_origin_once_, [this, latitude, longitude, altitude]() {
        if (!enu_origin_set_.load(std::memory_order_acquire)) {
            enu_origin_lat_ = latitude;
            enu_origin_lon_ = longitude;
            enu_origin_alt_ = altitude;
            enu_origin_set_.store(true, std::memory_order_release);
            ALOG_INFO(MOD, "ENU origin set from first GPS fix: lat={:.6f} lon={:.6f} alt={:.2f}",
                      latitude, longitude, altitude);
        } else {
            ALOG_INFO(MOD,
                      "ENU origin already configured (e.g., from YAML). "
                      "Skipping GPS-based override and writing existing origin to map_frame.cfg");
        }
        std::string cfg_path = ConfigManager::instance().mapFrameConfigPath();
        MapFrameConfig::write(cfg_path, enu_origin_lat_, enu_origin_lon_, enu_origin_alt_);
    });

    Eigen::Vector3d pos_enu;
    GPSQuality quality;
    double log_ts = 0.0;
    Eigen::Vector3d log_pos;
    std::vector<MeasurementLogCallback> log_cbs;

    {
        std::lock_guard<std::recursive_mutex> lk(mutex_);

        pos_enu = wgs84_to_enu(latitude, longitude, altitude);
        quality = hdop_to_quality(hdop);

        // 🔧 V3 修复：GPS 杆臂补偿
        // 原理：pos_body = pos_antenna - R_enu_body * lever_arm
        // 注意：此处我们需要 body 在 ENU 系下的姿态。
        // 1. 如果已对齐，使用当前对齐矩阵 R_enu_odom * R_odom_body
        // 2. 如果未对齐，暂不补偿或仅补偿 Z（此处选择使用最近邻姿态作为参考）
        if (lever_arm_imu_.norm() > 1e-6) {
            auto odom_opt = findNearestOdomPose(timestamp);
            if (odom_opt) {
                Eigen::Matrix3d R_enu_body;
                if (state_ == GPSAlignState::ALIGNED || state_ == GPSAlignState::DEGRADED) {
                    // R_enu_body = R_enu_odom * R_odom_body
                    // align_result_.R_gps_lidar 就是 R_enu_odom
                    R_enu_body = align_result_.R_gps_lidar * odom_opt->second.linear();
                } else {
                    // 未对齐时，假设 odom 系与 ENU 系水平对齐（仅用于初始对齐前的粗略补偿）
                    R_enu_body = odom_opt->second.linear();
                }
                pos_enu -= R_enu_body * lever_arm_imu_;
                
                static std::atomic<uint32_t> lever_arm_log_count{0};
                if (lever_arm_log_count.fetch_add(1) % 100 == 0) {
                    Eigen::Vector3d antenna_pos = pos_enu + R_enu_body * lever_arm_imu_;
                    ALOG_INFO(MOD, "[V3][POSE_DIAG] GPS lever arm comp: antenna=({:.2f},{:.2f},{:.2f}) -> imu=({:.2f},{:.2f},{:.2f}) R_enu_body_yaw={:.1f}deg state={}",
                              antenna_pos.x(), antenna_pos.y(), antenna_pos.z(),
                              pos_enu.x(), pos_enu.y(), pos_enu.z(),
                              std::atan2(R_enu_body(1,0), R_enu_body(0,0)) * 180.0 / M_PI,
                              static_cast<int>(state_.load()));
                }
            }
        }

        // 填入滑动窗口
        GPSRecord rec;
        rec.timestamp = timestamp;
        rec.pos_enu   = pos_enu;
        rec.quality   = quality;
        rec.hdop      = hdop;
        gps_window_.push_back(rec);
        const size_t max_gps_window = ConfigManager::instance().gpsMaxWindowSize();
        while (gps_window_.size() > max_gps_window) gps_window_.pop_front();

        // 诊断：首次与每 100 条打印，便于确认 GPS 是否进入系统（若轨迹 CSV 无 GPS 且从未见本日志，则 LivoBridge 未收到有效 fix）
        static std::atomic<uint32_t> add_count{0};
        uint32_t n = add_count++;
        size_t w = gps_window_.size();
        if (n == 0) {
            ALOG_INFO(MOD, "[GPS_DIAG] First GPS measurement added: ts={:.3f} lat={:.6f} lon={:.6f} hdop={:.2f} quality={} window_size={}",
                      timestamp, latitude, longitude, hdop, static_cast<int>(quality), w);
        } else if (n % 100 == 0) {
            ALOG_INFO(MOD, "[GPS_DIAG] GPS measurement #{} ts={:.3f} window_size={}", n + 1, timestamp, w);
        }

        // 【修复】复制回调列表与数据，在锁外执行，避免阻塞 Executor（死锁/长时间持锁）
        log_ts = timestamp;
        log_pos = pos_enu;
        log_cbs = measurement_log_cbs_;
    }

    // 轨迹对比：在锁外执行，避免 onGPSMeasurementForLog 等回调内锁与 mutex_ 形成死锁
    for (auto& cb : log_cbs) {
        try {
            cb(log_ts, log_pos);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[GPS_CALLBACK] measurement_log callback exception: {}", e.what());
        } catch (...) {
            ALOG_ERROR(MOD, "[GPS_CALLBACK] measurement_log callback unknown exception");
        }
    }

    bool need_sync_align = false;
    bool have_factor = false;
    double factor_ts = 0.0;
    Eigen::Vector3d factor_pos;
    Eigen::Matrix3d factor_cov;
    std::vector<GpsFactorCallback> factor_cbs;

    {
        std::lock_guard<std::recursive_mutex> lk(mutex_);

    // 统计连续高质量帧
    // 【修复】放宽质量要求：MEDIUM+ (HDOP≤5.0) 也计入good_sample，而非仅HIGH+ (HDOP≤2.0)
    // 原因：M2DGR等弱GPS场景HDOP≈10，仅要求HIGH会导致good_sample永远为0，对齐无法触发
    if (quality >= GPSQuality::MEDIUM) {
        good_sample_count_++;
        ALOG_DEBUG(MOD, "GPS good sample #{} hdop={:.2f} quality={} enu=({:.2f},{:.2f},{:.2f})",
                   good_sample_count_, hdop, static_cast<int>(quality),
                   pos_enu.x(), pos_enu.y(), pos_enu.z());
        // 诊断：接近或达到对齐触发阈值时打 INFO，便于分析为何未触发
        if (good_sample_count_ == 1 || good_sample_count_ == good_samples_needed_ ||
            (good_samples_needed_ > 5 && good_sample_count_ == (good_samples_needed_ * 5) / 10)) {
            ALOG_INFO(MOD, "[GPS_DIAG] good_sample_count={} (needed={}) hdop={:.2f} quality={} → align triggers when count>=needed and try_align passes dist/points",
                      good_sample_count_, good_samples_needed_, hdop, static_cast<int>(quality));
        }
    } else {
        ALOG_DEBUG(MOD, "GPS low quality: hdop={:.2f} quality={} sats={}",
                   hdop, static_cast<int>(quality), num_sats);
        good_sample_count_ = std::max(0, good_sample_count_ - 1);
        if (state_ == GPSAlignState::ALIGNED) {
            ALOG_WARN(MOD, "GPS signal degraded (hdop={:.2f}), pausing GPS constraints", hdop);
            ALOG_INFO(MOD, "[GPS_STATE] ALIGNED→DEGRADED reason=hdop_degraded hdop={:.2f}", hdop);
            state_ = GPSAlignState::DEGRADED;
        }
    }

    // 恢复：DEGRADED + GPS好转
    if (state_ == GPSAlignState::DEGRADED && good_sample_count_ >= good_samples_needed_ / 2) {
        ALOG_INFO(MOD, "GPS signal recovered, re-enabling constraints");
        ALOG_INFO(MOD, "[GPS_STATE] DEGRADED→ALIGNED reason=recovered good_samples={}", good_sample_count_);
        state_ = GPSAlignState::ALIGNED;
    }

    // 触发对齐判断：若设置了 align_scheduler_ 则异步调度，否则在锁外同步执行 try_align()，避免持锁执行回调导致死锁
    if (state_ == GPSAlignState::NOT_ALIGNED &&
        good_sample_count_ >= good_samples_needed_) {
        // 诊断：对齐触发条件满足（样本数）；try_align 内会再检查 kf 窗口与累积距离
        double acc_dist = 0.0;
        for (size_t i = 1; i < kf_window_.size(); ++i)
            acc_dist += (kf_window_[i].second.translation() - kf_window_[i-1].second.translation()).norm();
        ALOG_INFO(MOD, "[GPS_ALIGN] Trigger: good_sample_count={} (needed={}) kf_window={} gps_window={} accumulated_dist_m={:.1f} (min={:.1f}); try_align will run and check dist/points",
                  good_sample_count_, good_samples_needed_, kf_window_.size(), gps_window_.size(), acc_dist, min_align_dist_m_);
        pending_align_.store(true);
        if (align_scheduler_) {
            align_scheduler_();
        } else {
            need_sync_align = true;
        }
    }

    // 已对齐：发布 GPS 因子（协方差 = base_cov / (factor_weight * quality_scale)，质量越好约束越强）
    // 【修复】降低质量阈值：MEDIUM+ (HDOP≤5.0) 也发布GPS因子
    if (state_ == GPSAlignState::ALIGNED && quality >= GPSQuality::MEDIUM) {
        Eigen::Vector3d pos_map = enu_to_map(pos_enu);
        double hdop_safe = std::max(hdop, 0.1);
        double sigma_h = hdop_safe * 0.5;
        double sigma_v = hdop_safe * 1.0;
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        cov(0,0) = sigma_h * sigma_h;
        cov(1,1) = sigma_h * sigma_h;
        cov(2,2) = sigma_v * sigma_v;
        const auto& cfg = ConfigManager::instance();
        double weight = cfg.gpsFactorWeight();
        double quality_scale = 1.0;
        switch (quality) {
            case GPSQuality::EXCELLENT: quality_scale = cfg.gpsFactorQualityScaleExcellent(); break;
            case GPSQuality::HIGH:      quality_scale = cfg.gpsFactorQualityScaleHigh();      break;
            case GPSQuality::MEDIUM:    quality_scale = cfg.gpsFactorQualityScaleMedium();     break;
            default: break;
        }
        cov /= (weight * quality_scale);

        double dist = pos_map.norm();
        double interval = ConfigManager::instance().gpsFactorIntervalM();
        if (dist - last_gps_factor_dist_ >= interval) {
            last_gps_factor_dist_ = dist;
            aligned_gps_buffer_.push_back({timestamp, pos_map, cov});
            have_factor = true;
            factor_ts = timestamp;
            factor_pos = pos_map;
            factor_cov = cov;
            factor_cbs = gps_factor_cbs_;
        }
    }
    }  // end second lock scope

    if (need_sync_align) {
        pending_align_.store(false);
        try_align();
    }

    // 【异步友好】在锁外执行 gps_factor_cbs_，避免长时间持锁阻塞 ROS 回调
    if (have_factor) {
        for (auto& cb : factor_cbs) {
            try {
                cb(factor_ts, factor_pos, factor_cov);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "[GPS_CALLBACK] gps_factor callback exception: {}", e.what());
            } catch (...) {
                ALOG_ERROR(MOD, "[GPS_CALLBACK] gps_factor callback unknown exception");
            }
        }
    }
}

void GPSManager::addKeyFramePose(double timestamp, const Pose3d& T_odom_b) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    kf_window_.push_back({timestamp, T_odom_b});
    const size_t max_kf_window = ConfigManager::instance().gpsMaxWindowSize();
    while (kf_window_.size() > max_kf_window) kf_window_.pop_front();
    
    // 【修复】同时缓存所有里程计位姿用于精準插值
    all_odom_poses_.push_back({timestamp, T_odom_b});
    if (all_odom_poses_.size() > 5000) all_odom_poses_.pop_front();
}

void GPSManager::runScheduledAlignment() {
    // 不持锁调用 try_align，避免 try_align -> on_aligned -> align_cbs_ -> getHistoricalGPSBindings 重入锁死锁
    if (!pending_align_.exchange(false)) return;
    ALOG_INFO(MOD, "[GPS_ALIGN] runScheduledAlignment enter (pending_align was true, calling try_align)");
    auto t0 = std::chrono::steady_clock::now();
    try_align();
    double ms = 1e-3 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0).count();
    ALOG_INFO(MOD, "[GPS_ALIGN] runScheduledAlignment exit duration_ms={:.1f}", ms);
}

void GPSManager::try_align() {
    std::unique_lock<std::recursive_mutex> lk(mutex_);

    const size_t kf_sz = kf_window_.size();
    const size_t gps_sz = gps_window_.size();
    if (kf_window_.empty() || gps_window_.empty()) {
        ALOG_INFO(MOD, "[GPS_ALIGN] try_align skip: kf_window={} gps_window={} reason=empty (need both non-empty)",
                  kf_sz, gps_sz);
        return;
    }

    double total_dist = 0.0;
    for (size_t i = 1; i < kf_window_.size(); ++i) {
        total_dist += (kf_window_[i].second.translation() -
                       kf_window_[i-1].second.translation()).norm();
    }
    if (total_dist < min_align_dist_m_) {
        ALOG_INFO(MOD, "[GPS_ALIGN] try_align skip: total_dist_m={:.1f} min_required={:.1f} kf_window={} reason=dist_too_short (need more keyframes/distance)",
                  total_dist, min_align_dist_m_, kf_sz);
        return;
    }

    size_t good_gps_count = 0;
    for (const auto& g : gps_window_) {
        if (g.quality >= GPSQuality::MEDIUM) good_gps_count++;
    }
    if ((int)good_gps_count < min_align_points_) {
        ALOG_INFO(MOD, "[GPS_ALIGN] try_align skip: good_gps_count={} min_required={} total_dist_m={:.1f} reason=insufficient_points (need more MEDIUM+ GPS in window)",
                  good_gps_count, min_align_points_, total_dist);
        return;
    }

    ALOG_INFO(MOD, "[GPS_STATE] NOT_ALIGNED→ALIGNING reason=sufficient_samples good_gps={} dist={:.1f}m",
              good_gps_count, total_dist);
    state_ = GPSAlignState::ALIGNING;
    GPSAlignResult result = compute_svd_alignment();

    if (result.success && result.rmse_m < rmse_accept_thresh_) {
        double rot_deg = std::atan2(result.R_gps_lidar(1, 0), result.R_gps_lidar(0, 0)) * 180.0 / M_PI;
        ALOG_INFO(MOD, "GPS alignment SUCCESS: rmse={:.3f}m matched={} pts R_z_deg={:.2f} t=({:.2f},{:.2f},{:.2f})",
                  result.rmse_m, result.matched_points, rot_deg,
                  result.t_gps_lidar.x(), result.t_gps_lidar.y(), result.t_gps_lidar.z());
        align_result_ = result;
        state_ = GPSAlignState::ALIGNED;
        std::vector<AlignCallback> cbs = align_cbs_;
        GPSAlignResult res_copy = result;
        const size_t num_cbs = cbs.size();
        lk.unlock();
        ALOG_INFO(MOD, "[GPS_ALIGN] try_align callbacks_enter num_callbacks={} (lock released)", num_cbs);
        for (auto& cb : cbs) {
            try {
                cb(res_copy);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "[GPS_CALLBACK] align callback exception: {}", e.what());
            } catch (...) {
                ALOG_ERROR(MOD, "[GPS_CALLBACK] align callback unknown exception");
            }
        }
        ALOG_INFO(MOD, "[GPS_ALIGN] try_align callbacks_done");
    } else {
        ALOG_WARN(MOD, "GPS alignment FAILED: success={} rmse={:.3f}m > thresh={:.3f}m matched={}",
                  result.success, result.rmse_m, rmse_accept_thresh_, result.matched_points);
        ALOG_INFO(MOD, "[GPS_STATE] ALIGNING→NOT_ALIGNED reason=svd_failed rmse={:.3f}m thresh={:.3f}m",
                  result.rmse_m, rmse_accept_thresh_);
        state_ = GPSAlignState::NOT_ALIGNED;
        good_sample_count_ = 0;
        result.message = "RMSE too high: " + std::to_string(result.rmse_m) + "m > " + std::to_string(rmse_accept_thresh_) + "m";
    }
}

// 最大允许的“最近邻”时间差（秒），用于无前后双关键帧时的回退
static constexpr double kMaxNearestDtSec = 0.1;
// 最大允许的插值区间（秒），超过则退化为最近邻
static constexpr double kMaxInterpIntervalSec = 0.5;

/**
 * SVD 轨迹匹配算法（参考 HBA gps_factor.hpp 的 path_match 实现）
 *
 * 方法：对 GPS ENU 轨迹和 LiDAR 轨迹进行点集配准
 *   1. 对每个高质量 GPS 时间戳，找「前、后」关键帧，在二者间线性插值得到 LiDAR 位置；
 *      若无双侧关键帧或插值区间过大，则退化为 100ms 内最近邻。
 *   2. 中心化 → 协方差矩阵 → SVD → R, t
 */
GPSAlignResult GPSManager::compute_svd_alignment() {
    GPSAlignResult result;

    // 找时间对齐的 (GPS_pos, LiDAR_pos) 对：前后关键帧 + 线性插值
    struct MatchPair { Eigen::Vector3d gps_enu; Eigen::Vector3d lidar_pos; };
    std::vector<MatchPair> pairs;

    if (kf_window_.empty()) return result;

    for (const auto& gr : gps_window_) {
        // 【修复】放宽质量要求：MEDIUM+ (HDOP≤5.0) 也参与SVD对齐
        // 原因：弱GPS场景下HIGH+(HDOP≤2.0)可能为空，导致对齐失败
        if (gr.quality < GPSQuality::MEDIUM) continue;

        // 找前关键帧：kt <= gr.timestamp 中最大的
        const std::pair<double, Pose3d>* kf_prev = nullptr;
        const std::pair<double, Pose3d>* kf_next = nullptr;
        for (const auto& kv : kf_window_) {
            if (kv.first <= gr.timestamp) kf_prev = &kv;
            if (kv.first >= gr.timestamp) { kf_next = &kv; break; }
        }

        Eigen::Vector3d lidar_pos;
        bool found = false;

        if (kf_prev && kf_next) {
            double t_prev = kf_prev->first;
            double t_next = kf_next->first;
            double interval = t_next - t_prev;
            if (interval <= 1e-9) {
                lidar_pos = kf_prev->second.translation();
                found = true;
            } else if (interval <= kMaxInterpIntervalSec) {
                // 线性插值：alpha in [0,1], pos = (1-alpha)*prev + alpha*next
                double alpha = (gr.timestamp - t_prev) / interval;
                alpha = std::max(0.0, std::min(1.0, alpha));
                lidar_pos = (1.0 - alpha) * kf_prev->second.translation()
                            + alpha * kf_next->second.translation();
                found = true;
            } else {
                // 区间过大，退化为 100ms 内最近邻
                double dt_prev = gr.timestamp - t_prev;
                double dt_next = t_next - gr.timestamp;
                if (dt_prev <= dt_next && dt_prev <= kMaxNearestDtSec) {
                    lidar_pos = kf_prev->second.translation();
                    found = true;
                } else if (dt_next <= kMaxNearestDtSec) {
                    lidar_pos = kf_next->second.translation();
                    found = true;
                }
            }
        } else if (kf_prev) {
            if (gr.timestamp - kf_prev->first <= kMaxNearestDtSec) {
                lidar_pos = kf_prev->second.translation();
                found = true;
            }
        } else if (kf_next) {
            if (kf_next->first - gr.timestamp <= kMaxNearestDtSec) {
                lidar_pos = kf_next->second.translation();
                found = true;
            }
        }

        if (found) {
            pairs.push_back({gr.pos_enu, lidar_pos});
        }
    }

    if ((int)pairs.size() < min_align_points_) {
        ALOG_INFO(MOD, "[GPS_ALIGN] SVD skip: time_matched_pairs={} min_required={} (GPS timestamps not close enough to keyframes; check keyframe_match_window_s / GPS rate)",
                  static_cast<int>(pairs.size()), min_align_points_);
        return result;
    }

    // 中心化
    Eigen::Vector3d gps_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d lio_centroid = Eigen::Vector3d::Zero();
    for (const auto& p : pairs) {
        gps_centroid += p.gps_enu;
        lio_centroid += p.lidar_pos;
    }
    gps_centroid /= pairs.size();
    lio_centroid /= pairs.size();

    // 2D 协方差（仅 XY），避免 3x3 时 z 恒为 0 导致第三奇异值=0、cond=inf
    Eigen::Matrix2d cov_xy = Eigen::Matrix2d::Zero();
    for (const auto& p : pairs) {
        Eigen::Vector2d gd(p.gps_enu.x() - gps_centroid.x(), p.gps_enu.y() - gps_centroid.y());
        Eigen::Vector2d ld(p.lidar_pos.x() - lio_centroid.x(), p.lidar_pos.y() - lio_centroid.y());
        cov_xy += ld * gd.transpose();  // LiDAR←GPS
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd_xy(cov_xy, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d singular_vals = svd_xy.singularValues();
    const double min_sv = singular_vals.minCoeff();
    const double max_sv = singular_vals.maxCoeff();
    const double cond = (max_sv > 1e-12) ? (max_sv / min_sv) : 1e12;
    if (min_sv < 1e-6 || cond > 1e8) {
        ALOG_WARN(MOD, "GPS alignment: degenerate XY covariance (cond={:.2e}), rejecting", cond);
        return result;
    }

    Eigen::Matrix2d R_xy = svd_xy.matrixU() * svd_xy.matrixV().transpose();
    if (R_xy.determinant() < 0) {
        Eigen::Matrix2d diag = Eigen::Matrix2d::Identity();
        diag(1, 1) = -1;
        R_xy = svd_xy.matrixU() * diag * svd_xy.matrixV().transpose();
    }

    // 扩展为 3x3：绕 Z 轴旋转，R(2,2)=1
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R.block<2,2>(0,0) = R_xy;

    Eigen::Vector3d t = lio_centroid - R * gps_centroid;

    // 🔧 [修复] 修正 Z 轴平移：2D SVD 只处理了 XY 平面，这里手动计算 Z 轴平均偏差作为平移的一部分
    double z_sum = 0.0;
    int z_count = 0;
    for (const auto& p : pairs) {
        // LiDAR Z - (R_enu_to_map * GPS_ENU)_z
        // 注意 R 只是绕 Z 旋转，所以 (R*gps).z == gps.z
        z_sum += (p.lidar_pos.z() - p.gps_enu.z());
        z_count++;
    }
    if (z_count > 0) {
        t.z() = z_sum / z_count;
    }

    // 计算 RMSE（仅在XY平面，GPS高度不可靠）
    double rmse = 0.0;
    for (const auto& p : pairs) {
        Eigen::Vector3d pred = R * p.gps_enu + t;
        pred.z() = p.lidar_pos.z();  // 高度用LiDAR的
        Eigen::Vector3d diff = pred - p.lidar_pos;
        diff.z() = 0.0;  // 仅计算XY平面误差
        rmse += diff.squaredNorm();
    }
    rmse = std::sqrt(rmse / pairs.size());

    result.success       = true;
    result.R_gps_lidar   = R;
    result.t_gps_lidar   = t;
    result.R_enu_to_map  = R;   // R_gps_lidar就是ENU到Map的旋转
    result.t_enu_to_map  = t;   // t_gps_lidar就是ENU到Map的平移
    result.rmse_m        = rmse;
    result.matched_points = (int)pairs.size();
    result.used_measurements = pairs.size();
    result.message = "SVD alignment successful";
    return result;
}

void GPSManager::on_aligned(const GPSAlignResult& result) {
    ALOG_INFO(MOD, "[GPS_STATE] ALIGNING→ALIGNED reason=svd_ok rmse={:.3f}m matched={}",
              result.rmse_m, result.matched_points);
    state_ = GPSAlignState::ALIGNED;
    align_result_ = result;
    for (auto& cb : align_cbs_) cb(result);
}

void GPSManager::resetAlignmentToIdentity() {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    align_result_.R_gps_lidar = Eigen::Matrix3d::Identity();
    align_result_.t_gps_lidar = Eigen::Vector3d::Zero();
    align_result_.R_enu_to_map = Eigen::Matrix3d::Identity();
    align_result_.t_enu_to_map = Eigen::Vector3d::Zero();
    // 保持 state_ 为 ALIGNED，确保后续查询走 enu_to_map 返回 map (即 ENU)
    ALOG_INFO(MOD, "[GPS_ALIGN] Reset alignment to identity (system is now globalized)");
}

Eigen::Vector3d GPSManager::wgs84_to_enu(double lat, double lon, double alt) const {
    // ✅ 修复：检查ENU原点是否已设置，避免使用未初始化的原点坐标
    if (!enu_origin_set_.load(std::memory_order_acquire)) {
        ALOG_WARN(MOD, "ENU origin not set yet, returning zero vector for GPS (lat=%.6f, lon=%.6f)", lat, lon);
        return Eigen::Vector3d::Zero();
    }
    
    GeographicLib::LocalCartesian proj(enu_origin_lat_, enu_origin_lon_, enu_origin_alt_);
    double e, n, u;
    try {
        proj.Forward(lat, lon, alt, e, n, u);
    } catch (const GeographicLib::GeographicErr& e) {
        ALOG_ERROR(MOD, "GeographicLib conversion failed: %s (lat=%.6f, lon=%.6f, alt=%.2f)", e.what(), lat, lon, alt);
        return Eigen::Vector3d::Zero();
    } catch (...) {
        ALOG_ERROR(MOD, "Unknown GeographicLib conversion error (lat=%.6f, lon=%.6f, alt=%.2f)", lat, lon, alt);
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(e, n, u);
}

Eigen::Vector3d GPSManager::enu_to_map(const Eigen::Vector3d& enu) const {
    if (state_ != GPSAlignState::ALIGNED && state_ != GPSAlignState::DEGRADED)
        return enu;
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return align_result_.R_gps_lidar * enu + align_result_.t_gps_lidar;
}

Eigen::Vector3d GPSManager::map_to_enu(const Eigen::Vector3d& map_pos) const {
    if (state_ != GPSAlignState::ALIGNED && state_ != GPSAlignState::DEGRADED)
        return map_pos;
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return align_result_.R_gps_lidar.transpose() * (map_pos - align_result_.t_gps_lidar);
}

std::pair<Eigen::Vector3d, std::string> GPSManager::enu_to_map_with_frame(const Eigen::Vector3d& enu) const {
    if (state_ != GPSAlignState::ALIGNED && state_ != GPSAlignState::DEGRADED) {
        return {enu, "enu"};
    }
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    Eigen::Vector3d pos_map = align_result_.R_gps_lidar * enu + align_result_.t_gps_lidar;
    
    static std::atomic<uint32_t> query_log_count{0};
    if (query_log_count.fetch_add(1) % 500 == 0) {
        ALOG_INFO(MOD, "[GPS_QUERY_DIAG] Frame query: enu=({:.2f},{:.2f},{:.2f}) -> map=({:.2f},{:.2f},{:.2f}) frame=map",
                  enu.x(), enu.y(), enu.z(), pos_map.x(), pos_map.y(), pos_map.z());
    }
    
    return {pos_map, "map"};
}

int GPSManager::getGoodSampleCount() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGoodSampleCount enter");
    GpsWindowSnapshot snap = getSnapshot();
    int n = snap.good_sample_count;
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGoodSampleCount exit good_sample_count={}", n);
    return n;
}

double GPSManager::getAccumulatedDistM() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    double total_dist = 0.0;
    for (size_t i = 1; i < kf_window_.size(); ++i) {
        total_dist += (kf_window_[i].second.translation() - kf_window_[i - 1].second.translation()).norm();
    }
    return total_dist;
}

std::vector<std::pair<double, Eigen::Vector3d>> GPSManager::getGpsPositionsInMapFrame() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsPositionsInMapFrame enter");
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsPositionsInMapFrame got_snapshot window_size={} is_aligned={}", snap.window.size(), snap.is_aligned ? 1 : 0);
    std::vector<std::pair<double, Eigen::Vector3d>> out;
    if (!snap.is_aligned) {
        ALOG_INFO(MOD, "[GPS_QUERY_DIAG] getGpsPositionsInMapFrame exit reason=not_aligned out_size=0");
        return out;
    }
    out.reserve(snap.window.size());
    for (const auto& r : snap.window)
        out.emplace_back(r.timestamp, enu_to_map_from_snapshot(r.pos_enu, snap));
    ALOG_INFO(MOD, "[GPS_QUERY_DIAG] getGpsPositionsInMapFrame exit out_size={} (transformed to map frame)", out.size());
    return out;
}

GPSQuality GPSManager::hdop_to_quality(double hdop) const {
    // 使用配置文件中的 quality_threshold_hdop 值，而非硬编码
    // 原阈值：EXCELLENT≤1, HIGH≤2, MEDIUM≤5, LOW≤20
    // 适配弱GPS场景（如M2DGR HDOP≈10）：MEDIUM≤quality_hdop_thresh_（默认12.0）
    if (hdop <= 1.0) return GPSQuality::EXCELLENT;
    if (hdop <= 2.0) return GPSQuality::HIGH;
    if (hdop <= quality_hdop_thresh_) return GPSQuality::MEDIUM;
    if (hdop <= 20.0) return GPSQuality::LOW;
    return GPSQuality::INVALID;
}

GPSAlignState GPSManager::state() const {
    return state_.load();
}

GPSQuality GPSManager::currentQuality() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] currentQuality enter");
    GpsWindowSnapshot snap = getSnapshot();
    GPSQuality q = snap.window.empty() ? GPSQuality::INVALID : snap.window.back().quality;
    ALOG_DEBUG(MOD, "[GPS_QUERY] currentQuality exit empty={} quality={}", snap.window.empty() ? 1 : 0, static_cast<int>(q));
    return q;
}

std::optional<Eigen::Vector3d> GPSManager::getLatestPositionENU() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getLatestPositionENU enter");
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] getLatestPositionENU got_snapshot window_size={} is_aligned={}", snap.window.size(), snap.is_aligned ? 1 : 0);
    for (auto it = snap.window.rbegin(); it != snap.window.rend(); ++it) {
        if (it->quality >= GPSQuality::MEDIUM) {
            Eigen::Vector3d pos = snap.is_aligned ? enu_to_map_from_snapshot(it->pos_enu, snap) : it->pos_enu;
            ALOG_DEBUG(MOD, "[GPS_QUERY] getLatestPositionENU exit found quality={} ts={:.3f}", static_cast<int>(it->quality), it->timestamp);
            return pos;
        }
    }
    ALOG_DEBUG(MOD, "[GPS_QUERY] getLatestPositionENU exit not_found (no MEDIUM+ in window)");
    return std::nullopt;
}

GPSManager::GpsWindowSnapshot GPSManager::getSnapshot() const {
    ALOG_DEBUG(MOD, "[GPS_SNAPSHOT] getSnapshot enter (about to lock)");
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    ALOG_DEBUG(MOD, "[GPS_SNAPSHOT] getSnapshot lock_held copying window size={}", gps_window_.size());
    GpsWindowSnapshot snap;
    snap.window.reserve(gps_window_.size());
    for (const auto& r : gps_window_) {
        GpsWindowSnapshot::Record rec;
        rec.timestamp = r.timestamp;
        rec.pos_enu = r.pos_enu;
        rec.quality = r.quality;
        rec.hdop = r.hdop;
        snap.window.push_back(rec);
    }
    snap.is_aligned = (state_ == GPSAlignState::ALIGNED || state_ == GPSAlignState::DEGRADED);
    snap.R_gps_lidar = align_result_.R_gps_lidar;
    snap.t_gps_lidar = align_result_.t_gps_lidar;
    snap.keyframe_match_window_s = keyframe_match_window_s_;
    snap.max_interp_gap_s = max_interp_gap_s_;
    snap.extrapolation_margin_s = extrapolation_margin_s_;
    snap.extrapolation_uncertainty_scale = extrapolation_uncertainty_scale_;
    snap.velocity_estimation_window_s = velocity_estimation_window_s_;
    snap.keyframe_max_hdop = keyframe_max_hdop_;
    snap.good_sample_count = good_sample_count_;
    ALOG_DEBUG(MOD, "[GPS_SNAPSHOT] getSnapshot exit (lock released) window_size={} is_aligned={} good_sample_count={}", snap.window.size(), snap.is_aligned ? 1 : 0, snap.good_sample_count);
    ALOG_INFO(MOD, "[GPS_SNAPSHOT] getSnapshot done window_size={} is_aligned={} (grep GPS_SNAPSHOT 可定位卡在等锁或拷贝)", snap.window.size(), snap.is_aligned ? 1 : 0);
    return snap;
}

std::optional<GPSMeasurement> GPSManager::queryByTimestamp(double ts) const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestamp enter ts={:.3f}", ts);
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestamp got_snapshot window_size={} is_aligned={}", snap.window.size(), snap.is_aligned ? 1 : 0);
    auto m_opt = queryByTimestampOnSnapshot(ts, snap);
    if (!m_opt && !snap.window.empty()) {
        double win_min = snap.window.front().timestamp, win_max = win_min;
        for (const auto& r : snap.window) {
            if (r.timestamp < win_min) win_min = r.timestamp;
            if (r.timestamp > win_max) win_max = r.timestamp;
        }
        ALOG_WARN(MOD, "[GPS_QUERY] queryByTimestamp result=not_found ts={:.3f} window_size={} max_dt_s={:.2f} window_ts=[{:.3f},{:.3f}]",
                  ts, snap.window.size(), snap.keyframe_match_window_s, win_min, win_max);
    }
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestamp exit ts={:.3f} has_result={}", ts, m_opt ? 1 : 0);
    return m_opt;
}


std::optional<GPSMeasurement> GPSManager::queryByNearestPosition(const Eigen::Vector3d& position_map) const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition enter position_map=({:.2f},{:.2f},{:.2f})", position_map.x(), position_map.y(), position_map.z());
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition got_snapshot window_size={} is_aligned={}", snap.window.size(), snap.is_aligned ? 1 : 0);
    if (!snap.is_aligned) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition exit reason=not_aligned (fallback to time-based)");
        return std::nullopt;
    }
    if (snap.window.empty()) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition exit reason=window_empty");
        return std::nullopt;
    }
    double best_sq = std::numeric_limits<double>::max();
    double nearest_ts = 0.0;
    for (const auto& r : snap.window) {
        Eigen::Vector3d pos_map = enu_to_map_from_snapshot(r.pos_enu, snap);
        double sq = (pos_map - position_map).squaredNorm();
        if (sq < best_sq) {
            best_sq = sq;
            nearest_ts = r.timestamp;
        }
    }
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition nearest_ts={:.3f} best_sq={:.4f}", nearest_ts, best_sq);
    auto m_opt = queryByTimestampOnSnapshot(nearest_ts, snap);
    if (m_opt) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition exit found position_map=({:.2f},{:.2f},{:.2f}) nearest_ts={:.3f}", position_map.x(), position_map.y(), position_map.z(), nearest_ts);
    } else {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByNearestPosition exit not_found nearest_ts={:.3f}", nearest_ts);
    }
    return m_opt;
}

std::optional<GPSMeasurement> GPSManager::queryByTimestampForLog(double ts, double max_dt_s) const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampForLog enter ts={:.3f} max_dt_s={:.3f}", ts, max_dt_s);
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampForLog got_snapshot window_size={}", snap.window.size());
    double best_dt = max_dt_s + 1.0;
    const GpsWindowSnapshot::Record* best = nullptr;
    for (const auto& r : snap.window) {
        double dt = std::abs(r.timestamp - ts);
        if (dt <= max_dt_s && (best == nullptr || dt < best_dt)) {
            best_dt = dt;
            best = &r;
        }
    }
    if (!best) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampForLog exit not_found ts={:.3f}", ts);
        return std::nullopt;
    }
    GPSMeasurement m;
    m.timestamp    = best->timestamp;
    m.position_enu = best->pos_enu;
    m.quality      = best->quality;
    m.hdop         = best->hdop;
    m.is_valid     = (best->quality >= GPSQuality::MEDIUM);
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampForLog exit found ts={:.3f} best_dt={:.3f} gps_ts={:.3f} is_valid={}", ts, best_dt, best->timestamp, m.is_valid ? 1 : 0);
    return m;
}

std::optional<Eigen::Vector3d> GPSManager::estimateGpsVelocityLocked(double ts) const {
    const double half = velocity_estimation_window_s_ * 0.5;
    std::vector<std::pair<double, Eigen::Vector3d>> pts;
    for (const auto& r : gps_window_) {
        if (r.timestamp >= ts - half && r.timestamp <= ts + half && r.quality >= GPSQuality::MEDIUM) {
            pts.push_back({r.timestamp, isAligned() ? enu_to_map(r.pos_enu) : r.pos_enu});
        }
    }
    if (pts.size() < 2) return std::nullopt;
    std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
    Eigen::Vector3d sum_vel = Eigen::Vector3d::Zero();
    double sum_dt = 0.0;
    for (size_t i = 1; i < pts.size(); ++i) {
        double dt = pts[i].first - pts[i - 1].first;
        if (dt > 0.01 && dt < velocity_estimation_window_s_) {
            sum_vel += (pts[i].second - pts[i - 1].second) / dt;
            sum_dt += 1.0;
        }
    }
    if (sum_dt < 0.5) return std::nullopt;
    return sum_vel / sum_dt;
}

std::optional<Eigen::Vector3d> GPSManager::estimateGpsVelocity(double ts) const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] estimateGpsVelocity enter ts={:.3f}", ts);
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] estimateGpsVelocity got_snapshot window_size={}", snap.window.size());
    auto vel_opt = estimateVelocityOnSnapshot(ts, snap);
    ALOG_DEBUG(MOD, "[GPS_QUERY] estimateGpsVelocity exit ts={:.3f} has_vel={}", ts, vel_opt ? 1 : 0);
    return vel_opt;
}

std::optional<GPSMeasurement> GPSManager::queryByTimestampEnhanced(double ts) const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced enter ts={:.3f}", ts);
    GpsWindowSnapshot snap = getSnapshot();
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced got_snapshot window_size={} is_aligned={}", snap.window.size(), snap.is_aligned ? 1 : 0);
    auto m_opt = queryByTimestampOnSnapshot(ts, snap);
    if (m_opt) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced exit ts={:.3f} mode=interp_or_nearest", ts);
        return m_opt;
    }

    if (snap.window.size() < 2) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced exit ts={:.3f} reason=window_size<2 size={}", ts, snap.window.size());
        return std::nullopt;
    }

    double win_min = snap.window.front().timestamp, win_max = win_min;
    for (const auto& r : snap.window) {
        if (r.timestamp < win_min) win_min = r.timestamp;
        if (r.timestamp > win_max) win_max = r.timestamp;
    }
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced win_ts=[{:.3f},{:.3f}] margin={:.3f}", win_min, win_max, snap.extrapolation_margin_s);
    const double margin = snap.extrapolation_margin_s;
    const GpsWindowSnapshot::Record* anchor = nullptr;
    double dt_extrap = 0.0;
    if (ts < win_min && (win_min - ts) <= margin) {
        anchor = &snap.window.front();
        dt_extrap = ts - anchor->timestamp;
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced extrapolate_before anchor_ts={:.3f} dt_extrap={:.3f}", anchor->timestamp, dt_extrap);
    } else if (ts > win_max && (ts - win_max) <= margin) {
        anchor = &snap.window.back();
        dt_extrap = ts - anchor->timestamp;
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced extrapolate_after anchor_ts={:.3f} dt_extrap={:.3f}", anchor->timestamp, dt_extrap);
    } else {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced exit ts={:.3f} reason=ts_outside_margin win=[{:.3f},{:.3f}]", ts, win_min, win_max);
        return std::nullopt;
    }

    auto vel_opt = estimateVelocityOnSnapshot(anchor->timestamp, snap);
    if (!vel_opt) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced exit ts={:.3f} reason=vel_est_failed anchor_ts={:.3f}", ts, anchor->timestamp);
        return std::nullopt;
    }

    Eigen::Vector3d pos_map = enu_to_map_from_snapshot(anchor->pos_enu, snap) + (*vel_opt) * dt_extrap;
    Eigen::Vector3d pos_enu = snap.is_aligned ? map_to_enu_from_snapshot(pos_map, snap) : pos_map;
    double hdop_safe = std::max(anchor->hdop, 0.1);
    double sigma_h = hdop_safe * 0.5 * std::sqrt(snap.extrapolation_uncertainty_scale);
    double sigma_v = hdop_safe * 1.0 * std::sqrt(snap.extrapolation_uncertainty_scale);

    GPSMeasurement m;
    m.timestamp    = ts;
    m.position_enu = pos_enu;
    m.quality      = anchor->quality;
    m.hdop         = anchor->hdop;
    m.is_valid     = (anchor->hdop <= snap.keyframe_max_hdop);
    m.covariance   = Eigen::Matrix3d::Zero();
    m.covariance(0, 0) = sigma_h * sigma_h;
    m.covariance(1, 1) = sigma_h * sigma_h;
    m.covariance(2, 2) = sigma_v * sigma_v;
    ALOG_DEBUG(MOD, "[GPS_QUERY] queryByTimestampEnhanced exit ts={:.3f} mode=extrapolate anchor_ts={:.3f} dt_extrap={:.3f} is_valid={}", ts, anchor->timestamp, dt_extrap, m.is_valid ? 1 : 0);
    return m;
}

size_t GPSManager::getGpsWindowSize() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsWindowSize enter");
    GpsWindowSnapshot snap = getSnapshot();
    size_t n = snap.window.size();
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsWindowSize exit size={}", n);
    return n;
}

bool GPSManager::getGpsWindowTimeRange(double* out_min_ts, double* out_max_ts) const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsWindowTimeRange enter");
    if (!out_min_ts || !out_max_ts) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsWindowTimeRange exit reason=null_ptr");
        return false;
    }
    GpsWindowSnapshot snap = getSnapshot();
    if (snap.window.empty()) {
        ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsWindowTimeRange exit reason=window_empty");
        return false;
    }
    double min_t = snap.window.front().timestamp, max_t = min_t;
    for (const auto& r : snap.window) {
        if (r.timestamp < min_t) min_t = r.timestamp;
        if (r.timestamp > max_t) max_t = r.timestamp;
    }
    *out_min_ts = min_t;
    *out_max_ts = max_t;
    ALOG_DEBUG(MOD, "[GPS_QUERY] getGpsWindowTimeRange exit ok min_ts={:.3f} max_ts={:.3f}", min_t, max_t);
    return true;
}

double GPSManager::getFirstGpsTimestamp() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getFirstGpsTimestamp enter");
    GpsWindowSnapshot snap = getSnapshot();
    double t = snap.window.empty() ? 0.0 : snap.window.front().timestamp;
    ALOG_DEBUG(MOD, "[GPS_QUERY] getFirstGpsTimestamp exit empty={} first_ts={:.3f}", snap.window.empty() ? 1 : 0, t);
    return t;
}

double GPSManager::getLastGpsTimestamp() const {
    ALOG_DEBUG(MOD, "[GPS_QUERY] getLastGpsTimestamp enter");
    GpsWindowSnapshot snap = getSnapshot();
    double t = snap.window.empty() ? 0.0 : snap.window.back().timestamp;
    ALOG_DEBUG(MOD, "[GPS_QUERY] getLastGpsTimestamp exit empty={} last_ts={:.3f}", snap.window.empty() ? 1 : 0, t);
    return t;
}

void GPSManager::triggerRealign() {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    ALOG_INFO(MOD, "[GPS_STATE] *→NOT_ALIGNED reason=trigger_realign");
    state_ = GPSAlignState::NOT_ALIGNED;
    good_sample_count_ = 0;
}

// ═────────────────────────────────────────────────────────────────────────────
// 精准对齐增强方法实现
// ═────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d GPSManager::estimateGpsPositionByOdom(
    double gps_ts, double odom_start_ts, const Pose3d& odom_start_pose) {
    /* 使用里程计积分来估计GPS在里程计起始时刻的位置
     * 基于里程计速度积分：Δpos = v * Δt
     * 估算速度（使用最近几个里程计位姿） */
    Eigen::Vector3d avg_velocity = Eigen::Vector3d::Zero();
    {
        std::lock_guard<std::recursive_mutex> lk(mutex_);
        if (kf_window_.size() < 2) {
            return Eigen::Vector3d::Zero();
        }
        /* 计算平均速度 */
        int count = 0;
        Eigen::Vector3d sum_vel = Eigen::Vector3d::Zero();
        for (size_t i = 1; i < kf_window_.size(); ++i) {
            double dt = kf_window_[i].first - kf_window_[i-1].first;
            if (dt > 0.001 && dt < 1.0) {
                Eigen::Vector3d vel = (kf_window_[i].second.translation() -
                                            kf_window_[i-1].second.translation()) / dt;
                sum_vel += vel;
                count++;
            }
        }
        if (count > 0) {
            avg_velocity = sum_vel / count;
        }
    }

    /* 估算GPS位置 */
    double dt = gps_ts - odom_start_ts;
    Eigen::Vector3d estimated_pos = odom_start_pose.translation() + avg_velocity * dt;

    ALOG_DEBUG(MOD, "[GPS_ALIGN] Estimated GPS position at ts={:.3f}: pos=({:.2f},{:.2f},{:.2f}) by odom integration (dt={:.2f}s)",
              gps_ts, estimated_pos.x(), estimated_pos.y(), estimated_pos.z(), dt);
    return estimated_pos;
}

void GPSManager::onlineCalibrate(
    double /*gps_ts*/, const Eigen::Vector3d& gps_enu,
    double /*odom_ts*/, const Pose3d& odom_pose) {
    /* 在线校准：gps_enu 与 odom_pose.translation() 均应为 ENU，同系求差后累积偏差 */
    if (state_ != GPSAlignState::ALIGNED) return;
    Eigen::Vector3d offset = gps_enu - odom_pose.translation();
    double alpha = 0.05;
    accumulated_offset_ = accumulated_offset_ * (1.0 - alpha) + offset * alpha;
    int count = calib_count_.fetch_add(1) + 1;
    calib_count_.store(count);
    if (count % 10 == 0) {
        std::lock_guard<std::recursive_mutex> lk(mutex_);
        align_result_.t_gps_lidar += accumulated_offset_;
        ALOG_DEBUG(MOD, "[GPS_ALIGN] Online calibration: offset=({:.2f},{:.2f},{:.2f}) samples={}",
                  accumulated_offset_.x(), accumulated_offset_.y(), accumulated_offset_.z(), count);
    }
}

std::optional<std::pair<double, Pose3d>> GPSManager::findNearestOdomPose(double ts) const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);

    // 在缓存的所有里程计位姿中查找最近的
    const std::pair<double, Pose3d>* best = nullptr;
    double best_dt = std::numeric_limits<double>::max();

    // 先检查关键帧窗口
    for (const auto& kf : kf_window_) {
        double dt = std::abs(kf.first - ts);
        if (dt < best_dt) {
            best_dt = dt;
            best = &kf;
        }
    }

    // 再检查所有里程计位姿
    for (const auto& odom : all_odom_poses_) {
        double dt = std::abs(odom.first - ts);
        if (dt < best_dt) {
            best_dt = dt;
            best = &odom;
        }
    }

    if (best && best_dt < 0.5) {  // 500ms内的最近邻
        return *best;
    }
    return std::nullopt;
}

} // namespace automap_pro
