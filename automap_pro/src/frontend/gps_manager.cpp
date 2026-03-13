#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/map_frame_config.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/SVD>
#include <chrono>
#include <numeric>
#include <cmath>

#define MOD "GPSManager"

namespace automap_pro {

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
    ALOG_INFO(MOD, "[applyConfig] min_align_points={} min_align_dist_m={:.1f} keyframe_match_window_s={:.2f} good_samples_needed={} rmse_thresh={:.2f}m (align triggers when good_sample_count>=good_samples_needed and try_align passes dist/points)",
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
        std::lock_guard<std::mutex> lk(mutex_);

        pos_enu = wgs84_to_enu(latitude, longitude, altitude);
        quality = hdop_to_quality(hdop);

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
        std::lock_guard<std::mutex> lk(mutex_);

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

void GPSManager::addKeyFramePose(double timestamp, const Pose3d& T_w_b) {
    std::lock_guard<std::mutex> lk(mutex_);
    kf_window_.push_back({timestamp, T_w_b});
    const size_t max_kf_window = ConfigManager::instance().gpsMaxWindowSize();
    while (kf_window_.size() > max_kf_window) kf_window_.pop_front();
    
    // 【修复】同时缓存所有里程计位姿用于精準插值
    all_odom_poses_.push_back({timestamp, T_w_b});
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
    std::unique_lock<std::mutex> lk(mutex_);

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
    result.rmse_m        = rmse;
    result.matched_points = (int)pairs.size();
    return result;
}

void GPSManager::on_aligned(const GPSAlignResult& result) {
    ALOG_INFO(MOD, "[GPS_STATE] ALIGNING→ALIGNED reason=svd_ok rmse={:.3f}m matched={}",
              result.rmse_m, result.matched_points);
    state_ = GPSAlignState::ALIGNED;
    align_result_ = result;
    for (auto& cb : align_cbs_) cb(result);
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
    return align_result_.R_gps_lidar * enu + align_result_.t_gps_lidar;
}

std::pair<Eigen::Vector3d, std::string> GPSManager::enu_to_map_with_frame(const Eigen::Vector3d& enu) const {
    if (state_ != GPSAlignState::ALIGNED && state_ != GPSAlignState::DEGRADED)
        return {enu, "enu"};
    return {align_result_.R_gps_lidar * enu + align_result_.t_gps_lidar, "map"};
}

int GPSManager::getGoodSampleCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return good_sample_count_;
}

double GPSManager::getAccumulatedDistM() const {
    std::lock_guard<std::mutex> lk(mutex_);
    double total_dist = 0.0;
    for (size_t i = 1; i < kf_window_.size(); ++i) {
        total_dist += (kf_window_[i].second.translation() - kf_window_[i - 1].second.translation()).norm();
    }
    return total_dist;
}

std::vector<std::pair<double, Eigen::Vector3d>> GPSManager::getGpsPositionsInMapFrame() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<std::pair<double, Eigen::Vector3d>> out;
    if (state_ != GPSAlignState::ALIGNED && state_ != GPSAlignState::DEGRADED)
        return out;
    out.reserve(gps_window_.size());
    for (const auto& r : gps_window_)
        out.emplace_back(r.timestamp, enu_to_map(r.pos_enu));
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
    std::lock_guard<std::mutex> lk(mutex_);
    if (gps_window_.empty()) return GPSQuality::INVALID;
    return gps_window_.back().quality;
}

std::optional<Eigen::Vector3d> GPSManager::getLatestPositionENU() const {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto it = gps_window_.rbegin(); it != gps_window_.rend(); ++it) {
        if (it->quality >= GPSQuality::MEDIUM) {
            if (isAligned()) return enu_to_map(it->pos_enu);
            return it->pos_enu;
        }
    }
    return std::nullopt;
}

namespace {
    // 取两者中较差的 GPS 质量（用于插值结果的质量与 is_valid 判定）
    GPSQuality worseQuality(GPSQuality a, GPSQuality b) {
        return static_cast<int>(a) <= static_cast<int>(b) ? a : b;
    }
} // namespace

std::optional<GPSMeasurement> GPSManager::queryByTimestamp(double ts) const {
    std::lock_guard<std::mutex> lk(mutex_);
    const double max_dt = keyframe_match_window_s_;
    const double max_interp_gap_s = max_interp_gap_s_;

    const GPSRecord* r_before = nullptr;  // 最后一个 timestamp <= ts
    const GPSRecord* r_after  = nullptr;  // 第一个 timestamp >= ts
    for (const auto& r : gps_window_) {
        if (r.timestamp <= ts) r_before = &r;
        if (r.timestamp >= ts && r_after == nullptr) r_after = &r;
    }

    // 双样本线性插值：位置线性插值；姿态由里程计/前端提供，GPS 仅约束位置。
    // 若未来接入双天线 GNSS 朝向，可对四元数做球面插值(slerp)：q = slerp(q0, q1, alpha)。
    if (r_before && r_after && r_before != r_after) {
        const double t0 = r_before->timestamp;
        const double t1 = r_after->timestamp;
        if (t1 > t0 && (t1 - t0) <= max_interp_gap_s) {
            const double alpha = (ts - t0) / (t1 - t0);
            const Eigen::Vector3d p0 = isAligned() ? enu_to_map(r_before->pos_enu) : r_before->pos_enu;
            const Eigen::Vector3d p1 = isAligned() ? enu_to_map(r_after->pos_enu)  : r_after->pos_enu;
            GPSMeasurement m;
            m.timestamp    = ts;
            m.position_enu = (1.0 - alpha) * p0 + alpha * p1;
            m.hdop         = (1.0 - alpha) * r_before->hdop + alpha * r_after->hdop;
            m.quality     = worseQuality(r_before->quality, r_after->quality);
            // 【增强】使用 HDOP 阈值判定有效性，放宽 M2DGR 等弱 GPS 场景
            m.is_valid    = (m.hdop <= keyframe_max_hdop_);
            // 【诊断日志】
            ALOG_DEBUG(MOD, "[queryByTimestamp] ts={:.3f} mode=interp dt_before={:.3f} dt_after={:.3f} "
                      "interp_hdop={:.2f} interp_quality={} is_valid={} keyframe_max_hdop={:.1f}",
                      ts, ts - t0, t1 - ts, m.hdop, static_cast<int>(m.quality), m.is_valid, keyframe_max_hdop_);
            return m;
        }
    }

    // 单样本或间隔过大：退化为时间窗内最近邻
    double best_dt = max_dt + 1.0;
    const GPSRecord* best = nullptr;
    for (const auto& r : gps_window_) {
        double dt = std::abs(r.timestamp - ts);
        if (dt <= max_dt && (best == nullptr || dt < best_dt)) { best_dt = dt; best = &r; }
    }
    if (!best) {
        // 【精准分析】未命中时输出窗口时间范围，便于判断是“无数据”还是“ts 落在窗口外”
        double win_min = 0.0, win_max = 0.0;
        if (!gps_window_.empty()) {
            win_min = win_max = gps_window_.front().timestamp;
            for (const auto& r : gps_window_) {
                if (r.timestamp < win_min) win_min = r.timestamp;
                if (r.timestamp > win_max) win_max = r.timestamp;
            }
        }
        ALOG_WARN(MOD, "[queryByTimestamp] ts={:.3f} result=not_found window_size={} max_dt_s={:.2f} "
                  "window_ts=[{:.3f}, {:.3f}] (odom_ts outside GPS window or min_dt_to_nearest>max_dt; consider keyframe_match_window_s>={:.2f})",
                  ts, gps_window_.size(), max_dt, win_min, win_max, max_dt);
        return std::nullopt;
    }
    GPSMeasurement m;
    m.timestamp    = best->timestamp;
    m.position_enu = isAligned() ? enu_to_map(best->pos_enu) : best->pos_enu;
    m.quality      = best->quality;
    m.hdop         = best->hdop;
    // 【增强】使用 HDOP 阈值判定有效性，放宽 M2DGR 等弱 GPS 场景
    m.is_valid     = (best->hdop <= keyframe_max_hdop_);
    // 【诊断日志】
    ALOG_DEBUG(MOD, "[queryByTimestamp] ts={:.3f} mode=nearest dt={:.3f} gps_ts={:.3f} "
              "hdop={:.2f} quality={} is_valid={} keyframe_max_hdop={:.1f}",
              ts, best_dt, best->timestamp, best->hdop, static_cast<int>(best->quality), m.is_valid, keyframe_max_hdop_);
    return m;
}

std::optional<GPSMeasurement> GPSManager::queryByTimestampForLog(double ts, double max_dt_s) const {
    std::lock_guard<std::mutex> lk(mutex_);
    // 在 [0, max_dt_s] 内选最近的 GPS（含边界），避免 1Hz GPS + 10Hz odom 时边界 dt=0.5 漏匹配导致 CSV 每 5 行无 GPS
    double best_dt = max_dt_s + 1.0;
    const GPSRecord* best = nullptr;
    for (const auto& r : gps_window_) {
        double dt = std::abs(r.timestamp - ts);
        if (dt <= max_dt_s && (best == nullptr || dt < best_dt)) {
            best_dt = dt;
            best = &r;
        }
    }
    if (!best) return std::nullopt;
    GPSMeasurement m;
    m.timestamp    = best->timestamp;
    m.position_enu = isAligned() ? enu_to_map(best->pos_enu) : best->pos_enu;
    m.quality      = best->quality;
    m.hdop         = best->hdop;
    m.is_valid     = (best->quality >= GPSQuality::MEDIUM);
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
    std::lock_guard<std::mutex> lk(mutex_);
    return estimateGpsVelocityLocked(ts);
}

std::optional<GPSMeasurement> GPSManager::queryByTimestampEnhanced(double ts) const {
    auto m_opt = queryByTimestamp(ts);
    if (m_opt) return m_opt;

    std::lock_guard<std::mutex> lk(mutex_);
    if (gps_window_.size() < 2) return std::nullopt;

    double win_min = gps_window_.front().timestamp;
    double win_max = win_min;
    for (const auto& r : gps_window_) {
        if (r.timestamp < win_min) win_min = r.timestamp;
        if (r.timestamp > win_max) win_max = r.timestamp;
    }
    const double margin = extrapolation_margin_s_;
    const GPSRecord* anchor = nullptr;
    double dt_extrap = 0.0;
    if (ts < win_min && (win_min - ts) <= margin) {
        anchor = &gps_window_.front();
        dt_extrap = ts - anchor->timestamp;
    } else if (ts > win_max && (ts - win_max) <= margin) {
        anchor = &gps_window_.back();
        dt_extrap = ts - anchor->timestamp;
    } else {
        return std::nullopt;
    }

    auto vel_opt = estimateGpsVelocityLocked(anchor->timestamp);
    if (!vel_opt) return std::nullopt;

    Eigen::Vector3d pos = (isAligned() ? enu_to_map(anchor->pos_enu) : anchor->pos_enu) + (*vel_opt) * dt_extrap;
    double hdop_safe = std::max(anchor->hdop, 0.1);
    double sigma_h = hdop_safe * 0.5 * std::sqrt(extrapolation_uncertainty_scale_);
    double sigma_v = hdop_safe * 1.0 * std::sqrt(extrapolation_uncertainty_scale_);

    GPSMeasurement m;
    m.timestamp    = ts;
    m.position_enu = pos;
    m.quality      = anchor->quality;
    m.hdop         = anchor->hdop;
    m.is_valid     = (anchor->hdop <= keyframe_max_hdop_);
    m.covariance   = Eigen::Matrix3d::Zero();
    m.covariance(0, 0) = sigma_h * sigma_h;
    m.covariance(1, 1) = sigma_h * sigma_h;
    m.covariance(2, 2) = sigma_v * sigma_v;
    ALOG_DEBUG(MOD, "[queryByTimestampEnhanced] ts={:.3f} mode=extrapolate dt={:.3f} anchor_ts={:.3f}",
              ts, dt_extrap, anchor->timestamp);
    return m;
}

size_t GPSManager::getGpsWindowSize() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return gps_window_.size();
}

bool GPSManager::getGpsWindowTimeRange(double* out_min_ts, double* out_max_ts) const {
    if (!out_min_ts || !out_max_ts) return false;
    std::lock_guard<std::mutex> lk(mutex_);
    if (gps_window_.empty()) return false;
    double min_t = gps_window_.front().timestamp;
    double max_t = min_t;
    for (const auto& r : gps_window_) {
        if (r.timestamp < min_t) min_t = r.timestamp;
        if (r.timestamp > max_t) max_t = r.timestamp;
    }
    *out_min_ts = min_t;
    *out_max_ts = max_t;
    return true;
}

double GPSManager::getFirstGpsTimestamp() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (gps_window_.empty()) return 0.0;
    return gps_window_.front().timestamp;
}

double GPSManager::getLastGpsTimestamp() const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (gps_window_.empty()) return 0.0;
    return gps_window_.back().timestamp;
}

void GPSManager::triggerRealign() {
    std::lock_guard<std::mutex> lk(mutex_);
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
        std::lock_guard<std::mutex> lk(mutex_);
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
    /* 在线校准：持续校准GPS-里程计偏差 */
    if (state_ != GPSAlignState::ALIGNED) {
        return;  // 仅在已对齐状态下进行在线校准
    }
    /* 计算GPS-里程计偏差 */
    Eigen::Vector3d odom_enu = enu_to_map(odom_pose.translation());
    Eigen::Vector3d offset = gps_enu - odom_enu;
    /* 使用指数移动平均滤波偏差 */
    double alpha = 0.05;  // 平滑因子
    accumulated_offset_ = accumulated_offset_ * (1.0 - alpha) + offset * alpha;
    int count = calib_count_.fetch_add(1) + 1;
    calib_count_.store(count);
    /* 定期更新对齐结果 */
    if (count % 10 == 0) {
        /* 应用累积偏差到对齐变换 */
        align_result_.t_gps_lidar += accumulated_offset_;
        ALOG_DEBUG(MOD, "[GPS_ALIGN] Online calibration: offset=({:.2f},{:.2f},{:.2f}) samples={}",
                  accumulated_offset_.x(), accumulated_offset_.y(), accumulated_offset_.z(), count);
    }
}

std::optional<std::pair<double, Pose3d>> GPSManager::findNearestOdomPose(double ts) const {
    std::lock_guard<std::mutex> lk(mutex_);

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
