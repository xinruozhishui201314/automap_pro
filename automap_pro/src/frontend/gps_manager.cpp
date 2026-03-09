#include "automap_pro/frontend/gps_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/map_frame_config.h"

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/SVD>
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
    quality_hdop_thresh_    = cfg.gpsQualityThreshold();
    rmse_accept_thresh_     = cfg.gpsAlignRmseThresh();
    good_samples_needed_    = cfg.gpsGoodSamplesNeeded();

    // 【修复】初始化精準对齐增强参数
    online_calib_min_dist_    = 5.0;   // 最小累积距离触发在线校正（米）
    online_calib_max_rmse_    = 2.0;   // 在线校正RMSE阈值（米）
    online_calib_min_samples_ = 5;     // 在线校正最小样本数
}

void GPSManager::addGPSMeasurement(
    double timestamp,
    double latitude, double longitude, double altitude,
    double hdop, int num_sats)
{
    // ENU 原点仅设置一次，避免多线程竞态；并写入 map_frame.cfg 供后端统一使用
    std::call_once(enu_origin_once_, [this, latitude, longitude, altitude]() {
        enu_origin_lat_ = latitude;
        enu_origin_lon_ = longitude;
        enu_origin_alt_ = altitude;
        enu_origin_set_.store(true);
        ALOG_INFO(MOD, "ENU origin set: lat={:.6f} lon={:.6f} alt={:.2f}",
                  latitude, longitude, altitude);
        std::string cfg_path = ConfigManager::instance().mapFrameConfigPath();
        MapFrameConfig::write(cfg_path, latitude, longitude, altitude);
    });

    std::lock_guard<std::mutex> lk(mutex_);

    Eigen::Vector3d pos_enu = wgs84_to_enu(latitude, longitude, altitude);
    GPSQuality quality = hdop_to_quality(hdop);

    // 填入滑动窗口
    GPSRecord rec;
    rec.timestamp = timestamp;
    rec.pos_enu   = pos_enu;
    rec.quality   = quality;
    rec.hdop      = hdop;
    gps_window_.push_back(rec);
    if (gps_window_.size() > 2000) gps_window_.pop_front();

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

    // 轨迹对比：每条 GPS 测量通知外部落盘（用于与 odom 曲线对比分析建图精度）
    for (auto& cb : measurement_log_cbs_) cb(timestamp, pos_enu);

    // 统计连续高质量帧
    // 【修复】放宽质量要求：MEDIUM+ (HDOP≤5.0) 也计入good_sample，而非仅HIGH+ (HDOP≤2.0)
    // 原因：M2DGR等弱GPS场景HDOP≈10，仅要求HIGH会导致good_sample永远为0，对齐无法触发
    if (quality >= GPSQuality::MEDIUM) {
        good_sample_count_++;
        ALOG_DEBUG(MOD, "GPS good sample #{} hdop={:.2f} quality={} enu=({:.2f},{:.2f},{:.2f})",
                   good_sample_count_, hdop, static_cast<int>(quality),
                   pos_enu.x(), pos_enu.y(), pos_enu.z());
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

    // 触发对齐判断
    if (state_ == GPSAlignState::NOT_ALIGNED &&
        good_sample_count_ >= good_samples_needed_) {
        ALOG_INFO(MOD, "Sufficient good GPS samples ({}), triggering alignment...",
                  good_sample_count_);
        try_align();
    }

    // 已对齐：发布 GPS 因子（协方差 = base_cov / (factor_weight * quality_scale)，质量越好约束越强）
    // 【修复】降低质量阈值：MEDIUM+ (HDOP≤5.0) 也发布GPS因子，    // 原因：M2DGR弱GPS场景HDOP≈10 (质量LOW)，仅要求HIGH会完全禁用GPS约束
    if (state_ == GPSAlignState::ALIGNED && quality >= GPSQuality::MEDIUM) {
        Eigen::Vector3d pos_map = enu_to_map(pos_enu);
        double hdop_safe = std::max(hdop, 0.1);
        // 【修复】增大协方差基础值以反映弱GPS的实际精度
        // 原sigma_h = hdop * 0.3 对HDOP=10来说只有3m，实际误差可能更大
        double sigma_h = hdop_safe * 0.5;  // 水平精度估算（增大系数）
        double sigma_v = hdop_safe * 1.0;  // 垂直精度估算（增大系数）
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
        cov /= (weight * quality_scale);  // 质量越好 quality_scale 越大 -> 协方差越小 -> 约束越强

        // 距离控制（每 factor_interval_m 添加一个GPS因子）
        double dist = pos_map.norm();
        double interval = ConfigManager::instance().gpsFactorIntervalM();
        if (dist - last_gps_factor_dist_ >= interval) {
            last_gps_factor_dist_ = dist;
            aligned_gps_buffer_.push_back({timestamp, pos_map, cov});
            for (auto& cb : gps_factor_cbs_) cb(timestamp, pos_map, cov);
        }
    }
}

void GPSManager::addKeyFramePose(double timestamp, const Pose3d& T_w_b) {
    std::lock_guard<std::mutex> lk(mutex_);
    kf_window_.push_back({timestamp, T_w_b});
    if (kf_window_.size() > 2000) kf_window_.pop_front();
    
    // 【修复】同时缓存所有里程计位姿用于精準插值
    all_odom_poses_.push_back({timestamp, T_w_b});
    if (all_odom_poses_.size() > 5000) all_odom_poses_.pop_front();
}

void GPSManager::try_align() {
    // 检查轨迹长度
    if (kf_window_.empty() || gps_window_.empty()) return;

    // 计算轨迹总长度
    double total_dist = 0.0;
    for (size_t i = 1; i < kf_window_.size(); ++i) {
        total_dist += (kf_window_[i].second.translation() -
                       kf_window_[i-1].second.translation()).norm();
    }
    if (total_dist < min_align_dist_m_) return;

    // 检查GPS点数量
    // 【修复】放宽质量要求：MEDIUM+ (HDOP≤5.0) 也计入，而非仅HIGH+
    size_t good_gps_count = 0;
    for (const auto& g : gps_window_) {
        if (g.quality >= GPSQuality::MEDIUM) good_gps_count++;
    }
    if ((int)good_gps_count < min_align_points_) return;

    ALOG_INFO(MOD, "[GPS_STATE] NOT_ALIGNED→ALIGNING reason=sufficient_samples good_gps={} dist={:.1f}m",
              good_gps_count, total_dist);
    state_ = GPSAlignState::ALIGNING;
    GPSAlignResult result = compute_svd_alignment();

    if (result.success && result.rmse_m < rmse_accept_thresh_) {
        ALOG_INFO(MOD, "GPS alignment SUCCESS: rmse={:.3f}m matched={} pts",
                  result.rmse_m, result.matched_points);
        align_result_ = result;
        on_aligned(result);
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

    if ((int)pairs.size() < min_align_points_) return result;

    // 中心化
    Eigen::Vector3d gps_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d lio_centroid = Eigen::Vector3d::Zero();
    for (const auto& p : pairs) {
        gps_centroid += p.gps_enu;
        lio_centroid += p.lidar_pos;
    }
    gps_centroid /= pairs.size();
    lio_centroid /= pairs.size();

    // 协方差矩阵（仅XY平面，GPS高度不可靠）
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pairs) {
        Eigen::Vector3d gd = p.gps_enu   - gps_centroid;
        Eigen::Vector3d ld = p.lidar_pos - lio_centroid;
        gd.z() = 0.0;  // 忽略高度
        ld.z() = 0.0;
        cov += ld * gd.transpose();  // LiDAR←GPS
    }

    // SVD 求旋转（仅XY平面旋转，绕Z轴）
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // ✅ 退化场景检查
    Eigen::Vector3d singular_vals = svd.singularValues();
    const double min_sv = singular_vals.minCoeff();
    const double max_sv = singular_vals.maxCoeff();
    const double cond = (max_sv > 1e-12) ? (max_sv / min_sv) : 1e12;
    if (min_sv < 1e-6 || cond > 1e8) {
        ALOG_WARN(MOD, "GPS alignment: degenerate covariance (cond={:.2e}), rejecting", cond);
        return result;  // 返回失败
    }

    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

    // 反射修正
    if (R.determinant() < 0) {
        Eigen::Matrix3d diag = Eigen::Matrix3d::Identity();
        diag(2,2) = -1;
        R = svd.matrixU() * diag * svd.matrixV().transpose();
    }

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

GPSQuality GPSManager::hdop_to_quality(double hdop) const {
    // 【修复】放宽质量阈值，适配弱GPS场景（如M2DGR HDOP≈10）
    // 原阈值：EXCELLENT≤1, HIGH≤2, MEDIUM≤5, LOW≤20
    // 新阈值：EXCELLENT≤1, HIGH≤2, MEDIUM≤10, LOW≤20
    // 原因：M2DGR等城市场景GPS遮挡严重，HDOP通常8-12，按原阈值全为LOW，
    //       导致对齐永远无法触发、GPS约束完全禁用
    if (hdop <= 1.0) return GPSQuality::EXCELLENT;
    if (hdop <= 2.0) return GPSQuality::HIGH;
    if (hdop <= 10.0) return GPSQuality::MEDIUM;  // 修改：5.0 → 10.0
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
        ALOG_WARN(MOD, "[queryByTimestamp] ts={:.3f} result=not_found window_size={} max_dt={:.1f}s "
                  "window_ts=[{:.3f}, {:.3f}] (若 ts 不在区间内则扩大 keyframe_match_window_s)",
                  ts, gps_window_.size(), max_dt, win_min, win_max);
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
