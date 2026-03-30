/**
 * @file backend/delayed_gps_compensator.cpp
 * @brief 后端优化与因子实现。
 */
#include "automap_pro/backend/delayed_gps_compensator.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "DelayedGPSCompensator"

namespace automap_pro {

DelayedGPSCompensator::DelayedGPSCompensator() {
    const auto& cfg = ConfigManager::instance();
    enabled_ = cfg.gpsDelayedCompensationEnabled();
    compensate_on_loop_ = cfg.gpsCompensateOnLoop();
    compensate_on_align_ = cfg.gpsCompensateOnAlign();
    batch_size_ = cfg.gpsCompensationBatchSize();
    max_pending_submaps_ = cfg.gpsMaxPendingSubmaps();
    
    ALOG_INFO(MOD, "Initialized: enabled={} on_loop={} on_align={} batch_size={} max_pending={}",
              enabled_, compensate_on_loop_, compensate_on_align_, batch_size_, max_pending_submaps_);
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS对齐成功回调
// ─────────────────────────────────────────────────────────────────────────────

void DelayedGPSCompensator::onGPSAligned(const GPSAlignResult& result) {
    if (!enabled_) {
        ALOG_DEBUG(MOD, "[onGPSAligned] Compensation disabled, ignoring alignment");
        return;
    }
    
    if (!result.success) {
        ALOG_WARN(MOD, "[onGPSAligned] Alignment failed, skipping");
        return;
    }
    
    std::lock_guard<std::mutex> lk(mutex_);
    
    align_result_ = result;
    gps_aligned_ = true;
    
    ALOG_INFO(MOD, "[GPS_ALIGN] SUCCESS! RMSE={:.3f}m matched={} pts", 
              result.rmse_m, result.matched_points);
    ALOG_INFO(MOD, "[GPS_ALIGN] R_diag=[{:.3f},{:.3f},{:.3f}] t=[{:.2f},{:.2f},{:.2f}]",
              result.R_gps_lidar(0,0), result.R_gps_lidar(1,1), result.R_gps_lidar(2,2),
              result.t_gps_lidar.x(), result.t_gps_lidar.y(), result.t_gps_lidar.z());
}

int DelayedGPSCompensator::collectHistoricalSubmaps(const std::vector<SubMap::Ptr>& all_submaps) {
    if (!enabled_ || !gps_aligned_.load()) {
        ALOG_DEBUG(MOD, "[collectHistoricalSubmaps] Skip: enabled={} aligned={}",
                  enabled_.load(), gps_aligned_.load());
        return 0;
    }
    
    int collected = 0;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (const auto& sm : all_submaps) {
            if (!sm) continue;
            if (sm->state == SubMapState::FROZEN || sm->state == SubMapState::OPTIMIZED) {
                if (!compensated_submaps_.count(sm->id) && !gps_factors_.count(sm->id)) {
                    pending_gps_submaps_.insert(sm->id);
                    collected++;
                }
            }
        }
    }
    
    ALOG_INFO(MOD, "[GPS_COMPENSATE] Collected {} historical submaps for pending compensation (total pending={})",
              collected, pendingCount());
    
    if (compensate_on_align_ && collected > 0) {
        int compensated = compensateBatch(batch_size_.load());
        ALOG_INFO(MOD, "[GPS_COMPENSATE] Batch compensated {} submaps on align", compensated);
    }
    return collected;
}

// ─────────────────────────────────────────────────────────────────────────────
// 回环优化后触发补偿
// ─────────────────────────────────────────────────────────────────────────────

int DelayedGPSCompensator::onPoseOptimized(const std::unordered_map<int, Pose3d>& poses) {
    if (!enabled_.load() || !gps_aligned_.load() || !compensate_on_loop_.load()) {
        return 0;
    }
    
    std::vector<int> to_compensate;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (const auto& [sm_id, pose] : poses) {
            if (pending_gps_submaps_.count(sm_id) && !compensated_submaps_.count(sm_id)) {
                to_compensate.push_back(sm_id);
            }
        }
    }
    
    if (to_compensate.empty()) return 0;
    
    int batch = batch_size_.load();
    if (batch > 0 && static_cast<int>(to_compensate.size()) > batch) {
        to_compensate.resize(batch);
    }
    
    ALOG_INFO(MOD, "[GPS_COMPENSATE] Loop optimized, compensating {} submaps (pending={})",
              static_cast<int>(to_compensate.size()), pendingCount());
    
    int n = 0;
    for (int sm_id : to_compensate) {
        if (compensateSubmap(sm_id)) n++;
    }
    return n;
}

// ─────────────────────────────────────────────────────────────────────────────
// 子图冻结时注册
// ─────────────────────────────────────────────────────────────────────────────

void DelayedGPSCompensator::registerSubmap(const SubMap::Ptr& sm) {
    if (!enabled_ || !sm) return;
    
    {
        std::lock_guard<std::mutex> lk(mutex_);
        
        // 检查队列上限
        if (static_cast<int>(pending_gps_submaps_.size()) >= max_pending_submaps_) {
            ALOG_WARN(MOD, "[registerSubmap] Pending queue full ({}), dropping submap {}",
                      max_pending_submaps_, sm->id);
            return;
        }
    }
    
    if (!gps_aligned_) {
        // GPS未对齐，加入待补偿队列
        std::lock_guard<std::mutex> lk(mutex_);
        pending_gps_submaps_.insert(sm->id);
        ALOG_DEBUG(MOD, "[GPS_COMPENSATE] Submap {} registered for pending compensation", sm->id);
    } else {
        // GPS已对齐，立即补偿
        compensateSubmap(sm->id);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 批量补偿
// ─────────────────────────────────────────────────────────────────────────────

int DelayedGPSCompensator::compensateBatch(int max_count) {
    if (!enabled_.load() || !gps_aligned_.load()) return 0;
    
    std::vector<int> to_compensate;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        int count = 0;
        for (int sm_id : pending_gps_submaps_) {
            if (compensated_submaps_.count(sm_id)) continue;
            to_compensate.push_back(sm_id);
            count++;
            if (max_count >= 0 && count >= max_count) break;
        }
    }
    
    int compensated = 0;
    for (int sm_id : to_compensate) {
        if (compensateSubmap(sm_id)) {
            compensated++;
        }
    }
    
    if (compensated > 0) {
        ALOG_INFO(MOD, "[GPS_COMPENSATE] Batch compensated {}/{} submaps", 
                  compensated, to_compensate.size());
    }
    
    return compensated;
}

// ─────────────────────────────────────────────────────────────────────────────
// 补偿单个子图
// ─────────────────────────────────────────────────────────────────────────────

bool DelayedGPSCompensator::compensateSubmap(int sm_id) {
    if (!enabled_.load() || !gps_aligned_.load()) return false;
    
    // 幂等性检查：在补偿前先标记状态，避免回调重入时看到“未补偿”
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (compensated_submaps_.count(sm_id) || gps_factors_.count(sm_id)) {
            ALOG_DEBUG(MOD, "[compensateSubmap] Submap {} already compensated/has GPS factor, skip",
                       sm_id);
            return false;
        }
        // 预先将该子图从待补偿集合中移除，并标记为已补偿占位，防止重入
        pending_gps_submaps_.erase(sm_id);
        compensated_submaps_.insert(sm_id);
    }
    
    // 查询子图对应的GPS测量
    auto gps_meas = queryGPSForSubmap(sm_id);
    if (!gps_meas) {
        ALOG_WARN(MOD, "[GPS_COMPENSATE] No valid GPS for submap {}, skipping", sm_id);
        
        std::lock_guard<std::mutex> lk(mutex_);
        pending_gps_submaps_.erase(sm_id);
        return false;
    }
    
    // 转换到地图坐标系
    Eigen::Vector3d pos_map = enuToMap(gps_meas->position_enu);
    
    // 计算协方差
    Eigen::Matrix3d cov = computeGPSCovariance(gps_meas->quality, gps_meas->hdop);
    
    // 创建GPS因子信息
    GPSFactorInfo factor_info;
    factor_info.submap_id = sm_id;
    factor_info.timestamp = gps_meas->timestamp;
    factor_info.pos_map = pos_map;
    factor_info.covariance = cov;
    factor_info.quality = gps_meas->quality;
    factor_info.is_compensated = true;
    
    // 调用回调添加GPS因子
    {
        std::lock_guard<std::mutex> lk(mutex_);
        
        for (auto& cb : gps_factor_cbs_) {
            try {
                cb(sm_id, pos_map, cov, true);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "[GPS_COMPENSATE] Exception in GPS factor callback: {}", e.what());
            } catch (...) {
                ALOG_ERROR(MOD, "[GPS_COMPENSATE] Unknown exception in GPS factor callback");
            }
        }
        
        // 记录GPS因子（此时 compensated_submaps_ 中已包含 sm_id，回调重入也会被幂等保护拦截）
        gps_factors_[sm_id] = factor_info;
    }
    
    ALOG_INFO(MOD, "[GPS_COMPENSATE] Submap {} compensated: pos=[{:.2f},{:.2f},{:.2f}] quality={}",
              sm_id, pos_map.x(), pos_map.y(), pos_map.z(),
              static_cast<int>(gps_meas->quality));
    ALOG_INFO(MOD, "[POSE_JUMP_CAUSE] GPS 延迟补偿已为子图 {} 添加 GPS 因子 → 下次后端优化会更新位姿，RViz 可能跳变；查跳变: grep POSE_JUMP", sm_id);
    
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 查询子图对应的GPS测量
// ─────────────────────────────────────────────────────────────────────────────

std::optional<GPSMeasurement> DelayedGPSCompensator::queryGPSForSubmap(int sm_id) {
    if (!submap_gps_query_cb_) {
        ALOG_WARN(MOD, "[queryGPSForSubmap] No GPS query callback registered");
        return std::nullopt;
    }
    
    try {
        return submap_gps_query_cb_(sm_id);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[queryGPSForSubmap] Exception: {}", e.what());
        return std::nullopt;
    } catch (...) {
        ALOG_ERROR(MOD, "[queryGPSForSubmap] Unknown exception");
        return std::nullopt;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 计算GPS协方差
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Matrix3d DelayedGPSCompensator::computeGPSCovariance(GPSQuality quality, double hdop) {
    const auto& cfg = ConfigManager::instance();
    
    double base_sigma_h = cfg.gpsBaseSigmaH();  // 水平基础标准差
    double base_sigma_v = cfg.gpsBaseSigmaV();  // 垂直基础标准差
    
    // 基于HDOP调整
    double hdop_safe = std::max(hdop, 0.1);
    double sigma_h = base_sigma_h * hdop_safe;
    double sigma_v = base_sigma_v * hdop_safe;
    
    // 基于质量进一步调整
    double quality_scale = 1.0;
    switch (quality) {
        case GPSQuality::EXCELLENT: quality_scale = cfg.gpsFactorQualityScaleExcellent(); break;
        case GPSQuality::HIGH:      quality_scale = cfg.gpsFactorQualityScaleHigh(); break;
        case GPSQuality::MEDIUM:    quality_scale = cfg.gpsFactorQualityScaleMedium(); break;
        case GPSQuality::LOW:       quality_scale = cfg.gpsFactorQualityScaleLow(); break;
        default:                    quality_scale = 10.0; break;
    }
    
    sigma_h /= quality_scale;
    sigma_v /= quality_scale;
    
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = sigma_h * sigma_h;
    cov(1, 1) = sigma_h * sigma_h;
    cov(2, 2) = sigma_v * sigma_v;
    
    return cov;
}

// ─────────────────────────────────────────────────────────────────────────────
// ENU到地图坐标转换（仅在对齐后用于因子，align_result_ 与 GPSManager 一致）
// ─────────────────────────────────────────────────────────────────────────────

Eigen::Vector3d DelayedGPSCompensator::enuToMap(const Eigen::Vector3d& pos_enu) const {
    if (!gps_aligned_) {
        return pos_enu;  // 未对齐时返回原坐标；补偿逻辑应仅在对齐后执行，不向因子图写入 ENU
    }
    
    // 复制对齐结果到局部变量，避免在无锁读取时看到“半更新”状态
    GPSAlignResult align_copy;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        align_copy = align_result_;
    }
    return align_copy.R_gps_lidar * pos_enu + align_copy.t_gps_lidar;
}

// ─────────────────────────────────────────────────────────────────────────────
// 状态查询
// ─────────────────────────────────────────────────────────────────────────────

size_t DelayedGPSCompensator::pendingCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return pending_gps_submaps_.size();
}

size_t DelayedGPSCompensator::compensatedCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return compensated_submaps_.size();
}

std::vector<int> DelayedGPSCompensator::getPendingSubmapIds() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return std::vector<int>(pending_gps_submaps_.begin(), pending_gps_submaps_.end());
}

std::vector<int> DelayedGPSCompensator::getCompensatedSubmapIds() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return std::vector<int>(compensated_submaps_.begin(), compensated_submaps_.end());
}

std::optional<GPSFactorInfo> DelayedGPSCompensator::getGPSFactorInfo(int sm_id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = gps_factors_.find(sm_id);
    if (it != gps_factors_.end()) return it->second;
    return std::nullopt;
}

// ─────────────────────────────────────────────────────────────────────────────
// 重置
// ─────────────────────────────────────────────────────────────────────────────

void DelayedGPSCompensator::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    pending_gps_submaps_.clear();
    compensated_submaps_.clear();
    gps_factors_.clear();
    gps_aligned_ = false;
    align_result_ = GPSAlignResult{};
    ALOG_INFO(MOD, "[reset] Cleared all compensation state");
}

} // namespace automap_pro
