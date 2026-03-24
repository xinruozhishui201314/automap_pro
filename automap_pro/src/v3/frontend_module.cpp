#include "automap_pro/v3/frontend_module.h"
#include "automap_pro/backend/gps_constraint_policy.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/utils.h"
#include <pcl_conversions/pcl_conversions.h>
#include <atomic>
#include <chrono>
#include <cmath>

namespace automap_pro::v3 {
namespace {

// full.log 闭环：GPS 缓存命中/拒绝统计（grep GPS_CACHE KPI / GPS_CACHE VERIFY）
std::atomic<uint64_t> g_gps_cache_get_empty_total{0};
std::atomic<uint64_t> g_gps_cache_get_no_neighbor_total{0};
std::atomic<uint64_t> g_gps_cache_get_reject_invalid_total{0};
std::atomic<uint64_t> g_gps_cache_get_ok_total{0};


GPSQuality qualityFromHdopAndSats(double hdop, int sats, double medium_max_hdop) {
    // NavSatFix 常不提供卫星颗数；sats<=0 视为未知，不直接否决，改由 HDOP 门控。
    // 仅在明确给出且小于 4 时判定为无效。
    if ((sats > 0 && sats < 4) || !std::isfinite(hdop)) return GPSQuality::INVALID;
    if (hdop <= 1.0) return GPSQuality::EXCELLENT;
    if (hdop <= 2.0) return GPSQuality::HIGH;
    if (hdop <= medium_max_hdop) return GPSQuality::MEDIUM;
    if (hdop <= 20.0) return GPSQuality::LOW;
    return GPSQuality::INVALID;
}

Eigen::Matrix3d covarianceFromGpsQuality(GPSQuality q) {
    const auto& cfg = ConfigManager::instance();
    Eigen::Vector3d sigmas = Eigen::Vector3d::Ones();
    switch (q) {
        case GPSQuality::EXCELLENT: sigmas = cfg.gpsCovExcellent(); break;
        case GPSQuality::HIGH:      sigmas = cfg.gpsCovHigh(); break;
        case GPSQuality::MEDIUM:    sigmas = cfg.gpsCovMedium(); break;
        case GPSQuality::LOW:       sigmas = cfg.gpsCovLow(); break;
        default:
            return Eigen::Matrix3d::Identity() * 1e6;
    }
    if (!sigmas.allFinite() || (sigmas.array() < 0.0).any()) {
        return Eigen::Matrix3d::Identity() * 1e6;
    }
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    cov(0, 0) = sigmas(0) * sigmas(0);
    cov(1, 1) = sigmas(1) * sigmas(1);
    cov(2, 2) = sigmas(2) * sigmas(2);
    return cov;
}

} // namespace

FrontEndModule::FrontEndModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("FrontEndModule", event_bus, map_registry), node_(node) {
    
    const auto& cfg = ConfigManager::instance();
    bool gps_enabled = cfg.gpsEnabled();
    std::string gps_topic = cfg.gpsTopic();
    
    livo_bridge_.init(node, gps_enabled, gps_topic);
    
    livo_bridge_.registerOdomCallback([this](double ts, const Pose3d& pose, const Mat66d& cov) {
        this->onOdometry(ts, pose, cov);
    });
    livo_bridge_.registerCloudCallback([this](double ts, const CloudXYZIPtr& cloud) {
        this->onCloud(ts, cloud);
    });
    livo_bridge_.registerKFInfoCallback([this](const LivoKeyFrameInfo& info) {
        this->onKFInfo(info);
    });
    livo_bridge_.registerGPSCallback([this](double ts, double lat, double lon, double alt, double hdop, int sats) {
        this->onGPS(ts, lat, lon, alt, hdop, sats);
    });

    const size_t max_ingress_q = cfg.ingressQueueMaxSize();
    const size_t max_frame_q = cfg.frameQueueMaxSize();
    frame_processor_.init(max_ingress_q, max_frame_q);
    RCLCPP_INFO(node_->get_logger(),
        "[DIAG][GPS_CACHE][CONFIG] gps_enabled=%d gps_topic=%s quality_threshold_hdop=%.2f min_quality=%d "
        "keyframe_match_window_s=%.2f gps_cache_size=%zu",
        gps_enabled ? 1 : 0, gps_topic.c_str(),
        cfg.gpsQualityThreshold(), cfg.gpsMinAcceptedQualityLevel(),
        cfg.gpsKeyframeMatchWindowS(), kMaxGPSCacheSize);

    RCLCPP_INFO(node_->get_logger(),
                "[PIPELINE][FE] ctor OK LivoBridge+FrameProcessor ingress_max=%zu frame_max=%zu gps=%s topic=%s",
                max_ingress_q, max_frame_q, gps_enabled ? "on" : "off", gps_topic.c_str());

    // 🏛️ [架构契约] Cascading Backpressure (级联背压响应)
    onEvent<BackpressureWarningEvent>([this](const BackpressureWarningEvent& ev) {
        if (ev.queue_usage_ratio > 0.9f || ev.critical) {
            throttle_active_.store(true);
            const double now = node_->now().seconds();
            throttle_until_.store(now + 2.0); // 惩罚性限流 2s
            RCLCPP_WARN(node_->get_logger(),
                "[V3][BACKPRESSURE] FE reacting to %s overload: Enabling CRITICAL throttle.", ev.module_name.c_str());
        } else if (ev.queue_usage_ratio > 0.7f) {
            throttle_active_.store(true);
            const double now = node_->now().seconds();
            throttle_until_.store(now + 0.5); // 轻微限流 0.5s
            RCLCPP_INFO(node_->get_logger(),
                "[V3][BACKPRESSURE] FE reacting to %s pressure: Enabling soft throttle.", ev.module_name.c_str());
        }
    });

    // 🏛️ [架构契约] Frontend Pose Adjustment (位姿跳变对齐)
    onEvent<FrontendPoseAdjustEvent>([this](const FrontendPoseAdjustEvent& ev) {
        std::lock_guard<std::mutex> lk(data_mutex_);
        // 修正当前追踪器位姿，使其随地图跳变，避免重影
        last_odom_pose_ = ev.T_map_new_map_old * last_odom_pose_;
        
        RCLCPP_INFO(node_->get_logger(),
            "[V3][GHOST_GUARD] FE adjusted current pose by backend feedback: v%lu->v%lu",
            ev.from_version, ev.to_version);
    });
}

void FrontEndModule::start() {
    if (running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::start (override) begin");
    running_ = true;
    
    frame_processor_.start();
    thread_ = std::thread(&FrontEndModule::run, this);
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::start sync thread spawned");
}

void FrontEndModule::stop() {
    if (!running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::stop begin");
    running_ = false;
    shutdown_requested_.store(true);
    frame_processor_.stop();
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    ALOG_INFO("Pipeline", "[PIPELINE][FE] FrontEndModule::stop joined");
}

void FrontEndModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][FrontEndModule] Started sync thread");
    
    while (running_) {
        updateHeartbeat();

        // 🏛️ [架构契约] 背压限流应用
        if (throttle_active_.load()) {
            const double now = node_->now().seconds();
            if (now < throttle_until_.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            } else {
                throttle_active_.store(false);
            }
        }

        FrameToProcess f;
        bool woke_by_data = frame_processor_.tryPopFrame(100, f);
        if (!running_) break;
        if (!woke_by_data) continue;

        Pose3d pose = Pose3d::Identity();
        Mat66d cov = Mat66d::Identity() * 1e-4;

        if (!odomCacheGet(f.ts, pose, cov)) {
            // 插值失败时（多线程调度 / 短时滞后 / 历史边界）用最近一次里程计，与 LivoBridge「cloud 配 last odom」一致；过大时间差仍丢弃，避免错配
            std::lock_guard<std::mutex> lk(data_mutex_);
            constexpr double kMaxCloudOdomSkewS = 1.5;
            if (last_odom_ts_ >= 0.0 && std::abs(f.ts - last_odom_ts_) <= kMaxCloudOdomSkewS) {
                pose = last_odom_pose_;
                cov = last_cov_;
            } else {
                continue;
            }
        }

        LivoKeyFrameInfo kfinfo_copy;
        bool kfinfo_from_cache = kfinfoCacheGet(f.ts, kfinfo_copy);
        if (!kfinfo_from_cache) {
            std::lock_guard<std::mutex> lk(data_mutex_);
            kfinfo_copy = last_livo_info_;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[SEMANTIC][FrontEnd][KFINFO] source=fallback_last_livo frame_ts=%.3f fallback_kf_ts=%.3f",
                f.ts, kfinfo_copy.timestamp);
        }
        const bool kf_ts_valid = std::isfinite(kfinfo_copy.timestamp) && (kfinfo_copy.timestamp > 0.0);
        const double kf_dt = kf_ts_valid ? std::abs(f.ts - kfinfo_copy.timestamp) : -1.0;
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[SEMANTIC][FrontEnd][KFINFO] source=%s frame_ts=%.3f kf_ts=%.3f dt=%.3f valid=%d",
            kfinfo_from_cache ? "cache_leq_ts" : "fallback_last_livo",
            f.ts, kfinfo_copy.timestamp, kf_dt, kf_ts_valid ? 1 : 0);

        GPSMeasurement matched_gps;
        bool has_gps = gpsCacheGet(f.ts, matched_gps);
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 20000,
            "[GPS_CACHE][KPI] get_ok=%llu empty=%llu no_neighbor=%llu reject_invalid=%llu "
            "(grep GPS_CACHE; enable DEBUG for GPS_CACHE VERIFY per-sample)",
            static_cast<unsigned long long>(g_gps_cache_get_ok_total.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_gps_cache_get_empty_total.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_gps_cache_get_no_neighbor_total.load(std::memory_order_relaxed)),
            static_cast<unsigned long long>(g_gps_cache_get_reject_invalid_total.load(std::memory_order_relaxed)));

        SyncedFrameEvent event;
        event.timestamp = f.ts;
        event.cloud = f.cloud;
        event.cloud_ds = f.cloud_ds;
        event.T_odom_b = pose;
        event.covariance = cov;
        event.kf_info = kfinfo_copy;
        event.has_gps = has_gps;
        event.gps = matched_gps;
        event.pose_frame = PoseFrame::ODOM; // 🏛️ [架构契约] 显式标注前端产出为 ODOM 系
        event.cloud_frame = ConfigManager::instance().frontendCloudFrame();
        event.ref_map_version = map_registry_->getVersion();
        event.ref_alignment_epoch = map_registry_->getAlignmentEpoch();

        if (event.isValid()) {
            event_bus_->publish(event);
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[SEMANTIC][FrontEnd][run] step=publish SyncedFrameEvent ts=%.3f pts=%zu ref_version=%lu",
                event.timestamp, event.cloud ? event.cloud->size() : 0, event.ref_map_version);
        } else {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][CONTRACT] Dropping invalid SyncedFrameEvent (NaN/null/invalid cloud_frame=%s)",
                event.cloud_frame.c_str());
        }
    }
}

void FrontEndModule::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    RawOdometryEvent ev;
    ev.timestamp = ts;
    ev.pose = pose;
    ev.covariance = cov;
    event_bus_->publish(ev);

    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        last_odom_pose_ = pose;
        last_odom_ts_ = ts;
        last_cov_ = cov;
    }
    odomCacheAdd(ts, pose, cov);
}

void FrontEndModule::onCloud(double ts, const CloudXYZIPtr& cloud) {
    RawCloudEvent ev;
    ev.timestamp = ts;
    ev.cloud = cloud;
    event_bus_->publish(ev);

    frame_processor_.pushFrame(ts, cloud);
}

void FrontEndModule::onKFInfo(const LivoKeyFrameInfo& info) {
    RawKFInfoEvent ev;
    ev.info = info;
    event_bus_->publish(ev);

    { std::lock_guard<std::mutex> lk(data_mutex_); last_livo_info_ = info; }
    kfinfoCacheAdd(info.timestamp, info);
}

void FrontEndModule::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats) {
    RawGPSEvent ev;
    ev.timestamp = ts;
    ev.lat = lat; ev.lon = lon; ev.alt = alt; ev.hdop = hdop; ev.sats = sats;
    event_bus_->publish(ev);

    GPSMeasurement m;
    m.timestamp = ts;
    m.latitude = lat;
    m.longitude = lon;
    m.altitude = alt;
    m.hdop = hdop;
    m.num_satellites = sats;
    const double qthresh = ConfigManager::instance().gpsQualityThreshold();
    m.quality = qualityFromHdopAndSats(hdop, sats, qthresh);
    m.is_valid = (m.quality >= GPSQuality::MEDIUM);
    m.covariance = covarianceFromGpsQuality(m.quality);

    if (m.is_valid) {
        map_registry_->setGPSOrigin(lat, lon, alt);
        
        double olat, olon, oalt;
        if (map_registry_->getGPSOrigin(olat, olon, oalt)) {
            m.position_enu = utils::wgs84ToEnu(lat, lon, alt, olat, olon, oalt);
        }
    }
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][GPS_DIAG] ts=%.3f sats=%d hdop=%.3f quality=%d valid=%d cov_diag=[%.3f,%.3f,%.3f]",
        ts, sats, hdop, static_cast<int>(m.quality), m.is_valid ? 1 : 0,
        m.covariance(0, 0), m.covariance(1, 1), m.covariance(2, 2));

    gpsCacheAdd(ts, m);
}

void FrontEndModule::gpsCacheAdd(double ts, const GPSMeasurement& m) {
    std::lock_guard<std::mutex> lk(gps_cache_mutex_);
    gps_cache_.push_back({ts, m});
    while (gps_cache_.size() > kMaxGPSCacheSize) gps_cache_.pop_front();
}

bool FrontEndModule::gpsCacheGet(double ts, GPSMeasurement& out_m) {
    std::lock_guard<std::mutex> lk(gps_cache_mutex_);
    if (gps_cache_.empty()) {
        g_gps_cache_get_empty_total.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_DEBUG(node_->get_logger(),
            "[GPS_CACHE][VERIFY] miss reason=empty_cache ts_req=%.6f", ts);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[DIAG][GPS_CACHE][E_EMPTY] cache empty while querying ts=%.6f (no GPS can bind to keyframe)", ts);
        return false;
    }
    
    // 使用配置的 keyframe_match_window_s（与 GPSManager 一致），避免硬编码 0.5s 过严导致 has_valid_gps=0、HBA gps_factors=0
    const double max_dt = ConfigManager::instance().gpsKeyframeMatchWindowS();
    double min_dt = 1e9;
    const GPSCacheEntry* best = nullptr;
    
    for (auto it = gps_cache_.rbegin(); it != gps_cache_.rend(); ++it) {
        double dt = std::abs(it->ts - ts);
        if (dt < min_dt) {
            min_dt = dt;
            best = &(*it);
        }
        if (it->ts < ts - max_dt) break;
    }
    
    if (best && min_dt <= max_dt) {
        // 使用统一约束策略：前端 keyframe 绑定与后端/HBA 使用同一质量阈值语义，避免跨模块策略漂移。
        const auto& cfg = ConfigManager::instance();
        const auto decision = evaluateKeyframeGpsConstraint(
            best->m,
            best->m.is_valid,
            cfg.gpsMinAcceptedQualityLevel(),
            false,
            min_dt,
            max_dt);
        if (!decision.accepted) {
            g_gps_cache_get_reject_invalid_total.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_DEBUG(node_->get_logger(),
                "[GPS_CACHE][VERIFY] reject reason=policy_or_enu ts_req=%.6f dt=%.4f quality=%d "
                "min_quality=%d is_valid=%d pos_enu_finite=%d hdop=%.2f reject_reason=%d",
                ts, min_dt, static_cast<int>(best->m.quality), cfg.gpsMinAcceptedQualityLevel(),
                best->m.is_valid ? 1 : 0, best->m.position_enu.allFinite() ? 1 : 0,
                best->m.hdop, static_cast<int>(decision.reason));
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[DIAG][GPS_CACHE][E_BIND_REJECT] ts_req=%.6f dt=%.4f quality=%d min_quality=%d "
                "is_valid=%d pos_enu_finite=%d hdop=%.2f reject_reason=%d",
                ts, min_dt, static_cast<int>(best->m.quality), cfg.gpsMinAcceptedQualityLevel(),
                best->m.is_valid ? 1 : 0, best->m.position_enu.allFinite() ? 1 : 0,
                best->m.hdop, static_cast<int>(decision.reason));
            return false;
        }
        out_m = best->m;
        g_gps_cache_get_ok_total.fetch_add(1, std::memory_order_relaxed);
        return true;
    }
    g_gps_cache_get_no_neighbor_total.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_DEBUG(node_->get_logger(),
        "[GPS_CACHE][VERIFY] miss reason=no_neighbor_in_window ts_req=%.6f min_dt=%.4f max_dt=%.2f",
        ts, best ? min_dt : -1.0, max_dt);
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "[DIAG][GPS_CACHE][E_NO_NEIGHBOR] ts_req=%.6f min_dt=%.4f max_dt=%.2f",
        ts, best ? min_dt : -1.0, max_dt);
    return false;
}

void FrontEndModule::odomCacheAdd(double ts, const Pose3d& pose, const Mat66d& cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    odom_cache_.push_back({ts, pose, cov});
    while (odom_cache_.size() > kMaxOdomCacheSize) odom_cache_.pop_front();
}

bool FrontEndModule::odomCacheGet(double ts, Pose3d& out_pose, Mat66d& out_cov) {
    std::lock_guard<std::mutex> lk(odom_cache_mutex_);
    if (odom_cache_.empty()) return false;
    
    // 如果只有一个样本，直接返回
    if (odom_cache_.size() == 1) {
        if (std::abs(odom_cache_.front().ts - ts) < 0.5) {
            out_pose = odom_cache_.front().pose;
            out_cov  = odom_cache_.front().cov;
            return true;
        }
        return false;
    }

    // 寻找插值区间 [it_prev, it_next]
    auto it_next = odom_cache_.begin();
    while (it_next != odom_cache_.end() && it_next->ts < ts) {
        ++it_next;
    }

    if (it_next == odom_cache_.begin()) {
        // 请求时间在最旧数据之前 (通常是系统刚启动)
        if (std::abs(it_next->ts - ts) < 0.2) {
            out_pose = it_next->pose;
            out_cov = it_next->cov;
            return true;
        }
        return false;
    }

    if (it_next == odom_cache_.end()) {
        // 请求时间在最新数据之后 (后端处理慢，里程计还没跟上)
        auto it_last = std::prev(it_next);
        if (std::abs(it_last->ts - ts) < 0.5) {
            out_pose = it_last->pose;
            out_cov = it_last->cov;
            return true;
        }
        // 如果差得太多，可能需要等待数据，这里暂时返回 false，外部 run() 会 continue
        return false;
    }

    // 线性插值
    auto it_prev = std::prev(it_next);
    double ratio = (ts - it_prev->ts) / (it_next->ts - it_prev->ts);
    
    // 位姿线性插值（平移线性，旋转 SLERP）
    Eigen::Vector3d t = (1.0 - ratio) * it_prev->pose.translation() + ratio * it_next->pose.translation();
    Eigen::Quaterniond q_prev(it_prev->pose.rotation());
    Eigen::Quaterniond q_next(it_next->pose.rotation());
    Eigen::Quaterniond q = q_prev.slerp(ratio, q_next);
    
    out_pose = Pose3d::Identity();
    out_pose.linear() = q.toRotationMatrix();
    out_pose.translation() = t;
    
    // 协方差线性插值
    out_cov = (1.0 - ratio) * it_prev->cov + ratio * it_next->cov;
    
    return true;
}

void FrontEndModule::kfinfoCacheAdd(double ts, const LivoKeyFrameInfo& info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    kfinfo_cache_.push_back({ts, info});
    while (kfinfo_cache_.size() > kMaxKFinfoCacheSize) kfinfo_cache_.pop_front();
}

bool FrontEndModule::kfinfoCacheGet(double ts, LivoKeyFrameInfo& out_info) {
    std::lock_guard<std::mutex> lk(kfinfo_cache_mutex_);
    if (kfinfo_cache_.empty()) return false;
    const KFinfoCacheEntry* best = nullptr;
    for (auto it = kfinfo_cache_.rbegin(); it != kfinfo_cache_.rend(); ++it) {
        if (it->ts <= ts) { best = &(*it); break; }
    }
    if (!best) return false;
    out_info = best->info;
    return true;
}

} // namespace automap_pro::v3
