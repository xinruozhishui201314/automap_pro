#include "automap_pro/v3/semantic_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <vector>
#include <pcl/common/transforms.h>

namespace automap_pro::v3 {
namespace {

size_t dtBinIndex(const bool has_kf_ts, double dt_sec) {
    if (!has_kf_ts || !std::isfinite(dt_sec) || dt_sec < 0.0) return 7;
    if (dt_sec <= 0.01) return 0;
    if (dt_sec <= 0.03) return 1;
    if (dt_sec <= 0.05) return 2;
    if (dt_sec <= 0.10) return 3;
    if (dt_sec <= 0.20) return 4;
    if (dt_sec <= 0.50) return 5;
    return 6;
}

double percentileFromSorted(const std::vector<double>& v, double p) {
    if (v.empty()) return -1.0;
    if (p <= 0.0) return v.front();
    if (p >= 1.0) return v.back();
    const double idx = p * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(std::floor(idx));
    const size_t hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) return v[lo];
    const double w = idx - static_cast<double>(lo);
    return v[lo] * (1.0 - w) + v[hi] * w;
}

}  // namespace

void SemanticModule::enqueueTaskLocked(const SyncedFrameEvent& ev) {
    if (task_queue_.size() >= kCoalesceThreshold) {
        const double replaced_ts = task_queue_.back().timestamp;
        task_queue_.back() = ev;
        const auto coalesced = ++coalesced_tasks_;
        if ((coalesced % 50) == 0) {
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][ENQUEUE] step=coalesce_latest total=%lu replaced_ts=%.3f new_ts=%.3f queue_size=%zu",
                static_cast<unsigned long>(coalesced), replaced_ts, ev.timestamp, task_queue_.size());
        }
        return;
    }
    if (task_queue_.size() >= kMaxQueueSize) {
        const double dropped_ts = task_queue_.front().timestamp;
        task_queue_.pop_front();
        ++backpressure_drops_;
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][ENQUEUE] step=backpressure_drop ts=%.3f queue_full=%zu -> dropped oldest",
            dropped_ts, kMaxQueueSize);
    }
    task_queue_.push_back(ev);
}

void SemanticModule::flushPendingKeyframeTasksLocked() {
    if (!keyframes_only_ || pending_keyframe_queue_.empty()) return;
    // 关键帧驱动模式：pending 仅用于“尚未拿到首个有效 kf_ts”的暂存，不做 dt 对齐打捞。
    const double latest_kf_ts = latest_kf_ts_seen_.load(std::memory_order_relaxed);
    if (!(latest_kf_ts > 0.0) || !std::isfinite(latest_kf_ts)) return;
    const size_t dropped = pending_keyframe_queue_.size();
    for (const auto& ev : pending_keyframe_queue_) {
        const double age_sec = std::abs(ev.timestamp - latest_kf_ts);
        pending_drop_age_samples_sec_.push_back(age_sec);
        if (pending_drop_age_samples_sec_.size() > kMaxPendingAgeSamples) {
            pending_drop_age_samples_sec_.pop_front();
        }
    }
    pending_dropped_.fetch_add(dropped, std::memory_order_relaxed);
    pending_keyframe_queue_.clear();
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[SEMANTIC][Module][PENDING] step=flush_on_first_kf latest_kf_ts=%.3f dropped=%zu (keyframe-driven mode)",
        latest_kf_ts, dropped);
}

SemanticModule::SemanticModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("SemanticModule", event_bus, map_registry), node_(node) {
    
    const auto& cfg = ConfigManager::instance();
    const bool semantic_enabled = cfg.semanticEnabled();
    semantic_cfg_.model_path = cfg.semanticModelPath();
    semantic_cfg_.fov_up = cfg.semanticFovUp();
    semantic_cfg_.fov_down = cfg.semanticFovDown();
    semantic_cfg_.img_w = cfg.semanticImgW();
    semantic_cfg_.img_h = cfg.semanticImgH();
    semantic_cfg_.input_channels = cfg.semanticInputChannels();
    semantic_cfg_.num_classes = cfg.semanticNumClasses();
    semantic_cfg_.tree_class_id = cfg.semanticTreeClassId();
    semantic_cfg_.input_mean = cfg.semanticInputMean();
    semantic_cfg_.input_std = cfg.semanticInputStd();
    semantic_cfg_.do_destagger = cfg.semanticDoDestagger();
    keyframes_only_ = cfg.semanticKeyframesOnly();
    keyframe_time_tolerance_s_ = cfg.semanticKeyframeTimeToleranceS();

    if (!semantic_enabled) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][INIT] step=disabled reason=config semantic.enabled=false");
    } else if (semantic_cfg_.model_path.empty()) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][INIT] step=disabled reason=empty_model_path (set semantic.model_path to enable)");
    } else if (!std::filesystem::exists(semantic_cfg_.model_path)) {
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][Module][INIT] step=disabled reason=model_not_found path=%s",
            semantic_cfg_.model_path.c_str());
    } else {
        try {
            semantic_processor_ = std::make_shared<SemanticProcessor>(semantic_cfg_);
            semantic_runtime_ready_.store(
                semantic_processor_ && semantic_processor_->hasRuntimeCapability(),
                std::memory_order_relaxed);
        } catch (const std::exception& e) {
            semantic_runtime_ready_.store(false, std::memory_order_relaxed);
            semantic_processor_.reset();
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][Module][INIT] step=disabled reason=processor_init_exception error=%s",
                e.what());
        }
    }

    // 订阅同步帧事件
    onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
        handleSyncedFrameEvent(ev);
    });

    // 订阅系统静默请求 (🏛️ [架构契约] 屏障同步)
    onEvent<SystemQuiesceRequestEvent>([this](const SystemQuiesceRequestEvent& ev) {
        this->quiesce(ev.enable);
    });

    if (!semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][INIT] step=degraded enabled=0 reason=runtime_unavailable queue_max=%zu "
            "diag(enabled=%d model_path=%s model_path_empty=%d model_exists=%d keyframes_only=%d kf_tol=%.3f)",
            kMaxQueueSize, semantic_enabled ? 1 : 0, semantic_cfg_.model_path.c_str(),
            semantic_cfg_.model_path.empty() ? 1 : 0, std::filesystem::exists(semantic_cfg_.model_path) ? 1 : 0,
            keyframes_only_ ? 1 : 0, keyframe_time_tolerance_s_);
    } else {
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][INIT] step=ok enabled=1 queue_max=%zu keyframes_only=%d kf_tol=%.3f",
            kMaxQueueSize, keyframes_only_ ? 1 : 0, keyframe_time_tolerance_s_);
    }
}

void SemanticModule::start() {
    ModuleBase::start();
}

void SemanticModule::stop() {
    ModuleBase::stop();
}

void SemanticModule::handleSyncedFrameEvent(const SyncedFrameEvent& ev) {
    if (!running_.load()) return;
    
    // 🏛️ [架构契约] 静默状态拦截新数据输入
    if (quiescing_.load()) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][Module] Drop frame: Module is QUIESCING");
        return;
    }

    if (!semantic_runtime_ready_.load(std::memory_order_relaxed) ||
        semantic_degraded_.load(std::memory_order_relaxed)) return;
    SyncedFrameEvent normalized_ev;
    if (!normalizeSemanticInputFrame(ev, normalized_ev)) return;

    const double kf_ts = normalized_ev.kf_info.timestamp;
    if (std::isfinite(kf_ts) && kf_ts > 0.0) {
        latest_kf_ts_seen_.store(kf_ts, std::memory_order_relaxed);
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (keyframes_only_) {
        const bool has_kf_ts = std::isfinite(kf_ts) && (kf_ts > 0.0);
        const double dt_to_kf = has_kf_ts ? std::abs(normalized_ev.timestamp - kf_ts) : -1.0;
        dt_hist_bins_[dtBinIndex(has_kf_ts, dt_to_kf)].fetch_add(1, std::memory_order_relaxed);

        if (!has_kf_ts) {
            if (pending_keyframe_queue_.size() >= kMaxPendingKeyframeQueueSize) {
                pending_keyframe_queue_.pop_front();
                ++pending_dropped_;
                BackpressureWarningEvent warn;
                warn.module_name = name_ + "_pending";
                warn.queue_usage_ratio = 1.0f;
                warn.critical = true;
                event_bus_->publish(warn);
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "[SEMANTIC][Module][PENDING] step=overflow action=drop_oldest pending_cap=%zu latest_kf_ts=%.3f",
                    kMaxPendingKeyframeQueueSize, latest_kf_ts_seen_.load(std::memory_order_relaxed));
            }
            pending_keyframe_queue_.push_back(normalized_ev);

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[SEMANTIC][Module][PENDING] step=defer reason=%s frame_ts=%.3f kf_ts=%.3f dt=%.3f tol=%.3f pending=%zu task_q=%zu",
                has_kf_ts ? "dt_exceeds_tolerance" : "missing_kf_ts",
                normalized_ev.timestamp, kf_ts, dt_to_kf, keyframe_time_tolerance_s_,
                pending_keyframe_queue_.size(), task_queue_.size());
            
            const auto skipped = ++skipped_non_keyframe_;
            if ((skipped % 200) == 0) {
                RCLCPP_INFO(node_->get_logger(),
                    "[SEMANTIC][Module][ENQUEUE] step=defer_frame total=%lu ts=%.3f kf_ts=%.3f pending=%zu (waiting for first valid kf_info)",
                    static_cast<unsigned long>(skipped), normalized_ev.timestamp, kf_ts, pending_keyframe_queue_.size());
            }
            return;
        }

        // 关键帧驱动：同一 kf_ts 只入队一次（每个关键帧处理一帧）
        const double last_kf = last_enqueued_kf_ts_.load(std::memory_order_relaxed);
        constexpr double kKfEps = 1e-4;
        if (std::isfinite(last_kf) && (kf_ts <= last_kf + kKfEps)) {
            const auto dup = ++skipped_duplicate_keyframe_;
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[SEMANTIC][Module][ENQUEUE] step=skip_duplicate_kf total=%lu kf_ts=%.3f last_enqueued_kf_ts=%.3f dt=%.3f",
                static_cast<unsigned long>(dup), kf_ts, last_kf, dt_to_kf);
            return;
        }

        last_enqueued_kf_ts_.store(kf_ts, std::memory_order_relaxed);
        // 首次拿到有效 kf_ts 时，清理此前无 kf 标记的 pending 缓存
        flushPendingKeyframeTasksLocked();
    }

    enqueueTaskLocked(normalized_ev);
    
    // 🏛️ [架构契约] 任务队列背压监测
    if (task_queue_.size() > kMaxQueueSize * 0.8) {
        BackpressureWarningEvent warn;
        warn.module_name = name_;
        warn.queue_usage_ratio = static_cast<float>(task_queue_.size()) / kMaxQueueSize;
        warn.critical = (task_queue_.size() >= kMaxQueueSize);
        event_bus_->publish(warn);
    }

    cv_.notify_one();
}

void SemanticModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][SemanticModule] Started worker thread");
    
    while (running_) {
        updateHeartbeat();
        tryRecoverFromDegradedState(node_->now().seconds());
        SyncedFrameEvent event;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                return !running_ || !task_queue_.empty(); 
            });
            if (!running_) break;
            if (task_queue_.empty()) {
                if (!pending_keyframe_queue_.empty()) {
                    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                        "[SEMANTIC][Module][RUN] step=idle_wait task_q=0 pending=%zu latest_kf_ts=%.3f",
                        pending_keyframe_queue_.size(),
                        latest_kf_ts_seen_.load(std::memory_order_relaxed));
                }
                continue;
            }

            event = task_queue_.front();
            task_queue_.pop_front();
        }

        processTask(event);
    }
}

void SemanticModule::processTask(const SyncedFrameEvent& event) {
    if (!semantic_processor_ || semantic_degraded_.load(std::memory_order_relaxed)) return;
    if (!semantic_processor_->hasRuntimeCapability()) {
        if (semantic_runtime_ready_.load(std::memory_order_relaxed)) {
            semantic_runtime_ready_.store(false, std::memory_order_relaxed);
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=disabled reason=processor_runtime_lost");
        }
        return;
    }

    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Module][RUN] step=start ts=%.3f pts=%zu",
        event.timestamp, (event.cloud && event.cloud->size() > 0) ? event.cloud->size() : 0);

    auto t0 = std::chrono::steady_clock::now();
    std::vector<CylinderLandmark::Ptr> landmarks;
    try {
        landmarks = semantic_processor_->process(event.cloud);
        consecutive_errors_ = 0; // 重置错误计数
    } catch (const std::exception& e) {
        int err_count = ++consecutive_errors_;
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=process_exception ts=%.3f consecutive_errors=%d error=%s",
            event.timestamp, err_count, e.what());
        
        if (err_count >= kMaxConsecutiveErrors) {
            semantic_runtime_ready_.store(false, std::memory_order_relaxed);
            semantic_degraded_.store(true, std::memory_order_relaxed);
            next_recovery_retry_s_.store(node_->now().seconds() + kRecoveryCooldownSec, std::memory_order_relaxed);
            recovery_attempts_.store(0, std::memory_order_relaxed);
            recovery_exhausted_reported_.store(false, std::memory_order_relaxed);
            RCLCPP_FATAL(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=degraded reason=too_many_errors threshold=%d → SemanticModule disabled to protect system stability",
                kMaxConsecutiveErrors);
        }
        return;
    }
    auto t1 = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ++processed_tasks_;

    if (!semantic_processor_->hasRuntimeCapability() && semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=disabled reason=processor_runtime_lost_after_process");
    }

    if (!landmarks.empty()) {
        // 🏛️ [架构契约] 发布前准入校验
        std::vector<CylinderLandmark::Ptr> valid_landmarks;
        for (const auto& l : landmarks) {
            if (l && l->isValid()) valid_landmarks.push_back(l);
        }

        if (!valid_landmarks.empty()) {
            SemanticLandmarkEvent res_ev;
            res_ev.timestamp = event.timestamp;
            res_ev.keyframe_timestamp_hint = event.kf_info.timestamp;
            if (std::isfinite(event.kf_info.timestamp) && event.kf_info.timestamp > 0.0) {
                auto kf = map_registry_->getKeyFrameByTimestamp(event.kf_info.timestamp, keyframe_time_tolerance_s_);
                if (kf) {
                    res_ev.keyframe_id_hint = kf->id;
                }
            }
            res_ev.landmarks = valid_landmarks;
            event_bus_->publish(res_ev);

            RCLCPP_INFO(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=%zu elapsed_ms=%.1f → published SemanticLandmarkEvent",
                event.timestamp, valid_landmarks.size(), elapsed_ms);
        } else {
            RCLCPP_DEBUG(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 (all failed validity check) elapsed_ms=%.1f",
                event.timestamp, elapsed_ms);
        }
    } else {
        ++no_landmark_tasks_;
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 elapsed_ms=%.1f (no publish)",
            event.timestamp, elapsed_ms);
    }

    // 慢帧告警不再依赖“有无地标”，避免吞掉真实性能问题
    if (elapsed_ms > 250.0) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=SLOW ts=%.3f elapsed_ms=%.1f threshold_ms=250.0",
            event.timestamp, elapsed_ms);
    }

    // 周期统计：帮助判断“模块运行正常但无有效输出”还是“算法/配置问题”
    const auto now = std::chrono::steady_clock::now();
    const auto secs_since_stats = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log_tp_).count();
    const auto secs_since_delta = std::chrono::duration_cast<std::chrono::seconds>(now - last_dt_delta_log_tp_).count();
    if (secs_since_stats >= 10) {
        last_stats_log_tp_ = now;
        const auto processed = processed_tasks_.load();
        const auto no_landmark = no_landmark_tasks_.load();
        const auto coalesced = coalesced_tasks_.load();
        const auto bp_drop = backpressure_drops_.load();
        const auto skipped_non_kf = skipped_non_keyframe_.load();
        const auto skipped_dup_kf = skipped_duplicate_keyframe_.load();
        const auto pending_requeued = pending_requeued_.load();
        const auto pending_dropped = pending_dropped_.load();
        const uint64_t dt_b0 = dt_hist_bins_[0].load(std::memory_order_relaxed);
        const uint64_t dt_b1 = dt_hist_bins_[1].load(std::memory_order_relaxed);
        const uint64_t dt_b2 = dt_hist_bins_[2].load(std::memory_order_relaxed);
        const uint64_t dt_b3 = dt_hist_bins_[3].load(std::memory_order_relaxed);
        const uint64_t dt_b4 = dt_hist_bins_[4].load(std::memory_order_relaxed);
        const uint64_t dt_b5 = dt_hist_bins_[5].load(std::memory_order_relaxed);
        const uint64_t dt_b6 = dt_hist_bins_[6].load(std::memory_order_relaxed);
        const uint64_t dt_missing = dt_hist_bins_[7].load(std::memory_order_relaxed);
        size_t pending_size = 0;
        std::vector<double> age_samples;
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            pending_size = pending_keyframe_queue_.size();
            age_samples.assign(pending_drop_age_samples_sec_.begin(), pending_drop_age_samples_sec_.end());
        }
        std::sort(age_samples.begin(), age_samples.end());
        const double age_p50 = percentileFromSorted(age_samples, 0.50);
        const double age_p90 = percentileFromSorted(age_samples, 0.90);
        const double age_p99 = percentileFromSorted(age_samples, 0.99);
        const double no_landmark_ratio =
            (processed > 0) ? (100.0 * static_cast<double>(no_landmark) / static_cast<double>(processed)) : 0.0;
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][STATS] processed=%lu no_landmark=%lu no_landmark_ratio=%.1f%% coalesced=%lu backpressure_drop=%lu skipped_non_keyframe=%lu skipped_duplicate_kf=%lu pending_requeued=%lu pending_dropped=%lu pending_size=%zu dt_hist=[<=0.01:%lu <=0.03:%lu <=0.05:%lu <=0.10:%lu <=0.20:%lu <=0.50:%lu >0.50:%lu missing:%lu] pending_age_s[p50=%.3f p90=%.3f p99=%.3f n=%zu]",
            static_cast<unsigned long>(processed),
            static_cast<unsigned long>(no_landmark),
            no_landmark_ratio,
            static_cast<unsigned long>(coalesced),
            static_cast<unsigned long>(bp_drop),
            static_cast<unsigned long>(skipped_non_kf),
            static_cast<unsigned long>(skipped_dup_kf),
            static_cast<unsigned long>(pending_requeued),
            static_cast<unsigned long>(pending_dropped),
            pending_size,
            static_cast<unsigned long>(dt_b0),
            static_cast<unsigned long>(dt_b1),
            static_cast<unsigned long>(dt_b2),
            static_cast<unsigned long>(dt_b3),
            static_cast<unsigned long>(dt_b4),
            static_cast<unsigned long>(dt_b5),
            static_cast<unsigned long>(dt_b6),
            static_cast<unsigned long>(dt_missing),
            age_p50, age_p90, age_p99, age_samples.size());
    }

    if (secs_since_delta >= 100) {
        last_dt_delta_log_tp_ = now;
        const uint64_t dt_now[8] = {dt_hist_bins_[0].load(std::memory_order_relaxed),
                                    dt_hist_bins_[1].load(std::memory_order_relaxed),
                                    dt_hist_bins_[2].load(std::memory_order_relaxed),
                                    dt_hist_bins_[3].load(std::memory_order_relaxed),
                                    dt_hist_bins_[4].load(std::memory_order_relaxed),
                                    dt_hist_bins_[5].load(std::memory_order_relaxed),
                                    dt_hist_bins_[6].load(std::memory_order_relaxed),
                                    dt_hist_bins_[7].load(std::memory_order_relaxed)};
        const uint64_t d0 = dt_now[0] - dt_hist_last_snapshot_[0];
        const uint64_t d1 = dt_now[1] - dt_hist_last_snapshot_[1];
        const uint64_t d2 = dt_now[2] - dt_hist_last_snapshot_[2];
        const uint64_t d3 = dt_now[3] - dt_hist_last_snapshot_[3];
        const uint64_t d4 = dt_now[4] - dt_hist_last_snapshot_[4];
        const uint64_t d5 = dt_now[5] - dt_hist_last_snapshot_[5];
        const uint64_t d6 = dt_now[6] - dt_hist_last_snapshot_[6];
        const uint64_t d7 = dt_now[7] - dt_hist_last_snapshot_[7];
        uint64_t delta_total = d0 + d1 + d2 + d3 + d4 + d5 + d6 + d7;
        const double align_ratio = (delta_total > 0)
            ? (100.0 * static_cast<double>(d0 + d1) / static_cast<double>(delta_total))
            : 0.0;
        for (size_t i = 0; i < 8; ++i) dt_hist_last_snapshot_[i] = dt_now[i];

        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][DT_DELTA_100S] bins_delta=[<=0.01:%lu <=0.03:%lu <=0.05:%lu <=0.10:%lu <=0.20:%lu <=0.50:%lu >0.50:%lu missing:%lu] total=%lu aligned(<=0.03)=%.1f%%",
            static_cast<unsigned long>(d0),
            static_cast<unsigned long>(d1),
            static_cast<unsigned long>(d2),
            static_cast<unsigned long>(d3),
            static_cast<unsigned long>(d4),
            static_cast<unsigned long>(d5),
            static_cast<unsigned long>(d6),
            static_cast<unsigned long>(d7),
            static_cast<unsigned long>(delta_total),
            align_ratio);
    }
}

bool SemanticModule::normalizeSemanticInputFrame(const SyncedFrameEvent& in, SyncedFrameEvent& out) const {
    out = in;
    if (in.cloud_frame == "body") return true;
    if (in.cloud_frame != "world") {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][CONTRACT] Reject frame: unsupported cloud_frame=%s",
            in.cloud_frame.c_str());
        return false;
    }
    if (in.pose_frame != PoseFrame::ODOM) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][CONTRACT] Reject frame: cloud_frame=world requires pose_frame=ODOM, got %d",
            static_cast<int>(in.pose_frame));
        return false;
    }
    if (!in.cloud || in.cloud->empty()) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][CONTRACT] Reject frame: world cloud is null/empty");
        return false;
    }

    CloudXYZIPtr body_cloud(new CloudXYZI());
    pcl::transformPointCloud(*in.cloud, *body_cloud, in.T_odom_b.inverse().cast<float>());
    if (body_cloud->empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][CONTRACT] Drop frame: world->body transform produced empty cloud");
        return false;
    }
    out.cloud = body_cloud;
    out.cloud_frame = "body";
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "[SEMANTIC][CONTRACT] Adapted input cloud_frame world->body for semantic processing");
    return true;
}

void SemanticModule::tryRecoverFromDegradedState(double now_s) {
    if (!semantic_degraded_.load(std::memory_order_relaxed)) return;
    const double next_retry = next_recovery_retry_s_.load(std::memory_order_relaxed);
    if (now_s < next_retry) return;

    const int attempts = recovery_attempts_.load(std::memory_order_relaxed);
    if (attempts >= kMaxRecoveryAttempts) {
        bool expected = false;
        if (recovery_exhausted_reported_.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][RECOVERY] Exhausted recovery budget (%d attempts). Semantic remains degraded until restart/manual intervention.",
                kMaxRecoveryAttempts);
            BackpressureWarningEvent warn;
            warn.module_name = name_ + "_recovery_exhausted";
            warn.queue_usage_ratio = 1.0f;
            warn.critical = true;
            event_bus_->publish(warn);
        }
        return;
    }

    recovery_attempts_.store(attempts + 1, std::memory_order_relaxed);
    RCLCPP_WARN(node_->get_logger(),
        "[SEMANTIC][RECOVERY] Attempt %d/%d to recover degraded semantic runtime",
        attempts + 1, kMaxRecoveryAttempts);
    try {
        auto new_processor = std::make_shared<SemanticProcessor>(semantic_cfg_);
        const bool ready = new_processor->hasRuntimeCapability();
        semantic_processor_ = new_processor;
        semantic_runtime_ready_.store(ready, std::memory_order_relaxed);
        if (ready) {
            semantic_degraded_.store(false, std::memory_order_relaxed);
            consecutive_errors_.store(0, std::memory_order_relaxed);
            recovery_exhausted_reported_.store(false, std::memory_order_relaxed);
            RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][RECOVERY] Semantic runtime recovered successfully");
            return;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][RECOVERY] Recovery attempt failed: %s", e.what());
    }
    next_recovery_retry_s_.store(now_s + kRecoveryCooldownSec, std::memory_order_relaxed);
}

} // namespace automap_pro::v3
