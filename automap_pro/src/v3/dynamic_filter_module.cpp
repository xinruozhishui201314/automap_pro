#include "automap_pro/v3/dynamic_filter_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>

namespace automap_pro::v3 {
namespace {
void atomicAddRelaxed(std::atomic<double>& target, double delta) {
    double old = target.load(std::memory_order_relaxed);
    while (!target.compare_exchange_weak(old, old + delta, std::memory_order_relaxed)) {
    }
}
} // namespace

DynamicFilterModule::DynamicFilterModule(
    EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("DynamicFilterModule", event_bus, map_registry), node_(node) {
    const auto& cfg = ConfigManager::instance();
    enabled_ = cfg.dynamicFilterEnabled();
    shadow_mode_ = cfg.dynamicFilterShadowMode();
    drop_if_queue_full_ = cfg.dynamicFilterDropIfQueueFull();
    queue_max_size_ = cfg.dynamicFilterQueueMaxSize();
    min_static_observations_ = cfg.dynamicFilterMinStaticObservations();
    voxel_capacity_ = cfg.dynamicFilterVoxelCapacity();
    voxel_size_ = cfg.dynamicFilterVoxelSize();
    min_range_m_ = cfg.dynamicFilterMinRangeM();
    max_range_m_ = cfg.dynamicFilterMaxRangeM();
    fault_injection_enabled_ = cfg.dynamicFilterFaultInjectionEnabled();
    fault_injection_mode_ = cfg.dynamicFilterFaultInjectionMode();
    fault_injection_every_n_frames_ = cfg.dynamicFilterFaultInjectionEveryNFrames();
    fault_injection_max_count_ = cfg.dynamicFilterFaultInjectionMaxCount();

    onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (static_cast<int>(input_queue_.size()) >= queue_max_size_) {
            if (drop_if_queue_full_) {
                input_queue_.pop_front();
                kpi_queue_drop_total_.fetch_add(1, std::memory_order_relaxed);
            } else {
                return;
            }
        }
        input_queue_.push_back(ev);
        cv_.notify_one();
    });

    RCLCPP_INFO(node_->get_logger(),
                "[V3][DynamicFilter] init enabled=%d shadow=%d queue_max=%d voxel=%.2f min_static_obs=%d",
                enabled_ ? 1 : 0, shadow_mode_ ? 1 : 0, queue_max_size_, voxel_size_,
                min_static_observations_);
    RCLCPP_INFO(node_->get_logger(),
                "[V3][SELF_CHECK][CONTRACT] module=DynamicFilter contract_guard_publish_required_ds=on hard_capacity=on stale_cleanup=on mapping_prequeue_guard=expected_on");
    RCLCPP_INFO(node_->get_logger(),
                "[V3][SELF_CHECK][FAULT_INJECTION] module=DynamicFilter enabled=%d mode=%s every_n_frames=%d max_count=%d",
                fault_injection_enabled_ ? 1 : 0, fault_injection_mode_.c_str(),
                fault_injection_every_n_frames_, fault_injection_max_count_);
}

bool DynamicFilterModule::isIdle() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    return input_queue_.empty();
}

std::vector<std::pair<std::string, size_t>> DynamicFilterModule::queueDepths() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    return {{"input_queue", input_queue_.size()}};
}

std::string DynamicFilterModule::idleDetail() const {
    if (!enabled_) return "enabled=0";
    if (shadow_mode_) return "shadow_mode=1";
    return "";
}

void DynamicFilterModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][DynamicFilter] Started worker thread");
    while (running_) {
        try {
            updateHeartbeat();
            SyncedFrameEvent ev;
            bool has_item = false;
            {
                std::unique_lock<std::mutex> lk(queue_mutex_);
                cv_.wait_for(lk, std::chrono::milliseconds(100),
                             [this]() { return !running_ || !input_queue_.empty(); });
                if (!running_) break;
                if (!input_queue_.empty()) {
                    ev = input_queue_.front();
                    input_queue_.pop_front();
                    has_item = true;
                }
            }
            if (!has_item) continue;
            processOne(ev);
            logKpiIfNeeded();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][DynamicFilter][EXCEPTION] run loop caught: %s (drop item, continue)", e.what());
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][DynamicFilter][EXCEPTION] run loop unknown exception (drop item, continue)");
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
        }
    }
}

void DynamicFilterModule::processOne(const SyncedFrameEvent& in) {
    const auto start = std::chrono::steady_clock::now();
    auto publish_required = [this](const FilteredFrameEventRequiredDs& ev) -> bool {
        FilteredFrameEventRequiredDs to_publish = ev;
        applyFaultInjectionIfEnabled(to_publish);
        if (!to_publish.isValid()) {
            kpi_contract_drop_total_.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][CONTRACT] Drop invalid FilteredFrameEventRequiredDs: ts=%.3f cloud=%s cloud_ds=%s frame=%s",
                to_publish.timestamp,
                (to_publish.cloud && !to_publish.cloud->empty()) ? "ok" : "bad",
                (to_publish.cloud_ds && !to_publish.cloud_ds->empty()) ? "ok" : "bad",
                to_publish.cloud_frame.c_str());
            return false;
        }
        event_bus_->publish(to_publish);
        return true;
    };

    if (!in.isValid()) {
        auto fallback = passthrough(in, FilterFallbackReason::INPUT_INVALID);
        fallback.filter_latency_ms = 0.0;
        auto out = makeRequiredEvent(fallback);
        kpi_processed_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_fallback_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_shadow_passthrough_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_input_points_total_.fetch_add(out.input_points, std::memory_order_relaxed);
        kpi_output_points_total_.fetch_add(out.output_points, std::memory_order_relaxed);
        publish_required(out);
        return;
    }

    if (!enabled_) {
        auto out = passthrough(in, FilterFallbackReason::FILTER_DISABLED);
        out.filter_latency_ms = 0.0;
        auto req = makeRequiredEvent(out);
        kpi_processed_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_shadow_passthrough_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_input_points_total_.fetch_add(req.input_points, std::memory_order_relaxed);
        kpi_output_points_total_.fetch_add(req.output_points, std::memory_order_relaxed);
        publish_required(req);
        return;
    }

    // DUFO 风格动态过滤依赖时序一致坐标系；body 系下难以累计稳定体素。
    if (in.cloud_frame != "world") {
        auto out = passthrough(in, FilterFallbackReason::CLOUD_FRAME_UNSUPPORTED);
        out.filter_latency_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                                    std::chrono::steady_clock::now() - start)
                                    .count();
        auto req = makeRequiredEvent(out);
        kpi_processed_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_fallback_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_shadow_passthrough_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_input_points_total_.fetch_add(req.input_points, std::memory_order_relaxed);
        kpi_output_points_total_.fetch_add(req.output_points, std::memory_order_relaxed);
        atomicAddRelaxed(kpi_latency_sum_ms_, req.filter_latency_ms);
        double old_max = kpi_latency_max_ms_.load(std::memory_order_relaxed);
        while (req.filter_latency_ms > old_max &&
               !kpi_latency_max_ms_.compare_exchange_weak(old_max, req.filter_latency_ms,
                                                          std::memory_order_relaxed)) {
        }
        publish_required(req);
        return;
    }

    FilteredFrameEventOptionalDs out;
    out.timestamp = in.timestamp;
    out.T_odom_b = in.T_odom_b;
    out.covariance = in.covariance;
    out.pose_frame = in.pose_frame;
    out.cloud_frame = in.cloud_frame;
    out.kf_info = in.kf_info;
    out.has_gps = in.has_gps;
    out.gps = in.gps;
    out.ref_map_version = in.ref_map_version;
    out.ref_alignment_epoch = in.ref_alignment_epoch;
    out.meta = in.meta;
    out.processing_state = in.processing_state;
    out.input_points = in.cloud ? in.cloud->size() : 0;
    out.filter_executed = true;
    out.filtered_output_used = false;
    out.fallback_reason = FilterFallbackReason::NONE;

    CloudXYZIPtr static_cloud(new CloudXYZI());
    static_cloud->reserve(in.cloud ? in.cloud->size() : 0);

    const bool apply_filter = !shadow_mode_;
    ++seq_id_;
    for (const auto& p : in.cloud->points) {
        if (!pointInRange(p)) continue;
        VoxelKey key = voxelize(p);
        auto& st = voxel_stats_[key];
        st.last_seen_seq = seq_id_;
        touchVoxelLru(key, st);
        if (st.observations < 1000000u) {
            st.observations += 1u;
        }
        if (st.observations >= static_cast<uint32_t>(min_static_observations_)) {
            static_cloud->push_back(p);
        }
    }

    // 次级策略：按年龄清理陈旧体素；硬上限仍由 enforceHardCapacity() 保证。
    cleanupStaleVoxelsByAge();
    enforceHardCapacity();

    if (!apply_filter) {
        out.cloud = in.cloud ? std::make_shared<CloudXYZI>(*in.cloud) : nullptr;
        out.cloud_ds = in.cloud_ds ? std::make_shared<CloudXYZI>(*in.cloud_ds) : out.cloud;
        out.filtered_output_used = false;
        out.fallback_reason = FilterFallbackReason::SHADOW_MODE;
        out.output_points = out.cloud ? out.cloud->size() : 0;
    } else if (static_cloud->empty()) {
        out.cloud = in.cloud ? std::make_shared<CloudXYZI>(*in.cloud) : nullptr;
        out.cloud_ds = in.cloud_ds ? std::make_shared<CloudXYZI>(*in.cloud_ds) : out.cloud;
        out.filtered_output_used = false;
        out.fallback_reason = FilterFallbackReason::EMPTY_STATIC;
        out.output_points = out.cloud ? out.cloud->size() : 0;
    } else {
        out.cloud = static_cloud;
        CloudXYZIPtr static_cloud_ds(new CloudXYZI());
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(static_cloud);
        const float ds_leaf = static_cast<float>(std::max(0.05, voxel_size_ * 0.5));
        vg.setLeafSize(ds_leaf, ds_leaf, ds_leaf);
        vg.filter(*static_cloud_ds);
        out.cloud_ds = static_cloud_ds->empty() ? static_cloud : static_cloud_ds;
        out.filtered_output_used = true;
        out.fallback_reason = FilterFallbackReason::NONE;
        out.output_points = out.cloud->size();
    }

    if (out.input_points > 0) {
        out.dynamic_ratio =
            1.0 - static_cast<double>(out.output_points) / static_cast<double>(out.input_points);
        if (out.dynamic_ratio < 0.0) out.dynamic_ratio = 0.0;
        if (out.dynamic_ratio > 1.0) out.dynamic_ratio = 1.0;
    } else {
        out.dynamic_ratio = 0.0;
    }
    out.filter_latency_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                                std::chrono::steady_clock::now() - start)
                                .count();

    auto req = makeRequiredEvent(out);

    if (!req.isValid()) {
        auto fallback = passthrough(in, FilterFallbackReason::INTERNAL_ERROR);
        fallback.filter_latency_ms = out.filter_latency_ms;
        auto fallback_req = makeRequiredEvent(fallback);
        kpi_processed_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_fallback_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_shadow_passthrough_total_.fetch_add(1, std::memory_order_relaxed);
        kpi_input_points_total_.fetch_add(fallback_req.input_points, std::memory_order_relaxed);
        kpi_output_points_total_.fetch_add(fallback_req.output_points, std::memory_order_relaxed);
        atomicAddRelaxed(kpi_latency_sum_ms_, fallback_req.filter_latency_ms);
        double old_max = kpi_latency_max_ms_.load(std::memory_order_relaxed);
        while (fallback_req.filter_latency_ms > old_max &&
               !kpi_latency_max_ms_.compare_exchange_weak(old_max, fallback_req.filter_latency_ms,
                                                          std::memory_order_relaxed)) {
        }
        publish_required(fallback_req);
        return;
    }

    kpi_processed_total_.fetch_add(1, std::memory_order_relaxed);
    if (req.filtered_output_used) {
        kpi_filtered_applied_total_.fetch_add(1, std::memory_order_relaxed);
    } else {
        kpi_shadow_passthrough_total_.fetch_add(1, std::memory_order_relaxed);
    }
    if (req.fallback_reason != FilterFallbackReason::NONE) {
        kpi_fallback_total_.fetch_add(1, std::memory_order_relaxed);
    }
    kpi_input_points_total_.fetch_add(req.input_points, std::memory_order_relaxed);
    kpi_output_points_total_.fetch_add(req.output_points, std::memory_order_relaxed);
    atomicAddRelaxed(kpi_latency_sum_ms_, req.filter_latency_ms);
    double old_max = kpi_latency_max_ms_.load(std::memory_order_relaxed);
    while (req.filter_latency_ms > old_max &&
           !kpi_latency_max_ms_.compare_exchange_weak(old_max, req.filter_latency_ms,
                                                      std::memory_order_relaxed)) {
    }
    publish_required(req);
}

FilteredFrameEventOptionalDs DynamicFilterModule::passthrough(
    const SyncedFrameEvent& in, FilterFallbackReason reason) const {
    FilteredFrameEventOptionalDs out;
    out.timestamp = in.timestamp;
    out.cloud = in.cloud ? std::make_shared<CloudXYZI>(*in.cloud) : nullptr;
    out.cloud_ds = in.cloud_ds ? std::make_shared<CloudXYZI>(*in.cloud_ds) : out.cloud;
    out.T_odom_b = in.T_odom_b;
    out.covariance = in.covariance;
    out.pose_frame = in.pose_frame;
    out.cloud_frame = in.cloud_frame;
    out.kf_info = in.kf_info;
    out.has_gps = in.has_gps;
    out.gps = in.gps;
    out.ref_map_version = in.ref_map_version;
    out.ref_alignment_epoch = in.ref_alignment_epoch;
    out.meta = in.meta;
    out.processing_state = in.processing_state;
    out.filter_executed = false;
    out.filtered_output_used = false;
    out.fallback_reason = reason;
    out.dynamic_ratio = 0.0;
    out.input_points = in.cloud ? in.cloud->size() : 0;
    out.output_points = out.input_points;
    return out;
}

FilteredFrameEventRequiredDs DynamicFilterModule::makeRequiredEvent(
    const FilteredFrameEventOptionalDs& in) const {
    FilteredFrameEventRequiredDs out;
    static_cast<FilteredFrameEventOptionalDs&>(out) = in;
    if (!out.cloud_ds || out.cloud_ds->empty()) {
        out.cloud_ds = out.cloud;
    }
    return out;
}

DynamicFilterModule::VoxelKey DynamicFilterModule::voxelize(const pcl::PointXYZI& p) const {
    const double inv = 1.0 / voxel_size_;
    VoxelKey key;
    key.x = static_cast<int>(std::floor(p.x * inv));
    key.y = static_cast<int>(std::floor(p.y * inv));
    key.z = static_cast<int>(std::floor(p.z * inv));
    return key;
}

bool DynamicFilterModule::pointInRange(const pcl::PointXYZI& p) const {
    const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
                      static_cast<double>(p.z) * p.z;
    const double min2 = min_range_m_ * min_range_m_;
    const double max2 = max_range_m_ * max_range_m_;
    return (r2 >= min2 && r2 <= max2);
}

void DynamicFilterModule::touchVoxelLru(const VoxelKey& key, VoxelStat& stat) {
    stat.lru_stamp = ++lru_tick_;
    lru_queue_.push_back({key, stat.lru_stamp});
}

void DynamicFilterModule::enforceHardCapacity() {
    if (lru_queue_.size() > voxel_stats_.size() * 8 + 1024) {
        std::vector<std::pair<VoxelKey, uint64_t>> compact;
        compact.reserve(voxel_stats_.size());
        for (const auto& [k, v] : voxel_stats_) compact.push_back({k, v.lru_stamp});
        std::sort(compact.begin(), compact.end(),
                  [](const auto& a, const auto& b) { return a.second < b.second; });
        lru_queue_.clear();
        for (const auto& e : compact) lru_queue_.push_back(e);
    }
    while (static_cast<int>(voxel_stats_.size()) > voxel_capacity_ && !lru_queue_.empty()) {
        const auto [key, stamp] = lru_queue_.front();
        lru_queue_.pop_front();
        auto it = voxel_stats_.find(key);
        if (it == voxel_stats_.end()) continue;
        // 仅当队列记录仍是当前最新访问记录时才淘汰（旧记录自动失效）
        if (it->second.lru_stamp == stamp) voxel_stats_.erase(it);
    }
}

void DynamicFilterModule::cleanupStaleVoxelsByAge() {
    const uint64_t stale_before = (seq_id_ > 2000) ? (seq_id_ - 2000) : 0;
    for (auto it = voxel_stats_.begin(); it != voxel_stats_.end();) {
        if (it->second.last_seen_seq < stale_before) {
            it = voxel_stats_.erase(it);
        } else {
            ++it;
        }
    }
}

void DynamicFilterModule::applyFaultInjectionIfEnabled(FilteredFrameEventRequiredDs& ev) {
    if (!fault_injection_enabled_) return;
    if (fault_injection_mode_ == "none") return;
    if (fault_injection_every_n_frames_ <= 0) return;

    const uint64_t processed = kpi_processed_total_.load(std::memory_order_relaxed);
    if (processed == 0 || (processed % static_cast<uint64_t>(fault_injection_every_n_frames_)) != 0) return;

    const uint64_t done = fault_injection_done_count_.load(std::memory_order_relaxed);
    if (fault_injection_max_count_ > 0 && done >= static_cast<uint64_t>(fault_injection_max_count_)) return;

    if (fault_injection_mode_ == "null_cloud_ds") {
        ev.cloud_ds.reset();
    } else if (fault_injection_mode_ == "invalid_cloud_frame") {
        ev.cloud_frame = "invalid_frame";
    } else if (fault_injection_mode_ == "null_cloud") {
        ev.cloud.reset();
    } else if (fault_injection_mode_ == "nan_timestamp") {
        ev.timestamp = std::numeric_limits<double>::quiet_NaN();
    } else {
        return;
    }

    fault_injection_done_count_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN(node_->get_logger(),
                "[V3][FAULT_INJECTION] injected mode=%s done=%lu",
                fault_injection_mode_.c_str(),
                static_cast<unsigned long>(fault_injection_done_count_.load(std::memory_order_relaxed)));
}

void DynamicFilterModule::logKpiIfNeeded() {
    const uint64_t processed = kpi_processed_total_.load(std::memory_order_relaxed);
    if (processed == 0) return;
    const uint64_t dropped = kpi_queue_drop_total_.load(std::memory_order_relaxed);
    const uint64_t contract_drop = kpi_contract_drop_total_.load(std::memory_order_relaxed);
    const uint64_t fallback = kpi_fallback_total_.load(std::memory_order_relaxed);
    const uint64_t filtered = kpi_filtered_applied_total_.load(std::memory_order_relaxed);
    const uint64_t passthrough = kpi_shadow_passthrough_total_.load(std::memory_order_relaxed);
    const uint64_t in_pts = kpi_input_points_total_.load(std::memory_order_relaxed);
    const uint64_t out_pts = kpi_output_points_total_.load(std::memory_order_relaxed);
    const double latency_sum = kpi_latency_sum_ms_.load(std::memory_order_relaxed);
    const double latency_max = kpi_latency_max_ms_.load(std::memory_order_relaxed);

    const double avg_latency = latency_sum / static_cast<double>(std::max<uint64_t>(1, processed));
    const double keep_ratio = (in_pts > 0) ? (static_cast<double>(out_pts) / static_cast<double>(in_pts)) : 1.0;
    const double drop_ratio = static_cast<double>(dropped) / static_cast<double>(std::max<uint64_t>(1, processed + dropped));

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "[KPI][DynamicFilter] processed=%lu filtered_output_used=%lu passthrough=%lu fallback=%lu dropped=%lu contract_drop=%lu drop_ratio=%.3f keep_ratio=%.3f latency_ms(avg=%.2f,max=%.2f)",
        static_cast<unsigned long>(processed),
        static_cast<unsigned long>(filtered),
        static_cast<unsigned long>(passthrough),
        static_cast<unsigned long>(fallback),
        static_cast<unsigned long>(dropped),
        static_cast<unsigned long>(contract_drop),
        drop_ratio, keep_ratio, avg_latency, latency_max);
}

} // namespace automap_pro::v3
