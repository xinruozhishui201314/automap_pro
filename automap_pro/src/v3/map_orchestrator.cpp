/**
 * @file v3/map_orchestrator.cpp
 * @brief V3 流水线模块实现。
 */
#include "automap_pro/v3/map_orchestrator.h"
#include "automap_pro/core/config_manager.h"

#include <chrono>
#include <deque>
#include <string>

namespace automap_pro::v3 {
namespace {
double nowSteadySec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

void MapOrchestrator::enqueueObserved(const char* event_name, const EventMeta& meta) {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    queue_.push_back({std::string(event_name), meta});
    cv_.notify_one();
}

MapOrchestrator::MapOrchestrator(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("MapOrchestrator", event_bus, map_registry), node_(node) {
    takeover_enabled_ = ConfigManager::instance().orchestratorTakeoverEnabled();

    // 全量带 EventMeta 的生产事件观测（不含 RouteAdviceEvent：本模块自产，避免自激）。
    onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) { enqueueObserved("SyncedFrameEvent", ev.meta); });
    onEvent<FilteredFrameEventRequiredDs>(
        [this](const FilteredFrameEventRequiredDs& ev) { enqueueObserved("FilteredFrameEventRequiredDs", ev.meta); });
    onEvent<OptimizationResultEvent>(
        [this](const OptimizationResultEvent& ev) { enqueueObserved("OptimizationResultEvent", ev.meta); });
    onEvent<OptimizationDeltaEvent>(
        [this](const OptimizationDeltaEvent& ev) { enqueueObserved("OptimizationDeltaEvent", ev.meta); });
    onEvent<GPSAlignedEvent>([this](const GPSAlignedEvent& ev) { enqueueObserved("GPSAlignedEvent", ev.meta); });
    onEvent<SemanticLandmarkEvent>(
        [this](const SemanticLandmarkEvent& ev) { enqueueObserved("SemanticLandmarkEvent", ev.meta); });
    onEvent<SemanticCloudEvent>([this](const SemanticCloudEvent& ev) { enqueueObserved("SemanticCloudEvent", ev.meta); });
    onEvent<SemanticTrunkVizEvent>(
        [this](const SemanticTrunkVizEvent& ev) { enqueueObserved("SemanticTrunkVizEvent", ev.meta); });
    onEvent<GraphTaskEvent>([this](const GraphTaskEvent& ev) { enqueueObserved("GraphTaskEvent", ev.meta); });
    onEvent<SemanticInputEvent>([this](const SemanticInputEvent& ev) { enqueueObserved("SemanticInputEvent", ev.meta); });
    onEvent<BackpressureWarningEvent>(
        [this](const BackpressureWarningEvent& ev) { enqueueObserved("BackpressureWarningEvent", ev.meta); });
    onEvent<SystemQuiesceRequestEvent>(
        [this](const SystemQuiesceRequestEvent& ev) { enqueueObserved("SystemQuiesceRequestEvent", ev.meta); });
}

bool MapOrchestrator::isIdle() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    return queue_.empty();
}

std::vector<std::pair<std::string, size_t>> MapOrchestrator::queueDepths() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    return {{"queue", queue_.size()}};
}

std::string MapOrchestrator::idleDetail() const {
    return takeover_enabled_ ? "takeover_enabled=1" : "";
}

void MapOrchestrator::run() {
    while (running_) {
        try {
            updateHeartbeat();
            std::deque<Item> drained;
            {
                std::unique_lock<std::mutex> lk(queue_mutex_);
                cv_.wait_for(lk, std::chrono::milliseconds(100), [this] { return !running_ || !queue_.empty(); });
                if (!running_) break;
                if (queue_.empty()) continue;
                drained.swap(queue_);
            }

            for (const auto& item : drained) {
                observeMeta(item.event_name.c_str(), item.meta);
            }
            size_t q_size = 0;
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                q_size = queue_.size();
            }
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[ORCH][KPI] observed=%lu out_of_order=%lu stale=%lu queue=%zu takeover=%d",
                static_cast<unsigned long>(observed_total_.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(out_of_order_total_.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(stale_total_.load(std::memory_order_relaxed)),
                q_size,
                takeover_enabled_ ? 1 : 0);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(),
                "[ORCH][EXCEPTION] run loop caught: %s (isolate+continue)", e.what());
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(),
                "[ORCH][EXCEPTION] run loop unknown exception (isolate+continue)");
        }
    }
}

void MapOrchestrator::observeMeta(const char* event_name, const EventMeta& meta) {
    observed_total_.fetch_add(1, std::memory_order_relaxed);
    if (!meta.isValid()) {
        maybePublishAdvice(event_name, meta, "invalid_meta", false);
        return;
    }

    const std::string key = std::string(meta.producer) + ":" + event_name;
    auto it = last_producer_seq_.find(key);
    if (it != last_producer_seq_.end() && meta.producer_seq <= it->second) {
        out_of_order_total_.fetch_add(1, std::memory_order_relaxed);
        maybePublishAdvice(event_name, meta, "out_of_order_or_duplicate", takeover_enabled_);
        return;
    }
    last_producer_seq_[key] = meta.producer_seq;

    const uint64_t current_epoch = map_registry_->getAlignmentEpoch();
    if (meta.ref_epoch != 0 && meta.ref_epoch < current_epoch) {
        stale_total_.fetch_add(1, std::memory_order_relaxed);
        maybePublishAdvice(event_name, meta, "stale_epoch", takeover_enabled_);
        return;
    }

    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[ORCH][OBSERVE] event=%s producer=%s seq=%lu ref_v=%lu ref_e=%lu route=%s",
        event_name, meta.producer.c_str(),
        static_cast<unsigned long>(meta.producer_seq),
        static_cast<unsigned long>(meta.ref_version),
        static_cast<unsigned long>(meta.ref_epoch),
        meta.route_tag.c_str());
}

void MapOrchestrator::maybePublishAdvice(
    const char* event_name, const EventMeta& meta, const char* reason, bool takeover) {
    RouteAdviceEvent advice;
    advice.meta = meta;
    advice.meta.event_id = (++advice_seq_);
    advice.meta.idempotency_key = advice.meta.event_id;
    advice.meta.producer_seq = advice.meta.event_id;
    advice.meta.ref_version = map_registry_->getVersion();
    advice.meta.ref_epoch = map_registry_->getAlignmentEpoch();
    advice.meta.session_id = map_registry_->getSessionId();
    advice.meta.source_ts = nowSteadySec();
    advice.meta.publish_ts = advice.meta.source_ts;
    advice.meta.producer = "MapOrchestrator";
    advice.meta.route_tag = "advice";
    advice.event_type = event_name;
    advice.suggested_owner = "legacy";
    advice.takeover_enabled = takeover;
    advice.fallback_to_legacy = true;
    advice.reason = reason;
    event_bus_->publish(advice);
}

}  // namespace automap_pro::v3
