#include "automap_pro/v3/semantic_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"

namespace automap_pro::v3 {

SemanticModule::SemanticModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("SemanticModule", event_bus, map_registry), node_(node) {
    
    const auto& cfg = ConfigManager::instance();
    if (!cfg.semanticEnabled()) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][INIT] semantic.enabled=false in config but semantic is forced ON by product policy");
    }

    SemanticProcessor::Config s_cfg;
    s_cfg.model_path = cfg.semanticModelPath();
    s_cfg.fov_up = cfg.semanticFovUp();
    s_cfg.fov_down = cfg.semanticFovDown();
    s_cfg.img_w = cfg.semanticImgW();
    s_cfg.img_h = cfg.semanticImgH();
    s_cfg.do_destagger = cfg.semanticDoDestagger();

    semantic_processor_ = std::make_shared<SemanticProcessor>(s_cfg);

    // 订阅同步帧事件
    onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
        if (!running_.load()) return;
        if (!semantic_processor_) return;

        std::lock_guard<std::mutex> lock(queue_mutex_);
        // 🏛️ [产品化加固] 背压策略：若积压过多，丢弃最旧的任务
        if (task_queue_.size() >= kMaxQueueSize) {
            double dropped_ts = task_queue_.front().timestamp;
            task_queue_.pop_front();
            METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][ENQUEUE] step=backpressure_drop ts=%.3f queue_full=%zu → dropped oldest",
                dropped_ts, kMaxQueueSize);
        }
        task_queue_.push_back(ev);
        cv_.notify_one();
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Module][ENQUEUE] step=ok ts=%.3f pts=%zu queue_size=%zu",
            ev.timestamp, (ev.cloud && ev.cloud->size() > 0) ? ev.cloud->size() : 0, task_queue_.size());
    });

    RCLCPP_INFO(node_->get_logger(),
        "[SEMANTIC][Module][INIT] step=ok enabled=%d queue_max=%zu",
        (semantic_processor_ != nullptr) ? 1 : 0, kMaxQueueSize);
}

void SemanticModule::start() {
    ModuleBase::start();
}

void SemanticModule::stop() {
    ModuleBase::stop();
}

void SemanticModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][SemanticModule] Started worker thread");
    
    while (running_) {
        updateHeartbeat();
        SyncedFrameEvent event;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                return !running_ || !task_queue_.empty(); 
            });
            if (!running_) break;
            if (task_queue_.empty()) continue;

            event = task_queue_.front();
            task_queue_.pop_front();
        }

        processTask(event);
    }
}

void SemanticModule::processTask(const SyncedFrameEvent& event) {
    if (!semantic_processor_) return;

    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Module][RUN] step=start ts=%.3f pts=%zu",
        event.timestamp, (event.cloud && event.cloud->size() > 0) ? event.cloud->size() : 0);

    auto t0 = std::chrono::steady_clock::now();
    auto landmarks = semantic_processor_->process(event.cloud);
    auto t1 = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (!landmarks.empty()) {
        SemanticLandmarkEvent res_ev;
        res_ev.timestamp = event.timestamp;
        res_ev.landmarks = landmarks;
        event_bus_->publish(res_ev);

        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=%zu elapsed_ms=%.1f → published SemanticLandmarkEvent",
            event.timestamp, landmarks.size(), elapsed_ms);

        if (elapsed_ms > 500.0) {
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=SLOW ts=%.3f elapsed_ms=%.1f (>500ms threshold)",
                event.timestamp, elapsed_ms);
        }
    } else {
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 elapsed_ms=%.1f (no publish)",
            event.timestamp, elapsed_ms);
    }
}

} // namespace automap_pro::v3
