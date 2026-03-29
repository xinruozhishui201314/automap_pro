#pragma once

#include "automap_pro/v3/module_base.h"
#include "automap_pro/frontend/gps_manager.h"

#include <deque>
#include <mutex>

namespace automap_pro::v3 {

/**
 * @brief GPS 模块 (GPS Micro-Kernel Module)
 * 
 * 职责：
 * 1. 独立处理 GPS 数据的解析和对齐逻辑
 * 2. 监听 RawGPSEvent 事件并进行处理
 * 3. 产生 GPSAlignedEvent 和 GPSFactorEvent 事件
 */
class GPSModule : public ModuleBase {
public:
    GPSModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
        : ModuleBase("GPSModule", event_bus, map_registry), node_(node) {
        
        // 初始化内部依赖 (保持 V2 的逻辑，但移入微内核)
        gps_manager_.applyConfig();
        
        // 注册对齐回调
        // 对齐状态写入 MapRegistry 的唯一入口：MappingModule::updateGPSAlignment（避免与 GPSModule 双写）
        gps_manager_.registerAlignCallback([this](const GPSAlignResult& r) {
            GPSAlignedEvent event;
            event.success = r.success;
            event.event_seq = gps_align_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
            // 仅携带当前观测到的 epoch 作为提示值，不做 +1 预测。
            event.alignment_epoch = map_registry_->getAlignmentEpoch();
            if (r.success) {
                event.R_enu_to_map = r.R_enu_to_map;
                event.t_enu_to_map = r.t_enu_to_map;
                event.R_enu_odom = r.R_gps_lidar;
                event.rmse = r.rmse_m;
            } else {
                event.R_enu_to_map = Eigen::Matrix3d::Identity();
                event.t_enu_to_map = Eigen::Vector3d::Zero();
                event.R_enu_odom = Eigen::Matrix3d::Identity();
                event.rmse = 0.0;
            }
            event.meta.event_id = event.event_seq;
            event.meta.idempotency_key = event.event_seq;
            event.meta.producer_seq = event.event_seq;
            event.meta.ref_version = map_registry_->getVersion();
            event.meta.ref_epoch = event.alignment_epoch;
            event.meta.source_ts = node_->now().seconds();
            event.meta.publish_ts = node_->now().seconds();
            event.meta.producer = "GPSModule";
            event.meta.route_tag = "legacy";
            event.meta.session_id = map_registry_->getSessionId();

            if (event.isValid()) {
                RCLCPP_DEBUG(node_->get_logger(),
                    "[V3][DIAG] step=GPSAlignedEvent_publish success=%d rmse=%.3fm (grep V3 DIAG)",
                    r.success ? 1 : 0, r.rmse_m);
                event_bus_->publish(event);
            } else {
                RCLCPP_WARN(node_->get_logger(), "[V3][GPS] REJECTING invalid GPSAlignedEvent: NaN detected!");
            }
        });

        // 注册对 RawGPSEvent 的订阅
        onEvent<RawGPSEvent>([this](const RawGPSEvent& ev) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            gps_queue_.push_back(ev);
            cv_.notify_one();
        });

        // 注册对 RawOdometryEvent 的订阅 (用于 GPSManager 更新)
        onEvent<RawOdometryEvent>([this](const RawOdometryEvent& ev) {
            // 这里可能在 EventBus 线程，建议也入队异步处理，但 addKeyFramePose 内部持锁且快
            gps_manager_.addKeyFramePose(ev.timestamp, ev.pose);
        });

        // 订阅对齐请求
        onEvent<GPSAlignRequestEvent>([this](const GPSAlignRequestEvent& ev) {
            if (ev.force) {
                gps_manager_.triggerRealign();
            } else {
                gps_manager_.requestAlignment();
            }
        });
        RCLCPP_INFO(node_->get_logger(),
                    "[PIPELINE][GPS] ctor OK GPSManager+events RawGPS/Odom/AlignRequest");
    }

protected:
    void run() override {
        RCLCPP_INFO(node_->get_logger(), "[V3][GPSModule] Started worker thread");
        
        while (running_) {
            updateHeartbeat();
            RawGPSEvent event;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this] { return !running_ || !gps_queue_.empty(); });
                if (!running_) break;
                
                event = gps_queue_.front();
                gps_queue_.pop_front();
            }

            // 处理逻辑
            gps_manager_.addGPSMeasurement(event.timestamp, event.lat, event.lon, event.alt, event.hdop, event.sats);
            
            // 自动调度对齐逻辑已在 GPSManager 内部 addGPSMeasurement 触发
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    GPSManager gps_manager_;
    
    std::deque<RawGPSEvent> gps_queue_;
    std::mutex queue_mutex_;
    std::atomic<uint64_t> gps_align_event_seq_{0};
};

} // namespace automap_pro::v3
