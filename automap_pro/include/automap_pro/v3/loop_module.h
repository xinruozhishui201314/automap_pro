#pragma once
/**
 * @file v3/loop_module.h
 * @brief V3 微内核：模块编排、事件总线、Registry、前端/语义/优化流水线。
 */


#include "automap_pro/v3/module_base.h"
#include "automap_pro/loop_closure/loop_detector.h"

#include <deque>
#include <mutex>

namespace automap_pro::v3 {

/**
 * @brief 回环检测模块 (Loop Closure Micro-Kernel Module)
 * 
 * 职责：
 * 1. 独立运行回环检测
 * 2. 监听 MapUpdateEvent 中的 KEYFRAME_ADDED
 * 3. 产生 LoopConstraintEvent
 */
class LoopModule : public ModuleBase {
public:
    LoopModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
        : ModuleBase("LoopModule", event_bus, map_registry), node_(node) {
        
        loop_detector_.init(node_);
        
        // 注册回环回调
        loop_detector_.registerLoopCallback([this](const LoopConstraint::Ptr& lc) {
            LoopConstraintEvent ev;
            ev.constraint = lc;
            ev.meta.event_id = loop_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
            ev.meta.idempotency_key = ev.meta.event_id;
            ev.meta.producer_seq = ev.meta.event_id;
            ev.meta.ref_version = map_registry_->getVersion();
            ev.meta.ref_epoch = map_registry_->getAlignmentEpoch();
            ev.meta.source_ts = node_->now().seconds();
            ev.meta.publish_ts = ev.meta.source_ts;
            ev.meta.producer = "LoopModule";
            ev.meta.session_id = map_registry_->getSessionId();
            ev.meta.route_tag = "legacy";
            if (ev.isValid()) {
                event_bus_->publish(ev);
            }
        });

        // 订阅地图变更事件
        onEvent<MapUpdateEvent>([this](const MapUpdateEvent& ev) {
            if (ev.type == MapUpdateEvent::ChangeType::SUBMAP_ADDED) {
                for (int id : ev.affected_ids) {
                    auto sm = map_registry_->getSubMap(id);
                    if (sm) {
                        std::lock_guard<std::mutex> lock(queue_mutex_);
                        sm_queue_.push_back(sm);
                        cv_.notify_one();
                    }
                }
            } else if (ev.type == MapUpdateEvent::ChangeType::KEYFRAME_ADDED) {
                // 子图内回环：MapRegistry::addKeyFrame 在发布本事件后投递 IntraLoopTaskEvent（见 map_registry.cpp）。
            }
        });

        // 订阅子图内回环任务
        onEvent<IntraLoopTaskEvent>([this](const IntraLoopTaskEvent& ev) {
            std::lock_guard<std::mutex> lock(intra_mutex_);
            intra_tasks_.push_back(ev);
            cv_.notify_one();
        });
        RCLCPP_INFO(node_->get_logger(),
                    "[PIPELINE][LOOP] ctor OK LoopDetector+MapUpdate+IntraLoopTask subscribed");
    }

    void start() override {
        RCLCPP_INFO(node_->get_logger(), "[PIPELINE][LOOP] start step=LoopDetector::start then ModuleBase::start");
        loop_detector_.start();
        ModuleBase::start();
    }

    void stop() override {
        RCLCPP_INFO(node_->get_logger(), "[PIPELINE][LOOP] stop step=LoopDetector::stop");
        loop_detector_.stop();
        ModuleBase::stop();
    }

protected:
    void run() override {
        RCLCPP_INFO(node_->get_logger(), "[V3][LoopModule] Started worker thread");
        
        while (running_) {
            updateHeartbeat();
            SubMap::Ptr sm;
            IntraLoopTaskEvent intra_task;
            bool has_sm = false;
            bool has_intra = false;

            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                    return !running_ || !sm_queue_.empty() || !intra_tasks_.empty(); 
                });
                if (!running_) break;
                
                if (!sm_queue_.empty()) {
                    sm = sm_queue_.front();
                    sm_queue_.pop_front();
                    has_sm = true;
                } else if (!intra_tasks_.empty()) {
                    std::lock_guard<std::mutex> lock_intra(intra_mutex_);
                    intra_task = intra_tasks_.front();
                    intra_tasks_.pop_front();
                    has_intra = true;
                }
            }

            if (has_sm) {
                // 处理全局回环检测（提交给 LoopDetector，由其内部线程池处理）
                loop_detector_.addSubmap(sm);
            } else if (has_intra) {
                // 处理子图内回环检测（同步执行，类似于旧的 intraLoopWorkerLoop）
                auto loops = loop_detector_.detectIntraSubmapLoop(intra_task.submap, intra_task.query_idx);
                for (auto& lc : loops) {
                    LoopConstraintEvent ev;
                    ev.constraint = lc;
                    ev.meta.event_id = loop_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
                    ev.meta.idempotency_key = ev.meta.event_id;
                    ev.meta.producer_seq = ev.meta.event_id;
                    ev.meta.ref_version = map_registry_->getVersion();
                    ev.meta.ref_epoch = map_registry_->getAlignmentEpoch();
                    ev.meta.source_ts = node_->now().seconds();
                    ev.meta.publish_ts = ev.meta.source_ts;
                    ev.meta.producer = "LoopModule";
                    ev.meta.session_id = map_registry_->getSessionId();
                    ev.meta.route_tag = "legacy";
                    if (ev.isValid()) {
                        event_bus_->publish(ev);
                    }
                }
            }
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    LoopDetector loop_detector_;
    
    std::deque<SubMap::Ptr> sm_queue_;
    std::deque<IntraLoopTaskEvent> intra_tasks_;
    std::mutex queue_mutex_;
    std::mutex intra_mutex_;
    std::atomic<uint64_t> loop_event_seq_{0};
};

} // namespace automap_pro::v3
