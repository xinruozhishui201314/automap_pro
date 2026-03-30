/**
 * @file system/modules/service_handlers.cpp
 * @brief 系统节点与 ROS 服务实现。
 */
#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/protocol_contract.h" // 🏛️ [架构加固] 引入协议契约

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>

namespace automap_pro {

namespace fs = std::filesystem;

namespace {
std::atomic<uint64_t> g_trigger_optimize_seq{0};
}

void AutoMapSystem::handleSaveMap(
    const std::shared_ptr<automap_pro::srv::SaveMap::Request> req,
    std::shared_ptr<automap_pro::srv::SaveMap::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] SaveMap requested to %s", req->output_dir.c_str());
    state_ = SystemState::SAVING;
    
    v3::SaveMapRequestEvent ev;
    ev.output_dir = req->output_dir;
    v3_context_->eventBus()->publish(ev);
    
    res->success     = true;
    res->output_path = req->output_dir;
    res->message     = "SaveMap command published to MappingModule";
    
    state_ = SystemState::MAPPING;
}

void AutoMapSystem::handleGetStatus(
    const std::shared_ptr<automap_pro::srv::GetStatus::Request>,
    std::shared_ptr<automap_pro::srv::GetStatus::Response> res)
{
    if (!v3_context_ || !v3_context_->mapRegistry()) return;
    
    auto& registry = *v3_context_->mapRegistry();
    
    res->state         = stateToString(state_.load());
    res->session_id    = current_session_id_;
    res->keyframe_count = static_cast<uint32_t>(registry.keyframeCount());
    res->submap_count  = static_cast<uint32_t>(registry.submapCount());
    res->gps_aligned   = registry.isGPSAligned();
}

void AutoMapSystem::handleTriggerHBA(
    const std::shared_ptr<automap_pro::srv::TriggerHBA::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerHBA::Response> res)
{
    if (!v3_context_) return;
    
    v3::HBARequestEvent ev;
    ev.wait_for_result = req->wait_for_result;
    v3_context_->eventBus()->publish(ev);
    
    res->success = true;
    res->message = "HBA trigger event published";
}

void AutoMapSystem::handleTriggerOptimize(
    const std::shared_ptr<automap_pro::srv::TriggerOptimize::Request>,
    std::shared_ptr<automap_pro::srv::TriggerOptimize::Response> res)
{
    if (!v3_context_) return;

    auto reg = v3_context_->mapRegistry();
    if (!reg) return;

    const double ts = now().seconds();
    const uint64_t seq = g_trigger_optimize_seq.fetch_add(1, std::memory_order_relaxed) + 1;
    uint64_t eid = (seq << 24) ^ static_cast<uint64_t>(std::max(0.0, ts) * 1e6);
    if (eid == 0) {
        eid = 1;
    }

    v3::GraphTaskEvent ev;
    ev.task.type = OptTaskItem::Type::FORCE_UPDATE;
    ev.meta.event_id = eid;
    ev.meta.idempotency_key = eid;
    ev.meta.producer_seq = seq;
    ev.meta.session_id = reg->getSessionId();
    ev.meta.ref_version = reg->getVersion();
    ev.meta.ref_epoch = reg->getAlignmentEpoch();
    ev.meta.source_ts = ts;
    ev.meta.publish_ts = ts;
    ev.meta.producer = "AutoMapSystem";
    ev.meta.route_tag = "service";
    if (!ev.isValid()) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] TriggerOptimize: GraphTaskEvent failed EventMeta contract");
        res->success = false;
        return;
    }
    v3_context_->eventBus()->publish(ev);

    res->success = true;
    res->elapsed_seconds = 0.0;
    res->nodes_updated = 0;
}

void AutoMapSystem::handleTriggerGpsAlign(
    const std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Response> res)
{
    if (!v3_context_) return;
    
    v3::GPSAlignRequestEvent ev;
    ev.force = req->force;
    v3_context_->eventBus()->publish(ev);
    
    // 等待一小段时间检查结果 (可选，或者直接返回成功)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    auto registry = v3_context_->mapRegistry();
    res->success = registry->isGPSAligned();
    res->alignment_rmse_m = static_cast<float>(registry->getGPSRMSE());
    res->message = res->success ? "GPS Aligned" : "GPS Alignment requested/failed";
    
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    registry->getGPSTransform(R, t);
    for (int i = 0; i < 9; ++i) res->r_gps_lidar[i] = R(i/3, i%3);
    for (int i = 0; i < 3; ++i) res->t_gps_lidar[i] = t[i];
}

void AutoMapSystem::handleLoadSession(
    const std::shared_ptr<automap_pro::srv::LoadSession::Request> req,
    std::shared_ptr<automap_pro::srv::LoadSession::Response> res)
{
    if (!v3_context_ || !v3_context_->mapRegistry()) {
        res->success = false;
        res->message = "V3 Context not ready";
        return;
    }
    
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loading session from %s", req->session_dir.c_str());
    
    v3_context_->mapRegistry()->loadSession(req->session_dir, req->session_id);
    
    auto all_sm = v3_context_->mapRegistry()->getAllSubMaps();
    int count = 0;
    for (const auto& sm : all_sm) {
        if (sm->session_id == req->session_id) count++;
    }
    
    res->success = (count > 0);
    res->submaps_loaded = count;
    res->message = std::to_string(count) + " submaps loaded from session";
}

void AutoMapSystem::handleFinishMapping(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (finish_mapping_in_progress_.exchange(true)) {
        res->success = false;
        res->message = "Finish mapping already in progress";
        RCLCPP_WARN(get_logger(), "[AutoMapSystem] Finish mapping request ignored: already in progress.");
        return;
    }

    finish_mapping_requested_.store(true);
    res->success = true;
    res->message = "Finish mapping accepted; running asynchronously";
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Finish mapping accepted. Starting async finalize task.");

    std::thread([this]() {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Async finish task started. Sending QUIESCE request...");

        // 🏛️ [架构契约] 发布静默请求，进入排空阶段
        v3::SystemQuiesceRequestEvent quiesce_ev;
        quiesce_ev.enable = true;
        quiesce_ev.reason = "FinishMappingRequest";
        if (auto reg = v3_context_->mapRegistry()) {
            const double ts = now().seconds();
            const uint64_t seq = g_trigger_optimize_seq.fetch_add(1, std::memory_order_relaxed) + 1;
            uint64_t eid = (seq << 24) ^ static_cast<uint64_t>(std::max(0.0, ts) * 1e6);
            if (eid == 0) {
                eid = 1;
            }
            quiesce_ev.meta.event_id = eid;
            quiesce_ev.meta.idempotency_key = eid;
            quiesce_ev.meta.producer_seq = seq;
            quiesce_ev.meta.session_id = reg->getSessionId();
            quiesce_ev.meta.ref_version = reg->getVersion();
            quiesce_ev.meta.ref_epoch = reg->getAlignmentEpoch();
            quiesce_ev.meta.source_ts = ts;
            quiesce_ev.meta.publish_ts = ts;
            quiesce_ev.meta.producer = "AutoMapSystem";
            quiesce_ev.meta.route_tag = "quiesce";
        }
        if (quiesce_ev.isValid()) {
            v3_context_->eventBus()->publish(quiesce_ev);
        } else {
            RCLCPP_ERROR(get_logger(),
                "[AutoMapSystem] Finish mapping: skip QUIESCE publish (missing MapRegistry / invalid EventMeta)");
        }

        int wait_count = 0;
        while (!v3_context_->isAllIdle() && wait_count < 600) { // 最多等 10 分钟 (600 * 1s)
            if (wait_count % 10 == 0) {
                auto snap = v3_context_->getIdleStatusSnapshot();
                std::string busy;
                for (const auto& s : snap) {
                    if (s.is_idle) continue;
                    if (!busy.empty()) busy += " | ";
                    busy += s.name + "(quiesce=" + (s.is_quiescing ? "1" : "0") +
                            ",hb_age_s=" + std::to_string(s.heartbeat_age_s).substr(0, 6);
                    if (!s.queue_depths.empty()) {
                        busy += ",queues=";
                        bool first_q = true;
                        for (const auto& q : s.queue_depths) {
                            if (!first_q) busy += ",";
                            first_q = false;
                            busy += q.first + ":" + std::to_string(q.second);
                        }
                    }
                    if (!s.idle_detail.empty()) {
                        busy += ",detail=" + s.idle_detail;
                    }
                    busy += ")";
                }
                if (busy.empty()) busy = "<none>";
                RCLCPP_INFO(get_logger(),
                            "[AutoMapSystem] Backend still busy (draining queues), waiting... (%ds) non_idle=%s",
                            wait_count, busy.c_str());
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            wait_count++;
        }

        if (wait_count >= 600) {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem] Wait for idle TIMEOUT! Forcing save and exit.");
        } else {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] All modules QUIESCED and IDLE. Proceeding to finalize (freeze/HBA) then save.");
        }

        // 🏛️ [收尾] 强制冻结活跃子图，使最后一段轨迹与 onSubmapFrozen 链路一致；随后排空因新图任务产生的后端工作。
        if (mapping_module_) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Finish: force-freeze active submap (if any) for complete graph...");
            mapping_module_->forceFreezeActiveSubmapForFinish();
        } else {
            RCLCPP_WARN(get_logger(), "[AutoMapSystem] Finish: mapping_module_ null, skip force-freeze");
        }

        int drain_count = 0;
        while (!v3_context_->isAllIdle() && drain_count < 300) {
            if (drain_count % 10 == 0) {
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem] Finish: draining post-freeze graph tasks... (%ds)", drain_count);
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            drain_count++;
        }
        if (drain_count >= 300) {
            RCLCPP_WARN(get_logger(),
                "[AutoMapSystem] Finish: post-freeze drain TIMEOUT (300s); continuing to HBA/save anyway.");
        }

        // 🏛️ [收尾] backend.hba.trigger_on_finish：全图 HBA（wait=true）后再存档；配置关闭则跳过。
        if (mapping_module_) {
            const bool did_hba = mapping_module_->runFinalFullHbaBlockingIfConfigured();
            if (did_hba) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem] Finish: final full HBA worker returned; draining apply/rebuild...");
                int post_hba = 0;
                while (!v3_context_->isAllIdle() && post_hba < 120) {
                    if (post_hba % 10 == 0) {
                        RCLCPP_INFO(get_logger(),
                            "[AutoMapSystem] Finish: post-HBA idle wait... (%ds)", post_hba);
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    post_hba++;
                }
                if (post_hba >= 120) {
                    RCLCPP_WARN(get_logger(),
                        "[AutoMapSystem] Finish: post-HBA idle TIMEOUT (120s); save may see slightly stale viz state.");
                }
                RCLCPP_INFO(get_logger(), "[AutoMapSystem] Finish: final full HBA path done.");
            }
        }

        // 🏛️ [架构契约] 协议版本检查（即使 srv 还没更新，也要在日志中显式声明契约一致性）
        RCLCPP_INFO(get_logger(), "[PROTOCOL] Interface Check: Backend API v%s OK", protocol::getVersionString().c_str());

        std::string out_dir = getOutputDir();
        
        // 🏛️ [架构加固] 显式触发一次同步保存命令
        v3::SaveMapRequestEvent save_ev;
        save_ev.output_dir = out_dir;
        save_ev.completion = std::make_shared<std::promise<void>>();
        auto future = save_ev.completion->get_future();
        v3_context_->eventBus()->publish(save_ev);
        
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Waiting for final save completion...");
        if (future.wait_for(std::chrono::minutes(5)) == std::future_status::ready) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Final save COMPLETED.");
        } else {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] Final save TIMEOUT (5min)!");
        }

        std::this_thread::sleep_for(std::chrono::seconds(2));
        finish_mapping_in_progress_.store(false);
        rclcpp::shutdown();
    }).detach();
}

} // namespace automap_pro
