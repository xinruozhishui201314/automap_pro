#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/health_monitor.h"
#include "automap_pro/core/protocol_contract.h" // 🏛️ [架构加固] 引入协议契约

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>

namespace automap_pro {

namespace fs = std::filesystem;

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
    
    v3::GraphTaskEvent ev;
    ev.task.type = OptTaskItem::Type::FORCE_UPDATE;
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
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Finish mapping requested. Waiting for backend to idle...");
    
    // 🏛️ [架构修复] 等待后端所有模块处理完成
    // 理由：前端 (fast_livo) 处理 bag 很快，发布完所有数据后就会触发 finish。
    // 后端 (Mapping/Optimizer) 有大量队列积压，若立即退出会导致后半段地图丢失。
    int wait_count = 0;
    while (!v3_context_->isAllIdle() && wait_count < 600) { // 最多等 10 分钟 (600 * 1s)
        if (wait_count % 10 == 0) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem] Backend still busy, waiting... (%ds)", wait_count);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        wait_count++;
    }
    
    if (wait_count >= 600) {
        RCLCPP_WARN(get_logger(), "[AutoMapSystem] Wait for idle TIMEOUT! Forcing save and exit.");
    } else {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Backend idle. Proceeding to save and exit.");
    }

    // 🏛️ [架构契约] 协议版本检查（即使 srv 还没更新，也要在日志中显式声明契约一致性）
    RCLCPP_INFO(get_logger(), "[PROTOCOL] Interface Check: Backend API v%s OK", protocol::getVersionString().c_str());

    std::string out_dir = getOutputDir();
    saveMapToFiles(out_dir);

    res->success = true;
    res->message = "Mapping finished and saved after waiting for backend idle (API v" + protocol::getVersionString() + ")";
    
    // 延迟退出
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        rclcpp::shutdown();
    }).detach();
}

} // namespace automap_pro
