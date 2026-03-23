#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/health_monitor.h"

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
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Finish mapping requested");
    
    std::string out_dir = getOutputDir();
    saveMapToFiles(out_dir);
    
    res->success = true;
    res->message = "Mapping finished and saved";
    
    // 延迟退出
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        rclcpp::shutdown();
    }).detach();
}

} // namespace automap_pro
