// 模块7: 服务处理器
// 包含: handleSaveMap, handleGetStatus, handleTriggerHBA, handleTriggerOptimize, handleTriggerGpsAlign, handleLoadSession, handleFinishMapping

#include "automap_pro/system/automap_system.h"

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 服务处理函数
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::handleSaveMap(
    const std::shared_ptr<automap_pro::srv::SaveMap::Request> req,
    std::shared_ptr<automap_pro::srv::SaveMap::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] SaveMap to %s", req->output_dir.c_str());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_start output_dir=%s", req->output_dir.c_str());
    state_ = SystemState::SAVING;
    try {
        saveMapToFiles(req->output_dir);
        res->success     = true;
        res->output_path = req->output_dir;
        res->message     = "Map saved successfully";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_done output_dir=%s success=1", req->output_dir.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][EXCEPTION] SaveMap failed: %s", e.what());
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_done output_dir=%s success=0 error=%s", req->output_dir.c_str(), e.what());
        res->success = false;
        res->message = e.what();
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem][EXCEPTION] SaveMap failed: unknown exception");
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=save_map_done output_dir=%s success=0 error=unknown", req->output_dir.c_str());
        res->success = false;
        res->message = "unknown exception";
    }
    state_ = SystemState::MAPPING;
}

void AutoMapSystem::handleGetStatus(
    const std::shared_ptr<automap_pro::srv::GetStatus::Request>,
    std::shared_ptr<automap_pro::srv::GetStatus::Response> res)
{
    res->state         = stateToString(state_.load());
    res->session_id    = current_session_id_;
    res->keyframe_count = submap_manager_.keyframeCount();
    res->submap_count  = submap_manager_.submapCount();
    res->gps_aligned   = gps_aligned_.load();
}

void AutoMapSystem::handleTriggerHBA(
    const std::shared_ptr<automap_pro::srv::TriggerHBA::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerHBA::Response> res)
{
    auto all = submap_manager_.getAllSubmaps();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Triggered HBA (wait=%d)", req->wait_for_result);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=trigger_hba_called wait=%d submaps=%zu", req->wait_for_result ? 1 : 0, all.size());
    ensureBackendCompletedAndFlushBeforeHBA();
    hba_optimizer_.triggerAsync(all, req->wait_for_result, "TriggerHBA_srv");
    res->success = true;
    res->message = "HBA triggered";
}

void AutoMapSystem::handleTriggerOptimize(
    const std::shared_ptr<automap_pro::srv::TriggerOptimize::Request>,
    std::shared_ptr<automap_pro::srv::TriggerOptimize::Response> res)
{
    // 投递 FORCE_UPDATE 任务到 opt_worker 线程
    if (task_dispatcher_) {
        if (task_dispatcher_->submitForceUpdate()) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_SERVICE] FORCE_UPDATE task enqueued");
            
            // 等待 opt_worker 处理完成（带超时）
            const int max_wait_ms = 10000;
            const int check_interval_ms = 100;
            int waited_ms = 0;
            while (waited_ms < max_wait_ms) {
                bool queue_empty = false;
                {
                    std::lock_guard<std::mutex> lk(opt_task_mutex_);
                    queue_empty = opt_task_queue_.empty();
                }
                if (queue_empty && !opt_task_in_progress_.load(std::memory_order_acquire)) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
                waited_ms += check_interval_ms;
            }
            
            res->success = true;
            res->message = "Optimize completed (async)";
            return;
        }
    }
    
    // 降级方案
    RCLCPP_WARN(get_logger(), "[AutoMapSystem][OPT_SERVICE] task_dispatcher failed or null, using direct call");
    auto result = isam2_optimizer_.forceUpdate();
    res->success = result.success;
    res->message = "Optimize called directly (fallback)";
}

void AutoMapSystem::handleTriggerGpsAlign(
    const std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Request> req,
    std::shared_ptr<automap_pro::srv::TriggerGpsAlign::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_align_triggered force=%d", req->force ? 1 : 0);
    if (req->force) gps_manager_.triggerRealign();
    // 等待对齐完成（最多 5 秒）
    for (int i = 0; i < 50; ++i) {
        if (gps_manager_.isAligned()) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    const auto& r = gps_manager_.alignResult();
    res->success       = r.success;
    res->alignment_rmse_m = r.rmse_m;
    res->message = r.success ? "GPS aligned" : "GPS alignment failed";
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=gps_align_service_done success=%d rmse_m=%.3f", r.success ? 1 : 0, r.rmse_m);
    for (int i = 0; i < 9; ++i) res->r_gps_lidar[i] = r.R_gps_lidar(i/3, i%3);
    for (int i = 0; i < 3; ++i) res->t_gps_lidar[i] = r.t_gps_lidar[i];
}

void AutoMapSystem::handleLoadSession(
    const std::shared_ptr<automap_pro::srv::LoadSession::Request> req,
    std::shared_ptr<automap_pro::srv::LoadSession::Response> res)
{
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loading session from %s", req->session_dir.c_str());
    int loaded = 0;
    try {
        if (req->session_dir.empty()) {
            res->success = false;
            res->message = "session_dir is empty";
            res->submaps_loaded = 0;
            res->descriptors_loaded = 0;
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession: session_dir is empty");
            return;
        }
        for (auto& entry : fs::directory_iterator(req->session_dir)) {
            if (!entry.is_directory()) continue;
            std::string name = entry.path().filename().string();
            if (name.find("submap_") != 0) continue;

            int sm_id = 0;
            try {
                sm_id = std::stoi(name.substr(7));
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem] LoadSession: skip invalid dir '%s': %s", name.c_str(), e.what());
                continue;
            }
            SubMap::Ptr sm;
            if (submap_manager_.loadArchivedSubmap(req->session_dir, sm_id, sm)) {
                if (sm) {
                    sm->session_id = req->session_id;
                    loop_detector_.addToDatabase(sm);
                    loaded++;
                }
            }
        }
        res->success          = (loaded > 0);
        res->submaps_loaded   = loaded;
        res->descriptors_loaded = loaded;
        res->message          = std::to_string(loaded) + " submaps loaded";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] Loaded %d submaps from session %lu", loaded, req->session_id);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=session_loaded dir=%s submaps=%d success=%d", req->session_dir.c_str(), loaded, res->success ? 1 : 0);
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession filesystem error: %s", e.what());
        res->success = false;
        res->submaps_loaded = 0;
        res->descriptors_loaded = 0;
        res->message = std::string("filesystem error: ") + e.what();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession exception: %s", e.what());
        res->success = false;
        res->submaps_loaded = 0;
        res->descriptors_loaded = 0;
        res->message = std::string("exception: ") + e.what();
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] LoadSession unknown exception");
        res->success = false;
        res->submaps_loaded = 0;
        res->descriptors_loaded = 0;
        res->message = "unknown exception";
    }
}

void AutoMapSystem::handleFinishMapping(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (sensor_idle_finish_triggered_.exchange(true, std::memory_order_acq_rel)) {
        res->success = true;
        res->message = "finish_mapping already triggered";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] finish_mapping: already done, ignoring");
        return;
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_service (final HBA + save + shutdown)");
    try {
        if (submap_manager_.submapCount() > 0) {
            submap_manager_.forceFreezeActiveSubmapForFinish();
            auto all = submap_manager_.getAllSubmaps();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][HBA][TRACE] step=finish_mapping_ensureBackend_enter");
            ensureBackendCompletedAndFlushBeforeHBA();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][HBA][TRACE] step=finish_mapping_ensureBackend_done");
            if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnFinish()) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_final_hba_enter submaps=%zu", all.size());
                hba_optimizer_.triggerAsync(all, true, "finish_mapping");
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_final_hba_done");
            }
            std::string out_dir = getOutputDir();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_save_enter output_dir=%s", out_dir.c_str());
            saveMapToFiles(out_dir);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_save_done output_dir=%s", out_dir.c_str());
        }
        res->success = true;
        res->message = "Map saved, requesting shutdown";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] finish_mapping: requesting context shutdown (end mapping)");
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] finish_mapping failed: %s", e.what());
        res->success = false;
        res->message = std::string("exception: ") + e.what();
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "[AutoMapSystem] finish_mapping failed: unknown exception");
        res->success = false;
        res->message = "unknown exception";
    }
}

}  // namespace automap_pro
