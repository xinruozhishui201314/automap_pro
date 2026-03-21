// 模块7: 服务处理器
// 包含: handleSaveMap, handleGetStatus, handleTriggerHBA, handleTriggerOptimize, handleTriggerGpsAlign, handleLoadSession, handleFinishMapping

#include "automap_pro/system/automap_system.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/health_monitor.h"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <thread>

namespace automap_pro {

namespace fs = std::filesystem;

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
    std::vector<LoopConstraint::Ptr> loops;
    {
        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
        loops = loop_constraints_;
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem] Triggered HBA (wait=%d) loops=%zu", req->wait_for_result, loops.size());
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=trigger_hba_called wait=%d submaps=%zu loops=%zu", req->wait_for_result ? 1 : 0, all.size(), loops.size());
    ensureBackendCompletedAndFlushBeforeHBA();
    hba_optimizer_.triggerAsync(all, loops, req->wait_for_result, "TriggerHBA_srv");
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
            return;
        }
    }
    
    // 降级方案
    RCLCPP_WARN(get_logger(), "[AutoMapSystem][OPT_SERVICE] task_dispatcher failed or null, using direct call");
    auto result = isam2_optimizer_.forceUpdate();
    res->success = result.success;
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
                std::vector<LoopConstraint::Ptr> loops;
                {
                    std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
                    loops = loop_constraints_;
                }
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_final_hba_enter submaps=%zu loops=%zu", all.size(), loops.size());
                hba_optimizer_.triggerAsync(all, loops, true, "finish_mapping");
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_final_hba_done");
            }
            // 确保输出目录已初始化（含时间戳）
            ensureTrajectoryLogDir();
            std::string out_dir = getOutputDir();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_save_enter output_dir=%s", out_dir.c_str());
            saveMapToFiles(out_dir);
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=finish_mapping_save_done output_dir=%s", out_dir.c_str());
        }
        res->success = true;
        res->message = "Map saved, requesting shutdown";
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] finish_mapping: requesting context shutdown (end mapping)");
        
        // 🔧 关键修复：在 shutdown 之前显式停止所有子模块内部线程，打破循环引用并解决挂起问题
        shutdown_requested_.store(true, std::memory_order_release);

        const auto shutdown_start = std::chrono::steady_clock::now();
        auto elapsed_since_shutdown_ms = [&]() -> int64_t {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - shutdown_start).count();
        };

        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][finish_shutdown] step=before_stop_loop_detector elapsed_since_shutdown_ms=%ld",
            static_cast<long>(elapsed_since_shutdown_ms()));
        {
            const auto t_before = std::chrono::steady_clock::now();
            loop_detector_.stop();
            const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_before).count();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][finish_shutdown] step=after_stop_loop_detector stop_ms=%ld elapsed_since_shutdown_ms=%ld",
                static_cast<long>(stop_ms), static_cast<long>(elapsed_since_shutdown_ms()));
        }
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][finish_shutdown] step=before_stop_submap_manager elapsed_since_shutdown_ms=%ld",
            static_cast<long>(elapsed_since_shutdown_ms()));
        {
            const auto t_before = std::chrono::steady_clock::now();
            submap_manager_.stop();
            const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_before).count();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][finish_shutdown] step=after_stop_submap_manager stop_ms=%ld elapsed_since_shutdown_ms=%ld",
                static_cast<long>(stop_ms), static_cast<long>(elapsed_since_shutdown_ms()));
        }
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][finish_shutdown] step=before_stop_hba_optimizer elapsed_since_shutdown_ms=%ld",
            static_cast<long>(elapsed_since_shutdown_ms()));
        {
            const auto t_before = std::chrono::steady_clock::now();
            // 与 triggerAsync(..., wait=true) 的 5 分钟 idle 等待一致，避免 runHBA 内部卡住时无限 join
            hba_optimizer_.stopJoinWithTimeout(std::chrono::minutes(5));
            const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_before).count();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][finish_shutdown] step=after_stop_hba_optimizer stop_ms=%ld elapsed_since_shutdown_ms=%ld",
                static_cast<long>(stop_ms), static_cast<long>(elapsed_since_shutdown_ms()));
        }
        RCLCPP_INFO(get_logger(),
            "[AutoMapSystem][finish_shutdown] step=before_stop_health_monitor elapsed_since_shutdown_ms=%ld",
            static_cast<long>(elapsed_since_shutdown_ms()));
        {
            const auto t_before = std::chrono::steady_clock::now();
            HealthMonitor::instance().stop();
            const auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t_before).count();
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][finish_shutdown] step=after_stop_health_monitor stop_ms=%ld elapsed_since_shutdown_ms=%ld",
                static_cast<long>(stop_ms), static_cast<long>(elapsed_since_shutdown_ms()));
        }

        // 唤醒所有 AutoMapSystem 本地线程
        map_publish_cv_.notify_all();
        gps_queue_cv_.notify_all();
        loop_trigger_cv_.notify_all();
        intra_loop_task_cv_.notify_all();
        gps_align_cv_.notify_all();
        opt_task_cv_.notify_all();
        status_pub_cv_.notify_all();
        
        RCLCPP_INFO(get_logger(), "[AutoMapSystem] calling rclcpp::shutdown()...");
        rclcpp::shutdown();
        // MultiThreadedExecutor::spin() 在 shutdown 后仍可能阻塞：某 executor 线程卡在耗时订阅回调中。
        // 离线 launch 依赖本进程退出触发 OnProcessExit，从而结束 bag / fast_livo / rviz。地图已保存且模块已 stop。
        // 短延迟后 _Exit：给 DDS 时间发出 finish_mapping 响应；若 spin 已返回则 main 先结束进程，本线程随进程终止。
        std::thread([]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::_Exit(EXIT_SUCCESS);
        }).detach();
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
