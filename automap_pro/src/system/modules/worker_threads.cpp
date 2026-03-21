// 模块3: 工作线程
// 包含: feederLoop, backendWorkerLoop, mapPublishLoop, statusPublisherLoop
// 注意：loopOptThreadLoop 和 vizThreadLoop 已删除

#include "automap_pro/system/automap_system.h"
#include "automap_pro/config/thread_config.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_monitor.h"

namespace automap_pro {

namespace {
const char* optTaskTypeName(OptTaskItem::Type t) {
    switch (t) {
        case OptTaskItem::Type::LOOP_FACTOR: return "LOOP_FACTOR";
        case OptTaskItem::Type::GPS_FACTOR: return "GPS_FACTOR";
        case OptTaskItem::Type::SUBMAP_NODE: return "SUBMAP_NODE";
        case OptTaskItem::Type::ODOM_FACTOR: return "ODOM_FACTOR";
        case OptTaskItem::Type::REBUILD: return "REBUILD";
        case OptTaskItem::Type::GPS_ALIGN_COMPLETE: return "GPS_ALIGN_COMPLETE";
        case OptTaskItem::Type::RESET: return "RESET";
        case OptTaskItem::Type::FORCE_UPDATE: return "FORCE_UPDATE";
        case OptTaskItem::Type::INTRA_LOOP_BATCH: return "INTRA_LOOP_BATCH";
        case OptTaskItem::Type::KEYFRAME_CREATE: return "KEYFRAME_CREATE";
        case OptTaskItem::Type::ACTIVE_SUBMAP_GPS_BIND: return "ACTIVE_SUBMAP_GPS_BIND";
        default: return "UNKNOWN";
    }
}
}  // namespace

void AutoMapSystem::feederLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_feeder");
#endif
    // const int feeder_tid = static_cast<int>(syscall(__NR_gettid));
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] frame_processor_ starting, delegating to FrameProcessor");
    feeder_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);

    // 启动 FrameProcessor 的内部处理线程
    frame_processor_.start();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] frame_processor_ started, waiting for shutdown");

    // 无数据时阻塞在 cv 上，不空转；析构时 status_pub_cv_.notify_all() 会唤醒本线程
    {
        std::unique_lock<std::mutex> lock(status_pub_mutex_);
        status_pub_cv_.wait(lock, [this] { return shutdown_requested_.load(std::memory_order_acquire); });
        // 唤醒即更新心跳，避免 shutdown 前误报 Feeder stale（正常建图期仍依赖 checkThreadHeartbeats 跳过 Feeder）
        feeder_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] shutdown requested, stopping frame_processor_");
    frame_processor_.stop();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][FEEDER] frame_processor_ stopped");
}

void AutoMapSystem::backendWorkerLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_backend");
#endif

    ALOG_TRACE_STEP("AutoMapSystem", "backendWorkerLoop_start");

    const std::string cloud_frame = ConfigManager::instance().frontendCloudFrame();
    int process_every_n_config = ConfigManager::instance().backendProcessEveryNFrames();
    // 配置参数校验：确保值 >= 1，避免除零错误
    if (process_every_n_config < 1) {
        process_every_n_config = 1;
        RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][CONFIG] Invalid backendProcessEveryNFrames=%d, using default 1",
                    ConfigManager::instance().backendProcessEveryNFrames());
    }
    const int override_n = process_every_n_override_.load(std::memory_order_acquire);
    const int process_every_n = (override_n > 0) ? override_n : process_every_n_config;
    if (process_every_n > 1) {
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][CONFIG] process_every_n_frames=%d (skip %d frames per processed frame)%s",
                    process_every_n, process_every_n - 1, override_n > 0 ? " [degradation override]" : "");
    }
    static thread_local int no_odom_skip_count = 0;
    const double idle_timeout_sec = ConfigManager::instance().sensorIdleTimeoutSec();
    const bool auto_finish = ConfigManager::instance().autoFinishOnSensorIdle();
    backend_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    static auto s_last_backend_heartbeat_wall = std::chrono::steady_clock::now();
    constexpr int BACKEND_HEARTBEAT_INTERVAL_SEC = 30;

    // 新增：性能统计
    static std::atomic<int64_t> total_processing_time_ms{0};
    static std::atomic<int> processed_frame_count{0};

    // const auto backend_start_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] backendWorkerLoop started at t=0, waiting for frames...");

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        backend_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
        static int loop_iter = 0;
        static auto loop_start_time = std::chrono::steady_clock::now();
        loop_iter++;
        auto now_wall = std::chrono::steady_clock::now();
        double loop_elapsed_sec = std::chrono::duration<double>(now_wall - loop_start_time).count();
        if (loop_iter % 10 == 1) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] loop_iter=%d elapsed_sec=%.1f", loop_iter, loop_elapsed_sec);
        }

        FrameToProcess f;
        size_t qs_after_wait = 0;
        bool woke_by_data = false;
        {
            // 使用 FrameProcessor 获取帧
            const size_t queue_size_before = frame_processor_.frameQueueSize();
            const size_t ingress_size_before = frame_processor_.ingressQueueSize();
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] waiting for frame... frame_queue=%zu ingress=%zu",
                        queue_size_before, ingress_size_before);

            const auto wait_start = std::chrono::steady_clock::now();
            woke_by_data = frame_processor_.tryPopFrame(2000, f);
            const auto wait_end = std::chrono::steady_clock::now();
            const int64_t wait_ms = std::chrono::duration_cast<std::chrono::milliseconds>(wait_end - wait_start).count();

            qs_after_wait = frame_processor_.frameQueueSize();
            const size_t ingress_after = frame_processor_.ingressQueueSize();

            if (shutdown_requested_.load(std::memory_order_acquire)) break;

            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] wakeup: woke=%d queue=%zu->%zu ingress=%zu->%zu wait_ms=%ld",
                        woke_by_data ? 1 : 0, queue_size_before, qs_after_wait,
                        ingress_size_before, ingress_after, wait_ms);

            if (!woke_by_data) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] wait_done timeout queue=%zu", qs_after_wait);
            }

            if (!woke_by_data && frame_processor_.frameQueueSize() == 0) {
                const bool offline_finish_after_bag = ConfigManager::instance().offlineFinishAfterBag();
                if (offline_finish_after_bag) {
                } else {
                    auto now = std::chrono::steady_clock::now();
                    double idle_sec = std::chrono::duration<double>(now - last_sensor_data_wall_time_).count();
                    if (auto_finish && !sensor_idle_finish_triggered_.load(std::memory_order_acquire) &&
                        first_cloud_logged_.load(std::memory_order_acquire) && idle_sec >= idle_timeout_sec) {
                        sensor_idle_finish_triggered_.store(true, std::memory_order_release);
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE][IDLE] event=sensor_idle_timeout idle_sec=%.1f timeout=%.1f submaps=%d",
                                    idle_sec, idle_timeout_sec, submap_manager_.submapCount());
                        try {
                            if (submap_manager_.submapCount() > 0) {
                                if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnFinish()) {
                                    auto all = submap_manager_.getAllSubmaps();
                                    std::vector<LoopConstraint::Ptr> loops;
                                    {
                                        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
                                        loops = loop_constraints_;
                                    }
                                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE][HBA] event=sensor_idle_final_hba_enter submaps=%zu loops=%zu", all.size(), loops.size());
                                    ensureBackendCompletedAndFlushBeforeHBA();
                                    hba_optimizer_.triggerAsync(all, loops, true, "sensor_idle");
                                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE][HBA] event=sensor_idle_final_hba_done");
                                }
                                std::string out_dir = getOutputDir();
                                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE][SAVE] event=sensor_idle_save_enter output_dir=%s", out_dir.c_str());
                                saveMapToFiles(out_dir);
                                RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE][SAVE] event=sensor_idle_save_done output_dir=%s", out_dir.c_str());
                            }
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] Sensor idle: requesting context shutdown");
                            rclcpp::shutdown();
                        } catch (const std::exception& e) {
                            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][PIPELINE][ERROR] Sensor idle finish failed: %s", e.what());
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=auto_finish_failed error=%s", e.what());
                        } catch (...) {
                            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][PIPELINE][ERROR] Sensor idle finish failed: unknown");
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=auto_finish_failed error=unknown");
                        }
                        break;
                    }
                }
                // [GHOSTING_FIX] finish 已触发时不再由 frontend_idle 触发 HBA，避免双次写回导致重影
                // 需满足 frontend_idle_min_submaps，避免第一个子图刚建完就 HBA
                const double idle_trigger_sec = ConfigManager::instance().hbaFrontendIdleTriggerSec();
                const int idle_min_submaps = ConfigManager::instance().hbaFrontendIdleMinSubmaps();
                const int sm_count = submap_manager_.submapCount();
                const bool idle_submaps_ok = (idle_min_submaps <= 0) ? (sm_count > 0) : (sm_count >= idle_min_submaps);
                if (idle_trigger_sec > 0 && ConfigManager::instance().hbaEnabled() &&
                    !sensor_idle_finish_triggered_.load(std::memory_order_acquire) &&
                    first_cloud_logged_.load(std::memory_order_acquire) && idle_submaps_ok) {
                    auto now_idle = std::chrono::steady_clock::now();
                    double frontend_idle_sec = std::chrono::duration<double>(now_idle - last_sensor_data_wall_time_).count();
                    if (frontend_idle_sec >= idle_trigger_sec && !hba_triggered_by_frontend_idle_.exchange(true)) {
                        const size_t ingress_sz = frame_processor_.ingressQueueSize();
                        if (ingress_sz == 0) {
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE][HBA] event=hba_triggered_by_frontend_idle frontend_idle_sec=%.1f submaps=%d (min_submaps=%d)",
                                        frontend_idle_sec, submap_manager_.submapCount(), idle_min_submaps);
                            ensureBackendCompletedAndFlushBeforeHBA();
                            auto all = submap_manager_.getAllSubmaps();
                            std::vector<LoopConstraint::Ptr> loops;
                            {
                                std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
                                loops = loop_constraints_;
                            }
                            hba_optimizer_.triggerAsync(all, loops, false, "frontend_idle");
                            continue;
                        }
                    }
                }
                {
                    auto now_wall = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::seconds>(now_wall - s_last_backend_heartbeat_wall).count() >= BACKEND_HEARTBEAT_INTERVAL_SEC) {
                        s_last_backend_heartbeat_wall = now_wall;
                        const int popped = backend_cloud_frames_processed_.load(std::memory_order_acquire);
                        const int processed = backend_frames_actually_processed_.load(std::memory_order_acquire);
                        const int livo_cloud = livo_bridge_.cloudCount();
                        const int livo_odom  = livo_bridge_.odomCount();
                        const size_t ingress_sz = frame_processor_.ingressQueueSize();
                        const int64_t avg_time = processed > 0 ? total_processing_time_ms.load() / processed : 0;
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][HEARTBEAT] popped=%d processed=%d livo_cloud=%d livo_odom=%d ingress=%zu avg_time=%ldms",
                                    popped, processed, livo_cloud, livo_odom, ingress_sz, avg_time);
                        static int s_last_heartbeat_livo_cloud = -1;
                        static int s_last_heartbeat_livo_odom  = -1;
                        static auto s_last_stall_check_wall     = now_wall;
                        const auto elapsed_stall_sec = std::chrono::duration_cast<std::chrono::seconds>(now_wall - s_last_stall_check_wall).count();
                        if (elapsed_stall_sec >= 60 && s_last_heartbeat_livo_cloud >= 0 &&
                            livo_cloud == s_last_heartbeat_livo_cloud && livo_odom == s_last_heartbeat_livo_odom) {
                            RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND_STALL][WARN] 超过 60s 无新数据");
                        }
                        s_last_heartbeat_livo_cloud = livo_cloud;
                        s_last_heartbeat_livo_odom  = livo_odom;
                        s_last_stall_check_wall     = now_wall;
                    }
                }
                continue;
            }
        }
        const int frame_no = backend_cloud_frames_processed_.fetch_add(1) + 1;
        size_t qs_after_pop = frame_processor_.frameQueueSize();

        {
            auto now_wall = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(now_wall - s_last_backend_heartbeat_wall).count() >= BACKEND_HEARTBEAT_INTERVAL_SEC) {
                s_last_backend_heartbeat_wall = now_wall;
                const int processed_so_far = backend_frames_actually_processed_.load(std::memory_order_acquire);
                const int64_t avg_time = processed_so_far > 0 ? total_processing_time_ms.load() / processed_so_far : 0;
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][HEARTBEAT] popped=%d processed=%d queue=%zu last_ts=%.3f avg_time=%ldms",
                            frame_no, processed_so_far, qs_after_pop, f.ts, avg_time);
            }
        }

        try {
            const int pevery = (process_every_n_override_.load(std::memory_order_acquire) > 0)
                ? process_every_n_override_.load(std::memory_order_acquire) : process_every_n_config;
            const bool will_process = ((frame_no - 1) % pevery == 0);
            if (frame_no <= 50) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][POP] frame_no=%d ts=%.3f action=%s queue=%zu",
                            frame_no, f.ts, will_process ? "process" : "skip", qs_after_pop);
            }
            if (!will_process) {
                if (frame_no > 50 && (frame_no % 500 == 0)) {
                    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][SKIP] frame_no=%d ts=%.3f (process_every_n=%d)", frame_no, f.ts, pevery);
                }
                continue;
            }
            const int processed_no = backend_frames_actually_processed_.fetch_add(1) + 1;
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] about_to_process frame_no=%d processed_no=%d", frame_no, processed_no);
            if (frame_no <= 20) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] popped frame_no=%d ts=%.3f queue=%zu session_id=%lu",
                            frame_no, f.ts, qs_after_pop, current_session_id_);
            }
            if (processed_no <= 35 || processed_no % 100 == 0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][CRASH_CONTEXT] step=backend_worker_processing session_id=%lu frame_no=%d processed_no=%d ts=%.3f",
                            current_session_id_, frame_no, processed_no, f.ts);
            }
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND][DIAG] about_to_lock keyframe_mutex processed_no=%d", processed_no);

            Pose3d pose = Pose3d::Identity();
            Mat66d cov  = Mat66d::Identity() * 1e-4;
            if (!odomCacheGet(f.ts, pose, cov)) {
                no_odom_skip_count++;
                if (no_odom_skip_count <= 15) {
                    RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][WARN] no odom in cache ts=%.3f frame_no=%d skip #%d", f.ts, frame_no, no_odom_skip_count);
                    RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=skip reason=no_odom_in_cache frame_no=%d ts=%.3f pts=%zu", frame_no, f.ts, f.cloud ? f.cloud->size() : 0u);
                } else {
                    rclcpp::Clock::SharedPtr clk = get_clock();
                    if (clk) RCLCPP_WARN_THROTTLE(get_logger(), *clk, 2000, "[AutoMapSystem][BACKEND][WARN] no odom in cache ts=%.3f frame_no=%d, skip", f.ts, frame_no);
                }
                continue;
            }
            LivoKeyFrameInfo kfinfo_copy;
            if (!kfinfoCacheGet(f.ts, kfinfo_copy)) {
                std::lock_guard<std::mutex> lk(data_mutex_);
                kfinfo_copy = last_livo_info_;
            }
            kf_manager_.updateLivoInfo(kfinfo_copy);
            if (processed_no <= 5) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] tryCreateKeyFrame entered processed_no=%d ts=%.3f pts=%zu",
                            processed_no, f.ts, f.cloud ? f.cloud->size() : 0u);
            }
            CloudXYZIPtr cloud_for_kf = f.cloud;
            if (cloud_frame == "world" && f.cloud && !f.cloud->empty()) {
                cloud_for_kf = transformWorldToBody(f.cloud, pose);
                if (processed_no == 1) {
                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] first frame: cloud converted world->body pts=%zu", cloud_for_kf->size());
                }
            }
            using Clock = std::chrono::steady_clock;
            const auto t0 = Clock::now();
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=tryCreateKeyFrame_enter processed_no=%d ts=%.3f", processed_no, f.ts);
            try {
                // 关键帧创建与入图由 tryCreateKeyFrame 投递 KEYFRAME_CREATE，由 opt_worker 执行 addKeyFrame 与 ISAM2，避免后端持大锁（DESIGN_AVOID_BACKEND_BLOCKING.md）
                tryCreateKeyFrame(f.ts, pose, cov, cloud_for_kf, &kfinfo_copy,
                    (f.cloud_ds && !f.cloud_ds->empty()) ? &f.cloud_ds : nullptr);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker session_id=%lu frame_no=%d ts=%.3f: %s", current_session_id_, frame_no, f.ts, e.what());
                RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=fail reason=exception session_id=%lu frame_no=%d ts=%.3f what=%s",
                           current_session_id_, frame_no, f.ts, e.what());
                ErrorMonitor::instance().recordException(e, errors::LIVO_ODOMETRY_FAILED);
            } catch (...) {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker session_id=%lu frame_no=%d ts=%.3f: unknown", current_session_id_, frame_no, f.ts);
                RCLCPP_INFO(get_logger(), "[TRACE] step=backend_worker result=fail reason=unknown_exception session_id=%lu frame_no=%d ts=%.3f",
                           current_session_id_, frame_no, f.ts);
                ErrorMonitor::instance().recordError(ErrorDetail(errors::UNKNOWN_ERROR, "backend worker unknown exception"));
            }
            const double duration_ms = 1e-6 * std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count();

            // 更新性能统计
            total_processing_time_ms.fetch_add(static_cast<int64_t>(duration_ms));

            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STEP] step=tryCreateKeyFrame_exit processed_no=%d duration_ms=%.1f", processed_no, duration_ms);
            if (processed_no % 100 == 0 || duration_ms > 5000.0) {
                const int64_t avg_time = processed_no > 0 ? total_processing_time_ms.load() / processed_no : 0;
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][DIAG] frame_processed_done frame_no=%d processed_no=%d ts=%.3f duration_ms=%.1f avg=%ldms",
                            frame_no, processed_no, f.ts, duration_ms, avg_time);
            }
            if (duration_ms > 5000.0 && duration_ms <= 30000.0) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][SLOW] processed_no=%d ts=%.3f duration_ms=%.1f (>5s)", processed_no, f.ts, duration_ms);
            }
            if (duration_ms > 60000.0) {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][STUCK] processed_no=%d ts=%.3f duration_ms=%.1f (>60s) -投递ISAM2 reset任务!",
                            processed_no, f.ts, duration_ms);
                // 统一入口：投递任务到 opt_worker 线程执行 reset，避免在 backend 线程直接操作
                bool enqueued = false;
                const int max_waits = 10;
                const int wait_ms = 100;
                for (int wait_count = 0; wait_count < max_waits; ++wait_count) {
                    {
                        std::lock_guard<std::mutex> lk(opt_task_mutex_);
                        if (opt_task_queue_.size() < kMaxOptTaskQueueSize) {
                            OptTaskItem task;
                            task.type = OptTaskItem::Type::RESET;
                            opt_task_queue_.push_back(task);
                            opt_task_cv_.notify_one();
                            enqueued = true;
                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND][STUCK] ISAM2 reset task enqueued after %d waits", wait_count);
                            break;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                }
                if (!enqueued) {
                    RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][STUCK] opt_task_queue full after %d waits, cannot enqueue RESET!", max_waits);
                    // 🔧 V2 修复：不再直接调用 reset()，因为可能导致与 opt_worker 的死锁。
                    // 仅记录错误，心跳监控会检测到持续卡住。
                }
            } else if (duration_ms > 30000.0) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND][STUCK] processed_no=%d ts=%.3f duration_ms=%.1f (>30s) - possible commitAndUpdate deadlock!",
                            processed_no, f.ts, duration_ms);
            }
            const int map_interval = ConfigManager::instance().backendPublishGlobalMapEveryNProcessed();
            const int safe_map_interval = (map_interval > 0) ? map_interval : 10;
            if (map_interval <= 0) {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][BACKEND] Invalid backendPublishGlobalMapEveryNProcessed=%d, using %d",
                            map_interval, safe_map_interval);
            }
            if ((processed_no <= 3 || duration_ms > 200.0) && processed_no % safe_map_interval != 0) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] worker processed #%d ts=%.3f duration_ms=%.1f", processed_no, f.ts, duration_ms);
            } else if (processed_no % safe_map_interval == 0 || duration_ms > 200.0) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker processed #%d ts=%.3f duration_ms=%.1f", processed_no, f.ts, duration_ms);
            }
            if (processed_no <= 1 || processed_no % 10 == 0) {
                status_publish_pending_.store(true, std::memory_order_release);
                status_pub_cv_.notify_one();
            }
            if (processed_no % 50 == 0) {
                data_flow_publish_pending_.store(true, std::memory_order_release);
                status_pub_cv_.notify_one();
            }
            if (processed_no % map_interval == 0) {
                RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][BACKEND] processed_no=%d step=publishGlobalMap request (async, every_n=%d)", processed_no, map_interval);
                map_publish_pending_.store(true);
                map_publish_cv_.notify_one();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker round session_id=%lu frame_no=%d: %s", current_session_id_, frame_no, e.what());
        } catch (...) {
            RCLCPP_ERROR(get_logger(), "[AutoMapSystem][BACKEND][EXCEPTION] worker round session_id=%lu frame_no=%d: unknown", current_session_id_, frame_no);
        }
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][BACKEND] worker thread exiting");
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][PIPELINE] event=backend_worker_exited");
}

void AutoMapSystem::mapPublishLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_mappub");
#endif
    map_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        map_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
        std::unique_lock<std::mutex> lock(map_publish_mutex_);
        const bool map_woke = map_publish_cv_.wait_for(lock, std::chrono::seconds(5), [this] {
            return shutdown_requested_.load(std::memory_order_acquire) || map_publish_pending_.load(std::memory_order_acquire);
        });
        if (shutdown_requested_.load(std::memory_order_acquire)) break;
        if (!map_woke) {
            RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][MAP_PUB] wait_done reason=timeout");
            continue;
        }
        map_publish_pending_.store(false, std::memory_order_release);
        
        // 🔧 V2 修复：如果 HBA 正在运行，延迟发布地图，避免 HBA 写回期间点云与位姿不一致导致重影
        if (!hba_optimizer_.isIdle()) {
            RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP_PUB] HBA is busy, deferring publishGlobalMap...");
            map_publish_pending_.store(true, std::memory_order_release);
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        
        lock.unlock();
        const int processed = backend_frames_actually_processed_.load(std::memory_order_acquire);
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP_PUB] start processed=%d (async)", processed);
        publishGlobalMap();
        RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP_PUB] done processed=%d", processed);
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][MAP_PUB] thread exiting");
}

// 注意：loopOptThreadLoop 已删除，回环约束直接在 onLoopDetected 中投递到 opt_task_queue_
// 消除了原有的 loop_factor_queue_ 嵌套队列

void AutoMapSystem::gpsWorkerLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_gps");
#endif
    gps_worker_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_WORKER] thread started");
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        gps_worker_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
        GPSQueueItem gps_item;
        bool has_data = false;
        {
            std::unique_lock<std::mutex> lock(gps_queue_mutex_);
            const bool gps_woke = gps_queue_cv_.wait_for(lock, std::chrono::seconds(1), [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !gps_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!gps_woke || gps_queue_.empty()) {
                // 定期运行GPS对齐调度
                gps_manager_.runScheduledAlignment();
                continue;
            }
            gps_item = gps_queue_.front();
            gps_queue_.pop_front();
            has_data = true;
        }
        if (has_data) {
            gps_manager_.addGPSMeasurement(gps_item.timestamp, gps_item.lat, gps_item.lon,
                                          gps_item.alt, gps_item.hdop, gps_item.sats);
        }
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_WORKER] thread exiting");
}

// ─────────────────────────────────────────────────────────────────────────────
// 回环检测触发线程：从loop_trigger_queue取出关键帧并触发回环检测
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::loopTriggerThreadLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_loop_trig");
#endif
    loop_trigger_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP_TRIGGER] thread started");

    const auto wait_timeout = std::chrono::milliseconds(500);

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        loop_trigger_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);

        KeyFrame::Ptr kf;
        {
            std::unique_lock<std::mutex> lock(loop_trigger_mutex_);
            const bool has_data = loop_trigger_cv_.wait_for(lock, wait_timeout, [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !loop_trigger_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!has_data || loop_trigger_queue_.empty()) continue;
            kf = std::move(loop_trigger_queue_.front());
            loop_trigger_queue_.pop_front();
        }

        if (kf && loop_detector_.isRunning()) {
            loop_detector_.addKeyFrame(kf);
        }
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][LOOP_TRIGGER] thread exiting");
}

// ─────────────────────────────────────────────────────────────────────────────
// 子图内回环 worker：与主线程完全异步，后端只投递任务，本线程执行 detectIntraSubmapLoop 并投递 INTRA_LOOP_BATCH 到 opt_worker
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::intraLoopWorkerLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_intra_loop");
#endif
    intra_loop_worker_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INTRA_LOOP_WORKER] thread started (async from backend)");

    const auto wait_timeout = std::chrono::milliseconds(500);

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        intra_loop_worker_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);

        IntraLoopTask task;
        bool has_task = false;
        {
            std::unique_lock<std::mutex> lock(intra_loop_task_mutex_);
            const bool has_data = intra_loop_task_cv_.wait_for(lock, wait_timeout, [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !intra_loop_task_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!has_data || intra_loop_task_queue_.empty()) continue;
            task = std::move(intra_loop_task_queue_.front());
            intra_loop_task_queue_.pop_front();
            has_task = true;
        }

        if (!has_task || !task.submap || task.query_idx < 0) continue;

        auto loops = loop_detector_.detectIntraSubmapLoop(task.submap, task.query_idx);
        if (loops.empty()) continue;

        const size_t num_loops = loops.size();
        RCLCPP_INFO(get_logger(),
            "[INTRA_LOOP][ASYNC] submap_id=%d query_idx=%d detected=%zu loops (enqueue INTRA_LOOP_BATCH to opt_worker)",
            task.submap->id, task.query_idx, num_loops);

        {
            std::lock_guard<std::mutex> lk(opt_task_mutex_);
            if (opt_task_queue_.size() < kMaxOptTaskQueueSize) {
                OptTaskItem batch;
                batch.type = OptTaskItem::Type::INTRA_LOOP_BATCH;
                batch.intra_loop_submap = task.submap;
                batch.intra_loop_constraints = std::move(loops);
                batch.intra_loop_query_keyframe_idx = task.query_idx;
                opt_task_queue_.push_back(std::move(batch));
                opt_task_cv_.notify_one();
            } else {
                RCLCPP_WARN(get_logger(), "[AutoMapSystem][INTRA_LOOP_WORKER] opt_task_queue full, drop %zu intra loops", num_loops);
            }
        }
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][INTRA_LOOP_WORKER] thread exiting");
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS对齐处理线程：从gps_align_queue取出对齐结果并处理完整流程
// ─────────────────────────────────────────────────────────────────────────────
void AutoMapSystem::gpsAlignWorkerLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_gps_align");
#endif
    gps_align_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_WORKER] thread started");

    const auto wait_timeout = std::chrono::milliseconds(500);

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        gps_align_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);

        GPSAlignTaskItem align_task;
        bool has_task = false;
        {
            std::unique_lock<std::mutex> lk_queue(gps_align_mutex_);
            const bool has_data = gps_align_cv_.wait_for(lk_queue, wait_timeout, [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !gps_align_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!has_data || gps_align_queue_.empty()) continue;
            align_task = std::move(gps_align_queue_.front());
            gps_align_queue_.pop_front();
            has_task = true;
        }

        if (has_task) {
            // 0. 幂等性检查：如果已经对齐，不再重复处理（除非设计支持增量对齐，当前架构不支持）
            if (gps_aligned_.load()) {
                RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_WORKER] system already aligned, skipping task");
                continue;
            }
            pose_update_barrier_active_.store(true, std::memory_order_release);
            RCLCPP_INFO(get_logger(),
                        "[AutoMapSystem][POSE_TX][BARRIER] activated reason=GPS_ALIGN begin");

            // 1. 设置 GPS 对齐状态和变换矩阵 (Map -> ENU 逆变换)
            {
                std::lock_guard<std::mutex> lk(gps_transform_mutex_);
                gps_aligned_.store(true);
                gps_transform_R_ = align_task.R_enu_to_map.transpose();
                gps_transform_t_ = -gps_transform_R_ * align_task.t_enu_to_map;
            }

            // 2. 转换所有已有的位姿到 ENU 坐标系
            GPSAlignResult res_tmp;
            res_tmp.success = true;
            res_tmp.R_enu_to_map = align_task.R_enu_to_map;
            res_tmp.t_enu_to_map = align_task.t_enu_to_map;
            transformAllPosesAfterGPSAlign(res_tmp);

            // 3. 通知 GPSManager 重置对齐矩阵（同步坐标系）
            gps_manager_.resetAlignmentToIdentity();

            // 4. 通知 HBA 对齐状态（使用单位阵，因为子图位姿已转换到 ENU 全球系）
            GPSAlignResult hba_res = res_tmp;
            hba_res.R_enu_to_map = Eigen::Matrix3d::Identity();
            hba_res.t_enu_to_map = Eigen::Vector3d::Zero();
            // 在 HBA 内部，R_gps_lidar/t_gps_lidar 用于 GPS factor
            hba_res.R_gps_lidar = Eigen::Matrix3d::Identity();
            hba_res.t_gps_lidar = Eigen::Vector3d::Zero();
            hba_optimizer_.setGPSAlignedState(hba_res);

            // 4. 等待后端处理完当前任务，确保重建时拿到一致的图状态
            ensureBackendCompletedAndFlushBeforeHBA();

            // 4. 获取并转换所有子图位姿，用于全球化重建图
            auto submap_data = isam2_optimizer_.getAllSubmapData();
            {
                // 将 isam2 拿到的 local 位姿也转换到全球坐标系，保证 rebuild 后的图直接在全球系工作
                Eigen::Matrix3d R_inv = align_task.R_enu_to_map.transpose();
                Eigen::Vector3d t_inv = -R_inv * align_task.t_enu_to_map;
                int sampled_sm_logs = 0;
                for (auto& d : submap_data) {
                    const Eigen::Vector3d before_t = d.pose.translation();
                    Pose3d T = Pose3d::Identity();
                    T.linear() = R_inv * d.pose.linear();
                    T.translation() = R_inv * d.pose.translation() + t_inv;
                    d.pose = T;
                    if (sampled_sm_logs < 5) {
                        const Eigen::Vector3d after_t = d.pose.translation();
                        RCLCPP_INFO(get_logger(),
                            "[AutoMapSystem][GPS_ALIGN_WORKER][REBUILD_SM] sm_id=%d before=[%.3f,%.3f,%.3f] after=[%.3f,%.3f,%.3f] dtrans=%.3fm",
                            d.id,
                            before_t.x(), before_t.y(), before_t.z(),
                            after_t.x(), after_t.y(), after_t.z(),
                            (after_t - before_t).norm());
                        sampled_sm_logs++;
                    }
                }
            }
            auto odom_factors = isam2_optimizer_.getOdomFactors();
            auto loop_factors = isam2_optimizer_.getLoopFactors();
            auto keyframe_data = isam2_optimizer_.getKeyFrameData();
            auto kf_odom_factors = isam2_optimizer_.getKFOdomFactors();
            auto kf_loop_factors = isam2_optimizer_.getKFLoopFactors();

            {
                // 将 keyframe 位姿也转换到全球坐标系
                Eigen::Matrix3d R_inv = align_task.R_enu_to_map.transpose();
                Eigen::Vector3d t_inv = -R_inv * align_task.t_enu_to_map;
                int sampled_kf_logs = 0;
                for (auto& d : keyframe_data) {
                    const Eigen::Vector3d before_t = d.pose.translation();
                    Pose3d T = Pose3d::Identity();
                    T.linear() = R_inv * d.pose.linear();
                    T.translation() = R_inv * d.pose.translation() + t_inv;
                    d.pose = T;
                    if (sampled_kf_logs < 8) {
                        const Eigen::Vector3d after_t = d.pose.translation();
                        RCLCPP_INFO(get_logger(),
                            "[AutoMapSystem][GPS_ALIGN_WORKER][REBUILD_KF] kf_id=%d before=[%.3f,%.3f,%.3f] after=[%.3f,%.3f,%.3f] dtrans=%.3fm",
                            d.id,
                            before_t.x(), before_t.y(), before_t.z(),
                            after_t.x(), after_t.y(), after_t.z(),
                            (after_t - before_t).norm());
                        sampled_kf_logs++;
                    }
                }
            }

            // 5. 投递完整GPS对齐任务到opt_worker线程（循环尝试，不持锁等待）
            bool enqueued = false;
            const int max_tries = 20;
            const int wait_ms = 100;
            
            for (int try_count = 0; try_count < max_tries; ++try_count) {
                // 注意：这里传递的 R, t 实际上没被 rebuild 逻辑直接使用来变换位姿，
                // 而是由 submitGPSAlignComplete 后的 opt_worker 线程作为上下文。
                // 真正的变换已在上面的第 4 步完成。
                if (task_dispatcher_ && task_dispatcher_->submitGPSAlignComplete(
                    submap_data, odom_factors, loop_factors, 
                    keyframe_data, kf_odom_factors, kf_loop_factors,
                    align_task.R_enu_to_map, align_task.t_enu_to_map)) {
                    enqueued = true;
                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_WORKER] enqueued GPS_ALIGN_COMPLETE task via TaskDispatcher (try=%d)", try_count);
                    break;
                }
                
                // 如果入队失败（队列满），则睡眠并重试，不持锁
                if (try_count < max_tries - 1) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
                }
            }
            
            if (!enqueued) {
                RCLCPP_ERROR(get_logger(), "[AutoMapSystem][GPS_ALIGN_WORKER] Failed to enqueue GPS_ALIGN_COMPLETE task after %d tries, queue full!", max_tries);
                pose_update_barrier_active_.store(false, std::memory_order_release);
                RCLCPP_WARN(get_logger(),
                            "[AutoMapSystem][POSE_TX][BARRIER] deactivated reason=GPS_ALIGN enqueue_failed");
                flushDeferredPoseUpdates();
            }

            RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_WORKER] GPS align task processed");
        }
    }

    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_ALIGN_WORKER] thread exiting");
}

void AutoMapSystem::optWorkerLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_opt");
#endif
    opt_worker_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] thread started");

    // 性能统计
    static std::atomic<int64_t> total_task_time_ms{0};
    static std::atomic<int> task_count{0};
    auto deferredPoseTxSize = [this]() -> size_t {
        std::lock_guard<std::mutex> lk(deferred_pose_updates_mutex_);
        return deferred_pose_updates_.size();
    };

    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        opt_worker_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
        OptTaskItem task;
        bool has_task = false;
        size_t queue_size_after_pop = 0;
        {
            std::unique_lock<std::mutex> lock(opt_task_mutex_);
            const bool task_woke = opt_task_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return shutdown_requested_.load(std::memory_order_acquire) || !opt_task_queue_.empty();
            });
            if (shutdown_requested_.load(std::memory_order_acquire)) break;
            if (!task_woke || opt_task_queue_.empty()) continue;
            task = opt_task_queue_.front();
            opt_task_queue_.pop_front();
            queue_size_after_pop = opt_task_queue_.size();
            has_task = true;
        }
        if (has_task) {
            opt_task_in_progress_.store(true, std::memory_order_release);
            RCLCPP_INFO(get_logger(),
                "[AutoMapSystem][OPT_WORKER][TASK_BEGIN] type=%s queue_after_pop=%zu barrier_active=%d deferred_pose_tx=%zu",
                optTaskTypeName(task.type),
                queue_size_after_pop,
                pose_update_barrier_active_.load(std::memory_order_acquire) ? 1 : 0,
                deferredPoseTxSize());

            const auto t0 = std::chrono::steady_clock::now();
            std::string task_type_name;
            try {
                switch (task.type) {
                    case OptTaskItem::Type::LOOP_FACTOR: {
                        task_type_name = "LOOP_FACTOR";
                        if (task.loop_constraint) {
                            const auto& lc = *task.loop_constraint;
                            if (ConfigManager::instance().backendVerboseTrace()) {
                                RCLCPP_INFO(get_logger(),
                                    "[BACKEND_TRACE] opt_worker LOOP_FACTOR sm_i=%d sm_j=%d kf_idx_i=%d kf_idx_j=%d "
                                    "kf_global_i=%d kf_global_j=%d score=%.4f rmse=%.4f",
                                    lc.submap_i, lc.submap_j, lc.keyframe_i, lc.keyframe_j,
                                    lc.keyframe_global_id_i, lc.keyframe_global_id_j,
                                    static_cast<double>(lc.overlap_score), static_cast<double>(lc.rmse));
                            }
                            isam2_optimizer_.addLoopFactor(task.loop_constraint);
                        }
                        break;
                    }
                    case OptTaskItem::Type::GPS_FACTOR: {
                        task_type_name = "GPS_FACTOR";
                        if (ConfigManager::instance().backendVerboseTrace()) {
                            RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] opt_worker GPS_FACTOR to_id=%d pos=[%.2f,%.2f,%.2f]",
                                task.to_id, task.gps_pos.x(), task.gps_pos.y(), task.gps_pos.z());
                        }
                        isam2_optimizer_.addGPSFactor(task.to_id, task.gps_pos, task.gps_cov);
                        break;
                    }
                    case OptTaskItem::Type::SUBMAP_NODE: {
                        task_type_name = "SUBMAP_NODE";
                        isam2_optimizer_.addSubMapNode(task.to_id, task.rel_pose, false);
                        break;
                    }
                    case OptTaskItem::Type::ODOM_FACTOR: {
                        task_type_name = "ODOM_FACTOR";
                        if (ConfigManager::instance().backendVerboseTrace()) {
                            RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] opt_worker ODOM_FACTOR from=%d to=%d rel_norm=%.3f",
                                task.from_id, task.to_id, task.rel_pose.translation().norm());
                        }
                        isam2_optimizer_.addOdomFactor(task.from_id, task.to_id, task.rel_pose, task.info_matrix);
                        break;
                    }
                    case OptTaskItem::Type::REBUILD: {
                        task_type_name = "REBUILD";
                        isam2_optimizer_.rebuildAfterGPSAlign(
                            task.submap_data, task.odom_factors, task.loop_factors,
                            task.keyframe_data, task.kf_odom_factors, task.kf_loop_factors);
                        break;
                    }
                    case OptTaskItem::Type::GPS_ALIGN_COMPLETE: {
                        task_type_name = "GPS_ALIGN_COMPLETE";
                        isam2_optimizer_.waitForPendingTasks();
                        isam2_optimizer_.rebuildAfterGPSAlign(
                            task.submap_data, task.odom_factors, task.loop_factors,
                            task.keyframe_data, task.kf_odom_factors, task.kf_loop_factors);
                        addBatchGPSFactors();
                        pose_update_barrier_active_.store(false, std::memory_order_release);
                        RCLCPP_INFO(get_logger(),
                                    "[AutoMapSystem][POSE_TX][BARRIER] deactivated reason=GPS_ALIGN_COMPLETE");
                        flushDeferredPoseUpdates();
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] GPS_ALIGN_COMPLETE processed");
                        break;
                    }
                    case OptTaskItem::Type::RESET: {
                        task_type_name = "RESET";
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] processing RESET task");
                        isam2_optimizer_.reset();
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] RESET task done");
                        break;
                    }
                    case OptTaskItem::Type::FORCE_UPDATE: {
                        task_type_name = "FORCE_UPDATE";
                        if (ConfigManager::instance().backendVerboseTrace()) {
                            RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] opt_worker FORCE_UPDATE ENTER");
                        }
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] processing FORCE_UPDATE task");
                        auto result = isam2_optimizer_.forceUpdate();
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] FORCE_UPDATE done: success=%d nodes=%d elapsed_ms=%.1f",
                                    result.success ? 1 : 0, result.nodes_updated, result.elapsed_ms);
                        break;
                    }
                    case OptTaskItem::Type::INTRA_LOOP_BATCH: {
                        task_type_name = "INTRA_LOOP_BATCH";
                        if (!task.intra_loop_submap || task.intra_loop_constraints.empty()) break;
                        const auto& sm = task.intra_loop_submap;
                        // 最小化修复：子图内回环在 detectIntraSubmapLoop 中已通过 loop callback -> LOOP_FACTOR 入图。
                        // 这里不再重复 addLoopFactorDeferred，避免同一回环双路径重复入图导致图被过度拉扯。
                        if (task.intra_loop_query_keyframe_idx >= 0 && task.intra_loop_query_keyframe_idx < static_cast<int>(sm->keyframes.size())) {
                            const auto& qkf = sm->keyframes[task.intra_loop_query_keyframe_idx];
                            if (qkf) {
                                last_keyframe_id_after_loop_success_ = static_cast<int64_t>(qkf->id);
                                const int interval = ConfigManager::instance().loopDetectionMinKeyframeIntervalAfterSuccess();
                                loop_detector_.setLoopDetectionThrottleState(last_keyframe_id_after_loop_success_, interval);
                                RCLCPP_INFO(get_logger(), "[LOOP_THROTTLE] after intra success: query_kf_id=%lu, skip next %d keyframes",
                                    qkf->id, interval);
                            }
                        }
                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] INTRA_LOOP_BATCH done submap_id=%d loops=%zu",
                                    sm->id, task.intra_loop_constraints.size());
                        break;
                    }
                    case OptTaskItem::Type::KEYFRAME_CREATE: {
                        task_type_name = "KEYFRAME_CREATE";
                        if (task.keyframe) {
                            auto kf = task.keyframe;
                            if (ConfigManager::instance().backendVerboseTrace()) {
                                const int node_id = kf->submap_id * MAX_KF_PER_SUBMAP + kf->index_in_submap;
                                RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] opt_worker KEYFRAME_CREATE ENTER kf_id=%lu node_id=%d has_prev=%d",
                                    static_cast<unsigned long>(kf->id), node_id, task.has_prev_kf ? 1 : 0);
                            }

                            // 🔧 V2 修复：不再在 optWorkerLoop 中重复应用 GPS 变换，AutoMapSystem::tryCreateKeyFrame 已应用
                            /*
                            if (task.gps_aligned) {
                                Pose3d T = Pose3d::Identity();
                                T.linear() = task.gps_transform_R * kf->T_w_b.linear();
                                T.translation() = task.gps_transform_R * kf->T_w_b.translation() + task.gps_transform_t;
                                kf->T_w_b = T;
                            }
                            */

                            {
                                std::lock_guard<std::mutex> lk(submap_update_mutex_);
                                submap_manager_.addKeyFrame(kf);

                                // 🔧 V2 修复：触发子图内回环检测（通过专用异步线程 intra_loop_worker）
                                {
                                    std::lock_guard<std::mutex> lk_il(intra_loop_task_mutex_);
                                    if (intra_loop_task_queue_.size() < kMaxIntraLoopTaskQueueSize) {
                                        IntraLoopTask il_task;
                                        il_task.submap = submap_manager_.getSubmap(kf->submap_id);
                                        il_task.query_idx = kf->index_in_submap;
                                        intra_loop_task_queue_.push_back(il_task);
                                        intra_loop_task_cv_.notify_one();
                                    } else {
                                        RCLCPP_WARN(get_logger(), "[AutoMapSystem][OPT_WORKER] intra_loop_task_queue full, skip kf_id=%lu", kf->id);
                                    }
                                }
                            }

                            // 🔧 V2 修复：添加关键帧节点到 iSAM2 (Symbol 'x')
                            int node_id = kf->submap_id * MAX_KF_PER_SUBMAP + kf->index_in_submap;
                            bool is_first_kf_of_submap = (kf->index_in_submap == 0);
                            isam2_optimizer_.addKeyFrameNode(node_id, kf->T_w_b, kf->id == 0, is_first_kf_of_submap);

                            // 🔧 V3 修复：实时添加 GPS 因子，减少无约束漂移窗口
                            if (kf->has_valid_gps && gps_aligned_.load(std::memory_order_acquire)) {
                                Eigen::Vector3d pos_map = kf->gps.position_enu;
                                // 协方差计算逻辑与 ACTIVE_SUBMAP_GPS_BIND 一致
                                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                                double hdop_safe = std::max(kf->gps.hdop, 0.5);
                                double sigma_h = 0.5 * (hdop_safe / 10.0);
                                double sigma_v = 1.0 * (hdop_safe / 10.0);
                                cov(0, 0) = sigma_h * sigma_h;
                                cov(1, 1) = sigma_h * sigma_h;
                                cov(2, 2) = sigma_v * sigma_v;
                                
                                isam2_optimizer_.addGPSFactorForKeyFrame(node_id, pos_map, cov);
                                
                                static std::atomic<int> kf_gps_factor_count{0};
                                int cur_count = kf_gps_factor_count.fetch_add(1) + 1;
                                if (cur_count <= 10 || cur_count % 100 == 0) {
                                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][GPS_KF] GPS factor added to new KF: kf_id=%lu node_id=%d count=%d pos=[%.3f,%.3f,%.3f] hdop=%.2f",
                                        static_cast<unsigned long>(kf->id), node_id, cur_count, pos_map.x(), pos_map.y(), pos_map.z(), kf->gps.hdop);
                                }
                                
                                if (ConfigManager::instance().backendVerboseTrace()) {
                                    RCLCPP_INFO(get_logger(), "[BACKEND_TRACE] opt_worker GPS_KF kf_id=%lu node_id=%d pos=[%.3f,%.3f,%.3f]",
                                        static_cast<unsigned long>(kf->id), node_id, pos_map.x(), pos_map.y(), pos_map.z());
                                }
                            }

                            if (task.has_prev_kf) {
                                auto prev_kf = task.prev_keyframe;
                                if (prev_kf && prev_kf->index_in_submap >= 0) {
                                    int prev_node_id = prev_kf->submap_id * MAX_KF_PER_SUBMAP + prev_kf->index_in_submap;
                                    Pose3d rel = prev_kf->T_w_b.inverse() * kf->T_w_b;
                                    Mat66d info = computeOdomInfoMatrixForKeyframes(prev_kf, kf, rel);
                                    // 🔧 V2 修复：使用 addOdomFactorBetweenKeyframes (Symbol 'x')
                                    if (ConfigManager::instance().backendVerboseTrace()) {
                                        const double rel_trans = rel.translation().norm();
                                        const double rel_rot_deg =
                                            std::abs(Eigen::AngleAxisd(rel.rotation()).angle()) * 180.0 / M_PI;
                                        RCLCPP_INFO(get_logger(),
                                            "[BACKEND_TRACE] opt_worker KF_ODOM prev_kf_id=%lu curr_kf_id=%lu "
                                            "prev_node=%d curr_node=%d rel_trans_m=%.3f rel_rot_deg=%.2f",
                                            static_cast<unsigned long>(prev_kf->id), static_cast<unsigned long>(kf->id),
                                            prev_node_id, node_id, rel_trans, rel_rot_deg);
                                    }
                                    isam2_optimizer_.addOdomFactorBetweenKeyframes(prev_node_id, node_id, rel, info);
                                    RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] odom_factor_kf added: %d -> %d",
                                                prev_node_id, node_id);
                                }
                            }

                            // kf_manager_.addKeyFrame(kf); // ✅ V2 修复：删除冗余调用，AutoMapSystem::tryCreateKeyFrame 已完成状态更新

                            {
                                std::lock_guard<std::mutex> lk(submap_update_mutex_);
                                if (submap_manager_.needFreezeSubmap()) {
                                    auto frozen_sm = submap_manager_.freezeCurrentSubmap();
                                    if (frozen_sm) {
                                        // 🔧 V2 修复：当子图冻结时，添加子图级里程计因子（可选，取决于架构设计，此处保留子图节点管理）
                                        RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] SubMap %d frozen, keyframes=%zu",
                                                    frozen_sm->id, frozen_sm->keyframes.size());
                                    }
                                }
                            }

                            if (loop_detector_.isRunning()) {
                                std::lock_guard<std::mutex> lk(loop_trigger_mutex_);
                                if (loop_trigger_queue_.size() < kMaxLoopTriggerQueueSize) {
                                    loop_trigger_queue_.push_back(kf);
                                    loop_trigger_cv_.notify_one();
                                } else {
                                    RCLCPP_WARN(get_logger(), "[AutoMapSystem][OPT_WORKER] loop_trigger_queue full, dropping kf_id=%lu", static_cast<unsigned long>(kf->id));
                                }
                            }

                            RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] KEYFRAME_CREATE processed kf_id=%lu node_id=%d", static_cast<unsigned long>(kf->id), node_id);
                        }
                        break;
                    }
                    case OptTaskItem::Type::ACTIVE_SUBMAP_GPS_BIND: {
                        task_type_name = "ACTIVE_SUBMAP_GPS_BIND";
                        SubMap::Ptr active_sm;
                        {
                            std::lock_guard<std::mutex> lk(submap_update_mutex_);
                            active_sm = submap_manager_.getActiveSubmap();
                        }
                        
                        if (active_sm) {
                            // ✅ V2 修复：先收集需要处理的关键帧副本，避免在 GTSAM 操作时持锁导致死锁
                            std::vector<KeyFrame::Ptr> kfs_copy;
                            {
                                std::lock_guard<std::mutex> lk(submap_update_mutex_);
                                kfs_copy = active_sm->keyframes;
                            }

                            int added_count = 0;
                            for (const auto& kf : kfs_copy) {
                                if (!kf) continue;
                                auto gps_opt = gps_manager_.queryByTimestamp(kf->timestamp);
                                if (!gps_opt) continue;
                                // 由于整个系统已全球化（Map Frame == ENU Frame），
                                // 关键帧位姿 T_w_b 已通过 tryCreateKeyFrame 修正到 ENU 坐标系，
                                // GPS 观测直接使用 gps_opt->position_enu 即可，不应再应用 R, t。
                                Eigen::Vector3d pos_map = gps_opt->position_enu;
                                // 🔧 V3 修复：统一协方差计算逻辑
                                Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
                                double hdop_safe = std::max(gps_opt->hdop, 0.5);
                                double sigma_h = 0.5 * (hdop_safe / 10.0);
                                double sigma_v = 1.0 * (hdop_safe / 10.0);
                                cov(0, 0) = sigma_h * sigma_h;
                                cov(1, 1) = sigma_h * sigma_h;
                                cov(2, 2) = sigma_v * sigma_v;
                                
                                // 🔧 V2 修复：使用一致的 ID 编码方案 (sm_id * 100000 + kf_idx)
                                int node_id = kf->submap_id * MAX_KF_PER_SUBMAP + kf->index_in_submap;
                                bool is_first_kf_of_submap = (kf->index_in_submap == 0);
                                isam2_optimizer_.addKeyFrameNode(node_id, kf->T_w_b, kf->id == 0, is_first_kf_of_submap);
                                isam2_optimizer_.addGPSFactorForKeyFrame(node_id, pos_map, cov);
                                added_count++;
                            }
                            if (added_count > 0) {
                                isam2_optimizer_.forceUpdate();
                                RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] ACTIVE_SUBMAP_GPS_BIND added %d factors", added_count);
                            }
                        }
                        break;
                    }
                    default:
                        task_type_name = "UNKNOWN";
                }

                const auto t1 = std::chrono::steady_clock::now();
                const int64_t task_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                total_task_time_ms.fetch_add(task_time_ms);
                task_count.fetch_add(1);

                if (task_time_ms > 1000) {
                    RCLCPP_WARN(get_logger(), "[AutoMapSystem][OPT_WORKER][SLOW] task=%s time=%ldms", task_type_name.c_str(), task_time_ms);
                } else {
                    RCLCPP_DEBUG(get_logger(), "[AutoMapSystem][OPT_WORKER] task=%s time=%ldms", task_type_name.c_str(), task_time_ms);
                }
                RCLCPP_INFO(get_logger(),
                    "[AutoMapSystem][OPT_WORKER][TASK_END] type=%s time_ms=%ld queue_after_pop=%zu barrier_active=%d",
                    task_type_name.c_str(), task_time_ms, queue_size_after_pop,
                    pose_update_barrier_active_.load(std::memory_order_acquire) ? 1 : 0);

            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(),
                    "[AutoMapSystem][OPT_WORKER][EXCEPTION] task=%s failed: %s | queue_after_pop=%zu barrier_active=%d deferred_pose_tx=%zu gps_aligned=%d",
                    task_type_name.c_str(), e.what(),
                    queue_size_after_pop,
                    pose_update_barrier_active_.load(std::memory_order_acquire) ? 1 : 0,
                    deferredPoseTxSize(),
                    gps_aligned_.load(std::memory_order_acquire) ? 1 : 0);
            }
            opt_task_in_progress_.store(false, std::memory_order_release);
        }
    }

    const int64_t avg_task_time = task_count.load() > 0 ? total_task_time_ms.load() / task_count.load() : 0;
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][OPT_WORKER] thread exiting (total_tasks=%d avg_time=%ldms)", task_count.load(), avg_task_time);
}

// 注意：vizThreadLoop 已删除，可视化功能由其他模块处理

void AutoMapSystem::statusPublisherLoop() {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "automap_status");
#endif
    status_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
    while (!shutdown_requested_.load(std::memory_order_acquire)) {
        status_pub_heartbeat_ts_ms_.store(nowMs(), std::memory_order_release);
        std::unique_lock<std::mutex> lock(status_pub_mutex_);
        /* const bool status_woke = */ status_pub_cv_.wait_for(lock, std::chrono::seconds(5), [this] {
            return shutdown_requested_.load(std::memory_order_acquire) ||
                   status_publish_pending_.load(std::memory_order_acquire) ||
                   data_flow_publish_pending_.load(std::memory_order_acquire);
        });
        if (shutdown_requested_.load(std::memory_order_acquire)) break;
        const bool do_status = status_publish_pending_.exchange(false);
        const bool do_dataflow = data_flow_publish_pending_.exchange(false);
        lock.unlock();
        if (do_status) publishStatus();
        if (do_dataflow) publishDataFlowSummary();
    }
    RCLCPP_INFO(get_logger(), "[AutoMapSystem][STATUS_PUB] thread exiting");
}

}  // namespace automap_pro
