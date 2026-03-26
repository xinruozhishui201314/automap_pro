#include "automap_pro/v3/semantic_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/path_resolver.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <thread>
#include <vector>

namespace automap_pro::v3 {
namespace {

size_t dtBinIndex(const bool has_kf_ts, double dt_sec) {
    if (!has_kf_ts || !std::isfinite(dt_sec) || dt_sec < 0.0) return 7;
    if (dt_sec <= 0.01) return 0;
    if (dt_sec <= 0.03) return 1;
    if (dt_sec <= 0.05) return 2;
    if (dt_sec <= 0.10) return 3;
    if (dt_sec <= 0.20) return 4;
    if (dt_sec <= 0.50) return 5;
    return 6;
}

uint64_t semanticTraceId(uint64_t kf_id, double ts) {
    const uint64_t ts_us = std::isfinite(ts) ? static_cast<uint64_t>(std::max(0.0, ts) * 1e6) : 0ull;
    return (kf_id << 20) ^ ts_us;
}

uint64_t makeEventId(double ts, uint64_t seq) {
    const uint64_t ts_us = std::isfinite(ts) ? static_cast<uint64_t>(std::max(0.0, ts) * 1e6) : 0ull;
    return (seq << 20) ^ ts_us;
}

std::string findFirstByExtensionRecursively(const std::string& root, const std::string& ext) {
    if (root.empty() || !std::filesystem::is_directory(root)) return {};
    try {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() == ext) {
                return entry.path().string();
            }
        }
    } catch (const std::exception&) {
    }
    return {};
}

}  // namespace

[[noreturn]] void SemanticModule::terminateSystemOnInferenceFailure(
    const char* reason,
    size_t worker_idx,
    double ts,
    uint64_t trace_id,
    const std::string& detail) {
    RCLCPP_FATAL(node_->get_logger(),
        "[SEMANTIC][FATAL][CHAIN_ABORT] reason=%s worker=%zu ts=%.3f trace=%lu detail=%s",
        reason ? reason : "unknown",
        worker_idx,
        ts,
        static_cast<unsigned long>(trace_id),
        detail.empty() ? "(none)" : detail.c_str());
    RCLCPP_FATAL(node_->get_logger(),
        "[SEMANTIC][FATAL][CHAIN_ABORT] policy=exit_process_on_inference_chain_error action=std::exit(1)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::exit(1);
}

void SemanticModule::enqueueTaskLocked(const SyncedFrameEvent& ev) {
    if (task_queue_.size() >= kCoalesceThreshold) {
        const double replaced_ts = task_queue_.back().timestamp;
        task_queue_.back() = ev;
        const auto coalesced = ++coalesced_tasks_;
        if ((coalesced % 50) == 0) {
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][ENQUEUE] step=coalesce_latest total=%lu replaced_ts=%.3f new_ts=%.3f queue_size=%zu",
                static_cast<unsigned long>(coalesced), replaced_ts, ev.timestamp, task_queue_.size());
        }
        return;
    }
    if (task_queue_.size() >= kMaxQueueSize) {
        const double dropped_ts = task_queue_.front().timestamp;
        task_queue_.pop_front();
        ++backpressure_drops_;
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][ENQUEUE] step=backpressure_drop ts=%.3f queue_full=%zu -> dropped oldest",
            dropped_ts, kMaxQueueSize);
    }
    task_queue_.push_back(ev);
}

SemanticModule::SemanticModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("SemanticModule", event_bus, map_registry), node_(node) {
    
    const auto& cfg = ConfigManager::instance();
    const bool semantic_enabled = cfg.semanticEnabled();
    const bool strict_mode = cfg.semanticStrictMode();

    semantic_cfg_.model_type = cfg.semanticModelType();
    semantic_cfg_.model_path = PathResolver::resolve(cfg.semanticModelPath());
    semantic_cfg_.lsk3dnet_model_path = PathResolver::resolve(cfg.semanticLsk3dnetModelPath());
    semantic_cfg_.lsk3dnet_device = cfg.semanticLsk3dnetDevice();
    semantic_cfg_.fov_up = cfg.semanticFovUp();
    semantic_cfg_.fov_down = cfg.semanticFovDown();
    semantic_cfg_.img_w = cfg.semanticImgW();
    semantic_cfg_.img_h = cfg.semanticImgH();
    semantic_cfg_.input_channels = cfg.semanticInputChannels();
    semantic_cfg_.num_classes = cfg.semanticNumClasses();
    semantic_cfg_.tree_class_id = cfg.semanticTreeClassId();
    semantic_cfg_.input_mean = cfg.semanticInputMean();
    semantic_cfg_.input_std = cfg.semanticInputStd();
    semantic_cfg_.do_destagger = cfg.semanticDoDestagger();
    semantic_cfg_.beam_cluster_threshold = cfg.semanticBeamClusterThreshold();
    semantic_cfg_.max_dist_to_centroid = cfg.semanticMaxDistToCentroid();
    semantic_cfg_.min_vertex_size = cfg.semanticMinVertexSize();
    semantic_cfg_.min_landmark_size = cfg.semanticMinLandmarkSize();
    semantic_cfg_.min_landmark_height = cfg.semanticMinLandmarkHeight();
    semantic_cfg_.diag_enable_detailed_stats = cfg.semanticDiagEnableDetailedStats();
    semantic_cfg_.diag_log_class_histogram = cfg.semanticDiagLogClassHistogram();
    semantic_cfg_.diag_class_hist_top_k = cfg.semanticDiagClassHistTopK();
    semantic_cfg_.diag_class_hist_interval_frames = cfg.semanticDiagClassHistIntervalFrames();
    semantic_cfg_.diag_dump_all_classes = cfg.semanticDiagDumpAllClasses();
    semantic_cfg_.diag_dump_points_per_class_limit = cfg.semanticDiagDumpPointsPerClassLimit();
    semantic_cfg_.diag_override_tree_class_id = cfg.semanticDiagOverrideTreeClassId();
    semantic_cfg_.diag_cluster_profile = cfg.semanticDiagClusterProfile();
    semantic_cfg_.diag_cluster_input_mode = cfg.semanticDiagClusterInputMode();
    semantic_cfg_.diag_trellis_min_cluster_points = cfg.semanticDiagTrellisMinClusterPoints();
    semantic_cfg_.diag_trellis_min_tree_vertices = cfg.semanticDiagTrellisMinTreeVertices();

    // 🏛️ [架构演进] 使用 PathResolver 统一资源定位，消除环境耦合
    auto maybe_repo_root = PathResolver::resolve(cfg.semanticLsk3dnetRepoRoot());
    if (maybe_repo_root.empty()) {
        const auto pkg_root = PathResolver::getProjectRoot("automap_pro");
        if (!pkg_root.empty()) {
            const auto rr = (std::filesystem::path(pkg_root) / "thrid_party" / "LSK3DNet-main").string();
            if (std::filesystem::is_directory(rr)) {
                maybe_repo_root = rr;
            }
        }
    }
    semantic_cfg_.lsk3dnet_repo_root = maybe_repo_root;

    // 🏛️ [产品化加固] 自动解析 LSK3DNet 混合模式资产
    if (semantic_cfg_.model_type == "lsk3dnet_hybrid") {
        const std::string rr = semantic_cfg_.lsk3dnet_repo_root;
        semantic_cfg_.lsk3dnet_config_yaml = PathResolver::resolve(cfg.semanticLsk3dnetConfigYaml());
        semantic_cfg_.lsk3dnet_checkpoint = PathResolver::resolve(cfg.semanticLsk3dnetCheckpoint());
        semantic_cfg_.lsk3dnet_classifier_torchscript = PathResolver::resolve(cfg.semanticLsk3dnetClassifierTorchscript());
        
        RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Resolved hybrid assets: config=%s, checkpoint=%s, classifier=%s",
                    semantic_cfg_.lsk3dnet_config_yaml.c_str(),
                    semantic_cfg_.lsk3dnet_checkpoint.c_str(),
                    semantic_cfg_.lsk3dnet_classifier_torchscript.c_str());

        // 自动导出缺失的 TorchScript 资产
        if (!semantic_cfg_.lsk3dnet_classifier_torchscript.empty() && 
            !std::filesystem::exists(semantic_cfg_.lsk3dnet_classifier_torchscript) &&
            std::filesystem::exists(semantic_cfg_.lsk3dnet_checkpoint) &&
            std::filesystem::exists(semantic_cfg_.lsk3dnet_config_yaml)) {
            
            RCLCPP_WARN(node_->get_logger(), "[SEMANTIC][Module] Missing classifier TorchScript at %s, attempting auto-export...",
                        semantic_cfg_.lsk3dnet_classifier_torchscript.c_str());
            const std::string py_exe = cfg.semanticLsk3dnetPython();
            const std::string export_script = (std::filesystem::path(rr) / "scripts" / "export_lsk3dnet_classifier_torchscript.py").string();
            
            if (!std::filesystem::exists(export_script)) {
                RCLCPP_ERROR(node_->get_logger(), "[SEMANTIC][Module] Export script NOT found at: %s", export_script.c_str());
            }

            std::string cmd = py_exe + " " + export_script + 
                             " --config " + semantic_cfg_.lsk3dnet_config_yaml +
                             " --checkpoint " + semantic_cfg_.lsk3dnet_checkpoint +
                             " --output " + semantic_cfg_.lsk3dnet_classifier_torchscript +
                             " --device cpu 2>&1";
            
            RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Executing auto-export command: %s", cmd.c_str());
            
            int ret = std::system(cmd.c_str());
            if (ret == 0) {
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Auto-export successful: %s", semantic_cfg_.lsk3dnet_classifier_torchscript.c_str());
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[SEMANTIC][Module] Auto-export FAILED (code %d). This usually means the Python environment or script has issues.", ret);
            }
        }

        // 环境覆盖支持
        const char* cls_override = std::getenv("AUTOMAP_LSK3DNET_CLASSIFIER_PATH");
        if (cls_override != nullptr && std::strlen(cls_override) > 0) {
            if (std::filesystem::exists(cls_override)) {
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Overriding classifier path from ENV: %s", cls_override);
                semantic_cfg_.lsk3dnet_classifier_torchscript = cls_override;
            }
        }
        semantic_cfg_.lsk3dnet_hybrid_normal_mode = cfg.semanticLsk3dnetHybridNormalMode();
        semantic_cfg_.lsk3dnet_normal_fov_up_deg = cfg.semanticLsk3dnetNormalFovUpDeg();
        semantic_cfg_.lsk3dnet_normal_fov_down_deg = cfg.semanticLsk3dnetNormalFovDownDeg();
        semantic_cfg_.lsk3dnet_normal_proj_h = std::max(8, std::min(4096, cfg.semanticLsk3dnetNormalProjH()));
        semantic_cfg_.lsk3dnet_normal_proj_w = std::max(8, std::min(8192, cfg.semanticLsk3dnetNormalProjW()));
        const std::string ws_cfg = cfg.semanticLsk3dnetWorkerScript();
        if (ws_cfg.empty()) {
            std::string found;
            if (!rr.empty()) {
                found = (std::filesystem::path(rr) / "scripts" / "lsk3dnet_hybrid_worker.py").string();
            }
            semantic_cfg_.lsk3dnet_worker_script = found;
        } else {
            semantic_cfg_.lsk3dnet_worker_script = PathResolver::resolve(ws_cfg);
        }
        semantic_cfg_.lsk3dnet_python_exe = cfg.semanticLsk3dnetPython();
    }

    // 🏛️ [产品化加固] 当标准 LSK3DNet 缺失但混合模式资产完备时，自动切换以保证精度
    if (semantic_cfg_.model_type == "lsk3dnet" &&
        (semantic_cfg_.lsk3dnet_model_path.empty() || !std::filesystem::exists(semantic_cfg_.lsk3dnet_model_path))) {
        
        const std::string rr = semantic_cfg_.lsk3dnet_repo_root;
        const std::string hy_cfg = PathResolver::resolve(cfg.semanticLsk3dnetConfigYaml());
        std::string hy_ckpt = PathResolver::resolve(cfg.semanticLsk3dnetCheckpoint());
        std::string hy_cls = PathResolver::resolve(cfg.semanticLsk3dnetClassifierTorchscript());
        std::string hy_ws = PathResolver::resolve(cfg.semanticLsk3dnetWorkerScript());
        
        if (hy_ws.empty() && !rr.empty()) {
            hy_ws = (std::filesystem::path(rr) / "scripts" / "lsk3dnet_hybrid_worker.py").string();
        }
        if ((hy_ckpt.empty() || !std::filesystem::exists(hy_ckpt)) && std::filesystem::is_directory(rr)) {
            hy_ckpt = findFirstByExtensionRecursively(rr, ".pt");
        }
        
        const bool hybrid_ready = std::filesystem::exists(hy_cfg) && 
                                std::filesystem::exists(hy_ckpt) && 
                                std::filesystem::exists(hy_cls) && 
                                std::filesystem::exists(hy_ws);
        
        if (hybrid_ready) {
            semantic_cfg_.model_type = "lsk3dnet_hybrid";
            semantic_cfg_.lsk3dnet_config_yaml = hy_cfg;
            semantic_cfg_.lsk3dnet_checkpoint = hy_ckpt;
            semantic_cfg_.lsk3dnet_classifier_torchscript = hy_cls;
            semantic_cfg_.lsk3dnet_worker_script = hy_ws;
            semantic_cfg_.lsk3dnet_python_exe = cfg.semanticLsk3dnetPython();
            RCLCPP_WARN(node_->get_logger(), "[SEMANTIC][Module] Auto-switch to lsk3dnet_hybrid due to missing .ts model");
        }
    }

    // 配置日志
    RCLCPP_INFO(node_->get_logger(),
        "[SEMANTIC][Module][CONFIG] enabled=%d model_type=%s device=%s sloam_onnx=%s lsk_ts=%s img=%dx%d fov=[%.1f,%.1f]",
        semantic_enabled ? 1 : 0, semantic_cfg_.model_type.c_str(), semantic_cfg_.lsk3dnet_device.c_str(),
        semantic_cfg_.model_path.c_str(), semantic_cfg_.lsk3dnet_model_path.c_str(),
        semantic_cfg_.img_w, semantic_cfg_.img_h, semantic_cfg_.fov_up, semantic_cfg_.fov_down);

    // 线程与弹性伸缩配置
    const int system_threads = std::max(1, cfg.numThreads());
    const int configured_workers = cfg.semanticWorkerThreads();
    worker_thread_count_ = (configured_workers > 0) ? static_cast<size_t>(configured_workers) 
                                                   : static_cast<size_t>(std::clamp(system_threads / 4, 1, 6));
    
    autoscale_eval_interval_s_ = cfg.semanticAutoscaleEvalIntervalS();
    autoscale_high_watermark_ = cfg.semanticAutoscaleHighWatermark();
    autoscale_low_watermark_ = cfg.semanticAutoscaleLowWatermark();
    min_worker_count_ = std::min(worker_thread_count_, static_cast<size_t>(cfg.semanticAutoscaleMinActiveWorkers()));
    active_worker_count_.store(min_worker_count_, std::memory_order_relaxed);

    // 初始化校验 (Fail-Fast)
    bool config_error = false;
    std::string err_reason;

    if (!semantic_enabled) {
        err_reason = "disabled by config";
    } else if (semantic_cfg_.model_type == "sloam" && !std::filesystem::exists(semantic_cfg_.model_path)) {
        err_reason = "sloam model not found at: " + semantic_cfg_.model_path; config_error = true;
    } else if (semantic_cfg_.model_type == "lsk3dnet" && !std::filesystem::exists(semantic_cfg_.lsk3dnet_model_path)) {
        err_reason = "lsk3dnet model not found at: " + semantic_cfg_.lsk3dnet_model_path; config_error = true;
    } else if (semantic_cfg_.model_type == "lsk3dnet_hybrid") {
        if (!std::filesystem::exists(semantic_cfg_.lsk3dnet_checkpoint)) {
            err_reason = "lsk3dnet_hybrid checkpoint missing at: " + semantic_cfg_.lsk3dnet_checkpoint; config_error = true;
        } else if (!std::filesystem::exists(semantic_cfg_.lsk3dnet_classifier_torchscript)) {
            err_reason = "lsk3dnet_hybrid classifier TorchScript missing at: " + semantic_cfg_.lsk3dnet_classifier_torchscript; config_error = true;
        } else if (!std::filesystem::exists(semantic_cfg_.lsk3dnet_config_yaml)) {
            err_reason = "lsk3dnet_hybrid config YAML missing at: " + semantic_cfg_.lsk3dnet_config_yaml; config_error = true;
        }
    }

    if (config_error) {
        if (strict_mode) {
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][FATAL] %s (strict_mode=true)", err_reason.c_str());
            throw std::runtime_error(err_reason);
        }
        RCLCPP_ERROR(node_->get_logger(), "[SEMANTIC][INIT] %s, module will be idle", err_reason.c_str());
    } else if (semantic_enabled) {
        // 🏛️ [产品化加固] 自动推导 LSK3DNet 树木类别 ID (NuScenes=10, KITTI=15)
        if (semantic_cfg_.model_type.find("lsk3dnet") != std::string::npos && semantic_cfg_.tree_class_id == -1) {
            if (semantic_cfg_.num_classes == 17) {
                semantic_cfg_.tree_class_id = 16; // NuScenes Vegetation
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Auto-set tree_class_id=16 (NuScenes Vegetation)");
            } else if (semantic_cfg_.num_classes == 19 || semantic_cfg_.num_classes == 20) {
                semantic_cfg_.tree_class_id = 15; // SemanticKITTI Trunk
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Auto-set tree_class_id=15 (KITTI Trunk)");
            }
        }

        try {
            semantic_processors_.clear();
            semantic_processors_.reserve(worker_thread_count_);
            bool all_ready = true;
            for (size_t i = 0; i < worker_thread_count_; ++i) {
                auto processor = std::make_shared<SemanticProcessor>(semantic_cfg_);
                all_ready = all_ready && processor->hasRuntimeCapability();
                semantic_processors_.push_back(processor);
            }
            semantic_runtime_ready_.store(all_ready, std::memory_order_relaxed);
        } catch (const std::exception& e) {
            semantic_runtime_ready_.store(false, std::memory_order_relaxed);
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][Module][INIT] FATAL initialization failure: %s", e.what());
            
            // 🏛️ [架构加固] 满足用户契约：初始化失败直接退出工程
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][FATAL] Terminating entire system due to semantic initialization failure as requested.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 给日志落地留时间
            std::exit(1);
        }
    }

    // Hard requirement for LSK3DNet
    if (semantic_enabled && (semantic_cfg_.model_type.find("lsk3dnet") != std::string::npos) && 
        !semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][FATAL] LSK3DNet requires CUDA or specific assets, but runtime not ready. Terminating system.");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::exit(1);
    }

    // 事件注册与状态发布
    onEvent<GraphTaskEvent>([this](const GraphTaskEvent& ev) { handleGraphTaskEvent(ev); });
    onEvent<SystemQuiesceRequestEvent>([this](const SystemQuiesceRequestEvent& ev) { this->quiesce(ev.enable); });

    if (semantic_runtime_ready_.load()) {
        RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module][INIT] step=ok workers=%zu", worker_thread_count_);
    }
}

std::vector<std::pair<std::string, size_t>> SemanticModule::queueDepths() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return {
        {"task_queue", task_queue_.size()},
        {"workers_active", active_worker_count_.load(std::memory_order_relaxed)},
        {"workers_total", worker_thread_count_},
    };
}

std::string SemanticModule::idleDetail() const {
    if (semantic_degraded_.load(std::memory_order_relaxed)) {
        return "semantic_degraded=1";
    }
    if (!semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        return "semantic_runtime_ready=0";
    }
    return "";
}

void SemanticModule::start() {
    if (running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][V3] module START name={} semantic_workers={}", name_, worker_thread_count_);
    running_ = true;
    updateHeartbeat();
    thread_ = std::thread(&SemanticModule::run, this);
    worker_threads_.clear();
    for (size_t i = 1; i < worker_thread_count_; ++i) {
        worker_threads_.emplace_back(&SemanticModule::workerLoop, this, i);
    }
}

void SemanticModule::stop() {
    if (!running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][V3] module STOP name={} (joining semantic workers)", name_);
    running_ = false;
    cv_.notify_all();
    for (auto& t : worker_threads_) {
        if (t.joinable()) t.join();
    }
    worker_threads_.clear();
    if (thread_.joinable()) {
        thread_.join();
    }
    ALOG_INFO("Pipeline", "[PIPELINE][V3] module STOPPED name={}", name_);
}

void SemanticModule::handleGraphTaskEvent(const GraphTaskEvent& ev) {
    if (!running_.load()) return;
    if (ev.task.type != OptTaskItem::Type::KEYFRAME_CREATE) return;
    if (!semantic_runtime_ready_.load(std::memory_order_relaxed) ||
        semantic_degraded_.load(std::memory_order_relaxed)) return;
    if (quiescing_.load()) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B2 KF->SEM] action=reject reason=quiescing");
        return;
    }
    const auto& kf = ev.task.keyframe;
    if (!kf) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B2 KF->SEM] action=reject reason=null_keyframe");
        return;
    }

    const auto last_kf_id = last_keyframe_task_id_.load(std::memory_order_relaxed);
    if (last_kf_id != std::numeric_limits<uint64_t>::max() && kf->id <= last_kf_id) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B2 KF->SEM] action=skip reason=duplicate_or_stale kf_id=%lu last_kf_id=%lu",
            static_cast<unsigned long>(kf->id),
            static_cast<unsigned long>(last_kf_id));
        return;
    }

    CloudXYZIPtr sem_cloud = kf->cloud_body;
    if ((!sem_cloud || sem_cloud->empty()) && kf->cloud_ds_body && !kf->cloud_ds_body->empty()) {
        sem_cloud = kf->cloud_ds_body;
    }
    if (!sem_cloud || sem_cloud->empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B2 KF->SEM] action=reject reason=empty_keyframe_cloud kf_id=%lu ts=%.3f reason_code=E_EMPTY_KF_CLOUD",
            static_cast<unsigned long>(kf->id), kf->timestamp);
        return;
    }

    SyncedFrameEvent sem_ev;
    sem_ev.timestamp = kf->timestamp;
    sem_ev.cloud = sem_cloud;
    sem_ev.cloud_ds = kf->cloud_ds_body;
    sem_ev.T_odom_b = kf->T_odom_b;
    sem_ev.covariance = kf->covariance;
    sem_ev.pose_frame = PoseFrame::ODOM;
    sem_ev.cloud_frame = "body";
    sem_ev.kf_info = kf->livo_info;
    sem_ev.kf_info.timestamp = kf->timestamp;
    dt_hist_bins_[dtBinIndex(true, 0.0)].fetch_add(1, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(queue_mutex_);
    enqueueTaskLocked(sem_ev);
    last_keyframe_task_id_.store(kf->id, std::memory_order_relaxed);
    gate_accept_total_.fetch_add(1, std::memory_order_relaxed);
    const uint64_t trace_id = semanticTraceId(kf->id, kf->timestamp);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[CHAIN][B2 KF->SEM] action=accept trace=%lu kf_id=%lu ts=%.3f pts=%zu queue_size=%zu workers_active=%zu",
        static_cast<unsigned long>(trace_id),
        static_cast<unsigned long>(kf->id),
        kf->timestamp,
        sem_cloud->size(),
        task_queue_.size(),
        active_worker_count_.load(std::memory_order_relaxed));
    cv_.notify_one();
}

void SemanticModule::run() {
    workerLoop(0);
}

void SemanticModule::workerLoop(size_t worker_idx) {
    RCLCPP_INFO(node_->get_logger(), "[V3][SemanticModule] Started worker thread idx=%zu", worker_idx);
    while (running_) {
        try {
            if (worker_idx == 0) {
                updateHeartbeat();
            }
            tryRecoverFromDegradedState(node_->now().seconds());
            SyncedFrameEvent event;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                if (worker_idx == 0) {
                    maybeAdjustActiveWorkersLocked(node_->now().seconds());
                }
                const size_t active_workers = active_worker_count_.load(std::memory_order_relaxed);
                if (worker_idx >= active_workers) {
                    cv_.wait_for(lock, std::chrono::milliseconds(100), [this, worker_idx] {
                        return !running_ || worker_idx < active_worker_count_.load(std::memory_order_relaxed);
                    });
                    continue;
                }
                cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                    return !running_ || !task_queue_.empty(); 
                });
                if (!running_) break;
                if (task_queue_.empty()) {
                    continue;
                }

                event = task_queue_.front();
                task_queue_.pop_front();
                if ((processed_tasks_.load(std::memory_order_relaxed) < 20) ||
                    ((processed_tasks_.load(std::memory_order_relaxed) % 50) == 0)) {
                    RCLCPP_INFO(node_->get_logger(),
                        "[SEMANTIC][Module][DEQUEUE] worker=%zu ts=%.3f cloud_pts=%zu queue_left=%zu active_workers=%zu",
                        worker_idx,
                        event.timestamp,
                        event.cloud ? event.cloud->size() : 0,
                        task_queue_.size(),
                        active_worker_count_.load(std::memory_order_relaxed));
                }
            }

            processTask(event, worker_idx);
        } catch (const std::exception& e) {
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
            terminateSystemOnInferenceFailure(
                "worker_loop_exception",
                worker_idx,
                -1.0,
                0ull,
                e.what());
        } catch (...) {
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
            terminateSystemOnInferenceFailure(
                "worker_loop_unknown_exception",
                worker_idx,
                -1.0,
                0ull);
        }
    }
}

void SemanticModule::markSemanticDegraded(const char* reason) {
    const int err_count = ++consecutive_errors_;
    if (err_count >= kMaxConsecutiveErrors) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        semantic_degraded_.store(true, std::memory_order_relaxed);
        next_recovery_retry_s_.store(node_->now().seconds() + kRecoveryCooldownSec, std::memory_order_relaxed);
        recovery_attempts_.store(0, std::memory_order_relaxed);
        recovery_exhausted_reported_.store(false, std::memory_order_relaxed);
        RCLCPP_FATAL(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=degraded reason=%s threshold=%d → SemanticModule disabled to protect system stability",
            reason ? reason : "unknown", kMaxConsecutiveErrors);
    }
}

void SemanticModule::maybeAdjustActiveWorkersLocked(double now_s) {
    if (now_s < next_scale_eval_s_) return;
    next_scale_eval_s_ = now_s + autoscale_eval_interval_s_;

    const size_t q = task_queue_.size();
    const double usage_ratio = static_cast<double>(q) / static_cast<double>(kMaxQueueSize);
    const size_t prev = active_worker_count_.load(std::memory_order_relaxed);
    size_t target = prev;
    // 升速策略：高于高水位时按队列占用线性放大目标并发；降速策略：低于低水位回落到最小并发。
    if (usage_ratio >= autoscale_high_watermark_) {
        const double scale = (usage_ratio - autoscale_high_watermark_) /
                             std::max(1e-6, 1.0 - autoscale_high_watermark_);
        const size_t headroom = worker_thread_count_ - min_worker_count_;
        target = min_worker_count_ + static_cast<size_t>(std::ceil(scale * static_cast<double>(headroom)));
    } else if (usage_ratio <= autoscale_low_watermark_) {
        target = min_worker_count_;
    }

    target = std::max<size_t>(1, std::min(worker_thread_count_, target));
    if (target != prev) {
        active_worker_count_.store(target, std::memory_order_relaxed);
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][AUTO_SCALE] active_workers %zu -> %zu queue=%zu cap=%zu usage=%.2f high=%.2f low=%.2f",
            prev, target, q, kMaxQueueSize, usage_ratio, autoscale_high_watermark_, autoscale_low_watermark_);
        cv_.notify_all();
    }
}

void SemanticModule::processTask(const SyncedFrameEvent& event) {
    processTask(event, 0);
}

void SemanticModule::processTask(const SyncedFrameEvent& event, size_t worker_idx) {
    if (semantic_degraded_.load(std::memory_order_relaxed)) return;
    SemanticProcessor::Ptr processor;
    {
        std::lock_guard<std::mutex> lk(processor_mutex_);
        if (semantic_processors_.empty()) return;
        processor = semantic_processors_[worker_idx % semantic_processors_.size()];
    }
    if (!processor || !processor->hasRuntimeCapability()) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        terminateSystemOnInferenceFailure(
            "processor_runtime_lost_pre_infer",
            worker_idx,
            event.timestamp,
            semanticTraceId(0ull, event.timestamp));
    }

    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Module][RUN] step=start ts=%.3f pts=%zu",
        event.timestamp, (event.cloud && event.cloud->size() > 0) ? event.cloud->size() : 0);
    const auto trace_id = semanticTraceId(
        event.kf_info.timestamp > 0.0 ? static_cast<uint64_t>(event.kf_info.timestamp * 1000.0) : 0ull,
        event.timestamp);

    auto t0 = std::chrono::steady_clock::now();
    std::vector<CylinderLandmark::Ptr> landmarks;
    try {
        landmarks = processor->process(event.cloud);
        consecutive_errors_ = 0; // 重置错误计数
    } catch (const std::exception& e) {
        ++consecutive_errors_;
        terminateSystemOnInferenceFailure(
            "processor_process_exception",
            worker_idx,
            event.timestamp,
            trace_id,
            e.what());
    }
    auto t1 = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ++processed_tasks_;
    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Module][RUN_DIAG] worker=%zu ts=%.3f elapsed_ms=%.1f landmarks=%zu queue_now=%zu",
        worker_idx, event.timestamp, elapsed_ms, landmarks.size(), task_queue_.size());

    if (!processor->hasRuntimeCapability() && semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        terminateSystemOnInferenceFailure(
            "processor_runtime_lost_post_infer",
            worker_idx,
            event.timestamp,
            trace_id);
    }

    if (!landmarks.empty()) {
        // 🏛️ [架构契约] 发布前准入校验
        std::vector<CylinderLandmark::Ptr> valid_landmarks;
        for (const auto& l : landmarks) {
            if (l && l->isValid()) valid_landmarks.push_back(l);
        }

        if (!valid_landmarks.empty()) {
            SemanticLandmarkEvent res_ev;
            res_ev.timestamp = event.timestamp;
            res_ev.keyframe_timestamp_hint = event.kf_info.timestamp;
            if (std::isfinite(event.kf_info.timestamp) && event.kf_info.timestamp > 0.0) {
                const double match_tol = ConfigManager::instance().semanticTimestampMatchToleranceS();
                auto kf = map_registry_->getKeyFrameByTimestamp(event.kf_info.timestamp, match_tol);
                if (kf) {
                    res_ev.keyframe_id_hint = kf->id;
                }
            }
            res_ev.landmarks = valid_landmarks;
            const uint64_t seq = semantic_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
            res_ev.meta.event_id = makeEventId(event.timestamp, seq);
            res_ev.meta.idempotency_key = res_ev.meta.event_id;
            res_ev.meta.producer_seq = seq;
            res_ev.meta.ref_version = map_registry_->getVersion();
            res_ev.meta.ref_epoch = map_registry_->getAlignmentEpoch();
            res_ev.meta.source_ts = event.timestamp;
            res_ev.meta.publish_ts = node_->now().seconds();
            res_ev.meta.producer = "SemanticModule";
            res_ev.meta.route_tag = "legacy";
            res_ev.processing_state = semantic_degraded_.load(std::memory_order_relaxed)
                ? ProcessingState::DEGRADED
                : ProcessingState::NORMAL;
            event_bus_->publish(res_ev);
            sem_event_publish_total_.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "[CHAIN][B3 SEM->MAP] action=publish trace=%lu ts=%.3f landmarks=%zu kf_hint_ts=%.3f kf_hint_id=%lu total_publish=%lu",
                static_cast<unsigned long>(trace_id),
                event.timestamp,
                valid_landmarks.size(),
                res_ev.keyframe_timestamp_hint,
                static_cast<unsigned long>(res_ev.keyframe_id_hint),
                static_cast<unsigned long>(sem_event_publish_total_.load(std::memory_order_relaxed)));

            RCLCPP_INFO(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=%zu elapsed_ms=%.1f worker=%zu → published SemanticLandmarkEvent",
                event.timestamp, valid_landmarks.size(), elapsed_ms, worker_idx);
        } else {
            RCLCPP_DEBUG(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 (all failed validity check) elapsed_ms=%.1f",
                event.timestamp, elapsed_ms);
        }
    } else {
        ++no_landmark_tasks_;
        sem_event_drop_no_landmark_total_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "[CHAIN][B3 SEM->MAP] action=drop trace=%lu reason=no_landmarks ts=%.3f total_drop_no_landmark=%lu reason_code=E_NO_LANDMARKS",
            static_cast<unsigned long>(trace_id),
            event.timestamp,
            static_cast<unsigned long>(sem_event_drop_no_landmark_total_.load(std::memory_order_relaxed)));
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 elapsed_ms=%.1f (no publish)",
            event.timestamp, elapsed_ms);
    }

    // 慢帧告警不再依赖“有无地标”，避免吞掉真实性能问题
    if (elapsed_ms > 250.0) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=SLOW ts=%.3f elapsed_ms=%.1f threshold_ms=250.0",
            event.timestamp, elapsed_ms);
    }

    // 周期统计：帮助判断“模块运行正常但无有效输出”还是“算法/配置问题”
    const auto now = std::chrono::steady_clock::now();
    const auto secs_since_stats = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log_tp_).count();
    const auto secs_since_delta = std::chrono::duration_cast<std::chrono::seconds>(now - last_dt_delta_log_tp_).count();
    if (secs_since_stats >= 10) {
        last_stats_log_tp_ = now;
        const auto processed = processed_tasks_.load();
        const auto no_landmark = no_landmark_tasks_.load();
        const auto coalesced = coalesced_tasks_.load();
        const auto bp_drop = backpressure_drops_.load();
        const auto skipped_dup_kf = skipped_duplicate_keyframe_.load();
        const uint64_t dt_b0 = dt_hist_bins_[0].load(std::memory_order_relaxed);
        const uint64_t dt_b1 = dt_hist_bins_[1].load(std::memory_order_relaxed);
        const uint64_t dt_b2 = dt_hist_bins_[2].load(std::memory_order_relaxed);
        const uint64_t dt_b3 = dt_hist_bins_[3].load(std::memory_order_relaxed);
        const uint64_t dt_b4 = dt_hist_bins_[4].load(std::memory_order_relaxed);
        const uint64_t dt_b5 = dt_hist_bins_[5].load(std::memory_order_relaxed);
        const uint64_t dt_b6 = dt_hist_bins_[6].load(std::memory_order_relaxed);
        const uint64_t dt_missing = dt_hist_bins_[7].load(std::memory_order_relaxed);
        const double no_landmark_ratio =
            (processed > 0) ? (100.0 * static_cast<double>(no_landmark) / static_cast<double>(processed)) : 0.0;
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][STATS] processed=%lu no_landmark=%lu no_landmark_ratio=%.1f%% coalesced=%lu backpressure_drop=%lu skipped_duplicate_kf=%lu dt_hist=[<=0.01:%lu <=0.03:%lu <=0.05:%lu <=0.10:%lu <=0.20:%lu <=0.50:%lu >0.50:%lu missing:%lu]",
            static_cast<unsigned long>(processed),
            static_cast<unsigned long>(no_landmark),
            no_landmark_ratio,
            static_cast<unsigned long>(coalesced),
            static_cast<unsigned long>(bp_drop),
            static_cast<unsigned long>(skipped_dup_kf),
            static_cast<unsigned long>(dt_b0),
            static_cast<unsigned long>(dt_b1),
            static_cast<unsigned long>(dt_b2),
            static_cast<unsigned long>(dt_b3),
            static_cast<unsigned long>(dt_b4),
            static_cast<unsigned long>(dt_b5),
            static_cast<unsigned long>(dt_b6),
            static_cast<unsigned long>(dt_missing));
        if (processed >= 30 && no_landmark_ratio >= 95.0) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                "[SEMANTIC][Module][E_EFFECTIVE_OUTPUT] semantic chain running but output almost empty: "
                "no_landmark_ratio=%.1f%% processed=%lu no_landmark=%lu. "
                "Check tree class mapping / clustering thresholds / segmentation latency (reason_code=E_NO_EFFECTIVE_SEMANTIC_OUTPUT)",
                no_landmark_ratio,
                static_cast<unsigned long>(processed),
                static_cast<unsigned long>(no_landmark));
        }
        if (processed >= 30 && (no_landmark_ratio >= 95.0 || bp_drop > 0 || coalesced > 0)) {
            const size_t queue_now = task_queue_.size();
            const size_t workers_active = active_worker_count_.load(std::memory_order_relaxed);
            const size_t workers_total = worker_thread_count_;
            const char* dominant_chain_reason = "no_effective_landmark_output";
            if (bp_drop > 0 && bp_drop >= coalesced) {
                dominant_chain_reason = "backpressure_drop";
            } else if (coalesced > 0) {
                dominant_chain_reason = "task_coalescing";
            }
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                "[SEMANTIC][Module][CHAIN_ANOMALY] dominant_reason=%s "
                "processed=%lu no_landmark=%lu(%.1f%%) coalesced=%lu backpressure_drop=%lu "
                "queue_now=%zu workers_active=%zu/%zu dt_aligned_le_30ms=%lu "
                "reason_code=E_SEMANTIC_MODULE_CHAIN_ANOMALY",
                dominant_chain_reason,
                static_cast<unsigned long>(processed),
                static_cast<unsigned long>(no_landmark),
                no_landmark_ratio,
                static_cast<unsigned long>(coalesced),
                static_cast<unsigned long>(bp_drop),
                queue_now,
                workers_active,
                workers_total,
                static_cast<unsigned long>(dt_b0 + dt_b1));
        }
    }

    if (secs_since_delta >= 100) {
        last_dt_delta_log_tp_ = now;
        const uint64_t dt_now[8] = {dt_hist_bins_[0].load(std::memory_order_relaxed),
                                    dt_hist_bins_[1].load(std::memory_order_relaxed),
                                    dt_hist_bins_[2].load(std::memory_order_relaxed),
                                    dt_hist_bins_[3].load(std::memory_order_relaxed),
                                    dt_hist_bins_[4].load(std::memory_order_relaxed),
                                    dt_hist_bins_[5].load(std::memory_order_relaxed),
                                    dt_hist_bins_[6].load(std::memory_order_relaxed),
                                    dt_hist_bins_[7].load(std::memory_order_relaxed)};
        const uint64_t d0 = dt_now[0] - dt_hist_last_snapshot_[0];
        const uint64_t d1 = dt_now[1] - dt_hist_last_snapshot_[1];
        const uint64_t d2 = dt_now[2] - dt_hist_last_snapshot_[2];
        const uint64_t d3 = dt_now[3] - dt_hist_last_snapshot_[3];
        const uint64_t d4 = dt_now[4] - dt_hist_last_snapshot_[4];
        const uint64_t d5 = dt_now[5] - dt_hist_last_snapshot_[5];
        const uint64_t d6 = dt_now[6] - dt_hist_last_snapshot_[6];
        const uint64_t d7 = dt_now[7] - dt_hist_last_snapshot_[7];
        uint64_t delta_total = d0 + d1 + d2 + d3 + d4 + d5 + d6 + d7;
        const double align_ratio = (delta_total > 0)
            ? (100.0 * static_cast<double>(d0 + d1) / static_cast<double>(delta_total))
            : 0.0;
        for (size_t i = 0; i < 8; ++i) dt_hist_last_snapshot_[i] = dt_now[i];

        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][DT_DELTA_100S] bins_delta=[<=0.01:%lu <=0.03:%lu <=0.05:%lu <=0.10:%lu <=0.20:%lu <=0.50:%lu >0.50:%lu missing:%lu] total=%lu aligned(<=0.03)=%.1f%%",
            static_cast<unsigned long>(d0),
            static_cast<unsigned long>(d1),
            static_cast<unsigned long>(d2),
            static_cast<unsigned long>(d3),
            static_cast<unsigned long>(d4),
            static_cast<unsigned long>(d5),
            static_cast<unsigned long>(d6),
            static_cast<unsigned long>(d7),
            static_cast<unsigned long>(delta_total),
            align_ratio);
    }
}

void SemanticModule::tryRecoverFromDegradedState(double now_s) {
    if (!semantic_degraded_.load(std::memory_order_relaxed)) return;
    const double next_retry = next_recovery_retry_s_.load(std::memory_order_relaxed);
    if (now_s < next_retry) return;

    const int attempts = recovery_attempts_.load(std::memory_order_relaxed);
    if (attempts >= kMaxRecoveryAttempts) {
        bool expected = false;
        if (recovery_exhausted_reported_.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][RECOVERY] Exhausted recovery budget (%d attempts). Semantic remains degraded until restart/manual intervention.",
                kMaxRecoveryAttempts);
            BackpressureWarningEvent warn;
            warn.module_name = name_ + "_recovery_exhausted";
            warn.queue_usage_ratio = 1.0f;
            warn.critical = true;
            event_bus_->publish(warn);
        }
        return;
    }

    recovery_attempts_.store(attempts + 1, std::memory_order_relaxed);
    RCLCPP_WARN(node_->get_logger(),
        "[SEMANTIC][RECOVERY] Attempt %d/%d to recover degraded semantic runtime",
        attempts + 1, kMaxRecoveryAttempts);
    try {
        std::vector<SemanticProcessor::Ptr> new_processors;
        new_processors.reserve(worker_thread_count_);
        bool ready = true;
        for (size_t i = 0; i < worker_thread_count_; ++i) {
            auto p = std::make_shared<SemanticProcessor>(semantic_cfg_);
            ready = ready && p->hasRuntimeCapability();
            new_processors.push_back(p);
        }
        {
            std::lock_guard<std::mutex> lk(processor_mutex_);
            semantic_processors_ = std::move(new_processors);
        }
        semantic_runtime_ready_.store(ready, std::memory_order_relaxed);
        if (ready) {
            semantic_degraded_.store(false, std::memory_order_relaxed);
            consecutive_errors_.store(0, std::memory_order_relaxed);
            recovery_exhausted_reported_.store(false, std::memory_order_relaxed);
            RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][RECOVERY] Semantic runtime recovered successfully");
            return;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][RECOVERY] Recovery attempt failed: %s", e.what());
    }
    next_recovery_retry_s_.store(now_s + kRecoveryCooldownSec, std::memory_order_relaxed);
}

} // namespace automap_pro::v3
