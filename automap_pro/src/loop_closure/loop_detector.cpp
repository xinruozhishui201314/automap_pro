#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

// ScanContext 依赖
#include "Scancontext/Scancontext.h"
#include "nanoflann.hpp"

#include <algorithm>  // for std::max, std::min
#define MOD "LoopDetector"

#include <automap_pro/msg/loop_constraint_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <future>
#include <thread>

namespace automap_pro {

LoopDetector::LoopDetector() {
    const auto& cfg = ConfigManager::instance();
    overlap_threshold_ = cfg.overlapThreshold();
    top_k_             = cfg.loopTopK();
    min_temporal_gap_  = cfg.loopMinTemporalGap();
    min_submap_gap_    = cfg.loopMinSubmapGap();
    gps_search_radius_ = cfg.gpsSearchRadius();
    geo_prefilter_max_distance_m_ = cfg.loopGeoPrefilterMaxDistanceM();
    geo_prefilter_skip_above_score_ = cfg.loopGeoPrefilterSkipAboveScore();
    min_inlier_ratio_  = cfg.teaserMinInlierRatio();
    max_rmse_          = cfg.teaserMaxRMSE();
    use_icp_refine_    = cfg.teaserICPRefine();
    worker_thread_num_ = std::max(1, cfg.loopWorkerThreads());

    // 子图内回环检测参数
    intra_submap_enabled_ = cfg.intraSubmapLoopEnabled();
    intra_submap_min_temporal_gap_ = cfg.intraSubmapLoopMinTemporalGap();
    intra_submap_min_keyframe_gap_ = cfg.intraSubmapLoopMinKeyframeGap();
    intra_submap_min_distance_gap_ = cfg.intraSubmapLoopMinDistanceGap();
    intra_submap_overlap_threshold_ = cfg.intraSubmapLoopOverlapThreshold();
    intra_submap_max_teaser_candidates_ = cfg.intraSubmapLoopMaxTeaserCandidates();

    // 子图间关键帧级
    inter_keyframe_level_enabled_ = cfg.interKeyframeLevelEnabled();
    inter_keyframe_sample_step_ = cfg.interKeyframeSampleStep();
    inter_keyframe_top_k_per_submap_ = cfg.interKeyframeTopKPerSubmap();
    inter_submap_min_keyframe_gap_ = cfg.interSubmapMinKeyframeGap();

    // ScanContext 参数
    use_scancontext_ = cfg.scancontextEnabled();
    sc_dist_threshold_ = cfg.scancontextDistThreshold();
    sc_num_candidates_ = cfg.scancontextNumCandidates();
    sc_exclude_recent_ = cfg.scancontextExcludeRecent();
    sc_tree_making_period_ = cfg.scancontextTreeMakingPeriod();
    zero_accept_warn_consecutive_queries_ = cfg.loopZeroAcceptWarnConsecutiveQueries();
    ot_preferred_flow_ = cfg.loopOtPreferredFlow();
    allow_sc_fallback_ = cfg.loopAllowScFallback();
    allow_descriptor_fallback_ = cfg.loopAllowDescriptorFallback();
    allow_svd_geom_fallback_ = cfg.loopAllowSvdGeomFallback();
    log_effective_flow_ = cfg.loopLogEffectiveFlow();
    loop_flow_mode_ = cfg.loopFlowMode();
    parallel_teaser_max_inflight_ = cfg.parallelTeaserMaxInflight();
    svd_temp_enable_after_fpfh_critical_ = 3;
    svd_temp_enable_budget_max_ = 24;
    svd_temp_enable_budget_left_ = 0;
    consecutive_fpfh_critical_rejects_ = 0;

    // 缓存 pose_consistency 参数，避免 processMatchTask 中访问 ConfigManager 单例导致 shutdown 时 SIGSEGV（析构/卸载顺序不确定）
    pose_consistency_max_trans_m_ = cfg.loopPoseConsistencyMaxTransDiffM();
    pose_consistency_max_rot_deg_ = cfg.loopPoseConsistencyMaxRotDiffDeg();
    loop_max_desc_queue_size_ = cfg.loopMaxDescQueueSize();
    loop_max_match_queue_size_ = cfg.loopMaxMatchQueueSize();
    teaser_min_safe_inliers_ = cfg.teaserMinSafeInliers();
    parallel_teaser_match_ = cfg.parallelTeaserMatch();
}

LoopDetector::~LoopDetector() { stop(); }

std::string LoopDetector::makeSubmapKey(uint64_t session_id, int submap_id) const {
    return std::to_string(session_id) + ":" + std::to_string(submap_id);
}

double LoopDetector::submapRepresentativeTime(const SubMap::Ptr& submap) const {
    if (!submap) return 0.0;
    if (submap->t_start > 0.0 && submap->t_end >= submap->t_start) {
        return 0.5 * (submap->t_start + submap->t_end);
    }
    if (!submap->keyframes.empty()) {
        const auto& kf0 = submap->keyframes.front();
        const auto& kf1 = submap->keyframes.back();
        if (kf0 && kf1) return 0.5 * (kf0->timestamp + kf1->timestamp);
        if (kf0) return kf0->timestamp;
        if (kf1) return kf1->timestamp;
    }
    return 0.0;
}

bool LoopDetector::shouldAllowSvdFallbackNow(
    const TeaserMatcher::Result& res, int query_id, int target_id, const char* stage_tag) {
    if (allow_svd_geom_fallback_) {
        return true;
    }
    if (loop_flow_mode_ != "safe_degraded") {
        return false;
    }

    if (res.used_teaser) {
        consecutive_fpfh_critical_rejects_ = 0;
        svd_temp_enable_budget_left_ = 0;
        return false;
    }

    if (res.fpfh_garbage_rejected) {
        consecutive_fpfh_critical_rejects_++;
        if (consecutive_fpfh_critical_rejects_ >= svd_temp_enable_after_fpfh_critical_ &&
            svd_temp_enable_budget_left_ <= 0) {
            svd_temp_enable_budget_left_ = svd_temp_enable_budget_max_;
            ALOG_WARN(MOD,
                "[LOOP_FLOW] stage={} query_id={} target_id={} activate_temp_svd_fallback=1 "
                "reason=consecutive_fpfh_critical count={} budget={}",
                stage_tag, query_id, target_id, consecutive_fpfh_critical_rejects_,
                svd_temp_enable_budget_left_);
        }
    } else {
        consecutive_fpfh_critical_rejects_ = 0;
    }

    if (svd_temp_enable_budget_left_ > 0) {
        svd_temp_enable_budget_left_--;
        return true;
    }
    return false;
}

void LoopDetector::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    // 重新从 ConfigManager 加载所有回环参数（构造时早于 loadConfig，此处保证使用 YAML 配置）
    overlap_threshold_ = cfg.overlapThreshold();
    top_k_             = cfg.loopTopK();
    min_temporal_gap_  = cfg.loopMinTemporalGap();
    min_submap_gap_    = cfg.loopMinSubmapGap();
    gps_search_radius_ = cfg.gpsSearchRadius();
    geo_prefilter_max_distance_m_ = cfg.loopGeoPrefilterMaxDistanceM();
    geo_prefilter_skip_above_score_ = cfg.loopGeoPrefilterSkipAboveScore();
    min_inlier_ratio_  = cfg.teaserMinInlierRatio();
    max_rmse_          = cfg.teaserMaxRMSE();
    use_icp_refine_    = cfg.teaserICPRefine();
    intra_submap_enabled_ = cfg.intraSubmapLoopEnabled();
    intra_submap_min_temporal_gap_ = cfg.intraSubmapLoopMinTemporalGap();
    intra_submap_min_keyframe_gap_ = cfg.intraSubmapLoopMinKeyframeGap();
    intra_submap_min_distance_gap_ = cfg.intraSubmapLoopMinDistanceGap();
    intra_submap_overlap_threshold_ = cfg.intraSubmapLoopOverlapThreshold();
    intra_submap_max_teaser_candidates_ = cfg.intraSubmapLoopMaxTeaserCandidates();
    inter_keyframe_level_enabled_ = cfg.interKeyframeLevelEnabled();
    inter_keyframe_sample_step_ = cfg.interKeyframeSampleStep();
    inter_keyframe_top_k_per_submap_ = cfg.interKeyframeTopKPerSubmap();
    inter_submap_min_keyframe_gap_ = cfg.interSubmapMinKeyframeGap();
    use_scancontext_ = cfg.scancontextEnabled();
    sc_dist_threshold_ = cfg.scancontextDistThreshold();
    sc_num_candidates_ = cfg.scancontextNumCandidates();
    sc_exclude_recent_ = cfg.scancontextExcludeRecent();
    sc_tree_making_period_ = cfg.scancontextTreeMakingPeriod();
    zero_accept_warn_consecutive_queries_ = cfg.loopZeroAcceptWarnConsecutiveQueries();
    ot_preferred_flow_ = cfg.loopOtPreferredFlow();
    allow_sc_fallback_ = cfg.loopAllowScFallback();
    allow_descriptor_fallback_ = cfg.loopAllowDescriptorFallback();
    allow_svd_geom_fallback_ = cfg.loopAllowSvdGeomFallback();
    log_effective_flow_ = cfg.loopLogEffectiveFlow();
    loop_flow_mode_ = cfg.loopFlowMode();
    parallel_teaser_max_inflight_ = cfg.parallelTeaserMaxInflight();
    pose_consistency_max_trans_m_ = cfg.loopPoseConsistencyMaxTransDiffM();
    pose_consistency_max_rot_deg_ = cfg.loopPoseConsistencyMaxRotDiffDeg();
    loop_max_desc_queue_size_ = cfg.loopMaxDescQueueSize();
    loop_max_match_queue_size_ = cfg.loopMaxMatchQueueSize();
    teaser_min_safe_inliers_ = cfg.teaserMinSafeInliers();
    parallel_teaser_match_ = cfg.parallelTeaserMatch();
    svd_temp_enable_after_fpfh_critical_ = 3;
    svd_temp_enable_budget_max_ = 24;
    svd_temp_enable_budget_left_ = 0;
    consecutive_fpfh_critical_rejects_ = 0;

    teaser_matcher_.applyConfig();
    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector][CONFIG] TEASER applied from YAML: min_safe_inliers=%d min_inlier_ratio=%.2f max_rmse=%.2f (grep CONFIG 验证)",
        teaser_min_safe_inliers_, min_inlier_ratio_, max_rmse_);
    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector][CONFIG] retrieve params from YAML: overlap_threshold=%.2f top_k=%d sc_dist_threshold=%.3f min_temporal_gap_s=%.1f (grep CONFIG 验证)",
        overlap_threshold_, top_k_, sc_dist_threshold_, min_temporal_gap_);
    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector][CONFIG] 运行验证: grep CONFIG 查 TEASER/retrieve/geo_prefilter 参数; grep INTRA_LOOP 查子图内 SUMMARY/DETECTED; grep LOOP_ACCEPTED 或 addLoopFactor 或 loop_intra 查回环入图");

    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector][CONFIG] min_submap_gap=%d (0=allow adjacent submaps; intra-submap candidates not filtered by gap)",
        min_submap_gap_);
    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector][CONFIG] geo_prefilter: max_distance_m=%.1f skip_above_score=%.2f (0=off; grep CONFIG 验证)",
        geo_prefilter_max_distance_m_, geo_prefilter_skip_above_score_);
    RCLCPP_INFO(node->get_logger(),
        "[DIAG][LOOP][CONFIG] flow_mode=%s ot_preferred=%d allow_sc_fallback=%d allow_descriptor_fallback=%d "
        "allow_svd_geom_fallback=%d scancontext_enabled=%d overlap_threshold=%.2f top_k=%d min_temporal_gap_s=%.1f "
        "parallel_teaser_match=%d parallel_teaser_max_inflight=%d min_submap_gap=%d geo_prefilter_max_distance_m=%.1f",
        loop_flow_mode_.c_str(),
        ot_preferred_flow_ ? 1 : 0,
        allow_sc_fallback_ ? 1 : 0,
        allow_descriptor_fallback_ ? 1 : 0,
        allow_svd_geom_fallback_ ? 1 : 0,
        use_scancontext_ ? 1 : 0,
        overlap_threshold_, top_k_, min_temporal_gap_,
        parallel_teaser_match_ ? 1 : 0, parallel_teaser_max_inflight_,
        min_submap_gap_, geo_prefilter_max_distance_m_);

    // 加载 OverlapTransformer 模型（LibTorch Level 1）
    std::string model_path = cfg.overlapModelPath();
    RCLCPP_INFO(node->get_logger(),
        "[OT] overlap_transformer.model_path (resolved)=%s (empty=use ScanContext only; grep [OT] for load/inference state)",
        model_path.empty() ? "(empty)" : model_path.c_str());
    if (!model_path.empty()) {
        bool loaded = overlap_infer_.loadModel(model_path, true);
        RCLCPP_INFO(node->get_logger(),
            "[OT] load result: %s (from %s); loop closure descriptor=%s",
            loaded ? "OK" : "FAIL_or_SKIP",
            model_path.c_str(),
            loaded ? "LibTorch (overlapTransformer.pt)" : (use_scancontext_ ? "ScanContext" : "fallback"));
        if (!loaded && !use_scancontext_ && cfg.loopAutoEnableScancontextOnOtFailure()) {
            use_scancontext_ = true;
            RCLCPP_WARN(node->get_logger(),
                "[LOOP_HEALTH][DEGRADED->RECOVER] OT load failed and scancontext.disabled in config; auto-enable ScanContext to avoid fallback-only mode");
        }
    }
    const bool strict_mode = (loop_flow_mode_ == "strict");
    if (log_effective_flow_) {
        const bool ot_ready = overlap_infer_.isModelLoaded();
        const char* retrieval_path =
            ot_ready ? "OT" : (use_scancontext_ ? "ScanContext" : "DescriptorFallback");
        RCLCPP_INFO(node->get_logger(),
            "[LOOP_FLOW] effective_flow mode=%s ot_preferred=%d retrieval=%s allow_sc_fallback=%d allow_descriptor_fallback=%d allow_svd_geom_fallback=%d parallel_teaser_max_inflight=%d",
            loop_flow_mode_.c_str(),
            ot_preferred_flow_ ? 1 : 0, retrieval_path, allow_sc_fallback_ ? 1 : 0,
            allow_descriptor_fallback_ ? 1 : 0, allow_svd_geom_fallback_ ? 1 : 0,
            parallel_teaser_max_inflight_);
    }
    if (strict_mode && ot_preferred_flow_ && !overlap_infer_.isModelLoaded() &&
        !allow_sc_fallback_ && !allow_descriptor_fallback_) {
        loop_ot_unavailable_event_total_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR(node->get_logger(),
            "[LOOP_FLOW][STRICT] OT mandatory mode but model is unavailable. coarse matching is blocked until %s is loadable.",
            cfg.overlapModelPath().c_str());
        RCLCPP_ERROR(node->get_logger(),
            "[DIAG][LOOP][E_FLOW_BLOCKED] strict OT flow blocked: model_not_loaded=%d model_path=%s "
            "allow_sc_fallback=%d allow_descriptor_fallback=%d",
            overlap_infer_.isModelLoaded() ? 0 : 1,
            cfg.overlapModelPath().c_str(),
            allow_sc_fallback_ ? 1 : 0,
            allow_descriptor_fallback_ ? 1 : 0);
    } else if (!strict_mode && ot_preferred_flow_ && !overlap_infer_.isModelLoaded()) {
        loop_ot_unavailable_event_total_.fetch_add(1, std::memory_order_relaxed);
        if (!use_scancontext_) {
            use_scancontext_ = true;
            RCLCPP_WARN(node->get_logger(),
                "[LOOP_EVENT][OT_UNAVAILABLE] mode=safe_degraded action=enable_scancontext reason=ot_model_not_loaded model_path=%s",
                cfg.overlapModelPath().c_str());
        } else {
            RCLCPP_WARN(node->get_logger(),
                "[LOOP_EVENT][OT_UNAVAILABLE] mode=safe_degraded action=keep_scancontext reason=ot_model_not_loaded model_path=%s",
                cfg.overlapModelPath().c_str());
        }
    }
#ifndef USE_TEASER
    if (strict_mode && !allow_svd_geom_fallback_) {
        loop_teaser_unavailable_event_total_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_ERROR(node->get_logger(),
            "[LOOP_FLOW][STRICT] TEASER mandatory mode but USE_TEASER is OFF at build-time. geometric verification will be blocked.");
        RCLCPP_ERROR(node->get_logger(),
            "[DIAG][LOOP][E_GEOM_BLOCKED] strict geometry flow blocked: USE_TEASER=0 allow_svd_geom_fallback=%d",
            allow_svd_geom_fallback_ ? 1 : 0);
    } else {
        loop_teaser_unavailable_event_total_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN(node->get_logger(),
            "[LOOP_EVENT][TEASER_UNAVAILABLE] mode=%s action=svd_fallback_allowed=%d reason=build_without_use_teaser",
            loop_flow_mode_.c_str(), allow_svd_geom_fallback_ ? 1 : 0);
    }
#endif

    // 可选：外部 Python Service（Level 2，仅在 LibTorch 不可用时）
#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    if (!overlap_infer_.isModelLoaded()) {
        desc_client_ = node->create_client<overlap_transformer_msgs::srv::ComputeDescriptor>(
            "/automap/compute_descriptor");
        if (desc_client_->wait_for_service(std::chrono::seconds(2))) {
            use_external_desc_service_ = true;
            RCLCPP_INFO(node->get_logger(),
                "[LoopDetector] External descriptor service available (Level 2 fallback)");
        }
    }
#endif

    // 初始化 ScanContext
    if (use_scancontext_) {
        ALOG_INFO(MOD, "[ScanContext] Enabled: dist_threshold={:.3f} num_candidates={} exclude_recent={}",
                  sc_dist_threshold_, sc_num_candidates_, sc_exclude_recent_);
    } else if (!overlap_infer_.isModelLoaded()) {
        RCLCPP_ERROR(node->get_logger(),
            "[LOOP_HEALTH][CRITICAL] OT unavailable and ScanContext disabled: loop detection is in weak fallback mode and may reject most candidates");
    }

    constraint_pub_ = node->create_publisher<automap_pro::msg::LoopConstraintMsg>(
        "/automap/loop_constraint", 100);
    RCLCPP_INFO(node->get_logger(), "[LoopDetector][TOPIC] publish: /automap/loop_constraint");

    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector][LibTorch] compile-time USE_TORCH=%d (1=can load .pt, 0=fallback only; see build.log for CMake status)",
        OverlapTransformerInfer::builtWithTorch() ? 1 : 0);
    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector] Initialized (workers=%d, OT=%s, TEASER=%s)",
        worker_thread_num_,
        overlap_infer_.isModelLoaded() ? "LibTorch" : (use_scancontext_ ? "ScanContext" : "fallback"),
#ifdef USE_TEASER
        "enabled"
#else
        "disabled"
#endif
    );
    ALOG_INFO(MOD,
        "[LOOP_DESIGN] inter-submap=SUBMAP-level (one descriptor per submap, one constraint per pair). "
        "intra-submap=KEYFRAME-level (keyframe-to-keyframe within same submap).");
    RCLCPP_INFO(node->get_logger(),
        "[LOOP_DESIGN] inter=submap-level (1 desc/submap); intra=keyframe-level (kf-to-kf in same submap)");
    if (inter_keyframe_level_enabled_) {
        ALOG_INFO(MOD, "[LOOP_DESIGN] inter_keyframe_level=ON (子图间关键帧级): sample_step={} top_k_per_submap={}",
                  inter_keyframe_sample_step_, inter_keyframe_top_k_per_submap_);
        RCLCPP_INFO(node->get_logger(),
            "[LOOP_DESIGN] inter_keyframe_level=ON sample_step=%d top_k_per_submap=%d",
            inter_keyframe_sample_step_, inter_keyframe_top_k_per_submap_);
    }
}

void LoopDetector::start() {
    running_ = true;

    // 启动描述子 Worker 池（并行）
    for (int i = 0; i < worker_thread_num_; ++i) {
        desc_workers_.emplace_back(&LoopDetector::descWorkerLoop, this);
    }

    // 启动 TEASER++ 匹配 Worker（单线程，CPU密集）
    match_worker_ = std::thread(&LoopDetector::matchWorkerLoop, this);
}

void LoopDetector::stop() {
    ALOG_INFO(MOD, "[SHUTDOWN] LoopDetector::stop() entered (running_=false, joining desc_workers + match_worker)");
    running_ = false;
    desc_cv_.notify_all();
    match_cv_.notify_all();
    for (auto& t : desc_workers_) if (t.joinable()) t.join();
    if (match_worker_.joinable()) match_worker_.join();
    ALOG_INFO(MOD, "[SHUTDOWN] LoopDetector::stop() done (all workers joined)");
}

void LoopDetector::addSubmap(const SubMap::Ptr& submap) {
    // 添加数据有效性检查
    if (!submap) {
        ALOG_WARN(MOD, "addSubmap: null submap ignored");
        return;
    }
    if (!submap->downsampled_cloud || submap->downsampled_cloud->empty()) {
        ALOG_WARN(MOD, "addSubmap: submap #{} has empty cloud, skipped for loop detection", submap->id);
        return;
    }
    if (submap->keyframes.empty()) {
        ALOG_WARN(MOD, "addSubmap: submap #{} has no keyframes, skipped for loop detection", submap->id);
        return;
    }

    float priority = static_cast<float>(submap->id);
    size_t qsize;
    size_t dbsize;
    {
        std::lock_guard<std::mutex> lk(desc_mutex_);
        const size_t max_desc = loop_max_desc_queue_size_;
        while (desc_queue_.size() >= max_desc) {
            desc_queue_.pop();  // 丢弃最低优先级，防无界堆积
            loop_desc_queue_drop_total_.fetch_add(1, std::memory_order_relaxed);
        }
        desc_queue_.push({submap, priority});
        qsize = desc_queue_.size();
        dbsize = dbSize();
    }
    desc_cv_.notify_one();
    ALOG_INFO(MOD, "[LOOP_STEP] stage=addSubmap sm_id={} kf={} desc_pts={} desc_queue={} db_size={} (inter-submap: 1 desc per submap from downsampled_cloud; grep LOOP_STEP)",
              submap->id, submap->keyframes.size(),
              submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u, qsize, dbsize);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=addSubmap sm_id=%d kf=%zu desc_pts=%zu desc_queue=%zu db_size=%zu",
                    submap->id, submap->keyframes.size(),
                    submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u, qsize, dbsize);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Worker: 描述子计算（并行 Stage 0）
// ─────────────────────────────────────────────────────────────────────────────
void LoopDetector::descWorkerLoop() {
    while (running_) {
        SubMap::Ptr submap;
        {
            std::unique_lock<std::mutex> lk(desc_mutex_);
            desc_cv_.wait(lk, [this] {
                return !desc_queue_.empty() || !running_;
            });
            if (!running_ && desc_queue_.empty()) break;
            submap = desc_queue_.top().submap;
            desc_queue_.pop();
        }
        if (!submap) continue;

    size_t queue_remaining = 0;
    { std::lock_guard<std::mutex> lk(desc_mutex_); queue_remaining = desc_queue_.size(); }
    ALOG_INFO(MOD, "[LOOP_STEP] stage=desc_worker_pop submap_id={} desc_queue_remaining={} use_scancontext={}",
              submap->id, queue_remaining, use_scancontext_);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=desc_worker_pop submap_id=%d desc_queue_remaining=%zu use_scancontext=%d",
                    submap->id, queue_remaining, use_scancontext_ ? 1 : 0);
    }

    computeDescriptorAsync(submap);
    }
}

void LoopDetector::computeDescriptorAsync(const SubMap::Ptr& submap) {
    if (!submap->downsampled_cloud || submap->downsampled_cloud->empty()) return;

    // 回环强制使用 ScanContext 时，不计算 OT/fallback 描述子，直接进入候选检索（检索阶段用 ScanContext 计算描述子）
    if (use_scancontext_) {
        submap->has_descriptor = true;
        onDescriptorReady(submap);
        return;
    }

    // Level 1: LibTorch 进程内推理
    if (overlap_infer_.isModelLoaded()) {
        size_t pts = submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u;
        ALOG_INFO(MOD, "[LOOP_DESC][SUBMAP] phase=compute_enter submap_id={} pts={} (异步计算子图 OT 描述子)",
                  submap->id, pts);
        submap->overlap_descriptor = overlap_infer_.computeDescriptor(
            submap->downsampled_cloud);
        submap->overlap_descriptor_norm = submap->overlap_descriptor.norm();
        submap->has_descriptor = true;
        ALOG_INFO(MOD, "[LOOP_DESC][SUBMAP] phase=compute_done submap_id={} desc_norm={:.4f}",
                  submap->id, submap->overlap_descriptor_norm);
        onDescriptorReady(submap);
        return;
    }

    // Level 2: 外部 Python Service（异步回调，不阻塞 Worker 线程）
#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    if (use_external_desc_service_ && desc_client_) {
        auto req = std::make_shared<overlap_transformer_msgs::srv::ComputeDescriptor::Request>();
        pcl::toROSMsg(*submap->downsampled_cloud, req->pointcloud);
        req->pointcloud.header.stamp    = node()->now();
        req->pointcloud.header.frame_id = "map";  // 点云为世界系（与 merged_cloud/downsampled_cloud 一致）

        // ✅ 正确异步模式：async_send_request + callback，不阻塞 Worker 线程
        desc_client_->async_send_request(req,
            [this, submap](rclcpp::Client<overlap_transformer_msgs::srv::ComputeDescriptor>::SharedFuture future) {
                try {
                    const auto& data = future.get()->descriptor.data;
                    if ((int)data.size() == 256) {
                        submap->overlap_descriptor = Eigen::VectorXf::Map(data.data(), 256);
                        submap->has_descriptor = true;
                    } else {
                        // fallback
                        submap->overlap_descriptor = overlap_infer_.computeDescriptor(
                            submap->downsampled_cloud);
                        submap->has_descriptor = true;
                    }
                    onDescriptorReady(submap);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(node()->get_logger(),
                        "[LoopDetector] Descriptor service error: %s, using fallback", e.what());
                    submap->overlap_descriptor = overlap_infer_.computeDescriptor(
                        submap->downsampled_cloud);
                    submap->has_descriptor = true;
                    onDescriptorReady(submap);
                }
            });
        return;
    }
#endif

    // Level 3: fallback（直方图）
    size_t pts = submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u;
    ALOG_INFO(MOD, "[LOOP_DESC][SUBMAP] phase=compute_enter submap_id={} pts={} (fallback 描述子)",
              submap->id, pts);
    submap->overlap_descriptor = overlap_infer_.computeDescriptor(
        submap->downsampled_cloud);
    submap->overlap_descriptor_norm = submap->overlap_descriptor.norm();
    submap->has_descriptor = true;
    ALOG_INFO(MOD, "[LOOP_DESC][SUBMAP] phase=compute_done submap_id={} desc_norm={:.4f}",
              submap->id, submap->overlap_descriptor_norm);
    onDescriptorReady(submap);
}

void LoopDetector::onDescriptorReady(const SubMap::Ptr& submap) {
    // auto t_start = std::chrono::steady_clock::now();
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=descriptor_done submap_id={} query_pts={} (描述子就绪，进入候选检索)",
              submap->id, submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=descriptor_done submap_id=%d query_pts=%zu",
                    submap->id, submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u);
    }

    // [FIX] 先将当前 submap 添加到数据库，供后续 submap 进行回环检测
    size_t db_size_before = 0;
    { std::shared_lock<std::shared_mutex> lk(db_mutex_); db_size_before = db_submaps_.size(); }
    addToDatabase(submap);
    size_t db_size_after = 0;
    { std::shared_lock<std::shared_mutex> lk(db_mutex_); db_size_after = db_submaps_.size(); }
    ALOG_INFO(MOD, "[LOOP_STEP] stage=addToDatabase submap_id={} db_before={} db_after={}",
              submap->id, db_size_before, db_size_after);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=addToDatabase submap_id=%d db_before=%zu db_after=%zu",
                    submap->id, db_size_before, db_size_after);
    }

    // 检索候选（Stage 1）
    std::vector<SubMap::Ptr> db_copy;
    {
        std::shared_lock<std::shared_mutex> lk(db_mutex_);
        db_copy = db_submaps_;
    }
    // 诊断：db_size<=1 时不可能有子图间候选
    if (db_copy.size() <= 1u) {
        ALOG_INFO(MOD, "[LOOP_PHASE] stage=inter_submap_opportunity query_id={} db_size={} → 无子图间机会(db_size<=1)，仅子图内/自匹配",
                  submap->id, db_copy.size());
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=inter_submap_opportunity query_id=%d db_size=%zu (no inter-submap: db_size<=1)",
                        submap->id, db_copy.size());
        }
    }

    ALOG_INFO(MOD, "[LOOP_STEP] stage=retrieve_enter submap_id={} db_size={} overlap_threshold={:.3f} top_k={} min_submap_gap={} min_temporal_gap_s={:.1f}",
              submap->id, db_copy.size(), overlap_threshold_, top_k_, min_submap_gap_, min_temporal_gap_);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=retrieve_enter submap_id=%d db_size=%zu overlap_threshold=%.3f top_k=%d min_submap_gap=%d",
                    submap->id, db_copy.size(), overlap_threshold_, top_k_, min_submap_gap_);
    }

    auto t_retrieve_start = std::chrono::steady_clock::now();
    std::vector<OverlapTransformerInfer::Candidate> candidates;

    // 根据配置选择候选检索方法（OT 优先 + 显式 fallback guard）
    const bool ot_ready = overlap_infer_.isModelLoaded();
    const bool use_sc_path = use_scancontext_ && (!ot_preferred_flow_ || !ot_ready || allow_sc_fallback_);
    const bool use_desc_path = (!use_scancontext_) || (ot_preferred_flow_ && !ot_ready && !use_sc_path);
    const char* retrieval_path = use_sc_path ? "ScanContext" : (ot_ready ? "OT" : "DescriptorFallback");

    if (ot_preferred_flow_ && !ot_ready && !allow_sc_fallback_ && !allow_descriptor_fallback_) {
        ALOG_WARN(MOD, "[LOOP_FLOW] query_id={} path=reject reason=ot_unavailable_and_fallbacks_disabled", submap->id);
        return;
    }
    if (!ot_ready && use_desc_path && !allow_descriptor_fallback_) {
        ALOG_WARN(MOD, "[LOOP_FLOW] query_id={} path=reject reason=descriptor_fallback_disabled", submap->id);
        return;
    }

    if (use_sc_path) {
        loop_sc_retrieval_total_.fetch_add(1, std::memory_order_relaxed);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[OT] retrieve: method=ScanContext submap_id=%d db_size=%zu top_k=%d",
                        submap->id, db_copy.size(), top_k_);
        }
        std::lock_guard<std::mutex> lk(sc_mutex_);
        candidates = retrieveUsingScanContext(submap, db_copy);
    } else {
        loop_ot_retrieval_total_.fetch_add(1, std::memory_order_relaxed);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[OT] retrieve: method=%s submap_id=%d db_size=%zu top_k=%d",
                        ot_ready ? "LibTorch(overlapTransformer.pt)" : "fallback_descriptor",
                        submap->id, db_copy.size(), top_k_);
        }
        candidates = overlap_infer_.retrieve(
            submap->overlap_descriptor,
            db_copy,
            top_k_,
            static_cast<float>(overlap_threshold_),
            min_submap_gap_,
            min_temporal_gap_,
            gps_search_radius_,
            submap->gps_center,
            submap->has_valid_gps);
    }
    ALOG_INFO(MOD, "[LOOP_FLOW] query_id={} path={} ot_ready={} use_scancontext={} candidates={}",
              submap->id, retrieval_path, ot_ready ? 1 : 0, use_scancontext_ ? 1 : 0, candidates.size());
    auto t_retrieve_end = std::chrono::steady_clock::now();
    double retrieve_ms = std::chrono::duration<double, std::milli>(t_retrieve_end - t_retrieve_start).count();

    // ✅ 详细诊断日志
    if (candidates.empty()) {
        ALOG_INFO(MOD, "[LOOP_STEP] stage=retrieve_result NO_CAND query_id={} db_size={} overlap_threshold={:.3f} top_k={} retrieve_ms={:.1f} (若轨迹有闭环可尝试降低 loop_closure.overlap_threshold 或检查 ScanContext dist_threshold)",
                  submap->id, db_copy.size(), overlap_threshold_, top_k_, retrieve_ms);
        ALOG_INFO(MOD, "[LOOP_PHASE] stage=inter_submap_opportunity query_id={} db_size={} → 检索无过阈值候选，子图间=0 (grep LOOP_PHASE 统计)",
                  submap->id, db_copy.size());
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=retrieve_result NO_CAND query_id=%d db_size=%zu overlap_threshold=%.3f top_k=%d",
                        submap->id, db_copy.size(), overlap_threshold_, top_k_);
            RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=inter_submap_opportunity query_id=%d NO_CAND (inter_submap=0)",
                        submap->id);
        }
        return;
    }
    ALOG_INFO(MOD, "[LOOP_STEP] stage=retrieve_result OK query_id={} raw_candidates={} retrieve_ms={:.1f}",
              submap->id, candidates.size(), retrieve_ms);
    std::string cand_detail;
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (i > 0) cand_detail += " ";
        cand_detail += fmt::format("(tgt{}={:.3f})", candidates[i].submap_id, candidates[i].score);
    }
    ALOG_INFO(MOD, "[LOOP_STEP] stage=retrieve_detail query_id={} raw_targets_and_scores=[{}]",
              submap->id, cand_detail);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=retrieve_result OK query_id=%d raw_candidates=%zu retrieve_ms=%.1f",
                    submap->id, candidates.size(), retrieve_ms);
    }

    // 过滤候选（时间/子图间隔）：子图内回环（同 submap_id）不按间隔过滤。子图间：min_submap_gap_==0 时不按间隔过滤（允许相邻子图回环），>0 时要求 gap>min_submap_gap_
    std::vector<OverlapTransformerInfer::Candidate> valid_candidates;
    int filtered_by_gap = 0;
    int same_session_count = 0;
    int same_submap_count = 0;
    int different_submap_count = 0;
    for (size_t c = 0; c < candidates.size(); ++c) {
        const auto& cand = candidates[c];
        int gap = std::abs(cand.submap_id - submap->id);
        if (cand.session_id != submap->session_id) {
            valid_candidates.push_back(cand);
            ALOG_INFO(MOD, "[LOOP_STEP] stage=gap_filter_cand query_id={} cand_idx={} target_id={} score={:.3f} gap={} reason=diff_session PASS",
                submap->id, c, cand.submap_id, cand.score, gap);
            continue;
        }
        if (cand.submap_id == submap->id) {
            same_submap_count++;
            ALOG_INFO(MOD, "[LOOP_STEP] stage=gap_filter_cand query_id={} cand_idx={} target_id={} score={:.3f} gap={} reason=same_submap FILTERED (submap-level self loop is invalid)",
                submap->id, c, cand.submap_id, cand.score, gap);
            continue;
        }
        same_session_count++;
        if (min_submap_gap_ > 0 && gap <= min_submap_gap_) {
            filtered_by_gap++;
            ALOG_INFO(MOD, "[LOOP_STEP] stage=gap_filter_cand query_id={} cand_idx={} target_id={} score={:.3f} gap={} min_submap_gap={} FILTERED (need gap>{})",
                submap->id, c, cand.submap_id, cand.score, gap, min_submap_gap_, min_submap_gap_);
            continue;
        }
        different_submap_count++;
        valid_candidates.push_back(cand);
        ALOG_INFO(MOD, "[LOOP_STEP] stage=gap_filter_cand query_id={} cand_idx={} target_id={} score={:.3f} gap={} min_submap_gap={} PASS",
            submap->id, c, cand.submap_id, cand.score, gap, min_submap_gap_);
    }

    // 🔧 添加详细诊断日志
    ALOG_INFO(MOD, "[LoopDetector][CAND_STATS] query_id={}: candidates={} same_session={} same_submap={} diff_submap={} filtered_by_gap={} valid={}",
        submap->id, candidates.size(), same_session_count, same_submap_count, different_submap_count, filtered_by_gap, valid_candidates.size());
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LoopDetector][CAND_STATS] query_id=%d: candidates=%zu same_session=%d same_submap=%d diff_submap=%d filtered_by_gap=%d valid=%zu",
            submap->id, candidates.size(), same_session_count, same_submap_count, different_submap_count, filtered_by_gap, valid_candidates.size());
    }

    if (valid_candidates.empty()) {
        ALOG_WARN(MOD, "[LOOP_STEP] stage=gap_filter ALL_FILTERED query_id={} raw_candidates={} min_submap_gap={} (可尝试将 loop_closure.min_submap_gap 调小)",
                  submap->id, candidates.size(), min_submap_gap_);
        if (node()) {
            RCLCPP_WARN(node()->get_logger(), "[LOOP_STEP] stage=gap_filter ALL_FILTERED query_id=%d raw_candidates=%zu min_submap_gap=%d",
                        submap->id, candidates.size(), min_submap_gap_);
        }
        return;
    }

    // temporal gap 过滤：确保 loop_closure.min_temporal_gap_s 在主链路真实生效
    std::vector<OverlapTransformerInfer::Candidate> temporal_filtered_candidates;
    int filtered_by_temporal = 0;
    const double query_time = submapRepresentativeTime(submap);
    for (const auto& cand : valid_candidates) {
        SubMap::Ptr target_submap;
        for (const auto& sm : db_copy) {
            if (sm && sm->id == cand.submap_id && sm->session_id == cand.session_id) {
                target_submap = sm;
                break;
            }
        }
        if (!target_submap || min_temporal_gap_ <= 0.0) {
            temporal_filtered_candidates.push_back(cand);
            continue;
        }
        const double target_time = submapRepresentativeTime(target_submap);
        const double dt = std::abs(query_time - target_time);
        if (query_time > 0.0 && target_time > 0.0 && dt < min_temporal_gap_) {
            filtered_by_temporal++;
            ALOG_INFO(MOD,
                      "[LOOP_STEP] stage=temporal_filter query_id={} target_id={} dt={:.2f}s min_temporal_gap_s={:.2f} FILTERED",
                      submap->id, cand.submap_id, dt, min_temporal_gap_);
            continue;
        }
        temporal_filtered_candidates.push_back(cand);
    }
    if (filtered_by_temporal > 0) {
        ALOG_INFO(MOD,
                  "[LOOP_STEP] stage=temporal_filter query_id={} filtered={} remain={} min_temporal_gap_s={:.2f}",
                  submap->id, filtered_by_temporal, temporal_filtered_candidates.size(), min_temporal_gap_);
    }
    if (temporal_filtered_candidates.empty()) {
        ALOG_WARN(MOD,
                  "[LOOP_STEP] stage=temporal_filter ALL_FILTERED query_id={} min_temporal_gap_s={:.2f}",
                  submap->id, min_temporal_gap_);
        return;
    }
    valid_candidates.swap(temporal_filtered_candidates);

    // 【几何距离预筛】按两子图锚定位姿距离过滤，抑制重复结构导致的误检（相似但远距离的候选）
    Eigen::Vector3d query_pos = submap->pose_odom_anchor.translation();
    std::vector<OverlapTransformerInfer::Candidate> geo_filtered_candidates;
    int filtered_by_geo = 0;
    for (const auto& cand : valid_candidates) {
        SubMap::Ptr target_submap;
        for (const auto& sm : db_copy) {
            if (sm && sm->id == cand.submap_id) {
                target_submap = sm;
                break;
            }
        }
        double geo_dist_m = -1.0;
        if (target_submap) {
            geo_dist_m = (query_pos - target_submap->pose_odom_anchor.translation()).norm();
            ALOG_INFO(MOD, "[LOOP_CAND] query_id={} target_id={} score={:.3f} geo_dist={:.1f}m",
                      submap->id, cand.submap_id, cand.score, geo_dist_m);
        } else {
            ALOG_INFO(MOD, "[LOOP_CAND] query_id={} target_id={} score={:.3f} geo_dist=N/A (target not in db_copy)",
                      submap->id, cand.submap_id, cand.score);
        }
        // 高置信度绕过：描述子 score≥阈值时跳过几何预筛（参考 OverlapTransformer/SeqOT：高 overlap 即可靠，闭环时锚点距离可能很大）
        if (geo_prefilter_skip_above_score_ > 0.0 && cand.score >= geo_prefilter_skip_above_score_) {
            geo_filtered_candidates.push_back(cand);
            ALOG_INFO(MOD, "[LOOP_CAND] query_id={} target_id={} score={:.3f} geo_dist={:.1f}m BYPASS (score>={:.2f})",
                      submap->id, cand.submap_id, cand.score, geo_dist_m, geo_prefilter_skip_above_score_);
            continue;
        }
        if (geo_prefilter_max_distance_m_ > 0.0 && geo_dist_m >= 0.0 && geo_dist_m > geo_prefilter_max_distance_m_) {
            filtered_by_geo++;
            ALOG_DEBUG(MOD, "[LoopDetector][GEO_PREFILTER] query_id={} target_id={} geo_dist={:.1f}m > max={:.1f}m → 过滤",
                       submap->id, cand.submap_id, geo_dist_m, geo_prefilter_max_distance_m_);
            continue;
        }
        geo_filtered_candidates.push_back(cand);
    }

    if (geo_filtered_candidates.empty()) {
        ALOG_WARN(MOD, "[LOOP_STEP] stage=geo_prefilter ALL_FILTERED query_id={} valid_candidates={} geo_prefilter_max_distance_m={:.1f}",
                  submap->id, valid_candidates.size(), geo_prefilter_max_distance_m_);
        if (node()) {
            RCLCPP_WARN(node()->get_logger(), "[LOOP_STEP] stage=geo_prefilter ALL_FILTERED query_id=%d valid_candidates=%zu geo_max_m=%.1f",
                        submap->id, valid_candidates.size(), geo_prefilter_max_distance_m_);
        }
        return;
    }
    if (filtered_by_geo > 0) {
        ALOG_INFO(MOD, "[LoopDetector][GEO_PREFILTER] query_id={} 几何预筛过滤 {}/{} 个候选 (max_dist={:.1f}m)，剩余 {}",
                  submap->id, filtered_by_geo, valid_candidates.size(), geo_prefilter_max_distance_m_, geo_filtered_candidates.size());
    }

    // ✅ 输出候选的相似度分布 + 性能数据（使用几何预筛后的列表）
    std::string score_str;
    for (size_t i = 0; i < geo_filtered_candidates.size(); ++i) {
        score_str += fmt::format("{:.3f}", geo_filtered_candidates[i].score);
        if (i < geo_filtered_candidates.size() - 1) score_str += ", ";
    }
    // auto t_end = std::chrono::steady_clock::now();
    // double total_stage1_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    // 子图间候选数（用于诊断“为何子图间回环少”：是没候选还是被过滤）
    int inter_submap_in_geo = 0;
    for (const auto& c : geo_filtered_candidates)
        if (c.submap_id != submap->id) inter_submap_in_geo++;
    ALOG_INFO(MOD, "[LoopDetector][CAND] query_id={} db={} candidates={} scores=[{}] retrieve_ms={:.1f} → TEASER++",
              submap->id, db_copy.size(), geo_filtered_candidates.size(), score_str, retrieve_ms);
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=candidates_retrieved query_id={} candidate_count={} inter_submap_candidates={} retrieve_ms={:.1f} (inter_submap=0 表示无子图间候选；grep LOOP_PHASE 统计)",
              submap->id, geo_filtered_candidates.size(), inter_submap_in_geo, retrieve_ms);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=candidates_retrieved query_id=%d candidate_count=%zu inter_submap_candidates=%d retrieve_ms=%.1f",
                    submap->id, geo_filtered_candidates.size(), inter_submap_in_geo, retrieve_ms);
    }
    // ── 子图间关键帧级：为每个采样关键帧检索跨子图关键帧候选并入队 ──
    if (inter_keyframe_level_enabled_ && use_scancontext_ && !geo_filtered_candidates.empty() &&
        submap->keyframes.size() > 0) {
        prepareIntraSubmapDescriptors(submap);
        const int step = inter_keyframe_sample_step_;
        const int top_k_per_sm = inter_keyframe_top_k_per_submap_;
        int tasks_enqueued = 0;
        std::lock_guard<std::mutex> lk(match_mutex_);
        const size_t max_match = loop_max_match_queue_size_;
        for (size_t qkf = 0; qkf < submap->keyframes.size(); qkf += step) {
            if (qkf >= submap->keyframe_scancontexts_.size() || qkf >= submap->keyframe_clouds_ds.size()) break;
            const KeyFrame::Ptr qkf_ptr = (qkf < submap->keyframes.size()) ? submap->keyframes[qkf] : nullptr;
            if (qkf_ptr && min_keyframe_interval_after_success_ > 0 && last_keyframe_id_after_loop_success_ >= 0) {
                const int64_t delta = static_cast<int64_t>(qkf_ptr->id) - last_keyframe_id_after_loop_success_;
                if (delta >= 0 && delta <= static_cast<int64_t>(min_keyframe_interval_after_success_)) {
                    ALOG_INFO(MOD, "[LOOP_THROTTLE] skip inter query_kf qkf=%zu kf_id=%lu (within %d frames after last success)",
                        qkf, qkf_ptr->id, min_keyframe_interval_after_success_);
                    continue;
                }
            }
            const auto& query_sc = submap->keyframe_scancontexts_[qkf];
            if (query_sc.size() == 0) continue;
            const auto& query_kf_cloud = submap->keyframe_clouds_ds[qkf];
            if (!query_kf_cloud || query_kf_cloud->empty()) continue;

            std::vector<InterKfCandidate> kf_cands;
            for (const auto& gc : geo_filtered_candidates) {
                SubMap::Ptr tsm;
                for (const auto& sm : db_copy) {
                    if (sm->id == gc.submap_id && sm->session_id == gc.session_id) { tsm = sm; break; }
                }
                if (!tsm || tsm->keyframe_scancontexts_.size() != tsm->keyframes.size()) continue;
                std::vector<std::pair<int, double>> dists;
                for (size_t tkf = 0; tkf < tsm->keyframe_scancontexts_.size(); ++tkf) {
                    const auto& tsc = tsm->keyframe_scancontexts_[tkf];
                    if (tsc.size() == 0) continue;
                    std::pair<double, int> res;
                    {
                        std::lock_guard<std::mutex> sc_lk(sc_mutex_);
                        res = sc_manager_.distanceBtnScanContext(query_sc, tsc);
                    }
                    if (res.first >= sc_dist_threshold_) continue;
                    // float score = std::max(0.f, 1.f - static_cast<float>(res.first / sc_dist_threshold_));
                    dists.push_back({static_cast<int>(tkf), res.first});
                }
                std::sort(dists.begin(), dists.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
                for (int i = 0; i < top_k_per_sm && i < static_cast<int>(dists.size()); ++i) {
                    double d = dists[i].second;
                    float score = static_cast<float>(std::max(0., 1. - d / sc_dist_threshold_));
                    kf_cands.push_back({tsm->id, dists[i].first, score});
                }
            }
            if (kf_cands.empty()) continue;
            while (match_queue_.size() >= max_match) {
                match_queue_.pop();
                loop_match_queue_drop_total_.fetch_add(1, std::memory_order_relaxed);
            }
            CloudXYZIPtr kf_cloud_copy = std::make_shared<CloudXYZI>();
            *kf_cloud_copy = *query_kf_cloud;
            MatchTask kf_task;
            kf_task.query = submap;
            kf_task.query_cloud = kf_cloud_copy;
            kf_task.query_kf_idx = static_cast<int>(qkf);
            kf_task.candidates_kf = std::move(kf_cands);
            match_queue_.push(std::move(kf_task));
            tasks_enqueued++;
        }
        match_cv_.notify_one();
        ALOG_INFO(MOD, "[LOOP_PHASE] stage=match_enqueue INTER_KEYFRAME query_id={} sampled_kf_tasks={} (关键帧级子图间)",
                  submap->id, tasks_enqueued);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=match_enqueue INTER_KEYFRAME query_id=%d tasks=%d",
                        submap->id, tasks_enqueued);
        }
        return;
    }

    // ── 子图间关键帧级（OverlapTransformer）：query 关键帧描述子 vs 候选子图内关键帧描述子（参考 SeqOT/EINet 关键帧级匹配）
    if (inter_keyframe_level_enabled_ && !use_scancontext_ && overlap_infer_.isModelLoaded() &&
        !geo_filtered_candidates.empty() && submap->keyframes.size() > 0) {
        prepareIntraSubmapDescriptors(submap);
        if (submap->keyframe_descriptors.size() != submap->keyframes.size() ||
            submap->keyframe_clouds_ds.size() != submap->keyframes.size()) {
            ALOG_WARN(MOD, "[LOOP_PHASE] INTER_KEYFRAME_OT query_id={} keyframe_descriptors size mismatch, fallback to submap-level",
                      submap->id);
        } else {
            for (const auto& gc : geo_filtered_candidates) {
                if (gc.submap_id == submap->id) continue;
                SubMap::Ptr tsm;
                for (const auto& sm : db_copy) {
                    if (sm && sm->id == gc.submap_id) { tsm = sm; break; }
                }
                if (tsm) prepareIntraSubmapDescriptors(tsm);
            }
            const int step = inter_keyframe_sample_step_;
            const int top_k_per_sm = inter_keyframe_top_k_per_submap_;
            const float threshold = static_cast<float>(overlap_threshold_);
            int tasks_enqueued = 0;
            int total_candidates_kf = 0;
            std::lock_guard<std::mutex> lk(match_mutex_);
            const size_t max_match = loop_max_match_queue_size_;
            for (size_t qkf = 0; qkf < submap->keyframes.size(); qkf += step) {
                if (qkf >= submap->keyframe_descriptors.size() || qkf >= submap->keyframe_clouds_ds.size()) break;
                const KeyFrame::Ptr qkf_ptr = (qkf < submap->keyframes.size()) ? submap->keyframes[qkf] : nullptr;
                if (qkf_ptr && min_keyframe_interval_after_success_ > 0 && last_keyframe_id_after_loop_success_ >= 0) {
                    const int64_t delta = static_cast<int64_t>(qkf_ptr->id) - last_keyframe_id_after_loop_success_;
                    if (delta >= 0 && delta <= static_cast<int64_t>(min_keyframe_interval_after_success_)) {
                        ALOG_INFO(MOD, "[LOOP_THROTTLE] skip inter query_kf qkf=%zu kf_id=%lu (within %d frames after last success)",
                            qkf, qkf_ptr->id, min_keyframe_interval_after_success_);
                        continue;
                    }
                }
                const auto& query_desc = submap->keyframe_descriptors[qkf];
                CloudXYZIPtr query_kf_cloud = submap->keyframe_clouds_ds[qkf];
                if (!query_kf_cloud || query_kf_cloud->empty()) continue;
                float query_norm = query_desc.norm();
                if (query_norm < 1e-6f) continue;

                std::vector<InterKfCandidate> kf_cands;
                for (const auto& gc : geo_filtered_candidates) {
                    if (gc.submap_id == submap->id) continue;
                    SubMap::Ptr tsm;
                    for (const auto& sm : db_copy) {
                        if (sm && sm->id == gc.submap_id) { tsm = sm; break; }
                    }
                    if (!tsm || tsm->keyframe_descriptors.size() != tsm->keyframes.size()) continue;
                    std::vector<std::pair<int, float>> scored;
                    for (size_t tkf = 0; tkf < tsm->keyframe_descriptors.size(); ++tkf) {
                        float sim = query_desc.dot(tsm->keyframe_descriptors[tkf]) /
                            (query_norm * (tsm->keyframe_descriptors[tkf].norm() + 1e-8f));
                        if (sim >= threshold) scored.push_back({static_cast<int>(tkf), sim});
                    }
                    std::partial_sort(scored.begin(),
                        scored.begin() + std::min(top_k_per_sm, static_cast<int>(scored.size())),
                        scored.end(),
                        [](const auto& a, const auto& b) { return a.second > b.second; });
                    for (int i = 0; i < top_k_per_sm && i < static_cast<int>(scored.size()); ++i)
                        kf_cands.push_back({tsm->id, scored[i].first, scored[i].second});
                }
                if (kf_cands.empty()) continue;
                total_candidates_kf += static_cast<int>(kf_cands.size());
                while (match_queue_.size() >= max_match) {
                    match_queue_.pop();
                    loop_match_queue_drop_total_.fetch_add(1, std::memory_order_relaxed);
                }
                CloudXYZIPtr kf_cloud_copy = std::make_shared<CloudXYZI>();
                *kf_cloud_copy = *query_kf_cloud;
                MatchTask kf_task;
                kf_task.query = submap;
                kf_task.query_cloud = kf_cloud_copy;
                kf_task.query_kf_idx = static_cast<int>(qkf);
                kf_task.candidates_kf = std::move(kf_cands);
                match_queue_.push(std::move(kf_task));
                tasks_enqueued++;
            }
            match_cv_.notify_one();
            if (tasks_enqueued > 0) {
                ALOG_INFO(MOD, "[LOOP_PHASE] stage=match_enqueue INTER_KEYFRAME_OT query_id={} tasks={} total_candidates_kf={} (共 {} 次子图间关键帧对待 TEASER；grep LOOP_PHASE 统计)",
                          submap->id, tasks_enqueued, total_candidates_kf, total_candidates_kf);
                if (node()) {
                    RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=match_enqueue INTER_KEYFRAME_OT query_id=%d tasks=%d total_candidates_kf=%d",
                                submap->id, tasks_enqueued, total_candidates_kf);
                }
                return;
            }
        }
    }

    // 子图级入队（当未启用关键帧级或无候选时）
    CloudXYZIPtr query_cloud_copy;
    if (submap->downsampled_cloud && !submap->downsampled_cloud->empty()) {
        query_cloud_copy = std::make_shared<CloudXYZI>();
        *query_cloud_copy = *submap->downsampled_cloud;
        ALOG_DEBUG(MOD, "enqueue MatchTask query_id={} query_cloud_pts={} candidates={}",
                   submap->id, query_cloud_copy->size(), geo_filtered_candidates.size());
    }
    {
        std::lock_guard<std::mutex> lk(match_mutex_);
        const size_t max_match = loop_max_match_queue_size_;
        while (match_queue_.size() >= max_match) {
            match_queue_.pop();
            loop_match_queue_drop_total_.fetch_add(1, std::memory_order_relaxed);
        }
        MatchTask task;
        task.query = submap;
        task.query_cloud = query_cloud_copy;
        task.candidates = geo_filtered_candidates;
        match_queue_.push(std::move(task));
    }
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=match_enqueue query_id={} candidates={} query_pts={} (子图级)",
              submap->id, geo_filtered_candidates.size(), query_cloud_copy ? query_cloud_copy->size() : 0u);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_PHASE] stage=match_enqueue query_id=%d candidates=%zu query_pts=%zu",
                    submap->id, geo_filtered_candidates.size(), query_cloud_copy ? query_cloud_copy->size() : 0u);
    }
    match_cv_.notify_one();
}

// ─────────────────────────────────────────────────────────────────────────────
// Worker: TEASER++ 匹配（单线程 Stage 2）
// ─────────────────────────────────────────────────────────────────────────────
void LoopDetector::matchWorkerLoop() {
    // 限制本线程内 OpenMP/PMC 为单线程，缓解 TEASER++ 析构时 free() SIGSEGV（见 docs/TEASER_CRASH_ANALYSIS.md）
#if defined(__unix__) || defined(__linux__)
    setenv("OMP_NUM_THREADS", "1", 1);
#endif
    IcpRefiner icp_refiner;
    while (running_) {
        MatchTask task;
        {
            std::unique_lock<std::mutex> lk(match_mutex_);
            match_cv_.wait(lk, [this] {
                return !match_queue_.empty() || !running_;
            });
            if (!running_ && match_queue_.empty()) break;
            task = std::move(match_queue_.front());
            match_queue_.pop();
        }
        size_t match_queue_remaining = 0;
        { std::lock_guard<std::mutex> lk(match_mutex_); match_queue_remaining = match_queue_.size(); }
        ALOG_INFO(MOD, "[LOOP_STEP] stage=match_worker_pop query_id={} candidates={} candidates_kf={} match_queue_remaining={}",
                  task.query ? task.query->id : -1, task.candidates.size(), task.candidates_kf.size(), match_queue_remaining);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=match_worker_pop query_id=%d candidates=%zu candidates_kf=%zu match_queue_remaining=%zu",
                        task.query ? task.query->id : -1, task.candidates.size(), task.candidates_kf.size(), match_queue_remaining);
        }
        
        try {
            processMatchTask(task);
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] step=match_worker_exception query_id={} what={}", 
                      automap_pro::logThreadId(), task.query ? task.query->id : -1, e.what());
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=match_worker_unknown_exception query_id={}", 
                      automap_pro::logThreadId(), task.query ? task.query->id : -1);
        }
    }
}

void LoopDetector::processMatchTask(const MatchTask& task) {
    const unsigned tid = automap_pro::logThreadId();

    if (!task.query) {
        ALOG_WARN(MOD, "[tid=%u] processMatchTask: null query submap, skipping", tid);
        return;
    }

    // ── 子图间关键帧级：query_kf_idx >= 0 且 candidates_kf 非空 ──
    if (task.query_kf_idx >= 0 && !task.candidates_kf.empty()) {
        CloudXYZIPtr query_cloud = task.query_cloud;
        if (!query_cloud || query_cloud->empty()) {
            ALOG_DEBUG(MOD, "[tid=%u] INTER_KF task query_cloud empty query_id=%d kf_idx=%d", tid, task.query->id, task.query_kf_idx);
            return;
        }
        int inter_kf_tried = 0;
        int inter_kf_teaser_fail = 0;
        int inter_kf_reject_inconsistent = 0;
        int inter_kf_reject_pose_anomaly = 0;
        int inter_kf_published = 0;
        int inter_kf_skip_empty_target_cloud = 0;
        int inter_kf_skip_keyframe_gap = 0;
        int inter_kf_skip_odom_rel_rot_prefilter = 0;
        IcpRefiner icp;
        // 几何诊断：query 关键帧位姿（世界系）
        const KeyFrame::Ptr query_kf = (task.query_kf_idx >= 0 && task.query_kf_idx < static_cast<int>(task.query->keyframes.size()))
            ? task.query->keyframes[task.query_kf_idx] : nullptr;
        const Pose3d T_w_query = query_kf ? query_kf->T_odom_b : Pose3d::Identity();
        const Eigen::Vector3d pos_w_query = T_w_query.translation();

        for (const auto& kfc : task.candidates_kf) {
            SubMap::Ptr target;
            CloudXYZIPtr target_cloud;
            {
                std::shared_lock<std::shared_mutex> lk(db_mutex_);
                for (const auto& sm : db_submaps_) {
                    if (sm->id == kfc.submap_id) {
                        target = sm;
                        if (kfc.keyframe_idx >= 0 && kfc.keyframe_idx < static_cast<int>(sm->keyframe_clouds_ds.size()) && sm->keyframe_clouds_ds[kfc.keyframe_idx])
                            target_cloud = sm->keyframe_clouds_ds[kfc.keyframe_idx];
                        break;
                    }
                }
            }
            if (!target_cloud || target_cloud->empty()) {
                inter_kf_skip_empty_target_cloud++;
                ALOG_DEBUG(MOD, "[INTER_KF][VERIFY] skip_empty_target_cloud sm_i={} kf_i={} sm_j={} kf_j={}",
                           kfc.submap_id, kfc.keyframe_idx, task.query->id, task.query_kf_idx);
                continue;
            }

            inter_kf_tried++;
            // ── 子图间几何诊断：两关键帧在世界系下的距离与相对位姿（精准定位几何一致性差）──
            const KeyFrame::Ptr target_kf = (target && kfc.keyframe_idx >= 0 && kfc.keyframe_idx < static_cast<int>(target->keyframes.size()))
                ? target->keyframes[kfc.keyframe_idx] : nullptr;
            // 子图间回环：两关键帧对之间最小全局关键帧间隔（可配置，避免相邻帧假回环）
            if (inter_submap_min_keyframe_gap_ > 0 && target_kf && query_kf) {
                const int64_t gap = std::abs(static_cast<int64_t>(target_kf->id) - static_cast<int64_t>(query_kf->id));
                if (gap < static_cast<int64_t>(inter_submap_min_keyframe_gap_)) {
                    inter_kf_skip_keyframe_gap++;
                    ALOG_INFO(MOD, "[INTER_KF][FILTER] KEYFRAME_GAP: sm_i={} kf_i={} sm_j={} kf_j={} gap={} < {} (SKIP)",
                        kfc.submap_id, target_kf->id, task.query->id, query_kf->id, static_cast<long>(gap), inter_submap_min_keyframe_gap_);
                    continue;
                }
            }
            const Pose3d T_w_target = target_kf ? target_kf->T_odom_b : Pose3d::Identity();
            const Eigen::Vector3d pos_w_target = T_w_target.translation();
            const double dist_world_m = (pos_w_query - pos_w_target).norm();
            const Pose3d T_tgt_src_odom = T_w_target.inverse() * T_w_query;  // target 系下 query 位姿
            const double rel_trans_m = T_tgt_src_odom.translation().norm();
            const double rel_rot_deg = Eigen::AngleAxisd(T_tgt_src_odom.linear()).angle() * 180.0 / M_PI;
            ALOG_INFO(MOD,
                "[INTER_KF][GEOM_DIAG] sm_j={} kf_j={} sm_i={} kf_i={} | dist_world_m={:.3f} rel_trans_m={:.3f} rel_rot_deg={:.2f} | query_pts={} tgt_pts={} (同场景轨迹接得近时 dist_world 应小；过大则 ScanContext 可能匹配到错误关键帧)",
                task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                dist_world_m, rel_trans_m, rel_rot_deg, query_cloud->size(), target_cloud->size());
            if (node()) {
                RCLCPP_INFO(node()->get_logger(),
                    "[INTER_KF][GEOM_DIAG] sm_j=%d kf_j=%d sm_i=%d kf_i=%d dist_world_m=%.3f rel_trans_m=%.3f rel_rot_deg=%.2f query_pts=%zu tgt_pts=%zu",
                    task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                    dist_world_m, rel_trans_m, rel_rot_deg, query_cloud->size(), target_cloud->size());
            }

            // Odom 相对旋转过大时多为描述子误配（典型 ~180° + 近距），FPFH 互匹配 p90 飙高 → fpfh_garbage_input。
            // 在 TEASER/FPFH 前剔除，节省算力并降低无效日志；阈值与 pose_consistency 同阶（≥90° 且 ≥4×配置）。
            {
                const double max_rot_cfg = pose_consistency_max_rot_deg_;
                if (max_rot_cfg > 0.0) {
                    const double rot_cap_deg = std::max(90.0, 4.0 * max_rot_cfg);
                    if (rel_rot_deg > rot_cap_deg) {
                        inter_kf_skip_odom_rel_rot_prefilter++;
                        ALOG_INFO(MOD,
                            "[INTER_KF][FILTER] odom_rel_rot_skip sm_j={} kf_j={} sm_i={} kf_i={} rel_rot_deg={:.2f} cap={:.1f} (skip TEASER)",
                            task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                            rel_rot_deg, rot_cap_deg);
                        if (node()) {
                            RCLCPP_INFO(node()->get_logger(),
                                "[INTER_KF][FILTER] odom_rel_rot_skip sm_j=%d kf_j=%d sm_i=%d kf_i=%d rel_rot_deg=%.2f cap=%.1f",
                                task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                                rel_rot_deg, rot_cap_deg);
                        }
                        continue;
                    }
                }
            }

            CloudXYZIPtr tgt_copy = std::make_shared<CloudXYZI>();
            *tgt_copy = *target_cloud;
            TeaserMatcher::Result res;
            try {
                res = teaser_matcher_.match(query_cloud, tgt_copy, Pose3d::Identity());
            } catch (const std::exception& e) {
                inter_kf_teaser_fail++;
                ALOG_WARN(MOD, "[INTER_KF][TEASER][EXCEPTION] std::exception what={} sm_j={} kf_j={} sm_i={} kf_i={}",
                          e.what(), task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx);
                if (node()) {
                    RCLCPP_WARN(node()->get_logger(),
                        "[INTER_KF][TEASER][EXCEPTION] what=%s query_sm=%d q_kf_idx=%d tgt_sm=%d tgt_kf_idx=%d",
                        e.what(), task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx);
                }
                continue;
            } catch (...) {
                inter_kf_teaser_fail++;
                ALOG_WARN(MOD, "[INTER_KF][TEASER][EXCEPTION] unknown sm_j={} kf_j={} sm_i={} kf_i={}",
                          task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx);
                if (node()) {
                    RCLCPP_WARN(node()->get_logger(),
                        "[INTER_KF][TEASER][EXCEPTION] unknown query_sm=%d q_kf_idx=%d tgt_sm=%d tgt_kf_idx=%d",
                        task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx);
                }
                continue;
            }
            if (res.used_teaser) {
                loop_teaser_geom_total_.fetch_add(1, std::memory_order_relaxed);
            } else {
                loop_svd_geom_total_.fetch_add(1, std::memory_order_relaxed);
                const bool allow_svd_now =
                    shouldAllowSvdFallbackNow(res, task.query ? task.query->id : -1, kfc.submap_id, "inter_kf");
                if (!allow_svd_now) {
                    loop_fallback_reject_total_.fetch_add(1, std::memory_order_relaxed);
                    ALOG_WARN(MOD, "[INTER_KF][REJECT] sm_i={} sm_j={} geom_path=SVD_FALLBACK reason=svd_fallback_disabled",
                              kfc.submap_id, task.query->id);
                    inter_kf_teaser_fail++;
                    continue;
                }
            }

            // 几何诊断：TEASER 估计的相对位姿 vs 里程计相对位姿（若差异大则 FPFH/对应点或误匹配）
            const double teaser_trans_m = res.T_tgt_src.translation().norm();
            const double teaser_rot_deg = Eigen::AngleAxisd(res.T_tgt_src.linear()).angle() * 180.0 / M_PI;
            const double trans_diff_m = (res.T_tgt_src.translation() - T_tgt_src_odom.translation()).norm();
            const int valid = (res.success && res.inlier_ratio >= min_inlier_ratio_ && res.rmse <= max_rmse_) ? 1 : 0;
            ALOG_INFO(MOD,
                "[INTER_KF][GEOM_DIAG] after TEASER sm_j={} kf_j={} sm_i={} kf_i={} | odom: rel_trans={:.3f}m rel_rot={:.2f}deg | teaser: trans={:.3f}m rot={:.2f}deg | trans_diff_odom_teaser_m={:.3f} inlier_ratio={:.4f} valid={}",
                task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                rel_trans_m, rel_rot_deg, teaser_trans_m, teaser_rot_deg, trans_diff_m, res.inlier_ratio, valid);
            if (node()) {
                RCLCPP_INFO(node()->get_logger(),
                    "[INTER_KF][GEOM_DIAG] after TEASER sm_j=%d kf_j=%d sm_i=%d kf_i=%d odom_rel_trans=%.3f teaser_trans=%.3f trans_diff_m=%.3f inlier_ratio=%.4f",
                    task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx, rel_trans_m, teaser_trans_m, trans_diff_m, res.inlier_ratio);
            }
            if (!res.success || res.inlier_ratio < min_inlier_ratio_ || res.rmse > max_rmse_) {
                inter_kf_teaser_fail++;
                // 逐候选量级大：默认 DEBUG；需要时提高日志级别或 grep INTER_KF SUMMARY
                ALOG_DEBUG(MOD,
                    "[INTER_KF][TEASER][REJECT] sm_j={} kf_j={} sm_i={} kf_i={} success={} inlier={:.4f} rmse={:.4f} "
                    "need_inlier>={:.3f} need_rmse<={:.3f} geom_path={}",
                    task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                    res.success ? 1 : 0, res.inlier_ratio, res.rmse, min_inlier_ratio_, max_rmse_,
                    static_cast<int>(res.geom_path));
                if (node()) {
                    RCLCPP_DEBUG(node()->get_logger(),
                        "[INTER_KF][TEASER][REJECT] query_sm=%d q_kf=%d tgt_sm=%d tgt_kf_i=%d success=%d inlier=%.4f rmse=%.4f",
                        task.query->id, task.query_kf_idx, kfc.submap_id, kfc.keyframe_idx,
                        res.success ? 1 : 0, res.inlier_ratio, res.rmse);
                }
                continue;
            }

            // 子图间几何一致性：odom 相距很远但 TEASER 平移接近 0 → 假阳性，不发布（避免后端 trivial 过滤前先污染日志）
            constexpr double kInterOdomFarThresh = 5.0;   // odom 距离 > 5m 视为“非同一位置”
            constexpr double kInterTeaserNearThresh = 1.0; // TEASER 平移 < 1m 视为“接近单位阵”
            if (dist_world_m > kInterOdomFarThresh && teaser_trans_m < kInterTeaserNearThresh) {
                inter_kf_reject_inconsistent++;
                ALOG_WARN(MOD, "[INTER_KF][REJECT] odom_teaser_inconsistent sm_i={} kf_i={} sm_j={} kf_j={} dist_world={:.1f}m teaser_trans={:.3f}m (false positive, skip publish)",
                          kfc.submap_id, kfc.keyframe_idx, task.query->id, task.query_kf_idx, dist_world_m, teaser_trans_m);
                if (node()) {
                    RCLCPP_WARN(node()->get_logger(),
                        "[INTER_KF][REJECT] odom_teaser_inconsistent sm_i=%d sm_j=%d dist_world=%.1fm teaser_trans=%.3fm (skip)",
                        kfc.submap_id, task.query->id, dist_world_m, teaser_trans_m);
                }
                continue;
            }

            // 回环位姿一致性：TEASER 与 odom 相对位姿差异超过阈值视为异常（回环应是微调）。trans_diff/rot_diff 已涵盖反向运动（约180°方向差+平移偏移）：平移用向量差模、旋转用相对旋转角 [0,180]°。
            const double max_trans_diff = pose_consistency_max_trans_m_;
            const double max_rot_diff_deg_cfg = pose_consistency_max_rot_deg_;
            if (max_trans_diff > 0.0 || max_rot_diff_deg_cfg > 0.0) {
                const Eigen::Matrix3d R_teaser = res.T_tgt_src.linear();
                const Eigen::Matrix3d R_odom = T_tgt_src_odom.linear();
                const Eigen::Matrix3d R_diff = R_teaser * R_odom.transpose();
                const double trace_r = R_diff.trace();
                const double angle_rad = std::acos(std::max(-1.0, std::min(1.0, (trace_r - 1.0) * 0.5)));
                const double rot_diff_deg = angle_rad * 180.0 / M_PI;
                const bool trans_anomaly = (max_trans_diff > 0.0 && trans_diff_m > max_trans_diff);
                const bool rot_anomaly = (max_rot_diff_deg_cfg > 0.0 && rot_diff_deg > max_rot_diff_deg_cfg);
                if (trans_anomaly || rot_anomaly) {
                    inter_kf_reject_pose_anomaly++;
                    const Eigen::Vector3d t_odom = T_tgt_src_odom.translation();
                    const Eigen::Vector3d t_teaser = res.T_tgt_src.translation();
                    const Eigen::Vector3d t_diff = t_teaser - t_odom;
                    const double odom_rot_deg = Eigen::AngleAxisd(T_tgt_src_odom.linear()).angle() * 180.0 / M_PI;
                    const double teaser_rot_deg = Eigen::AngleAxisd(res.T_tgt_src.linear()).angle() * 180.0 / M_PI;
                    const KeyFrame::Ptr target_kf_log = (target && kfc.keyframe_idx >= 0 && kfc.keyframe_idx < static_cast<int>(target->keyframes.size()))
                        ? target->keyframes[kfc.keyframe_idx] : nullptr;
                    const int kf_id_target = target_kf_log ? target_kf_log->id : -1;
                    const int kf_id_query = query_kf ? query_kf->id : -1;
                    const double ts_target = target_kf_log ? target_kf_log->timestamp : 0.0;
                    const double ts_query = query_kf ? query_kf->timestamp : 0.0;
                    
                    // 增加根本原因猜测日志
                    std::string root_cause = "unknown";
                    if (rot_diff_deg > 150.0) root_cause = "possibly_matched_backwards (180deg flip)";
                    else if (trans_diff_m > 30.0) root_cause = "massive_odometry_drift_or_wrong_place";
                    else if (trans_anomaly && !rot_anomaly) root_cause = "translation_only_mismatch (check voxel/feature)";
                    else if (!trans_anomaly && rot_anomaly) root_cause = "rotation_only_mismatch (check feature ambiguity)";

                    ALOG_WARN(MOD,
                        "[INTER_KF][REJECT] pose_anomaly: sm_i={} kf_i={} sm_j={} kf_j={} kf_id_tgt={} kf_id_query={} ts_tgt={:.3f} ts_query={:.3f} cause={}",
                        kfc.submap_id, kfc.keyframe_idx, task.query->id, task.query_kf_idx, kf_id_target, kf_id_query, ts_target, ts_query, root_cause);
                    
                    // [GHOSTING_DIAG] 如果几何匹配很好但被位姿一致性检查拒绝，记录为潜在的“被错杀的真实回环”或“隐蔽的误匹配”
                    if (res.inlier_ratio > 0.15 && res.rmse < 0.3) {
                        RCLCPP_ERROR(node()->get_logger(),
                            "[INTER_KF][POSE_CONFLICT] STRONG geometric match REJECTED by pose consistency: "
                            "sm_i=%d kf_i=%d sm_j=%d kf_j=%d inlier=%.3f rmse=%.3f trans_diff=%.2fm rot_diff=%.2fdeg. "
                            "If ghosting persists, consider if this loop was actually correct!",
                            kfc.submap_id, kfc.keyframe_idx, task.query->id, task.query_kf_idx, res.inlier_ratio, res.rmse, trans_diff_m, rot_diff_deg);
                    }

                    ALOG_WARN(MOD,
                        "[INTER_KF][REJECT] pose_anomaly ODOM_rel:  trans_xyz=[{:.4f},{:.4f},{:.4f}] trans_norm={:.4f}m rot_deg={:.2f}",
                        t_odom.x(), t_odom.y(), t_odom.z(), t_odom.norm(), odom_rot_deg);
                    ALOG_WARN(MOD,
                        "[INTER_KF][REJECT] pose_anomaly TEASER_rel: trans_xyz=[{:.4f},{:.4f},{:.4f}] trans_norm={:.4f}m rot_deg={:.2f} inlier={:.4f} rmse={:.4f}",
                        t_teaser.x(), t_teaser.y(), t_teaser.z(), t_teaser.norm(), teaser_rot_deg, res.inlier_ratio, res.rmse);
                    ALOG_WARN(MOD,
                        "[INTER_KF][REJECT] pose_anomaly DIFF: trans_diff_xyz=[{:.4f},{:.4f},{:.4f}] trans_diff_norm={:.4f}m rot_diff_deg={:.2f} | thresholds trans={:.1f}m rot={:.1f}deg (高精建图不应出现大变化，可能为计算/逻辑错误)",
                        t_diff.x(), t_diff.y(), t_diff.z(), trans_diff_m, rot_diff_deg, max_trans_diff, max_rot_diff_deg_cfg);
                    ALOG_WARN(MOD,
                        "[INTER_KF][REJECT] pose_anomaly WORLD: dist_world={:.3f}m pos_tgt=[{:.3f},{:.3f},{:.3f}] pos_query=[{:.3f},{:.3f},{:.3f}]",
                        dist_world_m,
                        pos_w_target.x(), pos_w_target.y(), pos_w_target.z(),
                        pos_w_query.x(), pos_w_query.y(), pos_w_query.z());
                    if (node()) {
                        RCLCPP_WARN(node()->get_logger(),
                            "[INTER_KF][REJECT] pose_anomaly sm_i=%d kf_i=%d sm_j=%d kf_j=%d kf_id_tgt=%d kf_id_query=%d trans_diff=%.3fm rot_diff=%.2fdeg cause=%s (详见 ODOM_rel/TEASER_rel/DIFF/WORLD；grep INTER_KF REJECT)",
                            kfc.submap_id, kfc.keyframe_idx, task.query->id, task.query_kf_idx, kf_id_target, kf_id_query, trans_diff_m, rot_diff_deg, root_cause.c_str());
                    }
                    continue;
                }
            }

            auto lc = std::make_shared<LoopConstraint>();
            lc->submap_i = kfc.submap_id;
            lc->submap_j = task.query->id;
            lc->keyframe_i = kfc.keyframe_idx;
            lc->keyframe_j = task.query_kf_idx;
            lc->keyframe_global_id_i = target_kf ? static_cast<int>(target_kf->id) : -1;
            lc->keyframe_global_id_j = query_kf ? static_cast<int>(query_kf->id) : -1;
            lc->session_i = target ? target->session_id : 0;
            lc->session_j = task.query->session_id;
            lc->delta_T = res.T_tgt_src;
            lc->overlap_score = kfc.score;
            lc->inlier_ratio = res.inlier_ratio;
            lc->rmse = static_cast<float>(res.rmse);
            // 修复: 使用更合理的信息矩阵计算，避免过度自信
            // 之前的计算方式 lc->inlier_ratio / (lc->rmse + 1e-3f) 在 RMSE 极小时会导致权重过大
            // 改进：增加 RMSE 偏置 (0.01m) 并调整旋转/平移比例，使之更均衡
            double info_scale = 100.0 * lc->inlier_ratio / (lc->rmse + 0.01);
            // 限制最大信息尺度，避免数值过大导致优化不稳定（1e4 对应约 1cm 精度）
            info_scale = std::min(info_scale, 2e4);
            Mat66d information = Mat66d::Identity();
            information.block<3,3>(0,0) *= info_scale * 0.5;  // 适度增加平移权重 (0.1 -> 0.5)
            information.block<3,3>(3,3) *= info_scale;        // 旋转权重保持比例
            lc->information = information;

            // [GHOSTING_DIAG] 记录子图间回环信息矩阵强度
            RCLCPP_INFO(node()->get_logger(), 
                "[INTER_KF][INFO_DIAG] sm_%d->%d kf_%d->%d info_scale=%.2e trans_w=%.2e rot_w=%.2e",
                lc->submap_i, lc->submap_j, lc->keyframe_i, lc->keyframe_j, info_scale, lc->information(0,0), lc->information(3,3));
            lc->status = LoopStatus::ACCEPTED;
            ALOG_INFO(MOD, "[INTER_KF][LOOP_ACCEPTED] sm_i={} kf_i={} sm_j={} kf_j={} score={:.3f} inlier={:.3f} rmse={:.4f}",
                      lc->submap_i, lc->keyframe_i, lc->submap_j, lc->keyframe_j, lc->overlap_score, lc->inlier_ratio, lc->rmse);
            publishLoopConstraint(lc);
            loop_detected_count_++;
            inter_kf_published++;
            for (auto& cb : loop_cbs_) { try { cb(lc); } catch (...) {} }
        }
        ALOG_INFO(MOD,
            "[INTER_KF][SUMMARY] query_id={} kf_j={} candidates_kf={} tried={} skip_empty_tgt={} skip_kf_gap={} "
            "skip_odom_rot_prefilter={} teaser_fail={} reject_inconsistent={} reject_pose_anomaly={} published={} "
            "(grep INTER_KF SUMMARY 闭环统计)",
            task.query->id, task.query_kf_idx, static_cast<int>(task.candidates_kf.size()),
            inter_kf_tried, inter_kf_skip_empty_target_cloud, inter_kf_skip_keyframe_gap,
            inter_kf_skip_odom_rel_rot_prefilter, inter_kf_teaser_fail, inter_kf_reject_inconsistent,
            inter_kf_reject_pose_anomaly, inter_kf_published);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(),
                "[INTER_KF][SUMMARY] query_id=%d kf_j=%d candidates_kf=%zu tried=%d skip_empty_tgt=%d skip_kf_gap=%d "
                "skip_odom_rot_prefilter=%d teaser_fail=%d reject_inconsistent=%d reject_pose_anomaly=%d published=%d",
                task.query->id, task.query_kf_idx, task.candidates_kf.size(),
                inter_kf_tried, inter_kf_skip_empty_target_cloud, inter_kf_skip_keyframe_gap,
                inter_kf_skip_odom_rel_rot_prefilter, inter_kf_teaser_fail, inter_kf_reject_inconsistent,
                inter_kf_reject_pose_anomaly, inter_kf_published);
        }
        return;
    }

    if (task.candidates.empty()) {
        ALOG_DEBUG(MOD, "[tid=%u] processMatchTask: no candidates for SM#%d, skipping",
                   tid, task.query->id);
        return;
    }

    IcpRefiner icp;
    const auto& query = task.query;
    // 优先使用入队时拷贝的 query_cloud，避免读 SubMap::downsampled_cloud 与主线程并发
    CloudXYZIPtr query_cloud = task.query_cloud;
    if (!query_cloud || query_cloud->empty()) {
        CloudXYZIPtr query_ref = query ? query->downsampled_cloud : nullptr;
        if (!query_ref || query_ref->empty()) {
            ALOG_DEBUG(MOD, "[tid=%u] step=task_skip query_cloud_empty query_id=%d", tid, query ? query->id : -1);
            return;
        }
        query_cloud = std::make_shared<CloudXYZI>();
        *query_cloud = *query_ref;
        ALOG_DEBUG(MOD, "[tid=%u] step=task_fallback_copy query_id=%d query_pts=%zu", tid, query->id, query_cloud->size());
    }

    const int min_safe_inliers_cfg = teaser_min_safe_inliers_;
    ALOG_INFO(MOD, "[LOOP_STEP] stage=geom_verify_enter query_id={} query_pts={} candidates={} thresholds: overlap>={:.3f} min_inlier_ratio>={:.3f} max_rmse<={:.3f}m min_safe_inliers>={} (TEASER 拒绝条件，grep LOOP_STEP 精准分析)",
              query->id, query_cloud->size(), task.candidates.size(),
              overlap_threshold_, min_inlier_ratio_, max_rmse_, min_safe_inliers_cfg);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=geom_verify_enter query_id=%d query_pts=%zu candidates=%zu overlap_thresh=%.3f min_inlier_ratio=%.3f max_rmse=%.3f min_safe_inliers=%d",
                    query->id, query_cloud->size(), task.candidates.size(),
                    overlap_threshold_, min_inlier_ratio_, max_rmse_, min_safe_inliers_cfg);
    }
    AUTOMAP_TIMED_SCOPE(MOD, fmt::format("GeomVerify SM#{}", task.query->id), 3000.0);

    // 过滤掉无效候选（score 与 submap_id 有效性）
    std::vector<OverlapTransformerInfer::Candidate> valid_candidates;
    int dropped_low_score = 0, dropped_invalid_id = 0;
    for (const auto& cand : task.candidates) {
        if (cand.score < overlap_threshold_) {
            dropped_low_score++;
            continue;
        }
        if (cand.submap_id < 0) {
            dropped_invalid_id++;
            continue;
        }
        valid_candidates.push_back(cand);
    }
    ALOG_INFO(MOD, "[LOOP_STEP] stage=geom_verify_score_filter query_id={} input={} dropped_low_score={} dropped_invalid_id={} valid={} (threshold={:.3f})",
              query->id, task.candidates.size(), dropped_low_score, dropped_invalid_id, valid_candidates.size(), overlap_threshold_);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[LOOP_STEP] stage=geom_verify_score_filter query_id=%d input=%zu dropped_low_score=%d dropped_invalid_id=%d valid=%zu",
                    query->id, task.candidates.size(), dropped_low_score, dropped_invalid_id, valid_candidates.size());
    }
    for (size_t i = 0; i < valid_candidates.size(); ++i) {
        ALOG_INFO(MOD, "[LOOP_STEP] geom_verify_cand[{}] query_id={} target_id={} score={:.3f}",
                  i, query->id, valid_candidates[i].submap_id, valid_candidates[i].score);
    }

    if (valid_candidates.empty()) {
        ALOG_WARN(MOD, "[LOOP_STEP] stage=geom_verify_score_filter NONE_PASSED query_id={} all {} candidates below overlap_threshold={:.3f}",
                  query->id, task.candidates.size(), overlap_threshold_);
        if (node()) {
            RCLCPP_WARN(node()->get_logger(), "[LOOP_STEP] stage=geom_verify_score_filter NONE_PASSED query_id=%d candidates=%zu overlap_threshold=%.3f",
                        query->id, task.candidates.size(), overlap_threshold_);
        }
        return;
    }

    int count_reject_success_or_inlier = 0;
    int count_reject_rmse = 0;
    int count_accepted = 0;
    int count_exception = 0;

    // P1: 并行 TEASER 多候选匹配（performance.parallel_teaser_match=true）
    if (parallel_teaser_match_ && !valid_candidates.empty()) {
        struct CandWork {
            OverlapTransformerInfer::Candidate cand;
            SubMap::Ptr target;
            CloudXYZIPtr target_cloud;
        };
        std::vector<CandWork> works;
        {
            std::shared_lock<std::shared_mutex> lk(db_mutex_);
            for (const auto& cand : valid_candidates) {
                SubMap::Ptr target;
                CloudXYZIPtr target_ref;
                for (const auto& sm : db_submaps_) {
                    if (sm->id == cand.submap_id && sm->session_id == cand.session_id) {
                        target = sm;
                        target_ref = sm->downsampled_cloud;
                        break;
                    }
                }
                if (!target_ref || target_ref->empty()) continue;
                CloudXYZIPtr target_cloud = std::make_shared<CloudXYZI>();
                *target_cloud = *target_ref;
                works.push_back({cand, target, target_cloud});
            }
        }
        if (!works.empty()) {
            const size_t max_inflight = static_cast<size_t>(std::max(1, parallel_teaser_max_inflight_));
            IcpRefiner icp_par;
            for (size_t batch_begin = 0; batch_begin < works.size(); batch_begin += max_inflight) {
                const size_t batch_end = std::min(works.size(), batch_begin + max_inflight);
                std::vector<size_t> work_indices;
                std::vector<std::future<TeaserMatcher::Result>> futures;
                work_indices.reserve(batch_end - batch_begin);
                futures.reserve(batch_end - batch_begin);
                for (size_t wi = batch_begin; wi < batch_end; ++wi) {
                    CloudXYZIPtr q = query_cloud;
                    CloudXYZIPtr t = works[wi].target_cloud;
                    const auto inflight_now = loop_teaser_async_inflight_.fetch_add(1, std::memory_order_relaxed) + 1;
                    uint64_t old_peak = loop_teaser_async_inflight_max_.load(std::memory_order_relaxed);
                    while (inflight_now > old_peak &&
                           !loop_teaser_async_inflight_max_.compare_exchange_weak(
                               old_peak, inflight_now, std::memory_order_relaxed)) {
                    }
                    work_indices.push_back(wi);
                    futures.push_back(std::async(std::launch::async, [this, q, t]() {
                        const auto t0 = std::chrono::steady_clock::now();
                        try {
                            auto res = teaser_matcher_.match(q, t, Pose3d::Identity());
                            const auto cost_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                                                     std::chrono::steady_clock::now() - t0)
                                                     .count();
                            {
                                std::lock_guard<std::mutex> lk(loop_metrics_mutex_);
                                teaser_latency_samples_ms_.push_back(cost_ms);
                                while (static_cast<int>(teaser_latency_samples_ms_.size()) > teaser_latency_window_size_) {
                                    teaser_latency_samples_ms_.pop_front();
                                }
                            }
                            loop_teaser_async_inflight_.fetch_sub(1, std::memory_order_relaxed);
                            return res;
                        } catch (...) {
                            loop_teaser_async_inflight_.fetch_sub(1, std::memory_order_relaxed);
                            throw;
                        }
                    }));
                }
                for (size_t fi = 0; fi < futures.size(); ++fi) {
                    const size_t i = work_indices[fi];
                TeaserMatcher::Result teaser_res;
                try {
                    teaser_res = futures[fi].get();
                } catch (const std::exception& e) {
                    ALOG_ERROR(MOD, "[tid={}] parallel_teaser candidate {} exception: {}", tid, works[i].cand.submap_id, e.what());
                    ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=exception reason=teaser_exception what={} (parallel)",
                              query->id, works[i].cand.submap_id, e.what());
                    count_exception++;
                    if (node()) {
                        RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=exception what=%s",
                                    query->id, works[i].cand.submap_id, e.what());
                    }
                    continue;
                } catch (...) {
                    ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=exception reason=teaser_unknown (parallel)",
                              query->id, works[i].cand.submap_id);
                    count_exception++;
                    if (node()) {
                        RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=exception_unknown",
                                    query->id, works[i].cand.submap_id);
                    }
                    continue;
                }
                // 修复: 确保num_correspondences非负，避免负值导致的问题
                int inliers_approx = static_cast<int>(std::round(teaser_res.inlier_ratio * std::max(0, std::max(0, teaser_res.num_correspondences))));
                const char* reason_str = (teaser_res.success && teaser_res.inlier_ratio >= min_inlier_ratio_) ? "ok" : "teaser_fail_or_inlier_low";
                ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=done success={} inliers≈{} corrs={} inlier_ratio={:.3f} rmse={:.4f} reason={} (parallel)",
                          query->id, works[i].cand.submap_id, teaser_res.success, inliers_approx, teaser_res.num_correspondences,
                          teaser_res.inlier_ratio, teaser_res.rmse, reason_str);
                if (node()) {
                    RCLCPP_INFO(node()->get_logger(), "[LOOP_COMPUTE] query_id=%d target_id=%d success=%d inliers=%d corrs=%d inlier_ratio=%.3f reason=%s (parallel)",
                                query->id, works[i].cand.submap_id, teaser_res.success ? 1 : 0, inliers_approx, teaser_res.num_correspondences, teaser_res.inlier_ratio, reason_str);
                }
                if (teaser_res.used_teaser) {
                    loop_teaser_geom_total_.fetch_add(1, std::memory_order_relaxed);
                } else {
                    loop_svd_geom_total_.fetch_add(1, std::memory_order_relaxed);
                    const bool allow_svd_now =
                        shouldAllowSvdFallbackNow(teaser_res, query ? query->id : -1, works[i].cand.submap_id, "parallel");
                    if (!allow_svd_now) {
                        loop_fallback_reject_total_.fetch_add(1, std::memory_order_relaxed);
                        ALOG_WARN(MOD, "[LOOP_FLOW] query_id={} target_id={} geom_path=SVD_FALLBACK reject=1 reason=svd_fallback_disabled (parallel)",
                                  query->id, works[i].cand.submap_id);
                        continue;
                    }
                }
                if (!teaser_res.success || teaser_res.inlier_ratio < min_inlier_ratio_) {
                    count_reject_success_or_inlier++;
                    const char* primary_reason = !teaser_res.success ? "success_false" : "inlier_ratio_below_threshold";
                    ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} primary_reason={} success={} inlier_ratio={:.4f} min_ratio={:.3f} rmse={:.4f} max_rmse={:.3f} corrs={} (parallel)",
                              query->id, works[i].cand.submap_id, primary_reason, teaser_res.success, teaser_res.inlier_ratio, min_inlier_ratio_, teaser_res.rmse, max_rmse_, teaser_res.num_correspondences);
                    if (node()) {
                        RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=%s success=%d inlier_ratio=%.4f min_inlier_ratio=%.3f rmse=%.4f max_rmse=%.3f corrs=%d (分析: TEASER 返回 success=0 多为 insufficient_pts/corrs/solution_invalid；inlier_ratio 低可调 teaser.min_inlier_ratio 或改进重叠)",
                                    query->id, works[i].cand.submap_id, primary_reason, teaser_res.success ? 1 : 0, teaser_res.inlier_ratio, min_inlier_ratio_, teaser_res.rmse, max_rmse_, teaser_res.num_correspondences);
                    }
                    continue;
                }
                const auto& cand = works[i].cand;
                SubMap::Ptr target = works[i].target;
                CloudXYZIPtr target_cloud = works[i].target_cloud;
                Pose3d final_T = teaser_res.T_tgt_src;
                double final_rmse = teaser_res.rmse;
                if (use_icp_refine_) {
                    auto icp_res = icp_par.refine(query_cloud, target_cloud, final_T);
                    if (icp_res.converged && icp_res.rmse < final_rmse) {
                        final_T = icp_res.T_refined;
                        final_rmse = icp_res.rmse;
                    }
                }
                if (final_rmse > max_rmse_) {
                    count_reject_rmse++;
                    ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} primary_reason=rmse_too_high rmse={:.4f} max_rmse={:.4f} inlier_ratio={:.3f} (parallel)",
                              query->id, target->id, final_rmse, max_rmse_, teaser_res.inlier_ratio);
                    if (node()) {
                        RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=rmse_too_high rmse=%.4f max_rmse=%.4f (分析: 几何残差过大，可适当放宽 loop_closure.teaser.max_rmse_m 或检查点云重叠)",
                                    query->id, target->id, final_rmse, max_rmse_);
                    }
                    continue;
                }
                count_accepted++;
                auto lc = std::make_shared<LoopConstraint>();
                lc->submap_i = target->id;
                lc->submap_j = query->id;
                lc->session_i = target->session_id;
                lc->session_j = query->session_id;
                lc->delta_T = final_T;
                lc->overlap_score = cand.score;
                lc->inlier_ratio = teaser_res.inlier_ratio;
                lc->rmse = static_cast<float>(final_rmse);
                lc->is_inter_session = (query->session_id != target->session_id);
                lc->status = LoopStatus::ACCEPTED;
                // 修复: 使用更合理的信息矩阵计算，区分旋转和平移自由度
                double info_scale = 100.0 * lc->inlier_ratio / (lc->rmse + 0.01);
                // 限制最大信息尺度，避免数值过大导致优化不稳定
                info_scale = std::min(info_scale, 2e4);
                Mat66d information = Mat66d::Identity();
                information.block<3,3>(0,0) *= info_scale * 0.5;  // 适度增加平移权重 (0.1 -> 0.5)
                information.block<3,3>(3,3) *= info_scale;        // 旋转权重保持比例
                lc->information = information;
                ALOG_INFO(MOD, "[LoopDetector][LOOP_OK] 回环约束已发布(parallel) query_id={} target_id={} inlier={:.3f} rmse={:.4f}m",
                          query->id, target->id, lc->inlier_ratio, lc->rmse);
                if (node()) {
                    RCLCPP_INFO(node()->get_logger(), "[LOOP_ACCEPTED] query_id=%d target_id=%d inlier_ratio=%.3f rmse=%.4fm (grep LOOP_ACCEPTED to verify loop constraints)",
                                query->id, target->id, lc->inlier_ratio, lc->rmse);
                    RCLCPP_INFO(node()->get_logger(), "[LoopDetector] Loop %d↔%d | inlier=%.2f rmse=%.3f", lc->submap_i, lc->submap_j, lc->inlier_ratio, lc->rmse);
                }
                publishLoopConstraint(lc);
                loop_detected_count_++;
                for (auto& cb : loop_cbs_) cb(lc);
            }
            }
            ALOG_INFO(MOD, "[TEASER_SUMMARY] query_id={} total_candidates={} reject_success_or_inlier={} reject_rmse={} reject_exception={} accepted={} (若 accepted=0 则本 query 所有回环被 TEASER 拒绝，见上方 [TEASER_REJECT] primary_reason 或 grep LOOP_COMPUTE][TEASER] reason=)",
                      query->id, static_cast<int>(works.size()), count_reject_success_or_inlier, count_reject_rmse, count_exception, count_accepted);
            if (node()) {
                RCLCPP_INFO(node()->get_logger(), "[TEASER_SUMMARY] query_id=%d total=%zu reject_success_or_inlier=%d reject_rmse=%d reject_exception=%d accepted=%d (分析回环全被拒: grep TEASER_REJECT)",
                            query->id, works.size(), count_reject_success_or_inlier, count_reject_rmse, count_exception, count_accepted);
            }
            updateLoopHealthKpi(query->id, static_cast<int>(works.size()), count_accepted);
        } else {
            ALOG_WARN(MOD, "[LOOP_FLOW] query_id={} parallel_teaser=on but no valid work items (empty target clouds or stale candidates)",
                      query->id);
        }
        return;
    }

    count_reject_success_or_inlier = 0;
    count_reject_rmse = 0;
    count_exception = 0;
    count_accepted = 0;
    for (const auto& cand : valid_candidates) {
        ALOG_DEBUG(MOD, "  Checking candidate SM#%d score=%.3f", cand.submap_id, cand.score);
        SubMap::Ptr target;
        CloudXYZIPtr target_ref;  // 在持锁下取得强引用，避免读 submap 与主线程并发
        {
            std::shared_lock<std::shared_mutex> lk(db_mutex_);
            for (const auto& sm : db_submaps_) {
                if (sm->id == cand.submap_id && sm->session_id == cand.session_id) {
                    target = sm;
                    target_ref = sm->downsampled_cloud;  // 持锁时拷贝 shared_ptr，保证生命周期
                    break;
                }
            }
        }
        if (!target_ref || target_ref->empty()) {
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=target_cloud_empty score={:.3f}",
                      query->id, cand.submap_id, cand.score);
            continue;
        }

        // 深拷贝 target 点云，避免 PCL/TEASER 与主线程竞争
        CloudXYZIPtr target_cloud = std::make_shared<CloudXYZI>();
        *target_cloud = *target_ref;

        ALOG_DEBUG(MOD, "[tid={}] step=cand_geom query_id={} target_id={} target_pts={} score={:.3f}",
                   tid, query->id, target->id, target_cloud->size(), cand.score);
        ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} query_pts={} target_pts={} overlap_score={:.3f} (TEASER 计算开始，详见 LOOP_COMPUTE][TEASER])",
                  query->id, target->id, query_cloud->size(), target_cloud->size(), cand.score);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_COMPUTE] query_id=%d target_id=%d query_pts=%zu target_pts=%zu overlap_score=%.3f",
                        query->id, target->id, query_cloud->size(), target_cloud->size(), cand.score);
        }
        // Stage 2: TEASER++ 粗配准
        auto t0 = std::chrono::steady_clock::now();
        TeaserMatcher::Result teaser_res;
        try {
            ALOG_DEBUG(MOD, "[tid={}] step=match_invoke query_id={} target_id={}", tid, query->id, target->id);
            teaser_res = teaser_matcher_.match(
                query_cloud,
                target_cloud,
                Pose3d::Identity());
            ALOG_DEBUG(MOD, "[tid={}] step=match_returned query_id={} target_id={} success={}", 
                      tid, query->id, target->id, teaser_res.success);
        } catch (const std::exception& e) {
            count_exception++;
            ALOG_ERROR(MOD, "[tid={}] step=match_exception query_id={} target_id={} exception={}",
                      tid, query->id, target->id, e.what());
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=teaser_exception what={} score={:.3f}",
                      query->id, target->id, e.what(), cand.score);
            if (node()) {
                RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=exception what=%s",
                            query->id, target->id, e.what());
            }
            continue;
        } catch (...) {
            count_exception++;
            ALOG_ERROR(MOD, "[tid={}] step=match_exception query_id={} target_id={} unknown_exception",
                      tid, query->id, target->id);
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=teaser_unknown_exception score={:.3f}",
                      query->id, target->id, cand.score);
            if (node()) {
                RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=exception_unknown",
                            query->id, target->id);
            }
            continue;
        }
        double teaser_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        // 修复: 确保num_correspondences非负，避免负值导致的问题
        int inliers_approx = static_cast<int>(std::round(teaser_res.inlier_ratio * std::max(0, std::max(0, teaser_res.num_correspondences))));
        const char* reason_str = (teaser_res.success && teaser_res.inlier_ratio >= min_inlier_ratio_) ? "ok" : "teaser_fail_or_inlier_low";
        ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=done success={} inliers≈{} corrs={} inlier_ratio={:.3f} rmse={:.4f} reason={} ms={:.1f} (精准优化: 若 inliers<10 可调 safe_min; 若 ratio 低可调 FPFH/voxel)",
                  query->id, target->id, teaser_res.success, inliers_approx, teaser_res.num_correspondences,
                  teaser_res.inlier_ratio, teaser_res.rmse, reason_str, teaser_ms);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_COMPUTE] query_id=%d target_id=%d success=%d inliers=%d corrs=%d inlier_ratio=%.3f rmse=%.4f reason=%s",
                        query->id, target->id, teaser_res.success ? 1 : 0, inliers_approx, teaser_res.num_correspondences,
                        teaser_res.inlier_ratio, teaser_res.rmse, reason_str);
        }
        if (teaser_res.used_teaser) {
            loop_teaser_geom_total_.fetch_add(1, std::memory_order_relaxed);
        } else {
            loop_svd_geom_total_.fetch_add(1, std::memory_order_relaxed);
            const bool allow_svd_now =
                shouldAllowSvdFallbackNow(teaser_res, query ? query->id : -1, target ? target->id : -1, "serial");
            if (!allow_svd_now) {
                loop_fallback_reject_total_.fetch_add(1, std::memory_order_relaxed);
                ALOG_WARN(MOD, "[LOOP_FLOW] query_id={} target_id={} geom_path=SVD_FALLBACK reject=1 reason=svd_fallback_disabled",
                          query->id, target->id);
                continue;
            }
        }
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_result query_id={} target_id={} success={} inlier={:.2f} rmse={:.3f}m ms={:.1f}",
                   tid, query->id, target->id, teaser_res.success, teaser_res.inlier_ratio, teaser_res.rmse, teaser_ms);

        if (!teaser_res.success || teaser_res.inlier_ratio < min_inlier_ratio_) {
            count_reject_success_or_inlier++;
            const char* primary_reason = !teaser_res.success ? "success_false" : "inlier_ratio_below_threshold";
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} primary_reason={} success={} inlier_ratio={:.4f} min_ratio={:.3f} rmse={:.4f} max_rmse={:.3f} corrs={} score={:.3f}",
                      query->id, target->id, primary_reason, teaser_res.success, teaser_res.inlier_ratio, min_inlier_ratio_, teaser_res.rmse, max_rmse_, teaser_res.num_correspondences, cand.score);
            if (node()) {
                RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=%s success=%d inlier_ratio=%.4f min_inlier_ratio=%.3f rmse=%.4f max_rmse=%.3f corrs=%d (分析: success=0 见 [LOOP_COMPUTE][TEASER] reason=insufficient_pts/corrs/solution_invalid；inlier 低可调 teaser.min_inlier_ratio)",
                            query->id, target->id, primary_reason, teaser_res.success ? 1 : 0, teaser_res.inlier_ratio, min_inlier_ratio_, teaser_res.rmse, max_rmse_, teaser_res.num_correspondences);
            }
            continue;
        }

        // Stage 3: ICP 精配准（可选）
        Pose3d final_T = teaser_res.T_tgt_src;
        double final_rmse = teaser_res.rmse;

        if (use_icp_refine_) {
            auto icp_res = icp.refine(
                query_cloud, target_cloud, final_T);
            if (icp_res.converged && icp_res.rmse < final_rmse) {
                final_T    = icp_res.T_refined;
                final_rmse = icp_res.rmse;
            }
        }

        if (final_rmse > max_rmse_) {
            count_reject_rmse++;
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} primary_reason=rmse_too_high rmse={:.4f} max_rmse={:.4f} inlier_ratio={:.3f} score={:.3f}",
                      query->id, target->id, final_rmse, max_rmse_, teaser_res.inlier_ratio, cand.score);
            if (node()) {
                RCLCPP_INFO(node()->get_logger(), "[TEASER_REJECT] query_id=%d target_id=%d primary_reason=rmse_too_high rmse=%.4f max_rmse=%.4f (分析: 几何残差过大，可放宽 loop_closure.teaser.max_rmse_m)",
                            query->id, target->id, final_rmse, max_rmse_);
            }
            continue;
        }

        count_accepted++;
        // 构建回环约束
        auto lc = std::make_shared<LoopConstraint>();
        lc->submap_i      = target->id;
        lc->submap_j      = query->id;
        lc->session_i     = target->session_id;
        lc->session_j     = query->session_id;
        lc->delta_T       = final_T;
        lc->overlap_score = cand.score;
        lc->inlier_ratio  = teaser_res.inlier_ratio;
        lc->rmse          = static_cast<float>(final_rmse);
        lc->is_inter_session = (query->session_id != target->session_id);
        lc->status        = LoopStatus::ACCEPTED;

        // 信息矩阵：内点比率越高、RMSE越小 → 信息越大
        // 修复: 使用更合理的信息矩阵计算，区分旋转和平移自由度
        double info_scale = 100.0 * lc->inlier_ratio / (lc->rmse + 0.01);
        // 限制最大信息尺度，避免数值过大导致优化不稳定
        info_scale = std::min(info_scale, 2e4);
        Mat66d information = Mat66d::Identity();
        information.block<3,3>(0,0) *= info_scale * 0.5;  // 适度增加平移权重 (0.1 -> 0.5)
        information.block<3,3>(3,3) *= info_scale;        // 旋转权重保持比例
        lc->information = information;

        ALOG_INFO(MOD, "[LoopDetector][LOOP_OK] 回环约束已发布 query_id={} target_id={} inlier={:.3f} rmse={:.4f}m score={:.3f}",
                  query->id, target->id, lc->inlier_ratio, lc->rmse, lc->overlap_score);
        if (node()) {
            RCLCPP_INFO(node()->get_logger(), "[LOOP_ACCEPTED] query_id=%d target_id=%d inlier_ratio=%.3f rmse=%.4fm (grep LOOP_ACCEPTED to verify loop constraints)",
                        query->id, target->id, lc->inlier_ratio, lc->rmse);
            RCLCPP_INFO(node()->get_logger(),
                "[LoopDetector] Loop %d↔%d | inlier=%.2f rmse=%.3f score=%.2f",
                lc->submap_i, lc->submap_j, lc->inlier_ratio, lc->rmse, lc->overlap_score);
        }

        // 发布 ROS2 消息
        publishLoopConstraint(lc);

        loop_detected_count_++;
        // 触发回调（→ IncrementalOptimizer）
        for (auto& cb : loop_cbs_) cb(lc);
    }

    ALOG_INFO(MOD, "[TEASER_SUMMARY] query_id={} total_candidates={} reject_success_or_inlier={} reject_rmse={} reject_exception={} accepted={} (若 accepted=0 则本 query 所有回环被 TEASER 拒绝，见 [TEASER_REJECT] primary_reason 或 grep LOOP_COMPUTE][TEASER] reason=)",
              query->id, static_cast<int>(valid_candidates.size()), count_reject_success_or_inlier, count_reject_rmse, count_exception, count_accepted);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(), "[TEASER_SUMMARY] query_id=%d total=%zu reject_success_or_inlier=%d reject_rmse=%d reject_exception=%d accepted=%d (分析回环全被拒: grep TEASER_REJECT)",
                    query->id, valid_candidates.size(), count_reject_success_or_inlier, count_reject_rmse, count_exception, count_accepted);
    }
    updateLoopHealthKpi(query->id, static_cast<int>(valid_candidates.size()), count_accepted);
}

void LoopDetector::addToDatabase(const SubMap::Ptr& submap) {
    if (inter_keyframe_level_enabled_ && use_scancontext_ && submap && !submap->keyframes.empty()) {
        prepareIntraSubmapDescriptors(submap);
    }
    std::unique_lock<std::shared_mutex> lk(db_mutex_);
    for (const auto& sm : db_submaps_) {
        if (sm->id == submap->id && sm->session_id == submap->session_id) {
            ALOG_DEBUG(MOD, "[LOOP_STEP] addToDatabase skip duplicate sm_id={} session={}", submap->id, submap->session_id);
            return;
        }
    }
    db_submaps_.push_back(submap);
    ALOG_DEBUG(MOD, "[LOOP_STEP] addToDatabase added sm_id={} db_size={}", submap->id, db_submaps_.size());
}

void LoopDetector::clearCurrentSessionDB() {
    std::unique_lock<std::shared_mutex> lk(db_mutex_);
    uint64_t cur_session = db_submaps_.empty() ? 0 : db_submaps_.back()->session_id;
    std::vector<std::string> removed_keys;
    for (const auto& sm : db_submaps_) {
        if (sm && sm->session_id == cur_session) {
            removed_keys.push_back(makeSubmapKey(sm->session_id, sm->id));
        }
    }
    db_submaps_.erase(
        std::remove_if(db_submaps_.begin(), db_submaps_.end(),
            [cur_session](const SubMap::Ptr& sm) { return sm->session_id == cur_session; }),
        db_submaps_.end());
    lk.unlock();

    // 同步清理 SC 索引映射，避免 sc_idx 与 db_copy 的生命周期漂移
    std::lock_guard<std::mutex> sc_lk(sc_mutex_);
    for (const auto& key : removed_keys) {
        sc_submap_to_index_.erase(key);
    }
}

size_t LoopDetector::dbSize() const {
    std::shared_lock<std::shared_mutex> lk(db_mutex_);
    return db_submaps_.size();
}

size_t LoopDetector::queueSize() const {
    std::lock_guard<std::mutex> lk(desc_mutex_);
    return desc_queue_.size();
}

std::vector<std::pair<int, Eigen::VectorXf>> LoopDetector::exportDescriptorDB() const {
    std::shared_lock<std::shared_mutex> lk(db_mutex_);
    std::vector<std::pair<int, Eigen::VectorXf>> db;
    for (const auto& sm : db_submaps_) {
        if (sm->has_descriptor) {
            db.push_back({sm->id, sm->overlap_descriptor});
        }
    }
    return db;
}

void LoopDetector::publishLoopConstraint(const LoopConstraint::Ptr& lc) {
    if (!constraint_pub_) return;
    if (!lc) return;
    // 子图级同节点自回环（例如 sm_i==sm_j 且无有效 keyframe 对）会在后端退化为 same_node，被 iSAM2 丢弃。
    // 在发布侧提前拦截，保证“发布=可入图”语义一致。
    if (lc->submap_i == lc->submap_j) {
        const bool has_kf_pair = (lc->keyframe_i >= 0 && lc->keyframe_j >= 0);
        if (!has_kf_pair || lc->keyframe_i == lc->keyframe_j) {
            ALOG_WARN(MOD,
                "[LOOP_REJECTED] publishLoopConstraint reject self-loop sm_i=%d sm_j=%d kf_i=%d kf_j=%d reason=degenerate_same_submap",
                lc->submap_i, lc->submap_j, lc->keyframe_i, lc->keyframe_j);
            if (node()) {
                RCLCPP_WARN(node()->get_logger(),
                    "[LOOP_REJECTED] publish skip degenerate same-submap loop sm_i=%d sm_j=%d kf_i=%d kf_j=%d",
                    lc->submap_i, lc->submap_j, lc->keyframe_i, lc->keyframe_j);
            }
            return;
        }
    }
    automap_pro::msg::LoopConstraintMsg msg;
    msg.header.stamp  = node() ? node()->now() : rclcpp::Clock().now();
    msg.submap_i      = lc->submap_i;
    msg.submap_j      = lc->submap_j;
    msg.session_i     = lc->session_i;
    msg.session_j     = lc->session_j;
    msg.overlap_score = lc->overlap_score;
    msg.inlier_ratio  = lc->inlier_ratio;
    msg.rmse          = lc->rmse;
    msg.is_inter_session = lc->is_inter_session;
    // 完整发布回环位姿测量，避免 ROS 消费者丢失 delta_T 语义
    const Eigen::Vector3d t = lc->delta_T.translation();
    const Eigen::Quaterniond q(lc->delta_T.rotation());
    msg.delta_pose.position.x = t.x();
    msg.delta_pose.position.y = t.y();
    msg.delta_pose.position.z = t.z();
    msg.delta_pose.orientation.x = q.x();
    msg.delta_pose.orientation.y = q.y();
    msg.delta_pose.orientation.z = q.z();
    msg.delta_pose.orientation.w = q.w();
    switch (lc->status) {
        case LoopStatus::ACCEPTED: msg.status = "ACCEPTED"; break;
        case LoopStatus::REJECTED: msg.status = "REJECTED"; break;
        case LoopStatus::PENDING:
        default: msg.status = "PENDING"; break;
    }
    // 信息矩阵扁平化
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            msg.information_matrix[i*6+j] = lc->information(i,j);
    constraint_pub_->publish(msg);
    if (node()) {
        RCLCPP_INFO(node()->get_logger(),
            "[LOOP_ACCEPTED] loop_constraint published sm_i=%d sm_j=%d score=%.3f (回环入图可减轻点云结构重影；grep LOOP_ACCEPTED 统计入图回环数)",
            lc->submap_i, lc->submap_j, lc->overlap_score);
    }
}

void LoopDetector::updateLoopHealthKpi(int query_id, int candidates, int accepted) {
    const uint64_t query_total = loop_query_total_.fetch_add(1, std::memory_order_relaxed) + 1;
    loop_candidate_total_.fetch_add(static_cast<uint64_t>(std::max(0, candidates)), std::memory_order_relaxed);
    loop_accept_total_.fetch_add(static_cast<uint64_t>(std::max(0, accepted)), std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(loop_metrics_mutex_);
        query_accept_window_.push_back(accepted > 0 ? 1 : 0);
        while (static_cast<int>(query_accept_window_.size()) > query_accept_window_size_) {
            query_accept_window_.pop_front();
        }
    }
    int zero_streak = 0;
    if (accepted == 0) {
        zero_streak = consecutive_zero_accept_queries_.fetch_add(1, std::memory_order_relaxed) + 1;
    } else {
        consecutive_zero_accept_queries_.store(0, std::memory_order_relaxed);
    }

    if (node() && (query_total % 10 == 0 || accepted > 0)) {
        const uint64_t cand_total = loop_candidate_total_.load(std::memory_order_relaxed);
        const uint64_t accept_total = loop_accept_total_.load(std::memory_order_relaxed);
        const uint64_t ot_retrieval_total = loop_ot_retrieval_total_.load(std::memory_order_relaxed);
        const uint64_t sc_retrieval_total = loop_sc_retrieval_total_.load(std::memory_order_relaxed);
        const uint64_t teaser_geom_total = loop_teaser_geom_total_.load(std::memory_order_relaxed);
        const uint64_t svd_geom_total = loop_svd_geom_total_.load(std::memory_order_relaxed);
        const uint64_t fallback_reject_total = loop_fallback_reject_total_.load(std::memory_order_relaxed);
        const uint64_t desc_queue_drop_total = loop_desc_queue_drop_total_.load(std::memory_order_relaxed);
        const uint64_t match_queue_drop_total = loop_match_queue_drop_total_.load(std::memory_order_relaxed);
        const uint64_t inflight_now = loop_teaser_async_inflight_.load(std::memory_order_relaxed);
        const uint64_t inflight_peak = loop_teaser_async_inflight_max_.load(std::memory_order_relaxed);
        const uint64_t ot_unavail_events = loop_ot_unavailable_event_total_.load(std::memory_order_relaxed);
        const uint64_t teaser_unavail_events = loop_teaser_unavailable_event_total_.load(std::memory_order_relaxed);
        const double accept_ratio = cand_total > 0 ? static_cast<double>(accept_total) / static_cast<double>(cand_total) : 0.0;
        double accept_ratio_window = 0.0;
        double teaser_p95_ms = 0.0;
        {
            std::lock_guard<std::mutex> lk(loop_metrics_mutex_);
            if (!query_accept_window_.empty()) {
                uint64_t window_accepted = 0;
                for (uint8_t v : query_accept_window_) window_accepted += static_cast<uint64_t>(v);
                accept_ratio_window = static_cast<double>(window_accepted) / static_cast<double>(query_accept_window_.size());
            }
            if (!teaser_latency_samples_ms_.empty()) {
                std::vector<double> sorted(teaser_latency_samples_ms_.begin(), teaser_latency_samples_ms_.end());
                std::sort(sorted.begin(), sorted.end());
                const size_t idx = static_cast<size_t>(std::floor(0.95 * static_cast<double>(sorted.size() - 1)));
                teaser_p95_ms = sorted[idx];
            }
        }
        RCLCPP_INFO(node()->get_logger(),
            "[CONSTRAINT_KPI][LOOP] query_total=%lu candidate_total=%lu accepted_total=%lu accept_ratio=%.4f accept_ratio_window=%.4f retrieval_ot=%lu retrieval_sc=%lu geom_teaser=%lu geom_svd=%lu fallback_reject=%lu desc_queue_drop=%lu match_queue_drop=%lu teaser_inflight_now=%lu teaser_inflight_peak=%lu teaser_p95_ms=%.2f ot_unavailable_events=%lu teaser_unavailable_events=%lu last_query=%d last_candidates=%d last_accepted=%d zero_accept_streak=%d",
            static_cast<unsigned long>(query_total),
            static_cast<unsigned long>(cand_total),
            static_cast<unsigned long>(accept_total),
            accept_ratio,
            accept_ratio_window,
            static_cast<unsigned long>(ot_retrieval_total),
            static_cast<unsigned long>(sc_retrieval_total),
            static_cast<unsigned long>(teaser_geom_total),
            static_cast<unsigned long>(svd_geom_total),
            static_cast<unsigned long>(fallback_reject_total),
            static_cast<unsigned long>(desc_queue_drop_total),
            static_cast<unsigned long>(match_queue_drop_total),
            static_cast<unsigned long>(inflight_now),
            static_cast<unsigned long>(inflight_peak),
            teaser_p95_ms,
            static_cast<unsigned long>(ot_unavail_events),
            static_cast<unsigned long>(teaser_unavail_events),
            query_id, candidates, accepted, zero_streak);
    }

    if (node() && zero_accept_warn_consecutive_queries_ > 0 &&
        zero_streak >= zero_accept_warn_consecutive_queries_) {
        RCLCPP_WARN(node()->get_logger(),
            "[LOOP_HEALTH][WATCHDOG] consecutive_zero_accept_queries=%d threshold=%d (query_id=%d). Check OT model/ScanContext and TEASER thresholds.",
            zero_streak, zero_accept_warn_consecutive_queries_, query_id);
    }
}

void LoopDetector::prepareIntraSubmapDescriptors(const SubMap::Ptr& submap) {
    std::lock_guard<std::mutex> lk(intra_prepare_mutex_);
    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][PREPARE] 子图内描述子：强制使用 ScanContext，不使用 fallback
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][PREPARE] ====== START ====== submap_id={} keyframes={} use_scancontext={}",
        submap ? submap->id : -1,
        submap ? submap->keyframes.size() : size_t(0),
        use_scancontext_);

    if (!submap) {
        ALOG_ERROR(MOD, "[INTRA_LOOP][PREPARE] ERROR: submap is null");
        return;
    }

    if (submap->keyframes.empty()) {
        ALOG_WARN(MOD, "[INTRA_LOOP][PREPARE] WARN: submap_id={} has no keyframes, skip", submap->id);
        return;
    }

    // 若已准备好（与 keyframes 数量一致），跳过
    const bool use_sc = use_scancontext_;
    const size_t need = submap->keyframes.size();
    if (use_sc && submap->keyframe_scancontexts_.size() == need) {
        ALOG_INFO(MOD, "[INTRA_LOOP][PREPARE] ScanContext descriptors already ready, skip");
        return;
    }
    if (!use_sc && submap->keyframe_descriptors.size() == need) {
        ALOG_INFO(MOD, "[INTRA_LOOP][PREPARE] descriptors already ready, skip");
        return;
    }

    submap->keyframe_descriptors.clear();
    submap->keyframe_scancontexts_.clear();
    submap->keyframe_clouds_ds.clear();
    submap->keyframe_descriptors.reserve(need);
    submap->keyframe_scancontexts_.reserve(need);
    submap->keyframe_clouds_ds.reserve(need);

    int success_count = 0;
    int null_kf_skipped = 0;
    int null_cloud_skipped = 0;

    for (size_t i = 0; i < need; ++i) {
        const auto& kf = submap->keyframes[i];
        if (!kf) {
            null_kf_skipped++;
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
            if (use_sc) submap->keyframe_scancontexts_.push_back(Eigen::MatrixXd());
            submap->keyframe_clouds_ds.push_back(nullptr);
            ALOG_WARN(MOD, "[INTRA_LOOP][PREPARE] WARN: keyframe {} is null, skipped", i);
            continue;
        }

        if (!kf->cloud_body) {
            null_cloud_skipped++;
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
            if (use_sc) submap->keyframe_scancontexts_.push_back(Eigen::MatrixXd());
            submap->keyframe_clouds_ds.push_back(nullptr);
            ALOG_WARN(MOD, "[INTRA_LOOP][PREPARE] WARN: keyframe {} (id={}) has no cloud_body, skipped",
                      i, kf->id);
            continue;
        }

        CloudXYZIPtr cloud_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(0.5f, 0.5f, 0.5f);
        voxel.setInputCloud(kf->cloud_body);
        voxel.filter(*cloud_ds);

        size_t pts_before = kf->cloud_body->size();
        size_t pts_after = cloud_ds->size();
        float downsample_ratio = pts_before > 0 ? (float)pts_after / pts_before : 0.0f;

        auto t0 = std::chrono::steady_clock::now();
        if (use_sc) {
            // 回环强制使用 ScanContext 计算描述子（不使用 fallback）
            Eigen::MatrixXd sc;
            {
                std::lock_guard<std::mutex> sc_lk(sc_mutex_);
                sc = sc_manager_.makeScancontext(*cloud_ds);
            }
            submap->keyframe_scancontexts_.push_back(sc);
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));  // 占位，保持 size 一致
        } else {
            Eigen::VectorXf desc = overlap_infer_.computeDescriptor(cloud_ds);
            submap->keyframe_descriptors.push_back(desc);
        }
        submap->keyframe_clouds_ds.push_back(cloud_ds);
        auto t1 = std::chrono::steady_clock::now();
        float infer_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

        success_count++;
        if (use_sc) {
            ALOG_INFO(MOD,
                "[INTRA_LOOP][PREPARE] kf_idx={} kf_id={} pts_before={} pts_after={} "
                "downsample_ratio={:.2f} (ScanContext) infer_ms={:.2f}",
                i, kf->id, pts_before, pts_after, downsample_ratio, infer_ms);
        } else {
            float desc_norm = submap->keyframe_descriptors.back().norm();
            ALOG_INFO(MOD,
                "[INTRA_LOOP][PREPARE] kf_idx={} kf_id={} pts_before={} pts_after={} "
                "downsample_ratio={:.2f} desc_norm={:.4f} infer_ms={:.2f}",
                i, kf->id, pts_before, pts_after, downsample_ratio, desc_norm, infer_ms);
        }
    }

    ALOG_INFO(MOD,
        "[INTRA_LOOP][PREPARE] ====== DONE ====== "
        "submap_id={} total_kf={} success={} null_kf={} null_cloud={} "
        "desc_db_size={} sc_db_size={} clouds_db_size={}",
        submap->id, submap->keyframes.size(),
        success_count, null_kf_skipped, null_cloud_skipped,
        submap->keyframe_descriptors.size(),
        submap->keyframe_scancontexts_.size(),
        submap->keyframe_clouds_ds.size());
}

void LoopDetector::ensureIntraSubmapDescriptorsUpTo(const SubMap::Ptr& submap, int up_to_index_inclusive) {
    if (!submap || submap->keyframes.empty() || up_to_index_inclusive < 0) return;
    const size_t need_size = static_cast<size_t>(up_to_index_inclusive) + 1;
    const bool use_sc = use_scancontext_;
    if (use_sc ? (submap->keyframe_scancontexts_.size() >= need_size)
               : (submap->keyframe_descriptors.size() >= need_size)) return;

    std::lock_guard<std::mutex> lk(intra_prepare_mutex_);
    // 双重检查：可能另一线程已在持有锁期间填满
    if (use_sc ? (submap->keyframe_scancontexts_.size() >= need_size)
               : (submap->keyframe_descriptors.size() >= need_size)) return;

    const size_t start = use_sc ? submap->keyframe_scancontexts_.size() : submap->keyframe_descriptors.size();
    submap->keyframe_descriptors.reserve(need_size);
    submap->keyframe_scancontexts_.reserve(need_size);
    submap->keyframe_clouds_ds.reserve(need_size);

    for (size_t i = start; i < need_size && i < submap->keyframes.size(); ++i) {
        const auto& kf = submap->keyframes[i];
        if (!kf) {
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
            if (use_sc) submap->keyframe_scancontexts_.push_back(Eigen::MatrixXd());
            submap->keyframe_clouds_ds.push_back(nullptr);
            continue;
        }
        if (!kf->cloud_body || kf->cloud_body->empty()) {
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
            if (use_sc) submap->keyframe_scancontexts_.push_back(Eigen::MatrixXd());
            submap->keyframe_clouds_ds.push_back(nullptr);
            continue;
        }
        CloudXYZIPtr cloud_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(0.5f, 0.5f, 0.5f);
        voxel.setInputCloud(kf->cloud_body);
        voxel.filter(*cloud_ds);
        if (use_sc) {
            Eigen::MatrixXd sc;
            {
                std::lock_guard<std::mutex> sc_lk(sc_mutex_);
                sc = sc_manager_.makeScancontext(*cloud_ds);
            }
            submap->keyframe_scancontexts_.push_back(sc);
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
        } else {
            ALOG_INFO(MOD, "[OT_CRASH_LOC] step=intra_about_to_compute submap_id={} kf_idx={} pts={} tid={} lwp={} (崩溃时本条为当前关键帧)",
                     submap->id, static_cast<int>(i), cloud_ds ? cloud_ds->size() : 0u,
                     automap_pro::logThreadId(), automap_pro::logLwp());
            submap->keyframe_descriptors.push_back(overlap_infer_.computeDescriptor(cloud_ds));
        }
        submap->keyframe_clouds_ds.push_back(cloud_ds);
    }
    ALOG_INFO(MOD, "[LOOP_DESC][INTRA] ensureIntraSubmapDescriptorsUpTo done submap_id={} from_idx={} to_idx={} desc_count={} (子图内关键帧描述子补齐)",
              submap->id, static_cast<int>(start), up_to_index_inclusive,
              use_sc ? static_cast<int>(submap->keyframe_scancontexts_.size()) : static_cast<int>(submap->keyframe_descriptors.size()));
}

std::vector<LoopConstraint::Ptr> LoopDetector::detectIntraSubmapLoop(
    const SubMap::Ptr& submap,
    int query_keyframe_idx) {

    std::vector<LoopConstraint::Ptr> results;

    // 检查是否启用子图内回环检测
    if (!intra_submap_enabled_) {
        return results;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][DEBUG] 输入参数详细记录
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][DEBUG] ====== detectIntraSubmapLoop START ====== "
        "submap_id={} query_idx={} kf_count={} "
        "min_temporal_gap={:.1f}s min_keyframe_gap={} min_dist_gap={:.1f}m overlap_thresh={:.3f} "
        "min_inlier_ratio={:.3f} max_rmse={:.3f}",
        submap ? submap->id : -1, query_keyframe_idx,
        submap ? submap->keyframes.size() : 0u,
        intra_submap_min_temporal_gap_, intra_submap_min_keyframe_gap_,
        intra_submap_min_distance_gap_,
        intra_submap_overlap_threshold_,
        min_inlier_ratio_, max_rmse_);

    if (!submap || query_keyframe_idx < 0 ||
        static_cast<size_t>(query_keyframe_idx) >= submap->keyframes.size()) {
        ALOG_ERROR(MOD,
            "[INTRA_LOOP][ERROR] INVALID_INPUT: submap_id={} query_idx={} kf_size={} "
            "(reason: {})",
            submap ? submap->id : -1, query_keyframe_idx,
            submap ? submap->keyframes.size() : 0u,
            !submap ? "submap_null" :
            query_keyframe_idx < 0 ? "negative_index" :
            "index_out_of_range");
        return results;
    }

    const auto& query_kf = submap->keyframes[query_keyframe_idx];
    if (!query_kf) {
        ALOG_ERROR(MOD, "[INTRA_LOOP][ERROR] query keyframe is null at idx={}", query_keyframe_idx);
        return results;
    }

    if (!query_kf->cloud_body) {
        ALOG_ERROR(MOD, "[INTRA_LOOP][ERROR] query keyframe has no cloud: kf_id={} idx={} pts=0",
                   query_kf->id, query_keyframe_idx);
        return results;
    }

    ALOG_INFO(MOD,
        "[INTRA_LOOP][DEBUG] descriptor_db_status: stored_size={} sc_size={} expected_size={} use_scancontext={}",
        submap->keyframe_descriptors.size(), submap->keyframe_scancontexts_.size(),
        submap->keyframes.size(), use_scancontext_ ? 1 : 0);

    int last_idx = static_cast<int>(submap->keyframes.size()) - 1;
    if (last_idx >= 0) {
        ensureIntraSubmapDescriptorsUpTo(submap, last_idx);
    }

    const bool use_sc = use_scancontext_;
    const size_t kf_count = submap->keyframes.size();
    if (use_sc) {
        if (submap->keyframe_scancontexts_.empty() || submap->keyframe_scancontexts_.size() != kf_count) {
            ALOG_WARN(MOD,
                "[INTRA_LOOP][WARN] skip intra-loop: ScanContext not ready (sc_size={}, expected={})",
                submap->keyframe_scancontexts_.size(), kf_count);
            if (node()) {
                RCLCPP_WARN(node()->get_logger(),
                    "[INTRA_LOOP][WARN] skip intra-loop: ScanContext not ready sc_size=%zu expected=%zu (grep INTRA_LOOP)",
                    submap->keyframe_scancontexts_.size(), kf_count);
            }
            return results;
        }
    } else {
        if (submap->keyframe_descriptors.empty() || submap->keyframe_descriptors.size() != kf_count) {
            ALOG_WARN(MOD,
                "[INTRA_LOOP][WARN] skip intra-loop: descriptors not ready (size={}, expected={})",
                submap->keyframe_descriptors.size(), kf_count);
            if (node()) {
                RCLCPP_WARN(node()->get_logger(),
                    "[INTRA_LOOP][WARN] skip intra-loop: descriptors not ready size=%zu expected=%zu (grep INTRA_LOOP)",
                    submap->keyframe_descriptors.size(), kf_count);
            }
            return results;
        }
    }

    float query_desc_norm = 0.0f;
    if (!use_sc) {
        const auto& query_desc = submap->keyframe_descriptors[query_keyframe_idx];
        query_desc_norm = query_desc.norm();
        ALOG_INFO(MOD,
            "[INTRA_LOOP][DEBUG] query_descriptor: kf_idx={} kf_id={} norm={:.6f} pts={}",
            query_keyframe_idx, query_kf->id, query_desc_norm, query_kf->cloud_body->size());
        if (query_desc_norm < 1e-6f) {
            ALOG_ERROR(MOD, "[INTRA_LOOP][ERROR] query descriptor is zero: kf_idx={} (skip)", query_keyframe_idx);
            return results;
        }
    } else {
        const auto& query_sc = submap->keyframe_scancontexts_[query_keyframe_idx];
        if (query_sc.size() == 0) {
            ALOG_ERROR(MOD, "[INTRA_LOOP][ERROR] query ScanContext is empty: kf_idx={} (skip)", query_keyframe_idx);
            return results;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][INFO] 开始检索历史关键帧
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][INFO] START_SEARCH: submap_id={} query_kf_idx={} query_ts={:.6f} "
        "query_pos=[{:.2f},{:.2f},{:.2f}] history_kf_count={}",
        submap->id, query_keyframe_idx, query_kf->timestamp,
        query_kf->T_odom_b.translation().x(),
        query_kf->T_odom_b.translation().y(),
        query_kf->T_odom_b.translation().z(),
        submap->keyframes.size());

    // 检索历史关键帧（同一子图内）
    int candidates_found = 0;
    int temporal_filtered = 0;
    int index_filtered = 0;
    int distance_filtered = 0;
    int desc_filtered = 0;
    int null_cand_skipped = 0;
    int empty_cloud_skipped = 0;
    int teaser_failed = 0;
    int teaser_rejected_inlier = 0;
    int teaser_rejected_rmse = 0;
    int teaser_rejected_pose_anomaly = 0;
    int teaser_invoked = 0;  // 本帧已调用 TEASER 次数，超过上限则停止（避免 10s+ 卡住）

    for (int i = 0; i < query_keyframe_idx; ++i) {
        if (intra_submap_max_teaser_candidates_ > 0 && teaser_invoked >= intra_submap_max_teaser_candidates_) {
            ALOG_DEBUG(MOD, "[INTRA_LOOP][CAP] TEASER invoked {} >= max {}, skip remaining candidates",
                       teaser_invoked, intra_submap_max_teaser_candidates_);
            break;
        }
        const auto& cand_kf = submap->keyframes[i];
        if (!cand_kf) {
            null_cand_skipped++;
            continue;
        }

        // 时间间隔过滤
        double temporal_gap = query_kf->timestamp - cand_kf->timestamp;
        if (temporal_gap < intra_submap_min_temporal_gap_) {
            temporal_filtered++;
            ALOG_INFO(MOD,
                "[INTRA_LOOP][FILTER] TEMPORAL_GAP: submap_id={} query_idx={} cand_idx={} cand_ts={:.3f} gap={:.2f}s < {:.1f}s (SKIP)",
                submap->id, query_keyframe_idx, i, cand_kf->timestamp, temporal_gap, intra_submap_min_temporal_gap_);
            continue;
        }

        // 索引间隔过滤（避免相邻帧）
        int index_gap = query_keyframe_idx - i;
        if (index_gap < intra_submap_min_keyframe_gap_) {
            index_filtered++;
            ALOG_INFO(MOD,
                "[INTRA_LOOP][FILTER] INDEX_GAP: submap_id={} query_idx={} cand_idx={} gap={} < {} (SKIP)",
                submap->id, query_keyframe_idx, i, index_gap, intra_submap_min_keyframe_gap_);
            continue;
        }

        // 距离间隔过滤（避免过密帧）
        if (intra_submap_min_distance_gap_ > 0.0) {
            const auto& query_pos = query_kf->T_odom_b.translation();
            const auto& cand_pos = cand_kf->T_odom_b.translation();
            double dist_gap = (query_pos - cand_pos).norm();
            if (dist_gap < intra_submap_min_distance_gap_) {
                distance_filtered++;
                ALOG_INFO(MOD,
                    "[INTRA_LOOP][FILTER] DISTANCE_GAP: submap_id={} query_idx={} cand_idx={} dist={:.2f}m < {:.1f}m (SKIP)",
                    submap->id, query_keyframe_idx, i, dist_gap, intra_submap_min_distance_gap_);
                continue;
            }
        }

        // 描述子相似度过滤：强制使用 ScanContext 时用 SC 距离，否则用 OT/向量余弦
        float similarity = 0.0f;
        if (use_sc) {
            const auto& query_sc = submap->keyframe_scancontexts_[query_keyframe_idx];
            const auto& cand_sc = submap->keyframe_scancontexts_[i];
            if (query_sc.size() == 0 || cand_sc.size() == 0) {
                ALOG_INFO(MOD, "[INTRA_LOOP][FILTER] SC_EMPTY: submap_id={} query_idx={} cand_idx={} (SKIP)", submap->id, query_keyframe_idx, i);
                continue;
            }
            std::pair<double, int> sc_result;
            {
                std::lock_guard<std::mutex> sc_lk(sc_mutex_);
                sc_result = sc_manager_.distanceBtnScanContext(query_sc, cand_sc);
            }
            double sc_dist = sc_result.first;
            if (sc_dist > sc_dist_threshold_) {
                desc_filtered++;
                ALOG_INFO(MOD,
                    "[INTRA_LOOP][FILTER] SC_DIST: submap_id={} query_idx={} cand_idx={} sc_dist={:.4f} > {:.3f} (SKIP)",
                    submap->id, query_keyframe_idx, i, sc_dist, sc_dist_threshold_);
                continue;
            }
            similarity = std::max(0.0f, 1.0f - static_cast<float>(sc_dist / sc_dist_threshold_));
        } else {
            const auto& query_desc = submap->keyframe_descriptors[query_keyframe_idx];
            const auto& cand_desc = submap->keyframe_descriptors[i];
            float cand_desc_norm = cand_desc.norm();
            if (cand_desc_norm < 1e-6f) {
                ALOG_INFO(MOD, "[INTRA_LOOP][FILTER] CAND_DESC_ZERO: submap_id={} query_idx={} cand_idx={} (SKIP)", submap->id, query_keyframe_idx, i);
                continue;
            }
            float denominator = query_desc_norm * cand_desc_norm;
            if (denominator < 1e-10f) continue;
            similarity = query_desc.dot(cand_desc) / denominator;
            if (similarity < intra_submap_overlap_threshold_) {
                desc_filtered++;
                ALOG_INFO(MOD,
                    "[INTRA_LOOP][FILTER] SIMILARITY: submap_id={} query_idx={} cand_idx={} sim={:.4f} < {:.3f} (SKIP)",
                    submap->id, query_keyframe_idx, i, similarity, intra_submap_overlap_threshold_);
                continue;
            }
        }

        candidates_found++;
        ALOG_INFO(MOD,
            "[INTRA_LOOP][CANDIDATE] FOUND: submap_id={} query_idx={} cand_idx={} "
            "sim={:.4f} gap_idx={} gap_time={:.2f}s cand_kf_id={} cand_pos=[{:.2f},{:.2f},{:.2f}]",
            submap->id, query_keyframe_idx, i, similarity, index_gap, temporal_gap, cand_kf->id,
            cand_kf->T_odom_b.translation().x(),
            cand_kf->T_odom_b.translation().y(),
            cand_kf->T_odom_b.translation().z());

        // ═══════════════════════════════════════════════════════════════════════
        // [INTRA_LOOP][DEBUG] 几何验证（TEASER++）开始
        // ═══════════════════════════════════════════════════════════════════════
        const auto& query_cloud = submap->keyframe_clouds_ds[query_keyframe_idx];
        const auto& cand_cloud = submap->keyframe_clouds_ds[i];

        if (!query_cloud || !cand_cloud || query_cloud->empty() || cand_cloud->empty()) {
            empty_cloud_skipped++;
            ALOG_ERROR(MOD,
                "[INTRA_LOOP][ERROR] EMPTY_CLOUD: query_idx={} cand_idx={} "
                "query_valid={} cand_valid={} (SKIP)",
                query_keyframe_idx, i,
                query_cloud ? "valid" : "null",
                cand_cloud ? "valid" : "null");
            continue;
        }

        teaser_invoked++;
        ALOG_INFO(MOD,
            "[INTRA_LOOP][TEASER_START] query_idx={} cand_idx={} "
            "query_pts={} cand_pts={} invoked={}",
            query_keyframe_idx, i, query_cloud->size(), cand_cloud->size(), teaser_invoked);

        // TEASER++ 配准
        TeaserMatcher::Result teaser_res;
        bool teaser_success = false;
        try {
            teaser_res = teaser_matcher_.match(query_cloud, cand_cloud);
            teaser_success = teaser_res.success;
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD,
                "[INTRA_LOOP][EXCEPTION] TEASER_EXCEPTION: query_idx={} cand_idx={} exception='{}'",
                query_keyframe_idx, i, e.what());
            teaser_failed++;
            continue;
        } catch (...) {
            ALOG_ERROR(MOD,
                "[INTRA_LOOP][EXCEPTION] TEASER_UNKNOWN_EXCEPTION: query_idx={} cand_idx={}",
                query_keyframe_idx, i);
            teaser_failed++;
            continue;
        }

        if (!teaser_success) {
            teaser_failed++;
            ALOG_WARN(MOD,
                "[INTRA_LOOP][TEASER] FAILED_RETURN: query_idx={} cand_idx={} success=false (SKIP)",
                query_keyframe_idx, i);
            continue;
        }

        if (!teaser_res.success) {
            teaser_failed++;
            ALOG_WARN(MOD,
                "[INTRA_LOOP][TEASER] INVALID_RESULT: query_idx={} cand_idx={} is_valid=false (SKIP)",
                query_keyframe_idx, i);
            continue;
        }

        // 检查 inlier ratio
        if (teaser_res.inlier_ratio < min_inlier_ratio_) {
            teaser_rejected_inlier++;
            ALOG_INFO(MOD,
                "[INTRA_LOOP][TEASER] REJECT_INLIER: submap_id={} query_idx={} cand_idx={} "
                "inlier_ratio={:.4f} inliers≈{} corrs={} < min_ratio={:.3f} (SKIP)",
                submap->id, query_keyframe_idx, i, teaser_res.inlier_ratio,
                static_cast<int>(teaser_res.inlier_ratio * std::max(0, teaser_res.num_correspondences)),
                teaser_res.num_correspondences, min_inlier_ratio_);
            continue;
        }

        // 检查 RMSE
        if (teaser_res.rmse > max_rmse_) {
            teaser_rejected_rmse++;
            ALOG_INFO(MOD,
                "[INTRA_LOOP][TEASER] REJECT_RMSE: submap_id={} query_idx={} cand_idx={} "
                "rmse={:.4f} > max_rmse={:.3f} (SKIP)",
                submap->id, query_keyframe_idx, i, teaser_res.rmse, max_rmse_);
            continue;
        }

        // ── 回环位姿一致性检查：TEASER 结果应与里程计相对位姿接近（回环应是微调，不应出现巨大变化）──
        // 平移差 trans_diff_m = ||t_teaser - t_odom||：反向运动时两向量相反，差向量模约 2*|t|，会超阈值；
        // 旋转差 rot_diff_deg = angle(R_teaser * R_odom^T) ∈ [0,180]°：方向差约 180° 时相对旋转角≈180°，会超阈值。故反向运动（含平移偏移）会被正确拒绝。
        const double max_trans_diff = pose_consistency_max_trans_m_;
        const double max_rot_diff_deg = pose_consistency_max_rot_deg_;
        double trans_diff_m = 0.0;
        double rot_diff_deg = 0.0;
        if (max_trans_diff > 0.0 || max_rot_diff_deg > 0.0) {
            const KeyFrame::Ptr& cand_kf = submap->keyframes[i];
            if (cand_kf) {
                const Pose3d T_w_cand = cand_kf->T_odom_b;
                const Pose3d T_w_query = query_kf->T_odom_b;
                const Pose3d T_tgt_src_odom = T_w_cand.inverse() * T_w_query;
                trans_diff_m = (teaser_res.T_tgt_src.translation() - T_tgt_src_odom.translation()).norm();
                const Eigen::Matrix3d R_teaser = teaser_res.T_tgt_src.linear();
                const Eigen::Matrix3d R_odom = T_tgt_src_odom.linear();
                const Eigen::Matrix3d R_diff = R_teaser * R_odom.transpose();
                const double trace_r = R_diff.trace();
                const double angle_rad = std::acos(std::max(-1.0, std::min(1.0, (trace_r - 1.0) * 0.5)));
                rot_diff_deg = angle_rad * 180.0 / M_PI;
                const bool trans_anomaly = (max_trans_diff > 0.0 && trans_diff_m > max_trans_diff);
                const bool rot_anomaly = (max_rot_diff_deg > 0.0 && rot_diff_deg > max_rot_diff_deg);
                if (trans_anomaly || rot_anomaly) {
                    teaser_rejected_pose_anomaly++;
                    const Eigen::Vector3d t_odom = T_tgt_src_odom.translation();
                    const Eigen::Vector3d t_teaser = teaser_res.T_tgt_src.translation();
                    const Eigen::Vector3d t_diff = t_teaser - t_odom;
                    const double odom_rot_deg = Eigen::AngleAxisd(R_odom).angle() * 180.0 / M_PI;
                    const double teaser_rot_deg = Eigen::AngleAxisd(R_teaser).angle() * 180.0 / M_PI;
                    
                    // 增加根本原因猜测日志
                    std::string root_cause = "unknown";
                    if (rot_diff_deg > 150.0) root_cause = "possibly_matched_backwards (180deg flip)";
                    else if (trans_diff_m > 30.0) root_cause = "massive_odometry_drift_or_wrong_place";
                    else if (trans_anomaly && !rot_anomaly) root_cause = "translation_only_mismatch (check voxel/feature)";
                    else if (!trans_anomaly && rot_anomaly) root_cause = "rotation_only_mismatch (check feature ambiguity)";

                    ALOG_WARN(MOD,
                        "[INTRA_LOOP][REJECT] pose_anomaly: submap_id={} kf_i={} kf_j={} kf_id_cand={} kf_id_query={} ts_cand={:.3f} ts_query={:.3f} cause={} | trans_diff={:.2f}m rot_diff={:.2f}deg",
                        submap->id, i, query_keyframe_idx, cand_kf->id, query_kf->id, cand_kf->timestamp, query_kf->timestamp, root_cause, trans_diff_m, rot_diff_deg);
                    
                    // [GHOSTING_DIAG] 如果几何匹配很好但被位姿一致性检查拒绝，记录为潜在的“被错杀的真实回环”或“隐蔽的误匹配”
                    if (teaser_res.inlier_ratio > 0.10 && teaser_res.rmse < 0.4) {
                        RCLCPP_ERROR(node()->get_logger(),
                            "[INTRA_LOOP][POSE_CONFLICT] STRONG geometric match REJECTED by pose consistency: "
                            "submap_id=%d kf_%d->%d inlier=%.3f rmse=%.3f trans_diff=%.2fm (max=%.1f) rot_diff=%.2fdeg (max=%.1f). "
                            "If ghosting persists, consider if this loop was actually correct! (Maybe large drift?)",
                            submap->id, i, query_keyframe_idx, teaser_res.inlier_ratio, teaser_res.rmse, trans_diff_m, max_trans_diff, rot_diff_deg, max_rot_diff_deg);
                    }

                    ALOG_WARN(MOD,
                        "[INTRA_LOOP][REJECT] pose_anomaly ODOM_rel:  trans_xyz=[{:.4f},{:.4f},{:.4f}] trans_norm={:.4f}m rot_deg={:.2f}",
                        t_odom.x(), t_odom.y(), t_odom.z(), t_odom.norm(), odom_rot_deg);
                    ALOG_WARN(MOD,
                        "[INTRA_LOOP][REJECT] pose_anomaly TEASER_rel: trans_xyz=[{:.4f},{:.4f},{:.4f}] trans_norm={:.4f}m rot_deg={:.2f} inlier={:.4f} rmse={:.4f}",
                        t_teaser.x(), t_teaser.y(), t_teaser.z(), t_teaser.norm(), teaser_rot_deg,
                        teaser_res.inlier_ratio, teaser_res.rmse);
                    ALOG_WARN(MOD,
                        "[INTRA_LOOP][REJECT] pose_anomaly DIFF: trans_diff_xyz=[{:.4f},{:.4f},{:.4f}] trans_diff_norm={:.4f}m rot_diff_deg={:.2f} | thresholds trans={:.1f}m rot={:.1f}deg (高精建图不应出现大变化，可能为计算/逻辑错误)",
                        t_diff.x(), t_diff.y(), t_diff.z(), trans_diff_m, rot_diff_deg, max_trans_diff, max_rot_diff_deg);
                    ALOG_WARN(MOD,
                        "[INTRA_LOOP][REJECT] pose_anomaly WORLD: cand_pos=[{:.3f},{:.3f},{:.3f}] query_pos=[{:.3f},{:.3f},{:.3f}] dist_world={:.3f}m",
                        T_w_cand.translation().x(), T_w_cand.translation().y(), T_w_cand.translation().z(),
                        T_w_query.translation().x(), T_w_query.translation().y(), T_w_query.translation().z(),
                        (T_w_query.translation() - T_w_cand.translation()).norm());
                    if (node()) {
                        RCLCPP_WARN(node()->get_logger(),
                            "[INTRA_LOOP][REJECT] pose_anomaly submap_id=%d kf_i=%d kf_j=%d kf_id_cand=%lu kf_id_query=%lu trans_diff=%.3fm rot_diff=%.2fdeg cause=%s (详见上方 ODOM_rel/TEASER_rel/DIFF/WORLD；grep INTRA_LOOP REJECT)",
                            submap->id, i, query_keyframe_idx, static_cast<unsigned long>(cand_kf->id), static_cast<unsigned long>(query_kf->id), trans_diff_m, rot_diff_deg, root_cause.c_str());
                    }
                    continue;
                }
            }
        }

        // ═══════════════════════════════════════════════════════════════════════
        // [INTRA_LOOP][INFO] 检测到回环！
        // ═══════════════════════════════════════════════════════════════════════
        const Eigen::Vector3d& t = teaser_res.T_tgt_src.translation();
        const Eigen::Matrix3d rot = teaser_res.T_tgt_src.rotation();
        const Eigen::Vector3d rpy = Eigen::Matrix3d(rot).eulerAngles(2, 1, 0).reverse();
        // 根据 inlier ratio 构造信息矩阵（TeaserMatcher::Result 没有 information 字段）
        // 修复: 使用更合理的信息矩阵计算，区分旋转和平移自由度
        double info_scale = 100.0 * teaser_res.inlier_ratio / (teaser_res.rmse + 0.01);
        // 限制最大信息尺度，避免数值过大导致优化不稳定
        info_scale = std::min(info_scale, 2e4);
        Mat66d information = Mat66d::Identity();
        information.block<3,3>(0,0) *= info_scale * 0.5;  // 适度增加平移权重 (0.1 -> 0.5)
        information.block<3,3>(3,3) *= info_scale;        // 旋转权重保持比例

        // [GHOSTING_DIAG] 记录信息矩阵强度，防止过强导致系统“硬扭”
        RCLCPP_INFO(node()->get_logger(), 
            "[INTRA_LOOP][INFO_DIAG] sm_id=%d kf_%d->%d info_scale=%.2e trans_w=%.2e rot_w=%.2e",
            submap->id, i, query_keyframe_idx, info_scale, information(0,0), information(3,3));

        ALOG_INFO(MOD,
            "[INTRA_LOOP][DETECTED] ★★★ INTRA_SUBMAP_LOOP ★★★ "
            "submap_id={} kf_i={} kf_j={} "
            "overlap={:.4f} inlier={:.4f} rmse={:.4f} "
            "delta_t=[{:.3f},{:.3f},{:.3f}] delta_rpy=[{:.2f},{:.2f},{:.2f}] "
            "info_norm={:.2e}",
            submap->id, i, query_keyframe_idx,
            similarity, teaser_res.inlier_ratio, teaser_res.rmse,
            t.x(), t.y(), t.z(),
            rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI,
            information.norm());
        if (node()) {
            RCLCPP_INFO(node()->get_logger(),
                "[INTRA_LOOP][DETECTED] submap_id=%d kf_i=%d kf_j=%d overlap=%.4f inlier=%.4f rmse=%.4f (grep INTRA_LOOP)",
                submap->id, i, query_keyframe_idx, similarity, teaser_res.inlier_ratio, teaser_res.rmse);
        }

        // 创建回环约束
        // [GHOSTING_DIAG] 记录接近阈值的“危险”回环，这些往往是重影的源头
        if (trans_diff_m > max_trans_diff * 0.8 || rot_diff_deg > max_rot_diff_deg * 0.8) {
            RCLCPP_WARN(node()->get_logger(),
                "[INTRA_LOOP][RISK] Borderline loop accepted: submap_id=%d kf_i=%d kf_j=%d trans_diff=%.2fm (max=%.1f) rot_diff=%.2fdeg (max=%.1f)",
                submap->id, i, query_keyframe_idx, trans_diff_m, max_trans_diff, rot_diff_deg, max_rot_diff_deg);
        }

        auto lc = std::make_shared<LoopConstraint>();
        lc->submap_i = submap->id;
        lc->submap_j = submap->id;
        lc->session_i = submap->session_id;
        lc->session_j = submap->session_id;
        lc->keyframe_i = i;
        lc->keyframe_j = query_keyframe_idx;
        lc->overlap_score = similarity;
        lc->inlier_ratio = teaser_res.inlier_ratio;
        lc->rmse = teaser_res.rmse;
        lc->delta_T = teaser_res.T_tgt_src;
        lc->information = information;

        results.push_back(lc);

        // 发布回环约束
        publishLoopConstraint(lc);

        // 触发回调
        for (auto& cb : loop_cbs_) {
            try {
                cb(lc);
            } catch (const std::exception& e) {
                ALOG_ERROR(MOD, "[INTRA_LOOP][CALLBACK] exception: {}", e.what());
            }
        }

        loop_detected_count_++;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][SUMMARY] 统计信息（同时打 RCLCPP 便于 full.log 观测）
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][SUMMARY] ====== detectIntraSubmapLoop END ====== "
        "submap_id={} query_kf_idx={} total_history_kf={} "
        "candidates_found={} "
        "filtered: null={} temporal={} index={} dist={} desc={} empty_cloud={} "
        "teaser_invoked={} teaser_failed={} reject_inlier={} reject_rmse={} reject_pose_anomaly={} "
        "FINAL_detected={}",
        submap->id, query_keyframe_idx, submap->keyframes.size(),
        candidates_found,
        null_cand_skipped, temporal_filtered, index_filtered, distance_filtered, desc_filtered, empty_cloud_skipped,
        teaser_invoked, teaser_failed, teaser_rejected_inlier, teaser_rejected_rmse, teaser_rejected_pose_anomaly,
        results.size());
    if (node()) {
        RCLCPP_INFO(node()->get_logger(),
            "[INTRA_LOOP][SUMMARY] submap_id=%d query_kf_idx=%d history_kf=%zu candidates_found=%d "
            "filtered: temporal=%d index=%d dist=%d desc=%d teaser_invoked=%d failed=%d reject_inlier=%d reject_rmse=%d reject_pose_anomaly=%d FINAL_detected=%zu (grep INTRA_LOOP)",
            submap->id, query_keyframe_idx, submap->keyframes.size(), candidates_found,
            temporal_filtered, index_filtered, distance_filtered, desc_filtered,
            teaser_invoked, teaser_failed, teaser_rejected_inlier, teaser_rejected_rmse, teaser_rejected_pose_anomaly,
            static_cast<size_t>(results.size()));
    }

    return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// ScanContext 候选检索实现
// ─────────────────────────────────────────────────────────────────────────────
std::vector<OverlapTransformerInfer::Candidate> LoopDetector::retrieveUsingScanContext(
    const SubMap::Ptr& submap,
    const std::vector<SubMap::Ptr>& db_copy) {

    std::vector<OverlapTransformerInfer::Candidate> candidates;

    ALOG_INFO(MOD, "[LOOP_STEP] stage=ScanContext_retrieve_enter query_id={} db_size={} sc_size={} dist_threshold={:.3f} num_candidates={} exclude_recent={}",
              submap->id, db_copy.size(), sc_manager_.polarcontexts_.size(),
              sc_dist_threshold_, sc_num_candidates_, sc_exclude_recent_);
    ALOG_INFO(MOD, "[ScanContext][START] ====== retrieveUsingScanContext START ====== "
              "query_id=%d db_size=%zu sc_size=%zu",
              submap->id, db_copy.size(), sc_manager_.polarcontexts_.size());

    if (!use_scancontext_) {
        ALOG_WARN(MOD, "[ScanContext] ScanContext is disabled, skip retrieval");
        return candidates;
    }

    if (!submap || !submap->downsampled_cloud || submap->downsampled_cloud->empty()) {
        ALOG_WARN(MOD, "[ScanContext] Invalid submap or empty cloud, skip retrieval");
        return candidates;
    }

    // 将当前子图的下采样点云转换为 ScanContext 格式
    pcl::PointCloud<SCPointType> scan_down;
    scan_down.reserve(submap->downsampled_cloud->size());
    for (const auto& pt : submap->downsampled_cloud->points) {
        SCPointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = 0;
        scan_down.points.push_back(p);
    }
    ALOG_INFO(MOD, "[ScanContext] Converted cloud: query_id=%d pts=%zu",
              submap->id, scan_down.points.size());

    // 生成当前帧的 ScanContext
    sc_manager_.makeAndSaveScancontextAndKeys(scan_down);
    const std::string query_key = makeSubmapKey(submap->session_id, submap->id);
    const int latest_sc_idx = static_cast<int>(sc_manager_.polarcontexts_.size()) - 1;
    if (latest_sc_idx >= 0) {
        sc_submap_to_index_[query_key] = latest_sc_idx;
        if (sc_index_to_submap_.size() <= static_cast<size_t>(latest_sc_idx)) {
            sc_index_to_submap_.resize(static_cast<size_t>(latest_sc_idx) + 1);
        }
        sc_index_to_submap_[latest_sc_idx] = query_key;
    }
    ALOG_INFO(MOD, "[ScanContext] SC generated: query_id=%d total_sc=%zu",
              submap->id, sc_manager_.polarcontexts_.size());

    // 检查 SCManager 中是否有足够的历史帧用于检索
    // 注意：当前为子图级 SC（每子图 1 条），exclude_recent 按“子图”计；若未做 cap，
    // 需要 total >= exclude_recent+2 才有候选（如 exclude_recent=3 需 5 子图），导致前期永远无回环。
    // 修复：对子图级做 cap，保证至少 2 个子图即可开始检索（available >= 1）。
    const int total_sc = static_cast<int>(sc_manager_.polarcontexts_.size());
    const int effective_exclude_recent = std::min(
        sc_exclude_recent_,
        std::max(0, total_sc - 2));  // 最多排除 (total-2)，保证至少 1 条可搜
    int num_available_for_search = std::max(0, total_sc - 1 - effective_exclude_recent);
    ALOG_INFO(MOD, "[ScanContext] Check history: total_sc=%zu exclude_recent=%d effective_exclude=%d available=%d",
              sc_manager_.polarcontexts_.size(), sc_exclude_recent_, effective_exclude_recent, num_available_for_search);

    if (num_available_for_search < 1) {
        ALOG_INFO(MOD, "[ScanContext] Not enough history frames for search: available=%d < 1, skip",
                  num_available_for_search);
        return candidates;
    }

    // 获取当前帧的描述子（刚刚添加的，位于末尾）
    const auto& curr_key = sc_manager_.polarcontext_invkeys_mat_.back();
    const auto& curr_desc = sc_manager_.polarcontexts_.back();
    ALOG_INFO(MOD, "[ScanContext] Current frame descriptor ready: idx=%zu",
              sc_manager_.polarcontexts_.size() - 1);

    // 首次调用或每隔 N 帧重建 KD-Tree
    // KD-Tree 搜索空间：排除最近的 sc_exclude_recent_ 帧和当前帧
    bool need_rebuild = (sc_manager_.tree_making_period_conter == 0) ||
                        (sc_manager_.tree_making_period_conter % sc_tree_making_period_ == 0);

    ALOG_INFO(MOD, "[ScanContext] KD-Tree status: counter=%d period=%d rebuild_needed=%d",
              sc_manager_.tree_making_period_conter, sc_tree_making_period_, need_rebuild);

    if (need_rebuild) {
        sc_manager_.polarcontext_invkeys_to_search_.clear();
        // 与 available 计算一致：使用 effective_exclude（子图级 cap），排除最近 N 帧和当前帧
        const size_t key_mat_size = sc_manager_.polarcontext_invkeys_mat_.size();
        const int eff_exclude = std::min(sc_exclude_recent_, std::max(0, static_cast<int>(key_mat_size) - 2));
        size_t end_idx = (key_mat_size > static_cast<size_t>(eff_exclude + 1))
            ? (key_mat_size - static_cast<size_t>(eff_exclude) - 1) : 0;
        ALOG_INFO(MOD, "[ScanContext] KD-Tree rebuild: key_mat_size=%zu effective_exclude=%d end_idx=%zu",
                  key_mat_size, eff_exclude, end_idx);

        if (end_idx > 0) {
            sc_manager_.polarcontext_invkeys_to_search_.assign(
                sc_manager_.polarcontext_invkeys_mat_.begin(),
                sc_manager_.polarcontext_invkeys_mat_.begin() + end_idx);

            // 重建 KD-Tree
            sc_manager_.polarcontext_tree_.reset();
            sc_manager_.polarcontext_tree_ = std::make_unique<InvKeyTree>(
                sc_manager_.PC_NUM_RING,
                sc_manager_.polarcontext_invkeys_to_search_,
                10);
            ALOG_INFO(MOD, "[ScanContext] KD-Tree rebuilt: search_size=%zu, total_sc=%zu",
                      sc_manager_.polarcontext_invkeys_to_search_.size(),
                      sc_manager_.polarcontexts_.size());
        }
    }
    sc_manager_.tree_making_period_conter++;

    // 检查 KD-Tree 是否有效
    if (!sc_manager_.polarcontext_tree_ || sc_manager_.polarcontext_invkeys_to_search_.empty()) {
        ALOG_WARN(MOD, "[ScanContext] KD-Tree not available (tree=%p search_space=%zu), skip retrieval",
                  static_cast<void*>(sc_manager_.polarcontext_tree_.get()),
                  sc_manager_.polarcontext_invkeys_to_search_.empty() ? 0 : sc_manager_.polarcontext_invkeys_to_search_.size());
        return candidates;
    }

    // KD-Tree 检索候选
    std::vector<size_t> candidate_indexes(sc_num_candidates_);
    std::vector<float> out_dists_sqr(sc_num_candidates_);
    nanoflann::KNNResultSet<float> knnsearch_result(sc_num_candidates_);
    knnsearch_result.init(candidate_indexes.data(), out_dists_sqr.data());
    sc_manager_.polarcontext_tree_->index->findNeighbors(
        knnsearch_result, curr_key.data(), nanoflann::SearchParams(10));

    ALOG_INFO(MOD, "[ScanContext] KD-Tree search: num_candidates=%d", sc_num_candidates_);
    for (int i = 0; i < sc_num_candidates_; ++i) {
        ALOG_INFO(MOD, "[ScanContext] KD-Tree result[%d]: idx=%zu dist_sq=%.4f",
                  i, candidate_indexes[i], out_dists_sqr[i]);
    }

    // 对每个候选计算 SC 距离
    double min_dist = 10000000;
    std::vector<std::pair<int, double>> sc_dist_list;  // (db_index, sc_dist)

    ALOG_INFO(MOD, "[ScanContext] Computing SC distance for %d candidates...", sc_num_candidates_);

    for (int i = 0; i < sc_num_candidates_; ++i) {
        int cand_idx = static_cast<int>(candidate_indexes[i]);

        // cand_idx 是 KD-Tree 搜索空间中的索引，对应 polarcontext_invkeys_to_search_ 的索引
        // 需要映射到完整的 polarcontexts_ 索引
        // KD-Tree 搜索空间是 [0, end_idx)，所以完整索引就是 cand_idx

        if (cand_idx < 0 || cand_idx >= static_cast<int>(sc_manager_.polarcontexts_.size()) - 1) {
            ALOG_WARN(MOD, "[ScanContext] Skip invalid candidate idx=%d (max=%d)",
                      cand_idx, static_cast<int>(sc_manager_.polarcontexts_.size()) - 1);
            continue;
        }

        const auto& cand_sc = sc_manager_.polarcontexts_[cand_idx];
        auto sc_dist_result = sc_manager_.distanceBtnScanContext(curr_desc, cand_sc);
        double sc_dist = sc_dist_result.first;
        int align_shift = sc_dist_result.second;

        ALOG_INFO(MOD, "[ScanContext] Candidate[%d]: sc_idx=%d sc_dist=%.4f align_shift=%d threshold=%.3f",
                  i, cand_idx, sc_dist, align_shift, sc_dist_threshold_);

        sc_dist_list.push_back({cand_idx, sc_dist});

        if (sc_dist < min_dist) {
            min_dist = sc_dist;
        }
    }

    // 按距离排序，返回通过阈值的候选
    std::sort(sc_dist_list.begin(), sc_dist_list.end(),
               [](const auto& a, const auto& b) { return a.second < b.second; });

    ALOG_INFO(MOD, "[ScanContext] After sorting: best_dist=%.4f threshold=%.3f", min_dist, sc_dist_threshold_);

    // 转换为 Candidate 格式
    // 注意：sc_dist 越小，相似度越高
    for (const auto& dist_pair : sc_dist_list) {
        int sc_idx = dist_pair.first;  // SCManager 中的索引
        double sc_dist = dist_pair.second;

        if (sc_dist >= sc_dist_threshold_) {
            continue;
        }

        // 从 SC 索引映射到 submap key，再从 db_copy 找到同 key 子图（不再依赖 db_copy[sc_idx] 隐式对齐）
        if (sc_idx >= 0 && sc_idx < static_cast<int>(sc_index_to_submap_.size())) {
            const auto& target_key = sc_index_to_submap_[sc_idx];
            SubMap::Ptr matched_submap;
            for (const auto& sm : db_copy) {
                if (!sm) continue;
                if (makeSubmapKey(sm->session_id, sm->id) == target_key) {
                    matched_submap = sm;
                    break;
                }
            }
            if (!matched_submap) {
                ALOG_WARN(MOD, "[ScanContext] sc_idx=%d key=%s not found in db_copy, skip",
                          sc_idx, target_key.c_str());
                continue;
            }

            // 过滤：排除同一子图
            if (matched_submap->id == submap->id && matched_submap->session_id == submap->session_id) {
                continue;
            }

            // 计算相似度分数（距离越小，相似度越高）
            // clamp 到 [0, 1] 范围
            float raw_score = 1.0f - static_cast<float>(sc_dist / sc_dist_threshold_);
            float score = std::max(0.0f, std::min(1.0f, raw_score));

            OverlapTransformerInfer::Candidate cand;
            cand.submap_id = matched_submap->id;
            cand.session_id = matched_submap->session_id;
            cand.score = score;
            candidates.push_back(cand);

            ALOG_INFO(MOD, "[ScanContext] Candidate: query_id=%d target_id=%d (submap_id=%d) sc_dist=%.4f score=%.3f",
                      submap->id, sc_idx, matched_submap->id, sc_dist, score);
        }
    }

    if (candidates.empty()) {
        ALOG_INFO(MOD, "[ScanContext][END] No loop candidates found for submap_id=%d (best_dist=%.4f threshold=%.3f)",
                  submap->id, min_dist, sc_dist_threshold_);
    } else {
        ALOG_INFO(MOD, "[ScanContext][END] Found %zu loop candidates for submap_id=%d",
                  candidates.size(), submap->id);
    }

    return candidates;
}

} // namespace automap_pro
