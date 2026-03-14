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
#include <future>

namespace automap_pro {

LoopDetector::LoopDetector() {
    const auto& cfg = ConfigManager::instance();
    overlap_threshold_ = cfg.overlapThreshold();
    top_k_             = cfg.loopTopK();
    min_temporal_gap_  = cfg.loopMinTemporalGap();
    min_submap_gap_    = cfg.loopMinSubmapGap();
    gps_search_radius_ = cfg.gpsSearchRadius();
    geo_prefilter_max_distance_m_ = cfg.loopGeoPrefilterMaxDistanceM();
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

    // ScanContext 参数
    use_scancontext_ = cfg.scancontextEnabled();
    sc_dist_threshold_ = cfg.scancontextDistThreshold();
    sc_num_candidates_ = cfg.scancontextNumCandidates();
    sc_exclude_recent_ = cfg.scancontextExcludeRecent();
    sc_tree_making_period_ = cfg.scancontextTreeMakingPeriod();
}

LoopDetector::~LoopDetector() { stop(); }

void LoopDetector::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();

    RCLCPP_INFO(node_->get_logger(),
        "[LoopDetector][CONFIG] min_submap_gap=%d (from loop_closure.min_submap_gap; intra-submap candidates not filtered by gap)",
        min_submap_gap_);

    // 加载 OverlapTransformer 模型（LibTorch Level 1）
    std::string model_path = cfg.overlapModelPath();
    if (!model_path.empty()) {
        overlap_infer_.loadModel(model_path, true);
    }

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
        ALOG_INFO(MOD, "[ScanContext] Enabled: dist_threshold=%.3f num_candidates=%d exclude_recent=%d",
                  sc_dist_threshold_, sc_num_candidates_, sc_exclude_recent_);
    }

    constraint_pub_ = node->create_publisher<automap_pro::msg::LoopConstraintMsg>(
        "/automap/loop_constraint", 100);
    RCLCPP_INFO(node->get_logger(), "[LoopDetector][TOPIC] publish: /automap/loop_constraint");

    RCLCPP_INFO(node->get_logger(),
        "[LoopDetector] Initialized (workers=%d, OT=%s, TEASER=%s)",
        worker_thread_num_,
        overlap_infer_.isModelLoaded() ? "LibTorch" : "fallback",
#ifdef USE_TEASER
        "enabled"
#else
        "disabled"
#endif
    );
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
        ALOG_WARN(MOD, "addSubmap: submap #%d has empty cloud, skipped for loop detection", submap->id);
        return;
    }
    if (submap->keyframes.empty()) {
        ALOG_WARN(MOD, "addSubmap: submap #%d has no keyframes, skipped for loop detection", submap->id);
        return;
    }

    float priority = static_cast<float>(submap->id);
    size_t qsize;
    size_t dbsize;
    {
        std::lock_guard<std::mutex> lk(desc_mutex_);
        const size_t max_desc = ConfigManager::instance().loopMaxDescQueueSize();
        while (desc_queue_.size() >= max_desc) desc_queue_.pop();  // 丢弃最低优先级，防无界堆积
        desc_queue_.push({submap, priority});
        qsize = desc_queue_.size();
        dbsize = dbSize();
    }
    desc_cv_.notify_one();
    // RCLCPP_INFO(node_->get_logger(),
        // "[LoopDetector][DATA] addSubmap sm_id=%d kf=%zu desc_pts=%zu queue=%zu db=%zu",
        // submap->id, submap->keyframes.size(),
        // submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u, qsize, dbsize);
    ALOG_DEBUG(MOD, "SubMap#{} submitted for loop detection (queue_size={}, db_size={})",
               submap->id, qsize, dbsize);
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

    // ✅ 在存储描述子时更新 norm 缓存
    submap->overlap_descriptor = overlap_infer_.computeDescriptor(
        submap->downsampled_cloud);
    submap->overlap_descriptor_norm = submap->overlap_descriptor.norm();
    submap->has_descriptor = true;
    onDescriptorReady(submap);
    }
}

void LoopDetector::computeDescriptorAsync(const SubMap::Ptr& submap) {
    if (!submap->downsampled_cloud || submap->downsampled_cloud->empty()) return;

    // Level 1: LibTorch 进程内推理
    if (overlap_infer_.isModelLoaded()) {
        submap->overlap_descriptor = overlap_infer_.computeDescriptor(
            submap->downsampled_cloud);
        submap->has_descriptor = true;
        onDescriptorReady(submap);
        return;
    }

    // Level 2: 外部 Python Service（异步回调，不阻塞 Worker 线程）
#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    if (use_external_desc_service_ && desc_client_) {
        auto req = std::make_shared<overlap_transformer_msgs::srv::ComputeDescriptor::Request>();
        pcl::toROSMsg(*submap->downsampled_cloud, req->pointcloud);
        req->pointcloud.header.stamp    = node_->now();
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
                    RCLCPP_WARN(node_->get_logger(),
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
    submap->overlap_descriptor = overlap_infer_.computeDescriptor(
        submap->downsampled_cloud);
    submap->has_descriptor = true;
    onDescriptorReady(submap);
}

void LoopDetector::onDescriptorReady(const SubMap::Ptr& submap) {
    auto t_start = std::chrono::steady_clock::now();
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=descriptor_done submap_id={} query_pts={} (描述子就绪，进入候选检索)",
              submap->id, submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u);
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[LOOP_PHASE] stage=descriptor_done submap_id=%d query_pts=%zu",
                    submap->id, submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u);
    }
    // 检索候选（Stage 1）
    std::vector<SubMap::Ptr> db_copy;
    {
        std::shared_lock<std::shared_mutex> lk(db_mutex_);
        db_copy = db_submaps_;
    }

    auto t_retrieve_start = std::chrono::steady_clock::now();
    std::vector<OverlapTransformerInfer::Candidate> candidates;

    // 根据配置选择候选检索方法
    if (use_scancontext_) {
        // 使用 ScanContext 检索候选（需要加锁保护）
        std::lock_guard<std::mutex> lk(sc_mutex_);
        candidates = retrieveUsingScanContext(submap, db_copy);
    } else {
        // 使用 OverlapTransformer 检索候选
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
    auto t_retrieve_end = std::chrono::steady_clock::now();
    double retrieve_ms = std::chrono::duration<double, std::milli>(t_retrieve_end - t_retrieve_start).count();

    // ✅ 详细诊断日志
    if (candidates.empty()) {
        ALOG_INFO(MOD, "[LoopDetector][NO_CAND] query_id={} db_size={} 无重叠候选 (threshold={:.3f} top_k={})。"
                  " 若轨迹有闭环可尝试降低 loop_closure.overlap_threshold",
                  submap->id, db_copy.size(), overlap_threshold_, top_k_);
        return;
    }

    // 过滤候选（时间/子图间隔）：子图内回环（query 与 candidate 同 submap_id）不按 min_submap_gap 过滤，仅子图间过滤
    std::vector<OverlapTransformerInfer::Candidate> valid_candidates;
    int filtered_by_gap = 0;
    int same_session_count = 0;
    int same_submap_count = 0;
    int different_submap_count = 0;
    for (const auto& cand : candidates) {
        if (cand.session_id != submap->session_id) {
            // 跨会话
            valid_candidates.push_back(cand);
            continue;
        }
        if (cand.submap_id == submap->id) {
            // 子图内回环 - 允许
            same_submap_count++;
            valid_candidates.push_back(cand);
            continue;
        }
        // 同会话不同子图
        same_session_count++;
        if (std::abs(cand.submap_id - submap->id) <= min_submap_gap_) {
            filtered_by_gap++;
            continue;
        }
        different_submap_count++;
        valid_candidates.push_back(cand);
    }

    // 🔧 添加详细诊断日志
    ALOG_INFO(MOD, "[LoopDetector][CAND_STATS] query_id=%d: candidates=%d, same_session=%d, same_submap=%d, diff_submap=%d, filtered_by_gap=%d",
        submap->id, candidates.size(), same_session_count, same_submap_count, different_submap_count, filtered_by_gap);
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[LoopDetector][CAND_STATS] query_id=%d: candidates=%d, same_session=%d, same_submap=%d, diff_submap=%d, filtered_by_gap=%d",
            submap->id, candidates.size(), same_session_count, same_submap_count, different_submap_count, filtered_by_gap);
    }

    if (valid_candidates.empty()) {
        ALOG_WARN(MOD, "[LoopDetector][GAP_FILTER] query_id={} 候选数={} 均因 min_submap_gap={} 被过滤（子图内候选已放行）→ 无回环候选。"
                  " 可尝试将 loop_closure.min_submap_gap 调小或确保轨迹有闭环",
                  submap->id, candidates.size(), min_submap_gap_);
        return;
    }

    // 【几何距离预筛】按两子图锚定位姿距离过滤，抑制重复结构导致的误检（相似但远距离的候选）
    Eigen::Vector3d query_pos = submap->pose_w_anchor.translation();
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
            geo_dist_m = (query_pos - target_submap->pose_w_anchor.translation()).norm();
            ALOG_INFO(MOD, "[LOOP_CAND] query_id={} target_id={} score={:.3f} geo_dist={:.1f}m",
                      submap->id, cand.submap_id, cand.score, geo_dist_m);
        } else {
            ALOG_INFO(MOD, "[LOOP_CAND] query_id={} target_id={} score={:.3f} geo_dist=N/A (target not in db_copy)",
                      submap->id, cand.submap_id, cand.score);
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
        ALOG_WARN(MOD, "[LoopDetector][GEO_PREFILTER] query_id={} 候选数={} 均因几何距离 > {:.1f}m 被过滤 → 无回环候选。"
                  " 可调大 loop_closure.geo_prefilter_max_distance_m 或设为 0 关闭",
                  submap->id, valid_candidates.size(), geo_prefilter_max_distance_m_);
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
    auto t_end = std::chrono::steady_clock::now();
    double total_stage1_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    
    ALOG_INFO(MOD, "[LoopDetector][CAND] query_id={} db={} candidates={} scores=[{}] retrieve_ms={:.1f} → TEASER++",
              submap->id, db_copy.size(), geo_filtered_candidates.size(), score_str, retrieve_ms);
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=candidates_retrieved query_id={} candidate_count={} retrieve_ms={:.1f} (精准优化: 若无候选可调 overlap_threshold/min_submap_gap/geo_prefilter)",
              submap->id, geo_filtered_candidates.size(), retrieve_ms);
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[LOOP_PHASE] stage=candidates_retrieved query_id=%d candidate_count=%zu retrieve_ms=%.1f",
                    submap->id, geo_filtered_candidates.size(), retrieve_ms);
    }
    // 入队时深拷贝 query 点云，避免 worker 处理时读 SubMap::downsampled_cloud 与 buildGlobalMap 等并发
    CloudXYZIPtr query_cloud_copy;
    if (submap->downsampled_cloud && !submap->downsampled_cloud->empty()) {
        query_cloud_copy = std::make_shared<CloudXYZI>();
        *query_cloud_copy = *submap->downsampled_cloud;
        ALOG_DEBUG(MOD, "enqueue MatchTask query_id={} query_cloud_pts={} candidates={}",
                   submap->id, query_cloud_copy->size(), geo_filtered_candidates.size());
    }

    // 提交到 TEASER++ 匹配队列（Stage 2）
    {
        std::lock_guard<std::mutex> lk(match_mutex_);
        const size_t max_match = ConfigManager::instance().loopMaxMatchQueueSize();
        while (match_queue_.size() >= max_match) match_queue_.pop();  // 丢弃最旧任务，防无界堆积
        match_queue_.push({submap, query_cloud_copy, geo_filtered_candidates});
    }
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=match_enqueue query_id={} candidates={} query_pts={} (精准优化: grep LOOP_COMPUTE 查看每次 TEASER 的 inliers/corrs/reason)",
              submap->id, geo_filtered_candidates.size(), query_cloud_copy ? query_cloud_copy->size() : 0u);
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[LOOP_PHASE] stage=match_enqueue query_id=%d candidates=%zu query_pts=%zu",
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
        ALOG_DEBUG(MOD, "[tid={}] step=match_dequeue query_id={} candidates={}",
                   automap_pro::logThreadId(), task.query ? task.query->id : -1, task.candidates.size());
        
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

    // 添加任务有效性检查
    if (!task.query) {
        ALOG_WARN(MOD, "[tid=%u] processMatchTask: null query submap, skipping", tid);
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

    ALOG_DEBUG(MOD, "[tid=%u] step=task_start query_id=%d query_pts=%zu candidates=%zu",
               tid, query ? query->id : -1, query_cloud->size(), task.candidates.size());
    ALOG_INFO(MOD, "[LOOP_PHASE] stage=geom_verify_enter query_id={} query_pts={} candidates={} (精准优化: 下方每个候选会输出 LOOP_COMPUTE 与 TEASER 详情)",
              query->id, query_cloud->size(), task.candidates.size());
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[LOOP_PHASE] stage=geom_verify_enter query_id=%d query_pts=%zu candidates=%zu",
                    query->id, query_cloud->size(), task.candidates.size());
    }
    AUTOMAP_TIMED_SCOPE(MOD, fmt::format("GeomVerify SM#{}", task.query->id), 3000.0);

    // 过滤掉无效候选
    std::vector<OverlapTransformerInfer::Candidate> valid_candidates;
    for (const auto& cand : task.candidates) {
        if (cand.score < overlap_threshold_) {
            continue;
        }
        // 检查候选子图ID有效性
        if (cand.submap_id < 0) {
            continue;
        }
        valid_candidates.push_back(cand);
    }

    if (valid_candidates.empty()) {
        ALOG_DEBUG(MOD, "[tid=%u] No valid candidates after filtering for SM#%d",
                   tid, query->id);
        return;
    }

    // P1: 并行 TEASER 多候选匹配（performance.parallel_teaser_match=true）
    if (ConfigManager::instance().parallelTeaserMatch() && !valid_candidates.empty()) {
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
            std::vector<std::future<TeaserMatcher::Result>> futures;
            for (CandWork& w : works) {
                CloudXYZIPtr q = query_cloud;
                CloudXYZIPtr t = w.target_cloud;
                futures.push_back(std::async(std::launch::async, [this, q, t]() {
                    return teaser_matcher_.match(q, t, Pose3d::Identity());
                }));
            }
            IcpRefiner icp_par;
            for (size_t i = 0; i < works.size(); ++i) {
                TeaserMatcher::Result teaser_res;
                try {
                    teaser_res = futures[i].get();
                } catch (const std::exception& e) {
                    ALOG_ERROR(MOD, "[tid={}] parallel_teaser candidate {} exception: {}", tid, works[i].cand.submap_id, e.what());
                    ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=exception reason=teaser_exception what={} (parallel)",
                              query->id, works[i].cand.submap_id, e.what());
                    continue;
                } catch (...) {
                    ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=exception reason=teaser_unknown (parallel)",
                              query->id, works[i].cand.submap_id);
                    continue;
                }
                // 修复: 确保num_correspondences非负，避免负值导致的问题
                int inliers_approx = static_cast<int>(std::round(teaser_res.inlier_ratio * std::max(0, std::max(0, teaser_res.num_correspondences))));
                const char* reason_str = (teaser_res.success && teaser_res.inlier_ratio >= min_inlier_ratio_) ? "ok" : "teaser_fail_or_inlier_low";
                ALOG_INFO(MOD, "[LOOP_COMPUTE] query_id={} target_id={} result=done success={} inliers≈{} corrs={} inlier_ratio={:.3f} rmse={:.4f} reason={} (parallel)",
                          query->id, works[i].cand.submap_id, teaser_res.success, inliers_approx, teaser_res.num_correspondences,
                          teaser_res.inlier_ratio, teaser_res.rmse, reason_str);
                if (node_) {
                    RCLCPP_INFO(node_->get_logger(), "[LOOP_COMPUTE] query_id=%d target_id=%d success=%d inliers=%d corrs=%d inlier_ratio=%.3f reason=%s (parallel)",
                                query->id, works[i].cand.submap_id, teaser_res.success ? 1 : 0, inliers_approx, teaser_res.num_correspondences, teaser_res.inlier_ratio, reason_str);
                }
                if (!teaser_res.success || teaser_res.inlier_ratio < min_inlier_ratio_) {
                    ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=teaser_fail_or_inlier_low success={} inlier_ratio={:.3f} min_ratio={:.3f} score={:.3f} (parallel)",
                              query->id, works[i].cand.submap_id, teaser_res.success, teaser_res.inlier_ratio, min_inlier_ratio_, works[i].cand.score);
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
                    ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=rmse_too_high rmse={:.4f} max_rmse={:.4f} (parallel)",
                              query->id, target->id, final_rmse, max_rmse_);
                    continue;
                }
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
                double info_scale = lc->inlier_ratio / (lc->rmse + 1e-3f);
                // 限制最大信息尺度，避免数值过大导致优化不稳定
                info_scale = std::min(info_scale, 1e6);
                Mat66d information = Mat66d::Identity();
                information.block<3,3>(0,0) *= info_scale * 0.1;  // 平移自由度权重较小
                information.block<3,3>(3,3) *= info_scale;        // 旋转自由度权重较大
                lc->information = information;
                ALOG_INFO(MOD, "[LoopDetector][LOOP_OK] 回环约束已发布(parallel) query_id={} target_id={} inlier={:.3f} rmse={:.4f}m",
                          query->id, target->id, lc->inlier_ratio, lc->rmse);
                if (node_) {
                    RCLCPP_INFO(node_->get_logger(), "[LOOP_ACCEPTED] query_id=%d target_id=%d inlier_ratio=%.3f rmse=%.4fm (grep LOOP_ACCEPTED to verify loop constraints)",
                                query->id, target->id, lc->inlier_ratio, lc->rmse);
                    RCLCPP_INFO(node_->get_logger(), "[LoopDetector] Loop %d↔%d | inlier=%.2f rmse=%.3f", lc->submap_i, lc->submap_j, lc->inlier_ratio, lc->rmse);
                }
                publishLoopConstraint(lc);
                loop_detected_count_++;
                for (auto& cb : loop_cbs_) cb(lc);
            }
        }
        return;
    }

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
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "[LOOP_COMPUTE] query_id=%d target_id=%d query_pts=%zu target_pts=%zu overlap_score=%.3f",
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
            ALOG_ERROR(MOD, "[tid={}] step=match_exception query_id={} target_id={} exception={}",
                      tid, query->id, target->id, e.what());
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=teaser_exception what={} score={:.3f}",
                      query->id, target->id, e.what(), cand.score);
            continue;
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=match_exception query_id={} target_id={} unknown_exception",
                      tid, query->id, target->id);
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=teaser_unknown_exception score={:.3f}",
                      query->id, target->id, cand.score);
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
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "[LOOP_COMPUTE] query_id=%d target_id=%d success=%d inliers=%d corrs=%d inlier_ratio=%.3f rmse=%.4f reason=%s",
                        query->id, target->id, teaser_res.success ? 1 : 0, inliers_approx, teaser_res.num_correspondences,
                        teaser_res.inlier_ratio, teaser_res.rmse, reason_str);
        }
        ALOG_DEBUG(MOD, "[tid={}] step=teaser_result query_id={} target_id={} success={} inlier={:.2f} rmse={:.3f}m ms={:.1f}",
                   tid, query->id, target->id, teaser_res.success, teaser_res.inlier_ratio, teaser_res.rmse, teaser_ms);

        if (!teaser_res.success || teaser_res.inlier_ratio < min_inlier_ratio_) {
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=teaser_fail_or_inlier_low success={} inlier_ratio={:.3f} min_ratio={:.3f} rmse={:.4f} score={:.3f}",
                      query->id, target->id, teaser_res.success, teaser_res.inlier_ratio, min_inlier_ratio_, teaser_res.rmse, cand.score);
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
            ALOG_INFO(MOD, "[LOOP_REJECTED] query_id={} target_id={} reason=rmse_too_high rmse={:.4f} max_rmse={:.4f} inlier_ratio={:.3f} score={:.3f}",
                      query->id, target->id, final_rmse, max_rmse_, teaser_res.inlier_ratio, cand.score);
            continue;
        }

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
        double info_scale = lc->inlier_ratio / (lc->rmse + 1e-3f);
        // 限制最大信息尺度，避免数值过大导致优化不稳定
        info_scale = std::min(info_scale, 1e6);
        Mat66d information = Mat66d::Identity();
        information.block<3,3>(0,0) *= info_scale * 0.1;  // 平移自由度权重较小
        information.block<3,3>(3,3) *= info_scale;        // 旋转自由度权重较大
        lc->information = information;

        ALOG_INFO(MOD, "[LoopDetector][LOOP_OK] 回环约束已发布 query_id={} target_id={} inlier={:.3f} rmse={:.4f}m score={:.3f}",
                  query->id, target->id, lc->inlier_ratio, lc->rmse, lc->overlap_score);
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "[LOOP_ACCEPTED] query_id=%d target_id=%d inlier_ratio=%.3f rmse=%.4fm (grep LOOP_ACCEPTED to verify loop constraints)",
                        query->id, target->id, lc->inlier_ratio, lc->rmse);
            RCLCPP_INFO(node_->get_logger(),
                "[LoopDetector] Loop %d↔%d | inlier=%.2f rmse=%.3f score=%.2f",
                lc->submap_i, lc->submap_j, lc->inlier_ratio, lc->rmse, lc->overlap_score);
        }

        // 发布 ROS2 消息
        publishLoopConstraint(lc);

        loop_detected_count_++;
        // 触发回调（→ IncrementalOptimizer）
        for (auto& cb : loop_cbs_) cb(lc);
    }
}

void LoopDetector::addToDatabase(const SubMap::Ptr& submap) {
    std::unique_lock<std::shared_mutex> lk(db_mutex_);
    // 避免重复添加
    for (const auto& sm : db_submaps_) {
        if (sm->id == submap->id && sm->session_id == submap->session_id) return;
    }
    db_submaps_.push_back(submap);
}

void LoopDetector::clearCurrentSessionDB() {
    std::unique_lock<std::shared_mutex> lk(db_mutex_);
    uint64_t cur_session = db_submaps_.empty() ? 0 : db_submaps_.back()->session_id;
    db_submaps_.erase(
        std::remove_if(db_submaps_.begin(), db_submaps_.end(),
            [cur_session](const SubMap::Ptr& sm) { return sm->session_id == cur_session; }),
        db_submaps_.end());
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
    automap_pro::msg::LoopConstraintMsg msg;
    msg.header.stamp  = node_ ? node_->now() : rclcpp::Clock().now();
    msg.submap_i      = lc->submap_i;
    msg.submap_j      = lc->submap_j;
    msg.session_i     = lc->session_i;
    msg.session_j     = lc->session_j;
    msg.overlap_score = lc->overlap_score;
    msg.inlier_ratio  = lc->inlier_ratio;
    msg.rmse          = lc->rmse;
    msg.is_inter_session = lc->is_inter_session;
    // 信息矩阵扁平化
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            msg.information_matrix[i*6+j] = lc->information(i,j);
    constraint_pub_->publish(msg);
}

void LoopDetector::prepareIntraSubmapDescriptors(const SubMap::Ptr& submap) {
    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][PREPARE] 开始准备子图内描述子数据库
    // 使用 fallback 描述子（深度直方图），不依赖模型
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][PREPARE] ====== START ====== submap_id={} keyframes={} model_loaded={} (using fallback descriptor)",
        submap ? submap->id : -1,
        submap ? submap->keyframes.size() : size_t(0),
        overlap_infer_.isModelLoaded());

    if (!submap) {
        ALOG_ERROR(MOD, "[INTRA_LOOP][PREPARE] ERROR: submap is null");
        return;
    }

    if (submap->keyframes.empty()) {
        ALOG_WARN(MOD, "[INTRA_LOOP][PREPARE] WARN: submap_id={} has no keyframes, skip", submap->id);
        return;
    }

    // 不再检查模型加载状态，直接使用 computeDescriptor（会自动 fallback）
    // 如果已有数据，跳过重新计算
    if (submap->keyframe_descriptors.size() == submap->keyframes.size()) {
        ALOG_INFO(MOD, "[INTRA_LOOP][PREPARE] descriptors already ready, skip");
        return;
    }

    // 清空并重新计算
    submap->keyframe_descriptors.clear();
    submap->keyframe_clouds_ds.clear();
    submap->keyframe_descriptors.reserve(submap->keyframes.size());
    submap->keyframe_clouds_ds.reserve(submap->keyframes.size());

    int success_count = 0;
    int null_kf_skipped = 0;
    int null_cloud_skipped = 0;

    for (size_t i = 0; i < submap->keyframes.size(); ++i) {
        const auto& kf = submap->keyframes[i];
        if (!kf) {
            null_kf_skipped++;
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
            submap->keyframe_clouds_ds.push_back(nullptr);
            ALOG_WARN(MOD, "[INTRA_LOOP][PREPARE] WARN: keyframe {} is null, skipped", i);
            continue;
        }

        if (!kf->cloud_body) {
            null_cloud_skipped++;
            submap->keyframe_descriptors.push_back(Eigen::VectorXf::Zero(256));
            submap->keyframe_clouds_ds.push_back(nullptr);
            ALOG_WARN(MOD, "[INTRA_LOOP][PREPARE] WARN: keyframe {} (id={}) has no cloud_body, skipped",
                      i, kf->id);
            continue;
        }

        // 下采样点云
        CloudXYZIPtr cloud_ds = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(0.5f, 0.5f, 0.5f);
        voxel.setInputCloud(kf->cloud_body);
        voxel.filter(*cloud_ds);

        size_t pts_before = kf->cloud_body->size();
        size_t pts_after = cloud_ds->size();
        float downsample_ratio = pts_before > 0 ? (float)pts_after / pts_before : 0.0f;

        // 计算描述子
        auto t0 = std::chrono::steady_clock::now();
        Eigen::VectorXf desc = overlap_infer_.computeDescriptor(cloud_ds);
        auto t1 = std::chrono::steady_clock::now();
        float infer_ms = std::chrono::duration<float, std::milli>(t1 - t0).count();

        float desc_norm = desc.norm();
        submap->keyframe_descriptors.push_back(desc);
        submap->keyframe_clouds_ds.push_back(cloud_ds);

        success_count++;
        ALOG_INFO(MOD,
            "[INTRA_LOOP][PREPARE] kf_idx={} kf_id={} pts_before={} pts_after={} "
            "downsample_ratio={:.2f} desc_norm={:.4f} infer_ms={:.2f}",
            i, kf->id, pts_before, pts_after, downsample_ratio, desc_norm, infer_ms);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][PREPARE] 完成统计
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][PREPARE] ====== DONE ====== "
        "submap_id={} total_kf={} success={} null_kf={} null_cloud={} "
        "desc_db_size={} clouds_db_size={}",
        submap->id, submap->keyframes.size(),
        success_count, null_kf_skipped, null_cloud_skipped,
        submap->keyframe_descriptors.size(), submap->keyframe_clouds_ds.size());
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
        "submap_id=%d query_idx=%d kf_count=%zu "
        "min_temporal_gap=%.1fs min_keyframe_gap=%d min_dist_gap=%.1fm overlap_thresh=%.3f "
        "min_inlier_ratio=%.3f max_rmse=%.3f",
        submap ? submap->id : -1, query_keyframe_idx,
        submap ? submap->keyframes.size() : 0,
        intra_submap_min_temporal_gap_, intra_submap_min_keyframe_gap_,
        intra_submap_min_distance_gap_,
        intra_submap_overlap_threshold_,
        min_inlier_ratio_, max_rmse_);

    if (!submap || query_keyframe_idx < 0 ||
        static_cast<size_t>(query_keyframe_idx) >= submap->keyframes.size()) {
        ALOG_ERROR(MOD,
            "[INTRA_LOOP][ERROR] INVALID_INPUT: submap_id=%d query_idx=%d kf_size=%zu "
            "(reason: %s)",
            submap ? submap->id : -1, query_keyframe_idx,
            submap ? submap->keyframes.size() : 0,
            !submap ? "submap_null" :
            query_keyframe_idx < 0 ? "negative_index" :
            "index_out_of_range");
        return results;
    }

    const auto& query_kf = submap->keyframes[query_keyframe_idx];
    if (!query_kf) {
        ALOG_ERROR(MOD, "[INTRA_LOOP][ERROR] query keyframe is null at idx=%d", query_keyframe_idx);
        return results;
    }

    if (!query_kf->cloud_body) {
        ALOG_ERROR(MOD, "[INTRA_LOOP][ERROR] query keyframe has no cloud: kf_id=%lu idx=%d pts=0",
                   query_kf->id, query_keyframe_idx);
        return results;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][DEBUG] 描述子数据库状态检查
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][DEBUG] descriptor_db_status: stored_size=%zu expected_size=%zu match=%d",
        submap->keyframe_descriptors.size(), submap->keyframes.size(),
        submap->keyframe_descriptors.size() == submap->keyframes.size());

    // 确保描述子已准备好（调用 prepare 使用 fallback 计算）
    if (submap->keyframe_descriptors.size() != submap->keyframes.size()) {
        ALOG_WARN(MOD,
            "[INTRA_LOOP][WARN] descriptor_size_mismatch: stored=%zu expected=%zu, preparing now...",
            submap->keyframe_descriptors.size(), submap->keyframes.size());
        prepareIntraSubmapDescriptors(submap);
    }

    // prepare 后再次检查（理论上 fallback 都能成功计算）
    if (submap->keyframe_descriptors.empty() ||
        submap->keyframe_descriptors.size() != submap->keyframes.size()) {
        ALOG_WARN(MOD,
            "[INTRA_LOOP][WARN] skip intra-loop: descriptors not ready (size=%zu, expected=%zu)",
            submap->keyframe_descriptors.size(), submap->keyframes.size());
        return results;
    }

    const auto& query_desc = submap->keyframe_descriptors[query_keyframe_idx];
    float query_desc_norm = query_desc.norm();
    ALOG_INFO(MOD,
        "[INTRA_LOOP][DEBUG] query_descriptor: kf_idx=%d kf_id=%lu norm=%.6f is_zero=%d pts=%zu",
        query_keyframe_idx, query_kf->id, query_desc_norm,
        query_desc_norm < 1e-6f,
        query_kf->cloud_body->size());

    if (query_desc_norm < 1e-6f) {
        ALOG_ERROR(MOD,
            "[INTRA_LOOP][ERROR] query descriptor is zero: kf_idx=%d norm=%.6f (skip)",
            query_keyframe_idx, query_desc_norm);
        return results;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][INFO] 开始检索历史关键帧
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][INFO] START_SEARCH: submap_id=%d query_kf_idx=%d query_ts=%.6f "
        "query_pos=[%.2f,%.2f,%.2f] history_kf_count=%zu",
        submap->id, query_keyframe_idx, query_kf->timestamp,
        query_kf->T_w_b.translation().x(),
        query_kf->T_w_b.translation().y(),
        query_kf->T_w_b.translation().z(),
        submap->keyframes.size());

    // 检索历史关键帧（同一子图内）
    int candidates_found = 0;
    int temporal_filtered = 0;
    int index_filtered = 0;
    int desc_filtered = 0;
    int null_cand_skipped = 0;
    int empty_cloud_skipped = 0;
    int teaser_failed = 0;
    int teaser_rejected_inlier = 0;
    int teaser_rejected_rmse = 0;
    int teaser_invoked = 0;  // 本帧已调用 TEASER 次数，超过上限则停止（避免 10s+ 卡住）

    for (int i = 0; i < query_keyframe_idx; ++i) {
        if (intra_submap_max_teaser_candidates_ > 0 && teaser_invoked >= intra_submap_max_teaser_candidates_) {
            ALOG_DEBUG(MOD, "[INTRA_LOOP][CAP] TEASER invoked %d >= max %d, skip remaining candidates",
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
            ALOG_DEBUG(MOD,
                "[INTRA_LOOP][FILTER] TEMPORAL_GAP: query_idx=%d cand_idx=%d cand_ts=%.6f gap=%.2fs < %.1fs (SKIP)",
                query_keyframe_idx, i, cand_kf->timestamp, temporal_gap, intra_submap_min_temporal_gap_);
            continue;
        }

        // 索引间隔过滤（避免相邻帧）
        int index_gap = query_keyframe_idx - i;
        if (index_gap < intra_submap_min_keyframe_gap_) {
            index_filtered++;
            ALOG_DEBUG(MOD,
                "[INTRA_LOOP][FILTER] INDEX_GAP: query_idx=%d cand_idx=%d gap=%d < %d (SKIP)",
                query_keyframe_idx, i, index_gap, intra_submap_min_keyframe_gap_);
            continue;
        }

        // 距离间隔过滤（避免过密帧）
        if (intra_submap_min_distance_gap_ > 0.0) {
            const auto& query_pos = query_kf->T_w_b.translation();
            const auto& cand_pos = cand_kf->T_w_b.translation();
            double dist_gap = (query_pos - cand_pos).norm();
            if (dist_gap < intra_submap_min_distance_gap_) {
                ALOG_DEBUG(MOD,
                    "[INTRA_LOOP][FILTER] DISTANCE_GAP: query_idx=%d cand_idx=%d dist=%.2fm < %.1fm (SKIP)",
                    query_keyframe_idx, i, dist_gap, intra_submap_min_distance_gap_);
                continue;
            }
        }

        // 描述子相似度过滤
        const auto& cand_desc = submap->keyframe_descriptors[i];
        float cand_desc_norm = cand_desc.norm();
        if (cand_desc_norm < 1e-6f) {
            ALOG_DEBUG(MOD, "[INTRA_LOOP][FILTER] CAND_DESC_ZERO: cand_idx=%d norm=%.6f (SKIP)",
                       i, cand_desc_norm);
            continue;
        }

        // 修复: 添加除法安全检查，防止除零
        float denominator = query_desc_norm * cand_desc_norm;
        if (denominator < 1e-10f) {
            ALOG_DEBUG(MOD, "[INTRA_LOOP][FILTER] DENOMINATOR_ZERO: query_norm=%.6f cand_norm=%.6f (SKIP)",
                       query_desc_norm, cand_desc_norm);
            continue;
        }
        float similarity = query_desc.dot(cand_desc) / denominator;
        if (similarity < intra_submap_overlap_threshold_) {
            desc_filtered++;
            ALOG_DEBUG(MOD,
                "[INTRA_LOOP][FILTER] SIMILARITY: query_idx=%d cand_idx=%d sim=%.4f < %.3f (SKIP)",
                query_keyframe_idx, i, similarity, intra_submap_overlap_threshold_);
            continue;
        }

        candidates_found++;
        ALOG_INFO(MOD,
            "[INTRA_LOOP][CANDIDATE] FOUND: submap_id=%d query_idx=%d cand_idx=%d "
            "sim=%.4f gap_idx=%d gap_time=%.2fs cand_kf_id=%lu cand_pos=[%.2f,%.2f,%.2f]",
            submap->id, query_keyframe_idx, i, similarity, index_gap, temporal_gap, cand_kf->id,
            cand_kf->T_w_b.translation().x(),
            cand_kf->T_w_b.translation().y(),
            cand_kf->T_w_b.translation().z());

        // ═══════════════════════════════════════════════════════════════════════
        // [INTRA_LOOP][DEBUG] 几何验证（TEASER++）开始
        // ═══════════════════════════════════════════════════════════════════════
        const auto& query_cloud = submap->keyframe_clouds_ds[query_keyframe_idx];
        const auto& cand_cloud = submap->keyframe_clouds_ds[i];

        if (!query_cloud || !cand_cloud || query_cloud->empty() || cand_cloud->empty()) {
            empty_cloud_skipped++;
            ALOG_ERROR(MOD,
                "[INTRA_LOOP][ERROR] EMPTY_CLOUD: query_idx=%d cand_idx=%d "
                "query_valid=%s cand_valid=%s (SKIP)",
                query_keyframe_idx, i,
                query_cloud ? "valid" : "null",
                cand_cloud ? "valid" : "null");
            continue;
        }

        teaser_invoked++;
        ALOG_INFO(MOD,
            "[INTRA_LOOP][TEASER_START] query_idx=%d cand_idx=%d "
            "query_pts=%zu cand_pts=%zu invoked=%d",
            query_keyframe_idx, i, query_cloud->size(), cand_cloud->size(), teaser_invoked);

        // TEASER++ 配准
        TeaserMatcher::Result teaser_res;
        bool teaser_success = false;
        try {
            teaser_res = teaser_matcher_.match(query_cloud, cand_cloud);
            teaser_success = teaser_res.success;
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD,
                "[INTRA_LOOP][EXCEPTION] TEASER_EXCEPTION: query_idx=%d cand_idx=%d exception='%s'",
                query_keyframe_idx, i, e.what());
            teaser_failed++;
            continue;
        } catch (...) {
            ALOG_ERROR(MOD,
                "[INTRA_LOOP][EXCEPTION] TEASER_UNKNOWN_EXCEPTION: query_idx=%d cand_idx=%d",
                query_keyframe_idx, i);
            teaser_failed++;
            continue;
        }

        if (!teaser_success) {
            teaser_failed++;
            ALOG_WARN(MOD,
                "[INTRA_LOOP][TEASER] FAILED_RETURN: query_idx=%d cand_idx=%d success=false (SKIP)",
                query_keyframe_idx, i);
            continue;
        }

        if (!teaser_res.success) {
            teaser_failed++;
            ALOG_WARN(MOD,
                "[INTRA_LOOP][TEASER] INVALID_RESULT: query_idx=%d cand_idx=%d is_valid=false (SKIP)",
                query_keyframe_idx, i);
            continue;
        }

        // 检查 inlier ratio
        if (teaser_res.inlier_ratio < min_inlier_ratio_) {
            teaser_rejected_inlier++;
            ALOG_DEBUG(MOD,
                "[INTRA_LOOP][TEASER] REJECT_INLIER: query_idx=%d cand_idx=%d "
                "inlier_ratio=%.4f < %.3f (SKIP)",
                query_keyframe_idx, i, teaser_res.inlier_ratio, min_inlier_ratio_);
            continue;
        }

        // 检查 RMSE
        if (teaser_res.rmse > max_rmse_) {
            teaser_rejected_rmse++;
            ALOG_DEBUG(MOD,
                "[INTRA_LOOP][TEASER] REJECT_RMSE: query_idx=%d cand_idx=%d "
                "rmse=%.4f > %.3f (SKIP)",
                query_keyframe_idx, i, teaser_res.rmse, max_rmse_);
            continue;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // [INTRA_LOOP][INFO] 检测到回环！
        // ═══════════════════════════════════════════════════════════════════════
        const Eigen::Vector3d& t = teaser_res.T_tgt_src.translation();
        const Eigen::Matrix3d rot = teaser_res.T_tgt_src.rotation();
        const Eigen::Vector3d rpy = Eigen::Matrix3d(rot).eulerAngles(2, 1, 0).reverse();
        // 根据 inlier ratio 构造信息矩阵（TeaserMatcher::Result 没有 information 字段）
        // 修复: 使用更合理的信息矩阵计算，区分旋转和平移自由度
        double info_scale = teaser_res.inlier_ratio * 100.0;
        // 限制最大信息尺度，避免数值过大导致优化不稳定
        info_scale = std::min(info_scale, 1e6);
        Mat66d information = Mat66d::Identity();
        information.block<3,3>(0,0) *= info_scale * 0.1;  // 平移自由度权重较小
        information.block<3,3>(3,3) *= info_scale;        // 旋转自由度权重较大

        ALOG_INFO(MOD,
            "[INTRA_LOOP][DETECTED] ★★★ INTRA_SUBMAP_LOOP ★★★ "
            "submap_id=%d kf_i=%d kf_j=%d "
            "overlap=%.4f inlier=%.4f rmse=%.4f "
            "delta_t=[%.3f,%.3f,%.3f] delta_rpy=[%.2f,%.2f,%.2f] "
            "info_norm=%.2e",
            submap->id, i, query_keyframe_idx,
            similarity, teaser_res.inlier_ratio, teaser_res.rmse,
            t.x(), t.y(), t.z(),
            rpy.x() * 180.0 / M_PI, rpy.y() * 180.0 / M_PI, rpy.z() * 180.0 / M_PI,
            information.norm());

        // 创建回环约束
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
                ALOG_ERROR(MOD, "[INTRA_LOOP][CALLBACK] exception: %s", e.what());
            }
        }

        loop_detected_count_++;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // [INTRA_LOOP][SUMMARY] 统计信息
    // ═══════════════════════════════════════════════════════════════════════════
    ALOG_INFO(MOD,
        "[INTRA_LOOP][SUMMARY] ====== detectIntraSubmapLoop END ====== "
        "submap_id=%d query_kf_idx=%d "
        "total_history_kf=%zu "
        "candidates_found=%d "
        "filtered: null_cand=%d temporal=%d index=%d desc=%d "
        "teaser: failed=%d reject_inlier=%d reject_rmse=%d empty_cloud=%d "
        "FINAL_detected=%zu",
        submap->id, query_keyframe_idx,
        submap->keyframes.size(),
        candidates_found,
        null_cand_skipped, temporal_filtered, index_filtered, desc_filtered,
        teaser_failed, teaser_rejected_inlier, teaser_rejected_rmse, empty_cloud_skipped,
        results.size());

    return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// ScanContext 候选检索实现
// ─────────────────────────────────────────────────────────────────────────────
std::vector<OverlapTransformerInfer::Candidate> LoopDetector::retrieveUsingScanContext(
    const SubMap::Ptr& submap,
    const std::vector<SubMap::Ptr>& db_copy) {

    std::vector<OverlapTransformerInfer::Candidate> candidates;

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
    ALOG_INFO(MOD, "[ScanContext] SC generated: query_id=%d total_sc=%zu",
              submap->id, sc_manager_.polarcontexts_.size());

    // 检查 SCManager 中是否有足够的历史帧用于检索（排除最近的 exclude_recent 帧）
    int num_available_for_search = static_cast<int>(sc_manager_.polarcontexts_.size()) - sc_exclude_recent_;
    ALOG_INFO(MOD, "[ScanContext] Check history: total_sc=%zu exclude_recent=%d available=%d",
              sc_manager_.polarcontexts_.size(), sc_exclude_recent_, num_available_for_search);

    if (num_available_for_search <= 0) {
        ALOG_INFO(MOD, "[ScanContext] Not enough history frames for search: available=%d <= 0, skip",
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
        // 排除最近的 sc_exclude_recent_ 帧和当前帧（当前帧在末尾）
        size_t end_idx = sc_manager_.polarcontext_invkeys_mat_.size() - sc_exclude_recent_ - 1;
        ALOG_INFO(MOD, "[ScanContext] KD-Tree rebuild: key_mat_size=%zu exclude=%d end_idx=%zu",
                  sc_manager_.polarcontext_invkeys_mat_.size(), sc_exclude_recent_, end_idx);

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

        // 从 db_copy 中找到对应的子图
        // SCManager 中的索引 sc_idx 对应 db_copy[sc_idx]
        // 因为每次添加 SubMap 时都会同时添加 ScanContext
        if (sc_idx >= 0 && sc_idx < (int)db_copy.size()) {
            const auto& matched_submap = db_copy[sc_idx];

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
