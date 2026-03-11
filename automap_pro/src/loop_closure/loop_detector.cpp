#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "LoopDetector"

#include <automap_pro/msg/loop_constraint_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
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
}

LoopDetector::~LoopDetector() { stop(); }

void LoopDetector::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();

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
        req->pointcloud.header.frame_id = "body";

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
    
    // 先加入数据库
    addToDatabase(submap);

    // 检索候选（Stage 1）
    std::vector<SubMap::Ptr> db_copy;
    {
        std::shared_lock<std::shared_mutex> lk(db_mutex_);
        db_copy = db_submaps_;
    }

    auto t_retrieve_start = std::chrono::steady_clock::now();
    auto candidates = overlap_infer_.retrieve(
        submap->overlap_descriptor,
        db_copy,
        top_k_,
        static_cast<float>(overlap_threshold_),
        min_submap_gap_,
        min_temporal_gap_,
        gps_search_radius_,
        submap->gps_center,
        submap->has_valid_gps);
    auto t_retrieve_end = std::chrono::steady_clock::now();
    double retrieve_ms = std::chrono::duration<double, std::milli>(t_retrieve_end - t_retrieve_start).count();

    // ✅ 详细诊断日志
    if (candidates.empty()) {
        ALOG_INFO(MOD, "[LoopDetector][NO_CAND] query_id={} db_size={} 无重叠候选 (threshold={:.3f} top_k={})。"
                  " 若轨迹有闭环可尝试降低 loop_closure.overlap_threshold",
                  submap->id, db_copy.size(), overlap_threshold_, top_k_);
        return;
    }

    // 过滤候选（时间/子图间隔）
    std::vector<OverlapTransformerInfer::Candidate> valid_candidates;
    int filtered_by_gap = 0;
    for (const auto& cand : candidates) {
        if (cand.session_id == submap->session_id &&
            std::abs(cand.submap_id - submap->id) < min_submap_gap_) {
            filtered_by_gap++;
            continue;
        }
        valid_candidates.push_back(cand);
    }
    
    if (valid_candidates.empty()) {
        ALOG_WARN(MOD, "[LoopDetector][GAP_FILTER] query_id={} 候选数={} 均因 min_submap_gap={} 被过滤 → 无回环候选。"
                  " 可尝试将 loop_closure.min_submap_gap 设为 2，或确保轨迹有闭环",
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
                    continue;
                } catch (...) {
                    continue;
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
                double info_scale = lc->inlier_ratio / (lc->rmse + 1e-3f);
                lc->information = Mat66d::Identity() * info_scale;
                ALOG_INFO(MOD, "[LoopDetector][LOOP_OK] 回环约束已发布(parallel) query_id={} target_id={} inlier={:.3f} rmse={:.4f}m",
                          query->id, target->id, lc->inlier_ratio, lc->rmse);
                if (node_) {
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
        double info_scale = lc->inlier_ratio / (lc->rmse + 1e-3f);
        lc->information = Mat66d::Identity() * info_scale;

        ALOG_INFO(MOD, "[LoopDetector][LOOP_OK] 回环约束已发布 query_id={} target_id={} inlier={:.3f} rmse={:.4f}m score={:.3f}",
                  query->id, target->id, lc->inlier_ratio, lc->rmse, lc->overlap_score);
        if (node_) {
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

} // namespace automap_pro
