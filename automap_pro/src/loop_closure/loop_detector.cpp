#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/loop_closure/icp_refiner.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "LoopDetector"

#include <automap_pro/msg/loop_constraint_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cmath>

namespace automap_pro {

LoopDetector::LoopDetector() {
    const auto& cfg = ConfigManager::instance();
    overlap_threshold_ = cfg.overlapThreshold();
    top_k_             = cfg.loopTopK();
    min_temporal_gap_  = cfg.loopMinTemporalGap();
    min_submap_gap_    = cfg.loopMinSubmapGap();
    gps_search_radius_ = cfg.gpsSearchRadius();
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
    running_ = false;
    desc_cv_.notify_all();
    match_cv_.notify_all();
    for (auto& t : desc_workers_) if (t.joinable()) t.join();
    if (match_worker_.joinable()) match_worker_.join();
}

void LoopDetector::addSubmap(const SubMap::Ptr& submap) {
    float priority = static_cast<float>(submap->id);
    size_t qsize;
    size_t dbsize;
    {
        std::lock_guard<std::mutex> lk(desc_mutex_);
        desc_queue_.push({submap, priority});
        qsize = desc_queue_.size();
        dbsize = dbSize();
    }
    desc_cv_.notify_one();
    RCLCPP_INFO(node_->get_logger(),
        "[LoopDetector][DATA] addSubmap sm_id=%d kf=%zu desc_pts=%zu queue=%zu db=%zu",
        submap->id, submap->keyframes.size(),
        submap->downsampled_cloud ? submap->downsampled_cloud->size() : 0u, qsize, dbsize);
    ALOG_INFO(MOD, "SubMap#{} submitted for loop detection (queue_size={}, db_size={})",
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

        // 计算描述子
        if (!submap->has_descriptor) {
            computeDescriptorAsync(submap);
        } else {
            onDescriptorReady(submap);
        }
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
    // 先加入数据库
    addToDatabase(submap);

    // 检索候选（Stage 1）
    std::vector<SubMap::Ptr> db_copy;
    {
        std::shared_lock<std::shared_mutex> lk(db_mutex_);
        db_copy = db_submaps_;
    }

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

    if (candidates.empty()) return;

    // 过滤候选（时间/子图间隔）
    std::vector<OverlapTransformerInfer::Candidate> valid_candidates;
    for (const auto& cand : candidates) {
        if (cand.session_id == submap->session_id &&
            std::abs(cand.submap_id - submap->id) < min_submap_gap_) continue;
        valid_candidates.push_back(cand);
    }
    if (valid_candidates.empty()) return;

    // 入队时深拷贝 query 点云，避免 worker 处理时读 SubMap::downsampled_cloud 与 buildGlobalMap 等并发
    CloudXYZIPtr query_cloud_copy;
    if (submap->downsampled_cloud && !submap->downsampled_cloud->empty()) {
        query_cloud_copy = std::make_shared<CloudXYZI>();
        *query_cloud_copy = *submap->downsampled_cloud;
        ALOG_INFO(MOD, "enqueue MatchTask query_id={} query_cloud_pts={} candidates={}",
                  submap->id, query_cloud_copy->size(), valid_candidates.size());
    }

    // 提交到 TEASER++ 匹配队列（Stage 2）
    {
        std::lock_guard<std::mutex> lk(match_mutex_);
        match_queue_.push({submap, query_cloud_copy, valid_candidates});
    }
    match_cv_.notify_one();
}

// ─────────────────────────────────────────────────────────────────────────────
// Worker: TEASER++ 匹配（单线程 Stage 2）
// ─────────────────────────────────────────────────────────────────────────────
void LoopDetector::matchWorkerLoop() {
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
    IcpRefiner icp;
    const auto& query = task.query;
    // 优先使用入队时拷贝的 query_cloud，避免读 SubMap::downsampled_cloud 与主线程并发
    CloudXYZIPtr query_cloud = task.query_cloud;
    if (!query_cloud || query_cloud->empty()) {
        CloudXYZIPtr query_ref = query ? query->downsampled_cloud : nullptr;
        if (!query_ref || query_ref->empty()) {
            ALOG_DEBUG(MOD, "[tid={}] step=task_skip query_id={} query_cloud null/empty", tid, query ? query->id : -1);
            return;
        }
        query_cloud = std::make_shared<CloudXYZI>();
        *query_cloud = *query_ref;
        ALOG_INFO(MOD, "[tid={}] step=task_fallback_copy query_id={} query_pts={} (no enqueue copy)", tid, query->id, query_cloud->size());
    }

    ALOG_INFO(MOD, "[tid={}] step=task_start query_id={} query_cloud_ptr={} query_cloud_use_count={} query_pts={} candidates={}",
              tid, query ? query->id : -1, static_cast<const void*>(query_cloud.get()), query_cloud.use_count(), query_cloud->size(), task.candidates.size());
    AUTOMAP_TIMED_SCOPE(MOD, fmt::format("GeomVerify SM#{}", task.query->id), 3000.0);

    for (const auto& cand : task.candidates) {
        ALOG_DEBUG(MOD, "  Checking candidate SM#{} score={:.3f}", cand.submap_id, cand.score);
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
        if (!target_ref || target_ref->empty()) continue;

        // 深拷贝 target 点云，避免 PCL/TEASER 与主线程竞争
        CloudXYZIPtr target_cloud = std::make_shared<CloudXYZI>();
        *target_cloud = *target_ref;

        ALOG_INFO(MOD, "[tid={}] step=cand_geom query_id={} target_id={} target_ref_use_count={} target_cloud_ptr={} target_pts={} score={:.3f}",
                   tid, query->id, target->id, target_ref.use_count(), static_cast<const void*>(target_cloud.get()), target_cloud->size(), cand.score);

        // Stage 2: TEASER++ 粗配准
        auto t0 = std::chrono::steady_clock::now();
        ALOG_INFO(MOD, "[tid={}] step=teaser_call_enter query_id={} target_id={} query_cloud_ptr={} query_use_count={} query_pts={} target_cloud_ptr={} target_use_count={} target_pts={}",
                  tid, query->id, target->id, static_cast<const void*>(query_cloud.get()), query_cloud.use_count(), query_cloud->size(),
                  static_cast<const void*>(target_cloud.get()), target_cloud.use_count(), target_cloud->size());
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
            // 不重新throw，继续处理下一个候选（loop closure是可选的）
            continue;
        } catch (...) {
            ALOG_ERROR(MOD, "[tid={}] step=match_exception query_id={} target_id={} unknown_exception", 
                      tid, query->id, target->id);
            continue;
        }
        double teaser_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        ALOG_INFO(MOD, "[tid={}] step=teaser_result query_id={} target_id={} success={} inlier={:.2f} corrs={} rmse={:.3f}m ms={:.1f} ptr_src_after={} ptr_tgt_after={}",
                   tid, query->id, target->id, teaser_res.success, teaser_res.inlier_ratio, teaser_res.num_correspondences, teaser_res.rmse, teaser_ms,
                   static_cast<const void*>(query_cloud.get()), static_cast<const void*>(target_cloud.get()));

        if (!teaser_res.success || teaser_res.inlier_ratio < min_inlier_ratio_) continue;

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

        if (final_rmse > max_rmse_) continue;

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

        if (node_) {
            RCLCPP_INFO(node_->get_logger(),
                "[LoopDetector] Loop %d↔%d | inlier=%.2f rmse=%.3f score=%.2f teaser=%.1fms",
                lc->submap_i, lc->submap_j,
                lc->inlier_ratio, lc->rmse, lc->overlap_score, teaser_ms);
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
