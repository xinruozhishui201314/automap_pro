#include "automap_pro/loop_closure/loop_detector.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <automap_pro/msg/loop_constraint_msg.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_OVERLAP_TRANSFORMER_MSGS
#include <overlap_transformer_msgs/srv/compute_descriptor.hpp>
#endif

namespace automap_pro {

LoopDetector::LoopDetector() {
    const auto& cfg = ConfigManager::instance();
    overlap_threshold_ = cfg.overlapThreshold();
    top_k_             = cfg.loopTopK();
    min_temporal_gap_  = cfg.loopMinTemporalGap();
    min_submap_gap_    = cfg.loopMinSubmapGap();
    gps_search_radius_ = cfg.gpsSearchRadius();
    use_icp_refine_    = cfg.teaserICPRefine();
    min_inlier_ratio_  = cfg.teaserMinInlierRatio();
    max_rmse_          = cfg.teaserMaxRMSE();
}

LoopDetector::~LoopDetector() {
    stop();
}

void LoopDetector::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    const auto& cfg = ConfigManager::instance();
#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    if (cfg.overlapTransformerMode() == "external_service") {
        descriptor_client_ = node->create_client<overlap_transformer_msgs::srv::ComputeDescriptor>(
            cfg.overlapDescriptorServiceName());
        if (descriptor_client_->wait_for_service(std::chrono::seconds(2))) {
            use_external_descriptor_ = true;
            RCLCPP_INFO(node->get_logger(), "[LoopDetector] Using external OverlapTransformer service: %s",
                cfg.overlapDescriptorServiceName().c_str());
        } else {
            RCLCPP_WARN(node->get_logger(), "[LoopDetector] External descriptor service not available, falling back to internal.");
        }
    }
#endif
    if (!use_external_descriptor_)
        overlap_transformer_.loadModel(cfg.overlapModelPath());
    constraint_pub_   = node->create_publisher<automap_pro::msg::LoopConstraintMsg>("/automap/loop_constraint", 100);
    loop_marker_pub_  = node->create_publisher<visualization_msgs::msg::MarkerArray>("/automap/loop_markers", 10);
    RCLCPP_INFO(node->get_logger(), "[LoopDetector] Initialized.");
}

void LoopDetector::start() {
    running_ = true;
    worker_thread_ = std::thread(&LoopDetector::workerLoop, this);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[LoopDetector] Worker thread started.");
}

void LoopDetector::stop() {
    running_ = false;
    queue_cv_.notify_all();
    if (worker_thread_.joinable()) worker_thread_.join();
}

void LoopDetector::addSubmap(const SubMap::Ptr& submap) {
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        work_queue_.push(submap);
    }
    queue_cv_.notify_one();
}

void LoopDetector::addToDatabase(const SubMap::Ptr& submap) {
    std::lock_guard<std::mutex> lk(db_mutex_);
    if (!submap->has_descriptor && submap->downsampled_cloud &&
        !submap->downsampled_cloud->empty()) {
        if (use_external_descriptor_ && computeDescriptorExternal(submap->downsampled_cloud, submap->overlap_descriptor))
            submap->has_descriptor = true;
        else {
            submap->overlap_descriptor = overlap_transformer_.computeDescriptor(submap->downsampled_cloud);
            submap->has_descriptor = true;
        }
    }
    db_submaps_.push_back(submap);
    RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"), "[LoopDetector] DB now has %zu submaps", db_submaps_.size());
}

void LoopDetector::clearDatabase() {
    std::lock_guard<std::mutex> lk(db_mutex_);
    db_submaps_.clear();
}

void LoopDetector::registerCallback(LoopConstraintCallback cb) {
    callbacks_.push_back(std::move(cb));
}

size_t LoopDetector::dbSize() const {
    std::lock_guard<std::mutex> lk(db_mutex_);
    return db_submaps_.size();
}

void LoopDetector::workerLoop() {
    while (running_) {
        SubMap::Ptr query;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] { return !work_queue_.empty() || !running_; });
            if (!running_ && work_queue_.empty()) break;
            query = work_queue_.front();
            work_queue_.pop();
        }
        if (query) processSubmap(query);
    }
}

bool LoopDetector::computeDescriptorExternal(const CloudXYZIPtr& cloud, Eigen::VectorXf& out) const {
#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    if (!descriptor_client_ || !descriptor_client_->service_is_ready()) return false;
    auto req = std::make_shared<overlap_transformer_msgs::srv::ComputeDescriptor::Request>();
    pcl::toROSMsg(*cloud, req->pointcloud);
    req->pointcloud.header.stamp = node_->now();
    req->pointcloud.header.frame_id = "body";
    auto result = descriptor_client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(5)) !=
        rclcpp::FutureReturnCode::SUCCESS)
        return false;
    const auto& data = result.get()->descriptor.data;
    out.resize(std::min(static_cast<int>(data.size()), 256));
    for (int i = 0; i < out.size(); ++i) out(i) = data[i];
    return out.size() == 256;
#else
    (void)cloud;
    (void)out;
    return false;
#endif
}

void LoopDetector::processSubmap(const SubMap::Ptr& query) {
    utils::Timer timer;

    if (!query->has_descriptor) {
        if (!query->downsampled_cloud || query->downsampled_cloud->empty()) return;
        if (use_external_descriptor_ && computeDescriptorExternal(query->downsampled_cloud, query->overlap_descriptor))
            query->has_descriptor = true;
        else {
            query->overlap_descriptor = overlap_transformer_.computeDescriptor(query->downsampled_cloud);
            query->has_descriptor = true;
        }
    }

    // Stage 2: Retrieve candidates
    std::vector<SubMap::Ptr> db_copy;
    {
        std::lock_guard<std::mutex> lk(db_mutex_);
        db_copy = db_submaps_;
    }

    auto candidates = overlap_transformer_.retrieve(
        query->overlap_descriptor,
        db_copy,
        top_k_,
        overlap_threshold_,
        min_submap_gap_,
        min_temporal_gap_,
        gps_search_radius_,
        query->gps_center,
        query->has_valid_gps);

    RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"), "[LoopDetector] Submap %d: %zu candidates (%.1f ms)",
              query->id, candidates.size(), timer.elapsedMs());

    for (const auto& cand : candidates) {
        // Find candidate submap
        SubMap::Ptr tgt_sm;
        {
            std::lock_guard<std::mutex> lk(db_mutex_);
            for (const auto& sm : db_submaps_) {
                if (sm->id == cand.submap_id && sm->session_id == cand.session_id) {
                    tgt_sm = sm; break;
                }
            }
        }
        if (!tgt_sm || !tgt_sm->downsampled_cloud || tgt_sm->downsampled_cloud->empty()) continue;
        if (!query->downsampled_cloud || query->downsampled_cloud->empty()) continue;

        // Submap gap check
        if (std::abs(query->id - tgt_sm->id) < min_submap_gap_ &&
            query->session_id == tgt_sm->session_id) continue;

        // Temporal gap check
        double t_gap = std::abs(query->t_start - tgt_sm->t_start);
        if (t_gap < min_temporal_gap_ && query->session_id == tgt_sm->session_id) continue;

        // Stage 3: TEASER++ fine matching
        utils::Timer t_teaser;
        auto teaser_result = teaser_matcher_.match(
            query->downsampled_cloud, tgt_sm->downsampled_cloud,
            Pose3d::Identity());

        RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"), "[LoopDetector] TEASER++ %d→%d: inlier=%.2f rmse=%.3f (%.1f ms)",
                  query->id, tgt_sm->id,
                  teaser_result.inlier_ratio, teaser_result.rmse,
                  t_teaser.elapsedMs());

        if (!teaser_result.success) continue;

        // Stage 4 (optional): ICP refinement
        Pose3d final_T = teaser_result.T_tgt_src;
        double final_rmse = teaser_result.rmse;

        if (use_icp_refine_) {
            auto icp_result = icp_refiner_.refine(
                query->downsampled_cloud, tgt_sm->downsampled_cloud, final_T);
            if (icp_result.converged && icp_result.rmse < final_rmse) {
                final_T    = icp_result.T_refined;
                final_rmse = icp_result.rmse;
            }
        }

        // Build loop constraint
        auto lc = std::make_shared<LoopConstraint>();
        lc->submap_i        = tgt_sm->id;    // i = older (target)
        lc->submap_j        = query->id;     // j = newer (source)
        lc->delta_T         = final_T;
        lc->inlier_ratio    = teaser_result.inlier_ratio;
        lc->rmse            = final_rmse;
        lc->overlap_score   = cand.score;
        lc->fitness_score   = teaser_result.rmse;
        lc->is_inter_session = (query->session_id != tgt_sm->session_id);
        lc->status          = LoopStatus::ACCEPTED;

        // Information matrix: higher quality → larger information
        double info_scale = lc->inlier_ratio / (lc->rmse + 1e-3);
        lc->information = Mat66d::Identity() * info_scale;

        if (validate(lc)) {
            RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[LoopDetector] Loop: %d←→%d | inlier=%.2f rmse=%.3f score=%.2f",
                     lc->submap_i, lc->submap_j,
                     lc->inlier_ratio, lc->rmse, lc->overlap_score);

            for (auto& cb : callbacks_) cb(lc);

            // Publish ROS message
            automap_pro::msg::LoopConstraintMsg msg;
            msg.header.stamp  = node_->now();
            msg.submap_i      = lc->submap_i;
            msg.submap_j      = lc->submap_j;
            msg.overlap_score = lc->overlap_score;
            msg.inlier_ratio  = lc->inlier_ratio;
            msg.rmse          = lc->rmse;
            msg.is_inter_session = lc->is_inter_session;
            constraint_pub_->publish(msg);
        }
    }

    // Add query to DB for future loops
    addToDatabase(query);

    RCLCPP_DEBUG(rclcpp::get_logger("automap_pro"), "[LoopDetector] Submap %d processed in %.1f ms total",
              query->id, timer.elapsedMs());
}

bool LoopDetector::validate(const LoopConstraint::Ptr& lc) const {
    if (lc->inlier_ratio < min_inlier_ratio_) return false;
    if (lc->rmse          > max_rmse_)         return false;
    return true;
}

}  // namespace automap_pro
