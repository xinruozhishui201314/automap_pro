#pragma once

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "automap_pro/core/data_types.h"
#include "automap_pro/msg/loop_constraint_msg.hpp"
#include "automap_pro/loop_closure/overlap_transformer.h"
#include "automap_pro/loop_closure/teaser_matcher.h"
#include "automap_pro/loop_closure/icp_refiner.h"

#ifdef USE_OVERLAP_TRANSFORMER_MSGS
#include <overlap_transformer_msgs/srv/compute_descriptor.hpp>
#endif

namespace automap_pro {

class LoopDetector {
public:
    using LoopConstraintCallback = std::function<void(const LoopConstraint::Ptr&)>;

    LoopDetector();
    ~LoopDetector();

    void init(rclcpp::Node::SharedPtr node);
    void start();
    void stop();

    void addSubmap(const SubMap::Ptr& submap);
    void addToDatabase(const SubMap::Ptr& submap);
    void clearDatabase();
    void registerCallback(LoopConstraintCallback cb);
    size_t dbSize() const;

private:
    void workerLoop();
    void processSubmap(const SubMap::Ptr& query);
    bool validate(const LoopConstraint::Ptr& lc) const;
    bool computeDescriptorExternal(const CloudXYZIPtr& cloud, Eigen::VectorXf& out) const;

    OverlapTransformer overlap_transformer_;
#ifdef USE_OVERLAP_TRANSFORMER_MSGS
    rclcpp::Client<overlap_transformer_msgs::srv::ComputeDescriptor>::SharedPtr descriptor_client_;
#endif
    bool use_external_descriptor_ = false;
    TeaserMatcher      teaser_matcher_;
    ICPRefiner         icp_refiner_;

    std::vector<SubMap::Ptr> db_submaps_;
    mutable std::mutex db_mutex_;

    std::queue<SubMap::Ptr>       work_queue_;
    std::mutex                    queue_mutex_;
    std::condition_variable       queue_cv_;
    std::thread                   worker_thread_;
    std::atomic<bool>             running_{false};

    std::vector<LoopConstraintCallback> callbacks_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<automap_pro::msg::LoopConstraintMsg>::SharedPtr constraint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;

    double overlap_threshold_;
    int    top_k_;
    double min_temporal_gap_;
    int    min_submap_gap_;
    double gps_search_radius_;
    bool   use_icp_refine_;
    double min_inlier_ratio_;
    double max_rmse_;
};

}  // namespace automap_pro
