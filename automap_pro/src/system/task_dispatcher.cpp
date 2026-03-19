#include "automap_pro/system/task_dispatcher.h"

#include "automap_pro/core/logger.h"

#include <rclcpp/rclcpp.hpp>

namespace automap_pro {

TaskDispatcher::TaskDispatcher(IncrementalOptimizer& optimizer, rclcpp::Node* node)
    : optimizer_(&optimizer)
    , node_(node)
    , opt_task_queue_(nullptr)
    , opt_task_mutex_(nullptr)
    , opt_task_cv_(nullptr)
    , max_queue_size_(64) {
}

TaskDispatcher::TaskDispatcher(std::deque<OptTaskItem>* opt_task_queue,
                               std::mutex* opt_task_mutex,
                               std::condition_variable* opt_task_cv,
                               size_t max_queue_size)
    : optimizer_(nullptr)
    , node_(nullptr)
    , opt_task_queue_(opt_task_queue)
    , opt_task_mutex_(opt_task_mutex)
    , opt_task_cv_(opt_task_cv)
    , max_queue_size_(max_queue_size) {
}

TaskDispatcher::~TaskDispatcher() {
    stop();
}

void TaskDispatcher::start() {
    running_.store(true);
}

void TaskDispatcher::stop() {
    running_.store(false);
}

bool TaskDispatcher::submitLoopFactor(const LoopConstraint::Ptr& lc) {
    if (!lc) return false;
    
    OptTaskItem task;
    task.type = OptTaskItem::Type::LOOP_FACTOR;
    task.loop_constraint = lc;
    
    // 原有队列逻辑保留（通过外部设置）
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    // 如果没有外部队列，直接处理
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitGPSFactor(int sm_id, const Eigen::Vector3d& pos, 
                                    const Eigen::Matrix3d& cov) {
    OptTaskItem task;
    task.type = OptTaskItem::Type::GPS_FACTOR;
    task.to_id = sm_id;
    task.gps_pos = pos;
    task.gps_cov = cov;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitSubmapNode(int sm_id, const Pose3d& pose, bool fixed) {
    OptTaskItem task;
    task.type = OptTaskItem::Type::SUBMAP_NODE;
    task.to_id = sm_id;
    task.rel_pose = pose;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitOdomFactor(int from, int to, const Pose3d& rel, 
                                     const Mat66d& info) {
    OptTaskItem task;
    task.type = OptTaskItem::Type::ODOM_FACTOR;
    task.from_id = from;
    task.to_id = to;
    task.rel_pose = rel;
    task.info_matrix = info;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitRebuild(const std::vector<SubmapData>& submaps,
                                  const std::vector<OdomFactorItem>& odom_factors,
                                  const std::vector<LoopFactorItem>& loop_factors) {
    OptTaskItem task;
    task.type = OptTaskItem::Type::REBUILD;
    task.submap_data = submaps;
    task.odom_factors = odom_factors;
    task.loop_factors = loop_factors;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitGPSAlignComplete(const std::vector<SubmapData>& submaps,
                                          const std::vector<OdomFactorItem>& odom_factors,
                                          const std::vector<LoopFactorItem>& loop_factors,
                                          const Eigen::Matrix3d& R_enu_to_map,
                                          const Eigen::Vector3d& t_enu_to_map) {
    OptTaskItem task;
    task.type = OptTaskItem::Type::GPS_ALIGN_COMPLETE;
    task.submap_data = submaps;
    task.odom_factors = odom_factors;
    task.loop_factors = loop_factors;
    task.R_enu_to_map = R_enu_to_map;
    task.t_enu_to_map = t_enu_to_map;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitReset() {
    OptTaskItem task;
    task.type = OptTaskItem::Type::RESET;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitForceUpdate() {
    OptTaskItem task;
    task.type = OptTaskItem::Type::FORCE_UPDATE;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitKeyFrameCreate(const KeyFrame::Ptr& kf,
                                         bool has_prev_kf,
                                         int prev_kf_id,
                                         const KeyFrame::Ptr& prev_kf,
                                         bool gps_aligned,
                                         const Eigen::Matrix3d& gps_transform_R,
                                         const Eigen::Vector3d& gps_transform_t) {
    if (!kf) return false;
    
    OptTaskItem task;
    task.type = OptTaskItem::Type::KEYFRAME_CREATE;
    task.keyframe = kf;
    task.has_prev_kf = has_prev_kf;
    task.prev_kf_id = prev_kf_id;
    task.prev_keyframe = prev_kf;
    task.gps_aligned = gps_aligned;
    task.gps_transform_R = gps_transform_R;
    task.gps_transform_t = gps_transform_t;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

bool TaskDispatcher::submitActiveSubmapGPSBind(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    OptTaskItem task;
    task.type = OptTaskItem::Type::ACTIVE_SUBMAP_GPS_BIND;
    task.R_enu_to_map = R;
    task.t_enu_to_map = t;
    
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (opt_task_queue_->size() < max_queue_size_) {
            opt_task_queue_->push_back(task);
            if (opt_task_cv_) {
                opt_task_cv_->notify_one();
            }
            return true;
        }
    }
    
    if (external_handler_) {
        external_handler_(task);
        return true;
    }
    
    return false;
}

void TaskDispatcher::waitForCompletion() {
    if (opt_task_mutex_ && opt_task_cv_) {
        std::unique_lock<std::mutex> lock(*opt_task_mutex_);
        opt_task_cv_->wait(lock, [this] {
            return !running_.load() || (opt_task_queue_ && opt_task_queue_->empty());
        });
    }
}

size_t TaskDispatcher::queueSize() const {
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        return opt_task_queue_->size();
    }
    return 0;
}

bool TaskDispatcher::isRunning() const {
    return running_.load();
}

void TaskDispatcher::registerPoseCallback(PoseUpdateCallback cb) {
    std::lock_guard<std::mutex> lock(cbs_mutex_);
    pose_cbs_.push_back(std::move(cb));
}

void TaskDispatcher::setExternalHandler(std::function<void(const OptTaskItem&)> handler) {
    external_handler_ = std::move(handler);
}

bool TaskDispatcher::tryDequeue(OptTaskItem& task) {
    if (opt_task_mutex_ && opt_task_queue_) {
        std::lock_guard<std::mutex> lock(*opt_task_mutex_);
        if (!opt_task_queue_->empty()) {
            task = opt_task_queue_->front();
            opt_task_queue_->pop_front();
            return true;
        }
    }
    return false;
}

void TaskDispatcher::processTask(const OptTaskItem& task) {
    // 这里处理任务的实际逻辑，但当前版本任务由外部处理
    logInfo("Processing task type: " + std::to_string(static_cast<int>(task.type)));
}

void TaskDispatcher::logWarn(const std::string& msg) {
    if (node_) {
        RCLCPP_WARN(node_->get_logger(), "[TaskDispatcher] %s", msg.c_str());
    }
}

void TaskDispatcher::logInfo(const std::string& msg) {
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[TaskDispatcher] %s", msg.c_str());
    }
}

}  // namespace automap_pro
