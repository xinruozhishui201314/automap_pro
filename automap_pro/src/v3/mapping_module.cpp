#include "automap_pro/v3/mapping_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/opt_task_types.h"
#include "automap_pro/v3/map_registry.h"
#include "automap_pro/core/data_types.h"
#include <algorithm>
#include <vector>
#include <filesystem>
#include <chrono>
#include <future>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace fs = std::filesystem;

namespace automap_pro::v3 {
namespace {
uint64_t computeBatchHash(const std::unordered_map<int, Pose3d>& sm_poses,
                          const std::unordered_map<uint64_t, Pose3d>& kf_poses) {
    uint64_t h = 1469598103934665603ull;
    auto mix = [&h](uint64_t v) {
        h ^= v;
        h *= 1099511628211ull;
    };
    std::vector<int> sm_ids;
    sm_ids.reserve(sm_poses.size());
    for (const auto& [id, _] : sm_poses) sm_ids.push_back(id);
    std::sort(sm_ids.begin(), sm_ids.end());
    for (int id : sm_ids) {
        const auto& pose = sm_poses.at(id);
        mix(static_cast<uint64_t>(id));
        mix(static_cast<uint64_t>(pose.translation().x() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().y() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().z() * 1000.0));
    }
    std::vector<uint64_t> kf_ids;
    kf_ids.reserve(kf_poses.size());
    for (const auto& [id, _] : kf_poses) kf_ids.push_back(id);
    std::sort(kf_ids.begin(), kf_ids.end());
    for (uint64_t id : kf_ids) {
        const auto& pose = kf_poses.at(id);
        mix(id);
        mix(static_cast<uint64_t>(pose.translation().x() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().y() * 1000.0));
        mix(static_cast<uint64_t>(pose.translation().z() * 1000.0));
    }
    return h;
}
} // namespace

MappingModule::MappingModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("MappingModule", event_bus, map_registry), node_(node) {
    
    current_session_id_ = static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=SubMapManager::init");
    sm_manager_.init(node_);
    sm_manager_.startNewSession(current_session_id_);
    sm_manager_.registerSubmapFrozenCallback([this](const SubMap::Ptr& sm) { this->onSubmapFrozen(sm); });
    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=session_id=%lu SubMap frozen callback registered",
                static_cast<unsigned long>(current_session_id_));

    hba_optimizer_.init();
    hba_optimizer_.start();
    
    // 🏛️ [修复] 恢复 V3 架构中遗失的 HBA 结果处理逻辑
    hba_optimizer_.registerDoneCallback([this](const HBAResult& result) {
        // 🏛️ [产品化加固] 模块停止后拒绝一切异步回调
        if (!running_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[V3][Mapping] Rejecting HBA result: Module is STOPPED.");
            return;
        }

        if (!result.success || result.optimized_poses.empty()) return;
        if (result.pose_frame == PoseFrame::UNKNOWN) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject HBA result: success-path pose_frame=UNKNOWN");
            METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
            return;
        }
        if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" && result.pose_frame != PoseFrame::MAP) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] strict_map_only rejects non-MAP HBA result");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }

        // 🏛️ [防错判断] 数据合法性校验，防止 NaN 污染 MapRegistry
        if (!result.isFinite() || !result.isReasonable()) {
            RCLCPP_ERROR(node_->get_logger(), 
                "[V3][CRASH_GUARD] Rejecting HBA result: NaN or unreasonable translation detected! (Check HBA logs)");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] HBA optimization finished, applying results via gateway...");

        // 🏛️ [架构加固] 转换为统一 Map 格式并通过网关应用，确保坐标系语义安全
        std::unordered_map<uint64_t, Pose3d> kf_updates;
        if (result.optimized_keyframe_ids.size() == result.optimized_poses.size()) {
            for (size_t i = 0; i < result.optimized_poses.size(); ++i) {
                kf_updates[result.optimized_keyframe_ids[i]] = result.optimized_poses[i];
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][CONTRACT] Reject HBA result: keyframe id list mismatch (%zu vs %zu)",
                result.optimized_keyframe_ids.size(), result.optimized_poses.size());
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }

        OptimizationResultEvent ev;
        ev.version = map_registry_->getVersion() + 1;
        static std::atomic<uint64_t> hba_event_seq{1};
        ev.event_id = hba_event_seq.fetch_add(1, std::memory_order_relaxed);
        ev.submap_poses.clear();
        ev.keyframe_poses = std::move(kf_updates);
        ev.source_module = "HBAOptimizer";
        ev.transform_applied_flags = static_cast<uint32_t>(OptimizationTransformFlags::NONE);
        ev.batch_hash = computeBatchHash(ev.submap_poses, ev.keyframe_poses);
        ev.pose_frame = result.pose_frame;
        if (!ev.isValid()) {
            RCLCPP_ERROR(node_->get_logger(),
                "[V3][CONTRACT] Reject HBA event publish: invalid OptimizationResultEvent");
            METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
            return;
        }
        RCLCPP_INFO(node_->get_logger(),
            "[V3][HBA->GATEWAY] Publishing OptimizationResultEvent: event_id=%lu version=%lu pose_frame=%d gps_aligned=%d kf=%zu batch_hash=%lu",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            static_cast<int>(ev.pose_frame),
            gps_aligned_.load() ? 1 : 0,
            ev.keyframe_poses.size(),
            static_cast<unsigned long>(ev.batch_hash));
        event_bus_->publish(ev);
        RCLCPP_INFO(node_->get_logger(),
            "[V3][MappingModule] HBA results published as OptimizationResultEvent (pose_frame=%d, kf=%zu)",
            static_cast<int>(ev.pose_frame), ev.keyframe_poses.size());
    });

    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=HBAOptimizer init+start+callback OK");

    // 订阅同步帧
    onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(queue_mutex_);
        frame_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅 GPS 对齐结果
    onEvent<GPSAlignedEvent>([this](const GPSAlignedEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lock(queue_mutex_);
        gps_event_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅优化结果（用于更新 SubMapManager 内部位姿）
    // 使用 queue_mutex_ 与 run 循环一致，避免 pose_opt_queue_ 的读写竞态
    onEvent<OptimizationResultEvent>([this](const OptimizationResultEvent& ev) {
        if (!running_.load()) {
            RCLCPP_WARN(node_->get_logger(), "[V3][CONTRACT] Drop optimization event on stopping/stopped module");
            return;
        }
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pose_opt_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅回环（用于 HBA 缓存）
    onEvent<LoopConstraintEvent>([this](const LoopConstraintEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
        loop_constraints_.push_back(ev.constraint);
    });

    // 订阅地图保存与构建命令
    onEvent<SaveMapRequestEvent>([this](const SaveMapRequestEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        Command cmd;
        cmd.type = Command::Type::SAVE_MAP;
        cmd.output_dir = ev.output_dir;
        cmd.save_completion = ev.completion;
        command_queue_.push_back(cmd);
        cv_.notify_one();
    });

    onEvent<GlobalMapBuildRequestEvent>([this](const GlobalMapBuildRequestEvent& ev) {
        if (!running_.load()) return;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        Command cmd;
        cmd.type = Command::Type::BUILD_GLOBAL_MAP;
        cmd.voxel_size = ev.voxel_size;
        cmd.async = ev.async;
        command_queue_.push_back(cmd);
        cv_.notify_one();
    });

    // 订阅手动 HBA 触发
    onEvent<HBARequestEvent>([this](const HBARequestEvent& ev) {
        if (!running_.load()) return;
        auto all_frozen = sm_manager_.getFrozenSubmaps();
        std::vector<LoopConstraint::Ptr> loops;
        {
            std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
            loops = loop_constraints_;
        }
        hba_optimizer_.triggerAsync(all_frozen, loops, ev.wait_for_result, "ManualRequest");
    });
    RCLCPP_INFO(node_->get_logger(),
                "[PIPELINE][MAP] ctor OK events=SyncedFrame+GPSAligned+OptResult+Loop+SaveMap+GlobalMap+HBA");
}

void MappingModule::start() {
    ModuleBase::start();
}

void MappingModule::stop() {
    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] stop step=HBAOptimizer::stop");
    hba_optimizer_.stop();

    // 🏛️ [架构优化] 在正式停止前，强制执行一次全量保存，确保即便析构函数没跑完，数据也已落盘
    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] stop step=forceSyncSave");
    SaveMapRequestEvent final_save;
    final_save.output_dir = ConfigManager::instance().outputDir();
    handleSaveMap(final_save);

    ModuleBase::stop();
}

bool MappingModule::isIdle() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return frame_queue_.empty() && pose_opt_queue_.empty() &&
           gps_event_queue_.empty() && command_queue_.empty() &&
           sm_manager_.isIdle(); // 🏛️ [修复] 检查点云合并子管理器是否也空闲
}

void MappingModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Started worker thread");
    
    while (running_) {
        updateHeartbeat();
        SyncedFrameEvent event;
        OptimizationResultEvent opt_ev;
        GPSAlignedEvent gps_ev;
        Command cmd;
        bool has_frame = false;
        bool has_opt = false;
        bool has_gps_ev = false;
        bool has_cmd = false;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                return !running_ || !frame_queue_.empty() || !pose_opt_queue_.empty() || !gps_event_queue_.empty() || !command_queue_.empty(); 
            });
            if (!running_) break;
            
            // 优先处理优化结果和对齐事件，确保新帧基于最新的地图坐标系
            if (!pose_opt_queue_.empty()) {
                opt_ev = pose_opt_queue_.front();
                pose_opt_queue_.pop_front();
                has_opt = true;
            } else if (!gps_event_queue_.empty()) {
                gps_ev = gps_event_queue_.front();
                gps_event_queue_.pop_front();
                has_gps_ev = true;
            } else if (!command_queue_.empty()) {
                cmd = command_queue_.front();
                command_queue_.pop_front();
                has_cmd = true;
            } else if (!frame_queue_.empty()) {
                // 🏛️ [修复] 确定性因果序屏障：检查地图版本是否已在 Mapping 模块本地处理完成
                const uint64_t target_version = frame_queue_.front().ref_map_version;
                const uint64_t current_version = processed_map_version_.load();
                
                if (target_version <= current_version) {
                    event = frame_queue_.front();
                    frame_queue_.pop_front();
                    has_frame = true;
                    last_barrier_wait_start_time_ = -1.0; // 重置计时
                } else {
                    // 🏛️ [P1 稳定性修复] 引入屏障超时机制，防止后端（Optimizer）卡死导致前端无限阻塞内存撑爆
                    double now_s = node_->now().seconds();
                    if (last_barrier_wait_start_time_ < 0) {
                        last_barrier_wait_start_time_ = now_s;
                    }
                    
                    if (now_s - last_barrier_wait_start_time_ > 5.0) {
                        RCLCPP_ERROR(node_->get_logger(),
                            "[CRITICAL_V3][MappingModule] Version barrier TIMEOUT (5s)! Drop blocked frame: target_ref=%lu current=%lu. "
                            "OptimizerModule might be hung or severely congested.", target_version, current_version);
                        frame_queue_.pop_front();
                        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
                        last_barrier_wait_start_time_ = -1.0;
                    } else {
                        // 地图版本尚未追上，本轮循环跳过处理新帧，等待优化结果入队并处理
                        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "[V3][MappingModule] Frame blocked by LOCAL version barrier: frame_ref=%lu processed=%lu (waiting...)",
                            target_version, current_version);
                    }
                }
            }
        }

        if (has_opt) {
            onPoseOptimized(opt_ev);
        } else if (has_gps_ev) {
            updateGPSAlignment(gps_ev);
        } else if (has_cmd) {
            if (cmd.type == Command::Type::SAVE_MAP) {
                SaveMapRequestEvent ev;
                ev.output_dir = cmd.output_dir;
                try {
                    handleSaveMap(ev);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(),
                        "[V3][MappingModule] SaveMap command failed: %s", e.what());
                }
                if (cmd.save_completion) {
                    try {
                        cmd.save_completion->set_value();
                    } catch (const std::future_error&) {
                        // already satisfied
                    }
                }
            } else if (cmd.type == Command::Type::BUILD_GLOBAL_MAP) {
                GlobalMapBuildRequestEvent ev;
                ev.voxel_size = cmd.voxel_size;
                ev.async = cmd.async;
                handleGlobalMapBuild(ev);
            }
        } else if (has_frame) {
            processFrame(event);
        }
    }
}

void MappingModule::processFrame(const SyncedFrameEvent& event) {
    if (!kf_manager_.shouldCreateKeyFrame(event.T_odom_b, event.timestamp)) {
        return;
    }
    RCLCPP_INFO(node_->get_logger(),
        "[V3][CONTRACT] processFrame ingress ts=%.3f pose_frame=%d cloud_frame=%s has_gps=%d ref_map_version=%lu",
        event.timestamp, static_cast<int>(event.pose_frame), event.cloud_frame.c_str(),
        event.has_gps ? 1 : 0, static_cast<unsigned long>(event.ref_map_version));

    // 🏛️ [点云契约] 明确校验点云坐标系语义，禁止隐式混用
    if (event.cloud_frame != "body" && event.cloud_frame != "world") {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject frame: unsupported cloud_frame=%s (expected body/world)",
            event.cloud_frame.c_str());
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    // 当前仅允许 world 点云与 ODOM 语义位姿配对（通过 T_odom_b 反变换回 body）
    if (event.cloud_frame == "world" && event.pose_frame != PoseFrame::ODOM) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject frame: cloud_frame=world requires pose_frame=ODOM, got pose_frame=%d",
            static_cast<int>(event.pose_frame));
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return;
    }
    if (event.cloud_frame == "body" && event.pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject frame: cloud_frame=body with pose_frame=UNKNOWN");
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return;
    }

    KeyFrame::Ptr prev_kf = kf_manager_.getLastKeyFrame();
    
    // ── V3: 处理坐标系不匹配 (Double Transformation Fix) ──
    // 如果 cloud_frame 为 "world"，则云已经在世界系下。
    // Mapping 模块后续会再次应用 T_map_odom 变换到世界系，导致双重变换。
    // 因此这里先变换回 body 系。
    CloudXYZIPtr cloud = event.cloud;
    CloudXYZIPtr cloud_ds = event.cloud_ds;
    if (event.cloud_frame == "world") {
        CloudXYZIPtr body_cloud(new CloudXYZI());
        pcl::transformPointCloud(*event.cloud, *body_cloud, event.T_odom_b.inverse().cast<float>());
        cloud = body_cloud;

        CloudXYZIPtr body_cloud_ds(new CloudXYZI());
        pcl::transformPointCloud(*event.cloud_ds, *body_cloud_ds, event.T_odom_b.inverse().cast<float>());
        cloud_ds = body_cloud_ds;
        
        RCLCPP_DEBUG(node_->get_logger(), 
            "[V3][CONTRACT] cloud_frame=world converted to body (ts=%.3f pose_frame=%d)",
            event.timestamp, static_cast<int>(event.pose_frame));
    }
    
    // ── V3: 使用 SyncedFrameEvent 中的 GPS 观测 ──
    GPSMeasurement gps = event.has_gps ? event.gps : GPSMeasurement();
    bool has_gps = event.has_gps;
    
    KeyFrame::Ptr kf = kf_manager_.createKeyFrame(
        event.T_odom_b, event.covariance, event.timestamp, cloud, 
        cloud_ds, gps, has_gps, current_session_id_
    );
    
    if (!kf) return;
    kf->livo_info = event.kf_info;
    kf->pose_frame = event.pose_frame; // 🏛️ [架构契约] 继承来源语义

    // 应用 GPS 对齐变换
    bool aligned = gps_aligned_.load();
    Eigen::Matrix3d R_snapshot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_snapshot = Eigen::Vector3d::Zero();
    if (aligned) {
        {
            std::lock_guard<std::mutex> lk(gps_transform_mutex_);
            R_snapshot = gps_transform_R_;
            t_snapshot = gps_transform_t_;
        }

        // 🏛️ [契约核校] 仅当输入为 ODOM 时才需要进行 Map 补偿转换
        if (kf->pose_frame == PoseFrame::ODOM) {
            Pose3d T = Pose3d::Identity();
            T.linear() = R_snapshot * kf->T_odom_b.linear();
            T.translation() = R_snapshot * kf->T_odom_b.translation() + t_snapshot;
            kf->T_map_b_optimized = T;
            kf->pose_frame = PoseFrame::MAP; // 🏛️ [架构加固] 标注语义已提升为 MAP

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[V3][POSE_DIAG] Keyframe upgraded to MAP: T_odom_b=[%.2f,%.2f,%.2f] -> T_map_b_optimized=[%.2f,%.2f,%.2f]",
                kf->T_odom_b.translation().x(), kf->T_odom_b.translation().y(), kf->T_odom_b.translation().z(),
                kf->T_map_b_optimized.translation().x(), kf->T_map_b_optimized.translation().y(), kf->T_map_b_optimized.translation().z());
        }
    }

    // 注册到 MapRegistry（先完成 pose 数据语义升级再发布，避免并发读到半更新状态）
    map_registry_->addKeyFrame(kf);

    bool has_prev_kf = (prev_kf != nullptr);
    int prev_kf_id = has_prev_kf ? static_cast<int>(prev_kf->id) : -1;

    // 发布任务
    GraphTaskEvent task_ev;
    task_ev.task.type = OptTaskItem::Type::KEYFRAME_CREATE;
    task_ev.task.keyframe = kf;
    task_ev.task.has_prev_kf = has_prev_kf;
    task_ev.task.prev_kf_id = prev_kf_id;
    task_ev.task.prev_keyframe = prev_kf;
    task_ev.task.gps_aligned = aligned;
    task_ev.task.gps_transform_R = R_snapshot;
    task_ev.task.gps_transform_t = t_snapshot;
    
    event_bus_->publish(task_ev);
    
    // 同步到 submap_manager
    sm_manager_.addKeyFrame(kf);

    // 🏛️ [P0 优化] 重新引入基于帧数的地图发布逻辑
    // V3 架构此前仅依赖 AutoMapSystem 的 WallTimer (10s)，导致无数据时也冗余构建
    // 这里根据配置的 backend.publish_global_map_every_n_processed 触发构建请求
    const int count = ++processed_frame_count_;
    const int interval = ConfigManager::instance().backendPublishGlobalMapEveryNProcessed();
    if (count % interval == 0) {
        RCLCPP_INFO(node_->get_logger(), 
            "[V3][MappingModule] Triggering periodic global map build (processed_frames=%d interval=%d)",
            count, interval);
        
        GlobalMapBuildRequestEvent build_ev;
        build_ev.voxel_size = ConfigManager::instance().mapVoxelSize();
        build_ev.async = ConfigManager::instance().asyncGlobalMapBuild();
        event_bus_->publish(build_ev);
    }
}

void MappingModule::onSubmapFrozen(const SubMap::Ptr& submap) {
    if (!submap) return;
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=onSubmapFrozen sm_id=%d (grep V3 DIAG)", submap->id);
    map_registry_->addSubMap(submap);
    frozen_submap_count_++;
    const int count = frozen_submap_count_.load();

    // 发布子图节点任务
    GraphTaskEvent node_ev;
    node_ev.task.type = OptTaskItem::Type::SUBMAP_NODE;
    node_ev.task.to_id = submap->id;
    node_ev.task.rel_pose = submap->pose_odom_anchor;
    node_ev.task.fixed = (count == 1);
    event_bus_->publish(node_ev);

    // 子图间里程计因子：用冻结顺序上的「上一块」子图，避免 getFrozenSubmaps()（持 SubMapManager::mutex_，与 merge 长临界区死锁）
    if (prev_frozen_for_odom_) {
        auto prev = prev_frozen_for_odom_;
        Pose3d rel = prev->pose_odom_anchor.inverse() * submap->pose_odom_anchor;
        Mat66d info = computeOdomInfoMatrix(prev, submap, rel);

        GraphTaskEvent odom_ev;
        odom_ev.task.type = OptTaskItem::Type::ODOM_FACTOR;
        odom_ev.task.from_id = prev->id;
        odom_ev.task.to_id = submap->id;
        odom_ev.task.rel_pose = rel;
        odom_ev.task.info_matrix = info;
        event_bus_->publish(odom_ev);
    }
    prev_frozen_for_odom_ = submap;

    // 强制更新
    GraphTaskEvent force_ev;
    force_ev.task.type = OptTaskItem::Type::FORCE_UPDATE;
    event_bus_->publish(force_ev);

    // HBA 触发（子图列表来自 MapRegistry，避免 SubMapManager::mutex_）
    if (ConfigManager::instance().hbaEnabled() && ConfigManager::instance().hbaOnLoop()) {
        const int trigger_mod = ConfigManager::instance().hbaTriggerSubmaps();
        if (count % trigger_mod == 0) {
            std::vector<LoopConstraint::Ptr> loops;
            {
                std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
                loops = loop_constraints_;
            }
            std::vector<SubMap::Ptr> frozen_for_hba;
            for (const auto& sm : map_registry_->getAllSubMaps()) {
                if (sm && (sm->state == SubMapState::FROZEN || sm->state == SubMapState::OPTIMIZED))
                    frozen_for_hba.push_back(sm);
            }
            std::sort(frozen_for_hba.begin(), frozen_for_hba.end(),
                      [](const SubMap::Ptr& a, const SubMap::Ptr& b) {
                          return a->id < b->id;
                      });
            hba_optimizer_.triggerAsync(frozen_for_hba, loops, false, "onSubmapFrozen");
        }
    }
}

void MappingModule::updateGPSAlignment(const GPSAlignedEvent& ev) {
    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] GPS Alignment updated: rmse=%.3fm R_enu_map=[...], t_enu_map=[%.2f, %.2f, %.2f]",
        ev.rmse, ev.t_enu_to_map.x(), ev.t_enu_to_map.y(), ev.t_enu_to_map.z());
    {
        std::lock_guard<std::mutex> lk(gps_transform_mutex_);
        gps_transform_R_ = ev.R_enu_to_map;
        gps_transform_t_ = ev.t_enu_to_map;
        gps_aligned_.store(true);
    }

    // 🏛️ [SSoT 修复] 同步 GPS 对齐状态到 MapRegistry，确保全系统坐标系一致
    uint64_t version = map_registry_->setGPSAligned(true, ev.R_enu_to_map, ev.t_enu_to_map, ev.rmse);
    processed_map_version_.store(version);

    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] MapRegistry version updated to %lu after GPS alignment", version);

    // 关键修复：同步 GPS 对齐状态到 HBA，避免 HBA 在对齐后仍以 gps=0 / ODOM 语义运行
    GPSAlignResult hba_align;
    hba_align.success = true;
    hba_align.R_enu_to_map = ev.R_enu_to_map;
    hba_align.t_enu_to_map = ev.t_enu_to_map;
    hba_align.rmse_m = ev.rmse;
    hba_optimizer_.setGPSAlignedState(hba_align);
    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] HBA GPS alignment state synchronized: rmse=%.3fm", ev.rmse);

    // 🏛️ [修复] 解决中途对齐导致的严重重影：对齐后立即冻结当前活跃子图
    // 确保对齐后的关键帧在新的坐标系下开启新子图，避免与对齐前（Odom系）的关键帧混在同一子图导致 T_submap_kf 剧变
    auto sm = sm_manager_.getActiveSubmap();
    bool did_freeze = (sm && sm->state == SubMapState::ACTIVE);
    if (did_freeze)
        sm_manager_.freezeSubmap(sm);
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=updateGPSAlignment freezeSubmap=%s (grep V3 DIAG)",
        did_freeze ? "yes" : "no");

    // V3 addBatchGPSFactors 等价逻辑：若配置允许，投递 GPS_BATCH_KF任务，批量为历史关键帧添加 GPS 因子
    if (ConfigManager::instance().gpsAddConstraintsOnAlign()) {
        OptTaskItem batch_task;
        batch_task.type = OptTaskItem::Type::GPS_BATCH_KF;
        batch_task.R_enu_to_map = ev.R_enu_to_map;
        batch_task.t_enu_to_map = ev.t_enu_to_map;
        GraphTaskEvent task_ev;
        task_ev.task = batch_task;
        event_bus_->publish(task_ev);
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][DIAG] step=updateGPSAlignment enqueue GPS_BATCH_KF (grep V3 DIAG)");
    }
}

void MappingModule::onPoseOptimized(const OptimizationResultEvent& ev) {
    if (!running_.load()) {
        RCLCPP_WARN(node_->get_logger(), "[V3][CONTRACT] Reject optimization event: module is stopping/stopped");
        return;
    }
    if (!shouldAcceptOptimizationEvent(ev)) return;
    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] Gateway entry: event_id=%lu version=%lu pose_frame=%d sm=%zu kf=%zu src=%s flags=0x%x hash=%lu",
        static_cast<unsigned long>(ev.event_id),
        static_cast<unsigned long>(ev.version), static_cast<int>(ev.pose_frame),
        ev.submap_poses.size(), ev.keyframe_poses.size(),
        ev.source_module.c_str(),
        ev.transform_applied_flags,
        static_cast<unsigned long>(ev.batch_hash));
    
    // 🏛️ [架构加固] 统一通过语义网关应用位姿
    applyOptimizedPoses(ev.submap_poses, ev.keyframe_poses, ev.pose_frame, ev.version);
    if (ev.source_module == "HBAOptimizer") {
        sm_manager_.rebuildMergedCloudFromOptimizedPoses();
    }
}

void MappingModule::applyOptimizedPoses(const std::unordered_map<int, Pose3d>& sm_poses, 
                                       const std::unordered_map<uint64_t, Pose3d>& kf_poses, 
                                       PoseFrame frame, uint64_t version) {
    const auto t0 = std::chrono::steady_clock::now();
    // 🏛️ [产品化加固] 状态守卫
    if (!running_.load()) return;

    // 🏛️ [版本隔离] 拒绝过时的优化结果覆盖新数据
    if (version < processed_map_version_.load() && frame != PoseFrame::MAP) {
        RCLCPP_WARN(node_->get_logger(), 
            "[V3][GATEWAY] Rejecting stale optimization result: version=%lu, current_version=%lu",
            version, processed_map_version_.load());
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        return;
    }

    // 1. 语义校准：如果系统已 GPS 对齐，但输入是 ODOM，则自动补偿 T_map_odom
    std::unordered_map<int, Pose3d> sm_to_apply = sm_poses;
    std::unordered_map<uint64_t, Pose3d> kf_to_apply = kf_poses;

    if (gps_aligned_.load() && frame != PoseFrame::MAP) {
        Pose3d T_map_odom = Pose3d::Identity();
        {
            std::lock_guard<std::mutex> lk(gps_transform_mutex_);
            if (gps_transform_R_.allFinite() && gps_transform_t_.allFinite()) {
                T_map_odom.linear() = gps_transform_R_;
                T_map_odom.translation() = gps_transform_t_;
            }
        }

        // 🏛️ [熔断检查] 校验 T_map_odom 是否存在数值异常
        if (!T_map_odom.matrix().allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CRASH_GUARD] Rejecting compensation: non-finite T_map_odom");
            return;
        }

        // 🏛️ [熔断检查] 如果补偿量过大（平移 > 1000m），通常意味着 GPS 对齐异常
        if (T_map_odom.translation().norm() > 1000.0) {
            RCLCPP_ERROR(node_->get_logger(), 
                "[V3][CRASH_GUARD] ABORT apply: T_map_odom translation=%.1fm > 1000m. Refusing to destroy map structure!",
                T_map_odom.translation().norm());
            return;
        }
        RCLCPP_WARN(node_->get_logger(),
            "[V3][POSE_DIAG] ODOM->MAP compensation active: T_map_odom_t_norm=%.3f input_sm=%zu input_kf=%zu",
            T_map_odom.translation().norm(), sm_to_apply.size(), kf_to_apply.size());

        for (auto& [id, pose] : sm_to_apply) {
            pose = T_map_odom * pose;
        }
        for (auto& [id, pose] : kf_to_apply) {
            pose = T_map_odom * pose;
        }
        RCLCPP_WARN(node_->get_logger(),
            "[V3][POSE_DIAG] Applied T_map_odom compensation to %zu submaps and %zu keyframes",
            sm_to_apply.size(), kf_to_apply.size());
    }

    // 🏛️ [数值安全] 入库前最终扫描，确保不包含 NaN/Inf
    for (const auto& [id, pose] : sm_to_apply) {
        if (!pose.matrix().allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CRASH_GUARD] Found NaN in optimized submap pose %d! Rejecting batch.", id);
            return;
        }
    }
    for (const auto& [id, pose] : kf_to_apply) {
        if (!pose.matrix().allFinite()) {
            RCLCPP_ERROR(node_->get_logger(), "[V3][CRASH_GUARD] Found NaN in optimized keyframe pose %lu! Rejecting batch.", id);
            return;
        }
    }

    // 2. 批量分发到位姿后端 (SubMapManager / MapRegistry)
    if (!sm_to_apply.empty()) {
        sm_manager_.batchUpdateSubmapPoses(sm_to_apply, version, frame);
    }
    if (!kf_to_apply.empty()) {
        sm_manager_.batchUpdateKeyFramePoses(kf_to_apply, version, frame);
    }

    // 3. 屏障同步：更新本地处理版本，解除 frame_queue_ 阻塞
    processed_map_version_.store(version);
    last_applied_version_ = version;

    // 4. 可视化同步（节流：避免每次优化都触发一次全局构图造成后端抖动）
    const int optimize_interval = std::max(1, ConfigManager::instance().backendPublishGlobalMapEveryNProcessed());
    const int optimized_count = ++optimized_apply_count_;
    if (optimized_count % optimize_interval == 0) {
        GlobalMapBuildRequestEvent build_ev;
        build_ev.voxel_size = ConfigManager::instance().mapVoxelSize();
        build_ev.async = ConfigManager::instance().asyncGlobalMapBuild();
        event_bus_->publish(build_ev);
        RCLCPP_INFO(node_->get_logger(),
            "[V3][PERF] Trigger optimize-driven global map build: optimized_count=%d interval=%d version=%lu",
            optimized_count, optimize_interval, static_cast<unsigned long>(version));
    } else {
        RCLCPP_DEBUG(node_->get_logger(),
            "[V3][PERF] Skip optimize-triggered global map build (optimized_count=%d interval=%d)",
            optimized_count, optimize_interval);
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();
    RCLCPP_INFO(node_->get_logger(),
        "[V3][PERF] applyOptimizedPoses done: version=%lu frame=%d sm=%zu kf=%zu elapsed=%ldms",
        static_cast<unsigned long>(version), static_cast<int>(frame),
        sm_to_apply.size(), kf_to_apply.size(), static_cast<long>(elapsed_ms));
}

bool MappingModule::shouldAcceptOptimizationEvent(const OptimizationResultEvent& ev) {
    if (!ev.isValid()) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject invalid OptimizationResultEvent: event_id=%lu version=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            ev.source_module.c_str());
        return false;
    }
    if (ev.pose_frame == PoseFrame::UNKNOWN) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][CONTRACT] Reject event: pose_frame=UNKNOWN");
        METRICS_INCREMENT(metrics::UNKNOWN_FRAME_RESULT_TOTAL);
        return false;
    }
    if (ConfigManager::instance().contractFramePolicy() == "strict_map_only" && ev.pose_frame != PoseFrame::MAP) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject event by frame_policy=strict_map_only: pose_frame=%d",
            static_cast<int>(ev.pose_frame));
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return false;
    }
    if (ev.version < processed_map_version_.load()) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject stale event: event_id=%lu version=%lu current=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            static_cast<unsigned long>(processed_map_version_.load()),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        return false;
    }
    const bool has_map_compensation_flag =
        (ev.transform_applied_flags & static_cast<uint32_t>(OptimizationTransformFlags::MAP_COMPENSATION_APPLIED)) != 0u;
    if (ev.pose_frame == PoseFrame::ODOM && has_map_compensation_flag) {
        RCLCPP_ERROR(node_->get_logger(),
            "[V3][CONTRACT] Reject inconsistent event: ODOM frame cannot carry MAP compensation flag");
        METRICS_INCREMENT(metrics::FRAME_MISMATCH_TOTAL);
        return false;
    }
    if (ev.event_id == last_applied_event_id_) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject duplicate event_id: event_id=%lu version=%lu src=%s",
            static_cast<unsigned long>(ev.event_id),
            static_cast<unsigned long>(ev.version),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::DUPLICATE_OPTIMIZATION_EVENT_TOTAL);
        return false;
    }
    if (ev.version == last_applied_version_ && ev.batch_hash != 0 && ev.batch_hash == last_applied_batch_hash_) {
        RCLCPP_WARN(node_->get_logger(),
            "[V3][CONTRACT] Reject duplicate batch_hash: version=%lu hash=%lu src=%s",
            static_cast<unsigned long>(ev.version),
            static_cast<unsigned long>(ev.batch_hash),
            ev.source_module.c_str());
        METRICS_INCREMENT(metrics::DUPLICATE_OPTIMIZATION_EVENT_TOTAL);
        return false;
    }
    last_applied_event_id_ = ev.event_id;
    last_applied_batch_hash_ = ev.batch_hash;
    return true;
}

void MappingModule::handleSaveMap(const SaveMapRequestEvent& ev) {
    auto all_submaps = sm_manager_.getAllSubmaps();
    RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Saving %zu submaps to %s", 
                all_submaps.size(), ev.output_dir.c_str());

    fs::create_directories(ev.output_dir);
    for (const auto& sm : all_submaps) {
        if (sm) sm_manager_.archiveSubmap(sm, ev.output_dir);
    }

    // ========== [架构增强] 强制生成并保存一份全量全局地图 ==========
    try {
        RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Building final global map for saving...");
        float save_voxel_size = ConfigManager::instance().mapVoxelSize();
        CloudXYZIPtr global_map = sm_manager_.buildGlobalMap(save_voxel_size);
        if (global_map && !global_map->empty()) {
            std::string global_path = ev.output_dir + "/global_map_final.pcd";
            pcl::io::savePCDFileBinary(global_path, *global_map);
            RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Global map saved to %s (%zu pts)", 
                        global_path.c_str(), global_map->size());
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "[V3][MappingModule] Final global map empty or null; skip global_map_final.pcd (output_dir=%s)",
                ev.output_dir.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][MappingModule] Failed to save final global map: %s", e.what());
    }
    // ========================================================
}

void MappingModule::handleGlobalMapBuild(const GlobalMapBuildRequestEvent& ev) {
    try {
        CloudXYZIPtr global;
        if (ev.async) {
            global = sm_manager_.buildGlobalMapAsync(ev.voxel_size).get();
        } else {
            global = sm_manager_.buildGlobalMap(ev.voxel_size);
        }

        GlobalMapBuildResultEvent res;
        res.global_map = global;
        event_bus_->publish(res);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[V3][MappingModule] Global map build failed: %s", e.what());
    }
}

Mat66d MappingModule::computeOdomInfoMatrix(const SubMap::Ptr& prev, const SubMap::Ptr& curr, const Pose3d& rel) const {
    Mat66d info = Mat66d::Identity();
    const auto& cfg = ConfigManager::instance();
    double trans_noise = cfg.smOdomTransNoise();
    double rot_noise = cfg.smOdomRotNoise();
    for (int i = 0; i < 3; ++i) info(i, i) = 1.0 / (trans_noise * trans_noise);
    for (int i = 3; i < 6; ++i) info(i, i) = 1.0 / (rot_noise * rot_noise);
    return info;
}

} // namespace automap_pro::v3
