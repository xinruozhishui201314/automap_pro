#include "automap_pro/v3/mapping_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/opt_task_types.h"
#include "automap_pro/v3/map_registry.h"
#include "automap_pro/core/data_types.h"
#include <algorithm>
#include <filesystem>
#include <pcl/common/transforms.h>

namespace fs = std::filesystem;

namespace automap_pro::v3 {

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
        if (!result.success) return;
        
        RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] HBA optimization finished, applying results...");
        
        // 1. 将优化位姿写回所有关键帧与子图锚点
        sm_manager_.updateAllFromHBA(result);
        
        // 2. 根据新位姿重建各子图的合并点云，彻底消除“轨迹已纠偏、点云仍杂乱”的重影现象
        sm_manager_.rebuildMergedCloudFromOptimizedPoses();
        
        // 3. 异步触发一次全局地图构建，使 RViz 显示立即同步到优化后的状态
        GlobalMapBuildRequestEvent build_ev;
        build_ev.voxel_size = ConfigManager::instance().mapVoxelSize();
        build_ev.async = true;
        event_bus_->publish(build_ev);
        
        RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] HBA results applied and map refresh triggered");
    });

    RCLCPP_INFO(node_->get_logger(), "[PIPELINE][MAP] ctor step=HBAOptimizer init+start+callback OK");

    // 订阅同步帧
    onEvent<SyncedFrameEvent>([this](const SyncedFrameEvent& ev) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        frame_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅 GPS 对齐结果
    onEvent<GPSAlignedEvent>([this](const GPSAlignedEvent& ev) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        gps_event_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅优化结果（用于更新 SubMapManager 内部位姿）
    // 使用 queue_mutex_ 与 run 循环一致，避免 pose_opt_queue_ 的读写竞态
    onEvent<OptimizationResultEvent>([this](const OptimizationResultEvent& ev) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        pose_opt_queue_.push_back(ev);
        cv_.notify_one();
    });

    // 订阅回环（用于 HBA 缓存）
    onEvent<LoopConstraintEvent>([this](const LoopConstraintEvent& ev) {
        std::lock_guard<std::mutex> lk(loop_constraints_mutex_);
        loop_constraints_.push_back(ev.constraint);
    });

    // 订阅地图保存与构建命令
    onEvent<SaveMapRequestEvent>([this](const SaveMapRequestEvent& ev) {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        Command cmd;
        cmd.type = Command::Type::SAVE_MAP;
        cmd.output_dir = ev.output_dir;
        command_queue_.push_back(cmd);
        cv_.notify_one();
    });

    onEvent<GlobalMapBuildRequestEvent>([this](const GlobalMapBuildRequestEvent& ev) {
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
    ModuleBase::stop();
}

void MappingModule::run() {
    RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Started worker thread");
    
    while (running_) {
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
                // 使用 processed_map_version_ 而非 map_registry_->getVersion()，
                // 因为后者可能由 OptimizerModule 提前更新，而 MappingModule 的位姿更新可能还在 queue 中。
                if (frame_queue_.front().ref_map_version <= processed_map_version_.load()) {
                    event = frame_queue_.front();
                    frame_queue_.pop_front();
                    has_frame = true;
                } else {
                    // 地图版本尚未追上，本轮循环跳过处理新帧，等待优化结果入队并处理
                    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "[V3][MappingModule] Frame blocked by LOCAL version barrier: frame_ref=%lu processed=%lu (grep V3 barrier)",
                        frame_queue_.front().ref_map_version, processed_map_version_.load());
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
                handleSaveMap(ev);
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
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=processFrame_keyframe ts=%.3f (grep V3 DIAG)", event.timestamp);

    KeyFrame::Ptr prev_kf = kf_manager_.getLastKeyFrame();
    
    // ── V3: 处理坐标系不匹配 (Double Transformation Fix) ──
    // 如果 frontend.cloud_frame 为 "world"，则云已经在世界系下。
    // Mapping 模块后续会再次应用 T_map_odom 变换到世界系，导致双重变换。
    // 因此这里先变换回 body 系。
    CloudXYZIPtr cloud = event.cloud;
    CloudXYZIPtr cloud_ds = event.cloud_ds;
    if (ConfigManager::instance().frontendCloudFrame() == "world") {
        CloudXYZIPtr body_cloud(new CloudXYZI());
        pcl::transformPointCloud(*event.cloud, *body_cloud, event.T_odom_b.inverse().cast<float>());
        cloud = body_cloud;

        CloudXYZIPtr body_cloud_ds(new CloudXYZI());
        pcl::transformPointCloud(*event.cloud_ds, *body_cloud_ds, event.T_odom_b.inverse().cast<float>());
        cloud_ds = body_cloud_ds;
        
        RCLCPP_DEBUG(node_->get_logger(), 
            "[V3][MappingModule] Transformed cloud from world to body frame to avoid double-transformation (ts=%.3f)", 
            event.timestamp);
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

    // 注册到 MapRegistry
    map_registry_->addKeyFrame(kf);

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
        Pose3d T = Pose3d::Identity();
        T.linear() = R_snapshot * kf->T_odom_b.linear();
        T.translation() = R_snapshot * kf->T_odom_b.translation() + t_snapshot;
        kf->T_map_b_optimized = T;

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[V3][POSE_DIAG] Keyframe transformation: T_odom_b=[%.2f,%.2f,%.2f] -> T_map_b_optimized=[%.2f,%.2f,%.2f] | T_map_odom=[%.2f,%.2f,%.2f] yaw_diff=%.1fdeg",
            kf->T_odom_b.translation().x(), kf->T_odom_b.translation().y(), kf->T_odom_b.translation().z(),
            kf->T_map_b_optimized.translation().x(), kf->T_map_b_optimized.translation().y(), kf->T_map_b_optimized.translation().z(),
            t_snapshot.x(), t_snapshot.y(), t_snapshot.z(),
            std::atan2(R_snapshot(1,0), R_snapshot(0,0)) * 180.0 / M_PI);
    }

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
    RCLCPP_INFO(node_->get_logger(),
        "[V3][POSE_DIAG] Processing pose optimization result: version=%lu sm_updated=%zu kf_updated=%zu",
        static_cast<unsigned long>(ev.version), ev.submap_poses.size(), ev.keyframe_poses.size());
    
    // 🏛️ [优化] 批量更新位姿并执行必要的缓存重建
    sm_manager_.batchUpdateSubmapPoses(ev.submap_poses, ev.version);

    // 🏛️ 更新本地处理版本号，解除 frame_queue_ 的屏障
    processed_map_version_.store(ev.version);
    
    RCLCPP_DEBUG(node_->get_logger(),
        "[V3][DIAG] step=onPoseOptimized version=%lu local_version_updated (grep V3 DIAG)",
        static_cast<unsigned long>(ev.version));

    // 🏛️ [P0 优化] 优化后触发地图更新
    // 异步模式下开销极小（因 SubMapManager 有缓存，若无新关键帧则直接返回），保证可视化实时性
    GlobalMapBuildRequestEvent build_ev;
    build_ev.voxel_size = ConfigManager::instance().mapVoxelSize();
    build_ev.async = ConfigManager::instance().asyncGlobalMapBuild();
    event_bus_->publish(build_ev);
}

void MappingModule::handleSaveMap(const SaveMapRequestEvent& ev) {
    auto all_submaps = sm_manager_.getAllSubmaps();
    RCLCPP_INFO(node_->get_logger(), "[V3][MappingModule] Saving %zu submaps to %s", 
                all_submaps.size(), ev.output_dir.c_str());

    fs::create_directories(ev.output_dir);
    for (const auto& sm : all_submaps) {
        if (sm) sm_manager_.archiveSubmap(sm, ev.output_dir);
    }
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
