#include "automap_pro/backend/hba_optimizer.h"
#include "automap_pro/backend/gps_constraint_policy.h"
#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/backend/isam2_factor_types.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/v3/semantic_backend_gates.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#define MOD "HBAOptimizer"
/** 建图后端每步计算日志：与 incremental_optimizer 统一 tag [BACKEND_STEP]，便于 grep 定位 */
#define BACKEND_STEP(fmt, ...) \
    RCLCPP_INFO(rclcpp::get_logger("automap_system"), "[BACKEND_STEP] " fmt, ##__VA_ARGS__)

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <string>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <vector>

#ifdef USE_HBA_API
#include <hba_api/hba_api.h>
#endif
#ifdef USE_GTSAM_FALLBACK
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#endif
#include <yaml-cpp/yaml.h>

namespace automap_pro {
namespace {
std::atomic<uint64_t> g_hba_loop_added_total{0};
std::atomic<uint64_t> g_hba_gps_added_total{0};

// 与 ConfigManager::gpsLeverArmImu 同源；主配置未给出非零杆臂时，按主 YAML 所在目录回退 sensor_config/gps_imu_extrinsic.yaml（避免依赖进程 CWD）
void resolveGpsLeverArmForHba(Eigen::Vector3d& lever_arm) {
    lever_arm = Eigen::Vector3d::Zero();
    try {
        const auto& cfg = ConfigManager::instance();
        if (cfg.isLoaded()) {
            lever_arm = cfg.gpsLeverArmImu();
            if (lever_arm.norm() > 1e-12) {
                return;
            }
        }
    } catch (const std::exception&) {
        // 回退到外参文件
    }

    std::vector<std::filesystem::path> candidates;
    try {
        const auto& cfg = ConfigManager::instance();
        if (cfg.isLoaded() && !cfg.configFilePath().empty()) {
            std::filesystem::path base(cfg.configFilePath());
            candidates.push_back(base.parent_path() / "sensor_config" / "gps_imu_extrinsic.yaml");
        }
    } catch (...) {
    }
    candidates.emplace_back("automap_pro/config/sensor_config/gps_imu_extrinsic.yaml");

    for (const auto& extrinsic_path : candidates) {
        try {
            if (extrinsic_path.empty() || !std::filesystem::exists(extrinsic_path)) {
                continue;
            }
            YAML::Node config = YAML::LoadFile(extrinsic_path.string());
            if (config["T_gps_imu"] && config["T_gps_imu"]["translation"]) {
                auto trans = config["T_gps_imu"]["translation"];
                lever_arm = Eigen::Vector3d(trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>());
                return;
            }
        } catch (const std::exception&) {
            continue;
        }
    }
}
} // namespace

#ifdef USE_GTSAM_FALLBACK
namespace {
constexpr double kMaxReasonableTranslationNorm = 1e6;

static gtsam::Pose3 toGtsamPose3(const Pose3d& T) {
    Eigen::Quaterniond q(T.rotation());
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
    gtsam::Point3 t(T.translation().x(), T.translation().y(), T.translation().z());
    return gtsam::Pose3(rot, t);
}
static Pose3d fromGtsamPose3(const gtsam::Pose3& p) {
    Pose3d T = Pose3d::Identity();
    T.translation() = Eigen::Vector3d(p.x(), p.y(), p.z());
    const auto& q = p.rotation().toQuaternion();
    T.linear() = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
    return T;
}
static gtsam::Symbol KF(size_t i) { return gtsam::Symbol('x', static_cast<gtsam::Key>(i)); }
static gtsam::Symbol SM(int id) { return gtsam::Symbol('s', id); }
} // namespace
#endif

HBAOptimizer::HBAOptimizer() = default;

HBAOptimizer::~HBAOptimizer() { stop(); }

void HBAOptimizer::init() {
    const auto& cfg = ConfigManager::instance();
    hba_enabled_ = cfg.hbaEnabled();
    hba_gtsam_fallback_enabled_ = cfg.hbaGtsamFallbackEnabled();
    gps_min_accepted_quality_level_ = cfg.gpsMinAcceptedQualityLevel();
    gps_keyframe_match_window_s_ = cfg.gpsKeyframeMatchWindowS();
    hba_total_layers_ = cfg.hbaTotalLayers();
    hba_thread_num_ = cfg.hbaThreadNum();
    hba_trigger_submaps_ = cfg.hbaTriggerSubmaps();
    hba_on_loop_ = cfg.hbaOnLoop();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[DIAG][HBA][CONFIG] enabled=%d gtsam_fallback=%d trigger_submaps=%d on_loop=%d total_layers=%d "
        "thread_num=%d gps_min_quality=%d gps_match_window_s=%.2f",
        hba_enabled_ ? 1 : 0,
        hba_gtsam_fallback_enabled_ ? 1 : 0,
        hba_trigger_submaps_,
        hba_on_loop_ ? 1 : 0,
        hba_total_layers_,
        hba_thread_num_,
        gps_min_accepted_quality_level_,
        gps_keyframe_match_window_s_);

    Eigen::Vector3d cfg_lever_arm = Eigen::Vector3d::Zero();
    bool cfg_loaded = false;
    std::string cfg_path;
    try {
        cfg_loaded = cfg.isLoaded();
        cfg_path = cfg.configFilePath();
        if (cfg_loaded) cfg_lever_arm = cfg.gpsLeverArmImu();
    } catch (...) {
    }
    resolveGpsLeverArmForHba(lever_arm_);
    if (lever_arm_.norm() > 1e-12) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA] GPS lever arm resolved=[%.4f, %.4f, %.4f] m (cfg_loaded=%d cfg_path=%s cfg_gps_lever_arm=[%.4f, %.4f, %.4f])",
            lever_arm_.x(), lever_arm_.y(), lever_arm_.z(),
            cfg_loaded ? 1 : 0, cfg_path.c_str(),
            cfg_lever_arm.x(), cfg_lever_arm.y(), cfg_lever_arm.z());
    } else {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA] GPS lever arm is zero (cfg_loaded=%d cfg_path=%s cfg_gps_lever_arm=[%.4f, %.4f, %.4f]). "
            "Set gps.lever_arm_imu in main YAML or T_gps_imu.translation in config_dir/sensor_config/gps_imu_extrinsic.yaml",
            cfg_loaded ? 1 : 0, cfg_path.c_str(),
            cfg_lever_arm.x(), cfg_lever_arm.y(), cfg_lever_arm.z());
    }
}

void HBAOptimizer::start() {
    worker_thread_finished_.store(false, std::memory_order_release);
    running_ = true;
    worker_thread_ = std::thread(&HBAOptimizer::workerLoop, this);
}

void HBAOptimizer::stop() {
    running_ = false;
    queue_cv_.notify_all();
    // 🏛️ [修复] 使用带超时的 join，防止 HBA 优化任务（可能耗时较长）在程序退出时造成阻塞
    // 理由：AutoMapSystem 析构时会调用此函数。若 HBA 正在运行且 keyframe 数量巨大，
    // 原生的 join() 会导致整个进程卡死在此处，无法完成最后的日志落地。
    stopJoinWithTimeout(std::chrono::seconds(5));
}

void HBAOptimizer::stopJoinWithTimeout(std::chrono::milliseconds max_join) {
    running_ = false;
    queue_cv_.notify_all();
    if (!worker_thread_.joinable()) return;
    const auto deadline = std::chrono::steady_clock::now() + max_join;
    while (std::chrono::steady_clock::now() < deadline) {
        if (worker_thread_finished_.load(std::memory_order_acquire)) {
            worker_thread_.join();
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
        "[HBAOptimizer][SHUTDOWN] stopJoinWithTimeout: worker did not finish within %lld ms (likely blocked inside runHBA). "
        "Detaching HBA worker thread — system state: HBA worker abandoned; process exit should follow immediately.",
        static_cast<long long>(max_join.count()));
    worker_thread_.detach();
}

void HBAOptimizer::onSubmapFrozen(const SubMap::Ptr& submap) {
    BACKEND_STEP("step=HBA_onSubmapFrozen_enter sm_id=%d frozen_count=%d kf_count=%zu",
        submap->id, frozen_count_ + 1, submap->keyframes.size());
    frozen_count_++;

    // 增强诊断日志：记录触发条件检查（使用 init 缓存的 hba_trigger_submaps_/hba_on_loop_）
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][CHECK] onSubmapFrozen: frozen_count=%d trigger_mod=%d hbaOnLoop=%d hbaEnabled=%d gps_aligned=%d",
        frozen_count_, hba_trigger_submaps_, hba_on_loop_ ? 1 : 0,
        hba_enabled_ ? 1 : 0, gps_aligned_ ? 1 : 0);

    if (frozen_count_ % hba_trigger_submaps_ != 0) {
        BACKEND_STEP("step=HBA_onSubmapFrozen_skip sm_id=%d reason=trigger_mod frozen_count=%d mod=%d",
            submap->id, frozen_count_, hba_trigger_submaps_);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][CHECK] skip: frozen_count=%d mod %d != 0",
            frozen_count_, hba_trigger_submaps_);
        return;
    }

    if (!hba_on_loop_) {
        BACKEND_STEP("step=HBA_onSubmapFrozen_skip sm_id=%d reason=hbaOnLoop_false", submap->id);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][CHECK] skip: hbaOnLoop=false");
        return;
    }

    // 收集子图所有 KF（这里仅放入单个子图，实际由 AutoMapSystem 传入全量）
    std::lock_guard<std::mutex> lk(queue_mutex_);
    PendingTask task;
    for (const auto& kf : submap->keyframes) {
        task.keyframes.push_back(kf);
    }
    task.enable_gps = gps_aligned_;
    if (!task.keyframes.empty()) {
        size_t kf_count = task.keyframes.size();
        size_t qdepth = pending_queue_.size() + 1;
        pending_queue_.push(std::move(task));
        queue_cv_.notify_one();
        trigger_count_++;
        BACKEND_STEP("step=HBA_onSubmapFrozen_trigger sm_id=%d kf_count=%zu queue_depth=%zu result=ok",
            submap->id, kf_count, qdepth);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][TRIGGER] onSubmapFrozen: sm_id=%d kf_count=%zu queue_depth=%zu trigger_count=%d gps_aligned=%d",
            submap->id, kf_count, qdepth, trigger_count_, gps_aligned_ ? 1 : 0);
        ALOG_INFO(MOD, "HBA triggered by frozen submap: kf_count={} queue_depth={}", kf_count, qdepth);
    }
}

void HBAOptimizer::triggerAsync(
    const std::vector<SubMap::Ptr>& all_submaps,
    const std::vector<LoopConstraint::Ptr>& loops,
    bool wait,
    const char* trigger_source,
    uint64_t alignment_epoch_snapshot)
{
    auto kfs = collectKeyFramesFromSubmaps(all_submaps);
    if (kfs.empty()) return;

    // 建立 SubMap ID 到其首个有效关键帧全局 ID 的映射，用于解析回环约束
    std::unordered_map<int, uint64_t> sm_id_to_anchor_kf_id;
    std::unordered_map<int, SubMap::Ptr> sm_id_to_ptr;
    for (const auto& sm : all_submaps) {
        if (!sm) continue;
        sm_id_to_ptr[sm->id] = sm;
        for (const auto& kf : sm->keyframes) {
            if (kf && kf->cloud_body && !kf->cloud_body->empty() && 
                kf->T_odom_b.translation().allFinite() && kf->T_odom_b.rotation().matrix().allFinite()) {
                sm_id_to_anchor_kf_id[sm->id] = kf->id;
                break; // 找到第一个有效的关键帧作为该子图在 HBA 中的锚点
            }
        }
    }

    // 解析回环约束中的局部索引为全局 ID
    std::vector<LoopConstraint::Ptr> resolved_loops;
    size_t loops_total = 0;
    size_t loops_already_kf = 0;
    size_t loops_intra_resolved = 0;
    size_t loops_inter_resolved = 0;
    size_t loops_unresolved = 0;
    size_t loops_degenerate_same_node = 0;
    for (const auto& lc : loops) {
        if (!lc) continue;
        loops_total++;
        
        // 🏛️ [架构增强] 过滤 PENDING 状态的回环，防止未经验证的约束进入 HBA
        if (lc->status == LoopStatus::REJECTED) {
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_FILTER] skip rejected loop sm%d <-> sm%d", lc->submap_i, lc->submap_j);
            continue;
        }

        // 创建副本，避免修改原始回环数据（虽然是 shared_ptr，但为了安全建议新建）
        auto resolved_lc = std::make_shared<LoopConstraint>(*lc);
        bool resolved = false;

        // 情况1：已经是关键帧级回环（已有全局 ID）
        if (lc->keyframe_global_id_i >= 0 && lc->keyframe_global_id_j >= 0) {
            resolved = true;
            loops_already_kf++;
        }
        // 情况2：子图内回环（submap_i == submap_j）
        else if (lc->submap_i == lc->submap_j) {
            auto it = sm_id_to_ptr.find(lc->submap_i);
            if (it != sm_id_to_ptr.end() && !it->second->keyframes.empty()) {
                const auto& sm_kfs = it->second->keyframes;
                if (lc->keyframe_i >= 0 && lc->keyframe_i < (int)sm_kfs.size() &&
                    lc->keyframe_j >= 0 && lc->keyframe_j < (int)sm_kfs.size()) {
                    resolved_lc->keyframe_global_id_i = sm_kfs[lc->keyframe_i]->id;
                    resolved_lc->keyframe_global_id_j = sm_kfs[lc->keyframe_j]->id;
                    resolved = true;
                    loops_intra_resolved++;
                }
            }
        }
        // 情况3：子图间回环（submap_i != submap_j）
        else {
            auto it_i = sm_id_to_anchor_kf_id.find(lc->submap_i);
            auto it_j = sm_id_to_anchor_kf_id.find(lc->submap_j);
            if (it_i != sm_id_to_anchor_kf_id.end() && it_j != sm_id_to_anchor_kf_id.end()) {
                resolved_lc->keyframe_global_id_i = it_i->second;
                resolved_lc->keyframe_global_id_j = it_j->second;
                resolved = true;
                loops_inter_resolved++;
            }
        }

        if (resolved && resolved_lc->keyframe_global_id_i == resolved_lc->keyframe_global_id_j) {
            loops_degenerate_same_node++;
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_RESOLVE] drop degenerate loop resolved to same keyframe id=%d (sm%d/kf%d <-> sm%d/kf%d)",
                resolved_lc->keyframe_global_id_i, lc->submap_i, lc->keyframe_i, lc->submap_j, lc->keyframe_j);
            continue;
        }
        if (resolved) {
            resolved_loops.push_back(resolved_lc);
        } else {
            loops_unresolved++;
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_RESOLVE] failed to resolve loop sm%d(kf%d) <-> sm%d(kf%d)",
                lc->submap_i, lc->keyframe_i, lc->submap_j, lc->keyframe_j);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][LOOP_RESOLVE] total=%zu resolved=%zu already_kf=%zu intra=%zu inter=%zu unresolved=%zu degenerate_same_node=%zu",
        loops_total, resolved_loops.size(), loops_already_kf, loops_intra_resolved, loops_inter_resolved, loops_unresolved, loops_degenerate_same_node);

    size_t kf_count = kfs.size();
    size_t sm_count = all_submaps.size();
    size_t loop_count = resolved_loops.size();

    // 🏛️ [架构增强] 收集并构建所有子图内的语义因子，准备进行 HBA 语义优化
    std::vector<CylinderFactorItemKF> all_semantic_factors;
    std::vector<PlaneFactorItemKF> all_semantic_plane_factors;
    std::unordered_map<int, Pose3d> sm_anchor_poses;
    for (const auto& sm : all_submaps) {
        if (!sm) continue;
        sm_anchor_poses[sm->id] = sm->pose_map_anchor_optimized;
        
        for (const auto& kf : sm->keyframes) {
            if (!kf) continue;
            for (const auto& l_kf : kf->landmarks) {
                if (!l_kf) continue;

                CylinderFactorItemKF factor;
                factor.kf_id = kf->id;
                factor.sm_id = static_cast<uint64_t>(sm->id);
                factor.weight = l_kf->confidence;

                bool factor_ready = false;
                CylinderLandmark::Ptr l_sm_gate;
                if (l_kf->associated_idx >= 0 && l_kf->associated_idx < static_cast<int>(sm->landmarks.size())) {
                    const auto& l_sm = sm->landmarks[l_kf->associated_idx];
                    if (l_sm) {
                        l_sm_gate = l_sm;
                        factor.root_submap = l_sm->root;
                        factor.ray_submap = l_sm->ray;
                        factor.radius = l_sm->radius;
                        factor_ready = true;
                    }
                }

                if (!factor_ready) {
                    // Fallback: 关联信息缺失时，使用 KF 局部语义观测通过 T_submap_kf 投影到子图系，避免语义信息被整体丢弃。
                    factor.root_submap = kf->T_submap_kf * l_kf->root;
                    factor.ray_submap = (kf->T_submap_kf.rotation() * l_kf->ray).normalized();
                    factor.radius = l_kf->radius;
                    factor_ready = std::isfinite(factor.radius) && factor.radius > 0.0;
                }

                if (factor_ready && factor.root_submap.allFinite() && factor.ray_submap.allFinite()) {
                    const auto& ccfg = ConfigManager::instance();
                    // 与 MappingModule 一致：若 trunk root 贴近强支撑墙面，则视为墙边伪树干，跳过该因子。
                    if (v3::semanticBackendCylinderTooCloseToPlanes(ccfg, kf->T_submap_kf, l_kf, sm->plane_landmarks)) {
                        continue;
                    }
                    if (!v3::semanticBackendCylinderPassesGating(ccfg, kf->T_submap_kf, l_kf, l_sm_gate)) {
                        continue;
                    }
                    factor.point_body = v3::semanticBackendCylinderSampleBody(l_kf);
                    all_semantic_factors.push_back(factor);
                }
            }
            for (const auto& p_kf : kf->plane_landmarks) {
                if (!p_kf || !p_kf->isValid()) continue;
                PlaneFactorItemKF pf;
                pf.kf_id = kf->id;
                pf.sm_id = static_cast<uint64_t>(sm->id);
                // Prefer associated canonical plane in submap.
                bool ready = false;
                if (p_kf->associated_idx >= 0 && p_kf->associated_idx < static_cast<int>(sm->plane_landmarks.size())) {
                    const auto& p_sm = sm->plane_landmarks[p_kf->associated_idx];
                    if (p_sm && p_sm->isValid()) {
                        pf.normal_submap = p_sm->normal.normalized();
                        pf.distance_submap = p_sm->distance;
                        ready = true;
                    }
                }
                if (!ready) {
                    Eigen::Vector3d n_b = p_kf->normal;
                    if (!n_b.allFinite() || n_b.norm() < 1e-6) continue;
                    n_b.normalize();
                    pf.normal_submap = (kf->T_submap_kf.rotation() * n_b).normalized();
                    pf.distance_submap = p_kf->distance - pf.normal_submap.dot(kf->T_submap_kf.translation());
                    ready = true;
                }
                Eigen::Vector3d centroid_b = Eigen::Vector3d::Zero();
                size_t cnt = 0;
                if (p_kf->points && !p_kf->points->empty()) {
                    for (const auto& pt : p_kf->points->points) {
                        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                        centroid_b += Eigen::Vector3d(pt.x, pt.y, pt.z);
                        ++cnt;
                    }
                }
                if (cnt == 0) {
                    centroid_b = -p_kf->distance * p_kf->normal.normalized();
                } else {
                    centroid_b /= static_cast<double>(cnt);
                }
                pf.point_body = centroid_b;
                pf.weight = std::max(1e-3, p_kf->confidence);
                if (ready && pf.point_body.allFinite() && pf.normal_submap.allFinite() && std::isfinite(pf.distance_submap)) {
                    const auto& ccfg = ConfigManager::instance();
                    if (!v3::semanticBackendPlanePassesGating(ccfg, p_kf)) {
                        continue;
                    }
                    all_semantic_plane_factors.push_back(pf);
                }
            }
        }
    }

    size_t semantic_count = all_semantic_factors.size();
    size_t semantic_plane_count = all_semantic_plane_factors.size();
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        PendingTask task;
        task.keyframes  = std::move(kfs);
        task.loops      = std::move(resolved_loops);
        task.semantic_factors = std::move(all_semantic_factors);
        task.semantic_plane_factors = std::move(all_semantic_plane_factors);
        task.submap_anchor_poses = std::move(sm_anchor_poses);
        task.alignment_epoch_snapshot = alignment_epoch_snapshot;
        task.enable_gps = gps_aligned_;
        pending_queue_.push(std::move(task));
        queue_cv_.notify_one();
        trigger_count_++;
    }
    size_t queue_depth = 0;
    { std::lock_guard<std::mutex> lk(queue_mutex_); queue_depth = pending_queue_.size(); }
    const char* src = trigger_source ? trigger_source : "unknown";
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][STATE] enqueue source=%s submaps=%zu keyframes=%zu loops=%zu semantic_factors=%zu semantic_plane_factors=%zu gps=%d trigger_count=%d queue_depth=%zu (重影诊断: 同一轮建图内 trigger_count>1 表示被多次触发)",
        src, sm_count, kf_count, loop_count, semantic_count, semantic_plane_count, gps_aligned_ ? 1 : 0, trigger_count_, queue_depth);
    ALOG_INFO(MOD, "HBA triggerAsync: source={} submaps={} keyframes={} loops={} gps={} trigger_count={} queue_depth={}",
              src, sm_count, kf_count, loop_count, gps_aligned_ ? 1 : 0, trigger_count_, queue_depth);

    if (wait) {
        // 设置合理超时：最多等待5分钟，避免永久阻塞导致析构卡死
        constexpr auto kMaxWaitTime = std::chrono::minutes(5);
        waitUntilIdleFor(kMaxWaitTime);
        if (!isIdle()) {
            ALOG_ERROR(MOD, "HBA wait timeout after 5 minutes — save/stop may race with running HBA (use stopJoinWithTimeout on shutdown)");
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBAOptimizer][TIMEOUT] HBA did not become idle after 5 minutes (wait=true). "
                "saveMapToFiles / shutdown will use stopJoinWithTimeout to avoid unbounded join; expect possible abandoned HBA worker if still blocked in runHBA.");
        }
    }
}

bool HBAOptimizer::isIdle() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    return pending_queue_.empty() && !hba_running_.load();
}

size_t HBAOptimizer::queueDepth() const {
    std::lock_guard<std::mutex> lk(queue_mutex_);
    size_t n = pending_queue_.size();
    if (hba_running_.load()) n += 1;
    return n;
}

void HBAOptimizer::waitUntilIdleFor(std::chrono::milliseconds timeout_ms) {
    // timeout_ms <= 0 视为使用最大合理等待时间，避免无限阻塞析构
    constexpr auto kMaxWaitTime = std::chrono::minutes(5);
    if (timeout_ms.count() <= 0) {
        timeout_ms = std::chrono::duration_cast<std::chrono::milliseconds>(kMaxWaitTime);
    }

    auto deadline = std::chrono::steady_clock::now() + timeout_ms;
    while (std::chrono::steady_clock::now() < deadline && !isIdle()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void HBAOptimizer::onGPSAligned(
    const GPSAlignResult& align_result,
    const std::vector<SubMap::Ptr>& all_submaps,
    const std::vector<LoopConstraint::Ptr>& loops)
{
    gps_aligned_     = true;
    gps_align_result_ = align_result;

    // GPS 对齐后立即触发一次全局优化（backend.hba.enabled=false 时跳过，仅 ISAM2+GPS）
    if (hba_enabled_)
        triggerAsync(all_submaps, loops, false, "onGPSAligned", 0);
}

void HBAOptimizer::setGPSAlignedState(const GPSAlignResult& align_result) {
    gps_aligned_      = align_result.success;
    gps_align_result_ = align_result;
}

void HBAOptimizer::workerLoop() {
    while (running_) {
        PendingTask task;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] {
                return !pending_queue_.empty() || !running_;
            });
            if (!running_ && pending_queue_.empty()) break;

            // 取最新任务（丢弃旧任务，避免堆积）
            size_t old_size = pending_queue_.size();
            while (pending_queue_.size() > 1) pending_queue_.pop();
            task = std::move(pending_queue_.front());
            pending_queue_.pop();
            
            // 记录任务丢弃日志
            if (old_size > 1) {
                ALOG_WARN(MOD, "Dropped {} old HBA tasks to avoid backlog", old_size - 1);
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[HBAOptimizer][TASK_DROP] Dropped %zu old HBA tasks (queue size: %zu)",
                    static_cast<size_t>(old_size - 1), old_size);
            }
        }

        hba_running_ = true;
        BACKEND_STEP("step=HBA_workerLoop_start keyframes=%zu gps=%d", task.keyframes.size(), task.enable_gps ? 1 : 0);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][STATE] optimization start keyframes=%zu gps=%d (running=1)",
            task.keyframes.size(), task.enable_gps ? 1 : 0);
        ALOG_INFO(MOD, "HBA optimization starting: kf_count={} gps={}",
                  task.keyframes.size(), task.enable_gps);
        AUTOMAP_TIMED_SCOPE(MOD, "HBA full optimize", 60000.0);
        HBAResult result = runHBA(task);

        if (result.success) {
            BACKEND_STEP("step=HBA_workerLoop_done success=1 MME=%.4f elapsed_ms=%.1f poses=%zu",
                result.final_mme, result.elapsed_ms, result.optimized_poses.size());
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][STATE] optimization done success=1 MME=%.4f elapsed=%.1fms poses=%zu (running=1 until callbacks done)",
                result.final_mme, result.elapsed_ms, result.optimized_poses.size());
            ALOG_INFO(MOD, "HBA done: MME={:.4f} elapsed={:.1f}ms kf={}",
                      result.final_mme, result.elapsed_ms, result.optimized_poses.size());
        } else {
            BACKEND_STEP("step=HBA_workerLoop_done success=0 elapsed_ms=%.1f", result.elapsed_ms);
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[HBA][STATE] optimization done success=0 elapsed=%.1fms (running=1 until callbacks done)", result.elapsed_ms);
            ALOG_ERROR(MOD, "HBA failed after {:.1f}ms", result.elapsed_ms);
        }
        // 先执行回调（含 updateAllFromHBA + rebuildMergedCloudFromOptimizedPoses），再置 idle，
        // 保证 finish 线程的 waitUntilIdleFor() 在 rebuild 完成后才返回，避免 save 与 rebuild 竞态导致重影（见 docs/HBA_GHOSTING_ROOT_CAUSE_20260317.md）
        for (auto& cb : done_cbs_) cb(result);
        hba_running_ = false;
    }
    worker_thread_finished_.store(true, std::memory_order_release);
}

HBAResult HBAOptimizer::runHBA(const PendingTask& task) {
    BACKEND_STEP("step=HBA_runHBA_enter keyframes=%zu gps=%d", task.keyframes.size(), task.enable_gps ? 1 : 0);
    HBAResult result;
    result.success = false;
    result.alignment_epoch_snapshot = task.alignment_epoch_snapshot;

    // ========== 数据验证 ==========
    if (task.keyframes.empty()) {
        BACKEND_STEP("step=HBA_runHBA_skip reason=no_keyframes");
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] No keyframes provided, skipping optimization");
        ALOG_WARN(MOD, "HBA aborted: no keyframes in task");
        return result;
    }

    // 检查关键帧数据完整性
    int valid_kf_count = 0;
    int empty_cloud_count = 0;
    int invalid_pose_count = 0;
    for (const auto& kf : task.keyframes) {
        if (!kf) { invalid_pose_count++; continue; }
        if (!kf->cloud_body || kf->cloud_body->empty()) {
            empty_cloud_count++;
            continue;
        }
        const auto& t = kf->T_odom_b.translation();
        const auto& R = kf->T_odom_b.rotation();
        if (!t.allFinite() || !R.allFinite()) {
            invalid_pose_count++;
            continue;
        }
        valid_kf_count++;
    }

    if (valid_kf_count == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] All keyframes invalid: empty=%d invalid_pose=%d total=%zu, skipping optimization",
            empty_cloud_count, invalid_pose_count, task.keyframes.size());
        ALOG_ERROR(MOD, "HBA aborted: all {} keyframes are invalid", task.keyframes.size());
        return result;
    }

    if (valid_kf_count < static_cast<int>(task.keyframes.size())) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] Some keyframes invalid: valid=%d empty_cloud=%d invalid_pose=%d total=%zu",
            valid_kf_count, empty_cloud_count, invalid_pose_count, task.keyframes.size());
    }

    // 检查时间跨度是否合理
    double time_span = task.keyframes.back()->timestamp - task.keyframes.front()->timestamp;
    if (time_span <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] Invalid time span: %.3f s, skipping optimization", time_span);
        return result;
    }

    // 时间跨度超过2小时视为异常
    if (time_span > 7200.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][VALIDATION] Unusual time span: %.1f hours (>2h), proceeding anyway", time_span / 3600.0);
    }

    BACKEND_STEP("step=HBA_runHBA_validation_done valid_kf=%d time_span=%.1fs gps=%d",
        valid_kf_count, time_span, task.enable_gps ? 1 : 0);
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][DATA] valid_keyframes=%d time_span=%.1fs gps=%d",
        valid_kf_count, time_span, task.enable_gps ? 1 : 0);
    ALOG_INFO(MOD, "HBA starting: valid_kf={} time_span={:.1f}s gps={}",
              valid_kf_count, time_span, task.enable_gps ? 1 : 0);

    // [PCD_GHOSTING_VERIFY] HBA 输入顺序：与 updateAllFromHBA 的 WRITEBACK_ORDER 应对齐（同为 collect 的 filter+sort+dedupe 顺序）；若 first/last kf_id+ts 不一致则写回错位→PCD 重影（grep HBA_INPUT_ORDER WRITEBACK_ORDER）
    {
        const auto& kfs = task.keyframes;
        if (!kfs.empty()) {
            const auto& f = kfs.front();
            const auto& b = kfs.back();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA_INPUT_ORDER] first kf_id=%lu sm_id=%d ts=%.3f last kf_id=%lu sm_id=%d ts=%.3f count=%zu (与 WRITEBACK_ORDER 应对齐，否则写回错位)",
                f->id, f->submap_id, f->timestamp, b->id, b->submap_id, b->timestamp, kfs.size());
        }
    }

    // lever_arm_ 在 init() 中通过 resolveGpsLeverArmForHba 已缓存，worker 路径不再访问 ConfigManager

#ifdef USE_HBA_API
    // 语义契约强门控：一旦存在语义因子，禁止走 HBA API（当前 API 不支持语义因子）。
    // 为避免“看起来开了语义、运行时却丢语义”，强制切换到 GTSAM fallback。
    if (!task.semantic_factors.empty() || !task.semantic_plane_factors.empty()) {
#ifdef USE_GTSAM_FALLBACK
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][CONTRACT][SEMANTIC_GATE] semantic_factors(cyl/plane)=(%zu/%zu) detected: force backend switch API -> GTSAM_fallback",
            task.semantic_factors.size(), task.semantic_plane_factors.size());
        ALOG_WARN(MOD, "HBA semantic gate: forcing GTSAM fallback for {} semantic factors",
                  task.semantic_factors.size());
        return runGTSAMFallback(task);
#else
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][CONTRACT][SEMANTIC_GATE] semantic_factors(cyl/plane)=(%zu/%zu) detected but GTSAM fallback is unavailable. "
            "Rejecting HBA run to prevent semantic loss.",
            task.semantic_factors.size(), task.semantic_plane_factors.size());
        ALOG_ERROR(MOD, "HBA semantic gate: fallback unavailable, rejecting run to avoid semantic loss");
        result.success = false;
        return result;
#endif
    }

    hba_api::Config hba_cfg;
    hba_cfg.total_layer_num = hba_total_layers_;
    hba_cfg.thread_num      = hba_thread_num_;
    hba_cfg.voxel_size      = 0.5;
    hba_cfg.enable_gps      = task.enable_gps;

    // 若启用 GPS，构建 GPS 条目（仅采纳 HIGH/EXCELLENT 质量，避免低质量拉偏）
    if (task.enable_gps && gps_aligned_) {
        for (const auto& kf : task.keyframes) {
            if (kf->has_valid_gps &&
                gpsQualityAcceptedByPolicy(kf->gps.quality, gps_min_accepted_quality_level_)) {
                hba_api::Config::GPSEntry entry;
                entry.timestamp = kf->gps.timestamp;
                entry.lat       = kf->gps.latitude;
                entry.lon       = kf->gps.longitude;
                entry.alt       = kf->gps.altitude;
                hba_cfg.gps_entries.push_back(entry);
            }
        }
    }

    hba_api::HBAOptimizer optimizer(hba_cfg);

    // 添加关键帧（按时间戳排序）
    std::vector<KeyFrame::Ptr> sorted_kfs = task.keyframes;
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    for (const auto& kf : sorted_kfs) {
        hba_api::KeyFrameInput input;
        input.timestamp   = kf->timestamp;
        input.rotation    = Eigen::Quaterniond(kf->T_odom_b.rotation());
        input.translation = kf->T_odom_b.translation();
        // 使用 cloud_body（body 系下），HBA 内部会用位姿变换到世界系
        if (kf->cloud_body && !kf->cloud_body->empty()) {
            // 转换 CloudXYZI → CloudXYZIN（HBA 需要 PointXYZINormal）
            auto cloud_in = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
            for (const auto& pt : kf->cloud_body->points) {
                pcl::PointXYZINormal p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z;
                p.intensity = pt.intensity;
                cloud_in->push_back(p);
            }
            input.cloud = cloud_in;
        }
        optimizer.addKeyFrame(input);
    }

    // 执行 HBA 优化（进度输出到终端，与 [HBA][STATE] 一致）
    std::string params = "keyframes=" + std::to_string(task.keyframes.size()) + " gps=" + (task.enable_gps ? "1" : "0");
    
    // 🏛️ [架构报警] HBA API 目前不支持语义地标约束
    if (!task.semantic_factors.empty() || !task.semantic_plane_factors.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][SEMANTIC_GAP] HBA API path does NOT support semantic landmarks yet (cyl=%zu plane=%zu ignored). "
            "Consider enabling backend.hba.enable_gtsam_fallback for full semantic integration.",
            task.semantic_factors.size(), task.semantic_plane_factors.size());
    }

    GtsamCallScope scope(GtsamCaller::HBA, "PGO", params, true);
    auto api_result = optimizer.optimize([](int cur, int total, float pct) {
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][STATE] layer %d/%d %.0f%%", cur, total, pct);
    });
    scope.setSuccess(api_result.success);

    if (api_result.success) {
        BACKEND_STEP("step=HBA_runHBA_done backend=api success=1 MME=%.4f elapsed_ms=%.1f poses=%zu",
            api_result.final_mme, api_result.elapsed_ms, api_result.optimized_poses.size());
        // 不在此处写回 KF，由 updateAllFromHBA 持 SubMapManager::mutex_ 统一写回，避免与 buildGlobalMapAsync 取快照并发导致重影（见 GHOSTING 根因分析）
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GHOSTING_DIAG] runHBA API path no KF write here poses=%zu (writeback in updateAllFromHBA under mutex)",
            api_result.optimized_poses.size());
        result.success         = true;
        result.final_mme       = api_result.final_mme;
        result.elapsed_ms      = api_result.elapsed_ms;
        result.optimized_poses = api_result.optimized_poses;
        result.optimized_keyframe_ids.clear();
        result.optimized_keyframe_ids.reserve(sorted_kfs.size());
        for (const auto& kf : sorted_kfs) result.optimized_keyframe_ids.push_back(kf->id);
        result.pose_frame      = gps_aligned_ ? PoseFrame::MAP : PoseFrame::ODOM;
    } else {
        BACKEND_STEP("step=HBA_runHBA_done backend=api success=0 elapsed_ms=%.1f error=%s",
            api_result.elapsed_ms, api_result.error_msg.c_str());
        result.success = false;
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] HBA API failed: %s (grep HBA BACKEND EXCEPTION)", api_result.error_msg.c_str());
        ALOG_ERROR(MOD, "HBA API failed: {}", api_result.error_msg);
        fprintf(stderr, "[HBAOptimizer] HBA failed: %s\n",
                api_result.error_msg.c_str());
    }

#elif defined(USE_GTSAM_FALLBACK)
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[HBA][CONFIG] backend.hba.enable_gtsam_fallback=%d (1=run GTSAM fallback, 0=skip; verify config file has enable_gtsam_fallback: true)",
        hba_gtsam_fallback_enabled_ ? 1 : 0);
    if (!hba_gtsam_fallback_enabled_) {
        BACKEND_STEP("step=HBA_runHBA_skip reason=gtsam_fallback_disabled");
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] GTSAM fallback disabled (backend.hba.enable_gtsam_fallback=false), skipping HBA");
        ALOG_WARN(MOD, "HBA: GTSAM fallback disabled by config, skip");
        result.success = false;
    } else {
        result = runGTSAMFallback(task);
    }
#else
    BACKEND_STEP("step=HBA_runHBA_skip reason=hba_api_not_available");
    result.success = false;
    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
        "[HBA][BACKEND] hba_api not available, skipping optimization (no changes applied)");
    fprintf(stderr, "[HBAOptimizer] hba_api not available, skipping optimization\n");
#endif

    return result;
}

#ifdef USE_GTSAM_FALLBACK
HBAResult HBAOptimizer::runGTSAMFallback(const PendingTask& task) {
    auto t_start = std::chrono::steady_clock::now();
    BACKEND_STEP("step=HBA_runGTSAMFallback_enter keyframes=%zu loops=%zu gps=%d", 
                 task.keyframes.size(), task.loops.size(), task.enable_gps ? 1 : 0);
    HBAResult result;
    result.success = false;
    if (task.keyframes.empty()) {
        BACKEND_STEP("step=HBA_runGTSAMFallback_skip reason=no_keyframes");
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][VALIDATION] GTSAM fallback: no keyframes, abort");
        return result;
    }

    // lever_arm_ 在 init() 中已缓存，worker 路径不再访问 ConfigManager

    std::vector<KeyFrame::Ptr> sorted_kfs = task.keyframes;
    std::sort(sorted_kfs.begin(), sorted_kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    // 建立 ID 到索引的映射，用于添加回环约束
    std::unordered_map<uint64_t, size_t> kf_id_to_idx;
    for (size_t i = 0; i < sorted_kfs.size(); ++i) {
        kf_id_to_idx[sorted_kfs[i]->id] = i;
    }

    // [PCD_GHOSTING_VERIFY] GTSAM 路径下 result.optimized_poses 顺序 = sorted_kfs（时间戳序），与 SubMapManager collectKeyframesInHBAOrder 一致；打条便于与 WRITEBACK_ORDER 对照
    if (!sorted_kfs.empty()) {
        const auto& f = sorted_kfs.front();
        const auto& b = sorted_kfs.back();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA_INPUT_ORDER] first kf_id=%lu sm_id=%d ts=%.3f last kf_id=%lu sm_id=%d ts=%.3f count=%zu (GTSAM_sorted，与 WRITEBACK_ORDER 应对齐)",
            f->id, f->submap_id, f->timestamp, b->id, b->submap_id, b->timestamp, sorted_kfs.size());
    }

    // HBA 输出位姿坐标系语义探测（用于上层写回时避免 map/odom 混淆）
    // 若输入里已有明显 map!=odom 偏移，则将输出视作 MAP；否则视作 ODOM。
    bool input_has_map_offset = false;
    double max_trans_diff = 0.0;
    double max_rot_diff_deg = 0.0;
    for (const auto& kf : sorted_kfs) {
        if (!kf) continue;
        const double trans_diff = (kf->T_map_b_optimized.translation() - kf->T_odom_b.translation()).norm();
        const double rot_diff = Eigen::AngleAxisd(
            kf->T_map_b_optimized.rotation().inverse() * kf->T_odom_b.rotation()).angle();
        max_trans_diff = std::max(max_trans_diff, trans_diff);
        max_rot_diff_deg = std::max(max_rot_diff_deg, rot_diff * 180.0 / M_PI);
        if (trans_diff > 0.20 || rot_diff > (5.0 * M_PI / 180.0)) {
            input_has_map_offset = true;
            break;
        }
    }
    result.pose_frame = input_has_map_offset ? PoseFrame::MAP : PoseFrame::ODOM;
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[V3][CONTRACT] HBA fallback frame inference: input_has_map_offset=%d max_trans_diff=%.3fm max_rot_diff=%.2fdeg => pose_frame=%d",
        input_has_map_offset ? 1 : 0, max_trans_diff, max_rot_diff_deg, static_cast<int>(result.pose_frame));

    // 约束合理性：所有关键帧位姿有限且平移在合理范围内，否则不进入 GTSAM 避免崩溃
    auto poseForInitial = [](const KeyFrame::Ptr& kf) -> Pose3d {
        const Pose3d& o = kf->T_map_b_optimized;
        const Pose3d& t = kf->T_odom_b;
        if ((o.translation() - t.translation()).norm() > 1e-9 || !o.rotation().isApprox(t.rotation()))
            return o;
        return t;
    };
    for (size_t i = 0; i < sorted_kfs.size(); ++i) {
        Pose3d p = poseForInitial(sorted_kfs[i]);
        const auto& t = p.translation();
        const auto& R = p.rotation();
        if (!t.allFinite() || !R.matrix().allFinite()) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBA][BACKEND][VALIDATION] GTSAM fallback: keyframe[%zu] pose non-finite, abort (grep HBA VALIDATION)",
                i);
            return result;
        }
        if (t.norm() > kMaxReasonableTranslationNorm) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBA][BACKEND][VALIDATION] GTSAM fallback: keyframe[%zu] translation norm=%.1f > %.0f, abort",
                i, t.norm(), kMaxReasonableTranslationNorm);
            return result;
        }
    }

    ensureGtsamTbbSerialized();
    GtsamCallScope scope(GtsamCaller::HBA, "GTSAM_fallback",
                        "keyframes=" + std::to_string(sorted_kfs.size()) +
                        " loops=" + std::to_string(task.loops.size()) +
                        " gps=" + (task.enable_gps ? "1" : "0"), true);

    try {
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;
        // 记录每个因子的类型，用于崩溃时定位（与 ISAM2 双路 GTSAM 时 double free 诊断）
        std::vector<std::string> factor_type_log;

        // 每个因子使用独立噪声模型，避免多因子共享同一 shared_ptr 在 GTSAM 内触发 double free
        //（与 incremental_optimizer 及 FIX_GPS_BATCH_SIGSEGV 文档中 borglab/gtsam#1189 同类问题一致）
        // 🔧 [修复] 放宽首帧 Prior 约束：1e-9 -> 0.25 (0.5m std dev)
        // 原因：若初始对齐有微小偏差，过死的 Prior 会阻止轨迹向 GPS 整体偏移，导致拉花。
        const double prior_var = 0.25;
        const double between_rotate_var = 1e-6; // 降低旋转方差，增强轨迹刚度
        const double between_trans_var = 1e-4;  // 降低平移方差

        // 使用命名变量作为方差向量，避免将 Eigen 临时量传入 Variances() 导致 GTSAM 内部悬垂引用
        gtsam::Vector6 prior_var6;
        prior_var6 << prior_var, prior_var, prior_var, prior_var, prior_var, prior_var;
        
        gtsam::Vector6 between_var6;
        between_var6 << between_rotate_var, between_rotate_var, between_rotate_var, 
                        between_trans_var, between_trans_var, between_trans_var;

        // 🏛️ [架构增强] 添加子图锚点节点并固定（作为语义地标的参考坐标系）
        gtsam::SharedNoiseModel sm_prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6); 
        for (const auto& [sm_id, sm_pose] : task.submap_anchor_poses) {
            initial.insert(SM(sm_id), toGtsamPose3(sm_pose));
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(SM(sm_id), toGtsamPose3(sm_pose), sm_prior_noise));
            factor_type_log.push_back("SM_Prior(s" + std::to_string(sm_id) + ")");
        }

        for (size_t i = 0; i < sorted_kfs.size(); ++i) {
            Pose3d pose_i = poseForInitial(sorted_kfs[i]);
            initial.insert(KF(i), toGtsamPose3(pose_i));
            if (i == 0) {
                // 🔧 [修复] 为 PriorFactor 引入 Huber 鲁棒核函数
                // 原因：当 HBA 任务包含 GPS 约束或跨 Session 约束时，第一帧的 Prior 可能会与 GPS 冲突。
                // 如果 Prior 太死，LM 优化可能会为了满足 Prior 而牺牲 loop/gps 导致轨迹拉花。
                auto base_noise = gtsam::noiseModel::Diagonal::Variances(prior_var6);
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.0), base_noise);
                
                graph.add(gtsam::PriorFactor<gtsam::Pose3>(KF(0), toGtsamPose3(pose_i), robust_noise));
                factor_type_log.push_back("Prior(k0)");
            } else {
                // 🔧 [修复] 关键点：BetweenFactor 的测量值必须始终使用原始里程计 (T_odom_b)，
                // 而非初始值 (poseForInitial 可能返回已优化的位姿)。
                // 否则会产生“自引用”逻辑错误：将“上次优化的结果”作为“本次优化的真实测量”，
                // 导致轨迹无法纠偏且误差不断累积（重影根因）。
                Pose3d rel = sorted_kfs[i - 1]->T_odom_b.inverse() * sorted_kfs[i]->T_odom_b;
                auto between_noise = gtsam::noiseModel::Diagonal::Variances(between_var6);
                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(KF(i - 1), KF(i), toGtsamPose3(rel), between_noise));
                factor_type_log.push_back("Between(k" + std::to_string(i - 1) + "-k" + std::to_string(i) + ")");
            }
        }

        // 添加回环约束
        size_t loop_factors_added = 0;
        double total_loop_info_norm = 0.0;
        for (const auto& lc : task.loops) {
            if (!lc || lc->keyframe_global_id_i < 0 || lc->keyframe_global_id_j < 0) continue;
            
            // triggerAsync 已解析好全局 ID，此处映射到 task 内的关键帧索引
            auto it_i = kf_id_to_idx.find((uint64_t)lc->keyframe_global_id_i);
            auto it_j = kf_id_to_idx.find((uint64_t)lc->keyframe_global_id_j);
            
            if (it_i != kf_id_to_idx.end() && it_j != kf_id_to_idx.end()) {
                // 详细记录回环因子的测量值 vs 当前位姿，识别潜在重影风险
                Pose3d pose_i = sorted_kfs[it_i->second]->T_odom_b;
                Pose3d pose_j = sorted_kfs[it_j->second]->T_odom_b;
                Pose3d current_rel = pose_i.inverse() * pose_j;
                double trans_diff = (current_rel.translation() - lc->delta_T.translation()).norm();
                double rot_diff = Eigen::AngleAxisd(current_rel.rotation().inverse() * lc->delta_T.rotation()).angle() * 180.0 / M_PI;
                double info_norm = lc->information.norm();
                total_loop_info_norm += info_norm;

                if (trans_diff > 5.0 || rot_diff > 20.0) {
                    RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                        "[HBA][LOOP_CONFLICT] High conflict loop detected BEFORE OPT: k%zu-k%zu trans_diff=%.3fm rot_diff=%.2fdeg info_norm=%.2e (测量值与里程计差异过大，极可能导致重影)",
                        it_i->second, it_j->second, trans_diff, rot_diff, info_norm);
                }

                gtsam::Pose3 rel = toGtsamPose3(lc->delta_T);
                
                // 使用 Huber 鲁棒核函数，防止坏回环拉花地图
                auto base_noise = gtsam::noiseModel::Gaussian::Information(lc->information);
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.345), base_noise);

                graph.add(gtsam::BetweenFactor<gtsam::Pose3>(KF(it_i->second), KF(it_j->second), rel, robust_noise));
                factor_type_log.push_back("Loop(k" + std::to_string(it_i->second) + "-k" + std::to_string(it_j->second) + ")");
                loop_factors_added++;
            }
        }
        if (loop_factors_added > 0) {
            g_hba_loop_added_total.fetch_add(loop_factors_added, std::memory_order_relaxed);
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][BACKEND] Loop factors added to HBA graph count=%zu avg_info_norm=%.2e", 
                loop_factors_added, total_loop_info_norm / loop_factors_added);
        }

        // [POSE_DIAG] HBA 初始值：首/中/尾关键帧记录使用的位姿来源及数值
        {
            const size_t n_kf_init = sorted_kfs.size();
            for (size_t idx : {static_cast<size_t>(0), n_kf_init / 2, n_kf_init - 1}) {
                if (idx >= n_kf_init) continue;
                const auto& kf = sorted_kfs[idx];
                Pose3d p_init = poseForInitial(kf);
                const auto& to = kf->T_map_b_optimized.translation();
                const auto& td = kf->T_odom_b.translation();
                bool used_opt = (p_init.translation() - to).norm() < 1e-9;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][POSE_DIAG] initial idx=%zu kf_id=%lu source=%s trans=[%.4f,%.4f,%.4f] | T_odom_b=[%.4f,%.4f,%.4f] T_map_b_opt=[%.4f,%.4f,%.4f]",
                    idx, kf->id, used_opt ? "T_map_b_optimized" : "T_odom_b",
                    p_init.translation().x(), p_init.translation().y(), p_init.translation().z(),
                    td.x(), td.y(), td.z(), to.x(), to.y(), to.z());
            }
        }

        size_t gps_factors_added = 0;
        size_t gps_candidates_total = sorted_kfs.size();
        size_t gps_reject_no_valid = 0;
        size_t gps_reject_quality = 0;
        size_t gps_reject_non_finite_pos = 0;
        size_t gps_reject_non_finite_cov = 0;
        if (task.enable_gps && gps_aligned_) {
            for (size_t i = 0; i < sorted_kfs.size(); ++i) {
                const auto& kf = sorted_kfs[i];
                const double gps_dt = std::abs(kf->timestamp - kf->gps.timestamp);
                const auto decision = evaluateKeyframeGpsConstraint(
                    kf->gps, kf->has_valid_gps, gps_min_accepted_quality_level_, true,
                    gps_dt, gps_keyframe_match_window_s_);
                if (!decision.accepted) {
                    if (decision.reason == GPSConstraintRejectReason::NO_GPS_ON_KEYFRAME) gps_reject_no_valid++;
                    if (decision.reason == GPSConstraintRejectReason::QUALITY_BELOW_POLICY) gps_reject_quality++;
                    if (decision.reason == GPSConstraintRejectReason::ENU_NOT_FINITE) {
                        gps_reject_non_finite_pos++;
                        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                            "[HBA][GTSAM] skip GPS factor kf=%zu: non-finite position_enu", i);
                    }
                    if (decision.reason == GPSConstraintRejectReason::COV_NOT_FINITE) {
                        gps_reject_non_finite_cov++;
                        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                            "[HBA][GTSAM] skip GPS factor kf=%zu: non-finite covariance", i);
                    }
                    continue;
                }
                const auto& pos_enu = kf->gps.position_enu;
                // 与 iSAM2 一致：由于地图已整体变换到 ENU 系，GPS 观测直接使用 ENU 坐标即可
                // 🔧 [修复] GPS 杠臂补偿 (Lever-arm Compensation)
                // 原理：pos_imu = pos_gps_antenna - R_map_body * p_lever_arm
                // 注意：必须使用地图坐标系下的姿态 (T_map_b_optimized) 而非原始里程计 (T_odom_b)，
                // 否则当系统存在较大的对齐偏角时，补偿方向会完全错位导致重影。
                Eigen::Vector3d pos_map = pos_enu - kf->T_map_b_optimized.rotation() * lever_arm_;
                
                gtsam::Point3 pt(pos_map.x(), pos_map.y(), pos_map.z());
                Eigen::Matrix3d c = kf->gps.covariance;
                
                // 🔧 [修复] GPS Z轴降权：高度观测噪声通常远大于水平方向，放大 Z 轴方差
                double v0 = std::max(1e-6, std::min(1e6, c(0, 0)));
                double v1 = std::max(1e-6, std::min(1e6, c(1, 1)));
                double v2 = std::max(1e-6, std::min(1e6, c(2, 2))) * 20.0; // 放大20倍
                
                gtsam::Vector3 vars(v0, v1, v2);
                auto noise = gtsam::noiseModel::Diagonal::Variances(vars);
                
                // 🔧 [修复] 引入 Huber 鲁棒核函数：抑制异常 GPS 点的影响
                auto robust_noise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.0), noise);
                
                graph.add(gtsam::GPSFactor(KF(i), pt, robust_noise));
                factor_type_log.push_back("GPS(k" + std::to_string(i) + ")");
                gps_factors_added++;
            }
            if (gps_factors_added > 0) {
                g_hba_gps_added_total.fetch_add(gps_factors_added, std::memory_order_relaxed);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM][BACKEND] GPS positions in map frame (enu_to_map applied) gps_factors=%zu",
                    gps_factors_added);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][GPS_FILTER] candidates=%zu added=%zu reject_no_valid=%zu reject_quality=%zu reject_non_finite_pos=%zu reject_non_finite_cov=%zu",
                gps_candidates_total, gps_factors_added, gps_reject_no_valid, gps_reject_quality, gps_reject_non_finite_pos, gps_reject_non_finite_cov);
            if (gps_candidates_total > 0 && gps_factors_added == 0) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[DIAG][HBA][E_GPS_FACTORS_ZERO] all GPS candidates rejected: candidates=%zu "
                    "reject_no_valid=%zu reject_quality=%zu reject_non_finite_pos=%zu reject_non_finite_cov=%zu "
                    "gps_aligned=%d min_quality=%d match_window_s=%.2f",
                    gps_candidates_total, gps_reject_no_valid, gps_reject_quality,
                    gps_reject_non_finite_pos, gps_reject_non_finite_cov,
                    gps_aligned_ ? 1 : 0, gps_min_accepted_quality_level_, gps_keyframe_match_window_s_);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[CONSTRAINT_KPI][HBA] loop_added=%zu gps_added=%zu gps_reject_quality=%zu gps_min_accepted_quality_level=%d total_loop_added=%lu total_gps_added=%lu",
                loop_factors_added, gps_factors_added, gps_reject_quality,
                gps_min_accepted_quality_level_,
                static_cast<unsigned long>(g_hba_loop_added_total.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(g_hba_gps_added_total.load(std::memory_order_relaxed)));
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][GPS_FILTER] skipped task_enable_gps=%d gps_aligned=%d candidates=%zu",
                task.enable_gps ? 1 : 0, gps_aligned_ ? 1 : 0, gps_candidates_total);
        }

        // 🏛️ [架构增强] 添加语义地标约束 (CylinderFactor)
        // Keep consistency with IncrementalOptimizer: apply DCS-like dynamic down-weight
        // from current initial residual to prevent wrong semantic pulls in fallback path.
        constexpr double kSemanticSwitchableResidualScaleM = 0.25;
        auto compute_cylinder_residual = [&](size_t kf_idx, int sm_id, const CylinderFactorItemKF& factor) -> double {
            if (!initial.exists(KF(kf_idx)) || !initial.exists(SM(sm_id))) return 0.0;
            const gtsam::Pose3 T_map_kf = initial.at<gtsam::Pose3>(KF(kf_idx));
            const gtsam::Pose3 T_map_sm = initial.at<gtsam::Pose3>(SM(sm_id));
            const gtsam::Point3 p_body(factor.point_body.x(), factor.point_body.y(), factor.point_body.z());
            const gtsam::Point3 p_map = T_map_kf.transformFrom(p_body);
            const gtsam::Point3 p_sm = T_map_sm.transformTo(p_map);
            const Eigen::Vector3d p(p_sm.x(), p_sm.y(), p_sm.z());
            const Eigen::Vector3d root = factor.root_submap;
            Eigen::Vector3d ray = factor.ray_submap;
            if (!ray.allFinite() || ray.norm() < 1e-6) return 0.0;
            ray.normalize();
            const double dist_to_axis = (p - root).cross(ray).norm();
            return std::abs(dist_to_axis - std::max(1e-6, factor.radius));
        };
        auto compute_plane_residual = [&](size_t kf_idx, int sm_id, const PlaneFactorItemKF& factor) -> double {
            if (!initial.exists(KF(kf_idx)) || !initial.exists(SM(sm_id))) return 0.0;
            const gtsam::Pose3 T_map_kf = initial.at<gtsam::Pose3>(KF(kf_idx));
            const gtsam::Pose3 T_map_sm = initial.at<gtsam::Pose3>(SM(sm_id));
            const gtsam::Point3 p_body(factor.point_body.x(), factor.point_body.y(), factor.point_body.z());
            const gtsam::Point3 p_map = T_map_kf.transformFrom(p_body);
            const gtsam::Point3 p_sm = T_map_sm.transformTo(p_map);
            Eigen::Vector3d n = factor.normal_submap;
            if (!n.allFinite() || n.norm() < 1e-6) return 0.0;
            n.normalize();
            const double signed_dist = n.dot(Eigen::Vector3d(p_sm.x(), p_sm.y(), p_sm.z())) + factor.distance_submap;
            return std::abs(signed_dist);
        };
        size_t landmark_factors_added = 0;
        size_t plane_landmark_factors_added = 0;
        double semantic_residual_sum = 0.0;
        size_t semantic_residual_cnt = 0;
        for (const auto& factor : task.semantic_factors) {
            auto it_kf = kf_id_to_idx.find(factor.kf_id);
            if (it_kf == kf_id_to_idx.end()) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[HBA][SEMANTIC][VALIDATION] kf_id=%lu not found in HBA task sorted_kfs, skip landmark factor", factor.kf_id);
                continue;
            }

            if (task.submap_anchor_poses.find(static_cast<int>(factor.sm_id)) == task.submap_anchor_poses.end()) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[HBA][SEMANTIC][VALIDATION] sm_id=%lu anchor not found in HBA task, skip landmark factor", factor.sm_id);
                continue;
            }

            // 数值合法性检查
            if (!factor.point_body.allFinite() || !factor.root_submap.allFinite() || !factor.ray_submap.allFinite()) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[HBA][SEMANTIC][VALIDATION] Landmark factor for kf=%lu sm=%lu has NaN/Inf, skip", factor.kf_id, factor.sm_id);
                continue;
            }

            // 1D Isotropic Noise Model + DCS-like switch (align with IncrementalOptimizer)
            const double residual = compute_cylinder_residual(it_kf->second, static_cast<int>(factor.sm_id), factor);
            const double c = std::max(1e-3, kSemanticSwitchableResidualScaleM);
            const double switch_scale = 1.0 / (1.0 + (residual * residual) / (c * c));
            double sigma = 0.1 / std::max(1e-3, factor.weight);
            sigma = sigma / std::sqrt(std::max(0.05, switch_scale));
            auto noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);

            gtsam::Point3 point_body(factor.point_body.x(), factor.point_body.y(), factor.point_body.z());
            gtsam::Point3 root_submap(factor.root_submap.x(), factor.root_submap.y(), factor.root_submap.z());
            gtsam::Unit3 ray_submap(factor.ray_submap.x(), factor.ray_submap.y(), factor.ray_submap.z());

            graph.add(boost::make_shared<CylinderFactor>(
                KF(it_kf->second), SM(static_cast<int>(factor.sm_id)),
                point_body, root_submap, ray_submap, factor.radius, noise));
            
            factor_type_log.push_back("Landmark(k" + std::to_string(it_kf->second) + "-s" + std::to_string(factor.sm_id) + ")");
            landmark_factors_added++;
            semantic_residual_sum += residual;
            semantic_residual_cnt++;
        }
        for (const auto& factor : task.semantic_plane_factors) {
            auto it_kf = kf_id_to_idx.find(factor.kf_id);
            if (it_kf == kf_id_to_idx.end()) continue;
            if (task.submap_anchor_poses.find(static_cast<int>(factor.sm_id)) == task.submap_anchor_poses.end()) continue;
            if (!factor.point_body.allFinite() || !factor.normal_submap.allFinite() ||
                factor.normal_submap.norm() < 1e-6 || !std::isfinite(factor.distance_submap)) {
                continue;
            }
            const double residual = compute_plane_residual(it_kf->second, static_cast<int>(factor.sm_id), factor);
            const double c = std::max(1e-3, kSemanticSwitchableResidualScaleM);
            const double switch_scale = 1.0 / (1.0 + (residual * residual) / (c * c));
            double sigma = 0.15 / std::max(1e-3, factor.weight);
            sigma = sigma / std::sqrt(std::max(0.05, switch_scale));
            auto noise = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
            const Eigen::Vector3d n = factor.normal_submap.normalized();
            gtsam::Point3 point_body(factor.point_body.x(), factor.point_body.y(), factor.point_body.z());
            gtsam::Unit3 normal_submap(n.x(), n.y(), n.z());
            graph.add(boost::make_shared<PlaneFactor>(
                KF(it_kf->second), SM(static_cast<int>(factor.sm_id)),
                point_body, normal_submap, factor.distance_submap, noise));
            factor_type_log.push_back("PlaneLandmark(k" + std::to_string(it_kf->second) + "-s" + std::to_string(factor.sm_id) + ")");
            plane_landmark_factors_added++;
            semantic_residual_sum += residual;
            semantic_residual_cnt++;
        }

        if (landmark_factors_added > 0 || plane_landmark_factors_added > 0) {
            const double mean_semantic_residual =
                semantic_residual_cnt > 0 ? (semantic_residual_sum / static_cast<double>(semantic_residual_cnt)) : 0.0;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][BACKEND] Semantic landmark factors added to HBA graph cylinder=%zu plane=%zu mean_residual=%.3f switch_scale_m=%.2f",
                landmark_factors_added, plane_landmark_factors_added, mean_semantic_residual, kSemanticSwitchableResidualScaleM);
        }

        size_t n_factors = graph.size();
        size_t n_values = initial.size();
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] graph built: factors=%zu values=%zu loop_factors=%zu gps_factors=%zu landmark_factors(cyl/plane)=(%zu/%zu) (building LM optimizer...)",
            n_factors, n_values, loop_factors_added, gps_factors_added, landmark_factors_added, plane_landmark_factors_added);
        ALOG_INFO(MOD, "HBA GTSAM: factors={} values={} loops={} gps={} landmarks={}", 
                  n_factors, n_values, loop_factors_added, gps_factors_added, landmark_factors_added);

        // 诊断：逐因子打印类型，便于崩溃时定位是哪一个 factor 在 error() 路径触发 double free
        for (size_t idx = 0; idx < graph.size(); ++idx) {
            const char* type_str = idx < factor_type_log.size() ? factor_type_log[idx].c_str() : "?";
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM] factor[%zu] type=%s", idx, type_str);
        }

        // 可选：在 LM 构造前逐因子计算 error，若崩溃则最后一条 factor_error 即肇事因子（与 LM 构造内 graph.error() 同路径）
        const char* diag_env = std::getenv("AUTOMAP_HBA_FACTOR_ERROR_DIAG");
        bool run_factor_error_diag = (diag_env && (std::string(diag_env) == "1" || std::string(diag_env) == "true"));
        if (run_factor_error_diag) {
            double total_err = 0;
            for (size_t idx = 0; idx < graph.size(); ++idx) {
                const char* type_str = idx < factor_type_log.size() ? factor_type_log[idx].c_str() : "?";
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM] factor_error idx=%zu type=%s (pre-call)", idx, type_str);
                fflush(stdout);
                double e = graph[idx]->error(initial);
                total_err += e * e;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GTSAM] factor_error idx=%zu type=%s err_sq=%.6g", idx, type_str, e * e);
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM] factor_error total_sq=%.6g (LM constructor next)", total_err);
        }

        // 传入 graph/initial 的副本，避免 LM 内部持有对栈上对象的引用导致与 ISAM2 双路共用时的 double free（与 commitAndUpdate 中 graph_copy/values_copy 一致）
        gtsam::NonlinearFactorGraph graph_copy(graph);
        gtsam::Values initial_copy(initial);
        std::string key_sample;
        { size_t n = 0; for (gtsam::Key k : initial_copy.keys()) { if (n++) key_sample += ","; key_sample += std::to_string(k); if (n >= 5) { key_sample += ",..."; break; } } }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] pre-LM: graph_copy.size=%zu initial_copy.size=%zu initial_keys_sample=[%s]",
            graph_copy.size(), initial_copy.size(), key_sample.c_str());

        // 约束合理性：所有因子的 key 均在 initial 中，且所有 value 有限、平移在合理范围内
        bool key_ok = true;
        for (size_t idx = 0; idx < graph_copy.size() && key_ok; ++idx) {
            for (gtsam::Key k : graph_copy[idx]->keys()) {
                if (!initial_copy.exists(k)) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[HBA][BACKEND][VALIDATION] GTSAM fallback: factor[%zu] key %zu not in initial, abort",
                        idx, static_cast<size_t>(k));
                    key_ok = false;
                    break;
                }
            }
        }
        if (!key_ok) {
            ALOG_ERROR(MOD, "HBA GTSAM: factor key not in initial, skip");
            return result;
        }
        bool values_ok = true;
        for (gtsam::Key k : initial_copy.keys()) {
            try {
                auto p = initial_copy.at<gtsam::Pose3>(k);
                Eigen::Vector3d t = p.translation();
                Eigen::Matrix3d R = p.rotation().matrix();
                if (!t.array().isFinite().all() || !R.array().isFinite().all()) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[HBA][BACKEND][VALIDATION] GTSAM fallback: initial value key=%zu non-finite, abort",
                        static_cast<size_t>(k));
                    values_ok = false;
                    break;
                }
                if (t.norm() > kMaxReasonableTranslationNorm) {
                    RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                        "[HBA][BACKEND][VALIDATION] GTSAM fallback: initial value key=%zu translation norm=%.1f > %.0f, abort",
                        static_cast<size_t>(k), t.norm(), kMaxReasonableTranslationNorm);
                    values_ok = false;
                    break;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                    "[HBA][BACKEND][VALIDATION] GTSAM fallback: initial value key=%zu exception: %s, abort",
                    static_cast<size_t>(k), e.what());
                values_ok = false;
                break;
            }
        }
        if (!values_ok) {
            ALOG_ERROR(MOD, "HBA GTSAM: initial values validation failed, skip");
            return result;
        }

        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] LevenbergMarquardtOptimizer constructor enter (若崩溃在此后、无 exit→崩溃在 LM 构造/error 内)");
        fflush(stdout);

        gtsam::LevenbergMarquardtOptimizer opt(graph_copy, initial_copy);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GTSAM] LevenbergMarquardtOptimizer constructor exit (LM built ok)");

        // [GHOSTING_DIAG] 记录优化前各类因子的残差
        auto logFactorErrors = [&](const gtsam::NonlinearFactorGraph& g, const gtsam::Values& v, const std::string& label) {
            double total_err = 0, prior_err = 0, odom_err = 0, gps_err = 0, loop_err = 0, landmark_err = 0;
            size_t p_cnt = 0, o_cnt = 0, g_cnt = 0, l_cnt = 0, m_cnt = 0;
            
            for (size_t idx = 0; idx < g.size(); ++idx) {
                if (!g[idx]) continue;
                double e = g[idx]->error(v);
                total_err += e;
                
                std::string type = (idx < factor_type_log.size()) ? factor_type_log[idx] : "unknown";
                if (type.find("Prior") != std::string::npos) { prior_err += e; p_cnt++; }
                else if (type.find("Between") != std::string::npos) { odom_err += e; o_cnt++; }
                else if (type.find("GPS") != std::string::npos) { gps_err += e; g_cnt++; }
                else if (type.find("Loop") != std::string::npos) { loop_err += e; l_cnt++; }
                else if (type.find("Landmark") != std::string::npos) { landmark_err += e; m_cnt++; }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][ERROR_BREAKDOWN][%s] total=%.2f prior=%.2f(%zu) odom=%.2f(%zu) gps=%.2f(%zu) loop=%.2f(%zu) landmark=%.2f(%zu)",
                label.c_str(), total_err, prior_err, p_cnt, odom_err, o_cnt, gps_err, g_cnt, loop_err, l_cnt, landmark_err, m_cnt);
        };

        logFactorErrors(graph_copy, initial_copy, "BEFORE");

        gtsam::Values optimized;
        try {
            gtsam::LevenbergMarquardtOptimizer opt(graph_copy, initial_copy);
            optimized = opt.optimize();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM] LM optimize() exit iterations done. final_error=%.6g", graph_copy.error(optimized));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[HBA][GTSAM][EXCEPTION] LM Optimization CRASHED: %s. Returning empty result to prevent system death.", e.what());
            return result;
        }

        logFactorErrors(graph_copy, optimized, "AFTER");

        // [GHOSTING_DIAG] 写回前抽样记录 T_odom_b vs 即将写回的 T_map_b_optimized，便于定位重影是否来自写回不一致
        const size_t kSampleStep = std::max<size_t>(1, sorted_kfs.size() / 8);
        for (size_t i = 0; i < sorted_kfs.size(); ++i) {
            if (i % kSampleStep == 0 && optimized.exists(KF(i))) {
                const auto& kf = sorted_kfs[i];
                Pose3d new_pose = fromGtsamPose3(optimized.at<gtsam::Pose3>(KF(i)));
                double trans_diff = (new_pose.translation() - kf->T_odom_b.translation()).norm();
                double rot_diff = Eigen::AngleAxisd(new_pose.linear() * kf->T_odom_b.linear().transpose()).angle() * 180.0 / M_PI;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[HBA][GHOSTING_DIAG] pre_writeback kf_id=%lu idx=%zu: T_odom_b=[%.2f,%.2f,%.2f] -> T_map_b_optimized=[%.2f,%.2f,%.2f] trans_diff=%.3fm rot_diff=%.2fdeg",
                    kf->id, i,
                    kf->T_odom_b.translation().x(), kf->T_odom_b.translation().y(), kf->T_odom_b.translation().z(),
                    new_pose.translation().x(), new_pose.translation().y(), new_pose.translation().z(),
                    trans_diff, rot_diff);
            }
        }

        // 详细记录回环因子的残差，识别哪些回环被优化器认为“不可信”
        if (loop_factors_added > 0) {
            double total_loop_error = 0.0;
            int high_residual_count = 0;
            for (size_t idx = 0; idx < graph_copy.size(); ++idx) {
                if (idx < factor_type_log.size() && factor_type_log[idx].find("Loop") != std::string::npos) {
                    double err = graph_copy[idx]->error(optimized);
                    total_loop_error += err;
                    if (err > 1.0) { // 残差较大
                        high_residual_count++;
                        
                        // [GHOSTING_DIAG] 提取该因子的节点 ID
                        auto factor = dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(graph_copy[idx].get());
                        std::string nodes_str = "unknown";
                        if (factor) {
                            nodes_str = fmt::format("k{}-k{}", factor->key1(), factor->key2());
                        }

                        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                            "[HBA][LOOP_RESIDUAL] factor[%zu] nodes=%s residual=%.4f (High residual indicates conflict with odom/GPS/other loops)", 
                            idx, nodes_str.c_str(), err);
                        
                        // [GHOSTING_DIAG] 记录该坏因子的具体位姿偏移，便于量化对重影的贡献
                        if (err > 10.0) {
                            std::string info_str = "N/A";
                            if (factor) {
                                auto noise = dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor->noiseModel().get());
                                if (noise) {
                                    Eigen::MatrixXd info = noise->information();
                                    info_str = fmt::format("diag=[{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}]", 
                                        info(0,0), info(1,1), info(2,2), info(3,3), info(4,4), info(5,5));
                                }
                            }
                            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                                "[HBA][CRITICAL_LOOP_ERROR] EXTREME residual=%.2f at %s. Info: %s. This loop is definitely conflicting and likely causing ghosting!",
                                err, nodes_str.c_str(), info_str.c_str());
                        }
                    }
                }
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][LOOP_SUMMARY] total_loops=%zu high_residual=%d avg_err=%.4f",
                loop_factors_added, high_residual_count, total_loop_error / loop_factors_added);
        }

        const size_t n_kf = sorted_kfs.size();
        for (size_t i = 0; i < n_kf; ++i) {
            if (optimized.exists(KF(i))) {
                Pose3d new_pose = fromGtsamPose3(optimized.at<gtsam::Pose3>(KF(i)));
                result.optimized_poses.push_back(new_pose);
                result.optimized_keyframe_ids.push_back(sorted_kfs[i]->id);
            }
        }
        // [POSE_DIAG] 写回由 updateAllFromHBA 持锁统一执行；此处用 result.optimized_poses 打首/中/尾与 T_odom_b 的差值
        for (size_t idx : {static_cast<size_t>(0), n_kf / 2, n_kf - 1}) {
            if (idx >= n_kf || idx >= result.optimized_poses.size()) continue;
            const auto& kf = sorted_kfs[idx];
            const auto& t_new = result.optimized_poses[idx].translation();
            const auto& t_odom = kf->T_odom_b.translation();
            double trans_diff = (t_new - t_odom).norm();
            double yaw_new = std::atan2(result.optimized_poses[idx].rotation()(1, 0), result.optimized_poses[idx].rotation()(0, 0)) * 180.0 / M_PI;
            double yaw_odom = std::atan2(kf->T_odom_b.rotation()(1, 0), kf->T_odom_b.rotation()(0, 0)) * 180.0 / M_PI;
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[HBA][POSE_DIAG] writeback idx=%zu kf_id=%lu T_map_b_optimized=[%.4f,%.4f,%.4f] yaw=%.2fdeg | T_odom_b=[%.4f,%.4f,%.4f] yaw=%.2fdeg trans_diff=%.4fm",
                idx, kf->id, t_new.x(), t_new.y(), t_new.z(), yaw_new, t_odom.x(), t_odom.y(), t_odom.z(), yaw_odom, trans_diff);
        }
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][GHOSTING_DIAG] runHBA no KF write here poses=%zu (writeback in updateAllFromHBA under mutex；若重影请 grep GHOSTING_DIAG 核对 writeback_enter/done 与 pose_snapshot_taken 时间线)",
            result.optimized_poses.size());
        result.success = true;
        auto t_end = std::chrono::steady_clock::now();
        result.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        result.final_mme = 0.0; // fallback 路径暂不计算 MME
        scope.setSuccess(true);
        BACKEND_STEP("step=HBA_runHBA_done backend=GTSAM_fallback success=1 MME=%.4f elapsed_ms=%.1f poses=%zu", 
                     result.final_mme, result.elapsed_ms, result.optimized_poses.size());
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND] GTSAM fallback done: poses=%zu elapsed=%.1fms", result.optimized_poses.size(), result.elapsed_ms);
        ALOG_INFO(MOD, "HBA GTSAM fallback done: optimized_poses={} elapsed={:.1f}ms", result.optimized_poses.size(), result.elapsed_ms);
    } catch (const std::exception& e) {
        BACKEND_STEP("step=HBA_runHBA_done backend=GTSAM_fallback success=0 exception=%s", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] GTSAM fallback failed: %s (grep HBA BACKEND EXCEPTION)", e.what());
        ALOG_ERROR(MOD, "HBA GTSAM fallback exception: {}", e.what());
        fprintf(stderr, "[HBAOptimizer] GTSAM fallback exception: %s\n", e.what());
    } catch (...) {
        BACKEND_STEP("step=HBA_runHBA_done backend=GTSAM_fallback success=0 exception=unknown");
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[HBA][BACKEND][EXCEPTION] GTSAM fallback unknown exception (若为 double free，检查 FIX_GPS_BATCH_SIGSEGV 与 pre-LM 日志，grep HBA BACKEND EXCEPTION)");
        ALOG_ERROR(MOD, "HBA GTSAM fallback: unknown exception");
        fprintf(stderr, "[HBAOptimizer] GTSAM fallback: unknown exception\n");
    }
    return result;
}
#endif

std::vector<KeyFrame::Ptr> HBAOptimizer::collectKeyFramesFromSubmaps(
    const std::vector<SubMap::Ptr>& submaps) const
{
    std::vector<KeyFrame::Ptr> kfs;
    for (const auto& sm : submaps) {
        if (!sm || sm->keyframes.empty()) continue;
        for (const auto& kf : sm->keyframes) {
            // 跳过无效关键帧：空点云或无效位姿
            if (!kf) continue;
            if (!kf->cloud_body || kf->cloud_body->empty()) continue;
            // 检查位姿有效性：平移和旋转必须是有限的
            const auto& t = kf->T_odom_b.translation();
            const auto& R = kf->T_odom_b.rotation();
            if (!t.allFinite() || !R.allFinite()) continue;
            kfs.push_back(kf);
        }
    }

    // 按时间戳排序
    std::sort(kfs.begin(), kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    // 去重：相同时间戳只保留一个
    if (kfs.size() > 1) {
        std::vector<KeyFrame::Ptr> unique_kfs;
        unique_kfs.push_back(kfs[0]);
        for (size_t i = 1; i < kfs.size(); ++i) {
            if (std::abs(kfs[i]->timestamp - unique_kfs.back()->timestamp) > 0.001) {
                unique_kfs.push_back(kfs[i]);
            }
        }
        kfs.swap(unique_kfs);
    }

    return kfs;
}

} // namespace automap_pro
