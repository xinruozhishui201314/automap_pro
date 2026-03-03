#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#define MOD "SubMapMgr"

#include <automap_pro/msg/sub_map_event_msg.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace automap_pro {

SubMapManager::SubMapManager() {
    const auto& cfg = ConfigManager::instance();
    max_kf_       = cfg.submapMaxKF();
    max_spatial_  = cfg.submapMaxSpatial();
    max_temporal_ = cfg.submapMaxTemporal();
    match_res_    = cfg.submapMatchRes();
    merge_res_    = cfg.submapMergeRes();
}

void SubMapManager::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    event_pub_ = node->create_publisher<automap_pro::msg::SubMapEventMsg>(
        "/automap/submap_event", 50);
}

void SubMapManager::startNewSession(uint64_t session_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    current_session_id_ = session_id;
    active_submap_ = nullptr;  // 新 session 重新开始子图
}

void SubMapManager::addKeyFrame(const KeyFrame::Ptr& kf) {
    std::lock_guard<std::mutex> lk(mutex_);

    // 分配全局唯一 KF ID
    kf->id         = kf_id_counter_++;
    kf->session_id = current_session_id_;

    // 如果没有活跃子图，创建一个
    if (!active_submap_) {
        active_submap_ = createNewSubmap(kf);
        submaps_.push_back(active_submap_);
    }

    // 添加关键帧到子图
    kf->submap_id = active_submap_->id;
    active_submap_->keyframes.push_back(kf);
    active_submap_->t_end = kf->timestamp;

    // 更新锚定位姿（第一帧）
    if (active_submap_->keyframes.size() == 1) {
        active_submap_->pose_w_anchor           = kf->T_w_b;
        active_submap_->pose_w_anchor_optimized = kf->T_w_b;
        kf->is_anchor = true;
    }

    // 更新 GPS 中心（所有有效 GPS 的平均值）
    if (kf->has_valid_gps) {
        size_t gps_count = 0;
        Eigen::Vector3d gps_sum = Eigen::Vector3d::Zero();
        for (const auto& f : active_submap_->keyframes) {
            if (f->has_valid_gps) { gps_sum += f->gps.position_enu; gps_count++; }
        }
        if (gps_count > 0) {
            active_submap_->gps_center = gps_sum / gps_count;
            active_submap_->has_valid_gps = true;
        }
    }

    // 合并点云
    mergeCloudToSubmap(active_submap_, kf);

    // 更新空间范围（最近帧到锚定帧的最大距离）
    double dist = (kf->T_w_b.translation() -
                   active_submap_->pose_w_anchor.translation()).norm();
    active_submap_->spatial_extent_m = std::max(active_submap_->spatial_extent_m, dist);

    ALOG_DEBUG(MOD, "KF#{} added to SM#{}: kf_count={} dist={:.1fm}",
               kf->id, active_submap_->id,
               active_submap_->keyframes.size(), active_submap_->spatial_extent_m);

    if (isFull(active_submap_)) {
        ALOG_INFO(MOD, "SM#{} FULL (kf={} dist={:.1f}m) → freezing",
                  active_submap_->id, active_submap_->keyframes.size(),
                  active_submap_->spatial_extent_m);
        freezeActiveSubmap();
        active_submap_ = nullptr;
    }
}

bool SubMapManager::isFull(const SubMap::Ptr& sm) const {
    if ((int)sm->keyframes.size() >= max_kf_)         return true;
    if (sm->spatial_extent_m >= max_spatial_)         return true;
    if (!sm->keyframes.empty()) {
        double dt = sm->t_end - sm->t_start;
        if (dt >= max_temporal_)                      return true;
    }
    return false;
}

void SubMapManager::freezeActiveSubmap() {
    if (!active_submap_ || active_submap_->state != SubMapState::ACTIVE) return;
    AUTOMAP_TIMED_SCOPE(MOD, fmt::format("FreezeSubmap#{}", active_submap_->id), 500.0);

    // 降采样点云（用于回环匹配）
    if (active_submap_->merged_cloud && !active_submap_->merged_cloud->empty()) {
        CloudXYZIPtr ds(new CloudXYZI);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(active_submap_->merged_cloud);
        vg.setLeafSize(match_res_, match_res_, match_res_);
        vg.filter(*ds);
        active_submap_->downsampled_cloud = ds;
    }

    active_submap_->state = SubMapState::FROZEN;
    publishEvent(active_submap_, "FROZEN");

    // 触发回调（→ LoopDetector, HBAOptimizer, IncrementalOptimizer）
    for (auto& cb : frozen_cbs_) cb(active_submap_);
}

SubMap::Ptr SubMapManager::createNewSubmap(const KeyFrame::Ptr& first_kf) {
    auto sm = std::make_shared<SubMap>();
    sm->id          = submap_id_counter_++;
    sm->session_id  = current_session_id_;
    sm->state       = SubMapState::ACTIVE;
    sm->t_start     = first_kf->timestamp;
    sm->t_end       = first_kf->timestamp;
    sm->merged_cloud = std::make_shared<CloudXYZI>();
    publishEvent(sm, "CREATED");
    return sm;
}

void SubMapManager::mergeCloudToSubmap(SubMap::Ptr& sm, const KeyFrame::Ptr& kf) const {
    if (!kf->cloud_body || kf->cloud_body->empty()) return;

    // 将 body 系点云变换到世界系
    CloudXYZI world_cloud;
    Eigen::Affine3f T_wf;
    T_wf.matrix() = kf->T_w_b.cast<float>().matrix();
    pcl::transformPointCloud(*kf->cloud_body, world_cloud, T_wf);

    *sm->merged_cloud += world_cloud;

    // 如果点云过大，降采样（避免 OOM）
    if (sm->merged_cloud->size() > 500000) {
        CloudXYZIPtr temp(new CloudXYZI);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(sm->merged_cloud);
        vg.setLeafSize(merge_res_, merge_res_, merge_res_);
        vg.filter(*temp);
        sm->merged_cloud = temp;
    }
}

void SubMapManager::updateSubmapPose(int submap_id, const Pose3d& new_pose) {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& sm : submaps_) {
        if (sm->id != submap_id) continue;

        Pose3d old_anchor = sm->pose_w_anchor_optimized;
        sm->pose_w_anchor_optimized = new_pose;
        sm->state = SubMapState::OPTIMIZED;

        // 更新子图内所有关键帧的优化位姿
        Pose3d delta = new_pose * old_anchor.inverse();
        for (auto& kf : sm->keyframes) {
            kf->T_w_b_optimized = delta * kf->T_w_b_optimized;
        }
        publishEvent(sm, "OPTIMIZED");
        break;
    }
}

void SubMapManager::updateAllFromHBA(const HBAResult& result) {
    if (!result.success) return;
    std::lock_guard<std::mutex> lk(mutex_);

    // 收集所有 KF（按时间排序，与 HBA 输入一致）
    std::vector<KeyFrame::Ptr> all_kfs;
    for (const auto& sm : submaps_) {
        for (const auto& kf : sm->keyframes) {
            all_kfs.push_back(kf);
        }
    }
    std::sort(all_kfs.begin(), all_kfs.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });

    // 更新每个 KF 的优化位姿
    for (size_t i = 0; i < all_kfs.size() && i < result.optimized_poses.size(); ++i) {
        all_kfs[i]->T_w_b_optimized = result.optimized_poses[i];
    }

    // 更新每个子图的锚定位姿（取第一帧的优化位姿）
    for (auto& sm : submaps_) {
        if (!sm->keyframes.empty()) {
            sm->pose_w_anchor_optimized = sm->keyframes.front()->T_w_b_optimized;
            sm->state = SubMapState::OPTIMIZED;
        }
    }
}

SubMap::Ptr SubMapManager::getActiveSubmap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return active_submap_;
}

SubMap::Ptr SubMapManager::getSubmap(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& sm : submaps_) {
        if (sm->id == id) return sm;
    }
    return nullptr;
}

std::vector<SubMap::Ptr> SubMapManager::getAllSubmaps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return submaps_;
}

std::vector<SubMap::Ptr> SubMapManager::getFrozenSubmaps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<SubMap::Ptr> out;
    for (const auto& sm : submaps_) {
        if (sm->state == SubMapState::FROZEN ||
            sm->state == SubMapState::OPTIMIZED)
            out.push_back(sm);
    }
    return out;
}

int SubMapManager::submapCount() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return static_cast<int>(submaps_.size());
}

int SubMapManager::keyframeCount() const {
    return static_cast<int>(kf_id_counter_.load());
}

// ─────────────────────────────────────────────────────────────────────────────
// 子图持久化（MS-Mapping 增量式核心）
// ─────────────────────────────────────────────────────────────────────────────
bool SubMapManager::archiveSubmap(const SubMap::Ptr& sm, const std::string& dir) {
    std::string sm_dir = dir + "/submap_" + std::to_string(sm->id) + "/";
    fs::create_directories(sm_dir);

    // 1. meta.json（子图元数据）
    json meta;
    meta["id"]          = sm->id;
    meta["session_id"]  = sm->session_id;
    meta["kf_count"]    = sm->keyframes.size();
    meta["t_start"]     = sm->t_start;
    meta["t_end"]       = sm->t_end;
    meta["spatial_m"]   = sm->spatial_extent_m;
    meta["has_gps"]     = sm->has_valid_gps;
    meta["gps_center"]  = {sm->gps_center.x(), sm->gps_center.y(), sm->gps_center.z()};

    // 锚定位姿（4×4矩阵）
    Eigen::Matrix4d M = sm->pose_w_anchor_optimized.matrix();
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            meta["anchor_pose_matrix"][r*4+c] = M(r,c);

    // 描述子
    std::vector<float> desc(sm->overlap_descriptor.data(),
                             sm->overlap_descriptor.data() + sm->overlap_descriptor.size());
    meta["descriptor"] = desc;
    meta["has_descriptor"] = sm->has_descriptor;

    // 关键帧位姿列表
    json kf_list = json::array();
    for (const auto& kf : sm->keyframes) {
        json kf_json;
        kf_json["id"]        = kf->id;
        kf_json["timestamp"] = kf->timestamp;
        kf_json["has_gps"]   = kf->has_valid_gps;
        Eigen::Matrix4d Tkf = kf->T_w_b_optimized.matrix();
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                kf_json["pose_matrix"][r*4+c] = Tkf(r,c);
        kf_list.push_back(kf_json);
    }
    meta["keyframes"] = kf_list;

    std::ofstream mf(sm_dir + "meta.json");
    if (!mf) return false;
    mf << meta.dump(2);
    mf.close();

    // 2. cloud_match.pcd（降采样点云，用于跨 session 回环）
    if (sm->downsampled_cloud && !sm->downsampled_cloud->empty()) {
        pcl::io::savePCDFileBinary(sm_dir + "cloud_match.pcd",
                                    *sm->downsampled_cloud);
    }

    auto& sm_mut = const_cast<SubMap&>(*sm);
    sm_mut.state = SubMapState::ARCHIVED;
    return true;
}

bool SubMapManager::loadArchivedSubmap(
    const std::string& dir, int submap_id, SubMap::Ptr& out)
{
    std::string sm_dir = dir + "/submap_" + std::to_string(submap_id) + "/";
    std::ifstream mf(sm_dir + "meta.json");
    if (!mf) return false;

    json meta;
    mf >> meta;
    mf.close();

    out = std::make_shared<SubMap>();
    out->id         = meta["id"];
    out->session_id = meta["session_id"];
    out->t_start    = meta["t_start"];
    out->t_end      = meta["t_end"];
    out->spatial_extent_m = meta.value("spatial_m", 0.0);
    out->has_valid_gps    = meta.value("has_gps", false);
    out->gps_center = Eigen::Vector3d(
        meta["gps_center"][0], meta["gps_center"][1], meta["gps_center"][2]);

    // 锚定位姿
    Eigen::Matrix4d M;
    for (int i = 0; i < 16; ++i) M(i/4, i%4) = meta["anchor_pose_matrix"][i];
    out->pose_w_anchor_optimized.matrix() = M;
    out->pose_w_anchor = out->pose_w_anchor_optimized;

    // 描述子
    if (meta.contains("descriptor")) {
        const auto& d = meta["descriptor"];
        out->overlap_descriptor.resize(d.size());
        for (size_t i = 0; i < d.size(); ++i) out->overlap_descriptor(i) = d[i];
        out->has_descriptor = meta.value("has_descriptor", false);
    }

    // 点云
    std::string pcd_path = sm_dir + "cloud_match.pcd";
    if (fs::exists(pcd_path)) {
        out->downsampled_cloud = std::make_shared<CloudXYZI>();
        pcl::io::loadPCDFile(pcd_path, *out->downsampled_cloud);
    }

    out->state = SubMapState::ARCHIVED;
    return true;
}

CloudXYZIPtr SubMapManager::buildGlobalMap(float voxel_size) const {
    std::lock_guard<std::mutex> lk(mutex_);
    CloudXYZIPtr global(new CloudXYZI);

    for (const auto& sm : submaps_) {
        for (const auto& kf : sm->keyframes) {
            if (!kf->cloud_body || kf->cloud_body->empty()) continue;
            CloudXYZI world_cloud;
            Eigen::Affine3f T;
            T.matrix() = kf->T_w_b_optimized.cast<float>().matrix();
            pcl::transformPointCloud(*kf->cloud_body, world_cloud, T);
            *global += world_cloud;
        }
    }

    if (!global->empty() && voxel_size > 0) {
        CloudXYZIPtr ds(new CloudXYZI);
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(global);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*ds);
        return ds;
    }
    return global;
}

void SubMapManager::publishEvent(const SubMap::Ptr& sm, const std::string& event) {
    if (!event_pub_) return;
    automap_pro::msg::SubMapEventMsg msg;
    msg.header.stamp  = rclcpp::Clock().now();
    msg.submap_id     = sm->id;
    msg.session_id    = sm->session_id;
    msg.event_type    = event;
    msg.keyframe_count = static_cast<int>(sm->keyframes.size());
    msg.spatial_extent_m = sm->spatial_extent_m;
    msg.has_valid_gps = sm->has_valid_gps;
    event_pub_->publish(msg);
}

} // namespace automap_pro
