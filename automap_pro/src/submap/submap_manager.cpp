#include "automap_pro/submap/submap_manager.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/utils.h"

#include <automap_pro/msg/sub_map_event_msg.hpp>
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <fstream>

namespace automap_pro {

SubMapManager::SubMapManager() {
    const auto& cfg = ConfigManager::instance();
    max_kf_         = cfg.submapMaxKeyframes();
    max_spatial_    = cfg.submapMaxSpatialExtent();
    max_temporal_   = cfg.submapMaxTemporalExtent();
    match_resolution_ = cfg.submapMatchResolution();
    output_dir_     = cfg.outputDir();
}

void SubMapManager::init(rclcpp::Node::SharedPtr node, int session_id) {
    node_ = node;
    logger_ = node->get_logger();
    session_id_ = session_id;
    submap_event_pub_ = node->create_publisher<automap_pro::msg::SubMapEventMsg>("/automap/submap_event", 10);
    active_submap_ = createNewSubmap();
    RCLCPP_INFO(logger_, "[SubMapManager] Init session %d, first submap id=%d", session_id_, 0);
}

SubMap::Ptr SubMapManager::createNewSubmap() {
    auto sm = std::make_shared<SubMap>();
    sm->id         = submap_id_counter_++;
    sm->session_id = session_id_;
    sm->state      = SubMapState::ACTIVE;
    sm->merged_cloud     = std::make_shared<CloudXYZI>();
    sm->downsampled_cloud = std::make_shared<CloudXYZI>();

    std::lock_guard<std::mutex> lk(mutex_);
    submaps_.push_back(sm);
    publishSubmapEvent(sm, "CREATED");
    RCLCPP_INFO(logger_, "[SubMapManager] Created submap id=%d session=%d", sm->id, sm->session_id);
    return sm;
}

void SubMapManager::addKeyFrame(const KeyFrame::Ptr& kf) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!active_submap_) return;

    active_submap_->addKeyFrame(kf);

    // Merge cloud into submap
    if (kf->cloud_ds_body && !kf->cloud_ds_body->empty()) {
        auto cloud_world = utils::transformCloud(kf->cloud_ds_body, kf->T_w_b);
        *active_submap_->merged_cloud += *cloud_world;
    }
}

void SubMapManager::checkAndSplitSubmap() {
    SubMap::Ptr to_freeze;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (!active_submap_) return;
        if (!active_submap_->isFull(max_kf_, max_spatial_, max_temporal_)) return;

        // Freeze current submap
        active_submap_->freeze();
        // Compute downsampled cloud for matching
        active_submap_->downsampled_cloud =
            utils::voxelDownsample(active_submap_->merged_cloud, match_resolution_);

        to_freeze = active_submap_;
    }

    if (to_freeze) {
        RCLCPP_INFO(logger_, "[SubMapManager] Freezing submap %d (%zu kf, %.1f m, %.1f s)",
                 to_freeze->id, to_freeze->keyframes.size(),
                 to_freeze->spatial_extent, to_freeze->t_end - to_freeze->t_start);
        publishSubmapEvent(to_freeze, "FROZEN");
        for (auto& cb : frozen_cbs_) cb(to_freeze);
    }

    // Create new active submap
    active_submap_ = createNewSubmap();
}

void SubMapManager::updateSubmapPose(int submap_id, const Pose3d& new_anchor_pose) {
    std::lock_guard<std::mutex> lk(mutex_);
    for (auto& sm : submaps_) {
        if (sm->id == submap_id) {
            sm->updateAnchorPose(new_anchor_pose);
            sm->reproject();
            publishSubmapEvent(sm, "UPDATED");
            for (auto& cb : updated_cbs_) cb(sm);
            break;
        }
    }
}

bool SubMapManager::archiveSubmap(int submap_id, const std::string& dir) {
    SubMap::Ptr sm;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        for (auto& s : submaps_) {
            if (s->id == submap_id) { sm = s; break; }
        }
    }
    if (!sm) return false;

    std::string sm_dir = dir + "/submap_" + std::to_string(submap_id);
    utils::createDirectories(sm_dir);

    // Save metadata as JSON
    nlohmann::json meta;
    meta["id"]         = sm->id;
    meta["session_id"] = sm->session_id;
    meta["state"]      = static_cast<int>(sm->state);
    meta["t_start"]    = sm->t_start;
    meta["t_end"]      = sm->t_end;
    meta["spatial_extent"] = sm->spatial_extent;
    meta["anchor_keyframe_id"] = sm->anchor_keyframe_id;
    meta["has_valid_gps"] = sm->has_valid_gps;
    meta["num_keyframes"] = sm->keyframes.size();

    // Anchor pose
    const auto& M = sm->pose_w_anchor_optimized.matrix();
    std::vector<double> mat_vec(M.data(), M.data() + 16);
    meta["anchor_pose"] = mat_vec;

    // GPS center
    meta["gps_center"] = {sm->gps_center.x(), sm->gps_center.y(), sm->gps_center.z()};

    // Descriptor
    if (sm->has_descriptor) {
        std::vector<float> desc(sm->overlap_descriptor.data(),
                                 sm->overlap_descriptor.data() + sm->overlap_descriptor.size());
        meta["descriptor"] = desc;
    }

    std::ofstream ofs(sm_dir + "/meta.json");
    ofs << meta.dump(2);

    // Save downsampled cloud
    if (sm->downsampled_cloud && !sm->downsampled_cloud->empty()) {
        pcl::io::savePCDFileBinary(sm_dir + "/cloud_match.pcd", *sm->downsampled_cloud);
    }

    // Save keyframe poses
    nlohmann::json kf_json;
    for (const auto& kf : sm->keyframes) {
        nlohmann::json kf_entry;
        kf_entry["id"]        = kf->id;
        kf_entry["timestamp"] = kf->timestamp;
        const auto& Tkf = kf->T_w_b_optimized.matrix();
        std::vector<double> kf_mat(Tkf.data(), Tkf.data() + 16);
        kf_entry["pose"] = kf_mat;
        kf_entry["has_gps"] = kf->has_valid_gps;
        kf_json.push_back(kf_entry);
    }
    std::ofstream kf_ofs(sm_dir + "/keyframes.json");
    kf_ofs << kf_json.dump(2);

    sm->state = SubMapState::ARCHIVED;
    publishSubmapEvent(sm, "ARCHIVED");
    RCLCPP_INFO(logger_, "[SubMapManager] Archived submap %d to %s", submap_id, sm_dir.c_str());
    return true;
}

SubMap::Ptr SubMapManager::activeSubmap() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return active_submap_;
}

SubMap::Ptr SubMapManager::submap(int id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& sm : submaps_) {
        if (sm->id == id) return sm;
    }
    return nullptr;
}

std::vector<SubMap::Ptr> SubMapManager::frozenSubmaps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<SubMap::Ptr> out;
    for (const auto& sm : submaps_) {
        if (sm->state == SubMapState::FROZEN || sm->state == SubMapState::UPDATED) {
            out.push_back(sm);
        }
    }
    return out;
}

std::vector<SubMap::Ptr> SubMapManager::allSubmaps() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return submaps_;
}

int SubMapManager::numSubmaps()    const { std::lock_guard<std::mutex> lk(mutex_); return submaps_.size(); }
int SubMapManager::numKeyFrames()  const {
    std::lock_guard<std::mutex> lk(mutex_);
    int n = 0;
    for (const auto& sm : submaps_) n += sm->keyframes.size();
    return n;
}

void SubMapManager::registerFrozenCallback (SubmapFrozenCallback  cb) { frozen_cbs_.push_back(std::move(cb)); }
void SubMapManager::registerUpdatedCallback(SubmapUpdatedCallback cb) { updated_cbs_.push_back(std::move(cb)); }

void SubMapManager::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    submaps_.clear();
    active_submap_.reset();
    submap_id_counter_.store(0);
}

void SubMapManager::publishSubmapEvent(const SubMap::Ptr& sm, const std::string& event) {
    automap_pro::msg::SubMapEventMsg msg;
    msg.header.stamp = node_->now();
    msg.submap_id    = sm->id;
    msg.session_id   = sm->session_id;
    msg.event_type   = event;
    msg.keyframe_count = static_cast<int>(sm->keyframes.size());
    submap_event_pub_->publish(msg);
}

}  // namespace automap_pro
