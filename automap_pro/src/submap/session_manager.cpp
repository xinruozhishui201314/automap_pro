/**
 * @file submap/session_manager.cpp
 * @brief 子图与会话实现。
 */
#include "automap_pro/submap/session_manager.h"
#include "automap_pro/core/utils.h"

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>

namespace automap_pro {

SessionManager::SessionManager() = default;

int SessionManager::startNewSession(const std::string& data_dir) {
    std::lock_guard<std::mutex> lk(mutex_);
    int sid = static_cast<int>(sessions_.size());
    SessionInfo info;
    info.session_id = sid;
    info.data_dir   = data_dir;
    info.loaded     = true;
    sessions_[sid]  = info;
    current_session_id_ = sid;
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[SessionManager] Started session %d at %s", sid, data_dir.c_str());
    return sid;
}

bool SessionManager::loadSession(const std::string& session_dir, int session_id) {
    std::string meta_path = session_dir + "/session_meta.json";
    if (!utils::fileExists(meta_path)) {
        RCLCPP_WARN(rclcpp::get_logger("automap_pro"), "[SessionManager] No session metadata at %s", meta_path.c_str());
        return false;
    }

    std::ifstream ifs(meta_path);
    nlohmann::json meta;
    try { ifs >> meta; } catch (...) { return false; }

    std::lock_guard<std::mutex> lk(mutex_);
    SessionInfo info;
    info.session_id = session_id;
    info.data_dir   = session_dir;
    info.loaded     = true;
    if (meta.contains("num_keyframes"))  info.num_keyframes = meta["num_keyframes"];
    if (meta.contains("t_start"))        info.t_start = meta["t_start"];
    if (meta.contains("t_end"))          info.t_end   = meta["t_end"];
    if (meta.contains("submap_ids")) {
        for (int id : meta["submap_ids"]) info.submap_ids.push_back(id);
    }
    sessions_[session_id] = info;
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[SessionManager] Loaded session %d from %s (%d submaps)",
             session_id, session_dir.c_str(), (int)info.submap_ids.size());
    return true;
}

bool SessionManager::saveSession(int session_id, const std::string& output_dir,
                                  const std::vector<SubMap::Ptr>& submaps) {
    utils::createDirectories(output_dir);

    SessionInfo info;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        auto it = sessions_.find(session_id);
        if (it != sessions_.end()) info = it->second;
    }

    nlohmann::json meta;
    meta["session_id"] = session_id;
    meta["data_dir"]   = output_dir;

    int total_kf = 0;
    double t_start = 1e18, t_end = 0.0;
    nlohmann::json sm_ids;

    for (const auto& sm : submaps) {
        if (sm->session_id != session_id) continue;
        sm_ids.push_back(sm->id);
        total_kf += sm->keyframes.size();
        if (sm->t_start < t_start) t_start = sm->t_start;
        if (sm->t_end   > t_end)   t_end   = sm->t_end;
    }

    meta["num_keyframes"] = total_kf;
    meta["t_start"]       = t_start < 1e17 ? t_start : 0.0;
    meta["t_end"]         = t_end;
    meta["submap_ids"]    = sm_ids;

    std::ofstream ofs(output_dir + "/session_meta.json");
    ofs << meta.dump(2);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[SessionManager] Saved session %d to %s", session_id, output_dir.c_str());
    return true;
}

void SessionManager::registerSubmap(int session_id, int submap_id) {
    std::lock_guard<std::mutex> lk(mutex_);
    sessions_[session_id].submap_ids.push_back(submap_id);
}

int SessionManager::currentSessionId() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return current_session_id_;
}

std::vector<SessionManager::SessionInfo> SessionManager::allSessions() const {
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<SessionInfo> out;
    for (const auto& [id, info] : sessions_) out.push_back(info);
    return out;
}

SessionManager::SessionInfo SessionManager::getSession(int session_id) const {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = sessions_.find(session_id);
    if (it != sessions_.end()) return it->second;
    return {};
}

bool SessionManager::saveDescriptorDB(const std::string& path,
                                       const std::vector<SubMap::Ptr>& submaps) const {
    nlohmann::json db;
    for (const auto& sm : submaps) {
        if (!sm->has_descriptor) continue;
        nlohmann::json entry;
        entry["submap_id"]  = sm->id;
        entry["session_id"] = sm->session_id;
        entry["has_gps"]    = sm->has_valid_gps;
        entry["gps_center"] = {sm->gps_center.x(), sm->gps_center.y(), sm->gps_center.z()};
        std::vector<float> desc(sm->overlap_descriptor.data(),
                                 sm->overlap_descriptor.data() + sm->overlap_descriptor.size());
        entry["descriptor"] = desc;
        db.push_back(entry);
    }
    std::ofstream ofs(path);
    ofs << db.dump(2);
    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[SessionManager] Saved %zu descriptors to %s", submaps.size(), path.c_str());
    return true;
}

bool SessionManager::loadDescriptorDB(const std::string& path,
                                       std::vector<SubMap::Ptr>& submaps) {
    if (!utils::fileExists(path)) return false;
    std::ifstream ifs(path);
    nlohmann::json db;
    try { ifs >> db; } catch (...) { return false; }

    for (const auto& entry : db) {
        auto sm = std::make_shared<SubMap>();
        sm->id         = entry["submap_id"];
        sm->session_id = entry["session_id"];
        sm->has_valid_gps = entry["has_gps"];
        sm->gps_center = Eigen::Vector3d(
            entry["gps_center"][0], entry["gps_center"][1], entry["gps_center"][2]);

        const auto& desc_vec = entry["descriptor"];
        sm->overlap_descriptor.resize(desc_vec.size());
        for (size_t i = 0; i < desc_vec.size(); ++i) {
            sm->overlap_descriptor(i) = desc_vec[i];
        }
        sm->has_descriptor = true;
        submaps.push_back(sm);
    }

    RCLCPP_INFO(rclcpp::get_logger("automap_pro"), "[SessionManager] Loaded %zu descriptors from %s",
             submaps.size(), path.c_str());
    return true;
}

}  // namespace automap_pro
