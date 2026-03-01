#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <functional>
#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <automap_pro/msg/sub_map_event_msg.hpp>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

class SubMapManager {
public:
    using SubmapFrozenCallback = std::function<void(const SubMap::Ptr&)>;
    using SubmapUpdatedCallback = std::function<void(const SubMap::Ptr&)>;

    SubMapManager();
    ~SubMapManager() = default;

    void init(rclcpp::Node::SharedPtr node, int session_id = 0);

    void addKeyFrame(const KeyFrame::Ptr& kf);
    void checkAndSplitSubmap();
    SubMap::Ptr createNewSubmap();
    void updateSubmapPose(int submap_id, const Pose3d& new_anchor_pose);
    bool archiveSubmap(int submap_id, const std::string& dir);
    SubMap::Ptr loadSubmap(const std::string& path, bool load_cloud = true);

    SubMap::Ptr       activeSubmap() const;
    SubMap::Ptr       submap(int id)  const;
    std::vector<SubMap::Ptr> frozenSubmaps() const;
    std::vector<SubMap::Ptr> allSubmaps()    const;

    int numSubmaps()    const;
    int numKeyFrames()  const;

    void registerFrozenCallback(SubmapFrozenCallback cb);
    void registerUpdatedCallback(SubmapUpdatedCallback cb);

    void reset();

private:
    mutable std::mutex mutex_;
    std::vector<SubMap::Ptr>    submaps_;
    SubMap::Ptr                 active_submap_;
    std::atomic<int>            submap_id_counter_{0};

    int session_id_ = 0;
    int max_kf_;
    double max_spatial_;
    double max_temporal_;
    double match_resolution_;
    std::string output_dir_;

    std::vector<SubmapFrozenCallback>  frozen_cbs_;
    std::vector<SubmapUpdatedCallback> updated_cbs_;

    rclcpp::Publisher<automap_pro::msg::SubMapEventMsg>::SharedPtr submap_event_pub_;
    rclcpp::Logger logger_{rclcpp::get_logger("automap_pro.submap_manager")};
    rclcpp::Node::SharedPtr node_;
    void publishSubmapEvent(const SubMap::Ptr& sm, const std::string& event);
};

}  // namespace automap_pro
