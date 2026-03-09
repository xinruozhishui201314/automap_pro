#include "automap_pro/backend/hba_bridge.h"
#include "automap_pro/core/utils.h"

#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstdlib>

namespace automap_pro {

std::vector<KeyFrame::Ptr> HBABridge::collectKeyframesInOrder(
    const std::vector<SubMap::Ptr>& submaps) {
    std::vector<KeyFrame::Ptr> out;
    for (const auto& sm : submaps) {
        for (const auto& kf : sm->keyframes)
            out.push_back(kf);
    }
    std::sort(out.begin(), out.end(),
              [](const KeyFrame::Ptr& a, const KeyFrame::Ptr& b) {
                  return a->timestamp < b->timestamp;
              });
    return out;
}

bool HBABridge::exportToHBAFormat(
    const std::string& data_path,
    const std::vector<KeyFrame::Ptr>& keyframes,
    int pcd_name_fill_num) {
    if (keyframes.empty()) return false;
    if (data_path.back() != '/') return exportToHBAFormat(data_path + "/", keyframes, pcd_name_fill_num);

    utils::createDirectories(data_path);
    utils::createDirectories(data_path + "pcd/");

    std::ofstream pose_file(data_path + "pose.json");
    if (!pose_file) return false;

    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        const auto& T = kf->T_w_b;
        const Eigen::Vector3d t = T.translation();
        Eigen::Quaterniond q(T.rotation());
        pose_file << std::fixed << std::setprecision(6)
                  << kf->timestamp << " "
                  << t.x() << " " << t.y() << " " << t.z() << " "
                  << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
        if (i + 1 < keyframes.size()) pose_file << "\n";
    }
    pose_file.close();

    for (size_t i = 0; i < keyframes.size(); ++i) {
        if (!keyframes[i]->cloud_body || keyframes[i]->cloud_body->empty())
            continue;
        std::ostringstream ss;
        ss << std::setw(pcd_name_fill_num) << std::setfill('0') << i;
        std::string pcd_path = data_path + "pcd/" + ss.str() + ".pcd";
        if (pcl::io::savePCDFileBinary(pcd_path, *keyframes[i]->cloud_body) < 0)
            return false;
    }
    return true;
}

bool HBABridge::loadHBAResultAndApply(
    const std::string& data_path,
    std::vector<KeyFrame::Ptr>& keyframes) {
    std::string path = data_path;
    if (path.back() != '/') path += "/";
    std::ifstream file(path + "pose_trans.json");
    if (!file) return false;

    std::vector<Pose3d> poses;
    double lt, tx, ty, tz, qx, qy, qz, qw;
    while (file >> lt >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        Pose3d T;
        T.translation() = Eigen::Vector3d(tx, ty, tz);
        T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
        poses.push_back(T);
    }
    file.close();
    if (poses.size() != keyframes.size()) return false;

    for (size_t i = 0; i < keyframes.size(); ++i)
        keyframes[i]->T_w_b_optimized = poses[i];
    return true;
}

bool HBABridge::runHBAProcess(
    const std::string& data_path,
    const std::string& command_template) {
    char path_buf[4096];
    snprintf(path_buf, sizeof(path_buf), command_template.c_str(), data_path.c_str());
    int ret = std::system(path_buf);
    return (ret == 0);
}

}  // namespace automap_pro
