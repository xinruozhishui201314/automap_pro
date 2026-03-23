#include "automap_pro/backend/hba_bridge.h"
#include "automap_pro/core/utils.h"

#include <pcl/io/pcd_io.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstdlib>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>
#include <cstring>

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
        const auto& T = kf->T_odom_b;
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
        keyframes[i]->T_map_b_optimized = poses[i];
    return true;
}

bool HBABridge::runHBAProcess(
    const std::string& data_path,
    const std::string& command_template) {
    // 白名单：仅允许路径安全字符，避免命令注入
    for (char c : data_path) {
        if (!(std::isalnum(static_cast<unsigned char>(c)) ||
              c == '/' || c == '_' || c == '-' || c == '.')) {
            fprintf(stderr,
                    "[HBABridge] runHBAProcess: unsafe character '%c' in data_path, abort\n",
                    c);
            return false;
        }
    }

    char path_buf[4096];
    std::snprintf(path_buf, sizeof(path_buf), command_template.c_str(), data_path.c_str());

    // 将命令行拆成 argv[]（按空格），不经过 shell，完全消除注入面
    std::vector<std::string> tokens;
    for (const char* p = path_buf; *p;) {
        while (*p == ' ' || *p == '\t') ++p;
        if (!*p) break;
        const char* start = p;
        while (*p && *p != ' ' && *p != '\t') ++p;
        tokens.emplace_back(start, static_cast<size_t>(p - start));
    }
    if (tokens.empty()) {
        fprintf(stderr, "[HBABridge] runHBAProcess: empty command after substitute\n");
        return false;
    }

    std::vector<char*> argv;
    for (std::string& t : tokens)
        argv.push_back(&t[0]);
    argv.push_back(nullptr);

    pid_t pid = 0;
    int err = posix_spawnp(&pid, argv[0], nullptr, nullptr, argv.data(), environ);
    if (err != 0) {
        fprintf(stderr, "[HBABridge] runHBAProcess: posix_spawnp failed err=%d\n", err);
        return false;
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) {
        fprintf(stderr, "[HBABridge] runHBAProcess: waitpid failed\n");
        return false;
    }
    return WIFEXITED(status) && (WEXITSTATUS(status) == 0);
}

}  // namespace automap_pro
