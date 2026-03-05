#include "automap_pro/core/utils.h"
#include <filesystem>
#include <Eigen/Geometry>

namespace automap_pro {

namespace utils {

CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size) {
    if (!cloud || cloud->empty() || leaf_size <= 0.0f) return cloud;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    auto out = std::make_shared<CloudXYZI>();
    vg.filter(*out);
    return out;
}

bool fileExists(const std::string& path) {
    return std::filesystem::exists(path);
}

void createDirectories(const std::string& path) {
    std::filesystem::create_directories(path);
}

Vec6d poseToVec6d(const Pose3d& T) {
    Vec6d v;
    v.head<3>() = T.translation();
    Eigen::Vector3d euler = T.linear().eulerAngles(0, 1, 2);
    v.tail<3>() = euler;
    return v;
}

Pose3d vec6dToPose(const Vec6d& v) {
    Pose3d T = Pose3d::Identity();
    T.translation() = v.head<3>();
    Eigen::AngleAxisd rx(v(3), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(v(4), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(v(5), Eigen::Vector3d::UnitZ());
    T.linear() = (rz * ry * rx).toRotationMatrix();
    return T;
}

}  // namespace utils

}  // namespace automap_pro
