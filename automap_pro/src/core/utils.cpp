#include "automap_pro/core/utils.h"
#include "automap_pro/core/logger.h"
#include <filesystem>
#include <Eigen/Geometry>
#include <system_error>

namespace automap_pro {

namespace utils {

CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size) {
    try {
        if (!cloud || cloud->empty() || leaf_size <= 0.0f) {
            ALOG_WARN("Utils", "voxelDownsample: invalid input (cloud={:p}, size={}, leaf_size={})",
                      static_cast<const void*>(cloud.get()), 
                      cloud ? cloud->size() : 0, leaf_size);
            return cloud;
        }
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        auto out = std::make_shared<CloudXYZI>();
        vg.filter(*out);
        
        if (out->empty()) {
            ALOG_WARN("Utils", "voxelDownsample: output cloud is empty after filtering");
        }
        return out;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "voxelDownsample exception: {}", e.what());
        return cloud;
    } catch (...) {
        ALOG_ERROR("Utils", "voxelDownsample unknown exception");
        return cloud;
    }
}

bool fileExists(const std::string& path) {
    try {
        if (path.empty()) {
            ALOG_WARN("Utils", "fileExists: empty path provided");
            return false;
        }
        return std::filesystem::exists(path);
    } catch (const std::filesystem::filesystem_error& e) {
        ALOG_ERROR("Utils", "fileExists filesystem error for '{}': {}", path, e.what());
        return false;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "fileExists exception for '{}': {}", path, e.what());
        return false;
    }
}

void createDirectories(const std::string& path) {
    try {
        if (path.empty()) {
            ALOG_ERROR("Utils", "createDirectories: empty path provided");
            throw std::invalid_argument("createDirectories: path cannot be empty");
        }
        std::filesystem::create_directories(path);
        ALOG_DEBUG("Utils", "createDirectories: created '{}'", path);
    } catch (const std::filesystem::filesystem_error& e) {
        ALOG_ERROR("Utils", "createDirectories filesystem error for '{}': {}", path, e.what());
        throw;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "createDirectories exception for '{}': {}", path, e.what());
        throw;
    }
}

Vec6d poseToVec6d(const Pose3d& T) {
    Vec6d v = Vec6d::Zero();
    try {
        v.head<3>() = T.translation();
        
        // 检查旋转矩阵是否有效
        if (!T.linear().allFinite()) {
            ALOG_WARN("Utils", "poseToVec6d: rotation matrix contains NaN/Inf, using identity");
            return v;
        }
        
        Eigen::Vector3d euler = T.linear().eulerAngles(0, 1, 2);
        if (!euler.allFinite()) {
            ALOG_WARN("Utils", "poseToVec6d: euler angles contain NaN/Inf");
            euler.setZero();
        }
        v.tail<3>() = euler;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "poseToVec6d exception: {}", e.what());
    }
    return v;
}

Pose3d vec6dToPose(const Vec6d& v) {
    Pose3d T = Pose3d::Identity();
    try {
        if (!v.allFinite()) {
            ALOG_WARN("Utils", "vec6dToPose: input vector contains NaN/Inf, returning identity");
            return T;
        }
        
        T.translation() = v.head<3>();
        
        // 检查角度值是否合理（避免极端值）
        constexpr double kMaxAngle = 4 * M_PI;  // 允许2圈
        double rx_val = std::clamp(v(3), -kMaxAngle, kMaxAngle);
        double ry_val = std::clamp(v(4), -kMaxAngle, kMaxAngle);
        double rz_val = std::clamp(v(5), -kMaxAngle, kMaxAngle);
        
        Eigen::AngleAxisd rx(rx_val, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(ry_val, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(rz_val, Eigen::Vector3d::UnitZ());
        
        Eigen::Matrix3d R = (rz * ry * rx).toRotationMatrix();
        
        // 验证旋转矩阵有效性
        if (!R.allFinite()) {
            ALOG_WARN("Utils", "vec6dToPose: computed rotation matrix invalid, using identity");
            return T;
        }
        
        // 正交性检查（可选，用于调试）
        double ortho_error = (R * R.transpose() - Eigen::Matrix3d::Identity()).norm();
        if (ortho_error > 0.01) {
            ALOG_WARN("Utils", "vec6dToPose: rotation matrix not orthogonal (error={:.6f})", ortho_error);
        }
        
        T.linear() = R;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "vec6dToPose exception: {}", e.what());
        T = Pose3d::Identity();
    }
    return T;
}

}  // namespace utils

}  // namespace automap_pro
