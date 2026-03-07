#include "automap_pro/core/utils.h"
#include "automap_pro/core/logger.h"
#include <filesystem>
#include <Eigen/Geometry>
#include <system_error>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <tuple>
#include <cstdint>

namespace automap_pro {

namespace utils {

namespace {

// PCL VoxelGrid 在 applyFilter 中检查 (dx*dy*dz) > INT_MAX 会告警并返回；且后续 div_b_/min_b_/max_b_
// 使用 int，单轴过大仍可能导致线性索引或分配溢出并 SIGSEGV。PCL 源码: dx*dy*dz > std::numeric_limits<int>::max() 即告警。
// 保守取单轴上限为 INT_MAX 的立方根，使 dx*dy*dz <= INT_MAX，避免触发 PCL 告警与崩溃。
constexpr int kIntMaxCubeRoot = 1290;  // (1290^3 ≈ 2.15e9 >= INT_MAX)
constexpr float kMaxVoxelAxisIndex = static_cast<float>(kIntMaxCubeRoot);
constexpr float kDefaultMaxAbsCoord = 1e6f;

// 体素网格键：用于无 PCL 的体素下采样，避免 PCL VoxelGrid::filter 写越界导致析构时 SIGSEGV。
using VoxelKey = std::tuple<int64_t, int64_t, int64_t>;

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        const auto [ix, iy, iz] = k;
        return static_cast<size_t>(ix * 73856093ULL ^ iy * 19349663ULL ^ iz * 83492791ULL);
    }
};

/** 不调用 PCL VoxelGrid 的体素下采样：每体素保留第一个点，返回新点云，避免 PCL 堆损坏。 */
CloudXYZIPtr voxelDownsampleGridNoPCL(const CloudXYZIPtr& input, float leaf) {
    if (!input || input->empty() || leaf <= 0.f) return input;
    auto out = std::make_shared<CloudXYZI>();
    std::unordered_map<VoxelKey, size_t, VoxelKeyHash> voxel_to_index;
    voxel_to_index.reserve(std::min(input->size() / 4, size_t(500000)));
    const float inv_leaf = 1.f / leaf;
    for (size_t i = 0; i < input->points.size(); ++i) {
        const auto& p = input->points[i];
        int64_t ix = static_cast<int64_t>(std::floor(p.x * inv_leaf));
        int64_t iy = static_cast<int64_t>(std::floor(p.y * inv_leaf));
        int64_t iz = static_cast<int64_t>(std::floor(p.z * inv_leaf));
        VoxelKey key{ix, iy, iz};
        if (voxel_to_index.emplace(key, i).second)
            out->push_back(p);
    }
    return out;
}

}  // namespace

CloudXYZIPtr sanitizePointCloudForVoxel(const CloudXYZIPtr& cloud, float max_abs_coord) {
    if (!cloud || cloud->empty()) return cloud;
    auto out = std::make_shared<CloudXYZI>();
    out->reserve(cloud->size());
    int removed = 0;
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            ++removed;
            continue;
        }
        pcl::PointXYZI q;
        q.x = std::clamp(p.x, -max_abs_coord, max_abs_coord);
        q.y = std::clamp(p.y, -max_abs_coord, max_abs_coord);
        q.z = std::clamp(p.z, -max_abs_coord, max_abs_coord);
        q.intensity = std::isfinite(p.intensity) ? p.intensity : 0.0f;
        out->push_back(q);
    }
    if (removed > 0) {
        ALOG_WARN("Utils", "sanitizePointCloudForVoxel: removed {} non-finite points, kept {}", removed, out->size());
    }
    return out;
}

bool voxelGridWouldOverflow(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, float leaf_size) {
    if (leaf_size <= 0.0f) return true;
    float dx = (max_x - min_x) / leaf_size;
    float dy = (max_y - min_y) / leaf_size;
    float dz = (max_z - min_z) / leaf_size;
    return (dx > kMaxVoxelAxisIndex || dy > kMaxVoxelAxisIndex || dz > kMaxVoxelAxisIndex ||
            !std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz));
}

CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size) {
    try {
        if (!cloud || cloud->empty()) {
            ALOG_WARN("Utils", "voxelDownsample: invalid input (cloud={:p}, size={})",
                      static_cast<const void*>(cloud.get()),
                      cloud ? cloud->size() : 0);
            return cloud;
        }
        float leaf = std::max(leaf_size, kMinVoxelLeafSize);
        if (leaf_size < kMinVoxelLeafSize) {
            ALOG_DEBUG("Utils", "voxelDownsample: leaf_size={:.3f} clamped to {:.3f}", leaf_size, leaf);
        }
        CloudXYZIPtr input = sanitizePointCloudForVoxel(cloud, kDefaultMaxAbsCoord);
        if (!input || input->empty()) {
            ALOG_WARN("Utils", "voxelDownsample: cloud empty after sanitization");
            return cloud;
        }
        float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f, min_z = 1e9f, max_z = -1e9f;
        for (const auto& p : input->points) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        }
        for (int attempt = 0; attempt < 8; ++attempt) {
            if (!voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf)) break;
            leaf *= 2.0f;
            if (attempt == 0) {
                ALOG_WARN("Utils", "voxelDownsample: overflow risk with leaf={:.4f}, retrying with larger leaf", leaf_size);
            }
        }
        if (voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf)) {
            ALOG_WARN("Utils", "voxelDownsample: still overflow risk after enlarging leaf, returning copy without voxel filter");
            return input;
        }
        // 使用无 PCL 的体素下采样，避免 PCL VoxelGrid::filter 在部分 leaf/范围下写越界导致析构时 SIGSEGV。
        CloudXYZIPtr out = voxelDownsampleGridNoPCL(input, leaf);
        if (out && out->empty()) {
            ALOG_WARN("Utils", "voxelDownsample: output cloud is empty after filtering");
        }
        return out ? out : input;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "voxelDownsample exception: {}", e.what());
        return cloud;
    } catch (...) {
        ALOG_ERROR("Utils", "voxelDownsample unknown exception");
        return cloud;
    }
}

// 点云点数低于此值时直接单次体素滤波，不走分块循环，避免 PCL 在部分 chunk 上 SIGSEGV
static constexpr size_t kVoxelChunkedSizeThreshold = 250000u;

CloudXYZIPtr voxelDownsampleChunked(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m) {
    const unsigned tid = automap_pro::logThreadId();
    if (!cloud || cloud->empty()) return cloud;
    ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=enter in={} leaf={:.3f} chunk_m={:.1f}",
              tid, cloud->size(), leaf_size, chunk_size_m);
    float leaf = std::max(leaf_size, kMinVoxelLeafSize);
    if (chunk_size_m <= 0.f) {
        return voxelDownsample(cloud, leaf);
    }
    try {
        CloudXYZIPtr input = sanitizePointCloudForVoxel(cloud, kDefaultMaxAbsCoord);
        if (!input || input->empty()) return cloud;
        ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=sanitized in={}", tid, input->size());

        float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f, min_z = 1e9f, max_z = -1e9f;
        for (const auto& p : input->points) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        }
        // 仅当点数少且空间范围不会溢出时 bypass：否则必须走分块→逐块降采样→合并，避免 overflow
        if (input->size() <= kVoxelChunkedSizeThreshold &&
            !voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf)) {
            return voxelDownsample(cloud, leaf);
        }

        float span_x = max_x - min_x;
        float span_y = max_y - min_y;
        float span_z = max_z - min_z;
        if (span_x <= 0.f && span_y <= 0.f && span_z <= 0.f) return input;

        float cs = std::max(chunk_size_m, leaf * 2.f);
        int nx = std::max(1, static_cast<int>(std::ceil(span_x / cs)));
        int ny = std::max(1, static_cast<int>(std::ceil(span_y / cs)));
        int nz = std::max(1, static_cast<int>(std::ceil(span_z / cs)));
        // 块数过多时放大块尺寸直至块数可接受，避免回退到单次 voxelDownsample（大范围会溢出）
        const int kMaxChunks = 10000;
        while (nx * ny * nz > kMaxChunks && cs < std::max({span_x, span_y, span_z}) * 2.f) {
            cs *= 2.f;
            nx = std::max(1, static_cast<int>(std::ceil(span_x / cs)));
            ny = std::max(1, static_cast<int>(std::ceil(span_y / cs)));
            nz = std::max(1, static_cast<int>(std::ceil(span_z / cs)));
        }
        if (nx * ny * nz > kMaxChunks) {
            ALOG_WARN("Utils", "voxelDownsampleChunked: chunk grid nx={} ny={} nz={} still > {}, proceed anyway", nx, ny, nz, kMaxChunks);
        }

        auto merged = std::make_shared<CloudXYZI>();
        merged->reserve(input->size() / 4);

        // ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk_loop_enter nx={} ny={} nz={}", nx, ny, nz);
        int chunk_index = 0;
        for (int ix = 0; ix < nx; ++ix) {
            float cx_min = min_x + ix * cs;
            float cx_max = (ix + 1 == nx) ? max_x : (min_x + (ix + 1) * cs);
            for (int iy = 0; iy < ny; ++iy) {
                float cy_min = min_y + iy * cs;
                float cy_max = (iy + 1 == ny) ? max_y : (min_y + (iy + 1) * cs);
                for (int iz = 0; iz < nz; ++iz) {
                    float cz_min = min_z + iz * cs;
                    float cz_max = (iz + 1 == nz) ? max_z : (min_z + (iz + 1) * cs);

                    ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ix={} iy={} iz={} idx={} fill_enter", ix, iy, iz, chunk_index);
                    auto chunk = std::make_shared<CloudXYZI>();
                    chunk->reserve(std::min(input->size() / (static_cast<size_t>(nx) * ny * nz + 1), input->size()));
                    const auto& pts_ref = input->points;
                    const size_t num_pts = pts_ref.size();
                    for (size_t i = 0; i < num_pts; ++i) {
                        const auto& p = pts_ref[i];
                        if (p.x >= cx_min && p.x <= cx_max && p.y >= cy_min && p.y <= cy_max && p.z >= cz_min && p.z <= cz_max)
                            chunk->push_back(p);
                    }
                    ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ix={} iy={} iz={} idx={} fill_done pts={}", ix, iy, iz, chunk_index, chunk->size());
                    if (chunk->empty()) {
                        ++chunk_index;
                        continue;
                    }
                    ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ix={} iy={} iz={} idx={} voxel_enter", ix, iy, iz, chunk_index);
                    CloudXYZIPtr ds;
                    try {
                        ds = voxelDownsample(chunk, leaf);
                    } catch (const std::exception& e) {
                        // ALOG_ERROR("Utils", "[MAP] voxelDownsampleChunked chunk idx={} voxelDownsample exception: {}, use chunk as-is", chunk_index, e.what());
                        ds = chunk;
                    } catch (...) {
                        // ALOG_ERROR("Utils", "[MAP] voxelDownsampleChunked chunk idx={} voxelDownsample unknown exception, use chunk as-is", chunk_index);
                        ds = chunk;
                    }
                    // ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ix={} iy={} iz={} idx={} voxel_done out={}", ix, iy, iz, chunk_index, ds ? ds->size() : 0u);
                    if (ds && !ds->empty()) {
                        // ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ix={} iy={} iz={} idx={} merge_enter", ix, iy, iz, chunk_index);
                        const auto& ds_pts = ds->points;
                        const size_t ds_n = ds_pts.size();
                        merged->reserve(merged->size() + ds_n);
                        for (size_t i = 0; i < ds_n; ++i)
                            merged->push_back(ds_pts[i]);
                        // ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ix={} iy={} iz={} idx={} merge_done", ix, iy, iz, chunk_index);
                    }
                    ++chunk_index;
                }
            }
        }

        // ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk_merge_done merged={}", merged->size());
        if (merged->empty()) return input;
        // 合并后全图 AABB 仍可能过大，最终去重仅在不溢出时做，否则直接返回 merged（块边界少量重复可接受）
        float m_min_x = 1e9f, m_max_x = -1e9f, m_min_y = 1e9f, m_max_y = -1e9f, m_min_z = 1e9f, m_max_z = -1e9f;
        for (const auto& p : merged->points) {
            m_min_x = std::min(m_min_x, p.x); m_max_x = std::max(m_max_x, p.x);
            m_min_y = std::min(m_min_y, p.y); m_max_y = std::max(m_max_y, p.y);
            m_min_z = std::min(m_min_z, p.z); m_max_z = std::max(m_max_z, p.z);
        }
        CloudXYZIPtr result;
        if (!voxelGridWouldOverflow(m_min_x, m_max_x, m_min_y, m_max_y, m_min_z, m_max_z, leaf)) {
            ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=final_voxel_enter merged={}", tid, merged->size());
            result = voxelDownsample(merged, leaf);
        } else {
            ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=skip_final_voxel (extent overflow risk) merged={}", tid, merged->size());
            result = merged;
        }
        ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=exit out={}", tid, result ? result->size() : 0u);
        return result;
    } catch (const std::exception& e) {
        // ALOG_ERROR("Utils", "voxelDownsampleChunked exception: {}", e.what());
        return voxelDownsample(cloud, leaf);
    } catch (...) {
        // ALOG_ERROR("Utils", "voxelDownsampleChunked unknown exception");
        return voxelDownsample(cloud, leaf);
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
