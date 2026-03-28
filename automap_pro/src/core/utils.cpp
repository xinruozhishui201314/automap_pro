#include "automap_pro/core/utils.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
#include <GeographicLib/LocalCartesian.hpp>
#include <filesystem>
#include <Eigen/Geometry>
#include <system_error>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <tuple>
#include <vector>
#include <cstdint>
#include <future>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace automap_pro {

namespace utils {

namespace {

// PCL VoxelGrid 在 applyFilter 中检查 (dx*dy*dz) > INT_MAX 会告警并返回；且后续 div_b_/min_b_/max_b_
// 使用 int，单轴过大仍可能导致线性索引或分配溢出并 SIGSEGV。PCL 源码: dx*dy*dz > std::numeric_limits<int>::max() 即告警。
// 保守取单轴上限为 INT_MAX 的立方根，使 dx*dy*dz <= INT_MAX，避免触发 PCL 告警与崩溃。
constexpr int kIntMaxCubeRoot = 1290;  // (1290^3 ≈ 2.15e9 >= INT_MAX)
constexpr float kMaxVoxelAxisIndex = static_cast<float>(kIntMaxCubeRoot);
constexpr float kDefaultMaxAbsCoord = 1e6f;

/** 根据 leaf_size 计算保证不溢出的单块最大边长（米）。块内体素格数 = chunk_size/leaf <= 安全上限。 */
inline float safeChunkSizeForLeaf(float leaf_size) {
    if (leaf_size <= 0.f) return 50.f;
    float safe = leaf_size * (kMaxVoxelAxisIndex * 0.5f);  // 留余量
    return std::max(leaf_size * 2.f, std::min(safe, 200.f));  // 至少 2*leaf，最大 200m
}

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

/** P1: OpenMP 并行体素下采样（每线程独立 map，再合并，避免锁竞争） */
CloudXYZIPtr voxelDownsampleGridNoPCLParallel(const CloudXYZIPtr& input, float leaf) {
    if (!input || input->empty() || leaf <= 0.f) return input;
#if defined(_OPENMP)
    const size_t n = input->points.size();
    const float inv_leaf = 1.f / leaf;
    const int max_t = omp_get_max_threads();
    std::vector<std::unordered_map<VoxelKey, size_t, VoxelKeyHash>> thread_maps(
        static_cast<size_t>(max_t));
#pragma omp parallel
    {
        int t = omp_get_thread_num();
        auto& m = thread_maps[static_cast<size_t>(t)];
        m.reserve(std::min(n / 4 + 1, size_t(500000)));
#pragma omp for
        for (size_t i = 0; i < n; ++i) {
            const auto& p = input->points[i];
            int64_t ix = static_cast<int64_t>(std::floor(p.x * inv_leaf));
            int64_t iy = static_cast<int64_t>(std::floor(p.y * inv_leaf));
            int64_t iz = static_cast<int64_t>(std::floor(p.z * inv_leaf));
            VoxelKey key{ix, iy, iz};
            m.emplace(key, i);
        }
    }
    std::unordered_map<VoxelKey, size_t, VoxelKeyHash> voxel_to_index;
    voxel_to_index.reserve(std::min(n / 4, size_t(500000)));
    auto out = std::make_shared<CloudXYZI>();
    for (const auto& m : thread_maps) {
        for (const auto& [k, idx] : m) {
            if (voxel_to_index.emplace(k, idx).second)
                out->push_back(input->points[idx]);
        }
    }
    return out;
#else
    return voxelDownsampleGridNoPCL(input, leaf);
#endif
}

inline int semanticLabelFromIntensityLocal(float intensity) {
    if (!std::isfinite(intensity)) return 0;
    long v = std::lround(static_cast<double>(intensity));
    if (v < 0) return 0;
    if (v > 65535) return 65535;
    return static_cast<int>(v);
}

struct SemanticBodyCell {
    Eigen::Vector3d sum_xyz = Eigen::Vector3d::Zero();
    double sum_lidar_i = 0.0;
    uint32_t n = 0;
    std::unordered_map<int, uint32_t> label_votes;
};

CloudXYZIPtr voxelDownsampleBodyWithSemanticLabelsImpl(const CloudXYZIPtr& body,
                                                       const CloudXYZIPtr& sem_labels,
                                                       float leaf) {
    CloudXYZIPtr out(new CloudXYZI);
    if (!body || body->empty()) return out;
    const float inv = 1.0f / leaf;
    const bool use_lab = sem_labels && sem_labels->size() == body->size();
    std::unordered_map<VoxelKey, SemanticBodyCell, VoxelKeyHash> grid;
    grid.reserve(std::min(body->size() / 4, size_t(500000)));
    for (size_t i = 0; i < body->size(); ++i) {
        const auto& p = body->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        int64_t ix = static_cast<int64_t>(std::floor(static_cast<double>(p.x) * static_cast<double>(inv)));
        int64_t iy = static_cast<int64_t>(std::floor(static_cast<double>(p.y) * static_cast<double>(inv)));
        int64_t iz = static_cast<int64_t>(std::floor(static_cast<double>(p.z) * static_cast<double>(inv)));
        VoxelKey key{ix, iy, iz};
        SemanticBodyCell& c = grid[key];
        c.sum_xyz += Eigen::Vector3d(p.x, p.y, p.z);
        c.sum_lidar_i += static_cast<double>(p.intensity);
        ++c.n;
        if (use_lab) {
            const int lab = semanticLabelFromIntensityLocal(sem_labels->points[i].intensity);
            c.label_votes[lab]++;
        }
    }
    out->reserve(grid.size());
    for (const auto& e : grid) {
        const SemanticBodyCell& c = e.second;
        if (c.n == 0) continue;
        pcl::PointXYZI q;
        q.x = static_cast<float>(c.sum_xyz.x() / static_cast<double>(c.n));
        q.y = static_cast<float>(c.sum_xyz.y() / static_cast<double>(c.n));
        q.z = static_cast<float>(c.sum_xyz.z() / static_cast<double>(c.n));
        if (use_lab) {
            int best = 0;
            uint32_t best_c = 0;
            for (const auto& kv : c.label_votes) {
                if (kv.second > best_c) {
                    best_c = kv.second;
                    best = kv.first;
                }
            }
            q.intensity = static_cast<float>(best);
        } else {
            q.intensity = static_cast<float>(c.sum_lidar_i / static_cast<double>(c.n));
        }
        out->push_back(q);
    }
    return out;
}

struct MajorityCell {
    Eigen::Vector3d sum_xyz = Eigen::Vector3d::Zero();
    uint32_t n = 0;
    std::unordered_map<int, uint32_t> votes;
};

CloudXYZIPtr voxelDownsampleGridMajorityNoPCL(const CloudXYZIPtr& input, float leaf) {
    if (!input || input->empty() || leaf <= 0.f) return input;
    auto out = std::make_shared<CloudXYZI>();
    std::unordered_map<VoxelKey, MajorityCell, VoxelKeyHash> grid;
    grid.reserve(std::min(input->size() / 4, size_t(500000)));
    const float inv_leaf = 1.f / leaf;
    for (size_t i = 0; i < input->points.size(); ++i) {
        const auto& p = input->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        int64_t ix = static_cast<int64_t>(std::floor(static_cast<double>(p.x) * static_cast<double>(inv_leaf)));
        int64_t iy = static_cast<int64_t>(std::floor(static_cast<double>(p.y) * static_cast<double>(inv_leaf)));
        int64_t iz = static_cast<int64_t>(std::floor(static_cast<double>(p.z) * static_cast<double>(inv_leaf)));
        VoxelKey key{ix, iy, iz};
        MajorityCell& c = grid[key];
        c.sum_xyz += Eigen::Vector3d(p.x, p.y, p.z);
        ++c.n;
        c.votes[semanticLabelFromIntensityLocal(p.intensity)]++;
    }
    out->reserve(grid.size());
    for (const auto& e : grid) {
        const MajorityCell& c = e.second;
        if (c.n == 0) continue;
        pcl::PointXYZI q;
        q.x = static_cast<float>(c.sum_xyz.x() / static_cast<double>(c.n));
        q.y = static_cast<float>(c.sum_xyz.y() / static_cast<double>(c.n));
        q.z = static_cast<float>(c.sum_xyz.z() / static_cast<double>(c.n));
        int best = 0;
        uint32_t best_c = 0;
        for (const auto& kv : c.votes) {
            if (kv.second > best_c) {
                best_c = kv.second;
                best = kv.first;
            }
        }
        q.intensity = static_cast<float>(best);
        out->push_back(q);
    }
    return out;
}

#if defined(_OPENMP)
CloudXYZIPtr voxelDownsampleGridMajorityNoPCLParallel(const CloudXYZIPtr& input, float leaf) {
    if (!input || input->empty() || leaf <= 0.f) return input;
    const size_t n = input->points.size();
    const float inv_leaf = 1.f / leaf;
    const int max_t = omp_get_max_threads();
    std::vector<std::unordered_map<VoxelKey, MajorityCell, VoxelKeyHash>> thread_maps(static_cast<size_t>(max_t));
#pragma omp parallel
    {
        int t = omp_get_thread_num();
        auto& m = thread_maps[static_cast<size_t>(t)];
        m.reserve(std::min(n / 4 + 1, size_t(500000)));
#pragma omp for
        for (size_t i = 0; i < n; ++i) {
            const auto& p = input->points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            int64_t ix = static_cast<int64_t>(std::floor(static_cast<double>(p.x) * static_cast<double>(inv_leaf)));
            int64_t iy = static_cast<int64_t>(std::floor(static_cast<double>(p.y) * static_cast<double>(inv_leaf)));
            int64_t iz = static_cast<int64_t>(std::floor(static_cast<double>(p.z) * static_cast<double>(inv_leaf)));
            VoxelKey key{ix, iy, iz};
            MajorityCell& c = m[key];
            c.sum_xyz += Eigen::Vector3d(p.x, p.y, p.z);
            ++c.n;
            c.votes[semanticLabelFromIntensityLocal(p.intensity)]++;
        }
    }
    std::unordered_map<VoxelKey, MajorityCell, VoxelKeyHash> merged;
    merged.reserve(std::min(n / 4, size_t(500000)));
    for (const auto& m : thread_maps) {
        for (const auto& [k, c0] : m) {
            MajorityCell& c = merged[k];
            c.sum_xyz += c0.sum_xyz;
            c.n += c0.n;
            for (const auto& kv : c0.votes) {
                c.votes[kv.first] += kv.second;
            }
        }
    }
    auto out = std::make_shared<CloudXYZI>();
    out->reserve(merged.size());
    for (const auto& e : merged) {
        const MajorityCell& c = e.second;
        if (c.n == 0) continue;
        pcl::PointXYZI q;
        q.x = static_cast<float>(c.sum_xyz.x() / static_cast<double>(c.n));
        q.y = static_cast<float>(c.sum_xyz.y() / static_cast<double>(c.n));
        q.z = static_cast<float>(c.sum_xyz.z() / static_cast<double>(c.n));
        int best = 0;
        uint32_t best_c = 0;
        for (const auto& kv : c.votes) {
            if (kv.second > best_c) {
                best_c = kv.second;
                best = kv.first;
            }
        }
        q.intensity = static_cast<float>(best);
        out->push_back(q);
    }
    return out;
}
#else
CloudXYZIPtr voxelDownsampleGridMajorityNoPCLParallel(const CloudXYZIPtr& input, float leaf) {
    return voxelDownsampleGridMajorityNoPCL(input, leaf);
}
#endif

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

CloudXYZIPtr voxelDownsample(const CloudXYZIPtr& cloud, float leaf_size, bool parallel) {
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
        // 若单次体素网格会溢出，改用分块下采样（保持 leaf 不变，避免分辨率退化）
        if (voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf)) {
            ALOG_INFO("Utils", "voxelDownsample: overflow risk with leaf={:.4f}, using chunked downsample (leaf unchanged)", leaf_size);
            return voxelDownsampleChunked(input, leaf, 0.f, parallel);  // 0 = 自动块大小
        }
        // 使用无 PCL 的体素下采样；根据 parallel 参数决定是否走 OpenMP 并行路径。
        // 🏛️ [架构优化] 移除对 ConfigManager 单例的运行时依赖，避免多线程/析构时的 SIGSEGV。
        CloudXYZIPtr out = parallel
            ? voxelDownsampleGridNoPCLParallel(input, leaf)
            : voxelDownsampleGridNoPCL(input, leaf);
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

CloudXYZIPtr voxelDownsampleBodyWithSemanticLabels(const CloudXYZIPtr& body,
                                                   const CloudXYZIPtr& sem_labels,
                                                   float leaf_size) {
    const float leaf = std::max(leaf_size, kMinVoxelLeafSize);
    if (leaf_size < kMinVoxelLeafSize) {
        ALOG_DEBUG("Utils", "voxelDownsampleBodyWithSemanticLabels: leaf_size={:.3f} clamped to {:.3f}", leaf_size, leaf);
    }
    return voxelDownsampleBodyWithSemanticLabelsImpl(body, sem_labels, leaf);
}

CloudXYZIPtr voxelDownsampleMajorityIntensity(const CloudXYZIPtr& cloud, float leaf_size, bool parallel) {
    try {
        if (!cloud || cloud->empty()) {
            ALOG_WARN("Utils", "voxelDownsampleMajorityIntensity: invalid input");
            return cloud;
        }
        float leaf = std::max(leaf_size, kMinVoxelLeafSize);
        if (leaf_size < kMinVoxelLeafSize) {
            ALOG_DEBUG("Utils", "voxelDownsampleMajorityIntensity: leaf_size={:.3f} clamped to {:.3f}", leaf_size, leaf);
        }
        CloudXYZIPtr input = sanitizePointCloudForVoxel(cloud, kDefaultMaxAbsCoord);
        if (!input || input->empty()) {
            ALOG_WARN("Utils", "voxelDownsampleMajorityIntensity: cloud empty after sanitization");
            return cloud;
        }
        float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f, min_z = 1e9f, max_z = -1e9f;
        for (const auto& p : input->points) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        }
        if (voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf)) {
            ALOG_DEBUG("Utils",
                       "voxelDownsampleMajorityIntensity: overflow risk leaf={:.4f}, fallback to voxelDownsample",
                       leaf_size);
            return voxelDownsample(cloud, leaf_size, parallel);
        }
        CloudXYZIPtr out = parallel ? voxelDownsampleGridMajorityNoPCLParallel(input, leaf)
                                    : voxelDownsampleGridMajorityNoPCL(input, leaf);
        if (out && out->empty()) {
            ALOG_WARN("Utils", "voxelDownsampleMajorityIntensity: output empty after filter");
        }
        return out ? out : input;
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "voxelDownsampleMajorityIntensity exception: {}", e.what());
        return cloud;
    } catch (...) {
        ALOG_ERROR("Utils", "voxelDownsampleMajorityIntensity unknown exception");
        return cloud;
    }
}

// ============================================================================
// V1: 带超时保护的体素下采样
// ============================================================================

CloudXYZIPtr voxelDownsampleWithTimeout(const CloudXYZIPtr& cloud, float leaf_size,
                                          int timeout_ms, bool* timed_out, bool parallel) {
    const unsigned tid = automap_pro::logThreadId();
    const auto t_start = std::chrono::steady_clock::now();
    
    if (timed_out) *timed_out = false;
    
    if (!cloud || cloud->empty()) {
        return cloud;
    }
    
    // 对于小点云，直接处理（通常很快）
    if (cloud->size() < 50000) {
        try {
            return voxelDownsample(cloud, leaf_size, parallel);
        } catch (const std::exception& e) {
            ALOG_ERROR("Utils", "[tid={}] voxelDownsampleWithTimeout: small cloud exception: {}", tid, e.what());
            return cloud;
        } catch (...) {
            ALOG_ERROR("Utils", "[tid={}] voxelDownsampleWithTimeout: small cloud unknown exception", tid);
            return cloud;
        }
    }
    
    // 大点云使用异步 + 超时保护
    ALOG_DEBUG("Utils", "[tid={}] voxelDownsampleWithTimeout: starting async voxel for {} pts, timeout={}ms",
               tid, cloud->size(), timeout_ms);
    
    // 按值捕获 cloud（shared_ptr 副本），避免超时返回后异步任务悬空引用
    std::future<CloudXYZIPtr> future = std::async(std::launch::async, [cloud, leaf_size, tid, parallel]() -> CloudXYZIPtr {
        try {
            return voxelDownsample(cloud, leaf_size, parallel);
        } catch (const std::exception& e) {
            ALOG_ERROR("Utils", "[tid={}] voxelDownsampleWithTimeout async exception: {}", tid, e.what());
            return cloud;
        } catch (...) {
            ALOG_ERROR("Utils", "[tid={}] voxelDownsampleWithTimeout async unknown exception", tid);
            return cloud;
        }
    });
    
    std::future_status status = future.wait_for(std::chrono::milliseconds(timeout_ms));
    
    if (status == std::future_status::ready) {
        auto result = future.get();
        const auto t_end = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        ALOG_DEBUG("Utils", "[tid={}] voxelDownsampleWithTimeout: completed in {:.1f}ms", tid, elapsed_ms);
        return result;
    }
    
    // 超时（凡计算超时均打 [COMPUTE_TIMEOUT]，便于 grep 统一检索）
    const auto t_end = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    ALOG_WARN("Utils", "[COMPUTE_TIMEOUT] voxelDownsampleWithTimeout: TIMEOUT after {:.1f}ms (limit={}ms), returning raw copy for {} pts",
              elapsed_ms, timeout_ms, cloud->size());
    
    if (timed_out) *timed_out = true;
    
    // 返回原始点云的副本（经过 sanitize）
    CloudXYZIPtr result = sanitizePointCloudForVoxel(cloud, kDefaultMaxAbsCoord);
    return result ? result : cloud;
}

// ============================================================================
// 带超时的分块体素（merge 等大点云路径：OpenMP 分块并行 + 超时保护）
// ============================================================================

CloudXYZIPtr voxelDownsampleChunkedWithTimeout(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m,
                                                int timeout_ms, bool* timed_out) {
    const unsigned tid = automap_pro::logThreadId();
    const auto t_start = std::chrono::steady_clock::now();
    if (timed_out) *timed_out = false;
    if (!cloud || cloud->empty()) return cloud;
    float leaf = std::max(leaf_size, kMinVoxelLeafSize);
    float chunk_m = (chunk_size_m > 0.f) ? chunk_size_m : 0.f;

    // 🏛️ [架构优化] 在此处读取 ConfigManager 并显式向下传递，避免异步/OpenMP 线程内竞争
    const bool parallel = ConfigManager::instance().parallelVoxelDownsample();

    if (cloud->size() < 50000u) {
        try {
            return voxelDownsample(cloud, leaf, parallel);
        } catch (...) {
            return cloud;
        }
    }

    std::future<CloudXYZIPtr> future = std::async(std::launch::async, [cloud, leaf, chunk_m, tid, parallel]() -> CloudXYZIPtr {
        try {
            return voxelDownsampleChunked(cloud, leaf, chunk_m, parallel);
        } catch (const std::exception& e) {
            ALOG_ERROR("Utils", "[tid={}] voxelDownsampleChunkedWithTimeout async exception: {}", tid, e.what());
            return cloud;
        } catch (...) {
            return cloud;
        }
    });
    
    std::future_status status = future.wait_for(std::chrono::milliseconds(timeout_ms));
    if (status == std::future_status::ready) {
        auto result = future.get();
        double elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
        ALOG_DEBUG("Utils", "[tid={}] voxelDownsampleChunkedWithTimeout: completed in {:.1f}ms", tid, elapsed_ms);
        return result;
    }

    double elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
    ALOG_WARN("Utils", "[COMPUTE_TIMEOUT] voxelDownsampleChunkedWithTimeout: TIMEOUT after {:.1f}ms (limit={}ms), returning raw copy for {} pts",
              elapsed_ms, timeout_ms, cloud->size());
    if (timed_out) *timed_out = true;
    CloudXYZIPtr result = sanitizePointCloudForVoxel(cloud, kDefaultMaxAbsCoord);
    return result ? result : cloud;
}

// 点云点数低于此值时直接单次体素滤波，不走分块循环，避免 PCL 在部分 chunk 上 SIGSEGV
// 🔧 [修复] 降低分块阈值：250000 -> 100000，更积极地使用分块下采样，避免单次 PCL 网格过大导致精度损失
static constexpr size_t kVoxelChunkedSizeThreshold = 100000u;

CloudXYZIPtr voxelDownsampleSafe(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m) {
    if (!cloud || cloud->empty()) return cloud;
    float leaf = std::max(leaf_size, kMinVoxelLeafSize);
    CloudXYZIPtr input = sanitizePointCloudForVoxel(cloud, kDefaultMaxAbsCoord);
    if (!input || input->empty()) return cloud;
    float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f, min_z = 1e9f, max_z = -1e9f;
    for (const auto& p : input->points) {
        min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
    }
    // 🏛️ [架构优化] 在顶层读取配置并向下传递
    const bool parallel = ConfigManager::instance().parallelVoxelDownsample();
    bool overflow = voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf);
    if (!overflow && input->size() <= kVoxelChunkedSizeThreshold) {
        return voxelDownsample(cloud, leaf, parallel);
    }
    if (chunk_size_m <= 0.f) chunk_size_m = safeChunkSizeForLeaf(leaf);
    return voxelDownsampleChunked(input, leaf, chunk_size_m, parallel);
}

CloudXYZIPtr voxelDownsampleChunked(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m, bool parallel) {
    const unsigned tid = automap_pro::logThreadId();
    if (!cloud || cloud->empty()) return cloud;
    float leaf = std::max(leaf_size, kMinVoxelLeafSize);
    if (chunk_size_m <= 0.f) {
        chunk_size_m = safeChunkSizeForLeaf(leaf);
        ALOG_INFO("Utils", "[tid={}] voxelDownsampleChunked auto chunk_size_m={:.1f} (leaf={:.3f})",
                  tid, chunk_size_m, leaf);
    }
    ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=enter in={} leaf={:.3f} chunk_m={:.1f}",
              tid, cloud->size(), leaf, chunk_size_m);
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
            return voxelDownsample(cloud, leaf, parallel);
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

        const int total_chunks = nx * ny * nz;
        std::vector<CloudXYZIPtr> chunk_results(static_cast<size_t>(total_chunks));
        const auto& pts_ref = input->points;
        const size_t num_pts = pts_ref.size();
        const float cs_f = cs;

        // 🏛️ [P0 优化] 采用高效的一路分发 (One-pass distribution) 代替 O(Chunks * N) 的循环
        // 之前每个 chunk 都会遍历全图，导致在大范围地图（chunk 数多）时计算量爆炸。
        // 现在一次遍历即可将点分发到对应的 chunk 索引中。
        std::vector<std::vector<size_t>> chunk_indices(static_cast<size_t>(total_chunks));
        for (size_t i = 0; i < num_pts; ++i) {
            const auto& p = pts_ref[i];
            int ix = std::clamp(static_cast<int>((p.x - min_x) / cs_f), 0, nx - 1);
            int iy = std::clamp(static_cast<int>((p.y - min_y) / cs_f), 0, ny - 1);
            int iz = std::clamp(static_cast<int>((p.z - min_z) / cs_f), 0, nz - 1);
            int idx = ix * (ny * nz) + iy * nz + iz;
            chunk_indices[static_cast<size_t>(idx)].push_back(i);
        }

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int idx = 0; idx < total_chunks; ++idx) {
            if (chunk_indices[static_cast<size_t>(idx)].empty()) {
                chunk_results[static_cast<size_t>(idx)] = nullptr;
                continue;
            }

            auto chunk = std::make_shared<CloudXYZI>();
            chunk->points.reserve(chunk_indices[static_cast<size_t>(idx)].size());
            for (size_t pt_idx : chunk_indices[static_cast<size_t>(idx)]) {
                chunk->points.push_back(pts_ref[pt_idx]);
            }

            CloudXYZIPtr ds;
            try {
                ds = voxelDownsample(chunk, leaf, false); // 子块下采样不再二次并行，避免 OpenMP 嵌套竞争与 ConfigManager 访问
            } catch (const std::exception&) {
                ds = chunk;
            } catch (...) {
                ds = chunk;
            }
            chunk_results[static_cast<size_t>(idx)] = (ds && !ds->empty()) ? ds : nullptr;
        }

        for (const auto& c : chunk_results) {
            if (c && !c->empty()) {
                merged->reserve(merged->size() + c->size());
                for (const auto& p : c->points)
                    merged->push_back(p);
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
            result = voxelDownsample(merged, leaf, parallel);
        } else {
            ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=skip_final_voxel (extent overflow risk) merged={}", tid, merged->size());
            result = merged;
        }
        ALOG_INFO("Utils", "[tid={}] [MAP] voxelDownsampleChunked step=exit out={}", tid, result ? result->size() : 0u);
        return result;
    } catch (const std::exception& e) {
        // ALOG_ERROR("Utils", "voxelDownsampleChunked exception: {}", e.what());
        return voxelDownsample(cloud, leaf, parallel);
    } catch (...) {
        // ALOG_ERROR("Utils", "voxelDownsampleChunked unknown exception");
        return voxelDownsample(cloud, leaf, parallel);
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

Eigen::Vector3d wgs84ToEnu(double lat, double lon, double alt, 
                           double origin_lat, double origin_lon, double origin_alt) {
    try {
        GeographicLib::LocalCartesian proj(origin_lat, origin_lon, origin_alt);
        double x, y, z;
        proj.Forward(lat, lon, alt, x, y, z);
        return Eigen::Vector3d(x, y, z);
    } catch (const std::exception& e) {
        ALOG_ERROR("Utils", "wgs84ToEnu exception: {}", e.what());
        return Eigen::Vector3d::Zero();
    }
}

}  // namespace utils

}  // namespace automap_pro
