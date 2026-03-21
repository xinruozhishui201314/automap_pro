#include "automap_pro/core/utils.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/metrics.h"
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
        // 若单次体素网格会溢出，改用分块下采样（保持 leaf 不变，避免分辨率退化）
        if (voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf)) {
            ALOG_INFO("Utils", "voxelDownsample: overflow risk with leaf={:.4f}, using chunked downsample (leaf unchanged)", leaf_size);
            return voxelDownsampleChunked(input, leaf, 0.f);  // 0 = 自动块大小
        }
        // 使用无 PCL 的体素下采样；performance.parallel_voxel_downsample 为 true 时走 OpenMP 并行路径。
        CloudXYZIPtr out = ConfigManager::instance().parallelVoxelDownsample()
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

// ============================================================================
// V1: 带超时保护的体素下采样
// ============================================================================

CloudXYZIPtr voxelDownsampleWithTimeout(const CloudXYZIPtr& cloud, float leaf_size,
                                          int timeout_ms, bool* timed_out) {
    const unsigned tid = automap_pro::logThreadId();
    const auto t_start = std::chrono::steady_clock::now();
    
    if (timed_out) *timed_out = false;
    
    if (!cloud || cloud->empty()) {
        return cloud;
    }
    
    // 对于小点云，直接处理（通常很快）
    if (cloud->size() < 50000) {
        try {
            return voxelDownsample(cloud, leaf_size);
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
    std::future<CloudXYZIPtr> future = std::async(std::launch::async, [cloud, leaf_size, tid]() -> CloudXYZIPtr {
        try {
            return voxelDownsample(cloud, leaf_size);
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

    if (cloud->size() < 50000u) {
        try {
            return voxelDownsample(cloud, leaf);
        } catch (...) {
            return cloud;
        }
    }

    std::future<CloudXYZIPtr> future = std::async(std::launch::async, [cloud, leaf, chunk_m, tid]() -> CloudXYZIPtr {
        try {
            return voxelDownsampleChunked(cloud, leaf, chunk_m);
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
    bool overflow = voxelGridWouldOverflow(min_x, max_x, min_y, max_y, min_z, max_z, leaf);
    if (!overflow && input->size() <= kVoxelChunkedSizeThreshold) {
        return voxelDownsample(cloud, leaf);
    }
    if (chunk_size_m <= 0.f) chunk_size_m = safeChunkSizeForLeaf(leaf);
    return voxelDownsampleChunked(input, leaf, chunk_size_m);
}

CloudXYZIPtr voxelDownsampleChunked(const CloudXYZIPtr& cloud, float leaf_size, float chunk_size_m) {
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

        const int total_chunks = nx * ny * nz;
        std::vector<CloudXYZIPtr> chunk_results(static_cast<size_t>(total_chunks));
        const auto& pts_ref = input->points;
        const size_t num_pts = pts_ref.size();
        const float cs_f = cs;

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int idx = 0; idx < total_chunks; ++idx) {
            const int iz = idx % nz;
            const int iy = (idx / nz) % ny;
            const int ix = idx / (nz * ny);
            const float cx_min = min_x + ix * cs_f;
            const float cx_max = (ix + 1 == nx) ? max_x : (min_x + (ix + 1) * cs_f);
            const float cy_min = min_y + iy * cs_f;
            const float cy_max = (iy + 1 == ny) ? max_y : (min_y + (iy + 1) * cs_f);
            const float cz_min = min_z + iz * cs_f;
            const float cz_max = (iz + 1 == nz) ? max_z : (min_z + (iz + 1) * cs_f);

            auto chunk = std::make_shared<CloudXYZI>();
            chunk->reserve(std::min(num_pts / (static_cast<size_t>(total_chunks) + 1), num_pts));
            for (size_t i = 0; i < num_pts; ++i) {
                const auto& p = pts_ref[i];
                if (p.x >= cx_min && p.x <= cx_max && p.y >= cy_min && p.y <= cy_max && p.z >= cz_min && p.z <= cz_max)
                    chunk->push_back(p);
            }
            if (chunk->empty()) {
                chunk_results[static_cast<size_t>(idx)] = nullptr;
                continue;
            }
            CloudXYZIPtr ds;
            try {
                ds = voxelDownsample(chunk, leaf);
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
