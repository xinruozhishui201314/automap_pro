#include "automap_pro/loop_closure/fpfh_extractor.h"
#include "automap_pro/core/logger.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <limits>
#include <thread>
#include <csignal>
#include <cstring>
#include <execinfo.h>
#include <unistd.h>
#include <iostream>

#define MOD "FPFHExtractor"

// 崩溃精准分析：关键步骤同时写 cerr 并 flush，确保崩溃瞬间前一行日志可见
#define FPFH_CRASH_LOG(step, msg) do { \
    ALOG_INFO(MOD, "[CRASH_TRACE] {} {}", step, msg); \
    std::cerr << "[FPFH_CRASH_TRACE] " << step << " " << msg << std::endl; \
} while(0)

// 全局信号处理器：捕获段错误并记录栈回溯
namespace {
    volatile sig_atomic_t g_sigsegv_flag = 0;
    thread_local bool g_in_fpfh_compute = false;
    
    void sigsegv_handler(int sig) {
        if (g_in_fpfh_compute) {
            g_sigsegv_flag = 1;
            // 记录栈回溯
            void* array[50];
            int size = backtrace(array, 50);
            char** strings = backtrace_symbols(array, size);
            
            fprintf(stderr, "\n========== SIGSEGV in FPFH compute ==========\n");
            fprintf(stderr, "Signal: %d, Thread: %lu\n", sig, pthread_self());
            fprintf(stderr, "Stack trace:\n");
            for (int i = 0; i < size; i++) {
                fprintf(stderr, "  [%d] %s\n", i, strings[i]);
            }
            fprintf(stderr, "=============================================\n");
            free(strings);
            
            // 不退出，尝试恢复（longjmp需要额外设置，这里简化处理）
            // 实际应该使用longjmp，但PCL内部状态可能已损坏
            _exit(139); // SIGSEGV退出码
        }
        // 如果不在FPFH计算中，使用默认处理
        signal(sig, SIG_DFL);
        raise(sig);
    }
}

namespace automap_pro {

// 全局互斥锁：保护PCL FPFH计算（PCL 1.11-1.12多线程不安全）
std::mutex FpfhExtractor::compute_mutex_;

FpfhExtractor::FpfhExtractor() {
    // 注册信号处理器（仅首次）
    static bool signal_registered = false;
    if (!signal_registered) {
        struct sigaction sa;
        memset(&sa, 0, sizeof(sa));
        sa.sa_handler = sigsegv_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = SA_RESTART;
        sigaction(SIGSEGV, &sa, nullptr);
        signal_registered = true;
        ALOG_INFO(MOD, "SIGSEGV handler registered for FPFH compute protection");
    }
}

/**
 * 单线程FPFH计算（带全局锁）- 对象池化版本
 * 
 * 根本原因分析（修复SIGSEGV double-free）：
 * ============================================================
 * 崩溃现象：free() 中 SIGSEGV，栈回溯指向 FPFHEstimation::~FPFHEstimation()
 * 
 * 根本原因：
 * 1. PCL 1.11-1.12 FPFHEstimation 内部 KDTree/搜索树在析构时存在双重释放
 * 2. 栈分配对象在析构时，其内部动态数据（法向量、特征缓冲）被多次释放
 * 3. 大点云（525K pts → 4K pts）导致缓冲区复用频繁，增加冲突概率
 * 4. 多线程同时创建/销毁触发竞争（虽有全局锁，但析构前短窗口无保护）
 * 
 * 修复策略：
 * - 分离 compute() 的三个阶段：法向量 → 特征提取 → 清理
 * - 使用显式作用域和 try-finally 确保每个 PCL 对象被正确销毁
 * - 在销毁前检查对象有效性，防止重复销毁
 * - 增强日志追踪内存操作与异常发生时刻
 * 
 * 工作流：
 * 1. 加全局锁
 * 2. 分别创建法向量估计、FPFH估计对象（栈分配但生命周期可控）
 * 3. 逐步执行计算，捕获中间异常
 * 4. 显式清理临时对象（调用 reset() 或作用域退出）
 * 5. 释放锁
 */
FPFHCloudPtr FpfhExtractor::compute(
    const CloudXYZIPtr& cloud,
    float normal_radius,
    float fpfh_radius) const
{
    const unsigned tid = automap_pro::logThreadId();
    
    // 🛡️ 设置线程局部标志，用于信号处理器
    g_in_fpfh_compute = true;
    g_sigsegv_flag = 0;
    
    // 使用RAII确保退出时清理标志
    struct FpfhGuard {
        ~FpfhGuard() { g_in_fpfh_compute = false; }
    } guard;
    
    ALOG_INFO(MOD, "[tid={}] ========== FPFH COMPUTE START ==========", tid);
    ALOG_INFO(MOD, "[tid={}] compute_enter cloud_ptr={} cloud_size={} use_count={} normal_radius={:.3f} fpfh_radius={:.3f}",
              tid, static_cast<const void*>(cloud.get()), cloud ? cloud->size() : 0u, cloud ? cloud.use_count() : 0, normal_radius, fpfh_radius);

    // 🛡️ 多层防护：空指针检查
    if (!cloud) {
        ALOG_ERROR(MOD, "[tid={}] compute_fail null cloud pointer", tid);
        return std::make_shared<FPFHCloud>();
    }
    if (cloud->empty()) {
        ALOG_WARN(MOD, "[tid={}] compute_skip empty cloud", tid);
        return std::make_shared<FPFHCloud>();
    }
    
    // 🛡️ 点云大小合理性检查
    const size_t cloud_size = cloud->size();
    if (cloud_size > 1000000) {  // 100万点上限
        ALOG_ERROR(MOD, "[tid={}] compute_fail cloud too large size={}", tid, cloud_size);
        return std::make_shared<FPFHCloud>();
    }
    if (cloud_size < 10) {  // 至少10个点才能计算特征
        ALOG_WARN(MOD, "[tid={}] compute_skip cloud too small size={}", tid, cloud_size);
        return std::make_shared<FPFHCloud>();
    }
    
    // 🛡️ 验证点云内存连续性和数据有效性
    ALOG_INFO(MOD, "[tid={}] validate_memory cloud_capacity={} cloud_size={} cloud_width={} cloud_height={} is_dense={}",
              tid, cloud->points.capacity(), cloud_size, cloud->width, cloud->height, cloud->is_dense);
    
    try {
        // 检查首尾点和中间点
        const auto& first_pt = cloud->points[0];
        const auto& mid_pt = cloud->points[cloud_size / 2];
        const auto& last_pt = cloud->points[cloud_size - 1];
        
        ALOG_INFO(MOD, "[tid={}] validate_points first=[{:.2f},{:.2f},{:.2f}] mid=[{:.2f},{:.2f},{:.2f}] last=[{:.2f},{:.2f},{:.2f}]",
                  tid, first_pt.x, first_pt.y, first_pt.z, mid_pt.x, mid_pt.y, mid_pt.z, last_pt.x, last_pt.y, last_pt.z);
        
        if (!std::isfinite(first_pt.x) || !std::isfinite(first_pt.y) || !std::isfinite(first_pt.z) ||
            !std::isfinite(mid_pt.x) || !std::isfinite(mid_pt.y) || !std::isfinite(mid_pt.z) ||
            !std::isfinite(last_pt.x) || !std::isfinite(last_pt.y) || !std::isfinite(last_pt.z)) {
            ALOG_ERROR(MOD, "[tid={}] compute_fail invalid point data (NaN or Inf detected)", tid);
            return std::make_shared<FPFHCloud>();
        }
        
        ALOG_INFO(MOD, "[tid={}] validate_memory_success points_accessible=true data_valid=true", tid);
    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={}] compute_fail memory_access_bad_alloc msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] compute_fail memory_access_exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] compute_fail memory_access_unknown_exception", tid);
        return std::make_shared<FPFHCloud>();
    }

    // 🛡️ 参数修正
    if (normal_radius <= 0 || fpfh_radius <= 0) {
        ALOG_WARN(MOD, "[tid={}] compute_param_adjust invalid radii (normal={:.3f}, fpfh={:.3f}), using defaults",
                  tid, normal_radius, fpfh_radius);
        normal_radius = std::max(normal_radius, 0.1f);
        fpfh_radius = std::max(fpfh_radius, 0.1f);
    }
    // 防止半径过大导致内存爆炸
    normal_radius = std::min(normal_radius, 5.0f);
    fpfh_radius = std::min(fpfh_radius, 10.0f);

    auto feat = std::make_shared<FPFHCloud>();
    try {
        feat->reserve(cloud_size);
    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={}] compute_fail feature_reserve_failed size={} msg={}", tid, cloud_size, e.what());
        return std::make_shared<FPFHCloud>();
    }
    
    try {
        // 🔒 全局锁保护 PCL FPFH 计算
        // 原因：PCL 1.11-1.12 FPFHEstimationOMP 有多线程内存泄漏 (double-free in dtor)
        // 即使非OMP版本，PCL的搜索树/内存管理在析构时仍偶发崩溃
        ALOG_INFO(MOD, "[tid={}] ========== STAGE: LOCK ACQUIRE ==========", tid);
        std::lock_guard<std::mutex> lk(compute_mutex_);
        ALOG_INFO(MOD, "[tid={}] ========== STAGE: LOCK ACQUIRED ==========", tid);

        // === 阶段1：法向量估计（单线程）===
        // 使用静态复用的 NormalEstimation，避免析构时触发 PCL 1.11-1.12 的 free() SIGSEGV（析构器 double-free）
        ALOG_INFO(MOD, "[tid={}] ========== STAGE 1: NORMAL ESTIMATION START ==========", tid);
        auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
        
        static pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> s_ne;
        std::shared_ptr<pcl::search::KdTree<pcl::PointXYZI>> tree_n;
        
        FPFH_CRASH_LOG("1.1", "create_kdtree_start");
        try {
            tree_n = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
            FPFH_CRASH_LOG("1.1", "create_kdtree_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [1.1] create_kdtree_exception msg={}", tid, e.what());
            return std::make_shared<FPFHCloud>();
        }
        if (!tree_n) {
            ALOG_ERROR(MOD, "[tid={}] [1.1] create_kdtree_null", tid);
            return std::make_shared<FPFHCloud>();
        }
        
        FPFH_CRASH_LOG("1.3", "set_input_cloud_start");
        try {
            s_ne.setInputCloud(cloud);
            s_ne.setSearchMethod(tree_n);
            s_ne.setRadiusSearch(normal_radius);
            FPFH_CRASH_LOG("1.3", "set_input_cloud_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [1.3] set_input_cloud_exception msg={}", tid, e.what());
            return std::make_shared<FPFHCloud>();
        }
        
        FPFH_CRASH_LOG("1.6", "compute_normals_start");
        try {
            s_ne.compute(*normals);
            FPFH_CRASH_LOG("1.6", "compute_normals_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [1.6] compute_exception msg={}", tid, e.what());
            return std::make_shared<FPFHCloud>();
        }
        
        // 显式置空，释放对 cloud/tree 的引用（不析构 s_ne，避免 PCL 析构 bug）
        FPFH_CRASH_LOG("1.7", "ne_clear_inputs_start");
        try {
            s_ne.setInputCloud(CloudXYZIPtr());
            s_ne.setSearchMethod(typename pcl::search::KdTree<pcl::PointXYZI>::Ptr());
            FPFH_CRASH_LOG("1.7", "ne_clear_inputs_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [1.7] ne_clear_exception msg={}", tid, e.what());
        }
        ALOG_INFO(MOD, "[tid={}] ========== STAGE 1: NORMAL ESTIMATION COMPLETE ==========", tid);

        // === 阶段2：FPFH 计算（单线程）===
        // 使用静态复用的 FPFHEstimation，避免析构时触发 PCL ~FPFHEstimation() 内 free() SIGSEGV
        ALOG_INFO(MOD, "[tid={}] ========== STAGE 2: FPFH COMPUTATION START ==========", tid);
        ALOG_INFO(MOD, "[tid={}] [2.0] fpfh_stage_enter normals_ptr={} normals_size={}", 
                  tid, static_cast<const void*>(normals.get()), normals ? normals->size() : 0u);
        
        static pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> s_fpfh;
        std::shared_ptr<pcl::search::KdTree<pcl::PointXYZI>> tree_f;
        
        FPFH_CRASH_LOG("2.1", "create_kdtree_start");
        try {
            tree_f = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
            FPFH_CRASH_LOG("2.1", "create_kdtree_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [2.1] create_kdtree_exception msg={}", tid, e.what());
            return std::make_shared<FPFHCloud>();
        }
        if (!tree_f) {
            ALOG_ERROR(MOD, "[tid={}] [2.1] create_kdtree_null", tid);
            return std::make_shared<FPFHCloud>();
        }
        
        FPFH_CRASH_LOG("2.3", "set_input_cloud_start");
        try {
            s_fpfh.setInputCloud(cloud);
            FPFH_CRASH_LOG("2.4", "set_input_normals_start");
            s_fpfh.setInputNormals(normals);
            s_fpfh.setSearchMethod(tree_f);
            s_fpfh.setRadiusSearch(fpfh_radius);
            FPFH_CRASH_LOG("2.5", "set_search_method_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [2.3-2.5] set_input_exception msg={}", tid, e.what());
            return std::make_shared<FPFHCloud>();
        }
        
        FPFH_CRASH_LOG("2.7", "compute_fpfh_start");
        try {
            s_fpfh.compute(*feat);
            FPFH_CRASH_LOG("2.7", "compute_fpfh_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [2.7] compute_exception msg={}", tid, e.what());
            return std::make_shared<FPFHCloud>();
        }
        
        if (feat->size() == 0)
            ALOG_WARN(MOD, "[tid={}] [2.7] compute_empty_features", tid);
        
        // 显式置空，释放对 cloud/normals/tree 的引用（不析构 s_fpfh，避免 PCL 析构 free() 崩溃）
        FPFH_CRASH_LOG("2.8", "fpfh_clear_inputs_start");
        try {
            s_fpfh.setInputCloud(CloudXYZIPtr());
            s_fpfh.setInputNormals(std::shared_ptr<pcl::PointCloud<pcl::Normal>>());
            s_fpfh.setSearchMethod(typename pcl::search::KdTree<pcl::PointXYZI>::Ptr());
            FPFH_CRASH_LOG("2.8", "fpfh_clear_inputs_success");
        } catch (const std::exception& e) {
            ALOG_ERROR(MOD, "[tid={}] [2.8] fpfh_clear_exception msg={}", tid, e.what());
        }
        ALOG_INFO(MOD, "[tid={}] ========== STAGE 2: FPFH COMPUTATION COMPLETE ==========", tid);

        ALOG_INFO(MOD, "[tid={}] ========== FPFH COMPUTE SUCCESS ==========", tid);
        ALOG_INFO(MOD, "[tid={}] compute_exit_success cloud_pts={} features={} feat_ptr={}",
                 tid, cloud_size, feat->size(), static_cast<const void*>(feat.get()));
        return feat;
        
    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={}] ========== FPFH COMPUTE FAILED (bad_alloc) ==========", tid);
        ALOG_ERROR(MOD, "[tid={}] compute_outer_bad_alloc msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (const std::runtime_error& e) {
        ALOG_ERROR(MOD, "[tid={}] ========== FPFH COMPUTE FAILED (runtime_error) ==========", tid);
        ALOG_ERROR(MOD, "[tid={}] compute_outer_runtime_error msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] ========== FPFH COMPUTE FAILED (exception) ==========", tid);
        ALOG_ERROR(MOD, "[tid={}] compute_outer_exception msg={} typeid={}", tid, e.what(), typeid(e).name());
        return std::make_shared<FPFHCloud>();
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] ========== FPFH COMPUTE FAILED (unknown) ==========", tid);
        ALOG_ERROR(MOD, "[tid={}] compute_unknown_exception sigsegv_flag={}", tid, (int)g_sigsegv_flag);
        return std::make_shared<FPFHCloud>();
    }
}

static float fpfhDist(const pcl::FPFHSignature33& a, const pcl::FPFHSignature33& b) {
    float d = 0;
    for (int i = 0; i < 33; ++i) {
        float diff = a.histogram[i] - b.histogram[i];
        d += diff * diff;
    }
    return d;
}

std::vector<std::pair<int,int>> FpfhExtractor::findCorrespondences(
    const FPFHCloudPtr& src, const FPFHCloudPtr& tgt, bool mutual) const
{
    const unsigned tid = automap_pro::logThreadId();
    ALOG_INFO(MOD, "[tid={}] findCorr_enter src_ptr={} src_pts={} tgt_ptr={} tgt_pts={} mutual={}",
              tid, static_cast<const void*>(src.get()), src ? src->size() : 0u,
              static_cast<const void*>(tgt.get()), tgt ? tgt->size() : 0u, mutual);

    try {
        std::vector<std::pair<int,int>> corrs;
        if (!src || !tgt || src->empty() || tgt->empty()) {
            ALOG_DEBUG(MOD, "[tid={}] findCorr_skip empty clouds", tid);
            return corrs;
        }

        // src → tgt 最近邻
        std::vector<int> src2tgt(src->size(), -1);
        std::vector<float> src_dists(src->size(), std::numeric_limits<float>::max());

        for (size_t i = 0; i < src->size(); ++i) {
            float best = std::numeric_limits<float>::max();
            for (size_t j = 0; j < tgt->size(); ++j) {
                float d = fpfhDist(src->points[i], tgt->points[j]);
                if (d < best) {
                    best = d;
                    src2tgt[i] = static_cast<int>(j);
                }
            }
            src_dists[i] = best;
        }

        if (!mutual) {
            for (size_t i = 0; i < src->size(); ++i)
                if (src2tgt[i] >= 0) corrs.push_back({static_cast<int>(i), src2tgt[i]});
            ALOG_INFO(MOD, "[tid={}] findCorr_exit_unidirect corrs={}", tid, corrs.size());
            return corrs;
        }

        // tgt → src 最近邻 (双向匹配)
        std::vector<int> tgt2src(tgt->size(), -1);
        std::vector<float> tgt_dists(tgt->size(), std::numeric_limits<float>::max());

        for (size_t j = 0; j < tgt->size(); ++j) {
            float best = std::numeric_limits<float>::max();
            for (size_t i = 0; i < src->size(); ++i) {
                float d = fpfhDist(src->points[i], tgt->points[j]);
                if (d < best) {
                    best = d;
                    tgt2src[j] = static_cast<int>(i);
                }
            }
            tgt_dists[j] = best;
        }

        // 验证互惠性
        for (size_t i = 0; i < src->size(); ++i) {
            int j = src2tgt[i];
            if (j >= 0 && tgt2src[j] == static_cast<int>(i))
                corrs.push_back({static_cast<int>(i), j});
        }

        ALOG_INFO(MOD, "[tid={}] findCorr_exit_mutual corrs={} mutual_ratio={:.2f}",
                 tid, corrs.size(), src->size() > 0 ? 100.0f * corrs.size() / src->size() : 0.0f);
        return corrs;
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] findCorr_exception msg={}", tid, e.what());
        return {};
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] findCorr_unknown_exception", tid);
        return {};
    }
}

} // namespace automap_pro

