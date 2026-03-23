#include "automap_pro/loop_closure/fpfh_extractor.h"
#include "automap_pro/core/logger.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <limits>
#include <thread>
#include <chrono>
#include <csignal>
#include <cstring>
#include <execinfo.h>
#include <unistd.h>
#include <iostream>
#include <cxxabi.h>
#include <cstdlib>

#define MOD "FPFHExtractor"

// 崩溃精准分析：关键步骤带 ts_ms/lwp，使用 stderr 绕过 spdlog 以防 heap 损坏或 shutdown 导致的 logger 崩溃
#define _FPFH_TS_MS() (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())
#define FPFH_CRASH_LOG(step, msg) do { \
    auto _ts = _FPFH_TS_MS(); \
    fprintf(stderr, "[FPFH_CRASH_TRACE][lwp=%ld][ts_ms=%lu] %s %s\n", automap_pro::logLwp(), (unsigned long)_ts, step, msg); \
    fflush(stderr); \
} while(0)
#define FPFH_CRASH_LOG_PTR(step, msg, ptr, size) do { \
    auto _ts = _FPFH_TS_MS(); \
    fprintf(stderr, "[FPFH_CRASH_TRACE][lwp=%ld][ts_ms=%lu] %s %s ptr=%p size=%zu\n", automap_pro::logLwp(), (unsigned long)_ts, step, msg, static_cast<const void*>(ptr), (size_t)(size)); \
    fflush(stderr); \
} while(0)

// 在关键步骤可选打印 backtrace（环境变量 AUTOMAP_FPFH_BACKTRACE=1 时启用）
namespace {
    void logBacktraceToStderr(const char* tag) {
        void* array[32];
        int n = backtrace(array, 32);
        char** symbols = backtrace_symbols(array, n);
        fprintf(stderr, "\n[FPFH_BACKTRACE] %s (tid=%lu) frames=%d\n", tag, (unsigned long)pthread_self(), n);
        for (int i = 0; i < n && i < 20; i++) {
            if (symbols && symbols[i]) fprintf(stderr, "  [%2d] %s\n", i, symbols[i]);
        }
        if (symbols) free(symbols);
        fprintf(stderr, "[FPFH_BACKTRACE] ---\n");
    }
    bool enableBacktraceLog() {
        static bool once = []() {
            const char* e = std::getenv("AUTOMAP_FPFH_BACKTRACE");
            return e && (e[0] == '1' || e[0] == 'y' || e[0] == 'Y');
        }();
        return once;
    }
}

// 全局信号处理器：捕获段错误并记录栈回溯（含 demangle），然后优雅退出避免 core 污染
namespace {
    volatile sig_atomic_t g_sigsegv_flag = 0;
    thread_local bool g_in_fpfh_compute = false;
    static constexpr int kBacktraceMax = 64;

    void sigsegv_handler(int sig) {
        if (g_in_fpfh_compute) {
            g_sigsegv_flag = 1;
            void* array[kBacktraceMax];
            int size = backtrace(array, kBacktraceMax);
            char** strings = backtrace_symbols(array, size);

            fprintf(stderr, "\n");
            fprintf(stderr, "========== SIGSEGV in FPFH compute (tid=%lu) ==========\n", (unsigned long)pthread_self());
            fprintf(stderr, "Signal: %d\n", sig);
            fprintf(stderr, "Stack trace (raw):\n");
            for (int i = 0; i < size && i < 40; i++) {
                if (strings && strings[i]) fprintf(stderr, "  [%2d] %s\n", i, strings[i]);
            }
#ifdef __GNUC__
            fprintf(stderr, "Demangled (best-effort):\n");
            for (int i = 0; i < size && i < 20; i++) {
                if (!strings || !strings[i]) continue;
                char* sym = strings[i];
                char* paren = std::strchr(sym, '(');
                if (paren) {
                    char* start = paren + 1;
                    char* plus = std::strchr(start, '+');
                    char save = plus ? *plus : 0;
                    if (plus) *plus = '\0';
                    int status = -1;
                    char* demangled = abi::__cxa_demangle(start, nullptr, nullptr, &status);
                    if (plus) *plus = save;
                    if (status == 0 && demangled) {
                        fprintf(stderr, "  [%2d] %s\n", i, demangled);
                        free(demangled);
                    } else
                        fprintf(stderr, "  [%2d] %s\n", i, strings[i]);
                } else
                    fprintf(stderr, "  [%2d] %s\n", i, strings[i]);
            }
#endif
            fprintf(stderr, "=============================================\n");
            fflush(stderr);
            if (strings) free(strings);
            _exit(139);
        }
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
 * 单线程FPFH计算（带全局锁）- 静态 PCL 对象版本（避免析构崩溃）
 *
 * 根因：PCL 1.11-1.12 中 FPFHEstimation 析构时释放内部 Eigen 存储会触发 free() 崩溃
 * （与 PCL #4877 同类：Eigen::DenseStorage::~DenseStorage → aligned_free → free）。
 *
 * 策略：
 * 1. 使用 heap 分配 NormalEstimation/FPFHEstimation 且永不 delete（仅进程退出时由 OS 回收），
 *    避免 exit() 阶段静态析构顺序导致的 ~FPFHEstimation → free() SIGSEGV（function-local static 仍会在 exit 时析构）。
 * 2. 不在 compute 后用空 shared_ptr 清空 estimator（会致下次 setInputCloud(valid) 时 PCL 内部 SIGSEGV），下次 compute 直接覆盖即可
 * 3. 入口深拷贝点云，避免外部在计算过程中修改/释放
 * 4. 全局互斥锁保护，SIGSEGV 时 backtrace 后 _exit(139)
 */
FPFHCloudPtr FpfhExtractor::compute(
    const CloudXYZIPtr& cloud,
    float normal_radius,
    float fpfh_radius) const
{
    const unsigned tid = automap_pro::logThreadId();
    g_in_fpfh_compute = true;
    g_sigsegv_flag = 0;
    struct FpfhGuard { ~FpfhGuard() { g_in_fpfh_compute = false; } } guard;

    ALOG_INFO(MOD, "[tid={}] [STEP=0.0] FPFH COMPUTE START cloud_ptr={} cloud_size={} use_count={} normal_r={:.3f} fpfh_r={:.3f}",
              tid, static_cast<const void*>(cloud.get()), cloud ? cloud->size() : 0u, cloud ? cloud.use_count() : 0, normal_radius, fpfh_radius);

    if (!cloud) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.1] fail null cloud", tid);
        return std::make_shared<FPFHCloud>();
    }
    if (cloud->empty()) {
        ALOG_WARN(MOD, "[tid={}] [STEP=0.1] skip empty cloud", tid);
        return std::make_shared<FPFHCloud>();
    }
    size_t cloud_size = cloud->size();
    if (cloud_size > 1000000) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.1] fail cloud too large size={}", tid, cloud_size);
        return std::make_shared<FPFHCloud>();
    }
    if (cloud_size < 10) {
        ALOG_WARN(MOD, "[tid={}] [STEP=0.1] skip cloud too small size={}", tid, cloud_size);
        return std::make_shared<FPFHCloud>();
    }

    ALOG_INFO(MOD, "[tid={}] [STEP=0.2] validate_memory capacity={} width={} height={} is_dense={}",
              tid, cloud->points.capacity(), cloud->width, cloud->height, cloud->is_dense);
    try {
        const auto& p0 = cloud->points[0];
        const auto& pm = cloud->points[cloud_size / 2];
        const auto& pn = cloud->points[cloud_size - 1];
        ALOG_INFO(MOD, "[tid={}] [STEP=0.2] validate_points first=[{:.2f},{:.2f},{:.2f}] mid=[...] last=[{:.2f},{:.2f},{:.2f}]",
                  tid, p0.x, p0.y, p0.z, pn.x, pn.y, pn.z);
        if (!std::isfinite(p0.x) || !std::isfinite(p0.y) || !std::isfinite(p0.z) ||
            !std::isfinite(pm.x) || !std::isfinite(pm.y) || !std::isfinite(pm.z) ||
            !std::isfinite(pn.x) || !std::isfinite(pn.y) || !std::isfinite(pn.z)) {
            ALOG_ERROR(MOD, "[tid={}] [STEP=0.2] fail NaN/Inf in points", tid);
            return std::make_shared<FPFHCloud>();
        }
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.2] validate_exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.2] validate_unknown_exception", tid);
        return std::make_shared<FPFHCloud>();
    }

    if (normal_radius <= 0 || fpfh_radius <= 0) {
        normal_radius = std::max(normal_radius, 0.1f);
        fpfh_radius = std::max(fpfh_radius, 0.1f);
    }
    normal_radius = std::min(normal_radius, 5.0f);
    fpfh_radius = std::min(fpfh_radius, 10.0f);

    // 深拷贝点云，避免外部在计算过程中修改/释放导致 use-after-free
    CloudXYZIPtr working_cloud;
    try {
        working_cloud = std::make_shared<CloudXYZI>();
        *working_cloud = *cloud;
        if (working_cloud->size() != cloud_size) {
            ALOG_ERROR(MOD, "[tid={}] [STEP=0.3] copy size mismatch orig={} copy={}", tid, cloud_size, working_cloud->size());
            return std::make_shared<FPFHCloud>();
        }
    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.3] copy bad_alloc size={} msg={}", tid, cloud_size, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.3] copy exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }
    FPFH_CRASH_LOG_PTR("0.3", "working_cloud_copied", working_cloud.get(), working_cloud->size());

    auto feat = std::make_shared<FPFHCloud>();
    try {
        feat->reserve(cloud_size);
    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=0.4] feat reserve failed msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }

    try {
        ALOG_INFO(MOD, "[tid={}] [STEP=1.0] LOCK ACQUIRE", tid);
        std::lock_guard<std::mutex> lk(compute_mutex_);
        ALOG_INFO(MOD, "[tid={}] [STEP=1.0] LOCK ACQUIRED", tid);
        if (enableBacktraceLog()) logBacktraceToStderr("after_lock_acquired");

    // === 阶段1：法向量（heap 分配、永不 delete，避免 exit 时析构 free 崩溃，见 PCL #4877）===
    // 🏛️ [安全增强] 增加 PCL 1.11 状态机防御，防止 setInputCloud 内部引用计数损坏
    FPFH_CRASH_LOG("1.1", "ne_init_check");
    static pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal>* ne_ptr = nullptr;
    if (!ne_ptr) {
        ne_ptr = new pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal>();
        FPFH_CRASH_LOG("1.1", "ne_ptr_created");
    }
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal>& ne = *ne_ptr;

    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_n;
    try {
        tree_n = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=1.1] kdtree exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }
    FPFH_CRASH_LOG("1.2", "kdtree_created");

    // 🏛️ [P0 修复] 严防 setInputCloud(nullptr/empty)：PCL 1.11 会在下次 replace 引用时解引用 null 指针
    if (!working_cloud || working_cloud->empty()) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=1.3] CRITICAL: working_cloud is empty before ne.setInputCloud", tid);
        return std::make_shared<FPFHCloud>();
    }

    FPFH_CRASH_LOG_PTR("1.3a", "ne_setInputCloud_before", working_cloud.get(), working_cloud->size());
    try {
        ne.setInputCloud(working_cloud);
        ne.setSearchMethod(tree_n);
        ne.setRadiusSearch(normal_radius);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=1.3] ne.setInput/Search exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }
    FPFH_CRASH_LOG("1.5", "ne_params_set");

    FPFH_CRASH_LOG("1.6", "ne_compute_start");
    try {
        ne.compute(*normals);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=1.6] ne.compute exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }
    FPFH_CRASH_LOG_PTR("1.7", "ne_compute_success", normals.get(), normals->size());

    // === 阶段2：FPFH（heap 分配、永不 delete，避免 exit 时析构 Eigen aligned_free 崩溃）===
    FPFH_CRASH_LOG("2.0", "fpfh_init_check");
    static pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>* fpfh_ptr = nullptr;
    if (!fpfh_ptr) {
        fpfh_ptr = new pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>();
        FPFH_CRASH_LOG("2.0", "fpfh_ptr_created");
    }
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33>& fpfh = *fpfh_ptr;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_f;
    try {
        tree_f = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=2.1] fpfh kdtree exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }

    if (!normals || normals->size() != working_cloud->size()) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=2.3] normals size mismatch normals={} cloud={}", tid, normals ? normals->size() : 0u, working_cloud->size());
        return std::make_shared<FPFHCloud>();
    }

    FPFH_CRASH_LOG_PTR("2.3a", "fpfh_setInputCloud_before", working_cloud.get(), working_cloud->size());
    try {
        fpfh.setInputCloud(working_cloud);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(tree_f);
        fpfh.setRadiusSearch(fpfh_radius);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=2.3] fpfh setInput exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }
    FPFH_CRASH_LOG("2.5", "fpfh_params_set");

    FPFH_CRASH_LOG("2.7", "fpfh_compute_start");
    try {
        fpfh.compute(*feat);
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] [STEP=2.7] fpfh.compute exception msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    }
    FPFH_CRASH_LOG_PTR("2.8", "fpfh_compute_success", feat.get(), feat->size());

        if (feat->size() == 0)
            ALOG_WARN(MOD, "[tid={}] [STEP=2.7] compute_empty_features", tid);

        ALOG_INFO(MOD, "[tid={}] [STEP=3.0] FPFH COMPUTE SUCCESS cloud_pts={} features={} feat_ptr={}",
                  tid, cloud_size, feat->size(), static_cast<const void*>(feat.get()));

        // 不再用空 shared_ptr 清空静态 estimator：PCL 1.11-1.12 在 setInputCloud(empty) 后，
        // 下次 setInputCloud(valid) 时会在替换内部引用时解引用已置空状态导致 SIGSEGV（见崩溃栈 #0）。
        // 保持静态 ne/fpfh 持有本次 working_cloud/tree，下次 compute() 会直接覆盖，无需清空。

        return feat;

    } catch (const std::bad_alloc& e) {
        ALOG_ERROR(MOD, "[tid={}] FPFH FAILED bad_alloc msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (const std::runtime_error& e) {
        ALOG_ERROR(MOD, "[tid={}] FPFH FAILED runtime_error msg={}", tid, e.what());
        return std::make_shared<FPFHCloud>();
    } catch (const std::exception& e) {
        ALOG_ERROR(MOD, "[tid={}] FPFH FAILED exception msg={} typeid={}", tid, e.what(), typeid(e).name());
        return std::make_shared<FPFHCloud>();
    } catch (...) {
        ALOG_ERROR(MOD, "[tid={}] FPFH FAILED unknown sigsegv_flag={}", tid, (int)g_sigsegv_flag);
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

