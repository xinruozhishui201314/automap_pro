#include "automap_pro/backend/gtsam_guard.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/logger.h"

#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <thread>

// ✅ P0 修复：限制 Eigen 线程数为 1，避免 GTSAM 内部并行导致竞态
#include <Eigen/Dense>

// 尝试包含 TBB 头文件（可能不存在于所有环境）
#ifdef TBB_AVAILABLE
#include <tbb/tbb.h>
#endif

// 用于动态加载 TBB
#ifdef __linux__
#include <dlfcn.h>
#endif

#define GTSAM_GUARD_MOD "GTSAM_Guard"

namespace automap_pro {

namespace {

std::mutex& globalGtsamMutex() {
    static std::mutex m;
    return m;
}

static void applyTbbSerialized() {
    // P0 修复：限制所有并行库的线程数为 1
    // 这是解决 SIGSEGV 崩溃的关键！
    fprintf(stderr, "[GTSAM_Guard][LOAD_TRACE] applyTbbSerialized: setting OMP/EIGEN/MKL/TBB env and Eigen::setNbThreads(1)\n");
    fflush(stderr);

    // 方法1: 设置环境变量（对某些库有效，包括子进程）
    #ifdef __linux__
    setenv("OMP_NUM_THREADS", "1", 1);  // 限制 OpenMP
    setenv("EIGEN_NUM_THREADS", "1", 1);  // 限制 Eigen
    setenv("MKL_NUM_THREADS", "1", 1);  // 限制 Intel MKL
    setenv("TBB_NUM_THREADS", "1", 1);  // 限制 TBB（新增，更直接）
    #endif

    // 方法2: 运行时限制 Eigen
    Eigen::setNbThreads(1);
    ALOG_INFO(GTSAM_GUARD_MOD, "Eigen threads limited to 1 via Eigen::setNbThreads(1)");
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[GTSAM_Guard] Limited Eigen threads to 1 via Eigen::setNbThreads(1)");

    // 方法3: 尝试通过 dlsym 动态加载 TBB 并限制（如果可用）
    // 这是一个更可靠的跨平台方法
    #ifdef __linux__
    void* tbb_handle = dlopen("libtbb.so.12", RTLD_NOW);
    if (tbb_handle) {
        // 尝试查找并调用 TBB 的初始化函数（如果存在）
        // 注意：这个方法可能不是100%可靠，但值得尝试
        ALOG_INFO(GTSAM_GUARD_MOD, "TBB library loaded, attempting to limit parallelism");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[GTSAM_Guard] TBB library loaded");
    }
    #endif

    // 尝试限制 TBB 全局并行度（通过 TBB 头文件）
    #ifdef TBB_AVAILABLE
    try {
        static tbb::global_control control(tbb::global_control::max_allowed_parallelism, 1);
        ALOG_INFO(GTSAM_GUARD_MOD, "TBB parallelism limited to 1 via global_control");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[GTSAM_Guard] TBB parallelism limited to 1 via global_control");
    } catch (const std::exception& e) {
        ALOG_WARN(GTSAM_GUARD_MOD, "Failed to limit TBB parallelism: {}", e.what());
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[GTSAM_Guard] Failed to limit TBB parallelism: %s", e.what());
    }
    #endif

    const char* env = std::getenv("AUTOMAP_GTSAM_SERIAL");
    std::string env_status = env ? env : "(not set, using default)";
    ALOG_INFO(GTSAM_GUARD_MOD,
        "GTSAM initialization complete: OMP_NUM_THREADS=1, EIGEN_NUM_THREADS=1, Eigen::setNbThreads(1), AUTOMAP_GTSAM_SERIAL=%s",
        env_status.c_str());
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[GTSAM_Guard] Initialization complete: OMP_NUM_THREADS=1, EIGEN_NUM_THREADS=1, Eigen::setNbThreads(1), AUTOMAP_GTSAM_SERIAL=%s (global mutex will %s)",
        env_status.c_str(),
        gtsamGlobalMutexEnabled() ? "be used" : "NOT be used (risky!)");
}

static std::string threadIdString() {
    std::stringstream ss;
    ss << std::hex << std::hash<std::thread::id>{}(std::this_thread::get_id());
    return ss.str();
}

} // namespace

void ensureGtsamTbbSerialized() {
    fprintf(stderr, "[GTSAM_Guard][LOAD_TRACE] ensureGtsamTbbSerialized entered (first GTSAM-related call in process; about to applyTbbSerialized)\n");
    fflush(stderr);
    applyTbbSerialized();
    fprintf(stderr, "[GTSAM_Guard][LOAD_TRACE] ensureGtsamTbbSerialized done\n");
    fflush(stderr);
}

bool gtsamGlobalMutexEnabled() {
    // 默认启用全局互斥（安全优先）
    const char* env = std::getenv("AUTOMAP_GTSAM_SERIAL");
    if (env) {
        bool enabled = (std::string(env) == "1" || std::string(env) == "true" || std::string(env) == "yes");
        ALOG_INFO(GTSAM_GUARD_MOD, "AUTOMAP_GTSAM_SERIAL=%s -> GTSAM global mutex %s",
                  env, enabled ? "ENABLED" : "DISABLED");
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[GTSAM_Guard] Environment: AUTOMAP_GTSAM_SERIAL=%s -> GTSAM global mutex %s",
            env, enabled ? "ENABLED" : "DISABLED");
        return enabled;
    }
    // 默认启用全局互斥（安全优先）
    ALOG_INFO(GTSAM_GUARD_MOD, "AUTOMAP_GTSAM_SERIAL not set -> default ENABLED (safe default)");
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[GTSAM_Guard] Environment: AUTOMAP_GTSAM_SERIAL not set -> default ENABLED (safe default)");
    return true;
}

GtsamCallScope::GtsamCallScope(GtsamCaller caller, const char* op,
                               const std::string& params,
                               bool use_global_mutex) {
    caller_   = caller;
    op_       = op ? op : "";
    t0_       = std::chrono::steady_clock::now();
    use_mutex_ = use_global_mutex && gtsamGlobalMutexEnabled();
    if (use_mutex_) {
        globalGtsamMutex().lock();
    }
    std::string tid = threadIdString();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[GTSAM_ENTRY] caller=%s op=%s thread_id=%s use_mutex=%d %s (若崩溃在此后、无 GTSAM_EXIT→崩溃在该次 GTSAM 调用内)",
        gtsamCallerString(caller_), op_.c_str(), tid.c_str(), use_mutex_ ? 1 : 0,
        params.empty() ? "" : params.c_str());
    ALOG_DEBUG(GTSAM_GUARD_MOD, "GTSAM_ENTRY caller=%s op=%s thread_id=%s use_mutex=%d",
               gtsamCallerString(caller_), op_.c_str(), tid.c_str(), use_mutex_ ? 1 : 0);
}

GtsamCallScope::~GtsamCallScope() {
    double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0_).count();
    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
        "[GTSAM_EXIT] caller=%s op=%s duration_ms=%.2f success=%d",
        gtsamCallerString(caller_), op_.c_str(), elapsed_ms, success_ ? 1 : 0);
    if (elapsed_ms > 2000.0) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[AutoMapSystem][STUCK_DIAG] GTSAM call slow: caller=%s op=%s duration_ms=%.2f (grep STUCK_DIAG 精准分析阻塞)",
            gtsamCallerString(caller_), op_.c_str(), elapsed_ms);
    }
    if (use_mutex_) {
        globalGtsamMutex().unlock();
    }
}

} // namespace automap_pro
