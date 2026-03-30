#pragma once
/**
 * @file backend/gtsam_guard.h
 * @brief 后端优化：iSAM2、HBA、GPS/回环因子、任务队列与坐标管理。
 */

/**
 * GTSAM 多路使用防护与统一日志
 *
 * 同一进程内多处使用 GTSAM（iSAM2、Optimizer、HBA PGO）时：
 * - 通过全局互斥串行化 GTSAM 调用，避免 TBB 并发容器竞态导致 SIGSEGV
 * - 统一 [GTSAM_ENTRY] / [GTSAM_EXIT] 日志，崩溃时最后一条 ENTRY 即出事调用点
 *
 * 使用方式：在每次进入 GTSAM 前构造 GtsamCallScope，退出作用域时自动打 EXIT 日志并释放锁。
 * 定位崩溃：grep -E 'GTSAM_ENTRY|GTSAM_EXIT' full.log | tail -20
 */

#include <chrono>
#include <mutex>
#include <string>

namespace automap_pro {

/** 调用方标识，用于日志与统计 */
enum class GtsamCaller {
    ISAM2,    ///< IncrementalOptimizer (commitAndUpdate / addGPSFactorsBatch)
    Optimizer,///< Optimizer::optimizeGTSAM (位姿图批量优化)
    HBA,      ///< HBA PGO (hba_api LevenbergMarquardtOptimizer)
};

/** 将 GtsamCaller 转为短字符串 */
inline const char* gtsamCallerString(GtsamCaller c) {
    switch (c) {
        case GtsamCaller::ISAM2:     return "ISAM2";
        case GtsamCaller::Optimizer: return "Optimizer";
        case GtsamCaller::HBA:       return "HBA";
        default:                     return "Unknown";
    }
}

/**
 * RAII：进入 GTSAM 时打 ENTRY 并可选加锁，离开时打 EXIT（含耗时、success）并解锁。
 * 崩溃时若仅有 ENTRY 无 EXIT，即可判定崩溃发生在该次 GTSAM 调用内部。
 */
class GtsamCallScope {
public:
    /**
     * @param caller 调用方 (ISAM2 / Optimizer / HBA)
     * @param op     操作名，如 "commitAndUpdate", "optimize", "PGO"
     * @param params 可选键值对，如 "pending_factors=2 pending_values=1"
     * @param use_global_mutex 是否在构造时加全局 GTSAM 锁（推荐 true，串行化多路调用）
     */
    GtsamCallScope(GtsamCaller caller, const char* op,
                   const std::string& params = "",
                   bool use_global_mutex = true);

    ~GtsamCallScope();

    /** 在正常返回前调用，标记成功；若未调用则 EXIT 日志中 success=0 */
    void setSuccess(bool success) { success_ = success; }

    GtsamCallScope(const GtsamCallScope&) = delete;
    GtsamCallScope& operator=(const GtsamCallScope&) = delete;

private:
    GtsamCaller caller_;
    std::string op_;
    std::chrono::steady_clock::time_point t0_;
    bool success_ = false;
    bool use_mutex_ = false;
};

/**
 * 进程启动时调用一次：将 TBB 最大并行度设为 1，避免 GTSAM 内部 TBB 并发导致 SIGSEGV。
 * 在 automap_system 或 backend 首次使用 GTSAM 前调用；多次调用仅首次生效。
 */
void ensureGtsamTbbSerialized();

/** 是否启用全局 GTSAM 互斥（可通过环境变量 AUTOMAP_GTSAM_SERIAL=1 覆盖默认 true） */
bool gtsamGlobalMutexEnabled();

} // namespace automap_pro
