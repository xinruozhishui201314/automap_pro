#pragma once

/**
 * 后端任务超时与可观测选项（V2）
 *
 * 用于将阻塞调用（如 iSAM2 update、HBA 调用）统一为带超时的执行，
 * 避免长时间卡死；配合 MetricsRegistry / HealthMonitor 提供可观测性。
 */

#include <chrono>
#include <functional>
#include <future>
#include <optional>

namespace automap_pro {

/** 单次后端任务选项 */
struct BackendTaskOptions {
    std::chrono::milliseconds timeout_ms{0};  // 0 = 不限制
    bool cancel_on_timeout = true;            // 超时后是否视为取消（当前仅报告，无法真正中断 GTSAM）
};

/**
 * 在独立线程中执行可调用对象，等待最多 timeout_ms；超时返回 nullopt。
 * 注意：无法真正中断正在执行的 GTSAM/HBA，超时仅停止等待，子线程仍可能继续运行。
 * 用于：建图结束时的 waitForPendingTasks 可改为带超时；或外层对 commitAndUpdate 做“最大等待”包装。
 */
template <typename F, typename R = std::invoke_result_t<F>>
std::optional<R> runWithTimeout(F&& callable, std::chrono::milliseconds timeout_ms) {
    if (timeout_ms.count() <= 0) {
        return callable();
    }
    auto fut = std::async(std::launch::async, std::forward<F>(callable));
    if (fut.wait_for(timeout_ms) != std::future_status::ready) {
        return std::nullopt;  // 超时
    }
    return fut.get();
}

}  // namespace automap_pro
