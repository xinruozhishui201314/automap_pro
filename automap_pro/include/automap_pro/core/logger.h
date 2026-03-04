#pragma once
/**
 * AutoMap-Pro 统一日志模块
 *
 * 设计目标：
 *   - 每条日志包含 [模块][文件:行][函数] 三元定位信息
 *   - 支持 spdlog 多 sink：stdout（彩色）+ 滚动文件
 *   - 性能计时宏（AUTOMAP_TIMED_SCOPE）
 *   - 线程安全（spdlog async）
 *   - 可通过环境变量 AUTOMAP_LOG_LEVEL 动态调整级别
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/async.h>
#include <spdlog/fmt/ostr.h>

#include <memory>
#include <string>
#include <chrono>
#include <mutex>
#include <filesystem>

namespace automap_pro {

class Logger {
public:
    static Logger& instance() {
        static Logger inst;
        return inst;
    }

    void init(const std::string& log_dir = "/tmp/automap_logs",
              const std::string& level_str = "info") {
        std::lock_guard<std::mutex> lk(mutex_);
        if (initialized_) return;

        std::filesystem::create_directories(log_dir);

        // 异步日志线程池（8192 队列 + 1 后台线程）
        spdlog::init_thread_pool(8192, 1);

        std::vector<spdlog::sink_ptr> sinks;

        // Sink 1: 彩色 stdout
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_pattern(
            "%^[%Y-%m-%d %H:%M:%S.%e][%L]%$ %v");
        sinks.push_back(console_sink);

        // Sink 2: 滚动日志文件（50MB × 5 文件）
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_dir + "/automap.log", 50 * 1024 * 1024, 5, true);
        file_sink->set_pattern(
            "[%Y-%m-%d %H:%M:%S.%e][%L][tid=%t] %v");
        sinks.push_back(file_sink);

        logger_ = std::make_shared<spdlog::async_logger>(
            "automap",
            sinks.begin(), sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::block);

        // 环境变量覆盖日志级别
        const char* env_level = std::getenv("AUTOMAP_LOG_LEVEL");
        std::string eff_level = env_level ? std::string(env_level) : level_str;
        setLevel(eff_level);

        logger_->flush_on(spdlog::level::warn);
        spdlog::register_logger(logger_);
        spdlog::set_default_logger(logger_);

        initialized_ = true;
        SPDLOG_LOGGER_INFO(logger_,
            "=== AutoMap-Pro Logger initialized (level={}, log_dir={}) ===",
            eff_level, log_dir);
    }

    void setLevel(const std::string& level) {
        if (!logger_) return;
        if      (level == "trace") logger_->set_level(spdlog::level::trace);
        else if (level == "debug") logger_->set_level(spdlog::level::debug);
        else if (level == "info")  logger_->set_level(spdlog::level::info);
        else if (level == "warn")  logger_->set_level(spdlog::level::warn);
        else if (level == "error") logger_->set_level(spdlog::level::err);
        else                       logger_->set_level(spdlog::level::info);
    }

    void flush() { if (logger_) logger_->flush(); }

    std::shared_ptr<spdlog::logger> get() const { return logger_; }

private:
    Logger() = default;
    std::shared_ptr<spdlog::logger> logger_;
    std::mutex mutex_;
    bool initialized_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// 模块前缀宏（精准定位文件:行:函数）
// ─────────────────────────────────────────────────────────────────────────────
#define _AUTOMAP_LOC(mod) \
    fmt::format("[{}][{}:{}][{}] ", mod, \
        std::filesystem::path(__FILE__).filename().string(), __LINE__, __func__)

#define ALOG_TRACE(mod, ...) \
    SPDLOG_LOGGER_TRACE(spdlog::default_logger(), _AUTOMAP_LOC(mod) + fmt::format(__VA_ARGS__))
#define ALOG_DEBUG(mod, ...) \
    SPDLOG_LOGGER_DEBUG(spdlog::default_logger(), _AUTOMAP_LOC(mod) + fmt::format(__VA_ARGS__))
#define ALOG_INFO(mod, ...) \
    SPDLOG_LOGGER_INFO(spdlog::default_logger(),  _AUTOMAP_LOC(mod) + fmt::format(__VA_ARGS__))
#define ALOG_WARN(mod, ...) \
    SPDLOG_LOGGER_WARN(spdlog::default_logger(),  _AUTOMAP_LOC(mod) + fmt::format(__VA_ARGS__))
#define ALOG_ERROR(mod, ...) \
    SPDLOG_LOGGER_ERROR(spdlog::default_logger(), _AUTOMAP_LOC(mod) + fmt::format(__VA_ARGS__))
#define ALOG_CRITICAL(mod, ...) \
    SPDLOG_LOGGER_CRITICAL(spdlog::default_logger(), _AUTOMAP_LOC(mod) + fmt::format(__VA_ARGS__))

// ─────────────────────────────────────────────────────────────────────────────
// 性能计时宏（RAII 作用域计时）
// ─────────────────────────────────────────────────────────────────────────────
class ScopedTimer {
public:
    ScopedTimer(const std::string& mod, const std::string& label, double warn_ms = 100.0)
        : mod_(mod), label_(label), warn_ms_(warn_ms),
          t0_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
        double elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0_).count();
        if (elapsed > warn_ms_) {
            ALOG_WARN(mod_, "⏱ [SLOW] {} took {:.1f}ms (threshold {:.0f}ms)",
                      label_, elapsed, warn_ms_);
        } else {
            ALOG_DEBUG(mod_, "⏱ {} took {:.1f}ms", label_, elapsed);
        }
    }
private:
    std::string mod_, label_;
    double warn_ms_;
    std::chrono::steady_clock::time_point t0_;
};

// 用法：AUTOMAP_TIMED_SCOPE("LoopDetector", "TEASER match", 200.0)
#define AUTOMAP_TIMED_SCOPE(mod, label, warn_ms) \
    automap_pro::ScopedTimer _scoped_timer_##__LINE__(mod, label, warn_ms)

// 简化版（默认 100ms 警告）
#define AUTOMAP_TIMED(mod, label) \
    automap_pro::ScopedTimer _scoped_timer_##__LINE__(mod, label, 100.0)

// ─────────────────────────────────────────────────────────────────────
// 兼容旧代码的LOG_*宏（映射到ALOG_*系列）
// 这些宏用于error_handler.cpp、performance_monitor.cpp等文件
// ─────────────────────────────────────────────────────────────────────
#define LOG_TRACE(msg) ALOG_TRACE("Core", msg)
#define LOG_DEBUG(msg) ALOG_DEBUG("Core", msg)
#define LOG_INFO(msg) ALOG_INFO("Core", msg)
#define LOG_WARN(msg) ALOG_WARN("Core", msg)
#define LOG_ERROR(msg) ALOG_ERROR("Core", msg)
#define LOG_CRITICAL(msg) ALOG_CRITICAL("Core", msg)

} // namespace automap_pro
