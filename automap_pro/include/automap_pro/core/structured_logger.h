#pragma once
/**
 * AutoMap-Pro 结构化日志系统
 *
 * 设计目标:
*   - 支持 JSON 格式日志(便于 ELK/Splunk 等日志分析系统)
*   - 全链路追踪 (trace_id, span, 父子 span)
*   - 结构化日志字段(时间戳,级别,模块,错误码等)
*   - 支持日志采样和过滤
*   - 线程安全
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/async.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/ostr.h>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <random>
#include <sstream>
#include <iomanip>
#include <thread>
#include <unistd.h>
#include <fstream>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// Trace 上下文管理器（线程局部存储）
// ─────────────────────────────────────────────────────────────────────────────
class TraceContext {
public:
    std::string trace_id;
    std::string span_id;
    std::string parent_span_id;
    std::string operation;

    static thread_local std::string current_module;

    static TraceContext& current() {
        static thread_local TraceContext current_;
        return current_;
    }

    static TraceContext create(const std::string& op) {
        TraceContext ctx;
        ctx.trace_id = current().trace_id.empty() ? generateTraceId() : current().trace_id;
        ctx.span_id = generateSpanId();
        ctx.parent_span_id = current().span_id;
        ctx.operation = op;
        return ctx;
    }

    static TraceContext child(const std::string& op) {
        TraceContext ctx;
        ctx.trace_id = current().trace_id;
        ctx.span_id = generateSpanId();
        ctx.parent_span_id = current().span_id;
        ctx.operation = op;
        return ctx;
    }

    static std::string generateTraceId() {
        std::random_device rd;
        std::uniform_int_distribution<uint64_t> dist(0, UINT64_MAX);
        std::stringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0')
                  << dist(rd) << std::hex
                  << std::setw(8) << std::setfill('0')
                  << (static_cast<uint32_t>(rd()) & 0xFFFF);
        return ss.str();
    }

    static std::string generateSpanId() {
        std::random_device rd;
        std::uniform_int_distribution<uint64_t> dist(0, UINT64_MAX);
        std::stringstream ss;
        ss << std::hex << std::setw(16) << std::setfill('0')
                  << dist(rd) << std::hex
                  << std::setw(8) << std::setfill('0')
                  << (static_cast<uint32_t>(rd()) & 0xFF);
        return ss.str();
    }

    static void setModule(const std::string& mod) {
        current_module = mod;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// 结构化日志字段
// ─────────────────────────────────────────────────────────────────────────────
struct StructuredLogEntry {
    std::string timestamp;       // ISO 8601 格式
    std::string level;           // trace/debug/info/warn/error/critical
    std::string module;           // 模块名称
    std::string trace_id;        // 链路追踪ID
    std::string span_id;         // 当前 Span ID
    std::string parent_span_id;  // 父 Span ID
    std::string operation;       // 操作名称
    uint32_t error_code = 0;        // 错误码 (可选)
    std::string message;         // 日志消息
    double duration_ms = 0.0;        // 耗时(可选)
    std::map<std::string, std::string> metadata;  // 附加元数据

    std::string toJson() const {
        nlohmann::json j;
        j["timestamp"] = timestamp;
        j["level"] = level;
        j["module"] = module;
        if (!trace_id.empty()) j["trace_id"] = trace_id;
        if (!span_id.empty()) j["span_id"] = span_id;
        if (!parent_span_id.empty()) j["parent_span_id"] = parent_span_id;
        if (!operation.empty()) j["operation"] = operation;
        if (error_code != 0) j["error_code"] = error_code;
        j["message"] = message;
        if (duration_ms >= 0) j["duration_ms"] = duration_ms;
        for (const auto& [k, v] : metadata) {
            j["metadata"][k] = v;
        }
        return j.dump();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// 结构化日志配置
// ─────────────────────────────────────────────────────────────────────────────
struct StructuredLoggerConfig {
    bool enable_json_log = true;           // 启用 JSON 文件
    bool enable_console_log = true;       // 启用控制台输出
    bool enable_structured_fields = true; // 启用结构化字段
    std::string json_log_path = "/tmp/automap_logs/structured.json"; // JSON 日志路径
    size_t json_log_max_size = 100 * 1024 * 1024; // JSON 日志最大大小 (100MB)
    size_t json_log_max_files = 5;        // JSON 日志最大文件数
    std::string log_level = "info";        // 日志级别
    std::vector<std::string> sampled_modules; // 采样模块列表 (为空表示全部采样)
    double sample_rate = 1.0;            // 采样率 (0.0-1.0, 1.0 表示全部)
    bool include_thread_id = true;       // 包含线程ID
    bool include_hostname = true;       // 包含主机名
};

// ─────────────────────────────────────────────────────────────────────────────
// 结构化日志器
// ─────────────────────────────────────────────────────────────────────────────
class StructuredLogger {
public:
    static StructuredLogger& instance() {
        static StructuredLogger inst;
        return inst;
    }

    void init(const StructuredLoggerConfig& config = StructuredLoggerConfig()) {
        std::lock_guard<std::mutex> lk(mutex_);
        if (initialized_) return;

        config_ = config;
        initSpdlog();

        if (config_.enable_json_log) {
            initJsonLog();
        }

        initialized_ = true;
        ALOG_INFO("StructuredLogger", "Structured logger initialized (json={}, console={})",
                     config_.enable_json_log, config_.enable_console_log);
    }

    // 核心日志方法
    void trace(const std::string& module, const std::string& msg,
                const std::map<std::string, std::string>& metadata = {}) {
 log(spdlog::level::trace, module, msg, metadata); }
    void debug(const std::string& module, const std::string& msg,
                const std::map<std::string, std::string>& metadata = {}) { log(spdlog::level::debug, module, msg, metadata); }
    void info(const std::string& module, const std::string& msg,
               const std::map<std::string, std::string>& metadata = {}) { log(spdlog::level::info, module, msg, metadata); }
    void warn(const std::string& module, const std::string& msg,
               const std::map<std::string, std::string>& metadata = {}) { log(spdlog::level::warn, module, msg, metadata); }
    void error(const std::string& module, const std::string& msg,
                const std::map<std::string, std::string>& metadata = {}) { log(spdlog::level::err, module, msg, metadata); }
    void critical(const std::string& module, const std::string& msg,
                  const std::map<std::string, std::string>& metadata = {}) { log(spdlog::level::critical, module, msg, metadata); }

    // 错误日志（带错误码)
    void logError(const std::string& module, uint32_t error_code, const std::string& msg,
                  const std::map<std::string, std::string>& metadata = {}) {
        auto entry = createEntry(module, "error", msg, metadata);
        entry.error_code = error_code;
        writeStructuredLog(entry);
        spdlog::error("[{}] {} [code=0x{:04X}]", module, msg, error_code);
    }

    // 性能计时日志
    void logTiming(const std::string& module, const std::string& operation,
                   double duration_ms,
                   const std::map<std::string, std::string>& metadata = {}) {
        auto entry = createEntry(module, "info", operation + " completed", metadata);
        entry.operation = operation;
        entry.duration_ms = duration_ms;
        writeStructuredLog(entry);
        ALOG_DEBUG("StructuredLogger", "[{}] {} took {:.2f}ms", module, operation, duration_ms);
    }

    // 事件日志
    void logEvent(const std::string& module, const std::string& event_type,
                  const std::string& msg,
                  const std::map<std::string, std::string>& metadata = {}) {
        auto entry = createEntry(module, "info", msg, metadata);
        entry.metadata["event_type"] = event_type;
        writeStructuredLog(entry);
        ALOG_INFO("StructuredLogger", "[{}] EVENT: {}", module, msg);
    }
    // 获取日志条目
    StructuredLogEntry createEntry(const std::string& module, const std::string& level,
                                    const std::string& msg,
                                    const std::map<std::string, std::string>& metadata = {}) {
        StructuredLogEntry entry;
        entry.timestamp = getTimestampISO();
        entry.level = level;
        entry.module = module;
        entry.message = msg;

        // 复制 trace context
        const auto& ctx = TraceContext::current();
        entry.trace_id = ctx.trace_id;
        entry.span_id = ctx.span_id;
        entry.parent_span_id = ctx.parent_span_id;
        entry.operation = ctx.operation;

        entry.metadata = metadata;

        if (config_.include_thread_id) {
            entry.metadata["thread_id"] = std::to_string(
                std::hash<std::thread::id>{}(std::this_thread::get_id()));
        }
        if (config_.include_hostname) {
            char hostname[256];
            gethostname(hostname, sizeof(hostname));
            entry.metadata["hostname"] = hostname;
        }
        return entry;
    }
    // 写入结构化日志
    void writeStructuredLog(const StructuredLogEntry& entry) {
        std::lock_guard<std::mutex> lk(json_mutex_);
        if (!shouldSample(entry.module)) {
            return;
        }
        if (json_file_.is_open()) {
            json_file_ << entry.toJson() << std::endl;
        }
    }
    // 采样检查
    bool shouldSample(const std::string& module) {
        if (config_.sampled_modules.empty()) {
            return true;
        }
        for (const auto& m : config_.sampled_modules) {
            if (module.find(m) != std::string::npos) {
                return true;
            }
        }
        // 检查采样率
        std::uniform_real_distribution<double> dis(0.0, 1.0);
        return dis(gen_) < config_.sample_rate;
    }
    // 获取 ISO 时间戳
    std::string getTimestampISO() const {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count() % 1000000;
        std::stringstream ss;
        ss << std::put_time(std::gmtime(&t), "%Y-%m-%dT %H:%M:%S");
        ss << "." << std::setfill('6') << ms;
        return ss.str();
    }
    // spdlog 初始化
    void initSpdlog() {
        spdlog::init_thread_pool(8192, 1);
        std::vector<spdlog::sink_ptr> sinks;

        if (config_.enable_console_log) {
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%L][%n] %v");
            sinks.push_back(console_sink);
        }

        if (config_.enable_json_log) {
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                config_.json_log_path + ".log",
                config_.json_log_max_size,
                config_.json_log_max_files
            );
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e][%L][%n] %v");
            sinks.push_back(file_sink);
        }

        if (!sinks.empty()) {
            if (config_.enable_json_log && config_.enable_console_log) {
                json_logger_ = std::make_shared<spdlog::async_logger>(
                    "automap_json", sinks.begin(), sinks.end(),
                    spdlog::thread_pool(),
                    spdlog::async_overflow_policy::block
                );
                console_logger_ = json_logger_;
            } else if (config_.enable_json_log) {
                json_logger_ = std::make_shared<spdlog::async_logger>(
                    "automap_json", sinks.begin(), sinks.end(),
                    spdlog::thread_pool(),
                    spdlog::async_overflow_policy::block
                );
            } else {
                console_logger_ = std::make_shared<spdlog::async_logger>(
                    "automap_console", sinks.begin(), sinks.end(),
                    spdlog::thread_pool(),
                    spdlog::async_overflow_policy::block
                );
            }
            spdlog::register_logger(json_logger_ ? json_logger_ : console_logger_);
        }

        setLevel(config_.log_level);
    }
    // JSON 日志初始化
    void initJsonLog() {
        std::filesystem::path p(config_.json_log_path);
        p = p.parent_path();
        std::filesystem::create_directories(p);
        json_file_.open(config_.json_log_path, std::ios::app);
    }
    // 核心日志实现
    void log(spdlog::level::level_enum lvl, const std::string& module,
                 const std::string& msg, const std::map<std::string, std::string>& metadata) {
        auto entry = createEntry(module, levelToString(lvl), msg, metadata);
        writeStructuredLog(entry);
        auto logger = json_logger_ ? json_logger_ : console_logger_;
        if (logger) {
            logger->log(lvl, "[{}] {}", module, msg);
        }
    }
    // 级别转换
    std::string levelToString(spdlog::level::level_enum lvl) const {
        switch (lvl) {
            case spdlog::level::trace: return "trace";
            case spdlog::level::debug: return "debug";
            case spdlog::level::info:  return "info";
            case spdlog::level::warn:  return "warn";
            case spdlog::level::err:  return "error";
            case spdlog::level::critical: return "critical";
            default: return "unknown";
        }
    }
    // 设置日志级别
    void setLevel(const std::string& level) {
        auto logger = json_logger_ ? json_logger_ : console_logger_;
        if (!logger) return;
        if      (level == "trace")    logger->set_level(spdlog::level::trace);
        else if (level == "debug")   logger->set_level(spdlog::level::debug);
        else if (level == "info")    logger->set_level(spdlog::level::info);
        else if (level == "warn")    logger->set_level(spdlog::level::warn);
        else if (level == "error")   logger->set_level(spdlog::level::err);
        else if (level == "critical") logger->set_level(spdlog::level::critical);
        else                      logger->set_level(spdlog::level::info);
    }

private:
    StructuredLogger() = default;
    std::shared_ptr<spdlog::logger> json_logger_;
    std::shared_ptr<spdlog::logger> console_logger_;
    std::ofstream json_file_;
    std::mutex mutex_;
    std::mutex json_mutex_;
    StructuredLoggerConfig config_;
    bool initialized_ = false;
    std::mt19937 gen_{std::random_device{}()};
};

// ─────────────────────────────────────────────────────────────────────────────
// RAII 计时器
// ─────────────────────────────────────────────────────────────────────────────
class ScopedStructuredTimer {
public:
    ScopedStructuredTimer(const std::string& module, const std::string& operation,
                        double warn_ms = 100.0)
        : module_(module), operation_(operation), warn_ms_(warn_ms),
          t0_(std::chrono::steady_clock::now()) {}

    ~ScopedStructuredTimer() {
        auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0_).count();
        StructuredLogger::instance().logTiming(module_, operation_, elapsed, {});

        if (elapsed > warn_ms_) {
            StructuredLogger::instance().warn(
                module_,
                "[SLOW] {} took {:.1f}ms (threshold {:.0f}ms)",
                {{"actual_ms", std::to_string(elapsed)},
                 {"threshold_ms", std::to_string(warn_ms_)}
            });
        }
    }

private:
    std::string module_;
    std::string operation_;
    double warn_ms_;
    std::chrono::steady_clock::time_point t0_;
};

// ─────────────────────────────────────────────────────────────────────────────
// 便捷宏
// ─────────────────────────────────────────────────────────────────────────────

#define SLOG_TRACE(mod, msg, ...) \
    automap_pro::StructuredLogger::instance().trace(mod, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_DEBUG(mod, msg, ...) \
    automap_pro::StructuredLogger::instance().debug(mod, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_INFO(mod, msg, ...) \
    automap_pro::StructuredLogger::instance().info(mod, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_WARN(mod, msg, ...) \
    automap_pro::StructuredLogger::instance().warn(mod, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_ERROR(mod, msg, ...) \
    automap_pro::StructuredLogger::instance().error(mod, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_CRITICAL(mod, msg, ...) \
    automap_pro::StructuredLogger::instance().critical(mod, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_ERROR_CODE(mod, code, msg, ...) \
    automap_pro::StructuredLogger::instance().logError(mod, code, fmt::format(msg, ##__VA_ARGS__))
#define SLOG_TIMING(mod, op, ms) \
    automap_pro::StructuredLogger::instance().logTiming(mod, op, ms)
#define SLOG_EVENT(mod, type, msg, ...) \
    automap_pro::StructuredLogger::instance().logEvent(mod, type, fmt::format(msg, ##__VA_ARGS__))

// RAII 计时
#define SLOG_TIMED_SCOPE(mod, op, warn_ms) \
    automap_pro::ScopedStructuredTimer _slog_timer_##__LINE__(mod, op, warn_ms)
#define SLOG_TIMED(mod, op) \
    automap_pro::ScopedStructuredTimer _slog_timer_##__LINE__(mod, op, 100.0)

// Span 辅助宏
#define SLOG_START_SPAN(mod, op) \
    automap_pro::TraceContext::current() = automap_pro::TraceContext::create(op)
#define SLOG_END_SPAN() \
    automap_pro::TraceContext::current() = automap_pro::TraceContext()
#define SLOG_CHILD_SPAN(mod, op) \
    automap_pro::TraceContext::child(op)
#define SLOG_SET_MODULE(mod) \
    automap_pro::TraceContext::setModule(mod)

} // namespace automap_pro
