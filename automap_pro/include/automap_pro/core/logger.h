#pragma once

#include <string>
#include <memory>
#include <map>
#include <mutex>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <nlohmann/json.hpp>

namespace automap_pro {

// 日志级别定义
enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
    FATAL = 5
};

// 结构化日志上下文
struct LogContext {
    std::string trace_id;
    std::string session_id;
    std::string component;
    std::map<std::string, std::string> fields;
    
    nlohmann::json toJson() const;
};

// 性能指标
struct PerformanceMetrics {
    double duration_ms{0.0};
    double memory_mb{0.0};
    double cpu_percent{0.0};
    int queue_size{0};
    
    nlohmann::json toJson() const;
};

// 日志记录器
class Logger {
public:
    static Logger& instance();
    
    void init(const std::string& config_path = "");
    void shutdown();
    
    // 设置日志级别
    void setLevel(LogLevel level);
    void setLevel(const std::string& level_str);
    LogLevel getLevel() const;
    
    // Trace ID管理
    void setTraceId(const std::string& trace_id);
    std::string getTraceId() const;
    void clearTraceId();
    
    // 会话ID管理
    void setSessionId(const std::string& session_id);
    std::string getSessionId() const;
    
    // 组件设置
    void setComponent(const std::string& component);
    std::string getComponent() const;
    
    // 核心日志接口（单参数：std::string 或 const char*）
    void trace(const std::string& msg);
    void debug(const std::string& msg);
    void info(const std::string& msg);
    void warn(const std::string& msg);
    void error(const std::string& msg);
    void fatal(const std::string& msg);
    void trace(const char* msg) { trace(msg ? std::string(msg) : std::string()); }
    void debug(const char* msg) { debug(msg ? std::string(msg) : std::string()); }
    void info(const char* msg)  { info(msg ? std::string(msg) : std::string()); }
    void warn(const char* msg)  { warn(msg ? std::string(msg) : std::string()); }
    void error(const char* msg) { error(msg ? std::string(msg) : std::string()); }
    void fatal(const char* msg) { fatal(msg ? std::string(msg) : std::string()); }
    
    // 带上下文的日志
    void infoWithContext(const LogContext& ctx, const std::string& msg);
    void logWithContext(LogLevel level, const LogContext& ctx,
                        const std::string& msg, const char* file = nullptr, int line = 0);
    
    // 性能监控日志
    void logPerformance(const std::string& operation, const PerformanceMetrics& metrics);
    
    // 性能计时器
    class ScopedTimer {
    public:
        ScopedTimer(Logger& logger, const std::string& operation);
        ~ScopedTimer();
        
        void addContext(const std::string& key, const std::string& value);
        
    private:
        Logger& logger_;
        std::string operation_;
        std::chrono::steady_clock::time_point start_time_;
        LogContext context_;
    };
    
    std::unique_ptr<ScopedTimer> createTimer(const std::string& operation);
    
    // 添加上下文字段（线程局部）
    void addContextField(const std::string& key, const std::string& value);
    void clearContextFields();
    
    // 日志采样配置
    void setSamplingRate(const std::string& pattern, double rate);
    
    // 获取ROS2 logger（用于兼容）
    rclcpp::Logger getRclcppLogger(const std::string& name);
    
    // 获取底层spdlog logger
    std::shared_ptr<spdlog::logger> getSpdlogLogger() { return logger_; }
    
private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    void log(LogLevel level, const std::string& msg, const char* file = nullptr, int line = 0);
    
    LogContext buildContext() const;
    std::string levelToString(LogLevel level) const;
    LogLevel stringToLevel(const std::string& level_str) const;
    
    bool shouldLog(const std::string& pattern) const;
    
    std::shared_ptr<spdlog::logger> logger_;
    
    std::string trace_id_;
    std::string session_id_;
    std::string component_;
    
    LogLevel level_;
    bool initialized_;
    mutable std::mutex mutex_;
    
    // 日志采样配置
    std::map<std::string, double> sampling_rates_;
    std::map<std::string, uint64_t> sampling_counters_;
    
    // 线程局部上下文
    thread_local static std::map<std::string, std::string> thread_context_;
};

// 便捷宏（单参数：字符串或 std::string，多参数时请在调用处先拼接成字符串）
#define LOG_TRACE(msg) Logger::instance().trace(msg)
#define LOG_DEBUG(msg) Logger::instance().debug(msg)
#define LOG_INFO(msg)  Logger::instance().info(msg)
#define LOG_WARN(msg)  Logger::instance().warn(msg)
#define LOG_ERROR(msg) Logger::instance().error(msg)
#define LOG_FATAL(msg) Logger::instance().fatal(msg)

#define LOG_TIMER(op) auto CONCAT(timer_, __LINE__) = Logger::instance().createTimer(op)

// 字符串拼接宏
#define CONCAT_IMPL(x, y) x##y
#define CONCAT(x, y) CONCAT_IMPL(x, y)

} // namespace automap_pro
