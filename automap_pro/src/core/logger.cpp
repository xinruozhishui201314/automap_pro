#include "automap_pro/core/logger.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace automap_pro {

// 线程局部上下文
thread_local std::map<std::string, std::string> Logger::thread_context_;

// LogContext实现
nlohmann::json LogContext::toJson() const {
    nlohmann::json j;
    if (!trace_id.empty()) j["trace_id"] = trace_id;
    if (!session_id.empty()) j["session_id"] = session_id;
    if (!component.empty()) j["component"] = component;
    for (const auto& [k, v] : fields) {
        j[k] = v;
    }
    return j;
}

// PerformanceMetrics实现
nlohmann::json PerformanceMetrics::toJson() const {
    nlohmann::json j;
    if (duration_ms > 0.0) j["duration_ms"] = duration_ms;
    if (memory_mb > 0.0) j["memory_mb"] = memory_mb;
    if (cpu_percent > 0.0) j["cpu_percent"] = cpu_percent;
    if (queue_size >= 0) j["queue_size"] = queue_size;
    return j;
}

// Logger实现
Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

Logger::Logger()
    : level_(LogLevel::INFO)
    , initialized_(false) {
}

Logger::~Logger() {
    shutdown();
}

void Logger::init(const std::string& config_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        return;
    }
    
    // 创建console sink
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v");
    
    // 配置日志文件路径
    std::string log_file = "/tmp/automap_pro/automap_pro.log";
    
    if (!config_path.empty()) {
        try {
            std::ifstream config_file(config_path);
            if (config_file.is_open()) {
                nlohmann::json config;
                config_file >> config;
                
                if (config.contains("log_file")) {
                    log_file = config["log_file"];
                }
                
                if (config.contains("log_level")) {
                    level_ = stringToLevel(config["log_level"]);
                }
                
                if (config.contains("sampling_rates")) {
                    for (auto& [pattern, rate] : config["sampling_rates"].items()) {
                        sampling_rates_[pattern] = rate.get<double>();
                    }
                }
            }
        } catch (...) {
            // 配置加载失败，使用默认值
        }
    }
    
    // 确保日志目录存在
    std::filesystem::path log_path(log_file);
    std::filesystem::create_directories(log_path.parent_path());
    
    // 创建file sink（带滚动）
    try {
        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            log_file, 1024 * 1024 * 100, 10); // 100MB per file, 10 files
        file_sink->set_level(spdlog::level::trace);
        
        // 创建logger
        std::vector<spdlog::sink_ptr> sinks = {console_sink, file_sink};
        logger_ = std::make_shared<spdlog::logger>("automap_pro", sinks.begin(), sinks.end());
        logger_->set_level(spdlog::level::trace);
        logger_->flush_on(spdlog::level::warn);
        
        // 注册到spdlog
        spdlog::register_logger(logger_);
        spdlog::set_default_logger(logger_);
        
        initialized_ = true;
        
        LOG_INFO("Logger initialized. Log file: " + log_file);
        
    } catch (const std::exception& e) {
        // 如果文件sink失败，只用console sink
        logger_ = std::make_shared<spdlog::logger>("automap_pro", console_sink);
        logger_->set_level(spdlog::level::debug);
        logger_->flush_on(spdlog::level::warn);
        spdlog::register_logger(logger_);
        spdlog::set_default_logger(logger_);
        
        initialized_ = true;
        
        std::cerr << "Warning: File sink failed, using console only: " << e.what() << std::endl;
    }
}

void Logger::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_ && logger_) {
        LOG_INFO("Logger shutting down");
        logger_->flush();
        spdlog::shutdown();
        initialized_ = false;
    }
}

void Logger::setLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    level_ = level;
    
    if (logger_) {
        spdlog::level::level_enum spd_level = static_cast<spdlog::level::level_enum>(level);
        logger_->set_level(spd_level);
    }
}

void Logger::setLevel(const std::string& level_str) {
    setLevel(stringToLevel(level_str));
}

LogLevel Logger::getLevel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return level_;
}

void Logger::setTraceId(const std::string& trace_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    trace_id_ = trace_id;
}

std::string Logger::getTraceId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return trace_id_;
}

void Logger::clearTraceId() {
    setTraceId("");
}

void Logger::setSessionId(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    session_id_ = session_id;
}

std::string Logger::getSessionId() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_id_;
}

void Logger::setComponent(const std::string& component) {
    std::lock_guard<std::mutex> lock(mutex_);
    component_ = component;
}

std::string Logger::getComponent() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return component_;
}

void Logger::trace(const std::string& msg) {
    log(LogLevel::TRACE, msg);
}

void Logger::debug(const std::string& msg) {
    log(LogLevel::DEBUG, msg);
}

void Logger::info(const std::string& msg) {
    log(LogLevel::INFO, msg);
}

void Logger::warn(const std::string& msg) {
    log(LogLevel::WARN, msg);
}

void Logger::error(const std::string& msg) {
    log(LogLevel::ERROR, msg);
}

void Logger::fatal(const std::string& msg) {
    log(LogLevel::FATAL, msg);
}

void Logger::log(LogLevel level, const std::string& msg, const char* file, int line) {
    if (!initialized_ || level < level_) {
        return;
    }
    
    auto spd_level = static_cast<spdlog::level::level_enum>(level);
    
    // 构建上下文JSON
    LogContext ctx = buildContext();
    
    if (file) {
        std::string filename = std::filesystem::path(file).filename().string();
        ctx.fields["source_file"] = filename;
        ctx.fields["source_line"] = std::to_string(line);
    }
    
    nlohmann::json log_json;
    log_json["message"] = msg;
    if (!ctx.fields.empty() || !ctx.trace_id.empty() || !ctx.session_id.empty() || !ctx.component.empty()) {
        log_json["context"] = ctx.toJson();
    }
    
    logger_->log(spd_level, "{}", log_json.dump());
}

void Logger::infoWithContext(const LogContext& ctx, const std::string& msg) {
    logWithContext(LogLevel::INFO, ctx, msg);
}

void Logger::logWithContext(LogLevel level, const LogContext& ctx, 
                            const std::string& msg, const char* file, int line) {
    if (!initialized_ || level < level_) {
        return;
    }
    
    auto spd_level = static_cast<spdlog::level::level_enum>(level);
    
    // 合并上下文
    LogContext merged = buildContext();
    merged.trace_id = ctx.trace_id.empty() ? merged.trace_id : ctx.trace_id;
    merged.session_id = ctx.session_id.empty() ? merged.session_id : ctx.session_id;
    merged.component = ctx.component.empty() ? merged.component : ctx.component;
    
    for (const auto& [k, v] : ctx.fields) {
        merged.fields[k] = v;
    }
    
    if (file) {
        std::string filename = std::filesystem::path(file).filename().string();
        merged.fields["source_file"] = filename;
        merged.fields["source_line"] = std::to_string(line);
    }
    
    nlohmann::json log_json;
    log_json["message"] = msg;
    if (!merged.fields.empty() || !merged.trace_id.empty() || !merged.session_id.empty() || !merged.component.empty()) {
        log_json["context"] = merged.toJson();
    }
    
    logger_->log(spd_level, "{}", log_json.dump());
}

void Logger::logPerformance(const std::string& operation, const PerformanceMetrics& metrics) {
    LogContext ctx;
    ctx.component = component_;
    ctx.fields["operation"] = operation;
    
    auto metrics_json = metrics.toJson();
    for (auto& [k, v] : metrics_json.items()) {
        ctx.fields[k] = v.dump();
    }
    
    logWithContext(LogLevel::INFO, ctx, "Performance: " + operation);
}

Logger::ScopedTimer::ScopedTimer(Logger& logger, const std::string& operation)
    : logger_(logger), operation_(operation) {
    start_time_ = std::chrono::steady_clock::now();
}

Logger::ScopedTimer::~ScopedTimer() {
    auto end_time = std::chrono::steady_clock::now();
    double duration_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time_).count();
    
    PerformanceMetrics metrics;
    metrics.duration_ms = duration_ms;
    
    // 添加额外上下文
    for (const auto& [k, v] : context_.fields) {
        context_.fields[k] = v;
    }
    
    logger_.logWithContext(LogLevel::INFO, context_, "Performance: " + operation_);
}

void Logger::ScopedTimer::addContext(const std::string& key, const std::string& value) {
    context_.fields[key] = value;
}

std::unique_ptr<Logger::ScopedTimer> Logger::createTimer(const std::string& operation) {
    return std::make_unique<ScopedTimer>(*this, operation);
}

void Logger::addContextField(const std::string& key, const std::string& value) {
    thread_context_[key] = value;
}

void Logger::clearContextFields() {
    thread_context_.clear();
}

void Logger::setSamplingRate(const std::string& pattern, double rate) {
    std::lock_guard<std::mutex> lock(mutex_);
    sampling_rates_[pattern] = rate;
}

rclcpp::Logger Logger::getRclcppLogger(const std::string& name) {
    return rclcpp::get_logger(name);
}

LogContext Logger::buildContext() const {
    LogContext ctx;
    ctx.trace_id = trace_id_;
    ctx.session_id = session_id_;
    ctx.component = component_;
    
    // 添加线程局部字段
    for (const auto& [k, v] : thread_context_) {
        ctx.fields[k] = v;
    }
    
    return ctx;
}

std::string Logger::levelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO";
        case LogLevel::WARN:  return "WARN";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

LogLevel Logger::stringToLevel(const std::string& level_str) const {
    std::string upper = level_str;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    
    if (upper == "TRACE") return LogLevel::TRACE;
    if (upper == "DEBUG") return LogLevel::DEBUG;
    if (upper == "INFO")  return LogLevel::INFO;
    if (upper == "WARN" || upper == "WARNING") return LogLevel::WARN;
    if (upper == "ERROR") return LogLevel::ERROR;
    if (upper == "FATAL") return LogLevel::FATAL;
    
    return LogLevel::INFO; // 默认
}

bool Logger::shouldLog(const std::string& pattern) const {
    auto it = sampling_rates_.find(pattern);
    if (it == sampling_rates_.end()) {
        return true; // 无采样配置，全部记录
    }
    
    auto& counter = const_cast<std::map<std::string, uint64_t>&>(sampling_counters_)[pattern];
    counter++;
    return (counter % static_cast<uint64_t>(1.0 / it->second)) == 0;
}

} // namespace automap_pro
