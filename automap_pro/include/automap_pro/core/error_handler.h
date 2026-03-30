#pragma once
/**
 * @file core/error_handler.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */


#include <string>
#include <optional>
#include <variant>
#include <stdexcept>
#include <sstream>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include "automap_pro/core/logger.h"

namespace automap_pro {

// 错误码定义
enum class ErrorCode : uint32_t {
    SUCCESS = 0x0000,
    
    // Sensor Errors (0x1xxx)
    LIDAR_TIMEOUT = 0x1001,
    IMU_INVALID_DATA = 0x1002,
    GPS_HDOP_HIGH = 0x1003,
    SENSOR_NOT_READY = 0x1004,
    SENSOR_SYNC_FAILED = 0x1005,
    
    // Mapping Errors (0x2xxx)
    KEYFRAME_CREATE_FAILED = 0x2001,
    LOOP_CLOSURE_REJECTED = 0x2002,
    SUBMAP_OVERFLOW = 0x2003,
    FEATURE_EXTRACTION_FAILED = 0x2004,
    
    // Optimization Errors (0x3xxx)
    OPTIMIZATION_DIVERGED = 0x3001,
    COVARIANCE_FAILED = 0x3002,
    FACTOR_GRAPH_ERROR = 0x3003,
    SINGULAR_MATRIX = 0x3004,
    
    // IO Errors (0x4xxx)
    CONFIG_LOAD_FAILED = 0x4001,
    MAP_SAVE_FAILED = 0x4002,
    FILE_NOT_FOUND = 0x4003,
    FILE_WRITE_ERROR = 0x4004,
    
    // System Errors (0x5xxx)
    OUT_OF_MEMORY = 0x5001,
    TIMEOUT = 0x5002,
    UNKNOWN_ERROR = 0xFFFF
};

// 错误详情
struct Error {
    ErrorCode code;
    std::string message;
    std::string component;
    std::string context;
    
    Error() : code(ErrorCode::UNKNOWN_ERROR) {}
    
    Error(ErrorCode code, const std::string& msg)
        : code(code), message(msg) {}
    
    Error(ErrorCode code, const std::string& msg, const std::string& comp)
        : code(code), message(msg), component(comp) {}
    
    Error(ErrorCode code, const std::string& msg, const std::string& comp,
          const std::string& ctx)
        : code(code), message(msg), component(comp), context(ctx) {}
    
    std::string toString() const;
    std::string getCodeString() const;
    
    operator bool() const { return code == ErrorCode::SUCCESS; }
};

// Result类型
template<typename T>
class Result {
public:
    Result(T value) : data_(std::move(value)) {}
    Result(Error error) : data_(std::move(error)) {}
    
    bool isOk() const { return std::holds_alternative<T>(data_); }
    bool isError() const { return std::holds_alternative<Error>(data_); }
    
    T& unwrap() {
        if (isError()) {
            throw std::runtime_error(std::get<Error>(data_).toString());
        }
        return std::get<T>(data_);
    }
    
    const T& unwrap() const {
        if (isError()) {
            throw std::runtime_error(std::get<Error>(data_).toString());
        }
        return std::get<T>(data_);
    }
    
    T unwrapOr(T default_value) {
        if (isError()) return default_value;
        return std::move(std::get<T>(data_));
    }
    
    T unwrapOr(T default_value) const {
        if (isError()) return default_value;
        return std::get<T>(data_);
    }
    
    Error error() const {
        if (isError()) return std::get<Error>(data_);
        return Error(ErrorCode::SUCCESS, "Success");
    }
    
    std::string toString() const {
        if (isError()) {
            return std::get<Error>(data_).toString();
        }
        std::ostringstream oss;
        oss << "Success: " << std::get<T>(data_);
        return oss.str();
    }
    
    // 链式调用
    template<typename F>
    auto map(F&& func) -> Result<decltype(func(std::declval<T>()))> {
        if (isError()) return error();
        try {
            return func(std::get<T>(data_));
        } catch (...) {
            return Error(ErrorCode::UNKNOWN_ERROR, "Map function failed");
        }
    }
    
    template<typename F>
    auto andThen(F&& func) -> decltype(func(std::declval<T>())) {
        if (isError()) return error();
        try {
            return func(std::get<T>(data_));
        } catch (...) {
            return Error(ErrorCode::UNKNOWN_ERROR, "AndThen function failed");
        }
    }
    
    template<typename F>
    auto orElse(F&& func) -> Result<T> {
        if (isOk()) return *this;
        try {
            return func(error());
        } catch (...) {
            return error();
        }
    }
    
private:
    std::variant<T, Error> data_;
};

// void特化（data_ 为 Error 类型，非 variant，直接使用 data_）
template<>
class Result<void> {
public:
    Result() : data_(Error(ErrorCode::SUCCESS, "Success")) {}
    Result(Error error) : data_(std::move(error)) {}
    
    bool isOk() const { return data_.code == ErrorCode::SUCCESS; }
    bool isError() const { return !isOk(); }
    
    void unwrap() const {
        if (isError()) {
            throw std::runtime_error(data_.toString());
        }
    }
    
    Error error() const { return data_; }
    
    std::string toString() const {
        if (isError()) return data_.toString();
        return "Success";
    }
    
private:
    Error data_;
};

// 异常类层次
class AutomapException : public std::runtime_error {
public:
    AutomapException(ErrorCode code, const std::string& msg)
        : std::runtime_error(msg), error_(code, msg) {}
    
    ErrorCode code() const { return error_.code; }
    const Error& error() const { return error_; }
    
protected:
    Error error_;
};

class RuntimeException : public AutomapException {
public:
    using AutomapException::AutomapException;
};

class TimeoutException : public RuntimeException {
public:
    TimeoutException(const std::string& msg, double timeout_sec = 0.0)
        : RuntimeException(ErrorCode::TIMEOUT, msg), timeout_sec_(timeout_sec) {}
    
    double timeoutSec() const { return timeout_sec_; }
    
private:
    double timeout_sec_;
};

class DataInvalidException : public RuntimeException {
public:
    using RuntimeException::RuntimeException;
};

class ResourceException : public RuntimeException {
public:
    using RuntimeException::RuntimeException;
};

class SystemException : public AutomapException {
public:
    using AutomapException::AutomapException;
};

class IOException : public SystemException {
public:
    IOException(const std::string& path, const std::string& msg)
        : SystemException(ErrorCode::FILE_NOT_FOUND, msg), path_(path) {}
    
    std::string path() const { return path_; }
    
private:
    std::string path_;
};

class ConfigException : public SystemException {
public:
    using SystemException::SystemException;
};

class OptimizationException : public AutomapException {
public:
    using AutomapException::AutomapException;
};

class ConvergenceException : public OptimizationException {
public:
    ConvergenceException(int iterations, double residual)
        : OptimizationException(ErrorCode::OPTIMIZATION_DIVERGED, 
                              "Optimization failed to converge"),
          iterations_(iterations), residual_(residual) {}
    
    int iterations() const { return iterations_; }
    double residual() const { return residual_; }
    
private:
    int iterations_;
    double residual_;
};

// 辅助宏
#define RETURN_IF_ERROR(expr) \
    do { \
        auto _result = (expr); \
        if (_result.isError()) { \
            LOG_ERROR("Error in {}: {}", __func__, _result.error().toString()); \
            return _result.error(); \
        } \
    } while(0)

#define TRY(expr) \
    ({ \
        auto _result = (expr); \
        if (_result.isError()) { \
            LOG_ERROR("Error in {}: {}", __func__, _result.error().toString()); \
            return _result.error(); \
        } \
        _result.unwrap(); \
    })

#define TRY_OPTIONAL(expr) \
    ({ \
        auto _result = (expr); \
        if (!_result) { \
            return std::nullopt; \
        } \
        *_result; \
    })

// 错误处理工具类
class ErrorHandler {
public:
    static void logError(const Error& error);
    static void logException(const std::exception& e);
    
    static Result<void> safeExecute(std::function<Result<void>()> func);
    static Result<void> retry(std::function<Result<void>()> func, 
                             int max_attempts = 3,
                             int delay_ms = 100);
    
    static Error wrapException(const std::exception& e, 
                              const std::string& component,
                              ErrorCode default_code = ErrorCode::UNKNOWN_ERROR);
    
    static std::string generateTraceId();
    static std::string generateSessionId();
};

} // namespace automap_pro
