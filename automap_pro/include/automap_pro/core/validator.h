#pragma once
/**
 * @file core/validator.h
 * @brief 核心：指标、协议、错误、资源、ONNX 等横切能力。
 */


#include <string>
#include <optional>
#include <vector>
#include <sstream>
#include <fstream>
#include <functional>
#include <filesystem>
#include <Eigen/Dense>
#include "automap_pro/core/error_handler.h"
#include "automap_pro/core/logger.h"

namespace automap_pro {

// 验证结果
class ValidationResult {
public:
    ValidationResult() : valid_(true) {}
    ValidationResult(const std::string& error) 
        : valid_(false), error_(error) {}
    
    bool isValid() const { return valid_; }
    bool isInvalid() const { return !valid_; }
    const std::string& error() const { return error_; }
    
    operator bool() const { return valid_; }
    
    ValidationResult& operator&&(const ValidationResult& other) {
        if (!other.valid_) {
            valid_ = false;
            if (!error_.empty()) error_ += "; ";
            error_ += other.error_;
        }
        return *this;
    }
    
    Result<void> asResult() const {
        if (valid_) return Result<void>{};
        return Error(ErrorCode::IMU_INVALID_DATA, error_, "Validator");
    }
    
private:
    bool valid_;
    std::string error_;
};

// 参数验证器
class Validator {
public:
    // 空指针/optional检查
    template<typename T>
    static ValidationResult checkNotNull(const T* ptr, const std::string& name) {
        if (ptr == nullptr) {
            return ValidationResult(name + " is null");
        }
        return ValidationResult{};
    }
    
    template<typename T>
    static ValidationResult checkNotNull(const std::optional<T>& opt, 
                                         const std::string& name) {
        if (!opt.has_value()) {
            return ValidationResult(name + " is not set");
        }
        return ValidationResult{};
    }
    
    template<typename T>
    static ValidationResult checkNotNull(const std::shared_ptr<T>& ptr,
                                         const std::string& name) {
        if (ptr == nullptr) {
            return ValidationResult(name + " is null");
        }
        return ValidationResult{};
    }
    
    template<typename T>
    static ValidationResult checkNotNull(const std::unique_ptr<T>& ptr,
                                         const std::string& name) {
        if (ptr == nullptr) {
            return ValidationResult(name + " is null");
        }
        return ValidationResult{};
    }
    
    // 范围检查
    static ValidationResult checkRange(double value, double min, double max,
                                      const std::string& name) {
        if (value < min || value > max) {
            std::ostringstream oss;
            oss << name << "=" << value << " is out of range [" 
                << min << ", " << max << "]";
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    static ValidationResult checkRange(int value, int min, int max,
                                      const std::string& name) {
        return checkRange(static_cast<double>(value), 
                        static_cast<double>(min),
                        static_cast<double>(max), name);
    }
    
    static ValidationResult checkRange(size_t value, size_t min, size_t max,
                                      const std::string& name) {
        if (value < min || value > max) {
            std::ostringstream oss;
            oss << name << "=" << value << " is out of range [" 
                << min << ", " << max << "]";
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    static ValidationResult checkPositive(double value, const std::string& name) {
        return checkRange(value, 0.0, std::numeric_limits<double>::max(), name);
    }
    
    static ValidationResult checkNonNegative(double value, const std::string& name) {
        if (value < 0.0) {
            std::ostringstream oss;
            oss << name << "=" << value << " is negative";
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    // 时间戳验证
    static ValidationResult checkTimestamp(double timestamp, 
                                           const std::string& name = "timestamp") {
        // 检查是否在合理范围内（1970-2070年）
        double min_time = 0.0;
        double max_time = 3000000000.0;    // ~2070 (Unix time)
        
        return checkRange(timestamp, min_time, max_time, name);
    }
    
    static ValidationResult checkMonotonic(double current, double previous,
                                           const std::string& name = "timestamp") {
        if (current <= previous) {
            std::ostringstream oss;
            oss << name << " is not monotonic: current=" << current 
                << " <= previous=" << previous;
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    // 频率验证
    static ValidationResult checkFrequency(double freq, const std::string& name = "frequency") {
        return checkRange(freq, 0.1, 1000.0, name); // 0.1Hz - 1000Hz
    }
    
    // 文件存在性检查
    static ValidationResult checkFileExists(const std::string& path,
                                            const std::string& name = "file") {
        std::ifstream file(path);
        if (!file.good()) {
            std::ostringstream oss;
            oss << name << " does not exist or is not readable: " << path;
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    static ValidationResult checkDirectoryExists(const std::string& path,
                                                const std::string& name = "directory") {
        std::filesystem::path dir(path);
        if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) {
            std::ostringstream oss;
            oss << name << " does not exist or is not a directory: " << path;
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    // 字符串非空检查
    static ValidationResult checkNotEmpty(const std::string& str,
                                         const std::string& name = "string") {
        if (str.empty()) {
            return ValidationResult(name + " is empty");
        }
        return ValidationResult{};
    }
    
    // 向量/矩阵有效性检查
    static ValidationResult checkFinite(const Eigen::VectorXd& vec,
                                      const std::string& name = "vector") {
        if (!vec.allFinite()) {
            return ValidationResult(name + " contains NaN or Inf values");
        }
        return ValidationResult{};
    }
    
    static ValidationResult checkFinite(const Eigen::MatrixXd& mat,
                                      const std::string& name = "matrix") {
        if (!mat.allFinite()) {
            return ValidationResult(name + " contains NaN or Inf values");
        }
        return ValidationResult{};
    }
    
    static ValidationResult checkFinite(const Eigen::Vector3d& vec,
                                      const std::string& name = "vector") {
        if (!vec.allFinite()) {
            return ValidationResult(name + " contains NaN or Inf values");
        }
        return ValidationResult{};
    }
    
    static ValidationResult checkNorm(const Eigen::VectorXd& vec,
                                    double min_norm, double max_norm,
                                    const std::string& name = "vector") {
        double norm = vec.norm();
        return checkRange(norm, min_norm, max_norm, name + ".norm()");
    }
    
    static ValidationResult checkNorm(const Eigen::Vector3d& vec,
                                    double min_norm, double max_norm,
                                    const std::string& name = "vector") {
        double norm = vec.norm();
        return checkRange(norm, min_norm, max_norm, name + ".norm()");
    }
    
    // GPS相关验证
    static ValidationResult checkLatitude(double lat, const std::string& name = "latitude") {
        return checkRange(lat, -90.0, 90.0, name);
    }
    
    static ValidationResult checkLongitude(double lon, const std::string& name = "longitude") {
        return checkRange(lon, -180.0, 180.0, name);
    }
    
    static ValidationResult checkHDOP(double hdop, const std::string& name = "HDOP") {
        return checkRange(hdop, 0.0, 99.9, name);
    }
    
    // 点云数据验证
    static ValidationResult checkPointCloudSize(size_t size, size_t min_size,
                                             const std::string& name = "pointcloud") {
        if (size < min_size) {
            std::ostringstream oss;
            oss << name << " size=" << size << " is below minimum " << min_size;
            return ValidationResult(oss.str());
        }
        return ValidationResult{};
    }
    
    // 自定义验证
    static ValidationResult custom(const std::function<bool()>& predicate,
                                  const std::string& error_msg) {
        if (!predicate()) {
            return ValidationResult(error_msg);
        }
        return ValidationResult{};
    }
    
    // 链式验证
    class ValidatorChain {
    public:
        ValidatorChain() : result_(ValidationResult{}) {}
        
        template<typename T>
        ValidatorChain& checkNotNull(const T* ptr, const std::string& name) {
            result_ = result_ && Validator::checkNotNull(ptr, name);
            return *this;
        }
        
        ValidatorChain& checkRange(double value, double min, double max,
                                   const std::string& name) {
            result_ = result_ && Validator::checkRange(value, min, max, name);
            return *this;
        }
        
        ValidatorChain& checkTimestamp(double ts, const std::string& name = "timestamp") {
            result_ = result_ && Validator::checkTimestamp(ts, name);
            return *this;
        }
        
        ValidatorChain& checkFileExists(const std::string& path,
                                        const std::string& name = "file") {
            result_ = result_ && Validator::checkFileExists(path, name);
            return *this;
        }
        
        ValidatorChain& checkNotEmpty(const std::string& str,
                                    const std::string& name = "string") {
            result_ = result_ && Validator::checkNotEmpty(str, name);
            return *this;
        }
        
        template<typename Func>
        ValidatorChain& custom(Func&& predicate, const std::string& error_msg) {
            result_ = result_ && Validator::custom(predicate, error_msg);
            return *this;
        }
        
        ValidationResult build() const { return result_; }
        Result<void> asResult() const { return result_.asResult(); }
        
    private:
        ValidationResult result_;
    };
    
    static ValidatorChain begin() { return ValidatorChain{}; }
};

// 便捷宏
#define CHECK_NOT_NULL(ptr, name) \
    do { \
        auto _v = Validator::checkNotNull(ptr, name); \
        if (_v.isInvalid()) { \
            LOG_ERROR("Validation failed: {}", _v.error()); \
            return Error(ErrorCode::SENSOR_NOT_READY, _v.error()); \
        } \
    } while(0)

#define CHECK_RANGE(val, min, max, name) \
    do { \
        auto _v = Validator::checkRange(val, min, max, name); \
        if (_v.isInvalid()) { \
            LOG_ERROR("Validation failed: {}", _v.error()); \
            return Error(ErrorCode::IMU_INVALID_DATA, _v.error()); \
        } \
    } while(0)

#define CHECK_TIMESTAMP(ts, name) \
    do { \
        auto _v = Validator::checkTimestamp(ts, name); \
        if (_v.isInvalid()) { \
            LOG_ERROR("Validation failed: {}", _v.error()); \
            return Error(ErrorCode::IMU_INVALID_DATA, _v.error()); \
        } \
    } while(0)

#define CHECK_FILE_EXISTS(path, name) \
    do { \
        auto _v = Validator::checkFileExists(path, name); \
        if (_v.isInvalid()) { \
            LOG_ERROR("Validation failed: {}", _v.error()); \
            return Error(ErrorCode::CONFIG_LOAD_FAILED, _v.error()); \
        } \
    } while(0)

} // namespace automap_pro
