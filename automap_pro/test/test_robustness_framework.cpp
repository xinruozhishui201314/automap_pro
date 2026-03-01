#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include <fstream>

// 直接包含我们的头文件
#include "automap_pro/core/logger.h"
#include "automap_pro/core/error_handler.h"
#include "automap_pro/core/validator.h"
#include "automap_pro/core/performance_monitor.h"

using namespace automap_pro;

class RobustnessFrameworkTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 初始化日志系统
        Logger::instance().init();
        Logger::instance().setLevel(LogLevel::DEBUG);
        
        // 初始化性能监控
        PerformanceMonitor::instance().init(5.0, 50.0);
    }
    
    void TearDown() override {
        Logger::instance().shutdown();
        PerformanceMonitor::instance().shutdown();
    }
};

// ========== Logger Tests ==========

TEST_F(RobustnessFrameworkTest, LoggerInitialization) {
    Logger& logger = Logger::instance();
    EXPECT_EQ(logger.getLevel(), LogLevel::DEBUG);
    
    LOG_INFO("Logger test initialized");
}

TEST_F(RobustnessFrameworkTest, TraceIdPropagation) {
    Logger& logger = Logger::instance();
    
    std::string trace_id = "test_trace_123";
    logger.setTraceId(trace_id);
    EXPECT_EQ(logger.getTraceId(), trace_id);
    
    logger.setSessionId("test_session_456");
    EXPECT_EQ(logger.getSessionId(), "test_session_456");
    
    logger.setComponent("TestComponent");
    EXPECT_EQ(logger.getComponent(), "TestComponent");
    
    LOG_INFO("Test message with trace ID");
    
    logger.clearTraceId();
    EXPECT_EQ(logger.getTraceId(), "");
}

TEST_F(RobustnessFrameworkTest, ScopedTimer) {
    Logger& logger = Logger::instance();
    
    {
        LOG_TIMER("test_operation");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 计时器会在析构时自动记录
    LOG_INFO("Scoped timer test completed");
}

TEST_F(RobustnessFrameworkTest, LogContext) {
    LogContext ctx;
    ctx.trace_id = "test_trace";
    ctx.session_id = "test_session";
    ctx.component = "TestComponent";
    ctx.fields["key1"] = "value1";
    ctx.fields["key2"] = "value2";
    
    Logger::instance().infoWithContext(ctx, "Test with context");
}

// ========== ErrorHandler Tests ==========

TEST_F(RobustnessFrameworkTest, ErrorCreation) {
    Error err(ErrorCode::IMU_INVALID_DATA, "Test error", "TestComponent");
    
    EXPECT_FALSE(err);
    EXPECT_EQ(err.code, ErrorCode::IMU_INVALID_DATA);
    EXPECT_EQ(err.component, "TestComponent");
    EXPECT_FALSE(err.toString().empty());
    
    std::cout << "Error string: " << err.toString() << std::endl;
}

TEST_F(RobustnessFrameworkTest, ResultOk) {
    Result<int> result(42);
    
    EXPECT_TRUE(result.isOk());
    EXPECT_FALSE(result.isError());
    EXPECT_EQ(result.unwrap(), 42);
}

TEST_F(RobustnessFrameworkTest, ResultError) {
    Result<int> result(Error(ErrorCode::UNKNOWN_ERROR, "Test error"));
    
    EXPECT_FALSE(result.isOk());
    EXPECT_TRUE(result.isError());
    
    EXPECT_THROW(result.unwrap(), std::runtime_error);
    EXPECT_EQ(result.unwrapOr(0), 0);
}

TEST_F(RobustnessFrameworkTest, ResultVoid) {
    Result<void> result;
    EXPECT_TRUE(result.isOk());
    
    Result<void> result_err(Error(ErrorCode::UNKNOWN_ERROR, "Error"));
    EXPECT_TRUE(result_err.isError());
}

TEST_F(RobustnessFrameworkTest, SafeExecute) {
    auto func_ok = []() -> Result<void> {
        LOG_INFO("Executing safely");
        return Result<void>{};
    };
    
    auto result = ErrorHandler::safeExecute(func_ok);
    EXPECT_TRUE(result.isOk());
    
    auto func_fail = []() -> Result<void> {
        throw std::runtime_error("Test exception");
        return Result<void>{};
    };
    
    auto result_fail = ErrorHandler::safeExecute(func_fail);
    EXPECT_TRUE(result_fail.isError());
}

TEST_F(RobustnessFrameworkTest, Retry) {
    int attempt_count = 0;
    auto func = [&attempt_count]() -> Result<void> {
        attempt_count++;
        if (attempt_count < 3) {
            return Error(ErrorCode::TIMEOUT, "Simulated timeout");
        }
        return Result<void>{};
    };
    
    auto result = ErrorHandler::retry(func, 5, 10);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(attempt_count, 3);
}

TEST_F(RobustnessFrameworkTest, GenerateIds) {
    std::string trace_id = ErrorHandler::generateTraceId();
    EXPECT_FALSE(trace_id.empty());
    EXPECT_EQ(trace_id.length(), 16); // hex string
    
    std::string session_id = ErrorHandler::generateSessionId();
    EXPECT_FALSE(session_id.empty());
    EXPECT_TRUE(session_id.find("sess_") == 0);
    
    std::cout << "Trace ID: " << trace_id << std::endl;
    std::cout << "Session ID: " << session_id << std::endl;
}

// ========== Validator Tests ==========

TEST_F(RobustnessFrameworkTest, CheckNotNull) {
    int value = 42;
    auto result = Validator::checkNotNull(&value, "value");
    EXPECT_TRUE(result.isValid());
    
    auto result_null = Validator::checkNotNull<int>(nullptr, "null");
    EXPECT_FALSE(result_null.isValid());
    
    std::optional<int> opt = 10;
    auto result_opt = Validator::checkNotNull(opt, "optional");
    EXPECT_TRUE(result_opt.isValid());
    
    std::optional<int> empty_opt;
    auto result_empty_opt = Validator::checkNotNull(empty_opt, "empty_optional");
    EXPECT_FALSE(result_empty_opt.isValid());
}

TEST_F(RobustnessFrameworkTest, CheckRange) {
    auto result = Validator::checkRange(5.0, 0.0, 10.0, "test");
    EXPECT_TRUE(result.isValid());
    
    auto result_fail = Validator::checkRange(15.0, 0.0, 10.0, "test");
    EXPECT_FALSE(result_fail.isValid());
    
    auto result_int = Validator::checkRange(5, 0, 10, "int_test");
    EXPECT_TRUE(result_int.isValid());
}

TEST_F(RobustnessFrameworkTest, CheckTimestamp) {
    auto now = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    auto result = Validator::checkTimestamp(now);
    EXPECT_TRUE(result.isValid());
    
    auto result_fail = Validator::checkTimestamp(-1.0);
    EXPECT_FALSE(result_fail.isValid());
}

TEST_F(RobustnessFrameworkTest, CheckNotEmpty) {
    auto result = Validator::checkNotEmpty("test", "string");
    EXPECT_TRUE(result.isValid());
    
    auto result_fail = Validator::checkNotEmpty("", "empty_string");
    EXPECT_FALSE(result_fail.isValid());
}

TEST_F(RobustnessFrameworkTest, CheckFinite) {
    Eigen::Vector3d vec(1.0, 2.0, 3.0);
    auto result = Validator::checkFinite(vec, "vector");
    EXPECT_TRUE(result.isValid());
    
    Eigen::Vector3d vec_nan(NAN, 2.0, 3.0);
    auto result_nan = Validator::checkFinite(vec_nan, "nan_vector");
    EXPECT_FALSE(result_nan.isValid());
}

TEST_F(RobustnessFrameworkTest, ValidatorChain) {
    double value = 5.0;
    auto now = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    auto result = Validator::begin()
        .checkRange(value, 0.0, 10.0, "value")
        .checkTimestamp(now, "timestamp")
        .checkNotEmpty("test", "string")
        .build();
    
    EXPECT_TRUE(result.isValid());
}

TEST_F(RobustnessFrameworkTest, CheckGPS) {
    auto lat_result = Validator::checkLatitude(39.9, "latitude");
    EXPECT_TRUE(lat_result.isValid());
    
    auto lat_fail = Validator::checkLatitude(100.0, "latitude");
    EXPECT_FALSE(lat_fail.isValid());
    
    auto lon_result = Validator::checkLongitude(116.4, "longitude");
    EXPECT_TRUE(lon_result.isValid());
    
    auto lon_fail = Validator::checkLongitude(200.0, "longitude");
    EXPECT_FALSE(lon_fail.isValid());
    
    auto hdop_result = Validator::checkHDOP(1.5, "HDOP");
    EXPECT_TRUE(hdop_result.isValid());
}

// ========== PerformanceMonitor Tests ==========

TEST_F(RobustnessFrameworkTest, PerformanceMonitor) {
    PerformanceMonitor& monitor = PerformanceMonitor::instance();
    
    {
        PERF_TIMER("test_operation");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    auto stats = monitor.getStats("test_operation");
    EXPECT_GT(stats.count, 0);
    EXPECT_GT(stats.total_time_ms, 0.0);
    EXPECT_GT(stats.min_time_ms, 0.0);
    EXPECT_LT(stats.min_time_ms, stats.max_time_ms);
    
    std::cout << "Operation count: " << stats.count << std::endl;
    std::cout << "Total time: " << stats.total_time_ms << " ms" << std::endl;
    std::cout << "Average time: " << stats.getAverageTime() << " ms" << std::endl;
    std::cout << "Min time: " << stats.min_time_ms << " ms" << std::endl;
    std::cout << "Max time: " << stats.max_time_ms << " ms" << std::endl;
}

TEST_F(RobustnessFrameworkTest, PerformanceMonitorSlowOperation) {
    PerformanceMonitor& monitor = PerformanceMonitor::instance();
    
    // 降低慢操作阈值以触发告警
    monitor.setSlowThreshold(5.0);
    
    {
        PERF_TIMER("slow_operation");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 检查日志中是否有慢操作告警
    monitor.setSlowThreshold(50.0); // 恢复默认值
}

TEST_F(RobustnessFrameworkTest, PerformanceMonitorStats) {
    PerformanceMonitor& monitor = PerformanceMonitor::instance();
    
    monitor.resetStats();
    
    // 记录多次操作
    for (int i = 0; i < 10; ++i) {
        {
            PERF_TIMER("stats_test");
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    auto stats = monitor.getStats("stats_test");
    EXPECT_EQ(stats.count, 10);
    EXPECT_GT(stats.getAverageTime(), 0.0);
    
    // 测试百分位数
    if (!stats.recent_samples.empty()) {
        double p50 = stats.getPercentile(0.5);
        double p95 = stats.getPercentile(0.95);
        std::cout << "P50: " << p50 << " ms, P95: " << p95 << " ms" << std::endl;
    }
}

// ========== Integration Tests ==========

TEST_F(RobustnessFrameworkTest, EndToEndLogging) {
    // 完整的日志流程测试
    Logger::instance().setTraceId("e2e_trace_001");
    Logger::instance().setSessionId("e2e_session_001");
    Logger::instance().setComponent("IntegrationTest");
    
    {
        LOG_TIMER("end_to_end_operation");
        
        // 记录一些日志
        LOG_INFO("Starting end-to-end test");
        LOG_DEBUG("Debug information");
        LOG_WARN("Warning message");
        
        // 使用Result
        Result<int> calc_result = [](int a, int b) -> Result<int> {
            auto check1 = Validator::checkRange(a, 0, 100, "a");
            auto check2 = Validator::checkRange(b, 0, 100, "b");
            
            if (check1.isInvalid()) return Error(ErrorCode::IMU_INVALID_DATA, check1.error());
            if (check2.isInvalid()) return Error(ErrorCode::IMU_INVALID_DATA, check2.error());
            
            return a + b;
        }(10, 20);
        
        if (calc_result.isOk()) {
            LOG_INFO("Calculation result: {}", calc_result.unwrap());
        } else {
            LOG_ERROR("Calculation failed: {}", calc_result.error().toString());
        }
    }
    
    // 报告性能统计
    PerformanceMonitor::instance().reportStats();
}

TEST_F(RobustnessFrameworkTest, ErrorRecoveryScenario) {
    // 模拟错误恢复场景
    LOG_INFO("Simulating error recovery scenario");
    
    int attempt = 0;
    auto result = ErrorHandler::retry([&attempt]() -> Result<void> {
        attempt++;
        LOG_INFO("Attempt {}", attempt);
        
        if (attempt == 1) {
            return Error(ErrorCode::SENSOR_NOT_READY, "Sensor not ready");
        } else if (attempt == 2) {
            return Error(ErrorCode::TIMEOUT, "Operation timeout");
        }
        
        return Result<void>{};
    }, 5, 10);
    
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(attempt, 3);
    LOG_INFO("Operation succeeded after {} attempts", attempt);
}

// ========== Performance Tests ==========

TEST_F(RobustnessFrameworkTest, LoggingOverhead) {
    const int iterations = 1000;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; ++i) {
        LOG_DEBUG("Performance test message {}", i);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end - start).count();
    
    double avg_ms = duration / iterations;
    
    std::cout << "Logging overhead: " << duration << " ms for " 
              << iterations << " logs" << std::endl;
    std::cout << "Average: " << avg_ms << " ms per log" << std::endl;
    
    // 性能应该在合理范围内（每次日志 < 0.1ms）
    EXPECT_LT(avg_ms, 0.1);
}

TEST_F(RobustnessFrameworkTest, ValidationErrorPerformance) {
    const int iterations = 10000;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int i = 0; i < iterations; ++i) {
        auto result = Validator::checkRange(i % 100, 0.0, 100.0, "test");
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(end - start).count();
    
    double avg_us = duration * 1000.0 / iterations;
    
    std::cout << "Validation overhead: " << duration << " ms for " 
              << iterations << " validations" << std::endl;
    std::cout << "Average: " << avg_us << " us per validation" << std::endl;
    
    // 验证应该非常快（每次验证 < 1us）
    EXPECT_LT(avg_us, 1.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "========================================" << std::endl;
    std::cout << "AutoMap Pro Robustness Framework Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return RUN_ALL_TESTS();
}
