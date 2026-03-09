# AutoMap Pro 健壮性框架使用指南

## 概述

本框架为AutoMap Pro提供了完整的代码健壮性和可观测性支持，包括：

- **结构化日志系统**：JSON格式、Trace追踪、性能监控
- **错误处理机制**：类型安全的Result<T>、统一错误码
- **参数验证框架**：链式验证、丰富的验证规则
- **性能监控**：自动计时、慢操作告警

## 快速开始

### 1. 基础日志

```cpp
#include "automap_pro/core/logger.h"

// 初始化日志系统
Logger::instance().init("/path/to/logging.yaml");

// 记录不同级别的日志
LOG_TRACE("Detailed debug information");
LOG_DEBUG("Debug message");
LOG_INFO("Information message");
LOG_WARN("Warning message");
LOG_ERROR("Error message");
LOG_FATAL("Fatal error message");

// 设置Trace ID用于追踪
Logger::instance().setTraceId("request_123");

// 设置组件名称
Logger::instance().setComponent("MyComponent");
```

### 2. 性能监控

```cpp
#include "automap_pro/core/performance_monitor.h"

// 初始化性能监控
PerformanceMonitor::instance().init(10.0, 100.0); // 10s报告间隔，100ms慢操作阈值

// 使用RAII计时器
{
    PERF_TIMER("my_operation");
    // ... 执行操作 ...
} // 计时器在析构时自动记录

// 手动报告统计
PerformanceMonitor::instance().reportStats();
```

### 3. 错误处理

```cpp
#include "automap_pro/core/error_handler.h"

// 使用Result<T>返回值
Result<int> calculate(int a, int b) {
    if (a < 0 || b < 0) {
        return Error(ErrorCode::IMU_INVALID_DATA, "Negative values not allowed");
    }
    return a + b;
}

// 调用并处理结果
auto result = calculate(10, 20);
if (result.isOk()) {
    LOG_INFO("Result: {}", result.unwrap());
} else {
    LOG_ERROR("Error: {}", result.error().toString());
}

// 使用TRY宏自动处理错误
int value = TRY(calculate(10, 20));

// 重试机制
auto result = ErrorHandler::retry(
    []() -> Result<void> {
        // 可能失败的操作
        return doSomething();
    },
    3,    // 最大重试次数
    100   // 重试间隔ms
);
```

### 4. 参数验证

```cpp
#include "automap_pro/core/validator.h"

// 单个验证
auto result = Validator::checkRange(value, 0.0, 100.0, "value");
if (result.isInvalid()) {
    LOG_ERROR("Validation failed: {}", result.error());
}

// 链式验证
auto validation = Validator::begin()
    .checkNotNull(ptr, "pointer")
    .checkRange(val, 0.0, 10.0, "value")
    .checkTimestamp(ts, "timestamp")
    .build();

if (validation.isInvalid()) {
    return Error(ErrorCode::IMU_INVALID_DATA, validation.error());
}

// 使用便捷宏
CHECK_NOT_NULL(ptr, "pointer");
CHECK_RANGE(val, 0.0, 10.0, "value");
CHECK_TIMESTAMP(ts, "timestamp");
```

## 高级用法

### Trace追踪

```cpp
// 设置Trace ID
Logger::instance().setTraceId("operation_123");

// Trace ID会自动传播到所有日志
LOG_INFO("Step 1");
LOG_INFO("Step 2");
LOG_INFO("Step 3");

// 清除Trace ID
Logger::instance().clearTraceId();
```

### 性能分析

```cpp
// 记录多次操作
for (int i = 0; i < 100; ++i) {
    PERF_TIMER("repeated_operation");
    doSomething();
}

// 获取统计信息
auto stats = PerformanceMonitor::instance().getStats("repeated_operation");
LOG_INFO("Count: {}, Average: {}ms, P95: {}ms",
         stats.count,
         stats.getAverageTime(),
         stats.getPercentile(0.95));
```

### 自定义上下文

```cpp
// 添加线程局部上下文
Logger::instance().addContextField("request_id", "req_123");
Logger::instance().addContextField("user_id", "user_456");

// 这些字段会自动添加到所有日志
LOG_INFO("Processing request");

// 清除上下文
Logger::instance().clearContextFields();
```

### 错误链式处理

```cpp
Result<std::string> processStep1() {
    return "step1_result";
}

Result<std::string> processStep2(const std::string& input) {
    if (input.empty()) {
        return Error(ErrorCode::UNKNOWN_ERROR, "Empty input");
    }
    return input + "_processed";
}

auto result = processStep1()
    .map([](const std::string& s) {
        return s + "_modified";
    })
    .andThen([](const std::string& s) {
        return processStep2(s);
    });

if (result.isOk()) {
    LOG_INFO("Final result: {}", result.unwrap());
}
```

## 配置文件

### logging.yaml

```yaml
# 日志级别
log_level: INFO

# 日志文件路径
log_file: "/tmp/automap_pro/automap_pro.log"

# 日志滚动
rotation:
  max_size_mb: 100
  max_files: 10
  max_age_days: 30

# 采样率（用于高频日志）
sampling_rates:
  "imu_callback": 0.01        # 1% 采样
  "lidar_callback": 0.05      # 5% 采样

# 性能监控
performance:
  enable: true
  report_interval_sec: 10.0
  log_slow_threshold_ms: 100.0
```

## 日志分析

### 使用Python脚本

```bash
# 查看摘要
python3 scripts/analyze_logs.py /tmp/automap_pro/automap_pro.log --summary

# 查看所有错误
python3 scripts/analyze_logs.py /tmp/automap_pro/automap_pro.log --errors

# 搜索特定模式
python3 scripts/analyze_logs.py /tmp/automap_pro/automap_pro.log --search "ERROR"

# 查看性能统计
python3 scripts/analyze_logs.py /tmp/automap_pro/automap_pro.log --performance

# Trace追踪
python3 scripts/analyze_logs.py /tmp/automap_pro/automap_pro.log --trace <trace_id>
```

### 使用jq过滤

```bash
# 查看所有ERROR级别日志
cat /tmp/automap_pro/automap_pro.log | jq 'select(.level == "ERROR")'

# 查看特定组件的日志
cat /tmp/automap_pro/automap_pro.log | jq 'select(.context.component == "SensorManager")'

# 查看性能数据
cat /tmp/automap_pro/automap_pro.log | jq 'select(.context.duration_ms)'

# 按时间过滤
cat /tmp/automap_pro/automap_pro.log | jq 'select(.timestamp | startswith("2026-02-28"))'
```

## 最佳实践

### 1. 错误处理

- ✅ 使用`Result<T>`而不是异常处理可恢复的错误
- ✅ 使用`TRY()`宏简化错误传播
- ✅ 为错误提供清晰的上下文信息
- ❌ 不要忽略`Result<T>`的返回值

### 2. 日志记录

- ✅ 使用结构化日志，添加上下文信息
- ✅ 生产环境使用INFO或WARN级别
- ✅ 对高频操作使用采样
- ❌ 避免在热路径中记录DEBUG日志

### 3. 参数验证

- ✅ 在函数入口处验证参数
- ✅ 使用链式验证提高可读性
- ✅ 提供有意义的错误消息
- ❌ 不要信任外部输入

### 4. 性能监控

- ✅ 对关键操作使用性能计时器
- ✅ 定期查看性能统计
- ✅ 设置合理的慢操作阈值
- ❌ 不要监控过于细粒度的操作

## 故障排查

### 日志文件未生成

```bash
# 检查目录权限
mkdir -p /tmp/automap_pro
chmod 777 /tmp/automap_pro

# 检查配置文件路径
Logger::instance().init("/absolute/path/to/logging.yaml");
```

### 性能开销过大

```yaml
# 降低日志级别
log_level: WARN

# 增加采样率
sampling_rates:
  "high_freq_operation": 0.001  # 0.1% 采样
```

### Trace ID不连续

这是正常现象，因为：
- 异步操作可能在不同的线程执行
- 使用`session_id`来关联相关操作

## 性能指标

| 操作 | 平均耗时 | 说明 |
|------|---------|------|
| LOG_DEBUG | < 0.01ms | 快速日志记录 |
| LOG_INFO | < 0.01ms | 正常日志记录 |
| 参数验证 | < 0.001ms | 简单范围检查 |
| Result<T>构造 | < 0.001ms | 类型安全返回值 |

## 编译和测试

```bash
# 编译
colcon build --packages-select automap_pro

# 运行单元测试
colcon test --packages-select automap_pro

# 查看测试结果
colcon test-result --all
```

## 更多信息

- 查看头文件获取完整的API文档
- 参考测试用例了解使用示例
- 阅读日志文件了解运行时行为
