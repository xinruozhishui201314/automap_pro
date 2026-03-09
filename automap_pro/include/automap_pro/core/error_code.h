#pragma once
/**
 * AutoMap-Pro 增强型故障码系统
 *
 * 设计目标：
 *   - 细粒度错误码分类（按模块/组件/严重程度）
 *   - 支持错误链追踪（Error Chain）
 *   - 内置恢复建议
 *   - 结构化错误上下文
 *   - 兼容 ROS2 日志和监控系统
 */

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <sstream>
#include <chrono>
#include <memory>
#include <optional>

namespace automap_pro {

// ─────────────────────────────────────────────────────────────────────────────
// 错误严重等级
// ─────────────────────────────────────────────────────────────────────────────
enum class ErrorSeverity : uint8_t {
    INFO     = 0,    // 信息性提示，不影响功能
    WARNING  = 1,    // 警告，功能可能降级
    ERROR    = 2,    // 错误，功能失败但系统可恢复
    CRITICAL = 3,    // 严重错误，需要立即处理
    FATAL    = 4     // 致命错误，系统无法继续运行
};

// ─────────────────────────────────────────────────────────────────────────────
// 错误组件/模块标识
// ─────────────────────────────────────────────────────────────────────────────
enum class ErrorComponent : uint16_t {
    // 核心组件 (0x00xx)
    CORE_SYSTEM      = 0x0000,
    CORE_CONFIG      = 0x0001,
    CORE_LOGGER      = 0x0002,
    CORE_VALIDATOR   = 0x0003,

    // 传感器组件 (0x01xx)
    SENSOR_LIDAR     = 0x0100,
    SENSOR_IMU       = 0x0101,
    SENSOR_GPS       = 0x0102,
    SENSOR_CAMERA    = 0x0103,
    SENSOR_SYNC      = 0x0104,
    SENSOR_MANAGER   = 0x0110,

    // 前端组件 (0x02xx)
    FRONTEND_LIVO    = 0x0200,
    FRONTEND_KEYFRAME = 0x0201,
    FRONTEND_GPS     = 0x0202,
    FRONTEND_BRIDGE  = 0x0203,

    // 子图组件 (0x03xx)
    SUBMAP_MANAGER   = 0x0300,
    SUBMAP_SESSION   = 0x0301,
    SUBMAP_ARCHIVE   = 0x0302,

    // 回环检测组件 (0x04xx)
    LOOP_DETECTOR    = 0x0400,
    LOOP_DESCRIPTOR  = 0x0401,
    LOOP_MATCHER     = 0x0402,
    LOOP_ICP         = 0x0403,

    // 后端优化组件 (0x05xx)
    BACKEND_ISAM2    = 0x0500,
    BACKEND_HBA      = 0x0501,
    BACKEND_POSEGRAPH = 0x0502,
    BACKEND_GPS      = 0x0503,

    // 地图组件 (0x06xx)
    MAP_GLOBAL       = 0x0600,
    MAP_BUILDER      = 0x0601,
    MAP_EXPORTER     = 0x0602,
    MAP_FILTER       = 0x0603,

    // IO组件 (0x07xx)
    IO_FILE          = 0x0700,
    IO_PCD           = 0x0701,
    IO_PLY           = 0x0702,
    IO_DATABASE      = 0x0703,

    // 系统组件 (0x08xx)
    SYSTEM_MEMORY    = 0x0800,
    SYSTEM_THREAD    = 0x0801,
    SYSTEM_TIMEOUT   = 0x0802,
    SYSTEM_RESOURCE  = 0x0803,

    // 通信组件 (0x09xx)
    COMM_ROS         = 0x0900,
    COMM_SERVICE     = 0x0901,
    COMM_ACTION      = 0x0902,
    COMM_TF          = 0x0903,

    // 外部依赖 (0x0Axx)
    EXT_GTSAM        = 0x0A00,
    EXT_PCL          = 0x0A01,
    EXT_OPENCV       = 0x0A02,
    EXT_TORCH        = 0x0A03,
    EXT_TEASER       = 0x0A04,
    EXT_HBA          = 0x0A05,

    UNKNOWN          = 0xFFFF
};

// ─────────────────────────────────────────────────────────────────────────────
// 错误码定义（32位结构）
// Bit 31-28: 严重程度 (4 bits)
// Bit 27-20: 组件标识 (8 bits)
// Bit 19-0:  具体错误 (20 bits)
// ─────────────────────────────────────────────────────────────────────────────
struct ErrorCodeEx {
    uint32_t code;

    constexpr ErrorCodeEx() : code(0) {}
    constexpr ErrorCodeEx(uint32_t c) : code(c) {}
    constexpr ErrorCodeEx(ErrorSeverity sev, ErrorComponent comp, uint32_t specific)
        : code((static_cast<uint32_t>(sev) << 28) |
               (static_cast<uint32_t>(comp) << 20) |
               (specific & 0xFFFFF)) {}

    constexpr ErrorSeverity severity() const {
        return static_cast<ErrorSeverity>((code >> 28) & 0xF);
    }

    constexpr ErrorComponent component() const {
        return static_cast<ErrorComponent>((code >> 20) & 0xFF);
    }

    constexpr uint32_t specific() const {
        return code & 0xFFFFF;
    }

    constexpr operator uint32_t() const { return code; }
    constexpr bool operator==(ErrorCodeEx other) const { return code == other.code; }
    constexpr bool operator!=(ErrorCodeEx other) const { return code != other.code; }
};

// ─────────────────────────────────────────────────────────────────────────────
// 预定义错误码（按组件分类）
// ─────────────────────────────────────────────────────────────────────────────
namespace errors {

// 传感器错误 (SENSOR_xxx, 0x01xx)
constexpr ErrorCodeEx LIDAR_TIMEOUT           {ErrorSeverity::ERROR,   ErrorComponent::SENSOR_LIDAR,     0x001};
constexpr ErrorCodeEx LIDAR_DATA_CORRUPT      {ErrorSeverity::WARNING, ErrorComponent::SENSOR_LIDAR,     0x002};
constexpr ErrorCodeEx LIDAR_POINT_CLOUD_EMPTY {ErrorSeverity::WARNING, ErrorComponent::SENSOR_LIDAR,     0x003};
constexpr ErrorCodeEx LIDAR_UNDISTORTION_FAIL {ErrorSeverity::WARNING, ErrorComponent::SENSOR_LIDAR,     0x004};

constexpr ErrorCodeEx IMU_DATA_INVALID        {ErrorSeverity::ERROR,   ErrorComponent::SENSOR_IMU,       0x001};
constexpr ErrorCodeEx IMU_BIAS_OUT_OF_RANGE   {ErrorSeverity::WARNING, ErrorComponent::SENSOR_IMU,       0x002};
constexpr ErrorCodeEx IMU_SYNC_ERROR          {ErrorSeverity::ERROR,   ErrorComponent::SENSOR_IMU,       0x003};

constexpr ErrorCodeEx GPS_HDOP_HIGH           {ErrorSeverity::WARNING, ErrorComponent::SENSOR_GPS,       0x001};
constexpr ErrorCodeEx GPS_NO_FIX              {ErrorSeverity::WARNING, ErrorComponent::SENSOR_GPS,       0x002};
constexpr ErrorCodeEx GPS_DATA_STALE          {ErrorSeverity::WARNING, ErrorComponent::SENSOR_GPS,       0x003};
constexpr ErrorCodeEx GPS_COORDINATE_INVALID  {ErrorSeverity::ERROR,   ErrorComponent::SENSOR_GPS,       0x004};

constexpr ErrorCodeEx SENSOR_SYNC_FAILED      {ErrorSeverity::ERROR,   ErrorComponent::SENSOR_SYNC,      0x001};
constexpr ErrorCodeEx SENSOR_NOT_READY        {ErrorSeverity::WARNING, ErrorComponent::SENSOR_MANAGER,   0x001};
constexpr ErrorCodeEx SENSOR_CALIBRATION_MISSING {ErrorSeverity::ERROR, ErrorComponent::SENSOR_MANAGER,  0x002};

// 前端错误 (FRONTEND_xxx, 0x02xx)
constexpr ErrorCodeEx LIVO_ODOMETRY_FAILED    {ErrorSeverity::ERROR,   ErrorComponent::FRONTEND_LIVO,    0x001};
constexpr ErrorCodeEx LIVO_INITIALIZATION_FAIL{ErrorSeverity::CRITICAL,ErrorComponent::FRONTEND_LIVO,    0x002};
constexpr ErrorCodeEx LIVO_DEGENERACY_DETECTED{ErrorSeverity::WARNING, ErrorComponent::FRONTEND_LIVO,    0x003};
constexpr ErrorCodeEx LIVO_FEATURE_EXTRACTION_FAIL {ErrorSeverity::WARNING, ErrorComponent::FRONTEND_LIVO, 0x004};

constexpr ErrorCodeEx KEYFRAME_CREATE_FAILED  {ErrorSeverity::ERROR,   ErrorComponent::FRONTEND_KEYFRAME, 0x001};
constexpr ErrorCodeEx KEYFRAME_SELECTION_FAIL {ErrorSeverity::WARNING, ErrorComponent::FRONTEND_KEYFRAME, 0x002};
constexpr ErrorCodeEx KEYFRAME_QUALITY_LOW    {ErrorSeverity::WARNING, ErrorComponent::FRONTEND_KEYFRAME, 0x003};

constexpr ErrorCodeEx GPS_ALIGN_INSUFFICIENT  {ErrorSeverity::WARNING, ErrorComponent::FRONTEND_GPS,     0x001};
constexpr ErrorCodeEx GPS_ALIGN_FAILED        {ErrorSeverity::ERROR,   ErrorComponent::FRONTEND_GPS,     0x002};
constexpr ErrorCodeEx GPS_ALIGN_RMSE_HIGH     {ErrorSeverity::WARNING, ErrorComponent::FRONTEND_GPS,     0x003};

// 子图错误 (SUBMAP_xxx, 0x03xx)
constexpr ErrorCodeEx SUBMAP_OVERFLOW         {ErrorSeverity::WARNING, ErrorComponent::SUBMAP_MANAGER,   0x001};
constexpr ErrorCodeEx SUBMAP_EMPTY            {ErrorSeverity::WARNING, ErrorComponent::SUBMAP_MANAGER,   0x002};
constexpr ErrorCodeEx SUBMAP_MERGE_FAILED     {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_MANAGER,   0x003};
constexpr ErrorCodeEx SUBMAP_STATE_INVALID    {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_MANAGER,   0x004};

constexpr ErrorCodeEx SESSION_LOAD_FAILED     {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_SESSION,   0x001};
constexpr ErrorCodeEx SESSION_SAVE_FAILED     {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_SESSION,   0x002};
constexpr ErrorCodeEx SESSION_ID_CONFLICT     {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_SESSION,   0x003};

constexpr ErrorCodeEx ARCHIVE_WRITE_FAILED    {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_ARCHIVE,   0x001};
constexpr ErrorCodeEx ARCHIVE_READ_FAILED     {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_ARCHIVE,   0x002};
constexpr ErrorCodeEx ARCHIVE_CORRUPTED       {ErrorSeverity::ERROR,   ErrorComponent::SUBMAP_ARCHIVE,   0x003};

// 回环检测错误 (LOOP_xxx, 0x04xx)
constexpr ErrorCodeEx LOOP_CLOSURE_REJECTED   {ErrorSeverity::INFO,    ErrorComponent::LOOP_DETECTOR,    0x001};
constexpr ErrorCodeEx LOOP_NO_CANDIDATES      {ErrorSeverity::INFO,    ErrorComponent::LOOP_DETECTOR,    0x002};
constexpr ErrorCodeEx LOOP_DESCRIPTOR_FAIL    {ErrorSeverity::WARNING, ErrorComponent::LOOP_DESCRIPTOR,  0x001};
constexpr ErrorCodeEx LOOP_TEASER_FAIL        {ErrorSeverity::WARNING, ErrorComponent::LOOP_MATCHER,     0x001};
constexpr ErrorCodeEx LOOP_TEASER_LOW_INLIERS {ErrorSeverity::INFO,    ErrorComponent::LOOP_MATCHER,     0x002};
constexpr ErrorCodeEx LOOP_ICP_FAIL           {ErrorSeverity::WARNING, ErrorComponent::LOOP_ICP,         0x001};
constexpr ErrorCodeEx LOOP_ICP_RMSE_HIGH      {ErrorSeverity::WARNING, ErrorComponent::LOOP_ICP,         0x002};

// 后端优化错误 (BACKEND_xxx, 0x05xx)
constexpr ErrorCodeEx ISAM2_UPDATE_FAILED     {ErrorSeverity::ERROR,   ErrorComponent::BACKEND_ISAM2,    0x001};
constexpr ErrorCodeEx ISAM2_DIVERGED          {ErrorSeverity::CRITICAL,ErrorComponent::BACKEND_ISAM2,    0x002};
constexpr ErrorCodeEx ISAM2_COVARIANCE_FAIL   {ErrorSeverity::WARNING, ErrorComponent::BACKEND_ISAM2,    0x003};
constexpr ErrorCodeEx ISAM2_SINGULAR_MATRIX   {ErrorSeverity::ERROR,   ErrorComponent::BACKEND_ISAM2,    0x004};

constexpr ErrorCodeEx HBA_TRIGGER_FAILED      {ErrorSeverity::ERROR,   ErrorComponent::BACKEND_HBA,      0x001};
constexpr ErrorCodeEx HBA_OPTIMIZATION_FAIL   {ErrorSeverity::ERROR,   ErrorComponent::BACKEND_HBA,      0x002};
constexpr ErrorCodeEx HBA_TIMEOUT             {ErrorSeverity::WARNING, ErrorComponent::BACKEND_HBA,      0x003};
constexpr ErrorCodeEx HBA_MME_HIGH            {ErrorSeverity::INFO,    ErrorComponent::BACKEND_HBA,      0x004};

constexpr ErrorCodeEx POSEGRAPH_ADD_FAILED    {ErrorSeverity::ERROR,   ErrorComponent::BACKEND_POSEGRAPH, 0x001};
constexpr ErrorCodeEx POSEGRAPH_OPTIMIZE_FAIL {ErrorSeverity::ERROR,   ErrorComponent::BACKEND_POSEGRAPH, 0x002};

// 地图错误 (MAP_xxx, 0x06xx)
constexpr ErrorCodeEx MAP_BUILD_FAILED        {ErrorSeverity::ERROR,   ErrorComponent::MAP_GLOBAL,       0x001};
constexpr ErrorCodeEx MAP_MEMORY_LIMIT        {ErrorSeverity::WARNING, ErrorComponent::MAP_GLOBAL,       0x002};
constexpr ErrorCodeEx MAP_QUERY_FAILED        {ErrorSeverity::WARNING, ErrorComponent::MAP_GLOBAL,       0x003};

constexpr ErrorCodeEx MAP_EXPORT_FAILED       {ErrorSeverity::ERROR,   ErrorComponent::MAP_EXPORTER,     0x001};
constexpr ErrorCodeEx MAP_EXPORT_FORMAT_UNSUPPORT {ErrorSeverity::ERROR, ErrorComponent::MAP_EXPORTER, 0x002};
constexpr ErrorCodeEx MAP_EXPORT_DISK_FULL    {ErrorSeverity::CRITICAL,ErrorComponent::MAP_EXPORTER,     0x003};

constexpr ErrorCodeEx MAP_FILTER_FAILED       {ErrorSeverity::WARNING, ErrorComponent::MAP_FILTER,       0x001};

// IO错误 (IO_xxx, 0x07xx)
constexpr ErrorCodeEx FILE_NOT_FOUND          {ErrorSeverity::ERROR,   ErrorComponent::IO_FILE,          0x001};
constexpr ErrorCodeEx FILE_READ_ERROR         {ErrorSeverity::ERROR,   ErrorComponent::IO_FILE,          0x002};
constexpr ErrorCodeEx FILE_WRITE_ERROR        {ErrorSeverity::ERROR,   ErrorComponent::IO_FILE,          0x003};
constexpr ErrorCodeEx FILE_PERMISSION_DENIED  {ErrorSeverity::ERROR,   ErrorComponent::IO_FILE,          0x004};
constexpr ErrorCodeEx FILE_CORRUPTED          {ErrorSeverity::ERROR,   ErrorComponent::IO_FILE,          0x005};

constexpr ErrorCodeEx PCD_LOAD_FAILED         {ErrorSeverity::ERROR,   ErrorComponent::IO_PCD,           0x001};
constexpr ErrorCodeEx PCD_SAVE_FAILED         {ErrorSeverity::ERROR,   ErrorComponent::IO_PCD,           0x002};
constexpr ErrorCodeEx PLY_LOAD_FAILED         {ErrorSeverity::ERROR,   ErrorComponent::IO_PLY,           0x001};
constexpr ErrorCodeEx PLY_SAVE_FAILED         {ErrorSeverity::ERROR,   ErrorComponent::IO_PLY,           0x002};

// 系统错误 (SYSTEM_xxx, 0x08xx)
constexpr ErrorCodeEx OUT_OF_MEMORY           {ErrorSeverity::CRITICAL,ErrorComponent::SYSTEM_MEMORY,    0x001};
constexpr ErrorCodeEx MEMORY_ALLOCATION_FAIL  {ErrorSeverity::ERROR,   ErrorComponent::SYSTEM_MEMORY,    0x002};
constexpr ErrorCodeEx MEMORY_LEAK_DETECTED    {ErrorSeverity::WARNING, ErrorComponent::SYSTEM_MEMORY,    0x003};

constexpr ErrorCodeEx THREAD_POOL_EXHAUSTED   {ErrorSeverity::WARNING, ErrorComponent::SYSTEM_THREAD,    0x001};
constexpr ErrorCodeEx THREAD_START_FAILED     {ErrorSeverity::ERROR,   ErrorComponent::SYSTEM_THREAD,    0x002};
constexpr ErrorCodeEx DEADLOCK_DETECTED       {ErrorSeverity::FATAL,   ErrorComponent::SYSTEM_THREAD,    0x003};

constexpr ErrorCodeEx TIMEOUT                 {ErrorSeverity::WARNING, ErrorComponent::SYSTEM_TIMEOUT,   0x001};
constexpr ErrorCodeEx OPERATION_CANCELLED     {ErrorSeverity::INFO,    ErrorComponent::SYSTEM_TIMEOUT,   0x002};

constexpr ErrorCodeEx RESOURCE_EXHAUSTED      {ErrorSeverity::CRITICAL,ErrorComponent::SYSTEM_RESOURCE,  0x001};
constexpr ErrorCodeEx GPU_MEMORY_LOW          {ErrorSeverity::WARNING, ErrorComponent::SYSTEM_RESOURCE,  0x002};

// 通信错误 (COMM_xxx, 0x09xx)
constexpr ErrorCodeEx ROS_NODE_INIT_FAILED    {ErrorSeverity::CRITICAL,ErrorComponent::COMM_ROS,         0x001};
constexpr ErrorCodeEx ROS_PUBLISHER_FAIL      {ErrorSeverity::ERROR,   ErrorComponent::COMM_ROS,         0x002};
constexpr ErrorCodeEx ROS_SUBSCRIBER_FAIL     {ErrorSeverity::ERROR,   ErrorComponent::COMM_ROS,         0x003};

constexpr ErrorCodeEx SERVICE_CALL_FAILED     {ErrorSeverity::WARNING, ErrorComponent::COMM_SERVICE,     0x001};
constexpr ErrorCodeEx SERVICE_TIMEOUT         {ErrorSeverity::WARNING, ErrorComponent::COMM_SERVICE,     0x002};
constexpr ErrorCodeEx SERVICE_UNAVAILABLE     {ErrorSeverity::ERROR,   ErrorComponent::COMM_SERVICE,     0x003};

constexpr ErrorCodeEx TF_LOOKUP_FAILED        {ErrorSeverity::WARNING, ErrorComponent::COMM_TF,          0x001};
constexpr ErrorCodeEx TF_EXTRAPOLATION_FAIL   {ErrorSeverity::WARNING, ErrorComponent::COMM_TF,          0x002};

// 配置错误 (CORE_CONFIG)
constexpr ErrorCodeEx CONFIG_LOAD_FAILED      {ErrorSeverity::ERROR,   ErrorComponent::CORE_CONFIG,      0x001};
constexpr ErrorCodeEx CONFIG_PARSE_ERROR      {ErrorSeverity::ERROR,   ErrorComponent::CORE_CONFIG,      0x002};
constexpr ErrorCodeEx CONFIG_VALIDATION_FAIL  {ErrorSeverity::ERROR,   ErrorComponent::CORE_CONFIG,      0x003};
constexpr ErrorCodeEx CONFIG_KEY_NOT_FOUND    {ErrorSeverity::WARNING, ErrorComponent::CORE_CONFIG,      0x004};
constexpr ErrorCodeEx CONFIG_TYPE_MISMATCH    {ErrorSeverity::ERROR,   ErrorComponent::CORE_CONFIG,      0x005};

// 外部依赖错误 (EXT_xxx, 0x0Axx)
constexpr ErrorCodeEx GTSAM_EXCEPTION         {ErrorSeverity::ERROR,   ErrorComponent::EXT_GTSAM,        0x001};
constexpr ErrorCodeEx PCL_EXCEPTION           {ErrorSeverity::ERROR,   ErrorComponent::EXT_PCL,          0x001};
constexpr ErrorCodeEx OPENCV_EXCEPTION        {ErrorSeverity::ERROR,   ErrorComponent::EXT_OPENCV,       0x001};
constexpr ErrorCodeEx TORCH_EXCEPTION         {ErrorSeverity::ERROR,   ErrorComponent::EXT_TORCH,        0x001};
constexpr ErrorCodeEx TORCH_MODEL_LOAD_FAIL   {ErrorSeverity::ERROR,   ErrorComponent::EXT_TORCH,        0x002};
constexpr ErrorCodeEx TORCH_INFERENCE_FAIL    {ErrorSeverity::WARNING, ErrorComponent::EXT_TORCH,        0x003};
constexpr ErrorCodeEx TEASER_EXCEPTION        {ErrorSeverity::ERROR,   ErrorComponent::EXT_TEASER,       0x001};
constexpr ErrorCodeEx HBA_EXCEPTION           {ErrorSeverity::ERROR,   ErrorComponent::EXT_HBA,          0x001};

// 通用错误
constexpr ErrorCodeEx SUCCESS                 {ErrorSeverity::INFO,    ErrorComponent::CORE_SYSTEM,      0x000};
constexpr ErrorCodeEx UNKNOWN_ERROR           {ErrorSeverity::ERROR,   ErrorComponent::CORE_SYSTEM,      0xFFF};

} // namespace errors

// ─────────────────────────────────────────────────────────────────────────────
// 错误恢复建议
// ─────────────────────────────────────────────────────────────────────────────
struct RecoverySuggestion {
    std::string action;          // 建议操作
    std::string expected_result; // 预期结果
    int priority;                // 优先级 (1=高, 2=中, 3=低)
    bool auto_recoverable;       // 是否可自动恢复

    RecoverySuggestion(const std::string& act = "",
                       const std::string& result = "",
                       int pri = 2,
                       bool auto_rec = false)
        : action(act), expected_result(result), priority(pri), auto_recoverable(auto_rec) {}
};

// ─────────────────────────────────────────────────────────────────────────────
// 错误上下文（结构化）
// ─────────────────────────────────────────────────────────────────────────────
struct ErrorContext {
    std::string trace_id;        // 追踪ID（关联整个请求链）
    std::string span_id;         // 当前Span ID
    std::string parent_span_id;  // 父Span ID
    std::string operation;       // 操作名称
    std::string file;            // 文件名
    int line = 0;                // 行号
    std::string function;        // 函数名
    std::string thread_id;       // 线程ID

    // 附加数据（键值对）
    std::map<std::string, std::string> metadata;

    // 时间戳
    std::chrono::system_clock::time_point timestamp =
        std::chrono::system_clock::now();

    std::string toString() const;
};

// ─────────────────────────────────────────────────────────────────────────────
// 增强型错误详情
// ─────────────────────────────────────────────────────────────────────────────
class ErrorDetail {
public:
    ErrorDetail(ErrorCodeEx code = errors::UNKNOWN_ERROR);
    ErrorDetail(ErrorCodeEx code, const std::string& message);

    // 基本信息
    ErrorCodeEx code() const { return code_; }
    ErrorSeverity severity() const { return code_.severity(); }
    ErrorComponent component() const { return code_.component(); }
    const std::string& message() const { return message_; }

    // 上下文
    ErrorContext& context() { return context_; }
    const ErrorContext& context() const { return context_; }
    ErrorDetail& setContext(const ErrorContext& ctx) { context_ = ctx; return *this; }

    // 错误链（用于包装底层错误）
    const std::vector<ErrorDetail>& causes() const { return causes_; }
    ErrorDetail& addCause(const ErrorDetail& cause) {
        causes_.push_back(cause);
        return *this;
    }
    ErrorDetail& wrap(const ErrorDetail& inner, const std::string& outer_message) {
        message_ = outer_message;
        causes_.push_back(inner);
        return *this;
    }

    // 恢复建议
    const std::vector<RecoverySuggestion>& suggestions() const { return suggestions_; }
    ErrorDetail& addSuggestion(const RecoverySuggestion& sug) {
        suggestions_.push_back(sug);
        return *this;
    }

    // 重试信息
    bool isRetryable() const { return retryable_; }
    ErrorDetail& setRetryable(bool retryable, int max_attempts = 3, int delay_ms = 100) {
        retryable_ = retryable;
        max_retry_attempts_ = max_attempts;
        retry_delay_ms_ = delay_ms;
        return *this;
    }
    int maxRetryAttempts() const { return max_retry_attempts_; }
    int retryDelayMs() const { return retry_delay_ms_; }

    // 序列化
    std::string toString() const;
    std::string toJson() const;
    std::string toShortString() const;

    // 静态工厂方法
    static ErrorDetail fromException(const std::exception& e, ErrorCodeEx code);

private:
    ErrorCodeEx code_;
    std::string message_;
    ErrorContext context_;
    std::vector<ErrorDetail> causes_;
    std::vector<RecoverySuggestion> suggestions_;
    bool retryable_ = false;
    int max_retry_attempts_ = 3;
    int retry_delay_ms_ = 100;
};

// ─────────────────────────────────────────────────────────────────────────────
// 错误码工具函数
// ─────────────────────────────────────────────────────────────────────────────
namespace error_utils {

// 获取错误严重程度的字符串表示
std::string severityToString(ErrorSeverity sev);
ErrorSeverity stringToSeverity(const std::string& str);

// 获取组件的字符串表示
std::string componentToString(ErrorComponent comp);
ErrorComponent stringToComponent(const std::string& str);

// 获取预定义错误码的描述
std::string getErrorDescription(ErrorCodeEx code);

// 获取预定义错误码的恢复建议
std::vector<RecoverySuggestion> getDefaultSuggestions(ErrorCodeEx code);

// 检查错误是否可忽略
bool isIgnorable(ErrorCodeEx code);

// 检查错误是否需要立即处理
bool requiresImmediateAction(ErrorCodeEx code);

// 检查错误是否需要重启系统
bool requiresSystemRestart(ErrorCodeEx code);

} // namespace error_utils

} // namespace automap_pro
