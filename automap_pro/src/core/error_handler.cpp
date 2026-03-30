/**
 * @file core/error_handler.cpp
 * @brief 核心实现。
 */
#include "automap_pro/core/error_handler.h"
#include <random>
#include <iomanip>
#include <sstream>

namespace automap_pro {

// Error实现
std::string Error::toString() const {
    std::ostringstream oss;
    oss << "[" << getCodeString() << "]";
    if (!component.empty()) {
        oss << "[" << component << "]";
    }
    oss << " " << message;
    if (!context.empty()) {
        oss << " (context: " << context << ")";
    }
    return oss.str();
}

std::string Error::getCodeString() const {
    std::ostringstream oss;
    oss << "0x" << std::hex << std::setw(4) << std::setfill('0') 
        << static_cast<uint32_t>(code);
    return oss.str();
}

// ErrorHandler实现
void ErrorHandler::logError(const Error& error) {
    LogContext ctx;
    ctx.component = error.component;
    ctx.fields["error_code"] = error.getCodeString();
    ctx.fields["error_message"] = error.message;
    if (!error.context.empty()) {
        ctx.fields["error_context"] = error.context;
    }
    
    if (static_cast<uint32_t>(error.code) >= 0x5000) {
        LOG_ERROR(std::string("Error: ") + error.toString());
    } else {
        LOG_WARN(std::string("Recoverable error: ") + error.toString());
    }
}

void ErrorHandler::logException(const std::exception& e) {
    std::ostringstream oss;
    oss << "Exception caught: " << e.what() << " | type: " << typeid(e).name();
    LOG_ERROR(oss.str());
}

Result<void> ErrorHandler::safeExecute(std::function<Result<void>()> func) {
    try {
        return func();
    } catch (const AutomapException& e) {
        logException(e);
        return e.error();
    } catch (const std::exception& e) {
        logException(e);
        return Error(ErrorCode::UNKNOWN_ERROR, e.what());
    } catch (...) {
        LOG_ERROR("Unknown exception caught");
        return Error(ErrorCode::UNKNOWN_ERROR, "Unknown exception");
    }
}

Result<void> ErrorHandler::retry(std::function<Result<void>()> func,
                                 int max_attempts, int delay_ms) {
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        std::ostringstream oss;
        oss << "Attempt " << attempt << "/" << max_attempts;
        LOG_DEBUG(oss.str());
        
        auto result = func();
        if (result.isOk()) {
            return Result<void>{};
        }
        
        if (attempt < max_attempts) {
            std::ostringstream woss;
            woss << "Attempt " << attempt << "/" << max_attempts << " failed, retrying in "
                 << delay_ms << "ms: " << result.error().toString();
            LOG_WARN(woss.str());
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        } else {
            std::ostringstream eoss;
            eoss << "All " << max_attempts << " attempts failed: " << result.error().toString();
            LOG_ERROR(eoss.str());
            return result.error();
        }
    }
    
    return Error(ErrorCode::UNKNOWN_ERROR, "Retry loop should not reach here");
}

Error ErrorHandler::wrapException(const std::exception& e,
                                 const std::string& component,
                                 ErrorCode default_code) {
    if (const auto* ae = dynamic_cast<const AutomapException*>(&e)) {
        return ae->error();
    }
    
    return Error(default_code, e.what(), component);
}

std::string ErrorHandler::generateTraceId() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint64_t> dist;
    
    uint64_t id = dist(gen);
    std::ostringstream oss;
    oss << std::hex << std::setw(16) << std::setfill('0') << id;
    return oss.str();
}

std::string ErrorHandler::generateSessionId() {
    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    static std::atomic<uint32_t> counter{0};
    uint32_t seq = counter.fetch_add(1, std::memory_order_relaxed);
    
    std::ostringstream oss;
    oss << "sess_" << timestamp << "_" << std::setw(6) << std::setfill('0') << seq;
    return oss.str();
}

} // namespace automap_pro
