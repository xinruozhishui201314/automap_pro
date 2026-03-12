#pragma once

/**
 * 崩溃时在 stderr 输出最后一步上下文，便于即使日志缓冲未刷新也能定位。
 * 仅使用 async-signal-safe 的 write(2)，在 SIGABRT/SIGSEGV 时写入 last_step。
 */
namespace automap_pro {
namespace crash_report {

/** 设置“最后一步”标识（在每次关键 GTSAM/危险调用前调用），最多 127 字符 */
void setLastStep(const char* step);

/** 安装 SIGABRT/SIGSEGV 处理器，崩溃时向 stderr 写入 [CRASH_REPORT] signal=X last_step=... */
void installCrashHandler();

}  // namespace crash_report
}  // namespace automap_pro
