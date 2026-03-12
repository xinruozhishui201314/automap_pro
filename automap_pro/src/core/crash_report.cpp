#include "automap_pro/core/crash_report.h"

#include <csignal>
#include <cstring>
#include <unistd.h>

namespace automap_pro {
namespace crash_report {

namespace {

constexpr size_t kMaxStep = 128;
static char g_last_step[kMaxStep] = "unknown";
static std::sig_atomic_t g_installed = 0;

void crash_handler(int sig) {
    const char prefix[] = "[CRASH_REPORT] signal=";
    (void)write(2, prefix, sizeof(prefix) - 1);
    const char* sig_str = (sig == 6) ? "SIGABRT" : (sig == 11) ? "SIGSEGV" : "SIGOTHER";
    size_t n = 0;
    while (sig_str[n]) n++;
    (void)write(2, sig_str, n);
    const char mid[] = " last_step=";
    (void)write(2, mid, sizeof(mid) - 1);
    size_t len = 0;
    while (len < kMaxStep - 1 && g_last_step[len]) len++;
    if (len > 0)
        (void)write(2, g_last_step, len);
    (void)write(2, "\n", 1);
    /* 恢复默认并重新抛出，让进程按原样终止并生成 core */
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}

}  // namespace

void setLastStep(const char* step) {
    if (!step) return;
    size_t i = 0;
    while (i < kMaxStep - 1 && step[i]) {
        g_last_step[i] = step[i];
        i++;
    }
    g_last_step[i] = '\0';
}

void installCrashHandler() {
    if (g_installed) return;
    g_installed = 1;
    std::signal(SIGABRT, crash_handler);
#ifdef __linux__
    std::signal(SIGSEGV, crash_handler);
#endif
}

}  // namespace crash_report
}  // namespace automap_pro
