#include "LIVMapper.h"
#include <cstdio>
#include <cstring>
#include <csignal>
#include <cstdlib>
#if defined(__linux__) || defined(__APPLE__)
#include <execinfo.h>
#endif

namespace {
  volatile std::sig_atomic_t g_segv_received = 0;

  void segv_handler(int sig) {
    (void)sig;
    g_segv_received = 1;
    std::fprintf(stderr, "[fast_livo][SIGSEGV] process received SIGSEGV. Last LIO step in log = last '[fast_livo][LIO][TRACE] frame=N step=...' line.\n");
    std::fprintf(stderr, "[fast_livo][SIGSEGV] To locate: grep -E 'fast_livo\\\\[LIO\\\\]\\\\[TRACE\\\\]|fast_livo\\\\[CRASH_CONTEXT\\\\]' <log> | tail -10  (last line = crash step)\n");
    std::fprintf(stderr, "[fast_livo][SIGSEGV] Enable coredump: ulimit -c unlimited; then: gdb -c core.<pid> $(which fastlivo_mapping) -> 'bt full'\n");
#if defined(__linux__) || defined(__APPLE__)
    void* buf[64];
    int n = backtrace(buf, 64);
    if (n > 0) {
      std::fprintf(stderr, "[fast_livo][SIGSEGV] backtrace (best-effort, %d frames):\n", n);
      char** syms = backtrace_symbols(buf, n);
      if (syms) {
        for (int i = 0; i < n; i++) std::fprintf(stderr, "  #%d %s\n", i, syms[i]);
        std::free(syms);
      }
    }
#endif
    std::fflush(stderr);
    std::signal(SIGSEGV, SIG_DFL);
    std::raise(SIGSEGV);
  }
}

int main(int argc, char **argv)
{
  std::signal(SIGSEGV, segv_handler);
  // 启动即打印，便于确认是否运行了「已修复 parameter ''」的二进制（若未见此行说明未重新编译 fast_livo）
  std::fprintf(stderr, "[fast_livo] main: automatically_declare_parameters_from_overrides=false (parameter '' fix applied)\n");
  // 诊断：打印所有 --params-file 参数，便于定位「parameter ''」是否由多文件合并导致
  for (int i = 0; i < argc; ++i) {
    if (std::strcmp(argv[i], "--params-file") == 0 && i + 1 < argc)
      std::fprintf(stderr, "[fast_livo] [DIAG] --params-file #%d = %s\n", (i/2)+1, argv[i+1]);
  }
  std::fflush(stderr);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  // false: 仅声明代码中显式 declare 的参数，避免 YAML 解析产生空名 key 时触发 parameter '' has invalid type。
  // 参数文件中的 overrides 仍会在 declare_parameter() 时被应用。
  options.automatically_declare_parameters_from_overrides(false);

  rclcpp::Node::SharedPtr nh;
  image_transport::ImageTransport it_(nh);
  LIVMapper mapper(nh, "laserMapping", options);
  mapper.initializeSubscribersAndPublishers(nh, it_);
  mapper.run(nh);
  rclcpp::shutdown();
  return 0;
}