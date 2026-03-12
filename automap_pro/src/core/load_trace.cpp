/**
 * 库加载顺序追踪：在 .so 静态初始化时写 stderr，便于定位「double free in _GLOBAL__sub_I_lago」等启动崩溃。
 * 不依赖 Logger/rclcpp，仅用 fprintf，保证在任意 .so 初始化阶段可安全执行。
 * 若崩溃在 libgtsam 静态初始化，会看到 automap_core 的 LOAD_TRACE，不会看到 automap_backend 的。
 */
#include <cstdio>

#ifdef __linux__

namespace {
void automap_core_so_loaded() __attribute__((constructor(101)));
void automap_core_so_loaded() {
    fprintf(stderr, "[LOAD_TRACE] automap_core.so static init done (next: loader may load automap_backend -> libgtsam; if crash in lago, see FIX_GTSAM_LAGO_STATIC_INIT_DOUBLE_FREE.md)\n");
    fflush(stderr);
}
}  // namespace

#endif
