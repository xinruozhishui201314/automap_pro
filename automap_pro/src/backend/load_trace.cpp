/**
 * @file backend/load_trace.cpp
 * @brief 后端优化与因子实现。
 */
/**
 * 库加载顺序追踪：automap_backend.so 静态初始化时写 stderr。
 * 本 .so 依赖 libgtsam；加载时先加载 libgtsam 并执行其静态初始化，再执行本 constructor。
 * 若崩溃在 _GLOBAL__sub_I_lago.cpp，则不会看到本条 LOAD_TRACE。
 */
#include <cstdio>

#ifdef __linux__

namespace {
void automap_backend_so_loaded() __attribute__((constructor(102)));
void automap_backend_so_loaded() {
    fprintf(stderr, "[LOAD_TRACE] automap_backend.so static init done (libgtsam already loaded and inited; if you see this, crash was not in gtsam static init)\n");
    fflush(stderr);
}
}  // namespace

#endif
