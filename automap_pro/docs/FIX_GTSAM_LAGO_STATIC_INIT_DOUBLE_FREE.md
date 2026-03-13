# GTSAM LAGO 静态初始化 Double Free 修复

## 0) Executive Summary

| 项目 | 内容 |
|------|------|
| **崩溃类型** | `double free or corruption (out)` → SIGABRT |
| **触发场景** | 程序**启动时**，在 main 执行前或刚加载 automap 组件时即崩溃 |
| **调用栈** | `free()` ← `_GLOBAL__sub_I_lago.cpp` (libgtsam.so) ← dlopen/库加载 ← ClassLoader/main |
| **根本原因** | GTSAM 与 PCL/Eigen 的 **编译选项不一致**（尤其 `-march`/Eigen 对齐），导致在 libgtsam 的静态初始化（lago.cpp 中的全局 noise model）时发生堆损坏或二次释放。参见 borglab/gtsam#1450、#1436。 |
| **修复方案** | **构建层**：保证 GTSAM、PCL 及所有使用 Eigen 的库使用**同一套** Eigen 与 march 设置（见下）。 |
| **影响范围** | 仅影响使用 GTSAM 的节点启动；修复后无需改业务代码 |

---

## 1) 崩溃现象

### 1.1 典型日志

```text
double free or corruption (out)

Thread 1 "automap_system_" received signal SIGABRT, Aborted.
#6  free () from /lib/x86_64-linux-gnu/libc.so.6
#7  _GLOBAL__sub_I_lago.cpp () from install_deps/gtsam/lib/libgtsam.so.4
...
#19 rcutils_load_shared_library ()
#20 rcpputils::SharedLibrary::SharedLibrary(...)
#21 class_loader::impl::loadLibrary(...)
#22 class_loader::ClassLoader::loadLibrary()
#23 class_loader::ClassLoader::ClassLoader(...)
#24 main ()
```

### 1.2 触发条件

- 以 **standalone 可执行文件**（automap_system）或 **component 被容器加载** 方式启动时，动态链接器加载 `libgtsam.so`。
- 在 GTSAM 的 **静态初始化** 阶段，`gtsam/slam/lago.cpp` 中的全局对象（如 `priorOrientationNoise`、`priorPose2Noise`）构造时，若与当前进程内其他库（如 PCL、Eigen）的 ABI 不一致，会触发 `free()` 时的 double free 或堆损坏。

### 1.3 根据 LOAD_TRACE 日志定位

代码中已增加库加载与首次 GTSAM 调用的追踪日志（`[LOAD_TRACE]`、`[IncrementalOptimizer][LOAD_TRACE]`、`[GTSAM_Guard][LOAD_TRACE]`）。崩溃时看 **最后一条** 出现的 LOAD_TRACE 即可判断阶段：

| 最后一条 LOAD_TRACE | 崩溃阶段 |
|---------------------|----------|
| `automap_core.so static init done` | 随后加载 libgtsam 时在其**静态初始化**内崩溃（lago 等）→ 按 3.1–3.3 做构建层修复或 3.4/3.5 缓解。 |
| `automap_backend.so static init done` | 已过 libgtsam 静态初始化，崩溃在后续**首次使用 GTSAM**（如 IncrementalOptimizer 构造）→ 查 borglab/gtsam#1189 与 ISAM2 首次 update 文档。 |
| `main() entered` 或 `about to create AutoMapSystem` | 崩溃在 **Node/IncrementalOptimizer 构造** 内 → 看是否有 `[IncrementalOptimizer][LOAD_TRACE] about to construct gtsam::ISAM2`，若有则崩溃在 ISAM2 构造。 |
| `[IncrementalOptimizer][LOAD_TRACE] about to construct gtsam::ISAM2` 且无后续 `constructed ok` | 崩溃在 **gtsam::ISAM2(params)** 或 prior_noise_ 初始化。 |

---

## 2) 根本原因（与 borglab/gtsam#1450 一致）

- **PCL 1.13** 等默认开启 `-march=native`（或 AVX/SSE），导致 Eigen 的对齐与代码生成与其它库不一致。
- **GTSAM** 若使用自带的 Eigen 或不同的 `-march` 编译，与 PCL/应用侧使用的 Eigen 实现在**同一进程内混用**，会在静态初始化或后续使用中触发对齐断言失败或 double free。
- 社区结论：**所有使用 Eigen 的库必须用同一套编译设置（同一 Eigen、同一 march）**。

---

## 3) 修复方案（构建层，必须）

### 3.1 构建 GTSAM 时

使用与工作空间一致的 Eigen，并关闭 GTSAM 的“本地架构”优化：

```bash
# 推荐 CMake 选项（与 borglab/gtsam#1450 及 RTABMAP 等方案一致）
cmake ... \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
```

若仍崩溃，可再尝试：

```bash
-DCMAKE_CXX_FLAGS="-std=c++17"
```

### 3.2 构建 PCL 时（若从源码构建 PCL）

与 GTSAM 同机、同环境构建时，关闭 PCL 的 march/AVX/SSE，避免与 GTSAM/Eigen 冲突：

```bash
cmake ... \
  -DPCL_ENABLE_AVX=OFF \
  -DPCL_ENABLE_SSE=OFF \
  -DPCL_ENABLE_MARCHNATIVE=OFF
```

### 3.3 使用系统/第三方提供的 GTSAM 时

- 若 GTSAM 来自 **install_deps** 或其它预编译包，需确认其与当前 PCL/Eigen 的 ABI 一致；
- 若无法确认，建议在**同一工作空间内从源码构建 GTSAM**，并加上上述 `GTSAM_USE_SYSTEM_EIGEN=ON` 与 `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`。

### 3.3.1 GTSAM 已用系统 Eigen 时仍崩溃

若 GTSAM 编译时**已使用系统 Eigen**（`GTSAM_USE_SYSTEM_EIGEN=ON`），则 Eigen 来源已统一，剩余可疑点为 **-march 与 PCL 一致性**：

| 检查项 | 建议 |
|--------|------|
| **GTSAM** | 确认 `GTSAM_BUILD_WITH_MARCH_NATIVE=OFF`（与 borglab/gtsam#1450 一致），否则 GTSAM 的 SIMD 代码生成可能与其他库不一致。 |
| **PCL** | 若 PCL 从源码构建，使用 `-DPCL_ENABLE_MARCHNATIVE=OFF`（及 AVX/SSE=OFF）与 GTSAM 对齐。 |
| **automap_pro** | 本仓库已默认 `AUTOMAP_AVOID_MARCH_NATIVE=ON`，Release 不加 `-march=native`；需**重新配置并编译** automap_pro 后该设置才生效。 |

若 GTSAM 无法重编（如预编译包），可尝试：launch 参数 `gtsam_preload_path` 或环境变量 `LD_PRELOAD` 预加载 libgtsam（见 3.4、3.5）。

---

## 3.4 代码侧缓解（本仓库已做）

- **Release 不用 -march=native**：CMake 选项 `AUTOMAP_AVOID_MARCH_NATIVE` 默认 **ON**，Release 不再加 `-march=native`，与 GTSAM/PCL 的 Eigen ABI 更一致。若需极致性能可设 `-DAUTOMAP_AVOID_MARCH_NATIVE=OFF` 重新配置并**确保 GTSAM/PCL 同机同 flags 重编**。
- **Launch 可选 LD_PRELOAD**：`automap_offline.launch.py` 支持参数 `gtsam_preload_path`，指向 `libgtsam.so.4` 的绝对路径时，会为该节点设置 `LD_PRELOAD` 预加载 GTSAM：
  ```bash
  ros2 launch automap_pro automap_offline.launch.py bag_file:=/path/to/bag gtsam_preload_path:=/path/to/install_deps/gtsam/lib/libgtsam.so.4
  ```

## 3.5 临时缓解：手动 LD_PRELOAD（无法重编 GTSAM 时）

若暂时无法重编 GTSAM，可尝试**先加载 libgtsam**，再启动进程：

```bash
export LD_PRELOAD=/path/to/libgtsam.so.4   # 如 install_deps/gtsam/lib/libgtsam.so.4
ros2 launch automap_pro automap_offline.launch.py ...
# 或直接用 launch 参数：gtsam_preload_path:=/path/to/libgtsam.so.4
```

若仍崩溃，仍须按 3.1–3.3 做构建层修复。

---

## 4) 验证

1. 按上述选项重新构建 GTSAM（及如需时 PCL）。
2. 重新构建 automap_pro，并启动节点（standalone 或 component 方式）。
3. 确认不再出现 `double free or corruption (out)` 及 `_GLOBAL__sub_I_lago.cpp` 的 backtrace。

---

## 5) 与 ISAM2 首次 update double free 的区别

| 项目 | 本文（LAGO 静态初始化） | ISAM2 首次 update（FIX_ISAM2_FIRST_UPDATE_DOUBLE_FREE_20260311.md） |
|------|-------------------------|----------------------------------------------------------------------|
| 发生时机 | 启动时，main 前或库加载时 | 运行时，如调用 `finish_mapping` 触发 forceUpdate/ISAM2 首次 update |
| 调用栈 | `lago.cpp` 静态初始化 → `free()` | `NoiseModelFactor::error()` / `linearize()` → `free()` |
| 原因 | Eigen/PCL/GTSAM 编译选项不一致 | GTSAM 已知 bug borglab/gtsam#1189 |
| 修复 | 统一 GTSAM/PCL/Eigen 构建选项 | 首次 update 仅注入 values、不跑优化（V5） |

两者可同时存在；若启动即崩，先按本文修复构建；若在 finish_mapping 等场景崩，按 ISAM2 文档修复。

---

## 6) 参考

- [borglab/gtsam Discussion #1450](https://github.com/borglab/gtsam/discussions/1450) — Double free or corruption (out)
- [borglab/gtsam Issue #1189](https://github.com/borglab/gtsam/issues/1189) — ISAM2 update() double free
- [introlab/rtabmap Issue #586](https://github.com/introlab/rtabmap/issues/586) — RTABMAP + GTSAM segfault（Eigen/march 一致性）
