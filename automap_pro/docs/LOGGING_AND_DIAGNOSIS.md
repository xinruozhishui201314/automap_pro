# 日志与精准诊断说明

本文档说明关键流水线日志标签、如何根据“最后一条日志”定位崩溃/异常，以及对应修复策略。

---

## 1. 日志标签与流水线步骤

### 1.1 全局地图发布（publishGlobalMap，易崩溃路径）

| 标签 / 关键字 | 含义 | 崩溃时“最后一条”说明 |
|---------------|------|------------------------|
| `[AutoMapSystem][BACKEND] frame_no=... step=publishGlobalMap enter` | 即将发布全局地图 | 崩溃在 publishGlobalMap 内部 |
| `[AutoMapSystem][MAP] publishGlobalMap step=enter` | 进入 publishGlobalMap | 崩溃在取配置或其后 |
| `[AutoMapSystem][MAP] publishGlobalMap step=using_cached_voxel` | 使用缓存的 voxel_size | 崩溃在 buildGlobalMap |
| `[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_enter` | 进入 buildGlobalMap | 崩溃在 SubMapManager::buildGlobalMap |
| `[AutoMapSystem][MAP] buildGlobalMap step=locked_enter` | 准备加锁 | 崩溃在加锁或其后 |
| `[AutoMapSystem][MAP] buildGlobalMap step=locked_done submaps=N` | 锁内，子图数 N | 崩溃在合并或 downsample |
| `[AutoMapSystem][MAP] buildGlobalMap step=sm_add idx=... sm_id=... pts=...` | 正在合并第 idx 个子图 | 崩溃在该子图拷贝或下一子图 |
| `[AutoMapSystem][MAP] buildGlobalMap step=merge_done combined=N` | 合并完成，点数 N | 崩溃在 voxelDownsampleChunked |
| `[AutoMapSystem][MAP] buildGlobalMap step=before_downsample` | 即将体素滤波 | 崩溃在 utils::voxelDownsampleChunked |
| `[Utils] [MAP] voxelDownsampleChunked step=enter` | 进入分块体素滤波 | 崩溃在 sanitize 或其后 |
| `[Utils] [MAP] voxelDownsampleChunked step=bypass_chunked` | 点数≤250k，走单次体素（安全路径） | 若仍崩溃则在 voxelDownsample 内部 |
| `[Utils] [MAP] voxelDownsampleChunked step=chunk_grid nx=...` | 分块网格已算 | 崩溃在 chunk 循环内 |
| `[Utils] [MAP] voxelDownsampleChunked step=chunk ix=... voxel_enter` | 正在对某 chunk 做体素 | 崩溃在该次 voxelDownsample(chunk) |
| `[Utils] [MAP] voxelDownsampleChunked step=chunk_merge_done` | 所有 chunk 已合并 | 崩溃在最终 voxelDownsample(merged) |
| `[AutoMapSystem][MAP] publishGlobalMap step=buildGlobalMap_done` | 全局点云已生成 | 崩溃在 toROSMsg / 发布 / rviz |

### 1.2 数据流与后端

| 标签 | 含义 |
|------|------|
| `[AutoMapSystem][DATA_FLOW]` | 接收/缓存/队列/发布计数汇总 |
| `[AutoMapSystem][BACKEND][RECV]` | 后端收到点云/入队 |
| `[AutoMapSystem][BACKEND][FRAME]` | 本帧处理结果（kf_created / skip_no_kf 等） |
| `[LivoBridge][RECV]` / `[LivoBridge][FRAME]` | 前端点云接收与转发 |

---

## 2. 精准定位步骤（崩溃时）

1. **保留完整终端输出**，从 `step=publishGlobalMap enter` 到进程退出（含 exit code -11 等）。
2. **找最后一条带 `[AutoMapSystem][MAP]` 或 `[Utils] [MAP]` 的日志**，对照上表确定崩溃阶段。
3. 若最后是 `step=chunk ix=... voxel_enter`：崩溃在该 chunk 的 PCL 体素滤波；可配合 `--gdb` 拿 backtrace 确认是否在 `pcl::VoxelGrid::applyFilter`。
4. 若最后是 `step=buildGlobalMap_enter` 且无 `locked_enter`：崩溃在 buildGlobalMap 入口（如 logger / 锁）。

---

## 3. 已实施的加固与策略

| 措施 | 说明 |
|------|------|
| **小点云绕过 chunk** | 点数 ≤ 250000 时直接单次 `voxelDownsample`，不走分块循环，避免 M2DGR 等场景在 chunk 循环内 SIGSEGV。 |
| **子图合并上限** | buildGlobalMap 中单子图/合并总点数上限，避免 PCL 或内存异常。 |
| **合并用 reserve+push_back** | 不用 PCL `operator+=`，避免大块重分配导致崩溃。 |
| **chunk 内 try-catch** | 单 chunk 的 `voxelDownsample` 抛异常时记录日志并用该 chunk 原样合并，不中断整体。 |
| **map_voxel_size 缓存** | publishGlobalMap 不再在回调中调用 ConfigManager，避免析构顺序问题。 |

---

## 4. 常用 grep 示例

```bash
# 只看地图发布与体素相关
grep -E '\[MAP\]|publishGlobalMap|buildGlobalMap|voxelDownsampleChunked' 日志或终端输出

# 只看后端帧处理与地图触发
grep -E 'BACKEND.*FRAME|step=publishGlobalMap|DATA_FLOW' 日志或终端输出

# 崩溃前最后 20 行（若已重定向到文件）
grep -E '\[MAP\]|publishGlobalMap|buildGlobalMap|voxelDownsample' log.txt | tail -20
```

---

## 5. GDB 与 Core Dump

崩溃精确定位请配合 GDB：使用 `--gdb` 启动或事后用 core 分析。详见 [DEBUG_WITH_GDB.md](DEBUG_WITH_GDB.md)。

---

## 6. 修改阈值或日志量

- **放宽小点云绕过阈值**：在 `automap_pro/src/core/utils.cpp` 中修改 `kVoxelChunkedSizeThreshold`（当前 250000）。调大则更多场景走单次体素。
- **减少 chunk 步进日志**：将 `voxelDownsampleChunked` 内 `ALOG_INFO("Utils", "[MAP] voxelDownsampleChunked step=chunk ...")` 改为 `ALOG_DEBUG`，可减少刷屏，需要时再开 DEBUG 级别。
