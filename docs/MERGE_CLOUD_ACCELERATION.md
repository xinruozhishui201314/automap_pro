# mergeCloudToSubmap 并行与加速方案

## 1. 当前流程简述

`SubMapManager::mergeCloudToSubmap` 在 **backend 单线程**内被调用（与 createKeyFrame/addKeyFrame 串行），主要步骤：

| 步骤 | 操作 | 当前实现 | 耗时特点 |
|------|------|----------|----------|
| 1. 合并前体素 | merged_cloud > 20 万点时先降采样 | `voxelDownsampleWithTimeout(15s)` → 内部 `voxelDownsample` | 大点云时 30–40ms+，易超时 |
| 2. 变换 | body → world | `pcl::transformPointCloud` | 单线程逐点 |
| 3. 合并 | 将本帧世界系点云追加到 merged_cloud | `for (pt) push_back(pt)` | 顺序、可优化 |
| 4. 合并后体素 | merged_cloud > 20 万点时再降采样 | `voxelDownsampleWithTimeout(15s)` | 同步骤 1 |

**能否整体并行**：mergeCloudToSubmap 本身与「当前关键帧的 createKeyFrame / addKeyFrame」强耦合（读写同一 `sm->merged_cloud`），因此**不适合**把整段 merge 丢到另一线程与 createKeyFrame 并行；只能做**内部**并行与算法级加速。

---

## 2. 加速方法概览

### 2.1 体素 pre/post（已做 + 可选增强）

- **已有**：`voxelDownsample` 在 `performance.parallel_voxel_downsample=true` 时走 **OpenMP** 路径（`voxelDownsampleGridNoPCLParallel`），多核可加速单次体素。
- **配置**：`system_config*.yaml` 中 `performance.parallel_voxel_downsample: true`（高资源配置建议保持 true）。
- **可选增强**：对**超大点云**（如 > 50 万点），在 merge 的 pre/post 中改用 **分块体素**（`voxelDownsampleChunked`），其内部用 `#pragma omp parallel for schedule(dynamic)` 对多块并行降采样，再合并；外层仍用 async+wait_for 做超时（即实现 `voxelDownsampleChunkedWithTimeout`），这样既有并行又保留超时保护。

### 2.2 变换（transform）并行

- **现状**：`pcl::transformPointCloud` 单线程逐点变换。
- **做法**：对 `world_cloud->points` 按索引分块，用 OpenMP `#pragma omp parallel for` 对每点做 `T * pt` 写入目标，等价于一次并行 transform，可显著缩短大点云变换时间。
- **位置**：可在 `utils` 中新增 `transformPointCloudParallel(cloud_in, cloud_out, T)`，在 `mergeCloudToSubmap` 中调用；或直接在 submap_manager 内对点循环加 OMP。

### 2.3 合并（append）加速

- **现状**：`reserve(old_size + world_cloud->size())` 后 `for (pt) push_back(pt)`。
- **做法**：改为 **一次 insert**：  
  `sm->merged_cloud->points.insert(sm->merged_cloud->points.end(), world_cloud->points.begin(), world_cloud->points.end());`  
  减少循环开销、利于缓存，通常比逐点 push_back 更快（已实现）。

### 2.4 降低体素触发频率/负载

- **提高阈值**：将 `kDownsampleThreshold`（当前 200000）适当调大，可减少 pre/post 体素触发次数，降低单帧 merge 耗时；但 merged_cloud 会更大，内存与后续一次体素耗时增加，需折中。
- **保证 feeder 预降采样**：若每帧都有可用的 `cloud_ds`，createKeyFrame 不会在内部再做体素，减轻的是 createKeyFrame 路径；merge 仍依赖 merged_cloud 自身大小，所以仍需控制 merged_cloud 规模（阈值或分块体素）。

### 2.5 不推荐的做法

- **把整段 merge 放到独立线程与 addKeyFrame 并行**：会与「当前子图写入」竞争，需要加锁或复制子图，复杂度高且易出错；当前设计是 backend 单线程顺序执行 addKeyFrame → merge，更稳妥。
- **在 merge 内对 merged_cloud 多线程并发 push_back**：需按块划分写入区间、无锁合并，实现复杂；用单次 insert 已能获得大部分收益。

---

## 3. 实现状态与配置小结

| 项目 | 状态 | 说明 |
|------|------|------|
| 体素内部 OpenMP | ✅ 已有 | `parallel_voxel_downsample: true` |
| Append 改为 insert | ✅ 已实现 | 见 submap_manager mergeCloudToSubmap |
| 分块体素 + 超时（merge pre/post） | ✅ 已实现 | `voxelDownsampleChunkedWithTimeout`，内部 OpenMP 分块并行 |
| 变换并行 | 可选 | 需 `transformPointCloudParallel` 或在 merge 内 OMP for |

**建议**：先确保 `parallel_voxel_downsample: true` 并观察 merge 的 pre/post_voxel duration_ms；若仍经常超时或单次 > 1s，再考虑实现「分块体素+超时」或「变换并行」。
