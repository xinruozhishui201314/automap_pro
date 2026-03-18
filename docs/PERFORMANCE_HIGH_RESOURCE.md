# 高资源宿主机性能调优（20+ 核 / 64GB 内存）

宿主机 CPU 核心多、内存大时，可通过配置**充分发挥多核与内存**，加快建图、减少队列满/背压。

## 1. 为何不能无脑拉高所有线程？

- **GTSAM / Eigen 限制**：为避免 GTSAM 内部 TBB/Eigen 多线程导致 SIGSEGV（如 borglab/gtsam#1189），当前在进程内统一设置：
  - `OMP_NUM_THREADS=1`
  - `EIGEN_NUM_THREADS=1`
  - `TBB_NUM_THREADS=1`
  因此 **OpenMP 类并行**（如体素下采样的 `parallel_voxel_downsample`）在同一进程中实际也是单线程。多核利用主要靠**任务级并行**：更多 worker 线程、HBA 内部线程、异步任务队列等。

- **TEASER/PMC**：回环里 TEASER 的 PMC 最大团求解已固定为单线程（`max_clique_num_threads=1`），以规避已知崩溃；多核用在**多候选并行匹配**（`parallel_teaser_match`）和**回环 worker 数量**上。

## 2. 推荐配置项（20+ 核、64GB）

在 `system_config_M2DGR.yaml`（或你当前使用的配置）中可做如下调整。

### 2.1 系统与队列（内存与缓冲）

| 配置项 | 默认/当前 | 高资源建议 | 说明 |
|--------|-----------|------------|------|
| `system.num_threads` | 8 | 16 或 20 | 供部分模块参考的“逻辑核数”，可适当提高 |
| `system.frame_queue_max_size` | 5000 | 6000~8000 | 后端消费快时可略增，减少背压 |
| `system.ingress_queue_max_size` | 2000 | 3000 | 入口队列，多核时回调可更快入队 |
| `keyframe.max_memory_mb` | 4096 | 8192 或 12288 | 64GB 下可给关键帧/点云更多缓存，减少淘汰 |

### 2.2 性能开关（任务级并行）

| 配置项 | 默认/当前 | 高资源建议 | 说明 |
|--------|-----------|------------|------|
| `performance.parallel_voxel_downsample` | false (M2DGR) | true | 体素下采样内部可并行；当前 OMP=1 时仍为单线程，但与其他模块一致，建议开启 |
| `performance.parallel_teaser_match` | false (M2DGR) | true | **多候选 TEASER 匹配并行**（std::async），能明显吃多核 |
| `performance.max_optimization_queue_size` | 64 | 128 | 优化任务队列更大，减少丢任务 |

### 2.3 回环检测（多 worker + 队列）

| 配置项 | 默认/当前 | 高资源建议 | 说明 |
|--------|-----------|------------|------|
| `loop_closure.worker_threads` | 2 | 6~8 | 回环检测 worker 数量，多核时可显著提高 |
| `loop_closure.max_desc_queue_size` | 128 | 256 | 描述子队列 |
| `loop_closure.max_match_queue_size` | 128 | 256 | 匹配任务队列 |

### 2.4 后端 HBA（多线程）

| 配置项 | 默认/当前 | 高资源建议 | 说明 |
|--------|-----------|------------|------|
| `backend.hba.thread_num` | 8 | 16 或 20 | HBA 内部并行线程数，吃满多核 |

### 2.5 可选：后端发布频率

| 配置项 | 默认/当前 | 高资源建议 | 说明 |
|--------|-----------|------------|------|
| `backend.publish_global_map_every_n_processed` | 150 | 100~150 | 资源充足时可略增发布频率；再大则 buildGlobalMap 更频繁，按需权衡 |
| `backend.process_every_n_frames` | 5 | 3 或 5 | 更小=更密关键帧、负载更高；多核时可保持 5 或略降到 3 |

## 3. 配置片段示例（复制到 YAML 中替换或合并）

```yaml
# ── 高资源预设（20+ 核 / 64GB）────────────────────────────────────────────
system:
  num_threads: 20
  frame_queue_max_size: 7000
  ingress_queue_max_size: 3000

performance:
  parallel_voxel_downsample: true
  parallel_teaser_match: true
  max_optimization_queue_size: 128

keyframe:
  max_memory_mb: 10240   # 64GB 下可给 10GB 关键帧缓存

loop_closure:
  worker_threads: 8
  max_desc_queue_size: 256
  max_match_queue_size: 256

backend:
  hba:
    thread_num: 20
  publish_global_map_every_n_processed: 120
```

## 4. 运行时环境（保持 GTSAM 稳定）

脚本 `run_automap.sh` 和 GTSAM Guard 已设置：

- `OMP_NUM_THREADS=1`
- `EIGEN_NUM_THREADS=1`
- `TBB_NUM_THREADS=1`
- `AUTOMAP_GTSAM_SERIAL=1`

**不要**在运行 automap 时把这些改成大于 1，否则可能触发 GTSAM 崩溃。多核利用完全由上面 YAML 中的 **worker 数、HBA thread_num、并行开关** 来拉满。

## 5. 效果与监控

- **CPU**：提高 `worker_threads` 和 `backend.hba.thread_num` 后，应能看到多核利用率明显上升（如 `htop`）。
- **内存**：增大 `keyframe.max_memory_mb` 后，进程 RSS 会升高，64GB 下 10~12GB 给关键帧是合理范围。
- **队列**：若日志中 `[STUCK_DIAG] intra_loop_task_queue full` 仍频繁出现，可考虑在代码中适当提高 `kMaxIntraLoopTaskQueueSize`（当前为 8），或确保 `worker_threads` 与 HBA 已按上表提高。

总结：在**不提高 OMP/EIGEN/TBB 线程数**的前提下，通过 **回环 worker 数、HBA 线程数、并行 TEASER、队列与关键帧内存** 即可充分发挥 20+ 核与 64GB 内存。
