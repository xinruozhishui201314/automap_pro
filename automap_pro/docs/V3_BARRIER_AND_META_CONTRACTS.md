# V3 屏障策略与事件契约（EventMeta / ref / session）

本文档为产品语义单一来源：超时行为、降级策略、与 `EventMeta::isValid()` 对齐的发布要求。

## 1. EventMeta 完备性

凡携带 `EventMeta` 且可能被 `isValid()` 校验的事件，发布前必须设置：

| 字段 | 要求 |
|------|------|
| `event_id` / `idempotency_key` / `producer_seq` | 非 0 |
| `session_id` | 非 0（来自 `MapRegistry::getSessionId()` 或服务触发时的 Registry） |
| `ref_version` / `ref_epoch` | `ref_epoch` 非 0（与 Registry `alignment_epoch` 对齐） |
| `source_ts` / `publish_ts` | 有限浮点 |
| `producer` | 非空字符串 |
| `route_tag` | 建议非空（如 `legacy` / `service` / `advice`） |

**不含 EventMeta 的事件**（如 `RawOdometryEvent`、`SaveMapRequestEvent`）不适用上述规则。

## 2. 屏障与超时（统一语义）

### 2.1 MappingModule（`FilteredFrameEventRequiredDs`）

- **条件**：出队要求 `ref_map_version <= registry_version` 且 `ref_alignment_epoch <= processed_alignment_epoch`（本地已处理世代不低于帧契约）。
- **超时**：队列首帧长期不满足时，超时后**丢弃最旧帧**并计数（见 `STALE_VERSION_DROP_TOTAL` 等），避免死锁。
- **语义**：超时 = **丢帧**（不静默阻塞整条管线）。

### 2.2 VisualizationModule（`SyncedFrameEvent`）

- **条件**：`registry_version >= ev.ref_map_version` 且 `registry_alignment_epoch >= ev.ref_alignment_epoch`（与 Mapping 一致，不要求 `reg_ep == ref_ep`）。
- **超时（5s）**：**不丢事件**；**降级**为仍调用 `processSyncedFrameViz`（依赖 `shouldBypassAnchorChainForVisualization` + 会话级 `T_map_odom`）。指标：`v3_viz_registry_barrier_timeout_total`。
- **语义**：超时 = **降级显示**，非静默吞帧。

### 2.3 SemanticModule

- **输入**：`SyncedFrameEvent`（经队列）、`GraphTaskEvent` / `SemanticInputEvent`（独立语义输入）。
- **版本门控**：若配置启用，对过时 `ref_version` 的帧**丢弃**并计数（`STALE_VERSION_DROP_TOTAL`）；与「可视化降级」不同，因语义推理成本高且可安全跳过。
- **契约**：`SemanticInputEvent` 必须满足 `meta.isValid()`，否则 `handleSemanticInputEvent` 直接拒绝。

## 3. 共享位姿内核

所有「ODOM 关键帧 → map」「锚点链 world→map」「漂移混合」必须使用头文件 `include/automap_pro/v3/pose_chain.hpp` 中的函数，禁止在业务模块复制粘贴矩阵公式。

## 4. 多会话聚合可视化（`publishEverything`）

- 当 Registry 中关键帧出现**多个** `session_id` 时，全图 `optimized_path` / `keyframe_poses` 仍使用**当前快照全局** `R_enu_to_map` / `t_enu_to_map`。
- **语义**：该聚合**仅在单会话、单一对齐基准下严格正确**；多会话时指标 `v3_viz_multi_session_aggregate_total` 递增并打 throttle 日志。逐帧正确显示应依赖 `SyncedFrame` 路径与会话级 `session_alignments`。

## 5. 观测性指标（结构化）

| 指标名 | 含义 |
|--------|------|
| `v3_viz_registry_barrier_timeout_total` | SyncedFrame 可视化屏障等待超时次数 |
| `v3_viz_pose_drift_warn_total` | 当前云链式 vs GPS 平移差 > 0.5m 的告警次数 |
| `v3_viz_multi_session_aggregate_total` | 全图发布时检测到多 session 的次数 |
| `v3_viz_last_current_cloud_blend_w` | 最近一次混合权重（Gauge：SyncedFrame 当前云 / 语义 world / 树干 world 共用） |
| `v3_viz_current_cloud_chain_drift_m` | 链式 vs 直接对齐平移差（Histogram，米；同上多路径） |

（另：既有 `frame_mismatch_total`、`stale_version_drop_total` 等仍用于契约违反与 Mapping 侧丢帧。）

## 6. MapOrchestrator（全量 EventMeta 观测）

`MapOrchestrator` 订阅并顺序校验以下事件的 `meta`（`isValid`、乱序、`stale_epoch`），**不**订阅自产的 `RouteAdviceEvent`：

`SyncedFrameEvent`、`FilteredFrameEventRequiredDs`、`OptimizationResultEvent`、`OptimizationDeltaEvent`、`GPSAlignedEvent`、`SemanticLandmarkEvent`、`SemanticCloudEvent`、`SemanticTrunkVizEvent`、`GraphTaskEvent`、`SemanticInputEvent`、`BackpressureWarningEvent`、`SystemQuiesceRequestEvent`。

## 7. 发布点审计备注

- `GPSAlignedEvent`：`meta.session_id` 须来自 `MapRegistry::getSessionId()`，否则 `GPSAlignedEvent::isValid()` 失败、事件不会发布。
- `RouteAdviceEvent`：`maybePublishAdvice` 须写入 `session_id`（Registry），否则 `MappingModule` 中 `ev.meta.isValid()` 会直接丢弃建议。`MappingModule` 对**任意** `event_type` 的建议均更新 `route_takeover_enabled_`（与 `orchestrator.takeover_enabled` 配合：Semantic / OptimizationDelta / FilteredFrame 等与 Synced、OptResult 同权驱动 takeover）。
- `SystemQuiesceRequestEvent`：须 `meta.isValid()` 后才发布；无 Registry 时不发裸事件（见 `handleFinishMapping`）。
- `GraphTaskEvent` / `SemanticInputEvent`：发布前须 `isValid()`（Mapping `publishGraphTaskEvent`、独立语义输入、服务 `TriggerOptimize`、Optimizer 入队）。
- `SemanticCloudEvent` / `SemanticTrunkVizEvent`：结构体级 `isValid()`；Semantic 发布前校验，Viz 入队前丢弃非法事件。
- `BackpressureWarningEvent`：结构体级 `isValid()`；Mapping/Semantic 发布前校验。
- **KF 时间匹配**：凡按时间挂接 KF 的路径须使用 `getKeyFrameByTimestampPreferPrior(ts, tol, session_id)`（`session_id` 来自事件 `meta` 或当前 Registry），禁止跨 session 误挂。
