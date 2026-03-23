# 坐标系与接口契约更新（2026-03-23）

> 适用范围：V3 微内核链路（FrontEnd/GPS/Loop/Optimizer/Mapping/Visualization）  
> 目标：把“隐式假设”升级为“显式契约”，在不兼容时快速报错或拒绝数据。

---

## 1. 本次变更摘要

- 新增系统协议契约头文件：`automap_pro/include/automap_pro/core/protocol_contract.h`
- 引入接口版本与命名常量（topics/services）统一来源（SSoT）
- 在配置加载阶段增加协议版本校验（Fail-Fast）
- 在位姿数据链路中补齐 `PoseFrame` 语义标签并在关键入口做坐标系核校
- 在 Mapping/Visualization 网关增加 ODOM->MAP 的契约化转换与防错拦截

---

## 2. 协议契约（微服务接口）

### 2.1 契约定义

文件：`automap_pro/include/automap_pro/core/protocol_contract.h`

- 版本号：
  - `API_MAJOR`
  - `API_MINOR`
  - `API_PATCH`
- 统一命名：
  - `protocol::topics::*`
  - `protocol::services::*`
- 兼容策略：
  - `isCompatible(major, minor)` 要求主版本一致，小版本向后兼容

### 2.2 生效位置

- `system_init.cpp`
  - 发布器/服务名改为读取 `protocol_contract` 常量，避免散落硬编码
- `service_handlers.cpp`
  - 关键服务路径输出协议版本信息，便于运行期审计
- `config_manager.cpp`
  - 若配置含 `system.api_version`，启动时执行兼容性校验
  - 不兼容时直接 `FATAL + throw`，阻止系统在未知契约下运行

---

## 3. 坐标系契约（位姿数据）

### 3.1 数据结构契约

文件：`automap_pro/include/automap_pro/core/data_types.h`

- `OptimizationResult`：增加 `pose_frame`，并补充 `isFinite()/isReasonable()`
- `HBAResult`：补充 `isFinite()/isReasonable()`
- `KeyFrame`：增加 `pose_frame`
- `SubMap`：增加 `pose_frame`

### 3.2 事件契约

文件：`automap_pro/include/automap_pro/v3/map_registry.h`

- `SyncedFrameEvent` 增加 `pose_frame`（默认 `ODOM`）
- `OptimizationResultEvent` 增加 `pose_frame`（优化结果语义显式化）

### 3.3 生产-转换-消费链路

- **生产端（FrontEnd）**
  - 文件：`automap_pro/src/v3/frontend_module.cpp`
  - 发布 `SyncedFrameEvent` 时显式设置 `event.pose_frame = PoseFrame::ODOM`

- **转换端（Optimizer/Mapping）**
  - 文件：`automap_pro/src/backend/incremental_optimizer.cpp`
  - 维护 `current_pose_frame_`，在 `OptimizationResult` 输出时携带语义
  - GPS 因子注入/对齐重建后语义切到 `MAP`
  - 文件：`automap_pro/src/v3/mapping_module.cpp`
  - 建立统一位姿应用网关 `applyOptimizedPoses(...)`
  - 入库前做：版本检查、坐标系检查、有限值检查、异常补偿拦截

- **消费端（Visualization）**
  - 文件：`automap_pro/include/automap_pro/v3/visualization_module.h`
  - 渲染时根据 `SyncedFrameEvent.pose_frame` 判断是否应用 `T_map_odom`
  - 避免已是 `MAP` 语义的数据再次补偿（防双变换）

---

## 4. 失配时系统行为（Fail-Fast / Fail-Safe）

- 协议版本不兼容：启动阶段直接失败（Fail-Fast）
- 位姿含 NaN/Inf 或异常平移：网关拒绝本批结果（Fail-Safe）
- 过期版本优化结果：拒绝覆盖新版本结果（版本护盾）
- 模块停止后异步结果到达：拒绝处理（生命周期护栏）

---

## 5. 运维与调试建议

- 配置中显式声明：
  - `system.api_version: "<major>.<minor>"`
- 日志检索建议：
  - `PROTOCOL`
  - `POSE_DIAG`
  - `CRASH_GUARD`
  - `GATEWAY`
- 回归验证最小用例：
  1. 未对齐运行（纯 ODOM）；
  2. 中途 GPS 对齐；
  3. 触发 iSAM2 与 HBA；
  4. 检查 RViz 当前帧/轨迹/全局图是否同系。

---

## 6. 向后兼容说明

- 如果旧配置不含 `system.api_version`，当前实现会给出告警并进入兼容模式。
- 建议逐步把所有部署配置补齐 `system.api_version`，再切换到严格模式。
