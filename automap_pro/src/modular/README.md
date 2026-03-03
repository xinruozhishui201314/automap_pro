# AutoMap-Pro 模块（modular）

本目录为工程子模块源码，对应 ROS2 包：**fast_livo**、**overlap_transformer_ros2**、**hba** 等。  
**所有模块节点的推荐启动方式**：由 **`automap_pro/launch/`** 统一启动，参数唯一来源于 **`automap_pro/config/system_config.yaml`**。

---

## 与主 Launch 的关系

| 模块 | ROS2 包名 | 节点/可执行文件 | 主 launch 中的启动方式 |
|------|-----------|----------------|------------------------|
| fast-livo2-humble | `fast_livo` | `fastlivo_mapping` | **前端精度已验证**，FAST-LIVO2 官方 LIVMapper；`automap_offline/online/incremental` 中 `use_external_frontend:=true` 时启动，参数由 `params_from_system_config.get_fast_livo2_params` 生成 |
| OverlapTransformer ROS2 | `overlap_transformer_ros2` | `descriptor_server` | 同上，`use_external_overlap:=true` 时启动，参数来自 `get_overlap_transformer_params` |
| HBA-main/HBA_ROS2 | `hba` | `hba`, `calculate_MME`, `visualize` | 同上，`use_hba:=true`（默认）、`use_hba_cal_mme:=true`、`use_hba_visualize:=true` 时分别启动，参数来自 `get_hba_params` / `get_hba_cal_mme_params` / `get_hba_visualize_params` |

- **入口 launch**：`automap_pro/launch/automap_offline.launch.py`、`automap_online.launch.py`、`automap_incremental.launch.py`
- **参数生成**：`automap_pro/launch/params_from_system_config.py` 从 `system_config.yaml` 生成各节点参数字典，launch 中直接传入对应 Node 的 `parameters=[...]`。

---

## 各模块自带的 Launch 文件

用于**单独调试**该模块时；正常建图请使用 `automap_pro/launch`。

- **HBA**：`HBA-main/HBA_ROS2/launch/hba_launch.py`、`cal_MME_launch.py`、`visualize_launch.py`  
  可传入 `system_config_path:=/path/to/automap_pro/config/system_config.yaml` 以与主配置一致。
- **fast-livo2**：`fast-livo2-humble/launch/*.launch.py` 为不同雷达/数据集的示例，主流程使用 automap_pro launch + 生成的 `fast_livo_params.yaml`。

---

## 编译与工作空间

上述包需与 `automap_pro` 一起在**同一工作空间**中编译（如 `automap_ws`），以便 `ros2 launch automap_pro ...` 能解析到 `fast_livo`、`overlap_transformer_ros2`、`hba` 包并启动对应节点。
