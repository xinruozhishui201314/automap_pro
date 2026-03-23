#pragma once

#include <string>
#include <vector>
#include <memory>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

/**
 * HBA-main 批处理桥接：导出 KeyFrame 序列为 HBA 所需 pose.json + pcd/，
 * 可选调用 HBA 可执行文件，读回 pose_trans.json 并写回 KeyFrame 的 T_map_b_optimized。
 */
class HBABridge {
public:
    HBABridge() = default;

    /** 从所有子图中按时间顺序收集 KeyFrame（用于导出/读回顺序一致） */
    static std::vector<KeyFrame::Ptr> collectKeyframesInOrder(
        const std::vector<SubMap::Ptr>& submaps);

    /**
     * 导出到 HBA 数据目录：pose.json (lt tx ty tz qx qy qz qw) + pcd/0000.pcd ...
     * 返回导出的 keyframes 顺序（与 pose 行、pcd 编号一致）。
     */
    static bool exportToHBAFormat(
        const std::string& data_path,
        const std::vector<KeyFrame::Ptr>& keyframes,
        int pcd_name_fill_num = 4);

    /**
     * 读取 pose_trans.json，按 keyframes 顺序写回 T_map_b_optimized。
     * 若行数与 keyframes 不一致则返回 false。
     */
    static bool loadHBAResultAndApply(
        const std::string& data_path,
        std::vector<KeyFrame::Ptr>& keyframes);

    /**
     * 执行 HBA 可执行文件（阻塞），data_path 已由 exportToHBAFormat 写好。
     * command_template 例如 "ros2 run hba hba --ros-args -p data_path:=%s"
     * 返回是否执行成功（退出码 0）。
     */
    static bool runHBAProcess(
        const std::string& data_path,
        const std::string& command_template);
};

}  // namespace automap_pro
