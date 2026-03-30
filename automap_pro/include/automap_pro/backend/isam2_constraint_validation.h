#pragma once
/**
 * @file backend/isam2_constraint_validation.h
 * @brief 后端优化：iSAM2、HBA、GPS/回环因子、任务队列与坐标管理。
 */


/**
 * iSAM2 约束验证日志模块
 *
 * 提供优化前约束全量转储与合理性校验，便于崩溃精准定位。
 * 校验失败则中止 update 避免触发 GTSAM 崩溃。
 */

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <string>

namespace automap_pro {

/**
 * 约束验证结果结构
 */
struct ConstraintValidation {
    bool all_keys_exist = true;
    bool all_values_finite = true;
    bool all_values_reasonable = true;  // 平移/旋转在合理范围内
    std::string message;
};

/**
 * 平移合理范围（米），超出视为异常输入，避免 GTSAM 数值问题
 */
constexpr double kMaxReasonableTranslationNorm = 1e6;

/**
 * 优化前约束全量转储与合理性校验
 *
 * @param graph 因子图
 * @param values 值集合
 * @param tag 操作标签（用于日志定位）
 * @param out_validation 输出验证结果（可为 nullptr）
 */
void logAllConstraintsAndValidate(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,
    const char* tag,
    ConstraintValidation* out_validation = nullptr);

}  // namespace automap_pro
