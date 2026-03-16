#include "automap_pro/backend/isam2_constraint_validation.h"

#include <rclcpp/rclcpp.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <set>
#include <string>

namespace automap_pro {

void logAllConstraintsAndValidate(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values,
    const char* tag,
    ConstraintValidation* out_validation)
{
    auto log = rclcpp::get_logger("automap_system");
    const size_t nf = graph.size();
    const size_t nv = values.size();
    RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] tag=%s total_factors=%zu total_values=%zu (优化前全量约束，崩溃时 grep GTSAM_CONSTRAINTS 定位)",
                tag, nf, nv);

    std::set<gtsam::Key> value_key_set;
    for (const gtsam::Key k : values.keys())
        value_key_set.insert(k);

    for (size_t i = 0; i < nf; ++i) {
        const auto& f = graph[i];
        gtsam::KeyVector kv = f->keys();
        std::string keys_str;
        std::string in_vals_str;
        std::string finite_str;
        bool factor_ok = true;
        for (gtsam::Key k : kv) {
            keys_str += (keys_str.empty() ? "" : ",") + std::to_string(k);
            bool in_vals = (value_key_set.count(k) != 0);
            in_vals_str += (in_vals_str.empty() ? "" : ",") + std::string(in_vals ? "yes" : "NO");
            bool finite = false;
            if (in_vals && values.exists(k)) {
                try {
                    auto p = values.at<gtsam::Pose3>(k);
                    Eigen::Vector3d t = p.translation();
                    Eigen::Matrix3d R = p.rotation().matrix();
                    finite = t.array().isFinite().all() && R.array().isFinite().all();
                } catch (...) {
                    finite = false;
                }
                if (!finite) factor_ok = false;
            } else if (!in_vals) {
                factor_ok = false;
            }
            finite_str += (finite_str.empty() ? "" : ",") + std::string(finite ? "yes" : (in_vals ? "nan/inf" : "n/a"));
        }
        const char* type_str = "Other";
        if (dynamic_cast<const gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) type_str = "Prior";
        else if (dynamic_cast<const gtsam::GPSFactor*>(f.get())) type_str = "GPS";
        else if (dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(f.get())) type_str = "Between";
        RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] factor %zu type=%s keys=[%s] keys_in_values=[%s] value_finite=[%s] valid=%s",
                    i, type_str, keys_str.c_str(), in_vals_str.c_str(), finite_str.c_str(), factor_ok ? "yes" : "NO");
        if (out_validation) {
            for (gtsam::Key k : kv)
                if (value_key_set.count(k) == 0) out_validation->all_keys_exist = false;
            if (!factor_ok) out_validation->all_values_finite = false;
        }
    }

    for (const gtsam::Key k : values.keys()) {
        std::string pose_str = "n/a";
        bool finite = false;
        bool reasonable = false;
        try {
            auto p = values.at<gtsam::Pose3>(k);
            Eigen::Vector3d t = p.translation();
            Eigen::Matrix3d R = p.rotation().matrix();
            finite = t.array().isFinite().all() && R.array().isFinite().all();
            double tnorm = t.norm();
            reasonable = finite && (tnorm <= kMaxReasonableTranslationNorm);
            pose_str = "x=" + std::to_string(t.x()) + " y=" + std::to_string(t.y()) + " z=" + std::to_string(t.z());
            if (out_validation && finite && tnorm > kMaxReasonableTranslationNorm)
                out_validation->all_values_reasonable = false;
        } catch (...) {
            pose_str = "at_failed";
        }
        RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] value key=%zu %s finite=%s reasonable=%s",
                    static_cast<size_t>(k), pose_str.c_str(), finite ? "1" : "0", reasonable ? "1" : "0");
        if (out_validation && !finite) out_validation->all_values_finite = false;
    }

    if (out_validation) {
        for (size_t i = 0; i < nf; ++i) {
            for (gtsam::Key k : graph[i]->keys()) {
                if (value_key_set.count(k) == 0) {
                    out_validation->all_keys_exist = false;
                    break;
                }
            }
        }
        out_validation->message =
            (out_validation->all_keys_exist && out_validation->all_values_finite && out_validation->all_values_reasonable)
                ? "ok"
                : ("keys_exist=" + std::string(out_validation->all_keys_exist ? "1" : "0") +
                   " values_finite=" + std::string(out_validation->all_values_finite ? "1" : "0") +
                   " values_reasonable=" + std::string(out_validation->all_values_reasonable ? "1" : "0"));
    }
    RCLCPP_INFO(log, "[GTSAM_CONSTRAINTS] tag=%s validation done %s (若崩溃在 error() 或 optimize() 内，见上一 GTSAM_CONSTRAINTS 约束列表)",
                tag, out_validation ? out_validation->message.c_str() : "n/a");
}

}  // namespace automap_pro
