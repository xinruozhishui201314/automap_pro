#include "automap_pro/v3/semantic_processor.h"
#include "automap_pro/v3/semantic_segmentor_factory.h"
#include "automap_pro/core/logger.h"
#include "automap_pro/core/config_manager.h"
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <array>
#include <limits>
#include <sstream>
#include <iomanip>
#include <unordered_map>

// sloam_rec：include 根目录为包内 thrid_party/sloam_rec/sloam/include（由 CMake 注入）
#include <segmentation/trellis.h>
#include <helpers/definitions.h>

#include <ceres/ceres.h>
#include <ceres/manifold.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>

namespace automap_pro::v3 {

namespace {

int resolveTreeLabel(const SemanticProcessor::Config& cfg) {
    if (cfg.diag_override_tree_class_id >= 0) {
        return cfg.diag_override_tree_class_id;
    }
    if (cfg.tree_class_id >= 0) {
        return cfg.tree_class_id;
    }
    // Auto fallback by backend contract:
    // - sloam writes trunk as 255 in its mask output.
    // - lsk3dnet/lsk3dnet_hybrid outputs learning-label IDs; SemanticKITTI trunk=16.
    if (cfg.model_type == "lsk3dnet" || cfg.model_type == "lsk3dnet_hybrid") {
        return 16;
    }
    return 255;
}

}  // namespace

struct CylinderFittingCost {
    CylinderFittingCost(const Eigen::Vector3d& point) : point_(point) {}

    template <typename T>
    bool operator()(const T* const root, const T* const ray, const T* const radius, T* residual) const {
        Eigen::Matrix<T, 3, 1> p{T(point_.x()), T(point_.y()), T(point_.z())};
        Eigen::Matrix<T, 3, 1> r{root[0], root[1], root[2]};
        Eigen::Matrix<T, 3, 1> v{ray[0], ray[1], ray[2]};
        T rad = radius[0];

        // Distance from point p to line (r, v)
        // d = |(p - r) x v| / |v|
        Eigen::Matrix<T, 3, 1> pr = p - r;
        Eigen::Matrix<T, 3, 1> cross = pr.cross(v);
        T v_norm = v.norm();
        if (v_norm < T(1e-6)) return false;
        
        T dist_to_axis = cross.norm() / v_norm;
        residual[0] = dist_to_axis - rad;
        return true;
    }

    Eigen::Vector3d point_;
};

SemanticProcessor::SemanticProcessor(const Config& config) : config_(config) {
    try {
        if (config_.diag_cluster_profile == "relaxed") {
            // Diagnostic-only profile for quickly testing "cluster threshold too strict".
            config_.beam_cluster_threshold = std::max(config_.beam_cluster_threshold, 0.5f);
            config_.max_dist_to_centroid = std::max(config_.max_dist_to_centroid, 0.5f);
            config_.min_vertex_size = std::min(config_.min_vertex_size, 2);
            config_.min_landmark_size = std::min(config_.min_landmark_size, 2);
            config_.min_landmark_height = std::min(config_.min_landmark_height, 0.3f);
        }

        SegmentorConfig seg_cfg;
        seg_cfg.model_type = config.model_type;
        seg_cfg.model_path = config.model_path;
        seg_cfg.lsk3dnet_model_path = config.lsk3dnet_model_path;
        seg_cfg.lsk3dnet_device = config.lsk3dnet_device;
        seg_cfg.lsk3dnet_repo_root = config.lsk3dnet_repo_root;
        seg_cfg.lsk3dnet_config_yaml = config.lsk3dnet_config_yaml;
        seg_cfg.lsk3dnet_checkpoint = config.lsk3dnet_checkpoint;
        seg_cfg.lsk3dnet_classifier_torchscript = config.lsk3dnet_classifier_torchscript;
        seg_cfg.lsk3dnet_python_exe = config.lsk3dnet_python_exe;
        seg_cfg.lsk3dnet_worker_script = config.lsk3dnet_worker_script;
        seg_cfg.lsk3dnet_hybrid_normal_mode = config.lsk3dnet_hybrid_normal_mode;
        seg_cfg.lsk3dnet_normal_fov_up_deg = config.lsk3dnet_normal_fov_up_deg;
        seg_cfg.lsk3dnet_normal_fov_down_deg = config.lsk3dnet_normal_fov_down_deg;
        seg_cfg.lsk3dnet_normal_proj_h = config.lsk3dnet_normal_proj_h;
        seg_cfg.lsk3dnet_normal_proj_w = config.lsk3dnet_normal_proj_w;
        seg_cfg.fov_up = config.fov_up;
        seg_cfg.fov_down = config.fov_down;
        seg_cfg.img_w = config.img_w;
        seg_cfg.img_h = config.img_h;
        seg_cfg.input_channels = config.input_channels;
        seg_cfg.num_classes = config.num_classes;
        seg_cfg.tree_class_id = config.tree_class_id;
        seg_cfg.input_mean = config.input_mean;
        seg_cfg.input_std = config.input_std;
        seg_cfg.do_destagger = config.do_destagger;
        segmentor_ = SemanticSegmentorFactory::Create(seg_cfg);

        instance_detector_ = std::make_unique<Instance>();
        Instance::Params params;
        params.beam_cluster_threshold = config.beam_cluster_threshold;
        params.max_dist_to_centroid = config.max_dist_to_centroid;
        params.min_vertex_size = config.min_vertex_size;
        params.min_landmark_size = config.min_landmark_size;
        params.min_landmark_height = config.min_landmark_height;
        params.min_cluster_points = std::max(1, config.diag_trellis_min_cluster_points);
        params.min_tree_vertices = std::max(1, config.diag_trellis_min_tree_vertices);
        instance_detector_->set_params(params);
        
        RCLCPP_INFO(rclcpp::get_logger("automap_system"), 
            "[SEMANTIC][Processor][INIT] step=ok backend=%s model_path=%s fov=[%.1f,%.1f] img=%dx%d input_channels=%d num_classes=%d tree_class_id=%d",
            segmentor_->name(), config.model_path.c_str(), config.fov_up, config.fov_down, config.img_w, config.img_h,
            config.input_channels, config.num_classes, config.tree_class_id);
        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][INIT_DIAG] cluster(thr=%.3f max_dist=%.3f min_vertex=%d min_landmark_size=%d min_height=%.2f profile=%s input_mode=%s) trellis(min_cluster_points=%d min_tree_vertices=%d) diag(detailed=%d class_hist=%d topk=%d interval=%d override_tree_class_id=%d)",
            config_.beam_cluster_threshold, config_.max_dist_to_centroid, config_.min_vertex_size,
            config_.min_landmark_size, config_.min_landmark_height, config_.diag_cluster_profile.c_str(),
            config_.diag_cluster_input_mode.c_str(),
            params.min_cluster_points, params.min_tree_vertices,
            config_.diag_enable_detailed_stats ? 1 : 0, config_.diag_log_class_histogram ? 1 : 0,
            config_.diag_class_hist_top_k, config_.diag_class_hist_interval_frames, config_.diag_override_tree_class_id);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"), 
            "[SEMANTIC][Processor][INIT] step=FAILED model_path=%s error=%s → abort startup",
            config.model_path.c_str(), e.what());
        throw std::runtime_error(std::string("SemanticProcessor init failed: ") + e.what());
    }
}

SemanticProcessor::~SemanticProcessor() = default;

std::vector<CylinderLandmark::Ptr> SemanticProcessor::process(const CloudXYZIConstPtr& cloud) {
    std::vector<CylinderLandmark::Ptr> landmarks;
    if (runtime_disabled_.load(std::memory_order_relaxed)) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=runtime_disabled_after_repeated_exceptions");
        return landmarks;
    }
    if (!segmentor_ || !segmentor_->isReady()) {
        RCLCPP_WARN(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=segmentor_unavailable (feature disabled)");
        return landmarks;
    }
    if (!cloud || cloud->empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=skip reason=cloud_null_or_empty");
        return landmarks;
    }

    std::lock_guard<std::recursive_mutex> lock(resource_mutex_);

    try {
        const auto t0 = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=input_snapshot cloud_size=%zu cloud_wh=%ux%u expected_mask_wh=%dx%d",
            cloud->size(), cloud->width, cloud->height, config_.img_w, config_.img_h);
        if (config_.diag_enable_detailed_stats) {
            double x_min = std::numeric_limits<double>::infinity();
            double x_max = -std::numeric_limits<double>::infinity();
            double y_min = std::numeric_limits<double>::infinity();
            double y_max = -std::numeric_limits<double>::infinity();
            double z_min = std::numeric_limits<double>::infinity();
            double z_max = -std::numeric_limits<double>::infinity();
            for (const auto& p : cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                x_min = std::min(x_min, static_cast<double>(p.x));
                x_max = std::max(x_max, static_cast<double>(p.x));
                y_min = std::min(y_min, static_cast<double>(p.y));
                y_max = std::max(y_max, static_cast<double>(p.y));
                z_min = std::min(z_min, static_cast<double>(p.z));
                z_max = std::max(z_max, static_cast<double>(p.z));
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INPUT_DIAG] raw_pts=%zu raw_wh=%ux%u xyz_range=[x:(%.2f,%.2f) y:(%.2f,%.2f) z:(%.2f,%.2f)]",
                cloud->size(), cloud->width, cloud->height, x_min, x_max, y_min, y_max, z_min, z_max);
        }

        // 🏛️ [坐标系对齐] sloam_rec 内部使用 yaw = -atan2(y, x)
        // 标准 ROS 使用 atan2(y, x)。为了对齐，我们将 y 轴取反后再送入投影。
        CloudXYZIPtr flipped_cloud(new CloudXYZI());
        flipped_cloud->header = cloud->header;
        flipped_cloud->points.reserve(cloud->size());
        size_t dropped_invalid_points = 0;
        for (const auto& p : cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity)) {
                ++dropped_invalid_points;
                continue;
            }
            const float r2 = p.x * p.x + p.y * p.y + p.z * p.z;
            if (!(r2 > 1e-12f)) {
                ++dropped_invalid_points;
                continue;
            }
            auto p_f = p;
            p_f.y = -p.y;
            flipped_cloud->push_back(p_f);
        }
        flipped_cloud->width = static_cast<uint32_t>(flipped_cloud->points.size());
        flipped_cloud->height = 1;
        if (dropped_invalid_points > 0) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=input_filter dropped_invalid=%zu kept=%zu",
                dropped_invalid_points, flipped_cloud->size());
        }
        if (config_.diag_enable_detailed_stats) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][INPUT_FILTER_DIAG] kept=%zu dropped=%zu keep_ratio=%.2f%%",
                flipped_cloud->size(),
                dropped_invalid_points,
                100.0 * static_cast<double>(flipped_cloud->size()) /
                    static_cast<double>(std::max<size_t>(1, cloud->size())));
        }
        if (flipped_cloud->empty()) {
            RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=skip reason=all_points_invalid_after_filter raw_size=%zu",
                cloud->size());
            return landmarks;
        }
        const auto t1 = std::chrono::steady_clock::now();

        // 1. Semantic Segmentation — mask 必须为模型输出分辨率 (_img_h x _img_w)，
        // 不可用 cloud->width/height（无序点云常为 N×1）；否则 _mask 会 memcpy H*W 字节导致堆溢出。
        cv::Mat mask = cv::Mat::zeros(config_.img_h, config_.img_w, CV_8U);
        SemanticSegResult seg_result;
        segmentor_->run(flipped_cloud, mask, &seg_result);
        const auto t2 = std::chrono::steady_clock::now();
        const auto processed = processed_frames_.load(std::memory_order_relaxed) + 1;

        // High-signal segmentation log: first few frames + periodic + always on failure
        if (!seg_result.success || processed <= 5 || (processed % 50 == 0)) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][SEG] frame=%lu backend=%s success=%d infer_ms=%.2f msg=%s mask_wh=%dx%d cloud_pts=%zu",
                static_cast<unsigned long>(processed),
                seg_result.backend_name.empty() ? "(unknown)" : seg_result.backend_name.c_str(),
                seg_result.success ? 1 : 0,
                seg_result.inference_ms,
                seg_result.message.empty() ? "(none)" : seg_result.message.c_str(),
                mask.cols, mask.rows,
                flipped_cloud->size());
        }

        const bool log_class_hist = config_.diag_log_class_histogram;
        const bool dump_all_classes = config_.diag_dump_all_classes;
        if ((log_class_hist || dump_all_classes) &&
            (processed <= 10 || (processed % static_cast<uint64_t>(config_.diag_class_hist_interval_frames) == 0))) {
            std::array<uint64_t, 256> hist{};
            for (int r = 0; r < mask.rows; ++r) {
                const auto* row = mask.ptr<uint8_t>(r);
                for (int c = 0; c < mask.cols; ++c) {
                    ++hist[row[c]];
                }
            }
            if (log_class_hist) {
                const int topk = std::max(1, std::min(config_.diag_class_hist_top_k, 32));
                std::ostringstream os;
                std::array<bool, 256> used{};
                for (int i = 0; i < topk; ++i) {
                    int best_idx = -1;
                    uint64_t best_val = 0;
                    for (int k = 0; k < 256; ++k) {
                        if (used[k]) continue;
                        if (hist[k] > best_val) {
                            best_val = hist[k];
                            best_idx = k;
                        }
                    }
                    if (best_idx < 0 || best_val == 0) break;
                    used[best_idx] = true;
                    if (i > 0) os << ", ";
                    os << best_idx << ":" << best_val;
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][MASK_HIST] frame=%lu topk=%s non_zero_classes=%d class255=%lu override_tree_class_id=%d cfg_tree_class_id=%d",
                    static_cast<unsigned long>(processed), os.str().c_str(),
                    static_cast<int>(std::count_if(hist.begin(), hist.end(), [](uint64_t v) { return v > 0; })),
                    static_cast<unsigned long>(hist[255]),
                    config_.diag_override_tree_class_id, config_.tree_class_id);
            }
            if (config_.diag_enable_detailed_stats) {
                const uint64_t total_px = static_cast<uint64_t>(mask.rows) * static_cast<uint64_t>(mask.cols);
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][MASK_DIAG] frame=%lu mask_wh=%dx%d class255_ratio=%.3f%%",
                    static_cast<unsigned long>(processed), mask.cols, mask.rows,
                    100.0 * static_cast<double>(hist[255]) / static_cast<double>(std::max<uint64_t>(1, total_px)));
            }

            if (dump_all_classes) {
                struct ClassPointStats {
                    uint64_t points = 0;
                    double z_min = std::numeric_limits<double>::infinity();
                    double z_max = -std::numeric_limits<double>::infinity();
                    double z_sum = 0.0;
                    std::vector<double> ranges;
                    std::vector<pcl::PointXYZI> samples;
                };
                std::unordered_map<int, ClassPointStats> class_point_stats;
                class_point_stats.reserve(64);

                const double fov_up_rad = static_cast<double>(config_.fov_up) * M_PI / 180.0;
                const double fov_down_rad = static_cast<double>(config_.fov_down) * M_PI / 180.0;
                const double fov_rad = std::abs(fov_up_rad) + std::abs(fov_down_rad);
                const int sample_limit = config_.diag_dump_points_per_class_limit;
                const bool dump_all_points = (sample_limit < 0);

                for (const auto& p : flipped_cloud->points) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                    const double range = std::sqrt(static_cast<double>(p.x) * p.x +
                                                   static_cast<double>(p.y) * p.y +
                                                   static_cast<double>(p.z) * p.z);
                    if (!(range > 1e-6)) continue;

                    const double yaw = -std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
                    const double sin_pitch = std::clamp(static_cast<double>(p.z) / range, -1.0, 1.0);
                    const double pitch = std::asin(sin_pitch);

                    int proj_x = static_cast<int>(std::floor(0.5 * (yaw / M_PI + 1.0) * static_cast<double>(config_.img_w)));
                    int proj_y = static_cast<int>(std::floor((1.0 - (pitch + std::abs(fov_down_rad)) / std::max(1e-6, fov_rad)) *
                                                             static_cast<double>(config_.img_h)));
                    proj_x = std::clamp(proj_x, 0, config_.img_w - 1);
                    proj_y = std::clamp(proj_y, 0, config_.img_h - 1);

                    const int cls = static_cast<int>(mask.at<uint8_t>(proj_y, proj_x));
                    auto& stats = class_point_stats[cls];
                    ++stats.points;
                    stats.z_min = std::min(stats.z_min, static_cast<double>(p.z));
                    stats.z_max = std::max(stats.z_max, static_cast<double>(p.z));
                    stats.z_sum += static_cast<double>(p.z);
                    stats.ranges.push_back(range);
                    if (dump_all_points ||
                        (sample_limit > 0 && static_cast<int>(stats.samples.size()) < sample_limit)) {
                        stats.samples.push_back(p);
                    }
                }

                std::ostringstream classes_os;
                bool first = true;
                for (int cls = 0; cls < 256; ++cls) {
                    if (hist[cls] == 0) continue;
                    if (!first) classes_os << ", ";
                    first = false;
                    const auto it = class_point_stats.find(cls);
                    const uint64_t pts = (it == class_point_stats.end()) ? 0 : it->second.points;
                    classes_os << cls << "(px=" << hist[cls] << ",pts=" << pts << ")";
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CLASS_DUMP] frame=%lu classes=%s",
                    static_cast<unsigned long>(processed),
                    classes_os.str().c_str());

                const uint64_t total_classified_points = [&class_point_stats]() {
                    uint64_t sum = 0;
                    for (const auto& kv : class_point_stats) sum += kv.second.points;
                    return sum;
                }();
                auto get_quantile = [](std::vector<double> values, double q) -> double {
                    if (values.empty()) return std::numeric_limits<double>::quiet_NaN();
                    q = std::clamp(q, 0.0, 1.0);
                    const size_t idx = static_cast<size_t>(std::floor(q * static_cast<double>(values.size() - 1)));
                    std::nth_element(values.begin(), values.begin() + static_cast<long>(idx), values.end());
                    return values[idx];
                };
                for (const auto& kv : class_point_stats) {
                    const int cls = kv.first;
                    const auto& s = kv.second;
                    if (s.points == 0) continue;
                    const double z_mean = s.z_sum / static_cast<double>(s.points);
                    const double ratio = 100.0 * static_cast<double>(s.points) /
                                         static_cast<double>(std::max<uint64_t>(1, total_classified_points));
                    const double r_p25 = get_quantile(s.ranges, 0.25);
                    const double r_p50 = get_quantile(s.ranges, 0.50);
                    const double r_p75 = get_quantile(s.ranges, 0.75);
                    const double r_p95 = get_quantile(s.ranges, 0.95);
                    RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                        "[SEMANTIC][Processor][CLASS_SUMMARY] frame=%lu class=%d pts=%lu ratio=%.2f%% z[min/mean/max]=[%.2f/%.2f/%.2f] r[p25/p50/p75/p95]=[%.2f/%.2f/%.2f/%.2f]",
                        static_cast<unsigned long>(processed),
                        cls,
                        static_cast<unsigned long>(s.points),
                        ratio,
                        s.z_min, z_mean, s.z_max,
                        r_p25, r_p50, r_p75, r_p95);
                }

                if (sample_limit != 0) {
                    for (const auto& kv : class_point_stats) {
                        if (kv.second.samples.empty()) continue;
                        std::ostringstream sample_os;
                        for (size_t i = 0; i < kv.second.samples.size(); ++i) {
                            const auto& sp = kv.second.samples[i];
                            if (i > 0) sample_os << " ";
                            sample_os << "("
                                      << std::fixed << std::setprecision(2)
                                      << sp.x << "," << sp.y << "," << sp.z << ")";
                        }
                        RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                            "[SEMANTIC][Processor][CLASS_POINTS] frame=%lu class=%d samples=%s",
                            static_cast<unsigned long>(processed), kv.first, sample_os.str().c_str());
                    }
                }
            }
        }

        // 2. Mask Tree Points
        CloudXYZIPtr tree_cloud(new CloudXYZI());
        const int tree_label = resolveTreeLabel(config_);
        const bool dense_for_clustering = (config_.diag_cluster_input_mode == "dense_for_clustering");
        segmentor_->maskCloud(flipped_cloud, mask, tree_cloud, tree_label, dense_for_clustering);
        const auto t3 = std::chrono::steady_clock::now();
        ++processed_frames_;
        if (config_.diag_enable_detailed_stats || dense_for_clustering) {
            const bool organized = (tree_cloud->height > 1 && tree_cloud->width > 1 &&
                                    tree_cloud->size() == static_cast<size_t>(tree_cloud->height) * static_cast<size_t>(tree_cloud->width));
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][CLUSTER_INPUT] frame=%lu mode=%s tree_pts=%zu wh=%ux%u organized=%d",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                dense_for_clustering ? "dense_for_clustering" : "sparse",
                tree_cloud->size(),
                tree_cloud->width, tree_cloud->height,
                organized ? 1 : 0);
            if (dense_for_clustering && !organized) {
                RCLCPP_WARN(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][CLUSTER_INPUT] dense_for_clustering requested but tree_cloud is still non-organized (likely DENSE_LAYOUT_MISMATCH in segmentator)");
            }
            double t_x_min = std::numeric_limits<double>::infinity();
            double t_x_max = -std::numeric_limits<double>::infinity();
            double t_y_min = std::numeric_limits<double>::infinity();
            double t_y_max = -std::numeric_limits<double>::infinity();
            double t_z_min = std::numeric_limits<double>::infinity();
            double t_z_max = -std::numeric_limits<double>::infinity();
            for (const auto& p : tree_cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                t_x_min = std::min(t_x_min, static_cast<double>(p.x));
                t_x_max = std::max(t_x_max, static_cast<double>(p.x));
                t_y_min = std::min(t_y_min, static_cast<double>(p.y));
                t_y_max = std::max(t_y_max, static_cast<double>(p.y));
                t_z_min = std::min(t_z_min, static_cast<double>(p.z));
                t_z_max = std::max(t_z_max, static_cast<double>(p.z));
            }
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][TREE_CLOUD_DIAG] frame=%lu pts=%zu xyz_range=[x:(%.2f,%.2f) y:(%.2f,%.2f) z:(%.2f,%.2f)]",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                tree_cloud->size(),
                t_x_min, t_x_max, t_y_min, t_y_max, t_z_min, t_z_max);
        }

        if (tree_cloud->empty()) {
            ++empty_tree_frames_;
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=mask_tree pts=%zu tree_pts=0 → no tree points",
                cloud->size());
            return landmarks;
        }

        // 3. Instance Segmentation (Clustering)
        std::vector<std::vector<TreeVertex>> tree_clusters;
        instance_detector_->computeGraph(flipped_cloud, tree_cloud, tree_clusters);
        const auto t4 = std::chrono::steady_clock::now();

        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=instance tree_pts=%zu clusters=%zu",
            tree_cloud->size(), tree_clusters.size());
        if (config_.diag_enable_detailed_stats && !tree_clusters.empty()) {
            size_t min_cluster_pts = std::numeric_limits<size_t>::max();
            size_t max_cluster_pts = 0;
            size_t sum_cluster_pts = 0;
            std::ostringstream top_clusters;
            for (size_t i = 0; i < tree_clusters.size(); ++i) {
                size_t pts = 0;
                for (const auto& v : tree_clusters[i]) pts += v.points.size();
                min_cluster_pts = std::min(min_cluster_pts, pts);
                max_cluster_pts = std::max(max_cluster_pts, pts);
                sum_cluster_pts += pts;
                if (i < 10) {
                    if (i > 0) top_clusters << ", ";
                    top_clusters << "#" << i << "(v=" << tree_clusters[i].size() << ",pts=" << pts << ")";
                }
            }
            const double avg_cluster_pts = static_cast<double>(sum_cluster_pts) /
                                           static_cast<double>(std::max<size_t>(1, tree_clusters.size()));
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][CLUSTER_DIAG] frame=%lu clusters=%zu pts[min/avg/max]=[%zu/%.1f/%zu] top=%s",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                tree_clusters.size(),
                min_cluster_pts == std::numeric_limits<size_t>::max() ? 0 : min_cluster_pts,
                avg_cluster_pts,
                max_cluster_pts,
                top_clusters.str().c_str());
        }
        if (tree_clusters.empty()) {
            ++empty_cluster_frames_;
            if (config_.diag_enable_detailed_stats) {
                double z_min = std::numeric_limits<double>::infinity();
                double z_max = -std::numeric_limits<double>::infinity();
                for (const auto& p : tree_cloud->points) {
                    z_min = std::min(z_min, static_cast<double>(p.z));
                    z_max = std::max(z_max, static_cast<double>(p.z));
                }
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][EMPTY_CLUSTER_DIAG] frame=%lu tree_pts=%zu z_span=%.3f params(thr=%.3f max_dist=%.3f min_vertex=%d min_landmark_size=%d min_height=%.2f profile=%s tree_label=%d)",
                    static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                    tree_cloud->size(),
                    (std::isfinite(z_min) && std::isfinite(z_max)) ? (z_max - z_min) : -1.0,
                    config_.beam_cluster_threshold,
                    config_.max_dist_to_centroid,
                    config_.min_vertex_size,
                    config_.min_landmark_size,
                    config_.min_landmark_height,
                    config_.diag_cluster_profile.c_str(),
                    tree_label);
            }
        }

        // 4. Cylinder Fitting for each cluster
        size_t fit_input_clusters = 0;
        size_t fit_small_cluster_skips = 0;
        size_t fit_success = 0;
        for (const auto& cluster : tree_clusters) {
            std::vector<PointT> points;
            for (const auto& v : cluster) {
                for (const auto& p : v.points) {
                    points.push_back(p);
                }
            }
            ++fit_input_clusters;

            if (points.size() < 10) {
                ++fit_small_cluster_skips;
                continue;
            }

            auto landmark = fitCylinder(points);
            if (landmark) {
                // 🏛️ [坐标系对齐] 将拟合结果从翻转坐标系转回 body 系
                landmark->root.y() = -landmark->root.y();
                landmark->ray.y() = -landmark->ray.y();
                landmarks.push_back(landmark);
                ++fit_success;
            }
        }
        const auto t5 = std::chrono::steady_clock::now();
        if (config_.diag_enable_detailed_stats) {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][FIT_DIAG] frame=%lu fit_input_clusters=%zu small_cluster_skip=%zu fit_success=%zu fit_fail=%zu",
                static_cast<unsigned long>(processed_frames_.load(std::memory_order_relaxed)),
                fit_input_clusters,
                fit_small_cluster_skips,
                fit_success,
                fit_input_clusters >= fit_small_cluster_skips + fit_success
                    ? (fit_input_clusters - fit_small_cluster_skips - fit_success)
                    : 0);
        }

        if (landmarks.empty()) {
            ++empty_fit_frames_;
            RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=done clusters=%zu fitted=0 (all rejected)",
                tree_clusters.size());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=done clusters=%zu fitted=%zu",
                tree_clusters.size(), landmarks.size());
        }
        const auto processed_total = processed_frames_.load();
        const auto now = std::chrono::steady_clock::now();
        const auto secs_since_stats = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log_tp_).count();
        if (secs_since_stats >= 10) {
            last_stats_log_tp_ = now;
            const auto empty_tree = empty_tree_frames_.load();
            const auto empty_cluster = empty_cluster_frames_.load();
            const auto empty_fit = empty_fit_frames_.load();
            const double empty_tree_ratio = processed_total > 0 ? (100.0 * static_cast<double>(empty_tree) / static_cast<double>(processed_total)) : 0.0;
            const double empty_cluster_ratio = processed_total > 0 ? (100.0 * static_cast<double>(empty_cluster) / static_cast<double>(processed_total)) : 0.0;
            const double empty_fit_ratio = processed_total > 0 ? (100.0 * static_cast<double>(empty_fit) / static_cast<double>(processed_total)) : 0.0;
            const auto input_filter_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const auto seg_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
            const auto mask_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
            const auto cluster_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
            const auto fit_ms = std::chrono::duration<double, std::milli>(t5 - t4).count();
            RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][STATS] processed=%lu empty_tree=%lu(%.1f%%) empty_cluster=%lu(%.1f%%) empty_fit=%lu(%.1f%%) "
                "latest(tree_pts=%zu clusters=%zu fitted=%zu) stage_ms(filter=%.1f seg=%.1f mask=%.1f cluster=%.1f fit=%.1f)",
                static_cast<unsigned long>(processed_total),
                static_cast<unsigned long>(empty_tree), empty_tree_ratio,
                static_cast<unsigned long>(empty_cluster), empty_cluster_ratio,
                static_cast<unsigned long>(empty_fit), empty_fit_ratio,
                tree_cloud->size(), tree_clusters.size(), landmarks.size(),
                input_filter_ms, seg_ms, mask_ms, cluster_ms, fit_ms);
            if (config_.diag_enable_detailed_stats) {
                const double total_ms = input_filter_ms + seg_ms + mask_ms + cluster_ms + fit_ms;
                RCLCPP_INFO(rclcpp::get_logger("automap_system"),
                    "[SEMANTIC][Processor][STAGE_RATIO] total=%.1fms filter=%.1f%% seg=%.1f%% mask=%.1f%% cluster=%.1f%% fit=%.1f%%",
                    total_ms,
                    100.0 * input_filter_ms / std::max(1e-6, total_ms),
                    100.0 * seg_ms / std::max(1e-6, total_ms),
                    100.0 * mask_ms / std::max(1e-6, total_ms),
                    100.0 * cluster_ms / std::max(1e-6, total_ms),
                    100.0 * fit_ms / std::max(1e-6, total_ms));
            }
        }
        consecutive_failures_.store(0, std::memory_order_relaxed);
    } catch (const std::exception& e) {
        const size_t failures = consecutive_failures_.fetch_add(1, std::memory_order_relaxed) + 1;
        RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][process] step=EXCEPTION error=%s cloud_size=%zu cloud_wh=%ux%u cfg(mask_wh=%dx%d input_ch=%d classes=%d tree_class=%d) consecutive_failures=%zu",
            e.what(), cloud ? cloud->size() : 0, cloud ? cloud->width : 0, cloud ? cloud->height : 0,
            config_.img_w, config_.img_h, config_.input_channels, config_.num_classes, config_.tree_class_id, failures);
        if (failures >= kMaxConsecutiveFailures) {
            runtime_disabled_.store(true, std::memory_order_relaxed);
            RCLCPP_ERROR(rclcpp::get_logger("automap_system"),
                "[SEMANTIC][Processor][process] step=DISABLED reason=too_many_exceptions threshold=%zu",
                kMaxConsecutiveFailures);
        }
    }

    return landmarks;
}

bool SemanticProcessor::hasRuntimeCapability() const {
    return segmentor_ != nullptr && segmentor_->isReady() && !runtime_disabled_.load(std::memory_order_relaxed);
}

CylinderLandmark::Ptr SemanticProcessor::fitCylinder(const std::vector<pcl::PointXYZI>& points) {
    // 1. Initial Guess using RANSAC line fitting
    CloudXYZIPtr tree(new CloudXYZI());
    for (const auto& p : points) tree->push_back(p);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(tree);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=ransac_reject pts=%zu inliers=0", points.size());
        return nullptr;
    }

    double root[3] = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
    double ray[3] = {coefficients->values[3], coefficients->values[4], coefficients->values[5]};
    
    // Initial radius estimate: median distance to axis
    std::vector<double> distances;
    Eigen::Vector3d r(root[0], root[1], root[2]);
    Eigen::Vector3d v(ray[0], ray[1], ray[2]);
    v.normalize();
    
    for (const auto& p : points) {
        Eigen::Vector3d pt(p.x, p.y, p.z);
        double d = (pt - r).cross(v).norm();
        distances.push_back(d);
    }
    std::sort(distances.begin(), distances.end());
    double radius = distances[distances.size() / 2];

    // 2. Ceres Optimization
    ceres::Problem problem;
    for (const auto& p : points) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CylinderFittingCost, 1, 3, 3, 1>(
                new CylinderFittingCost(Eigen::Vector3d(p.x, p.y, p.z))),
            nullptr,
            root, ray, &radius);
    }

    problem.SetManifold(ray, new ceres::SphereManifold<3>());

    // 约束半径不为负
    problem.SetParameterLowerBound(&radius, 0, 0.005);
    problem.SetParameterUpperBound(&radius, 0, config_.max_tree_radius);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (summary.termination_type != ceres::CONVERGENCE && summary.termination_type != ceres::USER_SUCCESS) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=ceres_fail pts=%zu term=%d iters=%d",
            points.size(), static_cast<int>(summary.termination_type), summary.iterations.size());
        return nullptr;
    }

    auto landmark = std::make_shared<CylinderLandmark>();
    landmark->root = Eigen::Vector3d(root[0], root[1], root[2]);
    landmark->ray = Eigen::Vector3d(ray[0], ray[1], ray[2]).normalized();
    landmark->radius = radius;
    landmark->confidence = static_cast<double>(inliers->indices.size()) / points.size();

    // Sanity checks
    if (landmark->radius > config_.max_tree_radius || landmark->radius < 0.01) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=reject_radius radius=%.3f max=%.2f",
            landmark->radius, config_.max_tree_radius);
        return nullptr;
    }

    // Check axis orientation (tilt from vertical)
    double tilt = std::acos(std::abs(landmark->ray.dot(Eigen::Vector3d::UnitZ()))) * 180.0 / M_PI;
    if (tilt > config_.max_axis_theta) {
        RCLCPP_DEBUG(rclcpp::get_logger("automap_system"),
            "[SEMANTIC][Processor][fitCylinder] step=reject_tilt tilt=%.1f deg max=%.1f",
            tilt, config_.max_axis_theta);
        return nullptr;
    }

    return landmark;
}

} // namespace automap_pro::v3
