#include "automap_pro/v3/semantic_module.h"
#include "automap_pro/core/config_manager.h"
#include "automap_pro/core/metrics.h"
#include "automap_pro/core/path_resolver.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <thread>
#include <vector>

#include <pcl/common/transforms.h>

namespace automap_pro::v3 {
namespace {
constexpr int kSemanticLogThrottleMs = 15000;
constexpr int kSemanticErrorThrottleMs = 30000;
constexpr int kSemanticStatsIntervalSec = 60;
constexpr int kSemanticDtDeltaIntervalSec = 300;

size_t dtBinIndex(const bool has_kf_ts, double dt_sec) {
    if (!has_kf_ts || !std::isfinite(dt_sec) || dt_sec < 0.0) return 7;
    if (dt_sec <= 0.01) return 0;
    if (dt_sec <= 0.03) return 1;
    if (dt_sec <= 0.05) return 2;
    if (dt_sec <= 0.10) return 3;
    if (dt_sec <= 0.20) return 4;
    if (dt_sec <= 0.50) return 5;
    return 6;
}

uint64_t semanticTraceId(uint64_t kf_id, double ts) {
    const uint64_t ts_us = std::isfinite(ts) ? static_cast<uint64_t>(std::max(0.0, ts) * 1e6) : 0ull;
    return (kf_id << 20) ^ ts_us;
}

uint64_t makeEventId(double ts, uint64_t seq) {
    const uint64_t ts_us = std::isfinite(ts) ? static_cast<uint64_t>(std::max(0.0, ts) * 1e6) : 0ull;
    return (seq << 20) ^ ts_us;
}

std::string findFirstByExtensionRecursively(const std::string& root, const std::string& ext) {
    if (root.empty() || !std::filesystem::is_directory(root)) return {};
    try {
        for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
            if (!entry.is_regular_file()) continue;
            if (entry.path().extension() == ext) {
                return entry.path().string();
            }
        }
    } catch (const std::exception&) {
    }
    return {};
}

bool tryLoadLskDatasetVolumeBounds(
    const std::string& yaml_path,
    float* min_xyz,
    float* max_xyz,
    std::string* err_out) {
    if (yaml_path.empty() || min_xyz == nullptr || max_xyz == nullptr) {
        if (err_out) *err_out = "empty path or null output";
        return false;
    }
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node ds = root["dataset_params"];
        if (!ds || !ds.IsMap()) {
            if (err_out) *err_out = "missing dataset_params map";
            return false;
        }
        const YAML::Node mx = ds["max_volume_space"];
        const YAML::Node mn = ds["min_volume_space"];
        if (!mx || !mx.IsSequence() || mx.size() < 3 || !mn || !mn.IsSequence() || mn.size() < 3) {
            if (err_out) *err_out = "max_volume_space/min_volume_space must be length-3 sequences";
            return false;
        }
        max_xyz[0] = mx[0].as<float>();
        max_xyz[1] = mx[1].as<float>();
        max_xyz[2] = mx[2].as<float>();
        min_xyz[0] = mn[0].as<float>();
        min_xyz[1] = mn[1].as<float>();
        min_xyz[2] = mn[2].as<float>();
        return true;
    } catch (const std::exception& e) {
        if (err_out) *err_out = e.what();
        return false;
    } catch (...) {
        if (err_out) *err_out = "non-std exception parsing YAML";
        return false;
    }
}

bool shouldForceGateLog(std::atomic<uint64_t>& counter, uint64_t limit = 5) {
    uint64_t cur = counter.load(std::memory_order_relaxed);
    while (cur < limit) {
        if (counter.compare_exchange_weak(cur, cur + 1, std::memory_order_relaxed)) {
            return true;
        }
    }
    return false;
}

void appendNestedException(std::ostringstream& oss, const std::exception& e, int depth) {
    oss << "[depth=" << depth << "] " << e.what();
    try {
        std::rethrow_if_nested(e);
    } catch (const std::exception& nested) {
        oss << " | ";
        appendNestedException(oss, nested, depth + 1);
    } catch (...) {
        oss << " | [depth=" << (depth + 1) << "] <non-std nested exception>";
    }
}

std::string describeExceptionFull(const std::exception& e) {
    std::ostringstream oss;
    appendNestedException(oss, e, 0);
    return oss.str();
}

std::string describeCurrentExceptionFull() {
    try {
        throw;
    } catch (const std::exception& e) {
        return describeExceptionFull(e);
    } catch (...) {
        return "<non-std exception>";
    }
}

}  // namespace

[[noreturn]] void SemanticModule::terminateSystemOnInferenceFailure(
    const char* reason,
    size_t worker_idx,
    double ts,
    uint64_t trace_id,
    const std::string& detail) {
    RCLCPP_FATAL(node_->get_logger(),
        "[SEMANTIC][FATAL][CHAIN_ABORT] reason=%s worker=%zu ts=%.3f trace=%lu detail=%s",
        reason ? reason : "unknown",
        worker_idx,
        ts,
        static_cast<unsigned long>(trace_id),
        detail.empty() ? "(none)" : detail.c_str());
    RCLCPP_FATAL(node_->get_logger(),
        "[SEMANTIC][FATAL][CHAIN_ABORT][SNAPSHOT] running=%d runtime_ready=%d degraded=%d "
        "queue_size=%zu active_workers=%zu/%zu processed=%lu no_landmark=%lu coalesced=%lu backpressure_drop=%lu consecutive_errors=%d",
        running_.load() ? 1 : 0,
        semantic_runtime_ready_.load(std::memory_order_relaxed) ? 1 : 0,
        semantic_degraded_.load(std::memory_order_relaxed) ? 1 : 0,
        task_queue_.size(),
        active_worker_count_.load(std::memory_order_relaxed),
        worker_thread_count_,
        static_cast<unsigned long>(processed_tasks_.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(no_landmark_tasks_.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(coalesced_tasks_.load(std::memory_order_relaxed)),
        static_cast<unsigned long>(backpressure_drops_.load(std::memory_order_relaxed)),
        consecutive_errors_.load(std::memory_order_relaxed));
    SetSemanticHybridShutdownGuard(true);
    semantic_runtime_ready_.store(false, std::memory_order_relaxed);
    semantic_degraded_.store(true, std::memory_order_relaxed);
    running_ = false;
    cv_.notify_all();
    RCLCPP_FATAL(node_->get_logger(),
        "[SEMANTIC][FATAL][CHAIN_ABORT] shutdown_barrier=armed action=stop_semantic_workers_and_block_hybrid_recover");
    RCLCPP_FATAL(node_->get_logger(),
        "[SEMANTIC][FATAL][CHAIN_ABORT] policy=hard_exit_without_destructor action=_Exit(1)");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::_Exit(1);
}

void SemanticModule::enqueueTaskLocked(const SyncedFrameEvent& ev) {
    if (task_queue_.size() >= kCoalesceThreshold) {
        const double replaced_ts = task_queue_.back().timestamp;
        task_queue_.back() = ev;
        const auto coalesced = ++coalesced_tasks_;
        if ((coalesced % 50) == 0) {
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][ENQUEUE] step=coalesce_latest total=%lu replaced_ts=%.3f new_ts=%.3f queue_size=%zu",
                static_cast<unsigned long>(coalesced), replaced_ts, ev.timestamp, task_queue_.size());
        }
        return;
    }
    if (task_queue_.size() >= kMaxQueueSize) {
        const double dropped_ts = task_queue_.front().timestamp;
        task_queue_.pop_front();
        ++backpressure_drops_;
        METRICS_INCREMENT(metrics::STALE_VERSION_DROP_TOTAL);
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][ENQUEUE] step=backpressure_drop ts=%.3f queue_full=%zu -> dropped oldest",
            dropped_ts, kMaxQueueSize);
    }
    task_queue_.push_back(ev);
}

SemanticModule::SemanticModule(EventBus::Ptr event_bus, MapRegistry::Ptr map_registry, rclcpp::Node::SharedPtr node)
    : ModuleBase("SemanticModule", event_bus, map_registry), node_(node) {
    
    const auto& cfg = ConfigManager::instance();
    const bool semantic_enabled = cfg.semanticEnabled();
    const bool strict_mode = cfg.semanticStrictMode();

    semantic_cfg_.model_type = cfg.semanticModelType();
    semantic_cfg_.model_path = PathResolver::resolve(cfg.semanticModelPath());
    semantic_cfg_.lsk3dnet_model_path = PathResolver::resolve(cfg.semanticLsk3dnetModelPath());
    semantic_cfg_.lsk3dnet_device = cfg.semanticLsk3dnetDevice();
    semantic_cfg_.fov_up = cfg.semanticFovUp();
    semantic_cfg_.fov_down = cfg.semanticFovDown();
    semantic_cfg_.img_w = cfg.semanticImgW();
    semantic_cfg_.img_h = cfg.semanticImgH();
    semantic_cfg_.input_channels = cfg.semanticInputChannels();
    semantic_cfg_.num_classes = cfg.semanticNumClasses();
    semantic_cfg_.tree_class_id = cfg.semanticTreeClassId();
    semantic_cfg_.input_mean = cfg.semanticInputMean();
    semantic_cfg_.input_std = cfg.semanticInputStd();
    semantic_cfg_.do_destagger = cfg.semanticDoDestagger();
    semantic_cfg_.beam_cluster_threshold = cfg.semanticBeamClusterThreshold();
    semantic_cfg_.max_dist_to_centroid = cfg.semanticMaxDistToCentroid();
    semantic_cfg_.min_vertex_size = cfg.semanticMinVertexSize();
    semantic_cfg_.min_landmark_size = cfg.semanticMinLandmarkSize();
    semantic_cfg_.min_landmark_height = cfg.semanticMinLandmarkHeight();
    semantic_cfg_.tree_truncation_height = cfg.semanticTreeTruncationHeight();
    semantic_cfg_.cylinder_fit_rel_z_min = cfg.semanticCylinderFitRelZMin();
    semantic_cfg_.cylinder_fit_rel_z_max = cfg.semanticCylinderFitRelZMax();
    semantic_cfg_.cylinder_fit_ground_search_radius_m = cfg.semanticCylinderFitGroundSearchRadiusM();
    semantic_cfg_.cylinder_fit_ground_min_samples = cfg.semanticCylinderFitGroundMinSamples();
    semantic_cfg_.cylinder_fit_ground_percentile = cfg.semanticCylinderFitGroundPercentile();
    semantic_cfg_.geometric_ground_paint_class_id = cfg.semanticGeometricGroundPaintClassId();
    semantic_cfg_.geometric_only_shaft_enable = cfg.semanticGeometricOnlyShaftEnable();
    semantic_cfg_.geometric_only_shaft_xy_cell_m = cfg.semanticGeometricOnlyShaftXyCellM();
    semantic_cfg_.geometric_only_shaft_min_extent_m = cfg.semanticGeometricOnlyShaftMinExtentM();
    semantic_cfg_.geometric_only_shaft_min_points = cfg.semanticGeometricOnlyShaftMinPoints();
    semantic_cfg_.geometric_only_shaft_max_xy_spread_m = cfg.semanticGeometricOnlyShaftMaxXySpreadM();
    semantic_cfg_.geometric_only_shaft_refine_radius_m = cfg.semanticGeometricOnlyShaftRefineRadiusM();
    semantic_cfg_.geometric_only_shaft_rel_z_max_m = cfg.semanticGeometricOnlyShaftRelZMaxM();
    semantic_cfg_.geometric_only_shaft_max_candidates = cfg.semanticGeometricOnlyShaftMaxCandidates();
    semantic_cfg_.geometric_only_shaft_skip_near_line_xy_m = cfg.semanticGeometricOnlyShaftSkipNearLineXyM();
    semantic_cfg_.geometric_only_shaft_skip_near_tree_xy_m = cfg.semanticGeometricOnlyShaftSkipNearTreeXyM();
    semantic_cfg_.cylinder_radial_cv_max = cfg.semanticCylinderRadialCvMax();
    semantic_cfg_.cylinder_min_axial_extent_m = cfg.semanticCylinderMinAxialExtentM();
    semantic_cfg_.cylinder_fit_method = cfg.semanticCylinderFitMethod();
    semantic_cfg_.cylinder_pcl_sac_distance = cfg.semanticCylinderPclSacDistance();
    semantic_cfg_.cylinder_pcl_normal_radius = cfg.semanticCylinderPclNormalRadius();
    semantic_cfg_.min_tree_confidence = cfg.semanticMinTreeConfidence();
    semantic_cfg_.trunk_ego_xy_clearance_warn_m = cfg.semanticTrunkEgoXyClearanceWarnM();
    semantic_cfg_.trunk_ego_xy_clearance_reject_m = cfg.semanticTrunkEgoXyClearanceRejectM();
    semantic_cfg_.sparse_trunk_column_enable = cfg.semanticSparseTrunkColumnEnable();
    semantic_cfg_.sparse_trunk_column_radius_m = cfg.semanticSparseTrunkColumnRadiusM();
    semantic_cfg_.sparse_trunk_min_vertical_extent_m = cfg.semanticSparseTrunkMinVerticalExtentM();
    semantic_cfg_.sparse_trunk_upper_rel_z_min_m = cfg.semanticSparseTrunkUpperRelZMinM();
    semantic_cfg_.sparse_trunk_upper_rel_z_max_m = cfg.semanticSparseTrunkUpperRelZMaxM();
    semantic_cfg_.sparse_trunk_min_upper_points = cfg.semanticSparseTrunkMinUpperPoints();
    semantic_cfg_.sparse_trunk_min_column_points = cfg.semanticSparseTrunkMinColumnPoints();
    semantic_cfg_.sparse_trunk_max_fit_points = cfg.semanticSparseTrunkMaxFitPoints();
    semantic_cfg_.sparse_trunk_structural_enable = cfg.semanticSparseTrunkStructuralEnable();
    semantic_cfg_.sparse_trunk_structural_min_extent_m = cfg.semanticSparseTrunkStructuralMinExtentM();
    semantic_cfg_.sparse_trunk_structural_min_column_points = cfg.semanticSparseTrunkStructuralMinColumnPoints();
    semantic_cfg_.sparse_trunk_structural_min_foliage_points = cfg.semanticSparseTrunkStructuralMinFoliagePoints();
    semantic_cfg_.sparse_trunk_structural_foliage_radius_m = cfg.semanticSparseTrunkStructuralFoliageRadiusM();
    semantic_cfg_.sparse_trunk_structural_foliage_rel_z_min_m = cfg.semanticSparseTrunkStructuralFoliageRelZMinM();
    semantic_cfg_.sparse_trunk_structural_foliage_rel_z_max_m = cfg.semanticSparseTrunkStructuralFoliageRelZMaxM();
    semantic_cfg_.sparse_trunk_structural_taper_min_ratio = cfg.semanticSparseTrunkStructuralTaperMinRatio();
    semantic_cfg_.sparse_trunk_structural_taper_min_column_pts = cfg.semanticSparseTrunkStructuralTaperMinColumnPts();
    semantic_cfg_.sparse_trunk_structural_taper_max_rel_z_m = cfg.semanticSparseTrunkStructuralTaperMaxRelZM();
    semantic_cfg_.sparse_trunk_structural_merge_foliage_max_points = cfg.semanticSparseTrunkStructuralMergeFoliageMaxPoints();
    semantic_cfg_.sparse_trunk_structural_direct_cylinder = cfg.semanticSparseTrunkStructuralDirectCylinder();
    semantic_cfg_.sparse_trunk_structural_ambient_check_enable = cfg.semanticSparseTrunkStructuralAmbientCheckEnable();
    semantic_cfg_.sparse_trunk_structural_ambient_inner_margin_m = cfg.semanticSparseTrunkStructuralAmbientInnerMarginM();
    semantic_cfg_.sparse_trunk_structural_ambient_outer_radius_m = cfg.semanticSparseTrunkStructuralAmbientOuterRadiusM();
    semantic_cfg_.sparse_trunk_structural_ambient_max_points = cfg.semanticSparseTrunkStructuralAmbientMaxPoints();
    semantic_cfg_.sparse_trunk_structural_ambient_rel_z_pad_m = cfg.semanticSparseTrunkStructuralAmbientRelZPadM();
    semantic_cfg_.sparse_trunk_fallback_enable = cfg.semanticSparseTrunkFallbackEnable();
    semantic_cfg_.sparse_trunk_fallback_min_confidence = cfg.semanticSparseTrunkFallbackMinConfidence();
    semantic_cfg_.sparse_trunk_fallback_pca_after_merge = cfg.semanticSparseTrunkFallbackPcaAfterMerge();
    semantic_cfg_.sparse_trunk_fallback_r_med_slack = cfg.semanticSparseTrunkFallbackRMedSlack();
    semantic_cfg_.sparse_trunk_fallback_pca_min_up_cos = cfg.semanticSparseTrunkFallbackPcaMinUpCos();
    semantic_cfg_.sparse_trunk_fallback_pca_extra_tilt_deg = cfg.semanticSparseTrunkFallbackPcaExtraTiltDeg();
    semantic_cfg_.diag_enable_detailed_stats = cfg.semanticDiagEnableDetailedStats();
    semantic_cfg_.diag_log_class_histogram = cfg.semanticDiagLogClassHistogram();
    semantic_cfg_.diag_class_hist_top_k = cfg.semanticDiagClassHistTopK();
    semantic_cfg_.diag_class_hist_interval_frames = cfg.semanticDiagClassHistIntervalFrames();
    semantic_cfg_.diag_dump_all_classes = cfg.semanticDiagDumpAllClasses();
    semantic_cfg_.diag_trunk_chain_log = cfg.semanticDiagTrunkChainLog();
    semantic_cfg_.diag_dump_points_per_class_limit = cfg.semanticDiagDumpPointsPerClassLimit();
    semantic_cfg_.diag_override_tree_class_id = cfg.semanticDiagOverrideTreeClassId();
    semantic_cfg_.diag_cluster_profile = cfg.semanticDiagClusterProfile();
    semantic_cfg_.diag_cluster_input_mode = cfg.semanticDiagClusterInputMode();
    semantic_cfg_.diag_trellis_min_cluster_points = cfg.semanticDiagTrellisMinClusterPoints();
    semantic_cfg_.diag_trellis_min_tree_vertices = cfg.semanticDiagTrellisMinTreeVertices();

    // GeoSemantic-Fusion (Zhou22ral + Patchwork++)
    semantic_cfg_.mode = cfg.semanticMode();
    const bool semantic_run_model = (semantic_cfg_.mode != "geometric_only");
    semantic_cfg_.geometric_enabled = cfg.semanticGeometricEnabled();
    semantic_cfg_.geometric_log_level = cfg.semanticGeometricLogLevel();
    semantic_cfg_.geometric_log_detail = cfg.semanticGeometricLogDetail();
    semantic_cfg_.patchwork.sensor_height = static_cast<float>(cfg.semanticGeometricPatchworkSensorHeight());
    semantic_cfg_.patchwork.auto_sensor_height = cfg.semanticGeometricPatchworkAutoSensorHeight();
    semantic_cfg_.patchwork.auto_height_min_xy_m = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightMinXyM());
    semantic_cfg_.patchwork.auto_height_max_xy_m = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightMaxXyM());
    semantic_cfg_.patchwork.auto_height_min_samples = cfg.semanticGeometricPatchworkAutoHeightMinSamples();
    semantic_cfg_.patchwork.auto_height_percentile = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightPercentile());
    semantic_cfg_.patchwork.auto_height_clamp_min_m = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightClampMinM());
    semantic_cfg_.patchwork.auto_height_clamp_max_m = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightClampMaxM());
    semantic_cfg_.patchwork.auto_height_ema_alpha = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightEmaAlpha());
    semantic_cfg_.patchwork.auto_height_max_z_over_r = static_cast<float>(cfg.semanticGeometricPatchworkAutoHeightMaxZOverR());
    semantic_cfg_.patchwork.num_iter = cfg.semanticGeometricPatchworkNumIter();
    semantic_cfg_.patchwork.th_dist = static_cast<float>(cfg.semanticGeometricPatchworkThDist());
    semantic_cfg_.patchwork.max_range = static_cast<float>(cfg.semanticGeometricPatchworkMaxRange());
    semantic_cfg_.patchwork.use_odom_gravity = cfg.semanticGeometricPatchworkUseOdomGravity();
    semantic_cfg_.patchwork.odom_up_axis = cfg.semanticGeometricPatchworkOdomUpAxis();
    semantic_cfg_.patchwork.level_cloud_for_patchwork = cfg.semanticGeometricPatchworkLevelCloudForPatchwork();
    semantic_cfg_.wall_ransac.enabled = cfg.semanticGeometricWallRansacEnabled();
    semantic_cfg_.wall_ransac.distance_threshold = static_cast<float>(cfg.semanticGeometricWallRansacDistanceThresh());
    semantic_cfg_.wall_ransac.min_inliers = cfg.semanticGeometricWallRansacMinInliers();
    semantic_cfg_.wall_ransac.max_normal_tilt_deg = static_cast<float>(cfg.semanticGeometricWallRansacMaxNormalTiltDeg());
    semantic_cfg_.wall_ransac.line_distance_threshold = static_cast<float>(cfg.semanticGeometricLineRansacDistanceThresh());
    semantic_cfg_.wall_ransac.line_min_inliers = cfg.semanticGeometricLineRansacMinInliers();
    semantic_cfg_.wall_ransac.plane_distance_threshold = static_cast<float>(cfg.semanticGeometricPlaneRansacDistanceThresh());
    semantic_cfg_.wall_ransac.plane_min_inliers = cfg.semanticGeometricPlaneRansacMinInliers();
    semantic_cfg_.accumulator.enabled = cfg.semanticGeometricAccumulatorEnabled();
    semantic_cfg_.accumulator.max_frames = cfg.semanticGeometricAccumulatorMaxFrames();
    semantic_cfg_.accumulator.tag_intensity_with_scan_seq = cfg.semanticGeometricAccumulatorTagIntensityWithScanSeq();
    semantic_cfg_.primitive_classifier.enabled = cfg.semanticGeometricPrimitiveClassifierEnabled();
    semantic_cfg_.primitive_classifier.linearity_threshold = static_cast<float>(cfg.semanticGeometricPrimitiveClassifierLinearityThresh());
    semantic_cfg_.primitive_classifier.planarity_threshold = static_cast<float>(cfg.semanticGeometricPrimitiveClassifierPlanarityThresh());
    semantic_cfg_.euclidean_cluster.tolerance_m = static_cast<float>(cfg.semanticGeometricClusterToleranceM());
    semantic_cfg_.euclidean_cluster.min_points = cfg.semanticGeometricClusterMinPoints();
    semantic_cfg_.euclidean_cluster.max_points = cfg.semanticGeometricClusterMaxPoints();
    semantic_cfg_.primitive_roi.enabled = cfg.semanticGeometricPrimitiveRoiEnabled();
    semantic_cfg_.primitive_roi.body_xy_radius_m = static_cast<float>(cfg.semanticGeometricPrimitiveRoiBodyXyRadiusM());
    semantic_cfg_.primitive_roi.ring_min_xy_m = static_cast<float>(cfg.semanticGeometricPrimitiveRoiRingMinXyM());
    semantic_cfg_.primitive_roi.ring_max_xy_m = static_cast<float>(cfg.semanticGeometricPrimitiveRoiRingMaxXyM());
    semantic_cfg_.primitive_roi.voxel_leaf_m = static_cast<float>(cfg.semanticGeometricPrimitiveRoiVoxelLeafM());
    semantic_cfg_.range_view.enabled = cfg.semanticGeometricRangeViewEnabled();
    semantic_cfg_.range_view.mode = cfg.semanticGeometricRangeViewMode();
    semantic_cfg_.range_view.image_width = cfg.semanticGeometricRangeViewImageWidth();
    semantic_cfg_.range_view.image_height = cfg.semanticGeometricRangeViewImageHeight();
    semantic_cfg_.range_view.min_range_m = static_cast<float>(cfg.semanticGeometricRangeViewMinRangeM());
    semantic_cfg_.range_view.max_range_m = static_cast<float>(cfg.semanticGeometricRangeViewMaxRangeM());
    semantic_cfg_.range_view.elev_min_deg = static_cast<float>(cfg.semanticGeometricRangeViewElevMinDeg());
    semantic_cfg_.range_view.elev_max_deg = static_cast<float>(cfg.semanticGeometricRangeViewElevMaxDeg());
    semantic_cfg_.range_view.grad_mag_norm_thresh = static_cast<float>(cfg.semanticGeometricRangeViewGradMagNormThresh());
    semantic_cfg_.range_view.dilate_iterations = cfg.semanticGeometricRangeViewDilateIterations();
    semantic_cfg_.range_view.min_cc_pixels = cfg.semanticGeometricRangeViewMinCcPixels();
    semantic_cfg_.range_view.max_cc_pixels = cfg.semanticGeometricRangeViewMaxCcPixels();
    semantic_cfg_.range_view.wall_min_width_u = cfg.semanticGeometricRangeViewWallMinWidthU();
    semantic_cfg_.range_view.wall_max_aspect_h_over_w =
        static_cast<float>(cfg.semanticGeometricRangeViewWallMaxAspectHOverW());
    semantic_cfg_.range_view.trunk_max_width_u = cfg.semanticGeometricRangeViewTrunkMaxWidthU();
    semantic_cfg_.range_view.trunk_min_aspect_h_over_w =
        static_cast<float>(cfg.semanticGeometricRangeViewTrunkMinAspectHOverW());
    semantic_cfg_.range_view.bbox_margin_u = cfg.semanticGeometricRangeViewBboxMarginU();
    semantic_cfg_.range_view.bbox_margin_v = cfg.semanticGeometricRangeViewBboxMarginV();
    semantic_cfg_.range_view.max_patches_per_frame = cfg.semanticGeometricRangeViewMaxPatchesPerFrame();
    semantic_cfg_.range_view.max_patch_points = cfg.semanticGeometricRangeViewMaxPatchPoints();
    semantic_cfg_.range_view.fallback_full_cloud = cfg.semanticGeometricRangeViewFallbackFullCloud();
    semantic_cfg_.range_view.onnx_model_path = cfg.semanticGeometricRangeViewOnnxModelPath();
    semantic_cfg_.range_view.onnx_input_width = cfg.semanticGeometricRangeViewOnnxInputWidth();
    semantic_cfg_.range_view.onnx_input_height = cfg.semanticGeometricRangeViewOnnxInputHeight();
    semantic_cfg_.range_view.onnx_n_classes = cfg.semanticGeometricRangeViewOnnxNClasses();
    semantic_cfg_.range_view.onnx_wall_class_id = cfg.semanticGeometricRangeViewOnnxWallClassId();
    semantic_cfg_.range_view.onnx_trunk_class_id = cfg.semanticGeometricRangeViewOnnxTrunkClassId();
    semantic_cfg_.range_view.fusion_rv_boost_scale =
        static_cast<float>(cfg.semanticGeometricRangeViewFusionRvBoostScale());
    semantic_cfg_.max_lines_per_cluster = cfg.semanticGeometricMaxLinesPerCluster();
    semantic_cfg_.geometric_only_frame_merge.max_xy_m = static_cast<float>(cfg.semanticGeometricOnlyFrameMergeMaxXyM());
    semantic_cfg_.geometric_only_frame_merge.max_z_m = static_cast<float>(cfg.semanticGeometricOnlyFrameMergeMaxZM());
    semantic_cfg_.geometric_only_frame_merge.max_axis_angle_deg =
        static_cast<float>(cfg.semanticGeometricOnlyFrameMergeMaxAxisAngleDeg());

    // 🏛️ [架构演进] 使用 PathResolver 统一资源定位，消除环境耦合
    auto maybe_repo_root = PathResolver::resolve(cfg.semanticLsk3dnetRepoRoot());
    if (maybe_repo_root.empty()) {
        const auto pkg_root = PathResolver::getProjectRoot("automap_pro");
        if (!pkg_root.empty()) {
            const auto rr = (std::filesystem::path(pkg_root) / "thrid_party" / "LSK3DNet-main").string();
            if (std::filesystem::is_directory(rr)) {
                maybe_repo_root = rr;
            }
        }
    }
    semantic_cfg_.lsk3dnet_repo_root = maybe_repo_root;

    // 🏛️ [产品化加固] 自动解析 LSK3DNet 混合模式资产（geometric_only 不加载 Hybrid / Python）
    if (semantic_cfg_.model_type == "lsk3dnet_hybrid" && semantic_run_model) {
        const std::string rr = semantic_cfg_.lsk3dnet_repo_root;
        semantic_cfg_.lsk3dnet_config_yaml = PathResolver::resolve(cfg.semanticLsk3dnetConfigYaml());
        semantic_cfg_.lsk3dnet_checkpoint = PathResolver::resolve(cfg.semanticLsk3dnetCheckpoint());
        semantic_cfg_.lsk3dnet_classifier_torchscript = PathResolver::resolve(cfg.semanticLsk3dnetClassifierTorchscript());
        
        RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Resolved hybrid assets: config=%s, checkpoint=%s, classifier=%s",
                    semantic_cfg_.lsk3dnet_config_yaml.c_str(),
                    semantic_cfg_.lsk3dnet_checkpoint.c_str(),
                    semantic_cfg_.lsk3dnet_classifier_torchscript.c_str());

        // 自动导出缺失的 TorchScript 资产
        if (!semantic_cfg_.lsk3dnet_classifier_torchscript.empty() && 
            !std::filesystem::exists(semantic_cfg_.lsk3dnet_classifier_torchscript) &&
            std::filesystem::exists(semantic_cfg_.lsk3dnet_checkpoint) &&
            std::filesystem::exists(semantic_cfg_.lsk3dnet_config_yaml)) {
            
            RCLCPP_WARN(node_->get_logger(), "[SEMANTIC][Module] Missing classifier TorchScript at %s, attempting auto-export...",
                        semantic_cfg_.lsk3dnet_classifier_torchscript.c_str());
            const std::string py_exe = cfg.semanticLsk3dnetPython();
            const std::string export_script = (std::filesystem::path(rr) / "scripts" / "export_lsk3dnet_classifier_torchscript.py").string();
            
            if (!std::filesystem::exists(export_script)) {
                RCLCPP_ERROR(node_->get_logger(), "[SEMANTIC][Module] Export script NOT found at: %s", export_script.c_str());
            }

            std::string cmd = py_exe + " " + export_script + 
                             " --config " + semantic_cfg_.lsk3dnet_config_yaml +
                             " --checkpoint " + semantic_cfg_.lsk3dnet_checkpoint +
                             " --output " + semantic_cfg_.lsk3dnet_classifier_torchscript +
                             " --device cpu 2>&1";
            
            RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Executing auto-export command: %s", cmd.c_str());
            
            int ret = std::system(cmd.c_str());
            if (ret == 0) {
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Auto-export successful: %s", semantic_cfg_.lsk3dnet_classifier_torchscript.c_str());
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[SEMANTIC][Module] Auto-export FAILED (code %d). This usually means the Python environment or script has issues.", ret);
            }
        }

        // 环境覆盖支持
        const char* cls_override = std::getenv("AUTOMAP_LSK3DNET_CLASSIFIER_PATH");
        if (cls_override != nullptr && std::strlen(cls_override) > 0) {
            if (std::filesystem::exists(cls_override)) {
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Overriding classifier path from ENV: %s", cls_override);
                semantic_cfg_.lsk3dnet_classifier_torchscript = cls_override;
            }
        }
        semantic_cfg_.lsk3dnet_hybrid_normal_mode = cfg.semanticLsk3dnetHybridNormalMode();
        semantic_cfg_.lsk3dnet_normal_fov_up_deg = cfg.semanticLsk3dnetNormalFovUpDeg();
        semantic_cfg_.lsk3dnet_normal_fov_down_deg = cfg.semanticLsk3dnetNormalFovDownDeg();
        semantic_cfg_.lsk3dnet_normal_proj_h = std::max(8, std::min(4096, cfg.semanticLsk3dnetNormalProjH()));
        semantic_cfg_.lsk3dnet_normal_proj_w = std::max(8, std::min(8192, cfg.semanticLsk3dnetNormalProjW()));
        const std::string ws_cfg = cfg.semanticLsk3dnetWorkerScript();
        if (ws_cfg.empty()) {
            std::string found;
            if (!rr.empty()) {
                found = (std::filesystem::path(rr) / "scripts" / "lsk3dnet_hybrid_worker.py").string();
            }
            semantic_cfg_.lsk3dnet_worker_script = found;
        } else {
            semantic_cfg_.lsk3dnet_worker_script = PathResolver::resolve(ws_cfg);
        }
        semantic_cfg_.lsk3dnet_python_exe = cfg.semanticLsk3dnetPython();
    }

    // 🏛️ [产品化加固] 当标准 LSK3DNet 缺失但混合模式资产完备时，自动切换以保证精度
    if (semantic_run_model && semantic_cfg_.model_type == "lsk3dnet" &&
        (semantic_cfg_.lsk3dnet_model_path.empty() || !std::filesystem::exists(semantic_cfg_.lsk3dnet_model_path))) {
        
        const std::string rr = semantic_cfg_.lsk3dnet_repo_root;
        const std::string hy_cfg = PathResolver::resolve(cfg.semanticLsk3dnetConfigYaml());
        std::string hy_ckpt = PathResolver::resolve(cfg.semanticLsk3dnetCheckpoint());
        std::string hy_cls = PathResolver::resolve(cfg.semanticLsk3dnetClassifierTorchscript());
        std::string hy_ws = PathResolver::resolve(cfg.semanticLsk3dnetWorkerScript());
        
        if (hy_ws.empty() && !rr.empty()) {
            hy_ws = (std::filesystem::path(rr) / "scripts" / "lsk3dnet_hybrid_worker.py").string();
        }
        if ((hy_ckpt.empty() || !std::filesystem::exists(hy_ckpt)) && std::filesystem::is_directory(rr)) {
            hy_ckpt = findFirstByExtensionRecursively(rr, ".pt");
        }
        
        const bool hybrid_ready = std::filesystem::exists(hy_cfg) && 
                                std::filesystem::exists(hy_ckpt) && 
                                std::filesystem::exists(hy_cls) && 
                                std::filesystem::exists(hy_ws);
        
        if (hybrid_ready) {
            semantic_cfg_.model_type = "lsk3dnet_hybrid";
            semantic_cfg_.lsk3dnet_config_yaml = hy_cfg;
            semantic_cfg_.lsk3dnet_checkpoint = hy_ckpt;
            semantic_cfg_.lsk3dnet_classifier_torchscript = hy_cls;
            semantic_cfg_.lsk3dnet_worker_script = hy_ws;
            semantic_cfg_.lsk3dnet_python_exe = cfg.semanticLsk3dnetPython();
            RCLCPP_WARN(node_->get_logger(), "[SEMANTIC][Module] Auto-switch to lsk3dnet_hybrid due to missing .ts model");
        }
    }

    semantic_cfg_.lsk_num_vote = cfg.semanticLsk3dnetNumVote();
    semantic_cfg_.lsk_training_volume_crop = cfg.semanticLsk3dnetTrainingVolumeCrop();
    semantic_cfg_.lsk_volume_bounds_valid = false;

    if (cfg.semanticLsk3dnetAlignUpstreamEval()) {
        semantic_cfg_.fov_up = cfg.semanticLsk3dnetNormalFovUpDeg();
        semantic_cfg_.fov_down = cfg.semanticLsk3dnetNormalFovDownDeg();
        semantic_cfg_.img_w = std::max(8, std::min(8192, cfg.semanticLsk3dnetNormalProjW()));
        semantic_cfg_.img_h = std::max(8, std::min(4096, cfg.semanticLsk3dnetNormalProjH()));
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][UPSTREAM_ALIGN] align_upstream_eval=1: semantic.fov_* and img_* unified with "
            "normal map (mask range-image FOV matches utils.normalmap / hybrid worker)");
    }

    if (semantic_cfg_.lsk_training_volume_crop) {
        if (semantic_cfg_.model_type != "lsk3dnet_hybrid") {
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][UPSTREAM_ALIGN] training_volume_crop=1 ignored (model_type=%s; hybrid only)",
                semantic_cfg_.model_type.c_str());
        } else if (semantic_cfg_.lsk3dnet_config_yaml.empty() ||
                   !std::filesystem::exists(semantic_cfg_.lsk3dnet_config_yaml)) {
            RCLCPP_WARN(node_->get_logger(),
                "[SEMANTIC][Module][UPSTREAM_ALIGN] training_volume_crop=1 but config_yaml missing or not found: %s",
                semantic_cfg_.lsk3dnet_config_yaml.c_str());
        } else {
            std::string verr;
            float mn[3], mx[3];
            if (tryLoadLskDatasetVolumeBounds(semantic_cfg_.lsk3dnet_config_yaml, mn, mx, &verr)) {
                semantic_cfg_.lsk_vol_min_x = mn[0];
                semantic_cfg_.lsk_vol_min_y = mn[1];
                semantic_cfg_.lsk_vol_min_z = mn[2];
                semantic_cfg_.lsk_vol_max_x = mx[0];
                semantic_cfg_.lsk_vol_max_y = mx[1];
                semantic_cfg_.lsk_vol_max_z = mx[2];
                semantic_cfg_.lsk_volume_bounds_valid = true;
                RCLCPP_INFO(node_->get_logger(),
                    "[SEMANTIC][Module][UPSTREAM_ALIGN] training_volume_crop=1 from LSK yaml: "
                    "min=[%.1f,%.1f,%.1f] max=[%.1f,%.1f,%.1f]",
                    mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]);
            } else {
                RCLCPP_WARN(node_->get_logger(),
                    "[SEMANTIC][Module][UPSTREAM_ALIGN] training_volume_crop=1 parse failed: %s",
                    verr.c_str());
            }
        }
    }

    if (semantic_cfg_.lsk_num_vote > 1) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][UPSTREAM_ALIGN] num_vote=%d: deterministic xy transforms + logit sum "
            "(approximates LSK val TTA; multiplies hybrid backbone cost)",
            semantic_cfg_.lsk_num_vote);
    }

    // 配置日志
    RCLCPP_INFO(node_->get_logger(),
        "[SEMANTIC][Module][CONFIG] enabled=%d model_type=%s device=%s sloam_onnx=%s lsk_ts=%s img=%dx%d fov=[%.1f,%.1f]",
        semantic_enabled ? 1 : 0, semantic_cfg_.model_type.c_str(), semantic_cfg_.lsk3dnet_device.c_str(),
        semantic_cfg_.model_path.c_str(), semantic_cfg_.lsk3dnet_model_path.c_str(),
        semantic_cfg_.img_w, semantic_cfg_.img_h, semantic_cfg_.fov_up, semantic_cfg_.fov_down);
    if (semantic_cfg_.model_type == "lsk3dnet_hybrid" && semantic_run_model) {
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][CONFIG_AUDIT] source={semantic.* + semantic.lsk3dnet.* + env(AUTOMAP_LSK3DNET_CLASSIFIER_PATH)} "
            "effective={repo_root=%s config_yaml=%s checkpoint=%s classifier=%s python=%s worker=%s "
            "normal_mode=%s normal_profile=[up=%.1f,down=%.1f,h=%d,w=%d] sensor_fov=[up=%.1f,down=%.1f] img=[%dx%d]}",
            semantic_cfg_.lsk3dnet_repo_root.c_str(),
            semantic_cfg_.lsk3dnet_config_yaml.c_str(),
            semantic_cfg_.lsk3dnet_checkpoint.c_str(),
            semantic_cfg_.lsk3dnet_classifier_torchscript.c_str(),
            semantic_cfg_.lsk3dnet_python_exe.c_str(),
            semantic_cfg_.lsk3dnet_worker_script.c_str(),
            semantic_cfg_.lsk3dnet_hybrid_normal_mode.c_str(),
            semantic_cfg_.lsk3dnet_normal_fov_up_deg,
            semantic_cfg_.lsk3dnet_normal_fov_down_deg,
            semantic_cfg_.lsk3dnet_normal_proj_h,
            semantic_cfg_.lsk3dnet_normal_proj_w,
            semantic_cfg_.fov_up,
            semantic_cfg_.fov_down,
            semantic_cfg_.img_w,
            semantic_cfg_.img_h);
    }

    // 线程与弹性伸缩配置
    const int system_threads = std::max(1, cfg.numThreads());
    const int configured_workers = cfg.semanticWorkerThreads();
    worker_thread_count_ = (configured_workers > 0) ? static_cast<size_t>(configured_workers) 
                                                   : static_cast<size_t>(std::clamp(system_threads / 4, 1, 6));
    
    autoscale_eval_interval_s_ = cfg.semanticAutoscaleEvalIntervalS();
    autoscale_high_watermark_ = cfg.semanticAutoscaleHighWatermark();
    autoscale_low_watermark_ = cfg.semanticAutoscaleLowWatermark();
    min_worker_count_ = std::min(worker_thread_count_, static_cast<size_t>(cfg.semanticAutoscaleMinActiveWorkers()));
    active_worker_count_.store(min_worker_count_, std::memory_order_relaxed);
    RCLCPP_INFO(node_->get_logger(),
        "[SEMANTIC][Module][INIT_OBS] semantic_enabled=%d runtime_ready=%d model_type=%s worker_count=%zu",
        semantic_enabled ? 1 : 0,
        semantic_runtime_ready_.load(std::memory_order_relaxed) ? 1 : 0,
        semantic_cfg_.model_type.c_str(),
        worker_thread_count_);

    // 初始化校验 (Fail-Fast)
    bool config_error = false;
    std::string err_reason;

    const bool geometric_only_mode = (semantic_cfg_.mode == "geometric_only");

    if (semantic_enabled && geometric_only_mode && !semantic_cfg_.geometric_enabled) {
        RCLCPP_FATAL(node_->get_logger(),
            "[SEMANTIC][FATAL][GEOMETRIC_ONLY_GUARD] invalid_config: semantic.mode=geometric_only "
            "requires semantic.geometric.enabled=true, but got false. "
            "Refusing to start with a silently non-functional semantic pipeline.");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::exit(1);
    }

    if (!semantic_enabled) {
        err_reason = "disabled by config";
    } else if (semantic_run_model && semantic_cfg_.model_type == "sloam" && !std::filesystem::exists(semantic_cfg_.model_path)) {
        err_reason = "sloam model not found at: " + semantic_cfg_.model_path; config_error = true;
    } else if (semantic_run_model && semantic_cfg_.model_type == "lsk3dnet" && !std::filesystem::exists(semantic_cfg_.lsk3dnet_model_path)) {
        err_reason = "lsk3dnet model not found at: " + semantic_cfg_.lsk3dnet_model_path; config_error = true;
    } else if (semantic_run_model && semantic_cfg_.model_type == "lsk3dnet_hybrid") {
        if (!std::filesystem::exists(semantic_cfg_.lsk3dnet_checkpoint)) {
            err_reason = "lsk3dnet_hybrid checkpoint missing at: " + semantic_cfg_.lsk3dnet_checkpoint; config_error = true;
        } else if (!std::filesystem::exists(semantic_cfg_.lsk3dnet_classifier_torchscript)) {
            err_reason = "lsk3dnet_hybrid classifier TorchScript missing at: " + semantic_cfg_.lsk3dnet_classifier_torchscript; config_error = true;
        } else if (!std::filesystem::exists(semantic_cfg_.lsk3dnet_config_yaml)) {
            err_reason = "lsk3dnet_hybrid config YAML missing at: " + semantic_cfg_.lsk3dnet_config_yaml; config_error = true;
        }
    }

    if (config_error) {
        if (strict_mode) {
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][FATAL] %s (strict_mode=true)", err_reason.c_str());
            throw std::runtime_error(err_reason);
        }
        RCLCPP_ERROR(node_->get_logger(), "[SEMANTIC][INIT] %s, module will be idle", err_reason.c_str());
    } else if (semantic_enabled) {
        // 🏛️ [产品化加固] 自动推导 LSK3DNet 树木类别 ID (NuScenes=10, KITTI=15)
        if (semantic_run_model && semantic_cfg_.model_type.find("lsk3dnet") != std::string::npos &&
            semantic_cfg_.tree_class_id == -1) {
            if (semantic_cfg_.num_classes == 17) {
                semantic_cfg_.tree_class_id = 16; // NuScenes Vegetation
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Auto-set tree_class_id=16 (NuScenes Vegetation)");
            } else if (semantic_cfg_.num_classes == 19 || semantic_cfg_.num_classes == 20) {
                semantic_cfg_.tree_class_id = 16; // SemanticKITTI Trunk (learning ID 16)
                RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module] Auto-set tree_class_id=16 (KITTI Trunk)");
            }
        }

        try {
            semantic_processors_.clear();
            semantic_processors_.reserve(worker_thread_count_);
            bool all_ready = true;
            for (size_t i = 0; i < worker_thread_count_; ++i) {
                auto processor = std::make_shared<SemanticProcessor>(semantic_cfg_);
                all_ready = all_ready && processor->hasRuntimeCapability();
                semantic_processors_.push_back(processor);
            }
            semantic_runtime_ready_.store(all_ready, std::memory_order_relaxed);
        } catch (const std::exception& e) {
            semantic_runtime_ready_.store(false, std::memory_order_relaxed);
            const std::string full_err = describeExceptionFull(e);
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][Module][INIT] FATAL initialization failure: %s", e.what());
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][Module][INIT] FULL_EXCEPTION: %s", full_err.c_str());
            
            // 🏛️ [架构加固] 满足用户契约：初始化失败直接退出工程
            RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][FATAL] Terminating entire system due to semantic initialization failure as requested.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 给日志落地留时间
            std::exit(1);
        }
    }

    // Hard requirement for LSK3DNet
    if (semantic_enabled && semantic_run_model && (semantic_cfg_.model_type.find("lsk3dnet") != std::string::npos) &&
        !semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        RCLCPP_FATAL(node_->get_logger(), "[SEMANTIC][FATAL] LSK3DNet requires CUDA or specific assets, but runtime not ready. Terminating system.");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::exit(1);
    }
    // Hard requirement for geometric_only mode
    if (semantic_enabled && geometric_only_mode &&
        !semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        RCLCPP_FATAL(node_->get_logger(),
            "[SEMANTIC][FATAL][GEOMETRIC_ONLY_GUARD] geometric_only runtime unavailable. "
            "semantic.enabled=true and semantic.mode=geometric_only but geometric processor is not ready. "
            "Terminating to avoid silent semantic pipeline failure.");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::exit(1);
    }

    // 事件注册与状态发布
    const bool accept_graph_input = cfg.semanticInputAcceptGraphTask();
    const bool accept_independent_input = cfg.semanticInputAcceptIndependentEvent();
    if (accept_graph_input) {
        onEvent<GraphTaskEvent>([this](const GraphTaskEvent& ev) { handleGraphTaskEvent(ev); });
    }
    if (accept_independent_input) {
        onEvent<SemanticInputEvent>([this](const SemanticInputEvent& ev) { handleSemanticInputEvent(ev); });
    }
    onEvent<SystemQuiesceRequestEvent>([this](const SystemQuiesceRequestEvent& ev) { this->quiesce(ev.enable); });
    RCLCPP_INFO(node_->get_logger(),
        "[SEMANTIC][Module][INPUT] accept_graph_task=%d accept_independent_event=%d",
        accept_graph_input ? 1 : 0,
        accept_independent_input ? 1 : 0);

    if (semantic_runtime_ready_.load()) {
        RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][Module][INIT] step=ok workers=%zu", worker_thread_count_);
    }
}

std::vector<std::pair<std::string, size_t>> SemanticModule::queueDepths() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return {
        {"task_queue", task_queue_.size()},
        {"workers_active", active_worker_count_.load(std::memory_order_relaxed)},
        {"workers_total", worker_thread_count_},
    };
}

std::string SemanticModule::idleDetail() const {
    if (semantic_degraded_.load(std::memory_order_relaxed)) {
        return "semantic_degraded=1";
    }
    if (!semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        return "semantic_runtime_ready=0";
    }
    return "";
}

void SemanticModule::start() {
    if (running_) return;
    SetSemanticHybridShutdownGuard(false);
    ALOG_INFO("Pipeline", "[PIPELINE][V3] module START name={} semantic_workers={}", name_, worker_thread_count_);
    running_ = true;
    updateHeartbeat();
    thread_ = std::thread(&SemanticModule::run, this);
    worker_threads_.clear();
    for (size_t i = 1; i < worker_thread_count_; ++i) {
        worker_threads_.emplace_back(&SemanticModule::workerLoop, this, i);
    }
}

void SemanticModule::stop() {
    if (!running_) return;
    ALOG_INFO("Pipeline", "[PIPELINE][V3] module STOP name={} (joining semantic workers)", name_);
    running_ = false;
    cv_.notify_all();
    for (auto& t : worker_threads_) {
        if (t.joinable()) t.join();
    }
    worker_threads_.clear();
    if (thread_.joinable()) {
        thread_.join();
    }
    ALOG_INFO("Pipeline", "[PIPELINE][V3] module STOPPED name={}", name_);
}

void SemanticModule::handleKeyFrameInput(const KeyFrame::Ptr& kf, const char* source_tag) {
    if (!running_.load()) return;
    if (!kf) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B2 KF->SEM] action=reject reason=null_keyframe source=%s",
            source_tag ? source_tag : "unknown");
        return;
    }
    const bool force_gate_log = shouldForceGateLog(b2_gate_forced_log_count_);
    const bool runtime_ready = semantic_runtime_ready_.load(std::memory_order_relaxed);
    const bool degraded = semantic_degraded_.load(std::memory_order_relaxed);
    if (!runtime_ready || degraded) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B2 KF->SEM] action=reject reason=%s runtime_ready=%d degraded=%d",
            !runtime_ready ? "runtime_not_ready" : "degraded",
            runtime_ready ? 1 : 0,
            degraded ? 1 : 0);
        if (force_gate_log) {
            RCLCPP_INFO(node_->get_logger(),
                "[CHAIN][B2 KF->SEM][FORCED] action=reject reason=%s runtime_ready=%d degraded=%d queue_size=%zu forced_idx=%lu/5",
                !runtime_ready ? "runtime_not_ready" : "degraded",
                runtime_ready ? 1 : 0,
                degraded ? 1 : 0,
                task_queue_.size(),
                static_cast<unsigned long>(b2_gate_forced_log_count_.load(std::memory_order_relaxed)));
        }
        return;
    }
    if (quiescing_.load()) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B2 KF->SEM] action=reject reason=quiescing");
        if (force_gate_log) {
            RCLCPP_INFO(node_->get_logger(),
                "[CHAIN][B2 KF->SEM][FORCED] action=reject reason=quiescing queue_size=%zu forced_idx=%lu/5",
                task_queue_.size(),
                static_cast<unsigned long>(b2_gate_forced_log_count_.load(std::memory_order_relaxed)));
        }
        return;
    }
    const auto last_kf_id = last_keyframe_task_id_.load(std::memory_order_relaxed);
    if (last_kf_id != std::numeric_limits<uint64_t>::max() && kf->id <= last_kf_id) {
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B2 KF->SEM] action=skip reason=duplicate_or_stale source=%s kf_id=%lu last_kf_id=%lu",
            source_tag ? source_tag : "unknown",
            static_cast<unsigned long>(kf->id),
            static_cast<unsigned long>(last_kf_id));
        if (force_gate_log) {
            RCLCPP_INFO(node_->get_logger(),
                "[CHAIN][B2 KF->SEM][FORCED] action=skip reason=duplicate_or_stale source=%s kf_id=%lu last_kf_id=%lu forced_idx=%lu/5",
                source_tag ? source_tag : "unknown",
                static_cast<unsigned long>(kf->id),
                static_cast<unsigned long>(last_kf_id),
                static_cast<unsigned long>(b2_gate_forced_log_count_.load(std::memory_order_relaxed)));
        }
        return;
    }

    CloudXYZIPtr sem_cloud = kf->cloud_body;
    if ((!sem_cloud || sem_cloud->empty()) && kf->cloud_ds_body && !kf->cloud_ds_body->empty()) {
        sem_cloud = kf->cloud_ds_body;
    }
    if (!sem_cloud || sem_cloud->empty()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B2 KF->SEM] action=reject reason=empty_keyframe_cloud kf_id=%lu ts=%.3f reason_code=E_EMPTY_KF_CLOUD",
            static_cast<unsigned long>(kf->id), kf->timestamp);
        if (force_gate_log) {
            RCLCPP_INFO(node_->get_logger(),
                "[CHAIN][B2 KF->SEM][FORCED] action=reject reason=empty_keyframe_cloud kf_id=%lu ts=%.3f cloud_body=%zu cloud_ds_body=%zu forced_idx=%lu/5",
                static_cast<unsigned long>(kf->id),
                kf->timestamp,
                kf->cloud_body ? kf->cloud_body->size() : 0,
                kf->cloud_ds_body ? kf->cloud_ds_body->size() : 0,
                static_cast<unsigned long>(b2_gate_forced_log_count_.load(std::memory_order_relaxed)));
        }
        return;
    }

    SyncedFrameEvent sem_ev;
    sem_ev.timestamp = kf->timestamp;
    sem_ev.cloud = sem_cloud;
    sem_ev.cloud_ds = kf->cloud_ds_body;
    sem_ev.T_odom_b = kf->T_odom_b;
    sem_ev.covariance = kf->covariance;
    sem_ev.pose_frame = PoseFrame::ODOM;
    sem_ev.cloud_frame = "body";
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
        "[CHAIN][B2 KF->SEM][COORD] kf_id=%lu cloud_frame=body source=kf.cloud_body (FastLIVO path must store BODY after world->body when frontend.cloud_frame=world)",
        static_cast<unsigned long>(kf->id));
    sem_ev.kf_info = kf->livo_info;
    sem_ev.kf_info.timestamp = kf->timestamp;
    dt_hist_bins_[dtBinIndex(true, 0.0)].fetch_add(1, std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(queue_mutex_);
    enqueueTaskLocked(sem_ev);
    last_keyframe_task_id_.store(kf->id, std::memory_order_relaxed);
    gate_accept_total_.fetch_add(1, std::memory_order_relaxed);
    const uint64_t trace_id = semanticTraceId(kf->id, kf->timestamp);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
        "[CHAIN][B2 KF->SEM] action=accept source=%s trace=%lu kf_id=%lu ts=%.3f pts=%zu queue_size=%zu workers_active=%zu",
        source_tag ? source_tag : "unknown",
        static_cast<unsigned long>(trace_id),
        static_cast<unsigned long>(kf->id),
        kf->timestamp,
        sem_cloud->size(),
        task_queue_.size(),
        active_worker_count_.load(std::memory_order_relaxed));
    if (force_gate_log) {
        RCLCPP_INFO(node_->get_logger(),
            "[CHAIN][B2 KF->SEM][FORCED] action=accept source=%s trace=%lu kf_id=%lu ts=%.3f pts=%zu queue_size=%zu workers_active=%zu forced_idx=%lu/5",
            source_tag ? source_tag : "unknown",
            static_cast<unsigned long>(trace_id),
            static_cast<unsigned long>(kf->id),
            kf->timestamp,
            sem_cloud->size(),
            task_queue_.size(),
            active_worker_count_.load(std::memory_order_relaxed),
            static_cast<unsigned long>(b2_gate_forced_log_count_.load(std::memory_order_relaxed)));
    }
    cv_.notify_one();
}

void SemanticModule::handleGraphTaskEvent(const GraphTaskEvent& ev) {
    if (!running_.load()) return;
    if (ev.task.type != OptTaskItem::Type::KEYFRAME_CREATE) return;
    handleKeyFrameInput(ev.task.keyframe, "graph_task");
}

void SemanticModule::handleSemanticInputEvent(const SemanticInputEvent& ev) {
    if (!running_.load()) return;
    if (!ev.isValid()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B2 KF->SEM] action=reject reason=invalid_semantic_input_event");
        return;
    }
    handleKeyFrameInput(ev.keyframe, "semantic_input");
}

void SemanticModule::run() {
    workerLoop(0);
}

void SemanticModule::workerLoop(size_t worker_idx) {
    RCLCPP_INFO(node_->get_logger(), "[V3][SemanticModule] Started worker thread idx=%zu", worker_idx);
    while (running_) {
        try {
            if (worker_idx == 0) {
                updateHeartbeat();
            }
            tryRecoverFromDegradedState(node_->now().seconds());
            SyncedFrameEvent event;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                if (worker_idx == 0) {
                    maybeAdjustActiveWorkersLocked(node_->now().seconds());
                }
                const size_t active_workers = active_worker_count_.load(std::memory_order_relaxed);
                if (worker_idx >= active_workers) {
                    cv_.wait_for(lock, std::chrono::milliseconds(100), [this, worker_idx] {
                        return !running_ || worker_idx < active_worker_count_.load(std::memory_order_relaxed);
                    });
                    continue;
                }
                cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { 
                    return !running_ || !task_queue_.empty(); 
                });
                if (!running_) break;
                if (task_queue_.empty()) {
                    continue;
                }

                event = task_queue_.front();
                task_queue_.pop_front();
                if ((processed_tasks_.load(std::memory_order_relaxed) < 20) ||
                    ((processed_tasks_.load(std::memory_order_relaxed) % 50) == 0)) {
                    RCLCPP_INFO(node_->get_logger(),
                        "[SEMANTIC][Module][DEQUEUE] worker=%zu ts=%.3f cloud_pts=%zu queue_left=%zu active_workers=%zu",
                        worker_idx,
                        event.timestamp,
                        event.cloud ? event.cloud->size() : 0,
                        task_queue_.size(),
                        active_worker_count_.load(std::memory_order_relaxed));
                }
            }

            processTask(event, worker_idx);
        } catch (const std::exception& e) {
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
            const std::string full_err = describeExceptionFull(e);
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][TRIGGER][worker_loop_exception] worker=%zu queue_size=%zu active_workers=%zu/%zu runtime_ready=%d degraded=%d detail=%s",
                worker_idx,
                task_queue_.size(),
                active_worker_count_.load(std::memory_order_relaxed),
                worker_thread_count_,
                semantic_runtime_ready_.load(std::memory_order_relaxed) ? 1 : 0,
                semantic_degraded_.load(std::memory_order_relaxed) ? 1 : 0,
                full_err.c_str());
            terminateSystemOnInferenceFailure(
                "worker_loop_exception",
                worker_idx,
                -1.0,
                0ull,
                full_err);
        } catch (...) {
            METRICS_INCREMENT(metrics::ERRORS_TOTAL);
            const std::string full_err = describeCurrentExceptionFull();
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][TRIGGER][worker_loop_unknown_exception] worker=%zu queue_size=%zu active_workers=%zu/%zu runtime_ready=%d degraded=%d detail=%s",
                worker_idx,
                task_queue_.size(),
                active_worker_count_.load(std::memory_order_relaxed),
                worker_thread_count_,
                semantic_runtime_ready_.load(std::memory_order_relaxed) ? 1 : 0,
                semantic_degraded_.load(std::memory_order_relaxed) ? 1 : 0,
                full_err.c_str());
            terminateSystemOnInferenceFailure(
                "worker_loop_unknown_exception",
                worker_idx,
                -1.0,
                0ull,
                full_err);
        }
    }
}

void SemanticModule::markSemanticDegraded(const char* reason) {
    const int err_count = ++consecutive_errors_;
    if (err_count >= kMaxConsecutiveErrors) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        semantic_degraded_.store(true, std::memory_order_relaxed);
        next_recovery_retry_s_.store(node_->now().seconds() + kRecoveryCooldownSec, std::memory_order_relaxed);
        recovery_attempts_.store(0, std::memory_order_relaxed);
        recovery_exhausted_reported_.store(false, std::memory_order_relaxed);
        RCLCPP_FATAL(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=degraded reason=%s threshold=%d → SemanticModule disabled to protect system stability",
            reason ? reason : "unknown", kMaxConsecutiveErrors);
        RCLCPP_FATAL(node_->get_logger(),
            "[SEMANTIC][TRIGGER][semantic_mark_degraded_threshold_reached] err_count=%d threshold=%d queue_size=%zu active_workers=%zu/%zu processed=%lu",
            err_count,
            kMaxConsecutiveErrors,
            task_queue_.size(),
            active_worker_count_.load(std::memory_order_relaxed),
            worker_thread_count_,
            static_cast<unsigned long>(processed_tasks_.load(std::memory_order_relaxed)));
        terminateSystemOnInferenceFailure(
            "semantic_mark_degraded_threshold_reached",
            0,
            -1.0,
            0ull,
            reason ? reason : "unknown");
    }
}

void SemanticModule::maybeAdjustActiveWorkersLocked(double now_s) {
    if (now_s < next_scale_eval_s_) return;
    next_scale_eval_s_ = now_s + autoscale_eval_interval_s_;

    const size_t q = task_queue_.size();
    const double usage_ratio = static_cast<double>(q) / static_cast<double>(kMaxQueueSize);
    const size_t prev = active_worker_count_.load(std::memory_order_relaxed);
    size_t target = prev;
    // 升速策略：高于高水位时按队列占用线性放大目标并发；降速策略：低于低水位回落到最小并发。
    if (usage_ratio >= autoscale_high_watermark_) {
        const double scale = (usage_ratio - autoscale_high_watermark_) /
                             std::max(1e-6, 1.0 - autoscale_high_watermark_);
        const size_t headroom = worker_thread_count_ - min_worker_count_;
        target = min_worker_count_ + static_cast<size_t>(std::ceil(scale * static_cast<double>(headroom)));
    } else if (usage_ratio <= autoscale_low_watermark_) {
        target = min_worker_count_;
    }

    target = std::max<size_t>(1, std::min(worker_thread_count_, target));
    if (target != prev) {
        active_worker_count_.store(target, std::memory_order_relaxed);
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][AUTO_SCALE] active_workers %zu -> %zu queue=%zu cap=%zu usage=%.2f high=%.2f low=%.2f",
            prev, target, q, kMaxQueueSize, usage_ratio, autoscale_high_watermark_, autoscale_low_watermark_);
        cv_.notify_all();
    }
}

void SemanticModule::processTask(const SyncedFrameEvent& event) {
    processTask(event, 0);
}

void SemanticModule::processTask(const SyncedFrameEvent& event, size_t worker_idx) {
    if (semantic_degraded_.load(std::memory_order_relaxed)) return;
    const bool force_b3_log = shouldForceGateLog(b3_forced_log_count_);
    SemanticProcessor::Ptr processor;
    {
        std::lock_guard<std::mutex> lk(processor_mutex_);
        if (semantic_processors_.empty()) return;
        processor = semantic_processors_[worker_idx % semantic_processors_.size()];
    }
    if (!processor || !processor->hasRuntimeCapability()) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][TRIGGER][processor_runtime_lost_pre_infer] worker=%zu ts=%.3f queue_size=%zu active_workers=%zu/%zu processor_null=%d processor_runtime=%d",
            worker_idx,
            event.timestamp,
            task_queue_.size(),
            active_worker_count_.load(std::memory_order_relaxed),
            worker_thread_count_,
            processor ? 0 : 1,
            (processor && processor->hasRuntimeCapability()) ? 1 : 0);
        terminateSystemOnInferenceFailure(
            "processor_runtime_lost_pre_infer",
            worker_idx,
            event.timestamp,
            semanticTraceId(0ull, event.timestamp));
    }

    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Module][RUN] step=start ts=%.3f pts=%zu cloud_frame=%s",
        event.timestamp,
        (event.cloud && event.cloud->size() > 0) ? event.cloud->size() : 0,
        event.cloud_frame.c_str());
    const auto trace_id = semanticTraceId(
        event.kf_info.timestamp > 0.0 ? static_cast<uint64_t>(event.kf_info.timestamp * 1000.0) : 0ull,
        event.timestamp);

    auto t0 = std::chrono::steady_clock::now();
    SemanticProcessor::ProcessResult proc_result;
    CloudXYZIPtr labeled_cloud;
    CloudXYZIPtr trunk_pre_body;
    CloudXYZIPtr trunk_post_body;

    CloudXYZIConstPtr cloud_for_sem = event.cloud;
    CloudXYZIPtr world_to_body_storage;
    std::string publish_cloud_frame = event.cloud_frame;
    if (event.cloud_frame == "world" && event.cloud && !event.cloud->empty()) {
        world_to_body_storage.reset(new CloudXYZI());
        pcl::transformPointCloud(*event.cloud, *world_to_body_storage, event.T_odom_b.inverse().matrix().cast<float>());
        world_to_body_storage->header = event.cloud->header;
        world_to_body_storage->width = static_cast<uint32_t>(world_to_body_storage->size());
        world_to_body_storage->height = 1;
        cloud_for_sem = world_to_body_storage;
        publish_cloud_frame = "body";
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            3000,
            "[SEMANTIC][Module][COORD] step=world_to_body trace=%lu ts=%.3f pts_in=%zu pts_out=%zu "
            "(SyncedFrameEvent: geometric/Patchwork normalized to body; viz publishes frame_id=body)",
            static_cast<unsigned long>(trace_id),
            event.timestamp,
            event.cloud->size(),
            cloud_for_sem->size());
    }

    try {
        proc_result = processor->process(cloud_for_sem, event.timestamp, event.T_odom_b, &labeled_cloud, &trunk_pre_body, &trunk_post_body);
        consecutive_errors_ = 0; // 重置错误计数
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][Module][STAGE_RESULT] stage=processor_process worker=%zu ts=%.3f trace=%lu "
            "src_cloud_frame=%s cloud_in=%zu labeled_cloud=%zu tree_landmarks=%zu plane_landmarks=%zu",
            worker_idx,
            event.timestamp,
            static_cast<unsigned long>(trace_id),
            event.cloud_frame.c_str(),
            cloud_for_sem ? cloud_for_sem->size() : 0,
            labeled_cloud ? labeled_cloud->size() : 0,
            proc_result.tree_landmarks.size(),
            proc_result.plane_landmarks.size());
    } catch (const std::exception& e) {
        ++consecutive_errors_;
        const std::string full_err = describeExceptionFull(e);
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][TRIGGER][processor_process_exception] worker=%zu ts=%.3f trace=%lu queue_size=%zu active_workers=%zu/%zu consecutive_errors=%d detail=%s",
            worker_idx,
            event.timestamp,
            static_cast<unsigned long>(trace_id),
            task_queue_.size(),
            active_worker_count_.load(std::memory_order_relaxed),
            worker_thread_count_,
            consecutive_errors_.load(std::memory_order_relaxed),
            full_err.c_str());
        terminateSystemOnInferenceFailure(
            "processor_process_exception",
            worker_idx,
            event.timestamp,
            trace_id,
            full_err);
    } catch (...) {
        ++consecutive_errors_;
        const std::string full_err = describeCurrentExceptionFull();
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][TRIGGER][processor_process_unknown_exception] worker=%zu ts=%.3f trace=%lu queue_size=%zu active_workers=%zu/%zu consecutive_errors=%d detail=%s",
            worker_idx,
            event.timestamp,
            static_cast<unsigned long>(trace_id),
            task_queue_.size(),
            active_worker_count_.load(std::memory_order_relaxed),
            worker_thread_count_,
            consecutive_errors_.load(std::memory_order_relaxed),
            full_err.c_str());
        terminateSystemOnInferenceFailure(
            "processor_process_unknown_exception",
            worker_idx,
            event.timestamp,
            trace_id,
            full_err);
    }
    auto t1 = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    ++processed_tasks_;
    RCLCPP_DEBUG(node_->get_logger(),
        "[SEMANTIC][Module][RUN_DIAG] worker=%zu ts=%.3f elapsed_ms=%.1f tree_landmarks=%zu plane_landmarks=%zu queue_now=%zu",
        worker_idx, event.timestamp, elapsed_ms, proc_result.tree_landmarks.size(), proc_result.plane_landmarks.size(), task_queue_.size());

    if (!processor->hasRuntimeCapability() && semantic_runtime_ready_.load(std::memory_order_relaxed)) {
        semantic_runtime_ready_.store(false, std::memory_order_relaxed);
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][TRIGGER][processor_runtime_lost_post_infer] worker=%zu ts=%.3f trace=%lu queue_size=%zu active_workers=%zu/%zu",
            worker_idx,
            event.timestamp,
            static_cast<unsigned long>(trace_id),
            task_queue_.size(),
            active_worker_count_.load(std::memory_order_relaxed),
            worker_thread_count_);
        terminateSystemOnInferenceFailure(
            "processor_runtime_lost_post_infer",
            worker_idx,
            event.timestamp,
            trace_id);
    }

    // 🏛️ [Fix] Always publish semantic cloud for visualization if available
    if (labeled_cloud && !labeled_cloud->empty()) {
        SemanticCloudEvent cloud_ev;
        cloud_ev.timestamp = event.timestamp;
        cloud_ev.labeled_cloud = labeled_cloud;
        cloud_ev.frame_id = publish_cloud_frame;
        const uint64_t seq = semantic_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
        cloud_ev.meta.event_id = makeEventId(event.timestamp, seq);
        cloud_ev.meta.producer = "SemanticModule";
        cloud_ev.meta.publish_ts = node_->now().seconds();
        event_bus_->publish(cloud_ev);
        const int ground_cls = ConfigManager::instance().semanticGeometricGroundPaintClassId();
        size_t ground_labeled_pts = 0;
        for (const auto& p : labeled_cloud->points) {
            if (!std::isfinite(p.intensity)) {
                continue;
            }
            if (std::lround(static_cast<double>(p.intensity)) == ground_cls) {
                ++ground_labeled_pts;
            }
        }
        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            3000,
            "[SEMANTIC][Module][LABEL_DIGEST] trace=%lu ts=%.3f pts=%zu ground_class=%d ground_pts=%zu (SemanticCloudEvent -> viz/下游；后端因子仅树/柱/墙面)",
            static_cast<unsigned long>(trace_id),
            event.timestamp,
            labeled_cloud->size(),
            ground_cls,
            ground_labeled_pts);
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][Module][STAGE_RESULT] stage=publish_semantic_cloud trace=%lu ts=%.3f cloud_pts=%zu event_id=%lu",
            static_cast<unsigned long>(trace_id),
            event.timestamp,
            labeled_cloud->size(),
            static_cast<unsigned long>(cloud_ev.meta.event_id));
    }

    if ((trunk_pre_body && !trunk_pre_body->empty()) || (trunk_post_body && !trunk_post_body->empty())) {
        SemanticTrunkVizEvent trunk_ev;
        trunk_ev.timestamp = event.timestamp;
        trunk_ev.frame_id = publish_cloud_frame;
        trunk_ev.pre_cluster_body = trunk_pre_body;
        trunk_ev.post_cluster_body = trunk_post_body;
        const uint64_t seq = semantic_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
        trunk_ev.meta.event_id = makeEventId(event.timestamp, seq);
        trunk_ev.meta.producer = "SemanticModule";
        trunk_ev.meta.publish_ts = node_->now().seconds();
        event_bus_->publish(trunk_ev);
        RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "[SEMANTIC][Module][STAGE_RESULT] stage=publish_trunk_viz trace=%lu ts=%.3f pre_pts=%zu post_pts=%zu",
            static_cast<unsigned long>(trace_id),
            event.timestamp,
            trunk_pre_body ? trunk_pre_body->size() : 0u,
            trunk_post_body ? trunk_post_body->size() : 0u);
    }

    if (!proc_result.tree_landmarks.empty() || !proc_result.plane_landmarks.empty()) {
        // 🏛️ [架构契约] 发布前准入校验
        std::vector<CylinderLandmark::Ptr> valid_trees;
        for (const auto& l : proc_result.tree_landmarks) {
            if (l && l->isValid()) valid_trees.push_back(l);
        }
        std::vector<PlaneLandmark::Ptr> valid_planes;
        for (const auto& l : proc_result.plane_landmarks) {
            if (l && l->isValid()) valid_planes.push_back(l);
        }

        if (!valid_trees.empty() || !valid_planes.empty()) {
            SemanticLandmarkEvent res_ev;
            res_ev.timestamp = event.timestamp;
            res_ev.keyframe_timestamp_hint = event.kf_info.timestamp;
            if (std::isfinite(event.kf_info.timestamp) && event.kf_info.timestamp > 0.0) {
                const double match_tol = ConfigManager::instance().semanticTimestampMatchToleranceS();
                auto kf = map_registry_->getKeyFrameByTimestamp(event.kf_info.timestamp, match_tol);
                if (kf) {
                    res_ev.keyframe_id_hint = kf->id;
                }
            }
            res_ev.landmarks = valid_trees;
            res_ev.plane_landmarks = valid_planes;
            const uint64_t seq = semantic_event_seq_.fetch_add(1, std::memory_order_relaxed) + 1;
            res_ev.meta.event_id = makeEventId(event.timestamp, seq);
            res_ev.meta.idempotency_key = res_ev.meta.event_id;
            res_ev.meta.producer_seq = seq;
            res_ev.meta.ref_version = map_registry_->getVersion();
            res_ev.meta.ref_epoch = map_registry_->getAlignmentEpoch();
            res_ev.meta.source_ts = event.timestamp;
            res_ev.meta.publish_ts = node_->now().seconds();
            res_ev.meta.producer = "SemanticModule";
            res_ev.meta.route_tag = "legacy";
            res_ev.processing_state = semantic_degraded_.load(std::memory_order_relaxed)
                ? ProcessingState::DEGRADED
                : ProcessingState::NORMAL;
            event_bus_->publish(res_ev);
            sem_event_publish_total_.fetch_add(1, std::memory_order_relaxed);
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][Module][STAGE_RESULT] stage=publish_landmarks result=published trace=%lu ts=%.3f valid_trees=%zu valid_planes=%zu event_id=%lu",
                static_cast<unsigned long>(trace_id),
                event.timestamp,
                valid_trees.size(),
                valid_planes.size(),
                static_cast<unsigned long>(res_ev.meta.event_id));
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
                "[CHAIN][B3 SEM->MAP] action=publish trace=%lu ts=%.3f trees=%zu planes=%zu kf_hint_ts=%.3f kf_hint_id=%lu total_publish=%lu",
                static_cast<unsigned long>(trace_id),
                event.timestamp,
                valid_trees.size(),
                valid_planes.size(),
                res_ev.keyframe_timestamp_hint,
                static_cast<unsigned long>(res_ev.keyframe_id_hint),
                static_cast<unsigned long>(sem_event_publish_total_.load(std::memory_order_relaxed)));
            if (force_b3_log) {
                RCLCPP_INFO(node_->get_logger(),
                    "[CHAIN][B3 SEM->MAP][FORCED] action=publish trace=%lu ts=%.3f trees=%zu planes=%zu cloud_published=%d total_publish=%lu forced_idx=%lu/5",
                    static_cast<unsigned long>(trace_id),
                    event.timestamp,
                    valid_trees.size(),
                    valid_planes.size(),
                    (labeled_cloud && !labeled_cloud->empty()) ? 1 : 0,
                    static_cast<unsigned long>(sem_event_publish_total_.load(std::memory_order_relaxed)),
                    static_cast<unsigned long>(b3_forced_log_count_.load(std::memory_order_relaxed)));
            }

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][Module][RUN] step=done ts=%.3f trees=%zu planes=%zu elapsed_ms=%.1f worker=%zu → published SemanticLandmarkEvent",
                event.timestamp, valid_trees.size(), valid_planes.size(), elapsed_ms, worker_idx);
        } else {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "[SEMANTIC][Module][STAGE_RESULT] stage=publish_landmarks result=drop_all_invalid trace=%lu ts=%.3f raw_trees=%zu raw_planes=%zu",
                static_cast<unsigned long>(trace_id),
                event.timestamp,
                proc_result.tree_landmarks.size(),
                proc_result.plane_landmarks.size());
            RCLCPP_DEBUG(node_->get_logger(),
                "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 (all failed validity check) elapsed_ms=%.1f",
                event.timestamp, elapsed_ms);
            if (force_b3_log) {
                RCLCPP_INFO(node_->get_logger(),
                    "[CHAIN][B3 SEM->MAP][FORCED] action=drop reason=all_failed_validity_check trace=%lu ts=%.3f trees=%zu planes=%zu valid_landmarks=0 cloud_published=%d forced_idx=%lu/5",
                    static_cast<unsigned long>(trace_id),
                    event.timestamp,
                    proc_result.tree_landmarks.size(),
                    proc_result.plane_landmarks.size(),
                    (labeled_cloud && !labeled_cloud->empty()) ? 1 : 0,
                    static_cast<unsigned long>(b3_forced_log_count_.load(std::memory_order_relaxed)));
            }
        }
    } else {
        ++no_landmark_tasks_;
        sem_event_drop_no_landmark_total_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][STAGE_RESULT] stage=publish_landmarks result=drop_no_landmarks trace=%lu ts=%.3f",
            static_cast<unsigned long>(trace_id),
            event.timestamp);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticLogThrottleMs,
            "[CHAIN][B3 SEM->MAP] action=drop trace=%lu reason=no_landmarks ts=%.3f total_drop_no_landmark=%lu reason_code=E_NO_LANDMARKS",
            static_cast<unsigned long>(trace_id),
            event.timestamp,
            static_cast<unsigned long>(sem_event_drop_no_landmark_total_.load(std::memory_order_relaxed)));
        if (force_b3_log) {
            RCLCPP_INFO(node_->get_logger(),
                "[CHAIN][B3 SEM->MAP][FORCED] action=drop reason=no_landmarks trace=%lu ts=%.3f cloud_published=%d total_drop_no_landmark=%lu forced_idx=%lu/5",
                static_cast<unsigned long>(trace_id),
                event.timestamp,
                (labeled_cloud && !labeled_cloud->empty()) ? 1 : 0,
                static_cast<unsigned long>(sem_event_drop_no_landmark_total_.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(b3_forced_log_count_.load(std::memory_order_relaxed)));
        }
        RCLCPP_DEBUG(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=done ts=%.3f landmarks=0 elapsed_ms=%.1f (no publish)",
            event.timestamp, elapsed_ms);
    }

    // 慢帧告警不再依赖“有无地标”，避免吞掉真实性能问题
    if (elapsed_ms > 250.0) {
        RCLCPP_WARN(node_->get_logger(),
            "[SEMANTIC][Module][RUN] step=SLOW ts=%.3f elapsed_ms=%.1f threshold_ms=250.0",
            event.timestamp, elapsed_ms);
    }

    // 周期统计：帮助判断“模块运行正常但无有效输出”还是“算法/配置问题”
    const auto now = std::chrono::steady_clock::now();
    const auto secs_since_stats = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_log_tp_).count();
    const auto secs_since_delta = std::chrono::duration_cast<std::chrono::seconds>(now - last_dt_delta_log_tp_).count();
    if (secs_since_stats >= kSemanticStatsIntervalSec) {
        last_stats_log_tp_ = now;
        const auto processed = processed_tasks_.load();
        const auto no_landmark = no_landmark_tasks_.load();
        const auto coalesced = coalesced_tasks_.load();
        const auto bp_drop = backpressure_drops_.load();
        const auto skipped_dup_kf = skipped_duplicate_keyframe_.load();
        const uint64_t dt_b0 = dt_hist_bins_[0].load(std::memory_order_relaxed);
        const uint64_t dt_b1 = dt_hist_bins_[1].load(std::memory_order_relaxed);
        const uint64_t dt_b2 = dt_hist_bins_[2].load(std::memory_order_relaxed);
        const uint64_t dt_b3 = dt_hist_bins_[3].load(std::memory_order_relaxed);
        const uint64_t dt_b4 = dt_hist_bins_[4].load(std::memory_order_relaxed);
        const uint64_t dt_b5 = dt_hist_bins_[5].load(std::memory_order_relaxed);
        const uint64_t dt_b6 = dt_hist_bins_[6].load(std::memory_order_relaxed);
        const uint64_t dt_missing = dt_hist_bins_[7].load(std::memory_order_relaxed);
        const double no_landmark_ratio =
            (processed > 0) ? (100.0 * static_cast<double>(no_landmark) / static_cast<double>(processed)) : 0.0;
            RCLCPP_INFO(node_->get_logger(),
                "[SEMANTIC][Module][STATS] processed=%lu no_landmark=%lu no_landmark_ratio=%.1f%% coalesced=%lu backpressure_drop=%lu skipped_duplicate_kf=%lu dt_hist=[<=0.01:%lu <=0.03:%lu <=0.05:%lu <=0.10:%lu <=0.20:%lu <=0.50:%lu >0.50:%lu missing:%lu]",
                static_cast<unsigned long>(processed),
                static_cast<unsigned long>(no_landmark),
                no_landmark_ratio,
                static_cast<unsigned long>(coalesced),
                static_cast<unsigned long>(bp_drop),
                static_cast<unsigned long>(skipped_dup_kf),
                static_cast<unsigned long>(dt_b0),
                static_cast<unsigned long>(dt_b1),
                static_cast<unsigned long>(dt_b2),
                static_cast<unsigned long>(dt_b3),
                static_cast<unsigned long>(dt_b4),
                static_cast<unsigned long>(dt_b5),
                static_cast<unsigned long>(dt_b6),
                static_cast<unsigned long>(dt_missing));
            if (processed >= 30 && no_landmark_ratio >= 95.0) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticErrorThrottleMs,
                "[SEMANTIC][Module][E_EFFECTIVE_OUTPUT] semantic chain running but output almost empty: "
                "no_landmark_ratio=%.1f%% processed=%lu no_landmark=%lu. "
                "Check tree class mapping / clustering thresholds / segmentation latency (reason_code=E_NO_EFFECTIVE_SEMANTIC_OUTPUT)",
                no_landmark_ratio,
                static_cast<unsigned long>(processed),
                static_cast<unsigned long>(no_landmark));
        }
        if (processed >= 30 && (no_landmark_ratio >= 95.0 || bp_drop > 0 || coalesced > 0)) {
            const size_t queue_now = task_queue_.size();
            const size_t workers_active = active_worker_count_.load(std::memory_order_relaxed);
            const size_t workers_total = worker_thread_count_;
            const char* dominant_chain_reason = "no_effective_landmark_output";
            if (bp_drop > 0 && bp_drop >= coalesced) {
                dominant_chain_reason = "backpressure_drop";
            } else if (coalesced > 0) {
                dominant_chain_reason = "task_coalescing";
            }
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), kSemanticErrorThrottleMs,
                "[SEMANTIC][Module][CHAIN_ANOMALY] dominant_reason=%s "
                "processed=%lu no_landmark=%lu(%.1f%%) coalesced=%lu backpressure_drop=%lu "
                "queue_now=%zu workers_active=%zu/%zu dt_aligned_le_30ms=%lu "
                "reason_code=E_SEMANTIC_MODULE_CHAIN_ANOMALY",
                dominant_chain_reason,
                static_cast<unsigned long>(processed),
                static_cast<unsigned long>(no_landmark),
                no_landmark_ratio,
                static_cast<unsigned long>(coalesced),
                static_cast<unsigned long>(bp_drop),
                queue_now,
                workers_active,
                workers_total,
                static_cast<unsigned long>(dt_b0 + dt_b1));
        }
    }

    if (secs_since_delta >= kSemanticDtDeltaIntervalSec) {
        last_dt_delta_log_tp_ = now;
        const uint64_t dt_now[8] = {dt_hist_bins_[0].load(std::memory_order_relaxed),
                                    dt_hist_bins_[1].load(std::memory_order_relaxed),
                                    dt_hist_bins_[2].load(std::memory_order_relaxed),
                                    dt_hist_bins_[3].load(std::memory_order_relaxed),
                                    dt_hist_bins_[4].load(std::memory_order_relaxed),
                                    dt_hist_bins_[5].load(std::memory_order_relaxed),
                                    dt_hist_bins_[6].load(std::memory_order_relaxed),
                                    dt_hist_bins_[7].load(std::memory_order_relaxed)};
        const uint64_t d0 = dt_now[0] - dt_hist_last_snapshot_[0];
        const uint64_t d1 = dt_now[1] - dt_hist_last_snapshot_[1];
        const uint64_t d2 = dt_now[2] - dt_hist_last_snapshot_[2];
        const uint64_t d3 = dt_now[3] - dt_hist_last_snapshot_[3];
        const uint64_t d4 = dt_now[4] - dt_hist_last_snapshot_[4];
        const uint64_t d5 = dt_now[5] - dt_hist_last_snapshot_[5];
        const uint64_t d6 = dt_now[6] - dt_hist_last_snapshot_[6];
        const uint64_t d7 = dt_now[7] - dt_hist_last_snapshot_[7];
        uint64_t delta_total = d0 + d1 + d2 + d3 + d4 + d5 + d6 + d7;
        const double align_ratio = (delta_total > 0)
            ? (100.0 * static_cast<double>(d0 + d1) / static_cast<double>(delta_total))
            : 0.0;
        for (size_t i = 0; i < 8; ++i) dt_hist_last_snapshot_[i] = dt_now[i];

        RCLCPP_INFO(node_->get_logger(),
            "[SEMANTIC][Module][DT_DELTA_100S] bins_delta=[<=0.01:%lu <=0.03:%lu <=0.05:%lu <=0.10:%lu <=0.20:%lu <=0.50:%lu >0.50:%lu missing:%lu] total=%lu aligned(<=0.03)=%.1f%%",
            static_cast<unsigned long>(d0),
            static_cast<unsigned long>(d1),
            static_cast<unsigned long>(d2),
            static_cast<unsigned long>(d3),
            static_cast<unsigned long>(d4),
            static_cast<unsigned long>(d5),
            static_cast<unsigned long>(d6),
            static_cast<unsigned long>(d7),
            static_cast<unsigned long>(delta_total),
            align_ratio);
    }
}

void SemanticModule::tryRecoverFromDegradedState(double now_s) {
    if (!semantic_degraded_.load(std::memory_order_relaxed)) return;
    const double next_retry = next_recovery_retry_s_.load(std::memory_order_relaxed);
    if (now_s < next_retry) return;

    const int attempts = recovery_attempts_.load(std::memory_order_relaxed);
    if (attempts >= kMaxRecoveryAttempts) {
        bool expected = false;
        if (recovery_exhausted_reported_.compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
            RCLCPP_ERROR(node_->get_logger(),
                "[SEMANTIC][RECOVERY] Exhausted recovery budget (%d attempts). Semantic remains degraded until restart/manual intervention.",
                kMaxRecoveryAttempts);
            BackpressureWarningEvent warn;
            warn.module_name = name_ + "_recovery_exhausted";
            warn.queue_usage_ratio = 1.0f;
            warn.critical = true;
            event_bus_->publish(warn);
        }
        return;
    }

    recovery_attempts_.store(attempts + 1, std::memory_order_relaxed);
    RCLCPP_WARN(node_->get_logger(),
        "[SEMANTIC][RECOVERY] Attempt %d/%d to recover degraded semantic runtime",
        attempts + 1, kMaxRecoveryAttempts);
    try {
        std::vector<SemanticProcessor::Ptr> new_processors;
        new_processors.reserve(worker_thread_count_);
        bool ready = true;
        for (size_t i = 0; i < worker_thread_count_; ++i) {
            auto p = std::make_shared<SemanticProcessor>(semantic_cfg_);
            ready = ready && p->hasRuntimeCapability();
            new_processors.push_back(p);
        }
        {
            std::lock_guard<std::mutex> lk(processor_mutex_);
            semantic_processors_ = std::move(new_processors);
        }
        semantic_runtime_ready_.store(ready, std::memory_order_relaxed);
        if (ready) {
            semantic_degraded_.store(false, std::memory_order_relaxed);
            consecutive_errors_.store(0, std::memory_order_relaxed);
            recovery_exhausted_reported_.store(false, std::memory_order_relaxed);
            RCLCPP_INFO(node_->get_logger(), "[SEMANTIC][RECOVERY] Semantic runtime recovered successfully");
            return;
        }
    } catch (const std::exception& e) {
        const std::string full_err = describeExceptionFull(e);
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][RECOVERY] Recovery attempt failed: %s", e.what());
        RCLCPP_ERROR(node_->get_logger(),
            "[SEMANTIC][RECOVERY] FULL_EXCEPTION: %s", full_err.c_str());
    }
    next_recovery_retry_s_.store(now_s + kRecoveryCooldownSec, std::memory_order_relaxed);
}

} // namespace automap_pro::v3
