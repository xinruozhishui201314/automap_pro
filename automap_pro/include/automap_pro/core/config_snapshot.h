#pragma once

/**
 * ConfigSnapshot — 不可变配置快照
 *
 * load() 完成后由 ConfigManager 填充，模块在构造/init 时获取副本，
 * 在 worker 线程中只读 snapshot，避免 shutdown 时访问 ConfigManager 单例导致 SIGSEGV。
 *
 * 使用方式：
 *   ConfigSnapshot snap = ConfigManager::instance().getSnapshot();
 *   // 在 worker 中仅使用 snap.xxx
 */

#include <cstddef>
#include <cstdint>

namespace automap_pro {

struct ConfigSnapshot {
    // Loop
    size_t loop_max_desc_queue_size = 128;
    size_t loop_max_match_queue_size = 128;
    int teaser_min_safe_inliers = 10;
    bool parallel_teaser_match = true;
    double pose_consistency_max_trans_m = 2.0;
    double pose_consistency_max_rot_deg = 25.0;

    // Backend / iSAM2
    bool backend_verbose_trace = false;
    double isam2_relin_thresh = 0.01;
    int isam2_relin_skip = 1;
    bool isam2_enable_relin = true;
    int backend_max_pending_gps_kf = 1000;
    int max_optimization_queue_size = 64;

    // Submap
    bool retain_cloud_body = true;
    bool map_statistical_filter = true;
    int map_stat_filter_mean_k = 50;
    double map_stat_filter_std_mul = 1.0;
    double submap_rebuild_thresh_trans = 2.0;
    double submap_rebuild_thresh_rot = 5.0;
    bool parallel_voxel_downsample = true;
    double submap_match_res = 0.4;

    // HBA
    bool hba_enabled = true;
    bool hba_gtsam_fallback_enabled = false;
    int gps_min_accepted_quality_level = 3;
    int hba_total_layers = 3;
    int hba_thread_num = 8;
};

}  // namespace automap_pro
