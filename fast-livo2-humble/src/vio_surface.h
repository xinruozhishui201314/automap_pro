/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <list>
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>
#include <vikit/equidistant_camera.h>

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};



class Warp
{
public:
  Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};



class Surface {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d normal_;
  bool is_converged_;
  std::vector<VisualPoint *> voxel_points;
  Surface(VisualPoint* init_pt) {
    normal_ = init_pt->normal_;
    voxel_points.push_back(init_pt);
    is_converged_ = false;
  }
  ~Surface() {
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
    voxel_points.clear();
  }
  bool belongsTo(const Eigen::Vector3d& new_normal) {
    // 夹角小于约 30 度 (cos > 0.85)
    return normal_.dot(new_normal) > 0.85; 
  }
  void updateNormal(const Eigen::Vector3d& new_normal) {
    // 1. 获取当前点数 (权重)
    double n = (double)voxel_points.size();

    // 2. 加权更新法向量 (滑动平均)
    // 旧法向量 * 旧权重 + 新法向量
    Eigen::Vector3d accumulated_normal = normal_ * n + new_normal;
    
    // 3. 归一化，确保模长为 1
    normal_ = accumulated_normal.normalized();
  }
  void addVoxelPoint(VisualPoint* vp) {
    updateNormal(vp->normal_);
    voxel_points.push_back(vp);
  }
  void deletePoint(int i) {
    if (i < 0 || i >= voxel_points.size()) return;

    VisualPoint* vp_to_delete = voxel_points[i]; // 暂存要删的指针

    // 1. 将 vector 最后一个元素覆盖到位置 i
    voxel_points[i] = voxel_points.back();
    
    // --- 1. 反向更新法向量 ---
    double n = (double)voxel_points.size();
    
    if (n > 1) {
        // 恢复成未归一化的总向量 (近似值)
        Eigen::Vector3d sum_normal = normal_ * n; 
        
        // 减去要删除那个点的法向量
        sum_normal -= vp_to_delete->normal_;
        
        // 重新归一化
        normal_ = sum_normal.normalized();
    } else {
        // 如果只剩这一个点，删完就没有法向量了
        // 可以设为零向量，或者保留原值，取决于你的业务逻辑
        normal_ = Eigen::Vector3d::Zero(); 
    }

    // 2. 移除 vector 最后一个元素 (O(1) 操作)
    voxel_points.pop_back();

    // 3. 释放内存
    if (vp_to_delete != nullptr) {
        delete vp_to_delete;
    }
  }
  // 如果到达上限,并且所有点收敛，那么这个面收敛。
};

class VOXEL_POINTS
{
public:
  std::vector<Surface *> surfaces;
  std::list<VOXEL_LOCATION>::iterator lru_iter;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  {
    for (Surface* s : surfaces) 
    {
      if (s != nullptr) { delete s; s = nullptr; }
    }
    surfaces.clear();
  }
};

class VIOManager
{
public:
  int grid_size;
  vk::AbstractCamera *cam;
  vk::PinholeCamera *pinhole_cam;
  StatesGroup *state;
  StatesGroup *state_propagat;
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  V3D Pli, Pci, Pcl, Pcw;
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;

  int width, height, grid_n_width, grid_n_height, length;
  double image_resize_factor;
  double fx, fy, cx, cy;
  // 添加：等距投影参数
  double k1, k2, k3, k4;
  int patch_pyrimid_level, patch_size, patch_size_total, patch_size_half, border, warp_len;
  int max_iterations, total_points;

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;
  std::vector<std::vector<V3D>> rays_with_sample_points;

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  Eigen::MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;
  unordered_map<VOXEL_LOCATION, int> sub_feat_map; 
  unordered_map<int, Warp *> warp_map;
  vector<VisualPoint *> retrieve_voxel_points;
  vector<pointWithVar> append_voxel_points;
  FramePtr new_frame_;
  cv::Mat img_cp, img_rgb, img_test;


  // 【新增】LRU 辅助链表，只存 Key (VOXEL_LOCATION)
  std::list<VOXEL_LOCATION> lru_list;
  // 【新增】缓存大小上限 (比如 50000 个体素)
  int lru_capacity = 1250;
  int voxel_surface_capacity = 15;
  int surface_count = 3;
  
  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  void updateStateInverse(cv::Mat img, int level);
  void updateState(cv::Mat img, int level);
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  void initializeVIO();
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  void computeJacobianAndUpdateEKF(cv::Mat img);
  void resetGrid();
  void updateVisualMapPoints(cv::Mat img);
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3<double> &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3<double> &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  void plotTrackedPoints();
  void updateFrameState(StatesGroup state);
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  void precomputeReferencePatches(int level);
  void dumpDataForColmap();
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);


  std::string cam_model_type;
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_