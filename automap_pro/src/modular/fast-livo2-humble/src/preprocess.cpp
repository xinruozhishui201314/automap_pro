/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "preprocess.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// Must match pl_buff/typess capacity in preprocess.h (128).
namespace {
constexpr int kMaxLineNum = 128;
// Default scan angular speed when point timestamps are synthesized from yaw (Velodyne / XT32).
constexpr double kLidarYawOmega = 3.61;
inline double lidar_yaw_omega_safe()
{
  return (std::fabs(kLidarYawOmega) < 1e-12) ? 1.0 : kLidarYawOmega;
}
} // namespace

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess() : feature_enabled(0), lidar_type(OUST64), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 6;
  group_size = 8;
  disA = 0.01;
  disA = 0.1; // B?
  disB = 0.1; // used in plane_judge group_dis; was uninitialized in upstream
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = (bld >= 0.0 && std::isfinite(bld)) ? bld : 0.0;
  blind_sqr = blind * blind;
  point_filter_num = (pfilt_num < 1) ? 1 : pfilt_num;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  if (!pcl_out)
  {
    return;
  }
  if (!msg)
  {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    *pcl_out = pl_surf;
    return;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case L515:
    l515_handler(msg);
    break;

  case XT32:
    xt32_handler(msg);
    break;

  case PANDAR128:
    Pandar128_handler(msg);
    break;

  case ROBOSENSE:
    robosense_handler(msg);
    break;

  default:
    printf("Error LiDAR Type: %d \n", lidar_type);
    pl_surf.clear();
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  if (!msg) return;
  pcl::PointCloud<pcl::PointXYZRGB> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = stamp2Sec(msg->header.stamp);
  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
  for (int i = 0; i < pl_orig.points.size(); i++)
  {
    if (i % point_filter_num != 0) continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < blind_sqr) continue;
    if (!std::isfinite(pl_orig.points[i].x) || !std::isfinite(pl_orig.points[i].y) || !std::isfinite(pl_orig.points[i].z)) continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.normal_x = pl_orig.points[i].r;
    added_pt.normal_y = pl_orig.points[i].g;
    added_pt.normal_z = pl_orig.points[i].b;

    added_pt.curvature = 0.0;
    pl_surf.points.push_back(added_pt);
  }

  cout << "pl size:: " << pl_orig.points.size() << endl;
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  if (!msg) return;
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  if (plsize == 0) return;
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    const int scan_cap = std::min(std::max(N_SCANS, 0), kMaxLineNum);
    for (int i = 0; i < scan_cap; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind_sqr) continue;
      if (!std::isfinite(pl_orig.points[i].x) || !std::isfinite(pl_orig.points[i].y) || !std::isfinite(pl_orig.points[i].z)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      const int ring = static_cast<int>(pl_orig.points[i].ring);
      if (ring >= 0 && ring < scan_cap) { pl_buff[ring].push_back(added_pt); }
    }

    for (int j = 0; j < scan_cap; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = stamp2Sec(msg->header.stamp);
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range =
          pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < blind_sqr) continue;
      if (!std::isfinite(pl_orig.points[i].x) || !std::isfinite(pl_orig.points[i].y) || !std::isfinite(pl_orig.points[i].z)) continue;

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0) yaw_angle -= 360.0;
      if (yaw_angle <= -180.0) yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;

      // cout<<added_pt.curvature<<endl;

      pl_surf.points.push_back(added_pt);
    }
    std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
      return a.curvature < b.curvature;
    });
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  if (!msg) {
    return;
  }

  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  // pl_buff/typess are fixed at 128 entries; N_SCANS from YAML may exceed that.
  const int scan_cap = std::min(std::max(N_SCANS, 0), kMaxLineNum);

  bool is_first[kMaxLineNum];
  double yaw_fp[kMaxLineNum] = {0};     // yaw of first scan point
  const double omega_l = lidar_yaw_omega_safe();
  float yaw_last[kMaxLineNum] = {0.0};  // yaw of last scan point
  float time_last[kMaxLineNum] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].time > 0) { given_offset_time = true; }
  else
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  if (feature_enabled)
  {
    for (int i = 0; i < scan_cap; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = static_cast<int>(pl_orig.points[i].ring);
      if (layer < 0 || layer >= scan_cap) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0; // units: ms
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) continue;

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < scan_cap; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time / 1000.0;
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) continue;

      if (!given_offset_time)
      {
        int layer = static_cast<int>(pl_orig.points[i].ring);
        if (layer < 0 || layer >= scan_cap) {
          continue;
        }
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        // added_pt.curvature = pl_orig.points[i].t;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
      // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature,
      // prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time,
          // pl_orig.points[i].t);
        }
      }
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_surf, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::Pandar128_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  if (!msg) return;

  pcl::PointCloud<Pandar128_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  double time_head = pl_orig.points[0].timestamp;
  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;

    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = static_cast<float>(pl_orig.points[i].intensity) / 255.0f;
    added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;
    if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) continue;

    if (i % point_filter_num == 0)
    {
      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
      {
        pl_surf.points.push_back(added_pt);
        // printf("time mode: %d time: %d \n", given_offset_time,
        // pl_orig.points[i].t);
      }
    }
  }

  // define a lambda function for the comparison
  auto comparePoints = [](const PointType& a, const PointType& b) -> bool
  {
    return a.curvature < b.curvature;
  };
  
  // sort the points using the comparison function
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), comparePoints);
  
  // cout << GREEN << "pl_surf.points[0].timestamp: " << pl_surf.points[0].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[1000].timestamp: " << pl_surf.points[1000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[5000].timestamp: " << pl_surf.points[5000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[10000].timestamp: " << pl_surf.points[10000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[20000].timestamp: " << pl_surf.points[20000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[30000].timestamp: " << pl_surf.points[30000].curvature << RESET << endl;
  // cout << GREEN << "pl_surf.points[31000].timestamp: " << pl_surf.points[31000].curvature << RESET << endl;
}

void Preprocess::xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  if (!msg) return;

  pcl::PointCloud<xt32_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  const int scan_cap = std::min(std::max(N_SCANS, 0), kMaxLineNum);

  bool is_first[kMaxLineNum];
  double yaw_fp[kMaxLineNum] = {0};     // yaw of first scan point
  const double omega_l = lidar_yaw_omega_safe();
  float yaw_last[kMaxLineNum] = {0.0};  // yaw of last scan point
  float time_last[kMaxLineNum] = {0.0}; // last offset time

  if (pl_orig.points[plsize - 1].timestamp > 0) { given_offset_time = true; }
  else
  {
    given_offset_time = false;
    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  double time_head = pl_orig.points[0].timestamp;

  if (feature_enabled)
  {
    for (int i = 0; i < scan_cap; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer < 0 || layer >= scan_cap) continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].timestamp / 1000.0; // units: ms
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) continue;

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer]) { added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l; }
        else { added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l; }

        if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < scan_cap; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.f;
      if (!std::isfinite(added_pt.x) || !std::isfinite(added_pt.y) || !std::isfinite(added_pt.z)) continue;

      // printf("added_pt.curvature: %lf %lf \n", added_pt.curvature,
      // pl_orig.points[i].timestamp);

      // if(i==(plsize-1))  printf("index: %d layer: %d, yaw: %lf, offset-time:
      // %lf, condition: %d\n", i, layer, yaw_angle, added_pt.curvature,
      // prints);
      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > blind_sqr)
        {
          pl_surf.points.push_back(added_pt);
          // printf("time mode: %d time: %d \n", given_offset_time,
          // pl_orig.points[i].t);
        }
      }
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_surf, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::robosense_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
  pl_surf.clear();
  if (!msg) return;

  pcl::PointCloud<robosense_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  double time_head = pl_orig.points[0].timestamp;
  for (int i = 0; i < plsize; ++i)
  {
    if (i % point_filter_num != 0) continue;

    const auto& pt = pl_orig.points[i];
    const double x = pt.x, y = pt.y, z = pt.z;
    const double dist_sqr = x * x + y * y + z * z;
    const bool is_valid = (dist_sqr >= blind_sqr) && !std::isnan(x) && !std::isnan(y) && !std::isnan(z);
    if (!is_valid) continue;

    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = (pt.timestamp - time_head) * 1000.0;
    pl_surf.points.push_back(added_pt);
  }
  std::sort(pl_surf.points.begin(), pl_surf.points.end(), [](const PointType &a, const PointType &b) {
    return a.curvature < b.curvature;
  });
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  if (static_cast<int>(types.size()) != plsize)
  {
    return;
  }
  if (plsize < 2)
  {
    return;
  }
  uint head = 0;
  // 若环上所有点的 range 均 < blind_sqr，原逻辑会把 head 加到 plsize 仍访问 types[head] → 越界 SIGSEGV。
  // run_20260328_220428：Velodyne + blind=1.0 + feature_extract 下某 ring 全近距即可触发。
  while (head < static_cast<uint>(plsize) && types[head].range < blind_sqr) {
    head++;
  }
  if (head >= static_cast<uint>(plsize)) {
    return;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0;
  uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind_sqr) { continue; }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);

    if (plane_type == 1)
    {
      if (i_nex >= static_cast<uint>(plsize)) {
        i_nex = static_cast<uint>(plsize - 1);
      }
      if (i_nex < i)
      {
        last_state = 0;
      }
      else
      {
        for (uint j = i; j <= i_nex; j++)
        {
          if (j != i && j != i_nex) { types[j].ftype = Real_Plane; }
          else { types[j].ftype = Poss_Plane; }
        }

        if (last_state == 1 && last_direct.norm() > 0.1)
        {
          double mod = last_direct.transpose() * curr_direct;
          if (mod > -0.707 && mod < 0.707) { types[i].ftype = Edge_Plane; }
          else { types[i].ftype = Real_Plane; }
        }

        // i_nex==0 时 i_nex-1 在无符号下会下溢为 UINT_MAX，导致 for 循环死循环/越界
        if (i_nex > 0) { i = i_nex - 1; }
        last_state = 1;
      }
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for (uint i = head + 3; i < plsize2; i++)
  {
    if (types[i].range < blind_sqr || types[i].ftype >= Real_Plane) { continue; }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) { continue; }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];
    bool have_prev = false;
    bool have_next = false;

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1) { m = 1; }

      if (types[i + m].range < blind_sqr)
      {
        if (types[i].range > inf_bound) { types[i].edj[j] = Nr_inf; }
        else { types[i].edj[j] = Nr_blind; }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a;

      const double na = vec_a.norm();
      const double nb = vecs[j].norm();
      if (na < 1e-15 || nb < 1e-15) { continue; }

      types[i].angle[j] = vec_a.dot(vecs[j]) / na / nb;
      if (types[i].angle[j] < jump_up_limit) { types[i].edj[j] = Nr_180; }
      else if (types[i].angle[j] > jump_down_limit) { types[i].edj[j] = Nr_zero; }
      if (j == 0) { have_prev = true; }
      else { have_next = true; }
    }

    if (!have_prev || !have_next) { continue; }

    const double np = vecs[Prev].norm();
    const double nn = vecs[Next].norm();
    if (np < 1e-15 || nn < 1e-15) { continue; }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / np / nn;
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Prev)) { types[i].ftype = Edge_Jump; }
      }
    }
    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next)) { types[i].ftype = Edge_Jump; }
      }
    }
    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev)) { types[i].ftype = Edge_Jump; }
    }
    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next)) { types[i].ftype = Edge_Jump; }
    }
    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor) { types[i].ftype = Wire; }
    }
  }

  plsize2 = plsize - 1;
  double ratio;
  for (uint i = head + 1; i < plsize2; i++)
  {
    if (types[i].range < blind_sqr || types[i - 1].range < blind_sqr || types[i + 1].range < blind_sqr) { continue; }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) { continue; }

    if (types[i].ftype == Nor)
    {
      if (types[i - 1].dista > types[i].dista) { ratio = types[i - 1].dista / types[i].dista; }
      else { ratio = types[i].dista / types[i - 1].dista; }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {
        if (types[i - 1].ftype == Nor) { types[i - 1].ftype = Real_Plane; }
        if (types[i + 1].ftype == Nor) { types[i + 1].ftype = Real_Plane; }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {
    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1) { last_surface = j; }

      if (j == uint(last_surface + point_filter_num - 1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) { pl_corn.push_back(pl[j]); }
      if (last_surface != -1)
      {
        PointType ap{};
        const uint span = j - static_cast<uint>(last_surface);
        if (span > 0)
        {
          for (uint k = last_surface; k < j; k++)
          {
            ap.x += pl[k].x;
            ap.y += pl[k].y;
            ap.z += pl[k].z;
            ap.curvature += pl[k].curvature;
          }
          ap.x /= static_cast<float>(span);
          ap.y /= static_cast<float>(span);
          ap.z /= static_cast<float>(span);
          ap.curvature /= static_cast<float>(span);
          pl_surf.push_back(ap);
        }
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "lidar";
  output.header.stamp = ct;
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  if (group_size <= 0)
  {
    curr_direct.setZero();
    return 2;
  }
  if (pl.empty() || static_cast<size_t>(i_cur) >= pl.size())
  {
    curr_direct.setZero();
    return 2;
  }
  if (types.size() < pl.size())
  {
    curr_direct.setZero();
    return 2;
  }
  if (static_cast<size_t>(i_cur) + static_cast<size_t>(group_size) > pl.size())
  {
    curr_direct.setZero();
    return 2;
  }

  double group_dis = disA * types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis = 0.0;
  bool have_two_dis = false;
  vector<double> disarr;
  disarr.reserve(20);

  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)
  {
    if (types[i_nex].range < blind_sqr)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }

  for (;;)
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if (types[i_nex].range < blind_sqr)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;
    have_two_dis = true;
    if (two_dis >= group_dis) { break; }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  if (!have_two_dis)
  {
    curr_direct.setZero();
    if (!pl.empty() && i_nex >= pl.size()) {
      i_nex = static_cast<uint>(pl.size() - 1);
    }
    return 0;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid) { leng_wid = lw; }
  }

  if (leng_wid < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if ((two_dis * two_dis / leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  if (disarrsize < 2)
  {
    curr_direct.setZero();
    return 0;
  }
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  const double vnorm = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (!std::isfinite(vnorm) || vnorm < 1e-15)
  {
    curr_direct.setZero();
    return 0;
  }
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  // give_feature uses: for (j = i; j <= i_nex; j++) on types[] / pl[]. The inner for(;;)
  // can leave i_nex == pl.size() after i_nex++ then break on i_nex >= pl.size() — that
  // value must not be used as an index (run_20260329_105033 SIGSEGV in give_feature).
  if (!pl.empty() && i_nex >= pl.size()) {
    i_nex = static_cast<uint>(pl.size() - 1);
  }
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  const size_t n = types.size();
  if (n < 3 || pl.size() != n || i >= n) return false;

  double d1 = 0.0;
  double d2 = 0.0;

  if (nor_dir == Prev)
  {
    if (i < 2) return false;
    if (types[i - 1].range < blind_sqr || types[i - 2].range < blind_sqr) return false;
    d1 = types[i - 1].dista;
    d2 = types[i - 2].dista;
  }
  else if (nor_dir == Next)
  {
    if (i + 2 >= n) return false;
    if (types[i + 1].range < blind_sqr || types[i + 2].range < blind_sqr) return false;
    d1 = types[i].dista;
    d2 = types[i + 1].dista;
  }
  else
  {
    return false;
  }

  double d;

  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  if (d1 < 0.0 || d2 < 0.0) return false;

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb) { return false; }

  return true;
}