#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/polynomial_camera.h>
#include <vikit/params_helper.h>
#include "rclcpp/rclcpp.hpp"

namespace vk {
namespace camera_loader {

/// Load from ROS Namespace
/// 相比ros，传入参数上多了一个node参数，调用要注意
inline bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;
  
  // 获取相机模型
  std::string cam_model = node->get_parameter(ns + "/cam_model").get_value<std::string>();

  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(node->get_parameter(ns + "/cam_calib_file").get_value<std::string>());
  }
  else if(cam_model == "Pinhole")
  {
    double scale_val = 1.0;
    node->get_parameter(ns + "/scale", scale_val);
    double d0_val = 0.0, d1_val = 0.0, d2_val = 0.0, d3_val = 0.0;
    node->get_parameter(ns + "/cam_d0", d0_val);
    node->get_parameter(ns + "/cam_d1", d1_val);
    node->get_parameter(ns + "/cam_d2", d2_val);
    node->get_parameter(ns + "/cam_d3", d3_val);
    cam = new vk::PinholeCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        scale_val,
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        d0_val, d1_val, d2_val, d3_val);
  }
  else if(cam_model == "EquidistantCamera")
  {
    double scale_val = 1.0;
    node->get_parameter(ns + "/scale", scale_val);
    double k1_val = 0.0, k2_val = 0.0, k3_val = 0.0, k4_val = 0.0;
    node->get_parameter(ns + "/k1", k1_val);
    node->get_parameter(ns + "/k2", k2_val);
    node->get_parameter(ns + "/k3", k3_val);
    node->get_parameter(ns + "/k4", k4_val);
    cam = new vk::EquidistantCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        scale_val,
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        k1_val, k2_val, k3_val, k4_val);
  }
  else if(cam_model == "PolynomialCamera")
  {
    double k2_val = 0.0, k3_val = 0.0, k4_val = 0.0, k5_val = 0.0, k6_val = 0.0, k7_val = 0.0;
    node->get_parameter(ns + "/k2", k2_val);
    node->get_parameter(ns + "/k3", k3_val);
    node->get_parameter(ns + "/k4", k4_val);
    node->get_parameter(ns + "/k5", k5_val);
    node->get_parameter(ns + "/k6", k6_val);
    node->get_parameter(ns + "/k7", k7_val);
    cam = new vk::PolynomialCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        node->get_parameter(ns + "/cam_skew").get_value<double>(),
        k2_val, k3_val, k4_val, k5_val, k6_val, k7_val);
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        node->get_parameter(ns + "/cam_width").get_value<int>(),
        node->get_parameter(ns + "/cam_height").get_value<int>(),
        node->get_parameter(ns + "/cam_fx").get_value<double>(),
        node->get_parameter(ns + "/cam_fy").get_value<double>(),
        node->get_parameter(ns + "/cam_cx").get_value<double>(),
        node->get_parameter(ns + "/cam_cy").get_value<double>(),
        node->get_parameter(ns + "/cam_d0").get_value<double>());
  }
  else
  {
    cam = NULL;
    res = false;
  }
  return res;
}

inline bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, std::vector<vk::AbstractCamera*>& cam_list)
{
  bool res = true;

  std::string cam_model = node->get_parameter(ns + "/cam_model").get_value<std::string>();
  int cam_num = node->get_parameter(ns + "/cam_num").get_value<int>();
  
  for (int i = 0; i < cam_num; i++)
  {
    std::string cam_ns = ns + "/cam_" + std::to_string(i);
    std::string cam_model = node->get_parameter(cam_ns + "/cam_model").get_value<std::string>();
    
    if(cam_model == "FishPoly")
    {
      double k2_val = 0.0, k3_val = 0.0, k4_val = 0.0, k5_val = 0.0, k6_val = 0.0, k7_val = 0.0;
      node->get_parameter(cam_ns + "/k2", k2_val);
      node->get_parameter(cam_ns + "/k3", k3_val);
      node->get_parameter(cam_ns + "/k4", k4_val);
      node->get_parameter(cam_ns + "/k5", k5_val);
      node->get_parameter(cam_ns + "/k6", k6_val);
      node->get_parameter(cam_ns + "/k7", k7_val);
      cam_list.push_back(new vk::PolynomialCamera(
        node->get_parameter(cam_ns + "/image_width").get_value<int>(),
        node->get_parameter(cam_ns + "/image_height").get_value<int>(),
        node->get_parameter(cam_ns + "/A11").get_value<double>(),  // cam_fx
        node->get_parameter(cam_ns + "/A22").get_value<double>(),  // cam_fy
        node->get_parameter(cam_ns + "/u0").get_value<double>(),  // cam_cx
        node->get_parameter(cam_ns + "/v0").get_value<double>(),  // cam_cy
        node->get_parameter(cam_ns + "/A12").get_value<double>(), // cam_skew
        k2_val, k3_val, k4_val, k5_val, k6_val, k7_val));
    }
    else if(cam_model == "Pinhole")
    {
      double scale_val = 1.0;
      node->get_parameter(ns + "/scale", scale_val);
      double d0_val = 0.0, d1_val = 0.0, d2_val = 0.0, d3_val = 0.0;
      node->get_parameter(ns + "/cam_d0", d0_val);
      node->get_parameter(ns + "/cam_d1", d1_val);
      node->get_parameter(ns + "/cam_d2", d2_val);
      node->get_parameter(ns + "/cam_d3", d3_val);
      cam_list.push_back(new vk::PinholeCamera(
          node->get_parameter(ns + "/cam_width").get_value<int>(),
          node->get_parameter(ns + "/cam_height").get_value<int>(),
          scale_val,
          node->get_parameter(ns + "/cam_fx").get_value<double>(),
          node->get_parameter(ns + "/cam_fy").get_value<double>(),
          node->get_parameter(ns + "/cam_cx").get_value<double>(),
          node->get_parameter(ns + "/cam_cy").get_value<double>(),
          d0_val, d1_val, d2_val, d3_val));
    }
    else 
    {
      res = false;
    }
  }

  return res;
}

} // namespace camera_loader
} // namespace vk

#endif // VIKIT_CAMERA_LOADER_H_
