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
///
/// 参数命名约定（与 camera_pinhole.yaml / camera_fisheye_HILTI22.yaml 对齐）：
///   ROS2 parameter 名使用 "." 分隔（原 ROS1 版用 "/" 会导致 rclcpp 生成空名参数崩溃）
///   YAML 键名：model / width / height / scale / fx / fy / cx / cy / d0-d3 / k1-k7
inline bool loadFromRosNs(rclcpp::Node::SharedPtr node, const std::string& ns, vk::AbstractCamera*& cam)
{
  bool res = true;

  // ROS2 parameter names use "." as separator; ns+"/" is INVALID and causes
  // rclcpp to return a Parameter("", NOT_SET) → InvalidParameterTypeException("").
  // We use ns+"." here to match the camera YAML layout:
  //   /**:
  //     ros__parameters:
  //       parameter_blackboard:
  //         model: Pinhole   ← becomes "parameter_blackboard.model"
  const std::string sep = ".";

  // 安全读取 string 参数（参数不存在时返回 default_val，避免 NOT_SET 崩溃）
  auto get_str = [&](const std::string& key, const std::string& default_val = "") -> std::string {
    const std::string full = ns + sep + key;
    if (node->has_parameter(full)) {
      return node->get_parameter(full).get_value<std::string>();
    }
    return default_val;
  };
  auto get_int = [&](const std::string& key, int default_val = 0) -> int {
    const std::string full = ns + sep + key;
    if (node->has_parameter(full)) return node->get_parameter(full).get_value<int>();
    return default_val;
  };
  auto get_dbl = [&](const std::string& key, double default_val = 0.0) -> double {
    const std::string full = ns + sep + key;
    if (node->has_parameter(full)) return node->get_parameter(full).get_value<double>();
    return default_val;
  };

  // 读取相机模型类型；"model" 对应 YAML 中的 parameter_blackboard.model
  const std::string cam_model = get_str("model");
  if (cam_model.empty()) {
    cam = nullptr;
    return false;
  }

  if(cam_model == "Ocam")
  {
    cam = new vk::OmniCamera(get_str("cam_calib_file"));
  }
  else if(cam_model == "Pinhole")
  {
    cam = new vk::PinholeCamera(
        get_int("width"),  get_int("height"),
        get_dbl("scale", 1.0),
        get_dbl("fx"),     get_dbl("fy"),
        get_dbl("cx"),     get_dbl("cy"),
        get_dbl("d0"),     get_dbl("d1"),
        get_dbl("d2"),     get_dbl("d3"));
  }
  else if(cam_model == "EquidistantCamera")
  {
    cam = new vk::EquidistantCamera(
        get_int("width"),  get_int("height"),
        get_dbl("scale", 1.0),
        get_dbl("fx"),     get_dbl("fy"),
        get_dbl("cx"),     get_dbl("cy"),
        get_dbl("k1"),     get_dbl("k2"),
        get_dbl("k3"),     get_dbl("k4"));
  }
  else if(cam_model == "PolynomialCamera")
  {
    cam = new vk::PolynomialCamera(
        get_int("width"),  get_int("height"),
        get_dbl("fx"),     get_dbl("fy"),
        get_dbl("cx"),     get_dbl("cy"),
        get_dbl("skew"),
        get_dbl("k2"),     get_dbl("k3"),
        get_dbl("k4"),     get_dbl("k5"),
        get_dbl("k6"),     get_dbl("k7"));
  }
  else if(cam_model == "ATAN")
  {
    cam = new vk::ATANCamera(
        get_int("width"),  get_int("height"),
        get_dbl("fx"),     get_dbl("fy"),
        get_dbl("cx"),     get_dbl("cy"),
        get_dbl("d0"));
  }
  else
  {
    cam = nullptr;
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
