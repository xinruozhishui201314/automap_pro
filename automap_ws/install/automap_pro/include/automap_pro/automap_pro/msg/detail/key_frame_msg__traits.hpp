// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:msg/KeyFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__TRAITS_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/msg/detail/key_frame_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"
// Member 'gps'
#include "automap_pro/msg/detail/gps_measurement_msg__traits.hpp"

namespace automap_pro
{

namespace msg
{

inline void to_flow_style_yaml(
  const KeyFrameMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << ", ";
  }

  // member: submap_id
  {
    out << "submap_id: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_id, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: cloud
  {
    out << "cloud: ";
    to_flow_style_yaml(msg.cloud, out);
    out << ", ";
  }

  // member: has_gps
  {
    out << "has_gps: ";
    rosidl_generator_traits::value_to_yaml(msg.has_gps, out);
    out << ", ";
  }

  // member: gps
  {
    out << "gps: ";
    to_flow_style_yaml(msg.gps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KeyFrameMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: session_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << "\n";
  }

  // member: submap_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submap_id: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud:\n";
    to_block_style_yaml(msg.cloud, out, indentation + 2);
  }

  // member: has_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_gps: ";
    rosidl_generator_traits::value_to_yaml(msg.has_gps, out);
    out << "\n";
  }

  // member: gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps:\n";
    to_block_style_yaml(msg.gps, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KeyFrameMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::msg::KeyFrameMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::msg::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::msg::KeyFrameMsg & msg)
{
  return automap_pro::msg::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::msg::KeyFrameMsg>()
{
  return "automap_pro::msg::KeyFrameMsg";
}

template<>
inline const char * name<automap_pro::msg::KeyFrameMsg>()
{
  return "automap_pro/msg/KeyFrameMsg";
}

template<>
struct has_fixed_size<automap_pro::msg::KeyFrameMsg>
  : std::integral_constant<bool, has_fixed_size<automap_pro::msg::GPSMeasurementMsg>::value && has_fixed_size<geometry_msgs::msg::PoseWithCovariance>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<automap_pro::msg::KeyFrameMsg>
  : std::integral_constant<bool, has_bounded_size<automap_pro::msg::GPSMeasurementMsg>::value && has_bounded_size<geometry_msgs::msg::PoseWithCovariance>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<automap_pro::msg::KeyFrameMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__TRAITS_HPP_
