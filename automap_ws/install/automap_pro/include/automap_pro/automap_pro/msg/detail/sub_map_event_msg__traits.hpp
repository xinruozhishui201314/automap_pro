// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:msg/SubMapEventMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__TRAITS_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/msg/detail/sub_map_event_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'anchor_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace automap_pro
{

namespace msg
{

inline void to_flow_style_yaml(
  const SubMapEventMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: submap_id
  {
    out << "submap_id: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_id, out);
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << ", ";
  }

  // member: event_type
  {
    out << "event_type: ";
    rosidl_generator_traits::value_to_yaml(msg.event_type, out);
    out << ", ";
  }

  // member: keyframe_count
  {
    out << "keyframe_count: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframe_count, out);
    out << ", ";
  }

  // member: spatial_extent_m
  {
    out << "spatial_extent_m: ";
    rosidl_generator_traits::value_to_yaml(msg.spatial_extent_m, out);
    out << ", ";
  }

  // member: anchor_pose
  {
    out << "anchor_pose: ";
    to_flow_style_yaml(msg.anchor_pose, out);
    out << ", ";
  }

  // member: has_valid_gps
  {
    out << "has_valid_gps: ";
    rosidl_generator_traits::value_to_yaml(msg.has_valid_gps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SubMapEventMsg & msg,
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

  // member: submap_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submap_id: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_id, out);
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

  // member: event_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event_type: ";
    rosidl_generator_traits::value_to_yaml(msg.event_type, out);
    out << "\n";
  }

  // member: keyframe_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyframe_count: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframe_count, out);
    out << "\n";
  }

  // member: spatial_extent_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spatial_extent_m: ";
    rosidl_generator_traits::value_to_yaml(msg.spatial_extent_m, out);
    out << "\n";
  }

  // member: anchor_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "anchor_pose:\n";
    to_block_style_yaml(msg.anchor_pose, out, indentation + 2);
  }

  // member: has_valid_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_valid_gps: ";
    rosidl_generator_traits::value_to_yaml(msg.has_valid_gps, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SubMapEventMsg & msg, bool use_flow_style = false)
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
  const automap_pro::msg::SubMapEventMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::msg::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::msg::SubMapEventMsg & msg)
{
  return automap_pro::msg::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::msg::SubMapEventMsg>()
{
  return "automap_pro::msg::SubMapEventMsg";
}

template<>
inline const char * name<automap_pro::msg::SubMapEventMsg>()
{
  return "automap_pro/msg/SubMapEventMsg";
}

template<>
struct has_fixed_size<automap_pro::msg::SubMapEventMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::msg::SubMapEventMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::msg::SubMapEventMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__TRAITS_HPP_
