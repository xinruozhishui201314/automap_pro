// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:msg/MappingStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__TRAITS_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/msg/detail/mapping_status_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace automap_pro
{

namespace msg
{

inline void to_flow_style_yaml(
  const MappingStatusMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
    out << ", ";
  }

  // member: keyframe_count
  {
    out << "keyframe_count: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframe_count, out);
    out << ", ";
  }

  // member: submap_count
  {
    out << "submap_count: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_count, out);
    out << ", ";
  }

  // member: loop_count
  {
    out << "loop_count: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_count, out);
    out << ", ";
  }

  // member: gps_aligned
  {
    out << "gps_aligned: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_aligned, out);
    out << ", ";
  }

  // member: gps_alignment_score
  {
    out << "gps_alignment_score: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_alignment_score, out);
    out << ", ";
  }

  // member: map_entropy
  {
    out << "map_entropy: ";
    rosidl_generator_traits::value_to_yaml(msg.map_entropy, out);
    out << ", ";
  }

  // member: total_distance_m
  {
    out << "total_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.total_distance_m, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MappingStatusMsg & msg,
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

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
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

  // member: keyframe_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyframe_count: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframe_count, out);
    out << "\n";
  }

  // member: submap_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submap_count: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_count, out);
    out << "\n";
  }

  // member: loop_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "loop_count: ";
    rosidl_generator_traits::value_to_yaml(msg.loop_count, out);
    out << "\n";
  }

  // member: gps_aligned
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_aligned: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_aligned, out);
    out << "\n";
  }

  // member: gps_alignment_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_alignment_score: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_alignment_score, out);
    out << "\n";
  }

  // member: map_entropy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_entropy: ";
    rosidl_generator_traits::value_to_yaml(msg.map_entropy, out);
    out << "\n";
  }

  // member: total_distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.total_distance_m, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MappingStatusMsg & msg, bool use_flow_style = false)
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
  const automap_pro::msg::MappingStatusMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::msg::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::msg::MappingStatusMsg & msg)
{
  return automap_pro::msg::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::msg::MappingStatusMsg>()
{
  return "automap_pro::msg::MappingStatusMsg";
}

template<>
inline const char * name<automap_pro::msg::MappingStatusMsg>()
{
  return "automap_pro/msg/MappingStatusMsg";
}

template<>
struct has_fixed_size<automap_pro::msg::MappingStatusMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::msg::MappingStatusMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::msg::MappingStatusMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__TRAITS_HPP_
