// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__TRAITS_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/msg/detail/loop_constraint_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'delta_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace automap_pro
{

namespace msg
{

inline void to_flow_style_yaml(
  const LoopConstraintMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: submap_i
  {
    out << "submap_i: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_i, out);
    out << ", ";
  }

  // member: submap_j
  {
    out << "submap_j: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_j, out);
    out << ", ";
  }

  // member: session_i
  {
    out << "session_i: ";
    rosidl_generator_traits::value_to_yaml(msg.session_i, out);
    out << ", ";
  }

  // member: session_j
  {
    out << "session_j: ";
    rosidl_generator_traits::value_to_yaml(msg.session_j, out);
    out << ", ";
  }

  // member: overlap_score
  {
    out << "overlap_score: ";
    rosidl_generator_traits::value_to_yaml(msg.overlap_score, out);
    out << ", ";
  }

  // member: inlier_ratio
  {
    out << "inlier_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.inlier_ratio, out);
    out << ", ";
  }

  // member: rmse
  {
    out << "rmse: ";
    rosidl_generator_traits::value_to_yaml(msg.rmse, out);
    out << ", ";
  }

  // member: is_inter_session
  {
    out << "is_inter_session: ";
    rosidl_generator_traits::value_to_yaml(msg.is_inter_session, out);
    out << ", ";
  }

  // member: information_matrix
  {
    if (msg.information_matrix.size() == 0) {
      out << "information_matrix: []";
    } else {
      out << "information_matrix: [";
      size_t pending_items = msg.information_matrix.size();
      for (auto item : msg.information_matrix) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: delta_pose
  {
    out << "delta_pose: ";
    to_flow_style_yaml(msg.delta_pose, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoopConstraintMsg & msg,
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

  // member: submap_i
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submap_i: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_i, out);
    out << "\n";
  }

  // member: submap_j
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submap_j: ";
    rosidl_generator_traits::value_to_yaml(msg.submap_j, out);
    out << "\n";
  }

  // member: session_i
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_i: ";
    rosidl_generator_traits::value_to_yaml(msg.session_i, out);
    out << "\n";
  }

  // member: session_j
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_j: ";
    rosidl_generator_traits::value_to_yaml(msg.session_j, out);
    out << "\n";
  }

  // member: overlap_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "overlap_score: ";
    rosidl_generator_traits::value_to_yaml(msg.overlap_score, out);
    out << "\n";
  }

  // member: inlier_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inlier_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.inlier_ratio, out);
    out << "\n";
  }

  // member: rmse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rmse: ";
    rosidl_generator_traits::value_to_yaml(msg.rmse, out);
    out << "\n";
  }

  // member: is_inter_session
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_inter_session: ";
    rosidl_generator_traits::value_to_yaml(msg.is_inter_session, out);
    out << "\n";
  }

  // member: information_matrix
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.information_matrix.size() == 0) {
      out << "information_matrix: []\n";
    } else {
      out << "information_matrix:\n";
      for (auto item : msg.information_matrix) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: delta_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "delta_pose:\n";
    to_block_style_yaml(msg.delta_pose, out, indentation + 2);
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoopConstraintMsg & msg, bool use_flow_style = false)
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
  const automap_pro::msg::LoopConstraintMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::msg::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::msg::LoopConstraintMsg & msg)
{
  return automap_pro::msg::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::msg::LoopConstraintMsg>()
{
  return "automap_pro::msg::LoopConstraintMsg";
}

template<>
inline const char * name<automap_pro::msg::LoopConstraintMsg>()
{
  return "automap_pro/msg/LoopConstraintMsg";
}

template<>
struct has_fixed_size<automap_pro::msg::LoopConstraintMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::msg::LoopConstraintMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::msg::LoopConstraintMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__TRAITS_HPP_
