// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:msg/KeyFrameInfoMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__TRAITS_HPP_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/msg/detail/key_frame_info_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__traits.hpp"

namespace automap_pro
{

namespace msg
{

inline void to_flow_style_yaml(
  const KeyFrameInfoMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: keyframe_id
  {
    out << "keyframe_id: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframe_id, out);
    out << ", ";
  }

  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: esikf_covariance_norm
  {
    out << "esikf_covariance_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.esikf_covariance_norm, out);
    out << ", ";
  }

  // member: is_degenerate
  {
    out << "is_degenerate: ";
    rosidl_generator_traits::value_to_yaml(msg.is_degenerate, out);
    out << ", ";
  }

  // member: map_update_rate
  {
    out << "map_update_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.map_update_rate, out);
    out << ", ";
  }

  // member: gyro_bias
  {
    if (msg.gyro_bias.size() == 0) {
      out << "gyro_bias: []";
    } else {
      out << "gyro_bias: [";
      size_t pending_items = msg.gyro_bias.size();
      for (auto item : msg.gyro_bias) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: accel_bias
  {
    if (msg.accel_bias.size() == 0) {
      out << "accel_bias: []";
    } else {
      out << "accel_bias: [";
      size_t pending_items = msg.accel_bias.size();
      for (auto item : msg.accel_bias) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cloud_point_count
  {
    out << "cloud_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.cloud_point_count, out);
    out << ", ";
  }

  // member: cloud_valid
  {
    out << "cloud_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.cloud_valid, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const KeyFrameInfoMsg & msg,
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

  // member: keyframe_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "keyframe_id: ";
    rosidl_generator_traits::value_to_yaml(msg.keyframe_id, out);
    out << "\n";
  }

  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: esikf_covariance_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "esikf_covariance_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.esikf_covariance_norm, out);
    out << "\n";
  }

  // member: is_degenerate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_degenerate: ";
    rosidl_generator_traits::value_to_yaml(msg.is_degenerate, out);
    out << "\n";
  }

  // member: map_update_rate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "map_update_rate: ";
    rosidl_generator_traits::value_to_yaml(msg.map_update_rate, out);
    out << "\n";
  }

  // member: gyro_bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gyro_bias.size() == 0) {
      out << "gyro_bias: []\n";
    } else {
      out << "gyro_bias:\n";
      for (auto item : msg.gyro_bias) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: accel_bias
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.accel_bias.size() == 0) {
      out << "accel_bias: []\n";
    } else {
      out << "accel_bias:\n";
      for (auto item : msg.accel_bias) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: cloud_point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.cloud_point_count, out);
    out << "\n";
  }

  // member: cloud_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.cloud_valid, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KeyFrameInfoMsg & msg, bool use_flow_style = false)
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
  const automap_pro::msg::KeyFrameInfoMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::msg::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::msg::KeyFrameInfoMsg & msg)
{
  return automap_pro::msg::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::msg::KeyFrameInfoMsg>()
{
  return "automap_pro::msg::KeyFrameInfoMsg";
}

template<>
inline const char * name<automap_pro::msg::KeyFrameInfoMsg>()
{
  return "automap_pro/msg/KeyFrameInfoMsg";
}

template<>
struct has_fixed_size<automap_pro::msg::KeyFrameInfoMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::PoseWithCovariance>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<automap_pro::msg::KeyFrameInfoMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::PoseWithCovariance>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<automap_pro::msg::KeyFrameInfoMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__TRAITS_HPP_
