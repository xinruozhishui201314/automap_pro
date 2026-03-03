// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice

#ifndef OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__TRAITS_HPP_
#define OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "overlap_transformer_msgs/srv/detail/compute_descriptor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pointcloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace overlap_transformer_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeDescriptor_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pointcloud
  {
    out << "pointcloud: ";
    to_flow_style_yaml(msg.pointcloud, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeDescriptor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pointcloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pointcloud:\n";
    to_block_style_yaml(msg.pointcloud, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeDescriptor_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace overlap_transformer_msgs

namespace rosidl_generator_traits
{

[[deprecated("use overlap_transformer_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const overlap_transformer_msgs::srv::ComputeDescriptor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  overlap_transformer_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use overlap_transformer_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const overlap_transformer_msgs::srv::ComputeDescriptor_Request & msg)
{
  return overlap_transformer_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<overlap_transformer_msgs::srv::ComputeDescriptor_Request>()
{
  return "overlap_transformer_msgs::srv::ComputeDescriptor_Request";
}

template<>
inline const char * name<overlap_transformer_msgs::srv::ComputeDescriptor_Request>()
{
  return "overlap_transformer_msgs/srv/ComputeDescriptor_Request";
}

template<>
struct has_fixed_size<overlap_transformer_msgs::srv::ComputeDescriptor_Request>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct has_bounded_size<overlap_transformer_msgs::srv::ComputeDescriptor_Request>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct is_message<overlap_transformer_msgs::srv::ComputeDescriptor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'descriptor'
#include "std_msgs/msg/detail/float32_multi_array__traits.hpp"

namespace overlap_transformer_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ComputeDescriptor_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: descriptor
  {
    out << "descriptor: ";
    to_flow_style_yaml(msg.descriptor, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ComputeDescriptor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: descriptor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "descriptor:\n";
    to_block_style_yaml(msg.descriptor, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ComputeDescriptor_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace overlap_transformer_msgs

namespace rosidl_generator_traits
{

[[deprecated("use overlap_transformer_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const overlap_transformer_msgs::srv::ComputeDescriptor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  overlap_transformer_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use overlap_transformer_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const overlap_transformer_msgs::srv::ComputeDescriptor_Response & msg)
{
  return overlap_transformer_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<overlap_transformer_msgs::srv::ComputeDescriptor_Response>()
{
  return "overlap_transformer_msgs::srv::ComputeDescriptor_Response";
}

template<>
inline const char * name<overlap_transformer_msgs::srv::ComputeDescriptor_Response>()
{
  return "overlap_transformer_msgs/srv/ComputeDescriptor_Response";
}

template<>
struct has_fixed_size<overlap_transformer_msgs::srv::ComputeDescriptor_Response>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Float32MultiArray>::value> {};

template<>
struct has_bounded_size<overlap_transformer_msgs::srv::ComputeDescriptor_Response>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Float32MultiArray>::value> {};

template<>
struct is_message<overlap_transformer_msgs::srv::ComputeDescriptor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<overlap_transformer_msgs::srv::ComputeDescriptor>()
{
  return "overlap_transformer_msgs::srv::ComputeDescriptor";
}

template<>
inline const char * name<overlap_transformer_msgs::srv::ComputeDescriptor>()
{
  return "overlap_transformer_msgs/srv/ComputeDescriptor";
}

template<>
struct has_fixed_size<overlap_transformer_msgs::srv::ComputeDescriptor>
  : std::integral_constant<
    bool,
    has_fixed_size<overlap_transformer_msgs::srv::ComputeDescriptor_Request>::value &&
    has_fixed_size<overlap_transformer_msgs::srv::ComputeDescriptor_Response>::value
  >
{
};

template<>
struct has_bounded_size<overlap_transformer_msgs::srv::ComputeDescriptor>
  : std::integral_constant<
    bool,
    has_bounded_size<overlap_transformer_msgs::srv::ComputeDescriptor_Request>::value &&
    has_bounded_size<overlap_transformer_msgs::srv::ComputeDescriptor_Response>::value
  >
{
};

template<>
struct is_service<overlap_transformer_msgs::srv::ComputeDescriptor>
  : std::true_type
{
};

template<>
struct is_service_request<overlap_transformer_msgs::srv::ComputeDescriptor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<overlap_transformer_msgs::srv::ComputeDescriptor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__TRAITS_HPP_
