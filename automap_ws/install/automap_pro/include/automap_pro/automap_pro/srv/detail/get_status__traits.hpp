// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:srv/GetStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__TRAITS_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/srv/detail/get_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetStatus_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetStatus_Request & msg, bool use_flow_style = false)
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

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::srv::GetStatus_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::GetStatus_Request & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::GetStatus_Request>()
{
  return "automap_pro::srv::GetStatus_Request";
}

template<>
inline const char * name<automap_pro::srv::GetStatus_Request>()
{
  return "automap_pro/srv/GetStatus_Request";
}

template<>
struct has_fixed_size<automap_pro::srv::GetStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<automap_pro::srv::GetStatus_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<automap_pro::srv::GetStatus_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetStatus_Response & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: gps_align_score
  {
    out << "gps_align_score: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_align_score, out);
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
  const GetStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: gps_align_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_align_score: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_align_score, out);
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

inline std::string to_yaml(const GetStatus_Response & msg, bool use_flow_style = false)
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

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::srv::GetStatus_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::GetStatus_Response & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::GetStatus_Response>()
{
  return "automap_pro::srv::GetStatus_Response";
}

template<>
inline const char * name<automap_pro::srv::GetStatus_Response>()
{
  return "automap_pro/srv/GetStatus_Response";
}

template<>
struct has_fixed_size<automap_pro::srv::GetStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::GetStatus_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::GetStatus_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::srv::GetStatus>()
{
  return "automap_pro::srv::GetStatus";
}

template<>
inline const char * name<automap_pro::srv::GetStatus>()
{
  return "automap_pro/srv/GetStatus";
}

template<>
struct has_fixed_size<automap_pro::srv::GetStatus>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::srv::GetStatus_Request>::value &&
    has_fixed_size<automap_pro::srv::GetStatus_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::srv::GetStatus>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::srv::GetStatus_Request>::value &&
    has_bounded_size<automap_pro::srv::GetStatus_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::srv::GetStatus>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::srv::GetStatus_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::srv::GetStatus_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__TRAITS_HPP_
