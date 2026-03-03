// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:srv/TriggerOptimize.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__TRAITS_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/srv/detail/trigger_optimize__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const TriggerOptimize_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: full_optimization
  {
    out << "full_optimization: ";
    rosidl_generator_traits::value_to_yaml(msg.full_optimization, out);
    out << ", ";
  }

  // member: max_iterations
  {
    out << "max_iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.max_iterations, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TriggerOptimize_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: full_optimization
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "full_optimization: ";
    rosidl_generator_traits::value_to_yaml(msg.full_optimization, out);
    out << "\n";
  }

  // member: max_iterations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_iterations: ";
    rosidl_generator_traits::value_to_yaml(msg.max_iterations, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TriggerOptimize_Request & msg, bool use_flow_style = false)
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
  const automap_pro::srv::TriggerOptimize_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::TriggerOptimize_Request & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::TriggerOptimize_Request>()
{
  return "automap_pro::srv::TriggerOptimize_Request";
}

template<>
inline const char * name<automap_pro::srv::TriggerOptimize_Request>()
{
  return "automap_pro/srv/TriggerOptimize_Request";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerOptimize_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<automap_pro::srv::TriggerOptimize_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<automap_pro::srv::TriggerOptimize_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const TriggerOptimize_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: elapsed_seconds
  {
    out << "elapsed_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_seconds, out);
    out << ", ";
  }

  // member: nodes_updated
  {
    out << "nodes_updated: ";
    rosidl_generator_traits::value_to_yaml(msg.nodes_updated, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TriggerOptimize_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: elapsed_seconds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elapsed_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_seconds, out);
    out << "\n";
  }

  // member: nodes_updated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "nodes_updated: ";
    rosidl_generator_traits::value_to_yaml(msg.nodes_updated, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TriggerOptimize_Response & msg, bool use_flow_style = false)
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
  const automap_pro::srv::TriggerOptimize_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::TriggerOptimize_Response & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::TriggerOptimize_Response>()
{
  return "automap_pro::srv::TriggerOptimize_Response";
}

template<>
inline const char * name<automap_pro::srv::TriggerOptimize_Response>()
{
  return "automap_pro/srv/TriggerOptimize_Response";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerOptimize_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<automap_pro::srv::TriggerOptimize_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<automap_pro::srv::TriggerOptimize_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::srv::TriggerOptimize>()
{
  return "automap_pro::srv::TriggerOptimize";
}

template<>
inline const char * name<automap_pro::srv::TriggerOptimize>()
{
  return "automap_pro/srv/TriggerOptimize";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerOptimize>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::srv::TriggerOptimize_Request>::value &&
    has_fixed_size<automap_pro::srv::TriggerOptimize_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::srv::TriggerOptimize>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::srv::TriggerOptimize_Request>::value &&
    has_bounded_size<automap_pro::srv::TriggerOptimize_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::srv::TriggerOptimize>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::srv::TriggerOptimize_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::srv::TriggerOptimize_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__TRAITS_HPP_
