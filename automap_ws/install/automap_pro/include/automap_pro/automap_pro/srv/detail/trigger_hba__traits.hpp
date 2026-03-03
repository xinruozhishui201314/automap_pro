// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:srv/TriggerHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__TRAITS_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/srv/detail/trigger_hba__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const TriggerHBA_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: wait_for_result
  {
    out << "wait_for_result: ";
    rosidl_generator_traits::value_to_yaml(msg.wait_for_result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TriggerHBA_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: wait_for_result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wait_for_result: ";
    rosidl_generator_traits::value_to_yaml(msg.wait_for_result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TriggerHBA_Request & msg, bool use_flow_style = false)
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
  const automap_pro::srv::TriggerHBA_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::TriggerHBA_Request & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::TriggerHBA_Request>()
{
  return "automap_pro::srv::TriggerHBA_Request";
}

template<>
inline const char * name<automap_pro::srv::TriggerHBA_Request>()
{
  return "automap_pro/srv/TriggerHBA_Request";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerHBA_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<automap_pro::srv::TriggerHBA_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<automap_pro::srv::TriggerHBA_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const TriggerHBA_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: elapsed_seconds
  {
    out << "elapsed_seconds: ";
    rosidl_generator_traits::value_to_yaml(msg.elapsed_seconds, out);
    out << ", ";
  }

  // member: final_mme
  {
    out << "final_mme: ";
    rosidl_generator_traits::value_to_yaml(msg.final_mme, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TriggerHBA_Response & msg,
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

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
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

  // member: final_mme
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "final_mme: ";
    rosidl_generator_traits::value_to_yaml(msg.final_mme, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TriggerHBA_Response & msg, bool use_flow_style = false)
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
  const automap_pro::srv::TriggerHBA_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::TriggerHBA_Response & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::TriggerHBA_Response>()
{
  return "automap_pro::srv::TriggerHBA_Response";
}

template<>
inline const char * name<automap_pro::srv::TriggerHBA_Response>()
{
  return "automap_pro/srv/TriggerHBA_Response";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerHBA_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::TriggerHBA_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::TriggerHBA_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::srv::TriggerHBA>()
{
  return "automap_pro::srv::TriggerHBA";
}

template<>
inline const char * name<automap_pro::srv::TriggerHBA>()
{
  return "automap_pro/srv/TriggerHBA";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerHBA>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::srv::TriggerHBA_Request>::value &&
    has_fixed_size<automap_pro::srv::TriggerHBA_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::srv::TriggerHBA>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::srv::TriggerHBA_Request>::value &&
    has_bounded_size<automap_pro::srv::TriggerHBA_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::srv::TriggerHBA>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::srv::TriggerHBA_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::srv::TriggerHBA_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_HBA__TRAITS_HPP_
