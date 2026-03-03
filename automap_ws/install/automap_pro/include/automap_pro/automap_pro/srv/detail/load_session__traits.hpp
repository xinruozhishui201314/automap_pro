// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:srv/LoadSession.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__TRAITS_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/srv/detail/load_session__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadSession_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: session_dir
  {
    out << "session_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.session_dir, out);
    out << ", ";
  }

  // member: session_id
  {
    out << "session_id: ";
    rosidl_generator_traits::value_to_yaml(msg.session_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadSession_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: session_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "session_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.session_dir, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadSession_Request & msg, bool use_flow_style = false)
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
  const automap_pro::srv::LoadSession_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::LoadSession_Request & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::LoadSession_Request>()
{
  return "automap_pro::srv::LoadSession_Request";
}

template<>
inline const char * name<automap_pro::srv::LoadSession_Request>()
{
  return "automap_pro/srv/LoadSession_Request";
}

template<>
struct has_fixed_size<automap_pro::srv::LoadSession_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::LoadSession_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::LoadSession_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const LoadSession_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: submaps_loaded
  {
    out << "submaps_loaded: ";
    rosidl_generator_traits::value_to_yaml(msg.submaps_loaded, out);
    out << ", ";
  }

  // member: descriptors_loaded
  {
    out << "descriptors_loaded: ";
    rosidl_generator_traits::value_to_yaml(msg.descriptors_loaded, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LoadSession_Response & msg,
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

  // member: submaps_loaded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "submaps_loaded: ";
    rosidl_generator_traits::value_to_yaml(msg.submaps_loaded, out);
    out << "\n";
  }

  // member: descriptors_loaded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "descriptors_loaded: ";
    rosidl_generator_traits::value_to_yaml(msg.descriptors_loaded, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LoadSession_Response & msg, bool use_flow_style = false)
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
  const automap_pro::srv::LoadSession_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::LoadSession_Response & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::LoadSession_Response>()
{
  return "automap_pro::srv::LoadSession_Response";
}

template<>
inline const char * name<automap_pro::srv::LoadSession_Response>()
{
  return "automap_pro/srv/LoadSession_Response";
}

template<>
struct has_fixed_size<automap_pro::srv::LoadSession_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::LoadSession_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::LoadSession_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::srv::LoadSession>()
{
  return "automap_pro::srv::LoadSession";
}

template<>
inline const char * name<automap_pro::srv::LoadSession>()
{
  return "automap_pro/srv/LoadSession";
}

template<>
struct has_fixed_size<automap_pro::srv::LoadSession>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::srv::LoadSession_Request>::value &&
    has_fixed_size<automap_pro::srv::LoadSession_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::srv::LoadSession>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::srv::LoadSession_Request>::value &&
    has_bounded_size<automap_pro::srv::LoadSession_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::srv::LoadSession>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::srv::LoadSession_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::srv::LoadSession_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__SRV__DETAIL__LOAD_SESSION__TRAITS_HPP_
