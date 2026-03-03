// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__TRAITS_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/srv/detail/trigger_gps_align__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const TriggerGpsAlign_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TriggerGpsAlign_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TriggerGpsAlign_Request & msg, bool use_flow_style = false)
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
  const automap_pro::srv::TriggerGpsAlign_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::TriggerGpsAlign_Request & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::TriggerGpsAlign_Request>()
{
  return "automap_pro::srv::TriggerGpsAlign_Request";
}

template<>
inline const char * name<automap_pro::srv::TriggerGpsAlign_Request>()
{
  return "automap_pro/srv/TriggerGpsAlign_Request";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerGpsAlign_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<automap_pro::srv::TriggerGpsAlign_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<automap_pro::srv::TriggerGpsAlign_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const TriggerGpsAlign_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: alignment_rmse_m
  {
    out << "alignment_rmse_m: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_rmse_m, out);
    out << ", ";
  }

  // member: r_gps_lidar
  {
    if (msg.r_gps_lidar.size() == 0) {
      out << "r_gps_lidar: []";
    } else {
      out << "r_gps_lidar: [";
      size_t pending_items = msg.r_gps_lidar.size();
      for (auto item : msg.r_gps_lidar) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: t_gps_lidar
  {
    if (msg.t_gps_lidar.size() == 0) {
      out << "t_gps_lidar: []";
    } else {
      out << "t_gps_lidar: [";
      size_t pending_items = msg.t_gps_lidar.size();
      for (auto item : msg.t_gps_lidar) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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
  const TriggerGpsAlign_Response & msg,
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

  // member: alignment_rmse_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "alignment_rmse_m: ";
    rosidl_generator_traits::value_to_yaml(msg.alignment_rmse_m, out);
    out << "\n";
  }

  // member: r_gps_lidar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.r_gps_lidar.size() == 0) {
      out << "r_gps_lidar: []\n";
    } else {
      out << "r_gps_lidar:\n";
      for (auto item : msg.r_gps_lidar) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: t_gps_lidar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.t_gps_lidar.size() == 0) {
      out << "t_gps_lidar: []\n";
    } else {
      out << "t_gps_lidar:\n";
      for (auto item : msg.t_gps_lidar) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
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

inline std::string to_yaml(const TriggerGpsAlign_Response & msg, bool use_flow_style = false)
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
  const automap_pro::srv::TriggerGpsAlign_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::TriggerGpsAlign_Response & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::TriggerGpsAlign_Response>()
{
  return "automap_pro::srv::TriggerGpsAlign_Response";
}

template<>
inline const char * name<automap_pro::srv::TriggerGpsAlign_Response>()
{
  return "automap_pro/srv/TriggerGpsAlign_Response";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerGpsAlign_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::TriggerGpsAlign_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::TriggerGpsAlign_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::srv::TriggerGpsAlign>()
{
  return "automap_pro::srv::TriggerGpsAlign";
}

template<>
inline const char * name<automap_pro::srv::TriggerGpsAlign>()
{
  return "automap_pro/srv/TriggerGpsAlign";
}

template<>
struct has_fixed_size<automap_pro::srv::TriggerGpsAlign>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::srv::TriggerGpsAlign_Request>::value &&
    has_fixed_size<automap_pro::srv::TriggerGpsAlign_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::srv::TriggerGpsAlign>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::srv::TriggerGpsAlign_Request>::value &&
    has_bounded_size<automap_pro::srv::TriggerGpsAlign_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::srv::TriggerGpsAlign>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::srv::TriggerGpsAlign_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::srv::TriggerGpsAlign_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__TRAITS_HPP_
