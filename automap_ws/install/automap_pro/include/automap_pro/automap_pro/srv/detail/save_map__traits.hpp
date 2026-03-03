// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
#define AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/srv/detail/save_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: output_dir
  {
    out << "output_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.output_dir, out);
    out << ", ";
  }

  // member: save_pcd
  {
    out << "save_pcd: ";
    rosidl_generator_traits::value_to_yaml(msg.save_pcd, out);
    out << ", ";
  }

  // member: save_ply
  {
    out << "save_ply: ";
    rosidl_generator_traits::value_to_yaml(msg.save_ply, out);
    out << ", ";
  }

  // member: save_las
  {
    out << "save_las: ";
    rosidl_generator_traits::value_to_yaml(msg.save_las, out);
    out << ", ";
  }

  // member: save_trajectory_tum
  {
    out << "save_trajectory_tum: ";
    rosidl_generator_traits::value_to_yaml(msg.save_trajectory_tum, out);
    out << ", ";
  }

  // member: save_trajectory_kitti
  {
    out << "save_trajectory_kitti: ";
    rosidl_generator_traits::value_to_yaml(msg.save_trajectory_kitti, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SaveMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: output_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.output_dir, out);
    out << "\n";
  }

  // member: save_pcd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_pcd: ";
    rosidl_generator_traits::value_to_yaml(msg.save_pcd, out);
    out << "\n";
  }

  // member: save_ply
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_ply: ";
    rosidl_generator_traits::value_to_yaml(msg.save_ply, out);
    out << "\n";
  }

  // member: save_las
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_las: ";
    rosidl_generator_traits::value_to_yaml(msg.save_las, out);
    out << "\n";
  }

  // member: save_trajectory_tum
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_trajectory_tum: ";
    rosidl_generator_traits::value_to_yaml(msg.save_trajectory_tum, out);
    out << "\n";
  }

  // member: save_trajectory_kitti
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_trajectory_kitti: ";
    rosidl_generator_traits::value_to_yaml(msg.save_trajectory_kitti, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SaveMap_Request & msg, bool use_flow_style = false)
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
  const automap_pro::srv::SaveMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::SaveMap_Request & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::SaveMap_Request>()
{
  return "automap_pro::srv::SaveMap_Request";
}

template<>
inline const char * name<automap_pro::srv::SaveMap_Request>()
{
  return "automap_pro/srv/SaveMap_Request";
}

template<>
struct has_fixed_size<automap_pro::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::SaveMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::SaveMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace srv
{

inline void to_flow_style_yaml(
  const SaveMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: output_path
  {
    out << "output_path: ";
    rosidl_generator_traits::value_to_yaml(msg.output_path, out);
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
  const SaveMap_Response & msg,
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

  // member: output_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "output_path: ";
    rosidl_generator_traits::value_to_yaml(msg.output_path, out);
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

inline std::string to_yaml(const SaveMap_Response & msg, bool use_flow_style = false)
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
  const automap_pro::srv::SaveMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::srv::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::srv::SaveMap_Response & msg)
{
  return automap_pro::srv::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::srv::SaveMap_Response>()
{
  return "automap_pro::srv::SaveMap_Response";
}

template<>
inline const char * name<automap_pro::srv::SaveMap_Response>()
{
  return "automap_pro/srv/SaveMap_Response";
}

template<>
struct has_fixed_size<automap_pro::srv::SaveMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::srv::SaveMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::srv::SaveMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::srv::SaveMap>()
{
  return "automap_pro::srv::SaveMap";
}

template<>
inline const char * name<automap_pro::srv::SaveMap>()
{
  return "automap_pro/srv/SaveMap";
}

template<>
struct has_fixed_size<automap_pro::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::srv::SaveMap_Request>::value &&
    has_fixed_size<automap_pro::srv::SaveMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::srv::SaveMap>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::srv::SaveMap_Request>::value &&
    has_bounded_size<automap_pro::srv::SaveMap_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::srv::SaveMap>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::srv::SaveMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::srv::SaveMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // AUTOMAP_PRO__SRV__DETAIL__SAVE_MAP__TRAITS_HPP_
