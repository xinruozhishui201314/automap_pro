// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from automap_pro:action/RunHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__TRAITS_HPP_
#define AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "automap_pro/action/detail/run_hba__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_Goal & msg,
  std::ostream & out)
{
  out << "{";
  // member: data_path
  {
    out << "data_path: ";
    rosidl_generator_traits::value_to_yaml(msg.data_path, out);
    out << ", ";
  }

  // member: total_layer_num
  {
    out << "total_layer_num: ";
    rosidl_generator_traits::value_to_yaml(msg.total_layer_num, out);
    out << ", ";
  }

  // member: thread_num
  {
    out << "thread_num: ";
    rosidl_generator_traits::value_to_yaml(msg.thread_num, out);
    out << ", ";
  }

  // member: enable_gps
  {
    out << "enable_gps: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_gps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_path: ";
    rosidl_generator_traits::value_to_yaml(msg.data_path, out);
    out << "\n";
  }

  // member: total_layer_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_layer_num: ";
    rosidl_generator_traits::value_to_yaml(msg.total_layer_num, out);
    out << "\n";
  }

  // member: thread_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thread_num: ";
    rosidl_generator_traits::value_to_yaml(msg.thread_num, out);
    out << "\n";
  }

  // member: enable_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_gps: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_gps, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_Goal & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_Goal & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_Goal & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_Goal>()
{
  return "automap_pro::action::RunHBA_Goal";
}

template<>
inline const char * name<automap_pro::action::RunHBA_Goal>()
{
  return "automap_pro/action/RunHBA_Goal";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::action::RunHBA_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_Result & msg,
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
    out << ", ";
  }

  // member: total_keyframes
  {
    out << "total_keyframes: ";
    rosidl_generator_traits::value_to_yaml(msg.total_keyframes, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_Result & msg,
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

  // member: total_keyframes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_keyframes: ";
    rosidl_generator_traits::value_to_yaml(msg.total_keyframes, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_Result & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_Result & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_Result & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_Result>()
{
  return "automap_pro::action::RunHBA_Result";
}

template<>
inline const char * name<automap_pro::action::RunHBA_Result>()
{
  return "automap_pro/action/RunHBA_Result";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_Result>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_Result>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<automap_pro::action::RunHBA_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_Feedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: current_layer
  {
    out << "current_layer: ";
    rosidl_generator_traits::value_to_yaml(msg.current_layer, out);
    out << ", ";
  }

  // member: total_layers
  {
    out << "total_layers: ";
    rosidl_generator_traits::value_to_yaml(msg.total_layers, out);
    out << ", ";
  }

  // member: progress_percent
  {
    out << "progress_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percent, out);
    out << ", ";
  }

  // member: layer_elapsed_ms
  {
    out << "layer_elapsed_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.layer_elapsed_ms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: current_layer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_layer: ";
    rosidl_generator_traits::value_to_yaml(msg.current_layer, out);
    out << "\n";
  }

  // member: total_layers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_layers: ";
    rosidl_generator_traits::value_to_yaml(msg.total_layers, out);
    out << "\n";
  }

  // member: progress_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.progress_percent, out);
    out << "\n";
  }

  // member: layer_elapsed_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "layer_elapsed_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.layer_elapsed_ms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_Feedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_Feedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_Feedback & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_Feedback>()
{
  return "automap_pro::action::RunHBA_Feedback";
}

template<>
inline const char * name<automap_pro::action::RunHBA_Feedback>()
{
  return "automap_pro/action/RunHBA_Feedback";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<automap_pro::action::RunHBA_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "automap_pro/action/detail/run_hba__traits.hpp"

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_SendGoal_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: goal
  {
    out << "goal: ";
    to_flow_style_yaml(msg.goal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: goal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal:\n";
    to_block_style_yaml(msg.goal, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_SendGoal_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_SendGoal_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_SendGoal_Request & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_SendGoal_Request>()
{
  return "automap_pro::action::RunHBA_SendGoal_Request";
}

template<>
inline const char * name<automap_pro::action::RunHBA_SendGoal_Request>()
{
  return "automap_pro/action/RunHBA_SendGoal_Request";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<automap_pro::action::RunHBA_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<automap_pro::action::RunHBA_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<automap_pro::action::RunHBA_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_SendGoal_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: accepted
  {
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << ", ";
  }

  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accepted
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accepted: ";
    rosidl_generator_traits::value_to_yaml(msg.accepted, out);
    out << "\n";
  }

  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_SendGoal_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_SendGoal_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_SendGoal_Response & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_SendGoal_Response>()
{
  return "automap_pro::action::RunHBA_SendGoal_Response";
}

template<>
inline const char * name<automap_pro::action::RunHBA_SendGoal_Response>()
{
  return "automap_pro/action/RunHBA_SendGoal_Response";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<automap_pro::action::RunHBA_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::action::RunHBA_SendGoal>()
{
  return "automap_pro::action::RunHBA_SendGoal";
}

template<>
inline const char * name<automap_pro::action::RunHBA_SendGoal>()
{
  return "automap_pro/action/RunHBA_SendGoal";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::action::RunHBA_SendGoal_Request>::value &&
    has_fixed_size<automap_pro::action::RunHBA_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::action::RunHBA_SendGoal_Request>::value &&
    has_bounded_size<automap_pro::action::RunHBA_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::action::RunHBA_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::action::RunHBA_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::action::RunHBA_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_GetResult_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_GetResult_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_GetResult_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_GetResult_Request & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_GetResult_Request>()
{
  return "automap_pro::action::RunHBA_GetResult_Request";
}

template<>
inline const char * name<automap_pro::action::RunHBA_GetResult_Request>()
{
  return "automap_pro/action/RunHBA_GetResult_Request";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<automap_pro::action::RunHBA_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "automap_pro/action/detail/run_hba__traits.hpp"

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_GetResult_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: result
  {
    out << "result: ";
    to_flow_style_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result:\n";
    to_block_style_yaml(msg.result, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_GetResult_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_GetResult_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_GetResult_Response & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_GetResult_Response>()
{
  return "automap_pro::action::RunHBA_GetResult_Response";
}

template<>
inline const char * name<automap_pro::action::RunHBA_GetResult_Response>()
{
  return "automap_pro/action/RunHBA_GetResult_Response";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<automap_pro::action::RunHBA_Result>::value> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<automap_pro::action::RunHBA_Result>::value> {};

template<>
struct is_message<automap_pro::action::RunHBA_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<automap_pro::action::RunHBA_GetResult>()
{
  return "automap_pro::action::RunHBA_GetResult";
}

template<>
inline const char * name<automap_pro::action::RunHBA_GetResult>()
{
  return "automap_pro/action/RunHBA_GetResult";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<automap_pro::action::RunHBA_GetResult_Request>::value &&
    has_fixed_size<automap_pro::action::RunHBA_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<automap_pro::action::RunHBA_GetResult_Request>::value &&
    has_bounded_size<automap_pro::action::RunHBA_GetResult_Response>::value
  >
{
};

template<>
struct is_service<automap_pro::action::RunHBA_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<automap_pro::action::RunHBA_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<automap_pro::action::RunHBA_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "automap_pro/action/detail/run_hba__traits.hpp"

namespace automap_pro
{

namespace action
{

inline void to_flow_style_yaml(
  const RunHBA_FeedbackMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: goal_id
  {
    out << "goal_id: ";
    to_flow_style_yaml(msg.goal_id, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    to_flow_style_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RunHBA_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: goal_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "goal_id:\n";
    to_block_style_yaml(msg.goal_id, out, indentation + 2);
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback:\n";
    to_block_style_yaml(msg.feedback, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RunHBA_FeedbackMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace action

}  // namespace automap_pro

namespace rosidl_generator_traits
{

[[deprecated("use automap_pro::action::to_block_style_yaml() instead")]]
inline void to_yaml(
  const automap_pro::action::RunHBA_FeedbackMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  automap_pro::action::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use automap_pro::action::to_yaml() instead")]]
inline std::string to_yaml(const automap_pro::action::RunHBA_FeedbackMessage & msg)
{
  return automap_pro::action::to_yaml(msg);
}

template<>
inline const char * data_type<automap_pro::action::RunHBA_FeedbackMessage>()
{
  return "automap_pro::action::RunHBA_FeedbackMessage";
}

template<>
inline const char * name<automap_pro::action::RunHBA_FeedbackMessage>()
{
  return "automap_pro/action/RunHBA_FeedbackMessage";
}

template<>
struct has_fixed_size<automap_pro::action::RunHBA_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<automap_pro::action::RunHBA_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<automap_pro::action::RunHBA_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<automap_pro::action::RunHBA_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<automap_pro::action::RunHBA_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<automap_pro::action::RunHBA>
  : std::true_type
{
};

template<>
struct is_action_goal<automap_pro::action::RunHBA_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<automap_pro::action::RunHBA_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<automap_pro::action::RunHBA_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__TRAITS_HPP_
