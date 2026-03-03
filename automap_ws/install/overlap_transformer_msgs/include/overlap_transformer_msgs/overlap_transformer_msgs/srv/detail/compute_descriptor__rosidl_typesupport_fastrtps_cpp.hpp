// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice

#ifndef OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "overlap_transformer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "overlap_transformer_msgs/srv/detail/compute_descriptor__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace overlap_transformer_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
cdr_serialize(
  const overlap_transformer_msgs::srv::ComputeDescriptor_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  overlap_transformer_msgs::srv::ComputeDescriptor_Request & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
get_serialized_size(
  const overlap_transformer_msgs::srv::ComputeDescriptor_Request & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
max_serialized_size_ComputeDescriptor_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace overlap_transformer_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, overlap_transformer_msgs, srv, ComputeDescriptor_Request)();

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "overlap_transformer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
// already included above
// #include "overlap_transformer_msgs/srv/detail/compute_descriptor__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// already included above
// #include "fastcdr/Cdr.h"

namespace overlap_transformer_msgs
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
cdr_serialize(
  const overlap_transformer_msgs::srv::ComputeDescriptor_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  overlap_transformer_msgs::srv::ComputeDescriptor_Response & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
get_serialized_size(
  const overlap_transformer_msgs::srv::ComputeDescriptor_Response & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
max_serialized_size_ComputeDescriptor_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace overlap_transformer_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, overlap_transformer_msgs, srv, ComputeDescriptor_Response)();

#ifdef __cplusplus
}
#endif

#include "rmw/types.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "overlap_transformer_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_overlap_transformer_msgs
const rosidl_service_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, overlap_transformer_msgs, srv, ComputeDescriptor)();

#ifdef __cplusplus
}
#endif

#endif  // OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
