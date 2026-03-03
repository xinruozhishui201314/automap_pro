// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from automap_pro:msg/KeyFrameInfoMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "automap_pro/msg/detail/key_frame_info_msg__rosidl_typesupport_introspection_c.h"
#include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "automap_pro/msg/detail/key_frame_info_msg__functions.h"
#include "automap_pro/msg/detail/key_frame_info_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose_with_covariance.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__msg__KeyFrameInfoMsg__init(message_memory);
}

void automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_fini_function(void * message_memory)
{
  automap_pro__msg__KeyFrameInfoMsg__fini(message_memory);
}

size_t automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__size_function__KeyFrameInfoMsg__gyro_bias(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_const_function__KeyFrameInfoMsg__gyro_bias(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_function__KeyFrameInfoMsg__gyro_bias(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__fetch_function__KeyFrameInfoMsg__gyro_bias(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_const_function__KeyFrameInfoMsg__gyro_bias(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__assign_function__KeyFrameInfoMsg__gyro_bias(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_function__KeyFrameInfoMsg__gyro_bias(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__size_function__KeyFrameInfoMsg__accel_bias(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_const_function__KeyFrameInfoMsg__accel_bias(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_function__KeyFrameInfoMsg__accel_bias(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__fetch_function__KeyFrameInfoMsg__accel_bias(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_const_function__KeyFrameInfoMsg__accel_bias(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__assign_function__KeyFrameInfoMsg__accel_bias(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_function__KeyFrameInfoMsg__accel_bias(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_member_array[11] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "keyframe_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, keyframe_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "esikf_covariance_norm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, esikf_covariance_norm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_degenerate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, is_degenerate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "map_update_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, map_update_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gyro_bias",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, gyro_bias),  // bytes offset in struct
    NULL,  // default value
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__size_function__KeyFrameInfoMsg__gyro_bias,  // size() function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_const_function__KeyFrameInfoMsg__gyro_bias,  // get_const(index) function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_function__KeyFrameInfoMsg__gyro_bias,  // get(index) function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__fetch_function__KeyFrameInfoMsg__gyro_bias,  // fetch(index, &value) function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__assign_function__KeyFrameInfoMsg__gyro_bias,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "accel_bias",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, accel_bias),  // bytes offset in struct
    NULL,  // default value
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__size_function__KeyFrameInfoMsg__accel_bias,  // size() function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_const_function__KeyFrameInfoMsg__accel_bias,  // get_const(index) function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__get_function__KeyFrameInfoMsg__accel_bias,  // get(index) function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__fetch_function__KeyFrameInfoMsg__accel_bias,  // fetch(index, &value) function pointer
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__assign_function__KeyFrameInfoMsg__accel_bias,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud_point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, cloud_point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, cloud_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameInfoMsg, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_members = {
  "automap_pro__msg",  // message namespace
  "KeyFrameInfoMsg",  // message name
  11,  // number of fields
  sizeof(automap_pro__msg__KeyFrameInfoMsg),
  automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_member_array,  // message members
  automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_type_support_handle = {
  0,
  &automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, msg, KeyFrameInfoMsg)() {
  automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseWithCovariance)();
  if (!automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_type_support_handle.typesupport_identifier) {
    automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__msg__KeyFrameInfoMsg__rosidl_typesupport_introspection_c__KeyFrameInfoMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
