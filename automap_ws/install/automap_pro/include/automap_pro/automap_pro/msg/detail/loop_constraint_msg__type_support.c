// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "automap_pro/msg/detail/loop_constraint_msg__rosidl_typesupport_introspection_c.h"
#include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "automap_pro/msg/detail/loop_constraint_msg__functions.h"
#include "automap_pro/msg/detail/loop_constraint_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `delta_pose`
#include "geometry_msgs/msg/pose.h"
// Member `delta_pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__msg__LoopConstraintMsg__init(message_memory);
}

void automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_fini_function(void * message_memory)
{
  automap_pro__msg__LoopConstraintMsg__fini(message_memory);
}

size_t automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__size_function__LoopConstraintMsg__information_matrix(
  const void * untyped_member)
{
  (void)untyped_member;
  return 36;
}

const void * automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__get_const_function__LoopConstraintMsg__information_matrix(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__get_function__LoopConstraintMsg__information_matrix(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__fetch_function__LoopConstraintMsg__information_matrix(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__get_const_function__LoopConstraintMsg__information_matrix(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__assign_function__LoopConstraintMsg__information_matrix(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__get_function__LoopConstraintMsg__information_matrix(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_member_array[12] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "submap_i",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, submap_i),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "submap_j",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, submap_j),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "session_i",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, session_i),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "session_j",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, session_j),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "overlap_score",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, overlap_score),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "inlier_ratio",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, inlier_ratio),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rmse",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, rmse),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_inter_session",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, is_inter_session),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "information_matrix",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    36,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, information_matrix),  // bytes offset in struct
    NULL,  // default value
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__size_function__LoopConstraintMsg__information_matrix,  // size() function pointer
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__get_const_function__LoopConstraintMsg__information_matrix,  // get_const(index) function pointer
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__get_function__LoopConstraintMsg__information_matrix,  // get(index) function pointer
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__fetch_function__LoopConstraintMsg__information_matrix,  // fetch(index, &value) function pointer
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__assign_function__LoopConstraintMsg__information_matrix,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "delta_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, delta_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__LoopConstraintMsg, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_members = {
  "automap_pro__msg",  // message namespace
  "LoopConstraintMsg",  // message name
  12,  // number of fields
  sizeof(automap_pro__msg__LoopConstraintMsg),
  automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_member_array,  // message members
  automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_type_support_handle = {
  0,
  &automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, msg, LoopConstraintMsg)() {
  automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_type_support_handle.typesupport_identifier) {
    automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__msg__LoopConstraintMsg__rosidl_typesupport_introspection_c__LoopConstraintMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
