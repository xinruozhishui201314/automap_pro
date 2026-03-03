// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from automap_pro:msg/KeyFrameMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "automap_pro/msg/detail/key_frame_msg__rosidl_typesupport_introspection_c.h"
#include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "automap_pro/msg/detail/key_frame_msg__functions.h"
#include "automap_pro/msg/detail/key_frame_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose_with_covariance.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__rosidl_typesupport_introspection_c.h"
// Member `cloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"
// Member `gps`
#include "automap_pro/msg/gps_measurement_msg.h"
// Member `gps`
#include "automap_pro/msg/detail/gps_measurement_msg__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__msg__KeyFrameMsg__init(message_memory);
}

void automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_fini_function(void * message_memory)
{
  automap_pro__msg__KeyFrameMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "session_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, session_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "submap_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, submap_id),  // bytes offset in struct
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
    offsetof(automap_pro__msg__KeyFrameMsg, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, cloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_gps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, has_gps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__KeyFrameMsg, gps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_members = {
  "automap_pro__msg",  // message namespace
  "KeyFrameMsg",  // message name
  8,  // number of fields
  sizeof(automap_pro__msg__KeyFrameMsg),
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_member_array,  // message members
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_type_support_handle = {
  0,
  &automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, msg, KeyFrameMsg)() {
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseWithCovariance)();
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, msg, GPSMeasurementMsg)();
  if (!automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_type_support_handle.typesupport_identifier) {
    automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__msg__KeyFrameMsg__rosidl_typesupport_introspection_c__KeyFrameMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
