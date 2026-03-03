// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from automap_pro:msg/GPSMeasurementMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "automap_pro/msg/detail/gps_measurement_msg__rosidl_typesupport_introspection_c.h"
#include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "automap_pro/msg/detail/gps_measurement_msg__functions.h"
#include "automap_pro/msg/detail/gps_measurement_msg__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__msg__GPSMeasurementMsg__init(message_memory);
}

void automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_fini_function(void * message_memory)
{
  automap_pro__msg__GPSMeasurementMsg__fini(message_memory);
}

size_t automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__size_function__GPSMeasurementMsg__position_enu(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_const_function__GPSMeasurementMsg__position_enu(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_function__GPSMeasurementMsg__position_enu(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__fetch_function__GPSMeasurementMsg__position_enu(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_const_function__GPSMeasurementMsg__position_enu(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__assign_function__GPSMeasurementMsg__position_enu(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_function__GPSMeasurementMsg__position_enu(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__size_function__GPSMeasurementMsg__covariance(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_const_function__GPSMeasurementMsg__covariance(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_function__GPSMeasurementMsg__covariance(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__fetch_function__GPSMeasurementMsg__covariance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_const_function__GPSMeasurementMsg__covariance(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__assign_function__GPSMeasurementMsg__covariance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_function__GPSMeasurementMsg__covariance(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_enu",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, position_enu),  // bytes offset in struct
    NULL,  // default value
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__size_function__GPSMeasurementMsg__position_enu,  // size() function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_const_function__GPSMeasurementMsg__position_enu,  // get_const(index) function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_function__GPSMeasurementMsg__position_enu,  // get(index) function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__fetch_function__GPSMeasurementMsg__position_enu,  // fetch(index, &value) function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__assign_function__GPSMeasurementMsg__position_enu,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "quality",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, quality),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hdop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, hdop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_satellites",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, num_satellites),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "covariance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, covariance),  // bytes offset in struct
    NULL,  // default value
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__size_function__GPSMeasurementMsg__covariance,  // size() function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_const_function__GPSMeasurementMsg__covariance,  // get_const(index) function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__get_function__GPSMeasurementMsg__covariance,  // get(index) function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__fetch_function__GPSMeasurementMsg__covariance,  // fetch(index, &value) function pointer
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__assign_function__GPSMeasurementMsg__covariance,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, is_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__msg__GPSMeasurementMsg, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_members = {
  "automap_pro__msg",  // message namespace
  "GPSMeasurementMsg",  // message name
  10,  // number of fields
  sizeof(automap_pro__msg__GPSMeasurementMsg),
  automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_member_array,  // message members
  automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_type_support_handle = {
  0,
  &automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, msg, GPSMeasurementMsg)() {
  automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_type_support_handle.typesupport_identifier) {
    automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__msg__GPSMeasurementMsg__rosidl_typesupport_introspection_c__GPSMeasurementMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
