// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "automap_pro/srv/detail/trigger_gps_align__rosidl_typesupport_introspection_c.h"
#include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "automap_pro/srv/detail/trigger_gps_align__functions.h"
#include "automap_pro/srv/detail/trigger_gps_align__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__srv__TriggerGpsAlign_Request__init(message_memory);
}

void automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_fini_function(void * message_memory)
{
  automap_pro__srv__TriggerGpsAlign_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_member_array[1] = {
  {
    "force",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__srv__TriggerGpsAlign_Request, force),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_members = {
  "automap_pro__srv",  // message namespace
  "TriggerGpsAlign_Request",  // message name
  1,  // number of fields
  sizeof(automap_pro__srv__TriggerGpsAlign_Request),
  automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_member_array,  // message members
  automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_type_support_handle = {
  0,
  &automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign_Request)() {
  if (!automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_type_support_handle.typesupport_identifier) {
    automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__srv__TriggerGpsAlign_Request__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/srv/detail/trigger_gps_align__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/srv/detail/trigger_gps_align__functions.h"
// already included above
// #include "automap_pro/srv/detail/trigger_gps_align__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__srv__TriggerGpsAlign_Response__init(message_memory);
}

void automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_fini_function(void * message_memory)
{
  automap_pro__srv__TriggerGpsAlign_Response__fini(message_memory);
}

size_t automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__size_function__TriggerGpsAlign_Response__r_gps_lidar(
  const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_const_function__TriggerGpsAlign_Response__r_gps_lidar(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_function__TriggerGpsAlign_Response__r_gps_lidar(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__fetch_function__TriggerGpsAlign_Response__r_gps_lidar(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_const_function__TriggerGpsAlign_Response__r_gps_lidar(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__assign_function__TriggerGpsAlign_Response__r_gps_lidar(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_function__TriggerGpsAlign_Response__r_gps_lidar(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__size_function__TriggerGpsAlign_Response__t_gps_lidar(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_const_function__TriggerGpsAlign_Response__t_gps_lidar(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_function__TriggerGpsAlign_Response__t_gps_lidar(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__fetch_function__TriggerGpsAlign_Response__t_gps_lidar(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_const_function__TriggerGpsAlign_Response__t_gps_lidar(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__assign_function__TriggerGpsAlign_Response__t_gps_lidar(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_function__TriggerGpsAlign_Response__t_gps_lidar(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__srv__TriggerGpsAlign_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "alignment_rmse_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__srv__TriggerGpsAlign_Response, alignment_rmse_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "r_gps_lidar",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(automap_pro__srv__TriggerGpsAlign_Response, r_gps_lidar),  // bytes offset in struct
    NULL,  // default value
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__size_function__TriggerGpsAlign_Response__r_gps_lidar,  // size() function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_const_function__TriggerGpsAlign_Response__r_gps_lidar,  // get_const(index) function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_function__TriggerGpsAlign_Response__r_gps_lidar,  // get(index) function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__fetch_function__TriggerGpsAlign_Response__r_gps_lidar,  // fetch(index, &value) function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__assign_function__TriggerGpsAlign_Response__r_gps_lidar,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "t_gps_lidar",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(automap_pro__srv__TriggerGpsAlign_Response, t_gps_lidar),  // bytes offset in struct
    NULL,  // default value
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__size_function__TriggerGpsAlign_Response__t_gps_lidar,  // size() function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_const_function__TriggerGpsAlign_Response__t_gps_lidar,  // get_const(index) function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__get_function__TriggerGpsAlign_Response__t_gps_lidar,  // get(index) function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__fetch_function__TriggerGpsAlign_Response__t_gps_lidar,  // fetch(index, &value) function pointer
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__assign_function__TriggerGpsAlign_Response__t_gps_lidar,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__srv__TriggerGpsAlign_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_members = {
  "automap_pro__srv",  // message namespace
  "TriggerGpsAlign_Response",  // message name
  5,  // number of fields
  sizeof(automap_pro__srv__TriggerGpsAlign_Response),
  automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_member_array,  // message members
  automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_type_support_handle = {
  0,
  &automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign_Response)() {
  if (!automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_type_support_handle.typesupport_identifier) {
    automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__srv__TriggerGpsAlign_Response__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "automap_pro/srv/detail/trigger_gps_align__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_members = {
  "automap_pro__srv",  // service namespace
  "TriggerGpsAlign",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_Request_message_type_support_handle,
  NULL  // response message
  // automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_Response_message_type_support_handle
};

static rosidl_service_type_support_t automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_type_support_handle = {
  0,
  &automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign)() {
  if (!automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_type_support_handle.typesupport_identifier) {
    automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, srv, TriggerGpsAlign_Response)()->data;
  }

  return &automap_pro__srv__detail__trigger_gps_align__rosidl_typesupport_introspection_c__TriggerGpsAlign_service_type_support_handle;
}
