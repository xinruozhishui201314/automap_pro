// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "overlap_transformer_msgs/srv/detail/compute_descriptor__rosidl_typesupport_introspection_c.h"
#include "overlap_transformer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "overlap_transformer_msgs/srv/detail/compute_descriptor__functions.h"
#include "overlap_transformer_msgs/srv/detail/compute_descriptor__struct.h"


// Include directives for member types
// Member `pointcloud`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `pointcloud`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__init(message_memory);
}

void overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_fini_function(void * message_memory)
{
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_member_array[1] = {
  {
    "pointcloud",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(overlap_transformer_msgs__srv__ComputeDescriptor_Request, pointcloud),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_members = {
  "overlap_transformer_msgs__srv",  // message namespace
  "ComputeDescriptor_Request",  // message name
  1,  // number of fields
  sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Request),
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_member_array,  // message members
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_type_support_handle = {
  0,
  &overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_overlap_transformer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor_Request)() {
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_type_support_handle.typesupport_identifier) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &overlap_transformer_msgs__srv__ComputeDescriptor_Request__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "overlap_transformer_msgs/srv/detail/compute_descriptor__rosidl_typesupport_introspection_c.h"
// already included above
// #include "overlap_transformer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "overlap_transformer_msgs/srv/detail/compute_descriptor__functions.h"
// already included above
// #include "overlap_transformer_msgs/srv/detail/compute_descriptor__struct.h"


// Include directives for member types
// Member `descriptor`
#include "std_msgs/msg/float32_multi_array.h"
// Member `descriptor`
#include "std_msgs/msg/detail/float32_multi_array__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__init(message_memory);
}

void overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_fini_function(void * message_memory)
{
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_member_array[1] = {
  {
    "descriptor",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(overlap_transformer_msgs__srv__ComputeDescriptor_Response, descriptor),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_members = {
  "overlap_transformer_msgs__srv",  // message namespace
  "ComputeDescriptor_Response",  // message name
  1,  // number of fields
  sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Response),
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_member_array,  // message members
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_type_support_handle = {
  0,
  &overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_overlap_transformer_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor_Response)() {
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Float32MultiArray)();
  if (!overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_type_support_handle.typesupport_identifier) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &overlap_transformer_msgs__srv__ComputeDescriptor_Response__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "overlap_transformer_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "overlap_transformer_msgs/srv/detail/compute_descriptor__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_members = {
  "overlap_transformer_msgs__srv",  // service namespace
  "ComputeDescriptor",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_Request_message_type_support_handle,
  NULL  // response message
  // overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_Response_message_type_support_handle
};

static rosidl_service_type_support_t overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_type_support_handle = {
  0,
  &overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_overlap_transformer_msgs
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor)() {
  if (!overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_type_support_handle.typesupport_identifier) {
    overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, overlap_transformer_msgs, srv, ComputeDescriptor_Response)()->data;
  }

  return &overlap_transformer_msgs__srv__detail__compute_descriptor__rosidl_typesupport_introspection_c__ComputeDescriptor_service_type_support_handle;
}
