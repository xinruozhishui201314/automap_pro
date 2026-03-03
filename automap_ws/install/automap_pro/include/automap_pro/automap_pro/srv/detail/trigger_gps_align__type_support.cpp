// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "automap_pro/srv/detail/trigger_gps_align__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace automap_pro
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void TriggerGpsAlign_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) automap_pro::srv::TriggerGpsAlign_Request(_init);
}

void TriggerGpsAlign_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<automap_pro::srv::TriggerGpsAlign_Request *>(message_memory);
  typed_message->~TriggerGpsAlign_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TriggerGpsAlign_Request_message_member_array[1] = {
  {
    "force",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro::srv::TriggerGpsAlign_Request, force),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TriggerGpsAlign_Request_message_members = {
  "automap_pro::srv",  // message namespace
  "TriggerGpsAlign_Request",  // message name
  1,  // number of fields
  sizeof(automap_pro::srv::TriggerGpsAlign_Request),
  TriggerGpsAlign_Request_message_member_array,  // message members
  TriggerGpsAlign_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TriggerGpsAlign_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TriggerGpsAlign_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TriggerGpsAlign_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace automap_pro


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<automap_pro::srv::TriggerGpsAlign_Request>()
{
  return &::automap_pro::srv::rosidl_typesupport_introspection_cpp::TriggerGpsAlign_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, automap_pro, srv, TriggerGpsAlign_Request)() {
  return &::automap_pro::srv::rosidl_typesupport_introspection_cpp::TriggerGpsAlign_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "automap_pro/srv/detail/trigger_gps_align__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace automap_pro
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void TriggerGpsAlign_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) automap_pro::srv::TriggerGpsAlign_Response(_init);
}

void TriggerGpsAlign_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<automap_pro::srv::TriggerGpsAlign_Response *>(message_memory);
  typed_message->~TriggerGpsAlign_Response();
}

size_t size_function__TriggerGpsAlign_Response__r_gps_lidar(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__TriggerGpsAlign_Response__r_gps_lidar(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__TriggerGpsAlign_Response__r_gps_lidar(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void fetch_function__TriggerGpsAlign_Response__r_gps_lidar(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TriggerGpsAlign_Response__r_gps_lidar(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TriggerGpsAlign_Response__r_gps_lidar(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TriggerGpsAlign_Response__r_gps_lidar(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TriggerGpsAlign_Response__t_gps_lidar(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__TriggerGpsAlign_Response__t_gps_lidar(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__TriggerGpsAlign_Response__t_gps_lidar(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__TriggerGpsAlign_Response__t_gps_lidar(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TriggerGpsAlign_Response__t_gps_lidar(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TriggerGpsAlign_Response__t_gps_lidar(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TriggerGpsAlign_Response__t_gps_lidar(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TriggerGpsAlign_Response_message_member_array[5] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro::srv::TriggerGpsAlign_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "alignment_rmse_m",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro::srv::TriggerGpsAlign_Response, alignment_rmse_m),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "r_gps_lidar",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(automap_pro::srv::TriggerGpsAlign_Response, r_gps_lidar),  // bytes offset in struct
    nullptr,  // default value
    size_function__TriggerGpsAlign_Response__r_gps_lidar,  // size() function pointer
    get_const_function__TriggerGpsAlign_Response__r_gps_lidar,  // get_const(index) function pointer
    get_function__TriggerGpsAlign_Response__r_gps_lidar,  // get(index) function pointer
    fetch_function__TriggerGpsAlign_Response__r_gps_lidar,  // fetch(index, &value) function pointer
    assign_function__TriggerGpsAlign_Response__r_gps_lidar,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "t_gps_lidar",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(automap_pro::srv::TriggerGpsAlign_Response, t_gps_lidar),  // bytes offset in struct
    nullptr,  // default value
    size_function__TriggerGpsAlign_Response__t_gps_lidar,  // size() function pointer
    get_const_function__TriggerGpsAlign_Response__t_gps_lidar,  // get_const(index) function pointer
    get_function__TriggerGpsAlign_Response__t_gps_lidar,  // get(index) function pointer
    fetch_function__TriggerGpsAlign_Response__t_gps_lidar,  // fetch(index, &value) function pointer
    assign_function__TriggerGpsAlign_Response__t_gps_lidar,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "message",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro::srv::TriggerGpsAlign_Response, message),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TriggerGpsAlign_Response_message_members = {
  "automap_pro::srv",  // message namespace
  "TriggerGpsAlign_Response",  // message name
  5,  // number of fields
  sizeof(automap_pro::srv::TriggerGpsAlign_Response),
  TriggerGpsAlign_Response_message_member_array,  // message members
  TriggerGpsAlign_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TriggerGpsAlign_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TriggerGpsAlign_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TriggerGpsAlign_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace automap_pro


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<automap_pro::srv::TriggerGpsAlign_Response>()
{
  return &::automap_pro::srv::rosidl_typesupport_introspection_cpp::TriggerGpsAlign_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, automap_pro, srv, TriggerGpsAlign_Response)() {
  return &::automap_pro::srv::rosidl_typesupport_introspection_cpp::TriggerGpsAlign_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "automap_pro/srv/detail/trigger_gps_align__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace automap_pro
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers TriggerGpsAlign_service_members = {
  "automap_pro::srv",  // service namespace
  "TriggerGpsAlign",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<automap_pro::srv::TriggerGpsAlign>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t TriggerGpsAlign_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TriggerGpsAlign_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace automap_pro


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<automap_pro::srv::TriggerGpsAlign>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::automap_pro::srv::rosidl_typesupport_introspection_cpp::TriggerGpsAlign_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::automap_pro::srv::TriggerGpsAlign_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::automap_pro::srv::TriggerGpsAlign_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, automap_pro, srv, TriggerGpsAlign)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<automap_pro::srv::TriggerGpsAlign>();
}

#ifdef __cplusplus
}
#endif
