// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from automap_pro:action/RunHBA.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
#include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "automap_pro/action/detail/run_hba__functions.h"
#include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `data_path`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_Goal__init(message_memory);
}

void automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_member_array[4] = {
  {
    "data_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Goal, data_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_layer_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Goal, total_layer_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "thread_num",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Goal, thread_num),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enable_gps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Goal, enable_gps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_Goal",  // message name
  4,  // number of fields
  sizeof(automap_pro__action__RunHBA_Goal),
  automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_member_array,  // message members
  automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_Goal)() {
  if (!automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_Goal__rosidl_typesupport_introspection_c__RunHBA_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `output_path`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_Result__init(message_memory);
}

void automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Result, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "output_path",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Result, output_path),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "elapsed_seconds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Result, elapsed_seconds),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_mme",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Result, final_mme),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_keyframes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Result, total_keyframes),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_Result",  // message name
  5,  // number of fields
  sizeof(automap_pro__action__RunHBA_Result),
  automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_member_array,  // message members
  automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_Result)() {
  if (!automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_Result__rosidl_typesupport_introspection_c__RunHBA_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_Feedback__init(message_memory);
}

void automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_member_array[4] = {
  {
    "current_layer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Feedback, current_layer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_layers",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Feedback, total_layers),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "progress_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Feedback, progress_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "layer_elapsed_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_Feedback, layer_elapsed_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_Feedback",  // message name
  4,  // number of fields
  sizeof(automap_pro__action__RunHBA_Feedback),
  automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_member_array,  // message members
  automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_Feedback)() {
  if (!automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_Feedback__rosidl_typesupport_introspection_c__RunHBA_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "automap_pro/action/run_hba.h"
// Member `goal`
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_SendGoal_Request__init(message_memory);
}

void automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(automap_pro__action__RunHBA_SendGoal_Request),
  automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_member_array,  // message members
  automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal_Request)() {
  automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_Goal)();
  if (!automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_SendGoal_Request__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_SendGoal_Response__init(message_memory);
}

void automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(automap_pro__action__RunHBA_SendGoal_Response),
  automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_member_array,  // message members
  automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal_Response)() {
  automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_SendGoal_Response__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_members = {
  "automap_pro__action",  // service namespace
  "RunHBA_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_type_support_handle = {
  0,
  &automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal)() {
  if (!automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_type_support_handle.typesupport_identifier) {
    automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_SendGoal_Response)()->data;
  }

  return &automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_GetResult_Request__init(message_memory);
}

void automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(automap_pro__action__RunHBA_GetResult_Request),
  automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_member_array,  // message members
  automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult_Request)() {
  automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_GetResult_Request__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "automap_pro/action/run_hba.h"
// Member `result`
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_GetResult_Response__init(message_memory);
}

void automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(automap_pro__action__RunHBA_GetResult_Response),
  automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_member_array,  // message members
  automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult_Response)() {
  automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_Result)();
  if (!automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_GetResult_Response__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_members = {
  "automap_pro__action",  // service namespace
  "RunHBA_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_type_support_handle = {
  0,
  &automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult)() {
  if (!automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_type_support_handle.typesupport_identifier) {
    automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_GetResult_Response)()->data;
  }

  return &automap_pro__action__detail__run_hba__rosidl_typesupport_introspection_c__RunHBA_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"
// already included above
// #include "automap_pro/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "automap_pro/action/run_hba.h"
// Member `feedback`
// already included above
// #include "automap_pro/action/detail/run_hba__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  automap_pro__action__RunHBA_FeedbackMessage__init(message_memory);
}

void automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_fini_function(void * message_memory)
{
  automap_pro__action__RunHBA_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(automap_pro__action__RunHBA_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_members = {
  "automap_pro__action",  // message namespace
  "RunHBA_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(automap_pro__action__RunHBA_FeedbackMessage),
  automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_member_array,  // message members
  automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_type_support_handle = {
  0,
  &automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_automap_pro
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_FeedbackMessage)() {
  automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, automap_pro, action, RunHBA_Feedback)();
  if (!automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &automap_pro__action__RunHBA_FeedbackMessage__rosidl_typesupport_introspection_c__RunHBA_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
