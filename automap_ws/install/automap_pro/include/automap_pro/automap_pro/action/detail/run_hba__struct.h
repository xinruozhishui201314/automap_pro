// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:action/RunHBA.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__STRUCT_H_
#define AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data_path'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_Goal
{
  rosidl_runtime_c__String data_path;
  int32_t total_layer_num;
  int32_t thread_num;
  bool enable_gps;
} automap_pro__action__RunHBA_Goal;

// Struct for a sequence of automap_pro__action__RunHBA_Goal.
typedef struct automap_pro__action__RunHBA_Goal__Sequence
{
  automap_pro__action__RunHBA_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'output_path'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_Result
{
  bool success;
  rosidl_runtime_c__String output_path;
  double elapsed_seconds;
  double final_mme;
  int32_t total_keyframes;
} automap_pro__action__RunHBA_Result;

// Struct for a sequence of automap_pro__action__RunHBA_Result.
typedef struct automap_pro__action__RunHBA_Result__Sequence
{
  automap_pro__action__RunHBA_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_Feedback
{
  int32_t current_layer;
  int32_t total_layers;
  float progress_percent;
  float layer_elapsed_ms;
} automap_pro__action__RunHBA_Feedback;

// Struct for a sequence of automap_pro__action__RunHBA_Feedback.
typedef struct automap_pro__action__RunHBA_Feedback__Sequence
{
  automap_pro__action__RunHBA_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "automap_pro/action/detail/run_hba__struct.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  automap_pro__action__RunHBA_Goal goal;
} automap_pro__action__RunHBA_SendGoal_Request;

// Struct for a sequence of automap_pro__action__RunHBA_SendGoal_Request.
typedef struct automap_pro__action__RunHBA_SendGoal_Request__Sequence
{
  automap_pro__action__RunHBA_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} automap_pro__action__RunHBA_SendGoal_Response;

// Struct for a sequence of automap_pro__action__RunHBA_SendGoal_Response.
typedef struct automap_pro__action__RunHBA_SendGoal_Response__Sequence
{
  automap_pro__action__RunHBA_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} automap_pro__action__RunHBA_GetResult_Request;

// Struct for a sequence of automap_pro__action__RunHBA_GetResult_Request.
typedef struct automap_pro__action__RunHBA_GetResult_Request__Sequence
{
  automap_pro__action__RunHBA_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_GetResult_Response
{
  int8_t status;
  automap_pro__action__RunHBA_Result result;
} automap_pro__action__RunHBA_GetResult_Response;

// Struct for a sequence of automap_pro__action__RunHBA_GetResult_Response.
typedef struct automap_pro__action__RunHBA_GetResult_Response__Sequence
{
  automap_pro__action__RunHBA_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "automap_pro/action/detail/run_hba__struct.h"

/// Struct defined in action/RunHBA in the package automap_pro.
typedef struct automap_pro__action__RunHBA_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  automap_pro__action__RunHBA_Feedback feedback;
} automap_pro__action__RunHBA_FeedbackMessage;

// Struct for a sequence of automap_pro__action__RunHBA_FeedbackMessage.
typedef struct automap_pro__action__RunHBA_FeedbackMessage__Sequence
{
  automap_pro__action__RunHBA_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__action__RunHBA_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__ACTION__DETAIL__RUN_HBA__STRUCT_H_
