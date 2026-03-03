// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:srv/TriggerOptimize.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__STRUCT_H_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/TriggerOptimize in the package automap_pro.
typedef struct automap_pro__srv__TriggerOptimize_Request
{
  bool full_optimization;
  int32_t max_iterations;
} automap_pro__srv__TriggerOptimize_Request;

// Struct for a sequence of automap_pro__srv__TriggerOptimize_Request.
typedef struct automap_pro__srv__TriggerOptimize_Request__Sequence
{
  automap_pro__srv__TriggerOptimize_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__srv__TriggerOptimize_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/TriggerOptimize in the package automap_pro.
typedef struct automap_pro__srv__TriggerOptimize_Response
{
  bool success;
  double elapsed_seconds;
  int32_t nodes_updated;
} automap_pro__srv__TriggerOptimize_Response;

// Struct for a sequence of automap_pro__srv__TriggerOptimize_Response.
typedef struct automap_pro__srv__TriggerOptimize_Response__Sequence
{
  automap_pro__srv__TriggerOptimize_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__srv__TriggerOptimize_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__STRUCT_H_
