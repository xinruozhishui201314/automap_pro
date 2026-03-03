// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:srv/GetStatus.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__STRUCT_H_
#define AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetStatus in the package automap_pro.
typedef struct automap_pro__srv__GetStatus_Request
{
  uint8_t structure_needs_at_least_one_member;
} automap_pro__srv__GetStatus_Request;

// Struct for a sequence of automap_pro__srv__GetStatus_Request.
typedef struct automap_pro__srv__GetStatus_Request__Sequence
{
  automap_pro__srv__GetStatus_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__srv__GetStatus_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetStatus in the package automap_pro.
typedef struct automap_pro__srv__GetStatus_Response
{
  rosidl_runtime_c__String state;
  uint64_t session_id;
  uint32_t keyframe_count;
  uint32_t submap_count;
  uint32_t loop_count;
  bool gps_aligned;
  float gps_align_score;
  double total_distance_m;
} automap_pro__srv__GetStatus_Response;

// Struct for a sequence of automap_pro__srv__GetStatus_Response.
typedef struct automap_pro__srv__GetStatus_Response__Sequence
{
  automap_pro__srv__GetStatus_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__srv__GetStatus_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__SRV__DETAIL__GET_STATUS__STRUCT_H_
