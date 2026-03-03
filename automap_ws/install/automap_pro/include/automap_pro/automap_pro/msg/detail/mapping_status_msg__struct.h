// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:msg/MappingStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__STRUCT_H_
#define AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MappingStatusMsg in the package automap_pro.
typedef struct automap_pro__msg__MappingStatusMsg
{
  std_msgs__msg__Header header;
  /// "IDLE" | "MAPPING" | "OPTIMIZING" | "SAVING"
  rosidl_runtime_c__String state;
  uint64_t session_id;
  uint32_t keyframe_count;
  uint32_t submap_count;
  uint32_t loop_count;
  bool gps_aligned;
  float gps_alignment_score;
  double map_entropy;
  double total_distance_m;
} automap_pro__msg__MappingStatusMsg;

// Struct for a sequence of automap_pro__msg__MappingStatusMsg.
typedef struct automap_pro__msg__MappingStatusMsg__Sequence
{
  automap_pro__msg__MappingStatusMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__msg__MappingStatusMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__STRUCT_H_
