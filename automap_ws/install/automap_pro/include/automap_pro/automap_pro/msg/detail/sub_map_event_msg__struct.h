// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:msg/SubMapEventMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__STRUCT_H_
#define AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__STRUCT_H_

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
// Member 'event_type'
#include "rosidl_runtime_c/string.h"
// Member 'anchor_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/SubMapEventMsg in the package automap_pro.
typedef struct automap_pro__msg__SubMapEventMsg
{
  std_msgs__msg__Header header;
  int32_t submap_id;
  uint64_t session_id;
  /// "CREATED" | "FROZEN" | "OPTIMIZED" | "ARCHIVED"
  rosidl_runtime_c__String event_type;
  int32_t keyframe_count;
  double spatial_extent_m;
  geometry_msgs__msg__Pose anchor_pose;
  bool has_valid_gps;
} automap_pro__msg__SubMapEventMsg;

// Struct for a sequence of automap_pro__msg__SubMapEventMsg.
typedef struct automap_pro__msg__SubMapEventMsg__Sequence
{
  automap_pro__msg__SubMapEventMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__msg__SubMapEventMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__SUB_MAP_EVENT_MSG__STRUCT_H_
