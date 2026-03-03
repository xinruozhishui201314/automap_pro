// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__STRUCT_H_
#define AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__STRUCT_H_

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
// Member 'delta_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LoopConstraintMsg in the package automap_pro.
typedef struct automap_pro__msg__LoopConstraintMsg
{
  std_msgs__msg__Header header;
  int32_t submap_i;
  int32_t submap_j;
  uint64_t session_i;
  uint64_t session_j;
  float overlap_score;
  float inlier_ratio;
  float rmse;
  bool is_inter_session;
  double information_matrix[36];
  geometry_msgs__msg__Pose delta_pose;
  /// "ACCEPTED" | "REJECTED"
  rosidl_runtime_c__String status;
} automap_pro__msg__LoopConstraintMsg;

// Struct for a sequence of automap_pro__msg__LoopConstraintMsg.
typedef struct automap_pro__msg__LoopConstraintMsg__Sequence
{
  automap_pro__msg__LoopConstraintMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__msg__LoopConstraintMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__LOOP_CONSTRAINT_MSG__STRUCT_H_
