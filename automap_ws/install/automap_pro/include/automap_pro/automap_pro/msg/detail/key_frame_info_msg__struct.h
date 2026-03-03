// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:msg/KeyFrameInfoMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__STRUCT_H_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"

/// Struct defined in msg/KeyFrameInfoMsg in the package automap_pro.
/**
  * Extended keyframe info from FAST-LIVO2 composable node
 */
typedef struct automap_pro__msg__KeyFrameInfoMsg
{
  std_msgs__msg__Header header;
  uint64_t keyframe_id;
  double timestamp;
  /// ESIKF state quality
  /// Frobenius norm of covariance matrix
  double esikf_covariance_norm;
  /// Whether ESIKF is degenerate
  bool is_degenerate;
  /// Points updated per second
  float map_update_rate;
  /// IMU bias
  double gyro_bias[3];
  double accel_bias[3];
  /// Point cloud validity
  uint32_t cloud_point_count;
  bool cloud_valid;
  /// Pose with full covariance
  geometry_msgs__msg__PoseWithCovariance pose;
} automap_pro__msg__KeyFrameInfoMsg;

// Struct for a sequence of automap_pro__msg__KeyFrameInfoMsg.
typedef struct automap_pro__msg__KeyFrameInfoMsg__Sequence
{
  automap_pro__msg__KeyFrameInfoMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__msg__KeyFrameInfoMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_INFO_MSG__STRUCT_H_
