// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:msg/KeyFrameMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__STRUCT_H_
#define AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__STRUCT_H_

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
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"
// Member 'gps'
#include "automap_pro/msg/detail/gps_measurement_msg__struct.h"

/// Struct defined in msg/KeyFrameMsg in the package automap_pro.
typedef struct automap_pro__msg__KeyFrameMsg
{
  std_msgs__msg__Header header;
  uint64_t id;
  uint64_t session_id;
  int32_t submap_id;
  geometry_msgs__msg__PoseWithCovariance pose;
  sensor_msgs__msg__PointCloud2 cloud;
  bool has_gps;
  automap_pro__msg__GPSMeasurementMsg gps;
} automap_pro__msg__KeyFrameMsg;

// Struct for a sequence of automap_pro__msg__KeyFrameMsg.
typedef struct automap_pro__msg__KeyFrameMsg__Sequence
{
  automap_pro__msg__KeyFrameMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__msg__KeyFrameMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__KEY_FRAME_MSG__STRUCT_H_
