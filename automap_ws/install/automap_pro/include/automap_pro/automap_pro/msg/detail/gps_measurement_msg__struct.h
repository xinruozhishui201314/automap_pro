// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:msg/GPSMeasurementMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__STRUCT_H_
#define AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__STRUCT_H_

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

/// Struct defined in msg/GPSMeasurementMsg in the package automap_pro.
typedef struct automap_pro__msg__GPSMeasurementMsg
{
  std_msgs__msg__Header header;
  double position_enu[3];
  /// 0=INVALID 1=LOW 2=MEDIUM 3=HIGH 4=EXCELLENT
  int32_t quality;
  float hdop;
  int32_t num_satellites;
  double covariance[9];
  bool is_valid;
  double latitude;
  double longitude;
  double altitude;
} automap_pro__msg__GPSMeasurementMsg;

// Struct for a sequence of automap_pro__msg__GPSMeasurementMsg.
typedef struct automap_pro__msg__GPSMeasurementMsg__Sequence
{
  automap_pro__msg__GPSMeasurementMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__msg__GPSMeasurementMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__GPS_MEASUREMENT_MSG__STRUCT_H_
