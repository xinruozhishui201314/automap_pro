// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__STRUCT_H_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/TriggerGpsAlign in the package automap_pro.
typedef struct automap_pro__srv__TriggerGpsAlign_Request
{
  bool force;
} automap_pro__srv__TriggerGpsAlign_Request;

// Struct for a sequence of automap_pro__srv__TriggerGpsAlign_Request.
typedef struct automap_pro__srv__TriggerGpsAlign_Request__Sequence
{
  automap_pro__srv__TriggerGpsAlign_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__srv__TriggerGpsAlign_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/TriggerGpsAlign in the package automap_pro.
typedef struct automap_pro__srv__TriggerGpsAlign_Response
{
  bool success;
  double alignment_rmse_m;
  double r_gps_lidar[9];
  double t_gps_lidar[3];
  rosidl_runtime_c__String message;
} automap_pro__srv__TriggerGpsAlign_Response;

// Struct for a sequence of automap_pro__srv__TriggerGpsAlign_Response.
typedef struct automap_pro__srv__TriggerGpsAlign_Response__Sequence
{
  automap_pro__srv__TriggerGpsAlign_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} automap_pro__srv__TriggerGpsAlign_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_GPS_ALIGN__STRUCT_H_
