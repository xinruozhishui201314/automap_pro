// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice

#ifndef OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__STRUCT_H_
#define OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pointcloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in srv/ComputeDescriptor in the package overlap_transformer_msgs.
typedef struct overlap_transformer_msgs__srv__ComputeDescriptor_Request
{
  sensor_msgs__msg__PointCloud2 pointcloud;
} overlap_transformer_msgs__srv__ComputeDescriptor_Request;

// Struct for a sequence of overlap_transformer_msgs__srv__ComputeDescriptor_Request.
typedef struct overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence
{
  overlap_transformer_msgs__srv__ComputeDescriptor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'descriptor'
#include "std_msgs/msg/detail/float32_multi_array__struct.h"

/// Struct defined in srv/ComputeDescriptor in the package overlap_transformer_msgs.
typedef struct overlap_transformer_msgs__srv__ComputeDescriptor_Response
{
  std_msgs__msg__Float32MultiArray descriptor;
} overlap_transformer_msgs__srv__ComputeDescriptor_Response;

// Struct for a sequence of overlap_transformer_msgs__srv__ComputeDescriptor_Response.
typedef struct overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence
{
  overlap_transformer_msgs__srv__ComputeDescriptor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OVERLAP_TRANSFORMER_MSGS__SRV__DETAIL__COMPUTE_DESCRIPTOR__STRUCT_H_
