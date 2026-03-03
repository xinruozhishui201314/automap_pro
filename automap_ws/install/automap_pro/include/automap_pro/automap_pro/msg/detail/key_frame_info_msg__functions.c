// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:msg/KeyFrameInfoMsg.idl
// generated code does not contain a copyright notice
#include "automap_pro/msg/detail/key_frame_info_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"

bool
automap_pro__msg__KeyFrameInfoMsg__init(automap_pro__msg__KeyFrameInfoMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    automap_pro__msg__KeyFrameInfoMsg__fini(msg);
    return false;
  }
  // keyframe_id
  // timestamp
  // esikf_covariance_norm
  // is_degenerate
  // map_update_rate
  // gyro_bias
  // accel_bias
  // cloud_point_count
  // cloud_valid
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__init(&msg->pose)) {
    automap_pro__msg__KeyFrameInfoMsg__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__msg__KeyFrameInfoMsg__fini(automap_pro__msg__KeyFrameInfoMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // keyframe_id
  // timestamp
  // esikf_covariance_norm
  // is_degenerate
  // map_update_rate
  // gyro_bias
  // accel_bias
  // cloud_point_count
  // cloud_valid
  // pose
  geometry_msgs__msg__PoseWithCovariance__fini(&msg->pose);
}

bool
automap_pro__msg__KeyFrameInfoMsg__are_equal(const automap_pro__msg__KeyFrameInfoMsg * lhs, const automap_pro__msg__KeyFrameInfoMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // keyframe_id
  if (lhs->keyframe_id != rhs->keyframe_id) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // esikf_covariance_norm
  if (lhs->esikf_covariance_norm != rhs->esikf_covariance_norm) {
    return false;
  }
  // is_degenerate
  if (lhs->is_degenerate != rhs->is_degenerate) {
    return false;
  }
  // map_update_rate
  if (lhs->map_update_rate != rhs->map_update_rate) {
    return false;
  }
  // gyro_bias
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->gyro_bias[i] != rhs->gyro_bias[i]) {
      return false;
    }
  }
  // accel_bias
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->accel_bias[i] != rhs->accel_bias[i]) {
      return false;
    }
  }
  // cloud_point_count
  if (lhs->cloud_point_count != rhs->cloud_point_count) {
    return false;
  }
  // cloud_valid
  if (lhs->cloud_valid != rhs->cloud_valid) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__msg__KeyFrameInfoMsg__copy(
  const automap_pro__msg__KeyFrameInfoMsg * input,
  automap_pro__msg__KeyFrameInfoMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // keyframe_id
  output->keyframe_id = input->keyframe_id;
  // timestamp
  output->timestamp = input->timestamp;
  // esikf_covariance_norm
  output->esikf_covariance_norm = input->esikf_covariance_norm;
  // is_degenerate
  output->is_degenerate = input->is_degenerate;
  // map_update_rate
  output->map_update_rate = input->map_update_rate;
  // gyro_bias
  for (size_t i = 0; i < 3; ++i) {
    output->gyro_bias[i] = input->gyro_bias[i];
  }
  // accel_bias
  for (size_t i = 0; i < 3; ++i) {
    output->accel_bias[i] = input->accel_bias[i];
  }
  // cloud_point_count
  output->cloud_point_count = input->cloud_point_count;
  // cloud_valid
  output->cloud_valid = input->cloud_valid;
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

automap_pro__msg__KeyFrameInfoMsg *
automap_pro__msg__KeyFrameInfoMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__KeyFrameInfoMsg * msg = (automap_pro__msg__KeyFrameInfoMsg *)allocator.allocate(sizeof(automap_pro__msg__KeyFrameInfoMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__msg__KeyFrameInfoMsg));
  bool success = automap_pro__msg__KeyFrameInfoMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__msg__KeyFrameInfoMsg__destroy(automap_pro__msg__KeyFrameInfoMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__msg__KeyFrameInfoMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__msg__KeyFrameInfoMsg__Sequence__init(automap_pro__msg__KeyFrameInfoMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__KeyFrameInfoMsg * data = NULL;

  if (size) {
    data = (automap_pro__msg__KeyFrameInfoMsg *)allocator.zero_allocate(size, sizeof(automap_pro__msg__KeyFrameInfoMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__msg__KeyFrameInfoMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__msg__KeyFrameInfoMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
automap_pro__msg__KeyFrameInfoMsg__Sequence__fini(automap_pro__msg__KeyFrameInfoMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      automap_pro__msg__KeyFrameInfoMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

automap_pro__msg__KeyFrameInfoMsg__Sequence *
automap_pro__msg__KeyFrameInfoMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__KeyFrameInfoMsg__Sequence * array = (automap_pro__msg__KeyFrameInfoMsg__Sequence *)allocator.allocate(sizeof(automap_pro__msg__KeyFrameInfoMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__msg__KeyFrameInfoMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__msg__KeyFrameInfoMsg__Sequence__destroy(automap_pro__msg__KeyFrameInfoMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__msg__KeyFrameInfoMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__msg__KeyFrameInfoMsg__Sequence__are_equal(const automap_pro__msg__KeyFrameInfoMsg__Sequence * lhs, const automap_pro__msg__KeyFrameInfoMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__msg__KeyFrameInfoMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__msg__KeyFrameInfoMsg__Sequence__copy(
  const automap_pro__msg__KeyFrameInfoMsg__Sequence * input,
  automap_pro__msg__KeyFrameInfoMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__msg__KeyFrameInfoMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__msg__KeyFrameInfoMsg * data =
      (automap_pro__msg__KeyFrameInfoMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__msg__KeyFrameInfoMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__msg__KeyFrameInfoMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__msg__KeyFrameInfoMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
