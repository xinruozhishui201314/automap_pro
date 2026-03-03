// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:msg/KeyFrameMsg.idl
// generated code does not contain a copyright notice
#include "automap_pro/msg/detail/key_frame_msg__functions.h"

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
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"
// Member `gps`
#include "automap_pro/msg/detail/gps_measurement_msg__functions.h"

bool
automap_pro__msg__KeyFrameMsg__init(automap_pro__msg__KeyFrameMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    automap_pro__msg__KeyFrameMsg__fini(msg);
    return false;
  }
  // id
  // session_id
  // submap_id
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__init(&msg->pose)) {
    automap_pro__msg__KeyFrameMsg__fini(msg);
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud)) {
    automap_pro__msg__KeyFrameMsg__fini(msg);
    return false;
  }
  // has_gps
  // gps
  if (!automap_pro__msg__GPSMeasurementMsg__init(&msg->gps)) {
    automap_pro__msg__KeyFrameMsg__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__msg__KeyFrameMsg__fini(automap_pro__msg__KeyFrameMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // session_id
  // submap_id
  // pose
  geometry_msgs__msg__PoseWithCovariance__fini(&msg->pose);
  // cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud);
  // has_gps
  // gps
  automap_pro__msg__GPSMeasurementMsg__fini(&msg->gps);
}

bool
automap_pro__msg__KeyFrameMsg__are_equal(const automap_pro__msg__KeyFrameMsg * lhs, const automap_pro__msg__KeyFrameMsg * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // session_id
  if (lhs->session_id != rhs->session_id) {
    return false;
  }
  // submap_id
  if (lhs->submap_id != rhs->submap_id) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud), &(rhs->cloud)))
  {
    return false;
  }
  // has_gps
  if (lhs->has_gps != rhs->has_gps) {
    return false;
  }
  // gps
  if (!automap_pro__msg__GPSMeasurementMsg__are_equal(
      &(lhs->gps), &(rhs->gps)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__msg__KeyFrameMsg__copy(
  const automap_pro__msg__KeyFrameMsg * input,
  automap_pro__msg__KeyFrameMsg * output)
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
  // id
  output->id = input->id;
  // session_id
  output->session_id = input->session_id;
  // submap_id
  output->submap_id = input->submap_id;
  // pose
  if (!geometry_msgs__msg__PoseWithCovariance__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud), &(output->cloud)))
  {
    return false;
  }
  // has_gps
  output->has_gps = input->has_gps;
  // gps
  if (!automap_pro__msg__GPSMeasurementMsg__copy(
      &(input->gps), &(output->gps)))
  {
    return false;
  }
  return true;
}

automap_pro__msg__KeyFrameMsg *
automap_pro__msg__KeyFrameMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__KeyFrameMsg * msg = (automap_pro__msg__KeyFrameMsg *)allocator.allocate(sizeof(automap_pro__msg__KeyFrameMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__msg__KeyFrameMsg));
  bool success = automap_pro__msg__KeyFrameMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__msg__KeyFrameMsg__destroy(automap_pro__msg__KeyFrameMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__msg__KeyFrameMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__msg__KeyFrameMsg__Sequence__init(automap_pro__msg__KeyFrameMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__KeyFrameMsg * data = NULL;

  if (size) {
    data = (automap_pro__msg__KeyFrameMsg *)allocator.zero_allocate(size, sizeof(automap_pro__msg__KeyFrameMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__msg__KeyFrameMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__msg__KeyFrameMsg__fini(&data[i - 1]);
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
automap_pro__msg__KeyFrameMsg__Sequence__fini(automap_pro__msg__KeyFrameMsg__Sequence * array)
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
      automap_pro__msg__KeyFrameMsg__fini(&array->data[i]);
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

automap_pro__msg__KeyFrameMsg__Sequence *
automap_pro__msg__KeyFrameMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__KeyFrameMsg__Sequence * array = (automap_pro__msg__KeyFrameMsg__Sequence *)allocator.allocate(sizeof(automap_pro__msg__KeyFrameMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__msg__KeyFrameMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__msg__KeyFrameMsg__Sequence__destroy(automap_pro__msg__KeyFrameMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__msg__KeyFrameMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__msg__KeyFrameMsg__Sequence__are_equal(const automap_pro__msg__KeyFrameMsg__Sequence * lhs, const automap_pro__msg__KeyFrameMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__msg__KeyFrameMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__msg__KeyFrameMsg__Sequence__copy(
  const automap_pro__msg__KeyFrameMsg__Sequence * input,
  automap_pro__msg__KeyFrameMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__msg__KeyFrameMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__msg__KeyFrameMsg * data =
      (automap_pro__msg__KeyFrameMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__msg__KeyFrameMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__msg__KeyFrameMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__msg__KeyFrameMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
