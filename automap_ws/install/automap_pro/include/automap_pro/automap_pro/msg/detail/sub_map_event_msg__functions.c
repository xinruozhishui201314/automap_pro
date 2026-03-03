// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:msg/SubMapEventMsg.idl
// generated code does not contain a copyright notice
#include "automap_pro/msg/detail/sub_map_event_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `event_type`
#include "rosidl_runtime_c/string_functions.h"
// Member `anchor_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
automap_pro__msg__SubMapEventMsg__init(automap_pro__msg__SubMapEventMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    automap_pro__msg__SubMapEventMsg__fini(msg);
    return false;
  }
  // submap_id
  // session_id
  // event_type
  if (!rosidl_runtime_c__String__init(&msg->event_type)) {
    automap_pro__msg__SubMapEventMsg__fini(msg);
    return false;
  }
  // keyframe_count
  // spatial_extent_m
  // anchor_pose
  if (!geometry_msgs__msg__Pose__init(&msg->anchor_pose)) {
    automap_pro__msg__SubMapEventMsg__fini(msg);
    return false;
  }
  // has_valid_gps
  return true;
}

void
automap_pro__msg__SubMapEventMsg__fini(automap_pro__msg__SubMapEventMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // submap_id
  // session_id
  // event_type
  rosidl_runtime_c__String__fini(&msg->event_type);
  // keyframe_count
  // spatial_extent_m
  // anchor_pose
  geometry_msgs__msg__Pose__fini(&msg->anchor_pose);
  // has_valid_gps
}

bool
automap_pro__msg__SubMapEventMsg__are_equal(const automap_pro__msg__SubMapEventMsg * lhs, const automap_pro__msg__SubMapEventMsg * rhs)
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
  // submap_id
  if (lhs->submap_id != rhs->submap_id) {
    return false;
  }
  // session_id
  if (lhs->session_id != rhs->session_id) {
    return false;
  }
  // event_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->event_type), &(rhs->event_type)))
  {
    return false;
  }
  // keyframe_count
  if (lhs->keyframe_count != rhs->keyframe_count) {
    return false;
  }
  // spatial_extent_m
  if (lhs->spatial_extent_m != rhs->spatial_extent_m) {
    return false;
  }
  // anchor_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->anchor_pose), &(rhs->anchor_pose)))
  {
    return false;
  }
  // has_valid_gps
  if (lhs->has_valid_gps != rhs->has_valid_gps) {
    return false;
  }
  return true;
}

bool
automap_pro__msg__SubMapEventMsg__copy(
  const automap_pro__msg__SubMapEventMsg * input,
  automap_pro__msg__SubMapEventMsg * output)
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
  // submap_id
  output->submap_id = input->submap_id;
  // session_id
  output->session_id = input->session_id;
  // event_type
  if (!rosidl_runtime_c__String__copy(
      &(input->event_type), &(output->event_type)))
  {
    return false;
  }
  // keyframe_count
  output->keyframe_count = input->keyframe_count;
  // spatial_extent_m
  output->spatial_extent_m = input->spatial_extent_m;
  // anchor_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->anchor_pose), &(output->anchor_pose)))
  {
    return false;
  }
  // has_valid_gps
  output->has_valid_gps = input->has_valid_gps;
  return true;
}

automap_pro__msg__SubMapEventMsg *
automap_pro__msg__SubMapEventMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__SubMapEventMsg * msg = (automap_pro__msg__SubMapEventMsg *)allocator.allocate(sizeof(automap_pro__msg__SubMapEventMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__msg__SubMapEventMsg));
  bool success = automap_pro__msg__SubMapEventMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__msg__SubMapEventMsg__destroy(automap_pro__msg__SubMapEventMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__msg__SubMapEventMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__msg__SubMapEventMsg__Sequence__init(automap_pro__msg__SubMapEventMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__SubMapEventMsg * data = NULL;

  if (size) {
    data = (automap_pro__msg__SubMapEventMsg *)allocator.zero_allocate(size, sizeof(automap_pro__msg__SubMapEventMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__msg__SubMapEventMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__msg__SubMapEventMsg__fini(&data[i - 1]);
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
automap_pro__msg__SubMapEventMsg__Sequence__fini(automap_pro__msg__SubMapEventMsg__Sequence * array)
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
      automap_pro__msg__SubMapEventMsg__fini(&array->data[i]);
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

automap_pro__msg__SubMapEventMsg__Sequence *
automap_pro__msg__SubMapEventMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__SubMapEventMsg__Sequence * array = (automap_pro__msg__SubMapEventMsg__Sequence *)allocator.allocate(sizeof(automap_pro__msg__SubMapEventMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__msg__SubMapEventMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__msg__SubMapEventMsg__Sequence__destroy(automap_pro__msg__SubMapEventMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__msg__SubMapEventMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__msg__SubMapEventMsg__Sequence__are_equal(const automap_pro__msg__SubMapEventMsg__Sequence * lhs, const automap_pro__msg__SubMapEventMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__msg__SubMapEventMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__msg__SubMapEventMsg__Sequence__copy(
  const automap_pro__msg__SubMapEventMsg__Sequence * input,
  automap_pro__msg__SubMapEventMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__msg__SubMapEventMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__msg__SubMapEventMsg * data =
      (automap_pro__msg__SubMapEventMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__msg__SubMapEventMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__msg__SubMapEventMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__msg__SubMapEventMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
