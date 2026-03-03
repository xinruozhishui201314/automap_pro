// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:msg/MappingStatusMsg.idl
// generated code does not contain a copyright notice
#include "automap_pro/msg/detail/mapping_status_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__msg__MappingStatusMsg__init(automap_pro__msg__MappingStatusMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    automap_pro__msg__MappingStatusMsg__fini(msg);
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    automap_pro__msg__MappingStatusMsg__fini(msg);
    return false;
  }
  // session_id
  // keyframe_count
  // submap_count
  // loop_count
  // gps_aligned
  // gps_alignment_score
  // map_entropy
  // total_distance_m
  return true;
}

void
automap_pro__msg__MappingStatusMsg__fini(automap_pro__msg__MappingStatusMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // session_id
  // keyframe_count
  // submap_count
  // loop_count
  // gps_aligned
  // gps_alignment_score
  // map_entropy
  // total_distance_m
}

bool
automap_pro__msg__MappingStatusMsg__are_equal(const automap_pro__msg__MappingStatusMsg * lhs, const automap_pro__msg__MappingStatusMsg * rhs)
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
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // session_id
  if (lhs->session_id != rhs->session_id) {
    return false;
  }
  // keyframe_count
  if (lhs->keyframe_count != rhs->keyframe_count) {
    return false;
  }
  // submap_count
  if (lhs->submap_count != rhs->submap_count) {
    return false;
  }
  // loop_count
  if (lhs->loop_count != rhs->loop_count) {
    return false;
  }
  // gps_aligned
  if (lhs->gps_aligned != rhs->gps_aligned) {
    return false;
  }
  // gps_alignment_score
  if (lhs->gps_alignment_score != rhs->gps_alignment_score) {
    return false;
  }
  // map_entropy
  if (lhs->map_entropy != rhs->map_entropy) {
    return false;
  }
  // total_distance_m
  if (lhs->total_distance_m != rhs->total_distance_m) {
    return false;
  }
  return true;
}

bool
automap_pro__msg__MappingStatusMsg__copy(
  const automap_pro__msg__MappingStatusMsg * input,
  automap_pro__msg__MappingStatusMsg * output)
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
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // session_id
  output->session_id = input->session_id;
  // keyframe_count
  output->keyframe_count = input->keyframe_count;
  // submap_count
  output->submap_count = input->submap_count;
  // loop_count
  output->loop_count = input->loop_count;
  // gps_aligned
  output->gps_aligned = input->gps_aligned;
  // gps_alignment_score
  output->gps_alignment_score = input->gps_alignment_score;
  // map_entropy
  output->map_entropy = input->map_entropy;
  // total_distance_m
  output->total_distance_m = input->total_distance_m;
  return true;
}

automap_pro__msg__MappingStatusMsg *
automap_pro__msg__MappingStatusMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__MappingStatusMsg * msg = (automap_pro__msg__MappingStatusMsg *)allocator.allocate(sizeof(automap_pro__msg__MappingStatusMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__msg__MappingStatusMsg));
  bool success = automap_pro__msg__MappingStatusMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__msg__MappingStatusMsg__destroy(automap_pro__msg__MappingStatusMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__msg__MappingStatusMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__msg__MappingStatusMsg__Sequence__init(automap_pro__msg__MappingStatusMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__MappingStatusMsg * data = NULL;

  if (size) {
    data = (automap_pro__msg__MappingStatusMsg *)allocator.zero_allocate(size, sizeof(automap_pro__msg__MappingStatusMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__msg__MappingStatusMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__msg__MappingStatusMsg__fini(&data[i - 1]);
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
automap_pro__msg__MappingStatusMsg__Sequence__fini(automap_pro__msg__MappingStatusMsg__Sequence * array)
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
      automap_pro__msg__MappingStatusMsg__fini(&array->data[i]);
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

automap_pro__msg__MappingStatusMsg__Sequence *
automap_pro__msg__MappingStatusMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__MappingStatusMsg__Sequence * array = (automap_pro__msg__MappingStatusMsg__Sequence *)allocator.allocate(sizeof(automap_pro__msg__MappingStatusMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__msg__MappingStatusMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__msg__MappingStatusMsg__Sequence__destroy(automap_pro__msg__MappingStatusMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__msg__MappingStatusMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__msg__MappingStatusMsg__Sequence__are_equal(const automap_pro__msg__MappingStatusMsg__Sequence * lhs, const automap_pro__msg__MappingStatusMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__msg__MappingStatusMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__msg__MappingStatusMsg__Sequence__copy(
  const automap_pro__msg__MappingStatusMsg__Sequence * input,
  automap_pro__msg__MappingStatusMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__msg__MappingStatusMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__msg__MappingStatusMsg * data =
      (automap_pro__msg__MappingStatusMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__msg__MappingStatusMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__msg__MappingStatusMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__msg__MappingStatusMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
