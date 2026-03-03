// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:msg/LoopConstraintMsg.idl
// generated code does not contain a copyright notice
#include "automap_pro/msg/detail/loop_constraint_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `delta_pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `status`
#include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__msg__LoopConstraintMsg__init(automap_pro__msg__LoopConstraintMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    automap_pro__msg__LoopConstraintMsg__fini(msg);
    return false;
  }
  // submap_i
  // submap_j
  // session_i
  // session_j
  // overlap_score
  // inlier_ratio
  // rmse
  // is_inter_session
  // information_matrix
  // delta_pose
  if (!geometry_msgs__msg__Pose__init(&msg->delta_pose)) {
    automap_pro__msg__LoopConstraintMsg__fini(msg);
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__init(&msg->status)) {
    automap_pro__msg__LoopConstraintMsg__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__msg__LoopConstraintMsg__fini(automap_pro__msg__LoopConstraintMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // submap_i
  // submap_j
  // session_i
  // session_j
  // overlap_score
  // inlier_ratio
  // rmse
  // is_inter_session
  // information_matrix
  // delta_pose
  geometry_msgs__msg__Pose__fini(&msg->delta_pose);
  // status
  rosidl_runtime_c__String__fini(&msg->status);
}

bool
automap_pro__msg__LoopConstraintMsg__are_equal(const automap_pro__msg__LoopConstraintMsg * lhs, const automap_pro__msg__LoopConstraintMsg * rhs)
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
  // submap_i
  if (lhs->submap_i != rhs->submap_i) {
    return false;
  }
  // submap_j
  if (lhs->submap_j != rhs->submap_j) {
    return false;
  }
  // session_i
  if (lhs->session_i != rhs->session_i) {
    return false;
  }
  // session_j
  if (lhs->session_j != rhs->session_j) {
    return false;
  }
  // overlap_score
  if (lhs->overlap_score != rhs->overlap_score) {
    return false;
  }
  // inlier_ratio
  if (lhs->inlier_ratio != rhs->inlier_ratio) {
    return false;
  }
  // rmse
  if (lhs->rmse != rhs->rmse) {
    return false;
  }
  // is_inter_session
  if (lhs->is_inter_session != rhs->is_inter_session) {
    return false;
  }
  // information_matrix
  for (size_t i = 0; i < 36; ++i) {
    if (lhs->information_matrix[i] != rhs->information_matrix[i]) {
      return false;
    }
  }
  // delta_pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->delta_pose), &(rhs->delta_pose)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->status), &(rhs->status)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__msg__LoopConstraintMsg__copy(
  const automap_pro__msg__LoopConstraintMsg * input,
  automap_pro__msg__LoopConstraintMsg * output)
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
  // submap_i
  output->submap_i = input->submap_i;
  // submap_j
  output->submap_j = input->submap_j;
  // session_i
  output->session_i = input->session_i;
  // session_j
  output->session_j = input->session_j;
  // overlap_score
  output->overlap_score = input->overlap_score;
  // inlier_ratio
  output->inlier_ratio = input->inlier_ratio;
  // rmse
  output->rmse = input->rmse;
  // is_inter_session
  output->is_inter_session = input->is_inter_session;
  // information_matrix
  for (size_t i = 0; i < 36; ++i) {
    output->information_matrix[i] = input->information_matrix[i];
  }
  // delta_pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->delta_pose), &(output->delta_pose)))
  {
    return false;
  }
  // status
  if (!rosidl_runtime_c__String__copy(
      &(input->status), &(output->status)))
  {
    return false;
  }
  return true;
}

automap_pro__msg__LoopConstraintMsg *
automap_pro__msg__LoopConstraintMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__LoopConstraintMsg * msg = (automap_pro__msg__LoopConstraintMsg *)allocator.allocate(sizeof(automap_pro__msg__LoopConstraintMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__msg__LoopConstraintMsg));
  bool success = automap_pro__msg__LoopConstraintMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__msg__LoopConstraintMsg__destroy(automap_pro__msg__LoopConstraintMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__msg__LoopConstraintMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__msg__LoopConstraintMsg__Sequence__init(automap_pro__msg__LoopConstraintMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__LoopConstraintMsg * data = NULL;

  if (size) {
    data = (automap_pro__msg__LoopConstraintMsg *)allocator.zero_allocate(size, sizeof(automap_pro__msg__LoopConstraintMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__msg__LoopConstraintMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__msg__LoopConstraintMsg__fini(&data[i - 1]);
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
automap_pro__msg__LoopConstraintMsg__Sequence__fini(automap_pro__msg__LoopConstraintMsg__Sequence * array)
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
      automap_pro__msg__LoopConstraintMsg__fini(&array->data[i]);
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

automap_pro__msg__LoopConstraintMsg__Sequence *
automap_pro__msg__LoopConstraintMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__LoopConstraintMsg__Sequence * array = (automap_pro__msg__LoopConstraintMsg__Sequence *)allocator.allocate(sizeof(automap_pro__msg__LoopConstraintMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__msg__LoopConstraintMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__msg__LoopConstraintMsg__Sequence__destroy(automap_pro__msg__LoopConstraintMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__msg__LoopConstraintMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__msg__LoopConstraintMsg__Sequence__are_equal(const automap_pro__msg__LoopConstraintMsg__Sequence * lhs, const automap_pro__msg__LoopConstraintMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__msg__LoopConstraintMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__msg__LoopConstraintMsg__Sequence__copy(
  const automap_pro__msg__LoopConstraintMsg__Sequence * input,
  automap_pro__msg__LoopConstraintMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__msg__LoopConstraintMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__msg__LoopConstraintMsg * data =
      (automap_pro__msg__LoopConstraintMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__msg__LoopConstraintMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__msg__LoopConstraintMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__msg__LoopConstraintMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
