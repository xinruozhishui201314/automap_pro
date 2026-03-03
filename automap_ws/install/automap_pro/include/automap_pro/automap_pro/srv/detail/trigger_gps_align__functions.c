// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:srv/TriggerGpsAlign.idl
// generated code does not contain a copyright notice
#include "automap_pro/srv/detail/trigger_gps_align__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
automap_pro__srv__TriggerGpsAlign_Request__init(automap_pro__srv__TriggerGpsAlign_Request * msg)
{
  if (!msg) {
    return false;
  }
  // force
  return true;
}

void
automap_pro__srv__TriggerGpsAlign_Request__fini(automap_pro__srv__TriggerGpsAlign_Request * msg)
{
  if (!msg) {
    return;
  }
  // force
}

bool
automap_pro__srv__TriggerGpsAlign_Request__are_equal(const automap_pro__srv__TriggerGpsAlign_Request * lhs, const automap_pro__srv__TriggerGpsAlign_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  return true;
}

bool
automap_pro__srv__TriggerGpsAlign_Request__copy(
  const automap_pro__srv__TriggerGpsAlign_Request * input,
  automap_pro__srv__TriggerGpsAlign_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // force
  output->force = input->force;
  return true;
}

automap_pro__srv__TriggerGpsAlign_Request *
automap_pro__srv__TriggerGpsAlign_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__TriggerGpsAlign_Request * msg = (automap_pro__srv__TriggerGpsAlign_Request *)allocator.allocate(sizeof(automap_pro__srv__TriggerGpsAlign_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__srv__TriggerGpsAlign_Request));
  bool success = automap_pro__srv__TriggerGpsAlign_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__srv__TriggerGpsAlign_Request__destroy(automap_pro__srv__TriggerGpsAlign_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__srv__TriggerGpsAlign_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__srv__TriggerGpsAlign_Request__Sequence__init(automap_pro__srv__TriggerGpsAlign_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__TriggerGpsAlign_Request * data = NULL;

  if (size) {
    data = (automap_pro__srv__TriggerGpsAlign_Request *)allocator.zero_allocate(size, sizeof(automap_pro__srv__TriggerGpsAlign_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__srv__TriggerGpsAlign_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__srv__TriggerGpsAlign_Request__fini(&data[i - 1]);
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
automap_pro__srv__TriggerGpsAlign_Request__Sequence__fini(automap_pro__srv__TriggerGpsAlign_Request__Sequence * array)
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
      automap_pro__srv__TriggerGpsAlign_Request__fini(&array->data[i]);
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

automap_pro__srv__TriggerGpsAlign_Request__Sequence *
automap_pro__srv__TriggerGpsAlign_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__TriggerGpsAlign_Request__Sequence * array = (automap_pro__srv__TriggerGpsAlign_Request__Sequence *)allocator.allocate(sizeof(automap_pro__srv__TriggerGpsAlign_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__srv__TriggerGpsAlign_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__srv__TriggerGpsAlign_Request__Sequence__destroy(automap_pro__srv__TriggerGpsAlign_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__srv__TriggerGpsAlign_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__srv__TriggerGpsAlign_Request__Sequence__are_equal(const automap_pro__srv__TriggerGpsAlign_Request__Sequence * lhs, const automap_pro__srv__TriggerGpsAlign_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__srv__TriggerGpsAlign_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__srv__TriggerGpsAlign_Request__Sequence__copy(
  const automap_pro__srv__TriggerGpsAlign_Request__Sequence * input,
  automap_pro__srv__TriggerGpsAlign_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__srv__TriggerGpsAlign_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__srv__TriggerGpsAlign_Request * data =
      (automap_pro__srv__TriggerGpsAlign_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__srv__TriggerGpsAlign_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__srv__TriggerGpsAlign_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__srv__TriggerGpsAlign_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__srv__TriggerGpsAlign_Response__init(automap_pro__srv__TriggerGpsAlign_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // alignment_rmse_m
  // r_gps_lidar
  // t_gps_lidar
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    automap_pro__srv__TriggerGpsAlign_Response__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__srv__TriggerGpsAlign_Response__fini(automap_pro__srv__TriggerGpsAlign_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // alignment_rmse_m
  // r_gps_lidar
  // t_gps_lidar
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
automap_pro__srv__TriggerGpsAlign_Response__are_equal(const automap_pro__srv__TriggerGpsAlign_Response * lhs, const automap_pro__srv__TriggerGpsAlign_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // alignment_rmse_m
  if (lhs->alignment_rmse_m != rhs->alignment_rmse_m) {
    return false;
  }
  // r_gps_lidar
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->r_gps_lidar[i] != rhs->r_gps_lidar[i]) {
      return false;
    }
  }
  // t_gps_lidar
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->t_gps_lidar[i] != rhs->t_gps_lidar[i]) {
      return false;
    }
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__srv__TriggerGpsAlign_Response__copy(
  const automap_pro__srv__TriggerGpsAlign_Response * input,
  automap_pro__srv__TriggerGpsAlign_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // alignment_rmse_m
  output->alignment_rmse_m = input->alignment_rmse_m;
  // r_gps_lidar
  for (size_t i = 0; i < 9; ++i) {
    output->r_gps_lidar[i] = input->r_gps_lidar[i];
  }
  // t_gps_lidar
  for (size_t i = 0; i < 3; ++i) {
    output->t_gps_lidar[i] = input->t_gps_lidar[i];
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

automap_pro__srv__TriggerGpsAlign_Response *
automap_pro__srv__TriggerGpsAlign_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__TriggerGpsAlign_Response * msg = (automap_pro__srv__TriggerGpsAlign_Response *)allocator.allocate(sizeof(automap_pro__srv__TriggerGpsAlign_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__srv__TriggerGpsAlign_Response));
  bool success = automap_pro__srv__TriggerGpsAlign_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__srv__TriggerGpsAlign_Response__destroy(automap_pro__srv__TriggerGpsAlign_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__srv__TriggerGpsAlign_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__srv__TriggerGpsAlign_Response__Sequence__init(automap_pro__srv__TriggerGpsAlign_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__TriggerGpsAlign_Response * data = NULL;

  if (size) {
    data = (automap_pro__srv__TriggerGpsAlign_Response *)allocator.zero_allocate(size, sizeof(automap_pro__srv__TriggerGpsAlign_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__srv__TriggerGpsAlign_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__srv__TriggerGpsAlign_Response__fini(&data[i - 1]);
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
automap_pro__srv__TriggerGpsAlign_Response__Sequence__fini(automap_pro__srv__TriggerGpsAlign_Response__Sequence * array)
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
      automap_pro__srv__TriggerGpsAlign_Response__fini(&array->data[i]);
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

automap_pro__srv__TriggerGpsAlign_Response__Sequence *
automap_pro__srv__TriggerGpsAlign_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__TriggerGpsAlign_Response__Sequence * array = (automap_pro__srv__TriggerGpsAlign_Response__Sequence *)allocator.allocate(sizeof(automap_pro__srv__TriggerGpsAlign_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__srv__TriggerGpsAlign_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__srv__TriggerGpsAlign_Response__Sequence__destroy(automap_pro__srv__TriggerGpsAlign_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__srv__TriggerGpsAlign_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__srv__TriggerGpsAlign_Response__Sequence__are_equal(const automap_pro__srv__TriggerGpsAlign_Response__Sequence * lhs, const automap_pro__srv__TriggerGpsAlign_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__srv__TriggerGpsAlign_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__srv__TriggerGpsAlign_Response__Sequence__copy(
  const automap_pro__srv__TriggerGpsAlign_Response__Sequence * input,
  automap_pro__srv__TriggerGpsAlign_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__srv__TriggerGpsAlign_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__srv__TriggerGpsAlign_Response * data =
      (automap_pro__srv__TriggerGpsAlign_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__srv__TriggerGpsAlign_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__srv__TriggerGpsAlign_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__srv__TriggerGpsAlign_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
