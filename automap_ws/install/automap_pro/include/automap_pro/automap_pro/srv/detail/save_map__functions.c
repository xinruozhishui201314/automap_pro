// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:srv/SaveMap.idl
// generated code does not contain a copyright notice
#include "automap_pro/srv/detail/save_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `output_dir`
#include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__srv__SaveMap_Request__init(automap_pro__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // output_dir
  if (!rosidl_runtime_c__String__init(&msg->output_dir)) {
    automap_pro__srv__SaveMap_Request__fini(msg);
    return false;
  }
  // save_pcd
  // save_ply
  // save_las
  // save_trajectory_tum
  // save_trajectory_kitti
  return true;
}

void
automap_pro__srv__SaveMap_Request__fini(automap_pro__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // output_dir
  rosidl_runtime_c__String__fini(&msg->output_dir);
  // save_pcd
  // save_ply
  // save_las
  // save_trajectory_tum
  // save_trajectory_kitti
}

bool
automap_pro__srv__SaveMap_Request__are_equal(const automap_pro__srv__SaveMap_Request * lhs, const automap_pro__srv__SaveMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // output_dir
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->output_dir), &(rhs->output_dir)))
  {
    return false;
  }
  // save_pcd
  if (lhs->save_pcd != rhs->save_pcd) {
    return false;
  }
  // save_ply
  if (lhs->save_ply != rhs->save_ply) {
    return false;
  }
  // save_las
  if (lhs->save_las != rhs->save_las) {
    return false;
  }
  // save_trajectory_tum
  if (lhs->save_trajectory_tum != rhs->save_trajectory_tum) {
    return false;
  }
  // save_trajectory_kitti
  if (lhs->save_trajectory_kitti != rhs->save_trajectory_kitti) {
    return false;
  }
  return true;
}

bool
automap_pro__srv__SaveMap_Request__copy(
  const automap_pro__srv__SaveMap_Request * input,
  automap_pro__srv__SaveMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // output_dir
  if (!rosidl_runtime_c__String__copy(
      &(input->output_dir), &(output->output_dir)))
  {
    return false;
  }
  // save_pcd
  output->save_pcd = input->save_pcd;
  // save_ply
  output->save_ply = input->save_ply;
  // save_las
  output->save_las = input->save_las;
  // save_trajectory_tum
  output->save_trajectory_tum = input->save_trajectory_tum;
  // save_trajectory_kitti
  output->save_trajectory_kitti = input->save_trajectory_kitti;
  return true;
}

automap_pro__srv__SaveMap_Request *
automap_pro__srv__SaveMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__SaveMap_Request * msg = (automap_pro__srv__SaveMap_Request *)allocator.allocate(sizeof(automap_pro__srv__SaveMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__srv__SaveMap_Request));
  bool success = automap_pro__srv__SaveMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__srv__SaveMap_Request__destroy(automap_pro__srv__SaveMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__srv__SaveMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__srv__SaveMap_Request__Sequence__init(automap_pro__srv__SaveMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__SaveMap_Request * data = NULL;

  if (size) {
    data = (automap_pro__srv__SaveMap_Request *)allocator.zero_allocate(size, sizeof(automap_pro__srv__SaveMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__srv__SaveMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__srv__SaveMap_Request__fini(&data[i - 1]);
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
automap_pro__srv__SaveMap_Request__Sequence__fini(automap_pro__srv__SaveMap_Request__Sequence * array)
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
      automap_pro__srv__SaveMap_Request__fini(&array->data[i]);
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

automap_pro__srv__SaveMap_Request__Sequence *
automap_pro__srv__SaveMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__SaveMap_Request__Sequence * array = (automap_pro__srv__SaveMap_Request__Sequence *)allocator.allocate(sizeof(automap_pro__srv__SaveMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__srv__SaveMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__srv__SaveMap_Request__Sequence__destroy(automap_pro__srv__SaveMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__srv__SaveMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__srv__SaveMap_Request__Sequence__are_equal(const automap_pro__srv__SaveMap_Request__Sequence * lhs, const automap_pro__srv__SaveMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__srv__SaveMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__srv__SaveMap_Request__Sequence__copy(
  const automap_pro__srv__SaveMap_Request__Sequence * input,
  automap_pro__srv__SaveMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__srv__SaveMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__srv__SaveMap_Request * data =
      (automap_pro__srv__SaveMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__srv__SaveMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__srv__SaveMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__srv__SaveMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `output_path`
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__srv__SaveMap_Response__init(automap_pro__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // output_path
  if (!rosidl_runtime_c__String__init(&msg->output_path)) {
    automap_pro__srv__SaveMap_Response__fini(msg);
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    automap_pro__srv__SaveMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__srv__SaveMap_Response__fini(automap_pro__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // output_path
  rosidl_runtime_c__String__fini(&msg->output_path);
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
automap_pro__srv__SaveMap_Response__are_equal(const automap_pro__srv__SaveMap_Response * lhs, const automap_pro__srv__SaveMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // output_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->output_path), &(rhs->output_path)))
  {
    return false;
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
automap_pro__srv__SaveMap_Response__copy(
  const automap_pro__srv__SaveMap_Response * input,
  automap_pro__srv__SaveMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // output_path
  if (!rosidl_runtime_c__String__copy(
      &(input->output_path), &(output->output_path)))
  {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

automap_pro__srv__SaveMap_Response *
automap_pro__srv__SaveMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__SaveMap_Response * msg = (automap_pro__srv__SaveMap_Response *)allocator.allocate(sizeof(automap_pro__srv__SaveMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__srv__SaveMap_Response));
  bool success = automap_pro__srv__SaveMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__srv__SaveMap_Response__destroy(automap_pro__srv__SaveMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__srv__SaveMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__srv__SaveMap_Response__Sequence__init(automap_pro__srv__SaveMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__SaveMap_Response * data = NULL;

  if (size) {
    data = (automap_pro__srv__SaveMap_Response *)allocator.zero_allocate(size, sizeof(automap_pro__srv__SaveMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__srv__SaveMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__srv__SaveMap_Response__fini(&data[i - 1]);
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
automap_pro__srv__SaveMap_Response__Sequence__fini(automap_pro__srv__SaveMap_Response__Sequence * array)
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
      automap_pro__srv__SaveMap_Response__fini(&array->data[i]);
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

automap_pro__srv__SaveMap_Response__Sequence *
automap_pro__srv__SaveMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__SaveMap_Response__Sequence * array = (automap_pro__srv__SaveMap_Response__Sequence *)allocator.allocate(sizeof(automap_pro__srv__SaveMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__srv__SaveMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__srv__SaveMap_Response__Sequence__destroy(automap_pro__srv__SaveMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__srv__SaveMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__srv__SaveMap_Response__Sequence__are_equal(const automap_pro__srv__SaveMap_Response__Sequence * lhs, const automap_pro__srv__SaveMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__srv__SaveMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__srv__SaveMap_Response__Sequence__copy(
  const automap_pro__srv__SaveMap_Response__Sequence * input,
  automap_pro__srv__SaveMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__srv__SaveMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__srv__SaveMap_Response * data =
      (automap_pro__srv__SaveMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__srv__SaveMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__srv__SaveMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__srv__SaveMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
