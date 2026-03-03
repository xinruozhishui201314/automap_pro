// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:srv/LoadSession.idl
// generated code does not contain a copyright notice
#include "automap_pro/srv/detail/load_session__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `session_dir`
#include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__srv__LoadSession_Request__init(automap_pro__srv__LoadSession_Request * msg)
{
  if (!msg) {
    return false;
  }
  // session_dir
  if (!rosidl_runtime_c__String__init(&msg->session_dir)) {
    automap_pro__srv__LoadSession_Request__fini(msg);
    return false;
  }
  // session_id
  return true;
}

void
automap_pro__srv__LoadSession_Request__fini(automap_pro__srv__LoadSession_Request * msg)
{
  if (!msg) {
    return;
  }
  // session_dir
  rosidl_runtime_c__String__fini(&msg->session_dir);
  // session_id
}

bool
automap_pro__srv__LoadSession_Request__are_equal(const automap_pro__srv__LoadSession_Request * lhs, const automap_pro__srv__LoadSession_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // session_dir
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->session_dir), &(rhs->session_dir)))
  {
    return false;
  }
  // session_id
  if (lhs->session_id != rhs->session_id) {
    return false;
  }
  return true;
}

bool
automap_pro__srv__LoadSession_Request__copy(
  const automap_pro__srv__LoadSession_Request * input,
  automap_pro__srv__LoadSession_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // session_dir
  if (!rosidl_runtime_c__String__copy(
      &(input->session_dir), &(output->session_dir)))
  {
    return false;
  }
  // session_id
  output->session_id = input->session_id;
  return true;
}

automap_pro__srv__LoadSession_Request *
automap_pro__srv__LoadSession_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__LoadSession_Request * msg = (automap_pro__srv__LoadSession_Request *)allocator.allocate(sizeof(automap_pro__srv__LoadSession_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__srv__LoadSession_Request));
  bool success = automap_pro__srv__LoadSession_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__srv__LoadSession_Request__destroy(automap_pro__srv__LoadSession_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__srv__LoadSession_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__srv__LoadSession_Request__Sequence__init(automap_pro__srv__LoadSession_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__LoadSession_Request * data = NULL;

  if (size) {
    data = (automap_pro__srv__LoadSession_Request *)allocator.zero_allocate(size, sizeof(automap_pro__srv__LoadSession_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__srv__LoadSession_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__srv__LoadSession_Request__fini(&data[i - 1]);
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
automap_pro__srv__LoadSession_Request__Sequence__fini(automap_pro__srv__LoadSession_Request__Sequence * array)
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
      automap_pro__srv__LoadSession_Request__fini(&array->data[i]);
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

automap_pro__srv__LoadSession_Request__Sequence *
automap_pro__srv__LoadSession_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__LoadSession_Request__Sequence * array = (automap_pro__srv__LoadSession_Request__Sequence *)allocator.allocate(sizeof(automap_pro__srv__LoadSession_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__srv__LoadSession_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__srv__LoadSession_Request__Sequence__destroy(automap_pro__srv__LoadSession_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__srv__LoadSession_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__srv__LoadSession_Request__Sequence__are_equal(const automap_pro__srv__LoadSession_Request__Sequence * lhs, const automap_pro__srv__LoadSession_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__srv__LoadSession_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__srv__LoadSession_Request__Sequence__copy(
  const automap_pro__srv__LoadSession_Request__Sequence * input,
  automap_pro__srv__LoadSession_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__srv__LoadSession_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__srv__LoadSession_Request * data =
      (automap_pro__srv__LoadSession_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__srv__LoadSession_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__srv__LoadSession_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__srv__LoadSession_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__srv__LoadSession_Response__init(automap_pro__srv__LoadSession_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // submaps_loaded
  // descriptors_loaded
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    automap_pro__srv__LoadSession_Response__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__srv__LoadSession_Response__fini(automap_pro__srv__LoadSession_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // submaps_loaded
  // descriptors_loaded
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
automap_pro__srv__LoadSession_Response__are_equal(const automap_pro__srv__LoadSession_Response * lhs, const automap_pro__srv__LoadSession_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // submaps_loaded
  if (lhs->submaps_loaded != rhs->submaps_loaded) {
    return false;
  }
  // descriptors_loaded
  if (lhs->descriptors_loaded != rhs->descriptors_loaded) {
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
automap_pro__srv__LoadSession_Response__copy(
  const automap_pro__srv__LoadSession_Response * input,
  automap_pro__srv__LoadSession_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // submaps_loaded
  output->submaps_loaded = input->submaps_loaded;
  // descriptors_loaded
  output->descriptors_loaded = input->descriptors_loaded;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

automap_pro__srv__LoadSession_Response *
automap_pro__srv__LoadSession_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__LoadSession_Response * msg = (automap_pro__srv__LoadSession_Response *)allocator.allocate(sizeof(automap_pro__srv__LoadSession_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__srv__LoadSession_Response));
  bool success = automap_pro__srv__LoadSession_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__srv__LoadSession_Response__destroy(automap_pro__srv__LoadSession_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__srv__LoadSession_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__srv__LoadSession_Response__Sequence__init(automap_pro__srv__LoadSession_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__LoadSession_Response * data = NULL;

  if (size) {
    data = (automap_pro__srv__LoadSession_Response *)allocator.zero_allocate(size, sizeof(automap_pro__srv__LoadSession_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__srv__LoadSession_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__srv__LoadSession_Response__fini(&data[i - 1]);
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
automap_pro__srv__LoadSession_Response__Sequence__fini(automap_pro__srv__LoadSession_Response__Sequence * array)
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
      automap_pro__srv__LoadSession_Response__fini(&array->data[i]);
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

automap_pro__srv__LoadSession_Response__Sequence *
automap_pro__srv__LoadSession_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__srv__LoadSession_Response__Sequence * array = (automap_pro__srv__LoadSession_Response__Sequence *)allocator.allocate(sizeof(automap_pro__srv__LoadSession_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__srv__LoadSession_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__srv__LoadSession_Response__Sequence__destroy(automap_pro__srv__LoadSession_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__srv__LoadSession_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__srv__LoadSession_Response__Sequence__are_equal(const automap_pro__srv__LoadSession_Response__Sequence * lhs, const automap_pro__srv__LoadSession_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__srv__LoadSession_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__srv__LoadSession_Response__Sequence__copy(
  const automap_pro__srv__LoadSession_Response__Sequence * input,
  automap_pro__srv__LoadSession_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__srv__LoadSession_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__srv__LoadSession_Response * data =
      (automap_pro__srv__LoadSession_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__srv__LoadSession_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__srv__LoadSession_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__srv__LoadSession_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
