// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:action/RunHBA.idl
// generated code does not contain a copyright notice
#include "automap_pro/action/detail/run_hba__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data_path`
#include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__action__RunHBA_Goal__init(automap_pro__action__RunHBA_Goal * msg)
{
  if (!msg) {
    return false;
  }
  // data_path
  if (!rosidl_runtime_c__String__init(&msg->data_path)) {
    automap_pro__action__RunHBA_Goal__fini(msg);
    return false;
  }
  // total_layer_num
  // thread_num
  // enable_gps
  return true;
}

void
automap_pro__action__RunHBA_Goal__fini(automap_pro__action__RunHBA_Goal * msg)
{
  if (!msg) {
    return;
  }
  // data_path
  rosidl_runtime_c__String__fini(&msg->data_path);
  // total_layer_num
  // thread_num
  // enable_gps
}

bool
automap_pro__action__RunHBA_Goal__are_equal(const automap_pro__action__RunHBA_Goal * lhs, const automap_pro__action__RunHBA_Goal * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data_path
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data_path), &(rhs->data_path)))
  {
    return false;
  }
  // total_layer_num
  if (lhs->total_layer_num != rhs->total_layer_num) {
    return false;
  }
  // thread_num
  if (lhs->thread_num != rhs->thread_num) {
    return false;
  }
  // enable_gps
  if (lhs->enable_gps != rhs->enable_gps) {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_Goal__copy(
  const automap_pro__action__RunHBA_Goal * input,
  automap_pro__action__RunHBA_Goal * output)
{
  if (!input || !output) {
    return false;
  }
  // data_path
  if (!rosidl_runtime_c__String__copy(
      &(input->data_path), &(output->data_path)))
  {
    return false;
  }
  // total_layer_num
  output->total_layer_num = input->total_layer_num;
  // thread_num
  output->thread_num = input->thread_num;
  // enable_gps
  output->enable_gps = input->enable_gps;
  return true;
}

automap_pro__action__RunHBA_Goal *
automap_pro__action__RunHBA_Goal__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Goal * msg = (automap_pro__action__RunHBA_Goal *)allocator.allocate(sizeof(automap_pro__action__RunHBA_Goal), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_Goal));
  bool success = automap_pro__action__RunHBA_Goal__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_Goal__destroy(automap_pro__action__RunHBA_Goal * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_Goal__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_Goal__Sequence__init(automap_pro__action__RunHBA_Goal__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Goal * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_Goal *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_Goal), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_Goal__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_Goal__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_Goal__Sequence__fini(automap_pro__action__RunHBA_Goal__Sequence * array)
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
      automap_pro__action__RunHBA_Goal__fini(&array->data[i]);
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

automap_pro__action__RunHBA_Goal__Sequence *
automap_pro__action__RunHBA_Goal__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Goal__Sequence * array = (automap_pro__action__RunHBA_Goal__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_Goal__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_Goal__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_Goal__Sequence__destroy(automap_pro__action__RunHBA_Goal__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_Goal__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_Goal__Sequence__are_equal(const automap_pro__action__RunHBA_Goal__Sequence * lhs, const automap_pro__action__RunHBA_Goal__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_Goal__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_Goal__Sequence__copy(
  const automap_pro__action__RunHBA_Goal__Sequence * input,
  automap_pro__action__RunHBA_Goal__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_Goal);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_Goal * data =
      (automap_pro__action__RunHBA_Goal *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_Goal__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_Goal__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_Goal__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `output_path`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
automap_pro__action__RunHBA_Result__init(automap_pro__action__RunHBA_Result * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // output_path
  if (!rosidl_runtime_c__String__init(&msg->output_path)) {
    automap_pro__action__RunHBA_Result__fini(msg);
    return false;
  }
  // elapsed_seconds
  // final_mme
  // total_keyframes
  return true;
}

void
automap_pro__action__RunHBA_Result__fini(automap_pro__action__RunHBA_Result * msg)
{
  if (!msg) {
    return;
  }
  // success
  // output_path
  rosidl_runtime_c__String__fini(&msg->output_path);
  // elapsed_seconds
  // final_mme
  // total_keyframes
}

bool
automap_pro__action__RunHBA_Result__are_equal(const automap_pro__action__RunHBA_Result * lhs, const automap_pro__action__RunHBA_Result * rhs)
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
  // elapsed_seconds
  if (lhs->elapsed_seconds != rhs->elapsed_seconds) {
    return false;
  }
  // final_mme
  if (lhs->final_mme != rhs->final_mme) {
    return false;
  }
  // total_keyframes
  if (lhs->total_keyframes != rhs->total_keyframes) {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_Result__copy(
  const automap_pro__action__RunHBA_Result * input,
  automap_pro__action__RunHBA_Result * output)
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
  // elapsed_seconds
  output->elapsed_seconds = input->elapsed_seconds;
  // final_mme
  output->final_mme = input->final_mme;
  // total_keyframes
  output->total_keyframes = input->total_keyframes;
  return true;
}

automap_pro__action__RunHBA_Result *
automap_pro__action__RunHBA_Result__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Result * msg = (automap_pro__action__RunHBA_Result *)allocator.allocate(sizeof(automap_pro__action__RunHBA_Result), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_Result));
  bool success = automap_pro__action__RunHBA_Result__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_Result__destroy(automap_pro__action__RunHBA_Result * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_Result__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_Result__Sequence__init(automap_pro__action__RunHBA_Result__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Result * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_Result *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_Result), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_Result__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_Result__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_Result__Sequence__fini(automap_pro__action__RunHBA_Result__Sequence * array)
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
      automap_pro__action__RunHBA_Result__fini(&array->data[i]);
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

automap_pro__action__RunHBA_Result__Sequence *
automap_pro__action__RunHBA_Result__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Result__Sequence * array = (automap_pro__action__RunHBA_Result__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_Result__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_Result__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_Result__Sequence__destroy(automap_pro__action__RunHBA_Result__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_Result__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_Result__Sequence__are_equal(const automap_pro__action__RunHBA_Result__Sequence * lhs, const automap_pro__action__RunHBA_Result__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_Result__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_Result__Sequence__copy(
  const automap_pro__action__RunHBA_Result__Sequence * input,
  automap_pro__action__RunHBA_Result__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_Result);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_Result * data =
      (automap_pro__action__RunHBA_Result *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_Result__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_Result__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_Result__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
automap_pro__action__RunHBA_Feedback__init(automap_pro__action__RunHBA_Feedback * msg)
{
  if (!msg) {
    return false;
  }
  // current_layer
  // total_layers
  // progress_percent
  // layer_elapsed_ms
  return true;
}

void
automap_pro__action__RunHBA_Feedback__fini(automap_pro__action__RunHBA_Feedback * msg)
{
  if (!msg) {
    return;
  }
  // current_layer
  // total_layers
  // progress_percent
  // layer_elapsed_ms
}

bool
automap_pro__action__RunHBA_Feedback__are_equal(const automap_pro__action__RunHBA_Feedback * lhs, const automap_pro__action__RunHBA_Feedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_layer
  if (lhs->current_layer != rhs->current_layer) {
    return false;
  }
  // total_layers
  if (lhs->total_layers != rhs->total_layers) {
    return false;
  }
  // progress_percent
  if (lhs->progress_percent != rhs->progress_percent) {
    return false;
  }
  // layer_elapsed_ms
  if (lhs->layer_elapsed_ms != rhs->layer_elapsed_ms) {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_Feedback__copy(
  const automap_pro__action__RunHBA_Feedback * input,
  automap_pro__action__RunHBA_Feedback * output)
{
  if (!input || !output) {
    return false;
  }
  // current_layer
  output->current_layer = input->current_layer;
  // total_layers
  output->total_layers = input->total_layers;
  // progress_percent
  output->progress_percent = input->progress_percent;
  // layer_elapsed_ms
  output->layer_elapsed_ms = input->layer_elapsed_ms;
  return true;
}

automap_pro__action__RunHBA_Feedback *
automap_pro__action__RunHBA_Feedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Feedback * msg = (automap_pro__action__RunHBA_Feedback *)allocator.allocate(sizeof(automap_pro__action__RunHBA_Feedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_Feedback));
  bool success = automap_pro__action__RunHBA_Feedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_Feedback__destroy(automap_pro__action__RunHBA_Feedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_Feedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_Feedback__Sequence__init(automap_pro__action__RunHBA_Feedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Feedback * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_Feedback *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_Feedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_Feedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_Feedback__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_Feedback__Sequence__fini(automap_pro__action__RunHBA_Feedback__Sequence * array)
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
      automap_pro__action__RunHBA_Feedback__fini(&array->data[i]);
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

automap_pro__action__RunHBA_Feedback__Sequence *
automap_pro__action__RunHBA_Feedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_Feedback__Sequence * array = (automap_pro__action__RunHBA_Feedback__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_Feedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_Feedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_Feedback__Sequence__destroy(automap_pro__action__RunHBA_Feedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_Feedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_Feedback__Sequence__are_equal(const automap_pro__action__RunHBA_Feedback__Sequence * lhs, const automap_pro__action__RunHBA_Feedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_Feedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_Feedback__Sequence__copy(
  const automap_pro__action__RunHBA_Feedback__Sequence * input,
  automap_pro__action__RunHBA_Feedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_Feedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_Feedback * data =
      (automap_pro__action__RunHBA_Feedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_Feedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_Feedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_Feedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `goal`
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"

bool
automap_pro__action__RunHBA_SendGoal_Request__init(automap_pro__action__RunHBA_SendGoal_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    automap_pro__action__RunHBA_SendGoal_Request__fini(msg);
    return false;
  }
  // goal
  if (!automap_pro__action__RunHBA_Goal__init(&msg->goal)) {
    automap_pro__action__RunHBA_SendGoal_Request__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__action__RunHBA_SendGoal_Request__fini(automap_pro__action__RunHBA_SendGoal_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // goal
  automap_pro__action__RunHBA_Goal__fini(&msg->goal);
}

bool
automap_pro__action__RunHBA_SendGoal_Request__are_equal(const automap_pro__action__RunHBA_SendGoal_Request * lhs, const automap_pro__action__RunHBA_SendGoal_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // goal
  if (!automap_pro__action__RunHBA_Goal__are_equal(
      &(lhs->goal), &(rhs->goal)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_SendGoal_Request__copy(
  const automap_pro__action__RunHBA_SendGoal_Request * input,
  automap_pro__action__RunHBA_SendGoal_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // goal
  if (!automap_pro__action__RunHBA_Goal__copy(
      &(input->goal), &(output->goal)))
  {
    return false;
  }
  return true;
}

automap_pro__action__RunHBA_SendGoal_Request *
automap_pro__action__RunHBA_SendGoal_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_SendGoal_Request * msg = (automap_pro__action__RunHBA_SendGoal_Request *)allocator.allocate(sizeof(automap_pro__action__RunHBA_SendGoal_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_SendGoal_Request));
  bool success = automap_pro__action__RunHBA_SendGoal_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_SendGoal_Request__destroy(automap_pro__action__RunHBA_SendGoal_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_SendGoal_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_SendGoal_Request__Sequence__init(automap_pro__action__RunHBA_SendGoal_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_SendGoal_Request * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_SendGoal_Request *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_SendGoal_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_SendGoal_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_SendGoal_Request__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_SendGoal_Request__Sequence__fini(automap_pro__action__RunHBA_SendGoal_Request__Sequence * array)
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
      automap_pro__action__RunHBA_SendGoal_Request__fini(&array->data[i]);
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

automap_pro__action__RunHBA_SendGoal_Request__Sequence *
automap_pro__action__RunHBA_SendGoal_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_SendGoal_Request__Sequence * array = (automap_pro__action__RunHBA_SendGoal_Request__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_SendGoal_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_SendGoal_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_SendGoal_Request__Sequence__destroy(automap_pro__action__RunHBA_SendGoal_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_SendGoal_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_SendGoal_Request__Sequence__are_equal(const automap_pro__action__RunHBA_SendGoal_Request__Sequence * lhs, const automap_pro__action__RunHBA_SendGoal_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_SendGoal_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_SendGoal_Request__Sequence__copy(
  const automap_pro__action__RunHBA_SendGoal_Request__Sequence * input,
  automap_pro__action__RunHBA_SendGoal_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_SendGoal_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_SendGoal_Request * data =
      (automap_pro__action__RunHBA_SendGoal_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_SendGoal_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_SendGoal_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_SendGoal_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
automap_pro__action__RunHBA_SendGoal_Response__init(automap_pro__action__RunHBA_SendGoal_Response * msg)
{
  if (!msg) {
    return false;
  }
  // accepted
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    automap_pro__action__RunHBA_SendGoal_Response__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__action__RunHBA_SendGoal_Response__fini(automap_pro__action__RunHBA_SendGoal_Response * msg)
{
  if (!msg) {
    return;
  }
  // accepted
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
}

bool
automap_pro__action__RunHBA_SendGoal_Response__are_equal(const automap_pro__action__RunHBA_SendGoal_Response * lhs, const automap_pro__action__RunHBA_SendGoal_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accepted
  if (lhs->accepted != rhs->accepted) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_SendGoal_Response__copy(
  const automap_pro__action__RunHBA_SendGoal_Response * input,
  automap_pro__action__RunHBA_SendGoal_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // accepted
  output->accepted = input->accepted;
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  return true;
}

automap_pro__action__RunHBA_SendGoal_Response *
automap_pro__action__RunHBA_SendGoal_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_SendGoal_Response * msg = (automap_pro__action__RunHBA_SendGoal_Response *)allocator.allocate(sizeof(automap_pro__action__RunHBA_SendGoal_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_SendGoal_Response));
  bool success = automap_pro__action__RunHBA_SendGoal_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_SendGoal_Response__destroy(automap_pro__action__RunHBA_SendGoal_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_SendGoal_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_SendGoal_Response__Sequence__init(automap_pro__action__RunHBA_SendGoal_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_SendGoal_Response * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_SendGoal_Response *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_SendGoal_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_SendGoal_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_SendGoal_Response__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_SendGoal_Response__Sequence__fini(automap_pro__action__RunHBA_SendGoal_Response__Sequence * array)
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
      automap_pro__action__RunHBA_SendGoal_Response__fini(&array->data[i]);
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

automap_pro__action__RunHBA_SendGoal_Response__Sequence *
automap_pro__action__RunHBA_SendGoal_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_SendGoal_Response__Sequence * array = (automap_pro__action__RunHBA_SendGoal_Response__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_SendGoal_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_SendGoal_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_SendGoal_Response__Sequence__destroy(automap_pro__action__RunHBA_SendGoal_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_SendGoal_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_SendGoal_Response__Sequence__are_equal(const automap_pro__action__RunHBA_SendGoal_Response__Sequence * lhs, const automap_pro__action__RunHBA_SendGoal_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_SendGoal_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_SendGoal_Response__Sequence__copy(
  const automap_pro__action__RunHBA_SendGoal_Response__Sequence * input,
  automap_pro__action__RunHBA_SendGoal_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_SendGoal_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_SendGoal_Response * data =
      (automap_pro__action__RunHBA_SendGoal_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_SendGoal_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_SendGoal_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_SendGoal_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"

bool
automap_pro__action__RunHBA_GetResult_Request__init(automap_pro__action__RunHBA_GetResult_Request * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    automap_pro__action__RunHBA_GetResult_Request__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__action__RunHBA_GetResult_Request__fini(automap_pro__action__RunHBA_GetResult_Request * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
}

bool
automap_pro__action__RunHBA_GetResult_Request__are_equal(const automap_pro__action__RunHBA_GetResult_Request * lhs, const automap_pro__action__RunHBA_GetResult_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_GetResult_Request__copy(
  const automap_pro__action__RunHBA_GetResult_Request * input,
  automap_pro__action__RunHBA_GetResult_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  return true;
}

automap_pro__action__RunHBA_GetResult_Request *
automap_pro__action__RunHBA_GetResult_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_GetResult_Request * msg = (automap_pro__action__RunHBA_GetResult_Request *)allocator.allocate(sizeof(automap_pro__action__RunHBA_GetResult_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_GetResult_Request));
  bool success = automap_pro__action__RunHBA_GetResult_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_GetResult_Request__destroy(automap_pro__action__RunHBA_GetResult_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_GetResult_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_GetResult_Request__Sequence__init(automap_pro__action__RunHBA_GetResult_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_GetResult_Request * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_GetResult_Request *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_GetResult_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_GetResult_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_GetResult_Request__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_GetResult_Request__Sequence__fini(automap_pro__action__RunHBA_GetResult_Request__Sequence * array)
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
      automap_pro__action__RunHBA_GetResult_Request__fini(&array->data[i]);
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

automap_pro__action__RunHBA_GetResult_Request__Sequence *
automap_pro__action__RunHBA_GetResult_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_GetResult_Request__Sequence * array = (automap_pro__action__RunHBA_GetResult_Request__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_GetResult_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_GetResult_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_GetResult_Request__Sequence__destroy(automap_pro__action__RunHBA_GetResult_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_GetResult_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_GetResult_Request__Sequence__are_equal(const automap_pro__action__RunHBA_GetResult_Request__Sequence * lhs, const automap_pro__action__RunHBA_GetResult_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_GetResult_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_GetResult_Request__Sequence__copy(
  const automap_pro__action__RunHBA_GetResult_Request__Sequence * input,
  automap_pro__action__RunHBA_GetResult_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_GetResult_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_GetResult_Request * data =
      (automap_pro__action__RunHBA_GetResult_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_GetResult_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_GetResult_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_GetResult_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `result`
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"

bool
automap_pro__action__RunHBA_GetResult_Response__init(automap_pro__action__RunHBA_GetResult_Response * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // result
  if (!automap_pro__action__RunHBA_Result__init(&msg->result)) {
    automap_pro__action__RunHBA_GetResult_Response__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__action__RunHBA_GetResult_Response__fini(automap_pro__action__RunHBA_GetResult_Response * msg)
{
  if (!msg) {
    return;
  }
  // status
  // result
  automap_pro__action__RunHBA_Result__fini(&msg->result);
}

bool
automap_pro__action__RunHBA_GetResult_Response__are_equal(const automap_pro__action__RunHBA_GetResult_Response * lhs, const automap_pro__action__RunHBA_GetResult_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // result
  if (!automap_pro__action__RunHBA_Result__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_GetResult_Response__copy(
  const automap_pro__action__RunHBA_GetResult_Response * input,
  automap_pro__action__RunHBA_GetResult_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // result
  if (!automap_pro__action__RunHBA_Result__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  return true;
}

automap_pro__action__RunHBA_GetResult_Response *
automap_pro__action__RunHBA_GetResult_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_GetResult_Response * msg = (automap_pro__action__RunHBA_GetResult_Response *)allocator.allocate(sizeof(automap_pro__action__RunHBA_GetResult_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_GetResult_Response));
  bool success = automap_pro__action__RunHBA_GetResult_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_GetResult_Response__destroy(automap_pro__action__RunHBA_GetResult_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_GetResult_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_GetResult_Response__Sequence__init(automap_pro__action__RunHBA_GetResult_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_GetResult_Response * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_GetResult_Response *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_GetResult_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_GetResult_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_GetResult_Response__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_GetResult_Response__Sequence__fini(automap_pro__action__RunHBA_GetResult_Response__Sequence * array)
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
      automap_pro__action__RunHBA_GetResult_Response__fini(&array->data[i]);
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

automap_pro__action__RunHBA_GetResult_Response__Sequence *
automap_pro__action__RunHBA_GetResult_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_GetResult_Response__Sequence * array = (automap_pro__action__RunHBA_GetResult_Response__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_GetResult_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_GetResult_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_GetResult_Response__Sequence__destroy(automap_pro__action__RunHBA_GetResult_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_GetResult_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_GetResult_Response__Sequence__are_equal(const automap_pro__action__RunHBA_GetResult_Response__Sequence * lhs, const automap_pro__action__RunHBA_GetResult_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_GetResult_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_GetResult_Response__Sequence__copy(
  const automap_pro__action__RunHBA_GetResult_Response__Sequence * input,
  automap_pro__action__RunHBA_GetResult_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_GetResult_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_GetResult_Response * data =
      (automap_pro__action__RunHBA_GetResult_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_GetResult_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_GetResult_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_GetResult_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__functions.h"
// Member `feedback`
// already included above
// #include "automap_pro/action/detail/run_hba__functions.h"

bool
automap_pro__action__RunHBA_FeedbackMessage__init(automap_pro__action__RunHBA_FeedbackMessage * msg)
{
  if (!msg) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__init(&msg->goal_id)) {
    automap_pro__action__RunHBA_FeedbackMessage__fini(msg);
    return false;
  }
  // feedback
  if (!automap_pro__action__RunHBA_Feedback__init(&msg->feedback)) {
    automap_pro__action__RunHBA_FeedbackMessage__fini(msg);
    return false;
  }
  return true;
}

void
automap_pro__action__RunHBA_FeedbackMessage__fini(automap_pro__action__RunHBA_FeedbackMessage * msg)
{
  if (!msg) {
    return;
  }
  // goal_id
  unique_identifier_msgs__msg__UUID__fini(&msg->goal_id);
  // feedback
  automap_pro__action__RunHBA_Feedback__fini(&msg->feedback);
}

bool
automap_pro__action__RunHBA_FeedbackMessage__are_equal(const automap_pro__action__RunHBA_FeedbackMessage * lhs, const automap_pro__action__RunHBA_FeedbackMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__are_equal(
      &(lhs->goal_id), &(rhs->goal_id)))
  {
    return false;
  }
  // feedback
  if (!automap_pro__action__RunHBA_Feedback__are_equal(
      &(lhs->feedback), &(rhs->feedback)))
  {
    return false;
  }
  return true;
}

bool
automap_pro__action__RunHBA_FeedbackMessage__copy(
  const automap_pro__action__RunHBA_FeedbackMessage * input,
  automap_pro__action__RunHBA_FeedbackMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // goal_id
  if (!unique_identifier_msgs__msg__UUID__copy(
      &(input->goal_id), &(output->goal_id)))
  {
    return false;
  }
  // feedback
  if (!automap_pro__action__RunHBA_Feedback__copy(
      &(input->feedback), &(output->feedback)))
  {
    return false;
  }
  return true;
}

automap_pro__action__RunHBA_FeedbackMessage *
automap_pro__action__RunHBA_FeedbackMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_FeedbackMessage * msg = (automap_pro__action__RunHBA_FeedbackMessage *)allocator.allocate(sizeof(automap_pro__action__RunHBA_FeedbackMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__action__RunHBA_FeedbackMessage));
  bool success = automap_pro__action__RunHBA_FeedbackMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__action__RunHBA_FeedbackMessage__destroy(automap_pro__action__RunHBA_FeedbackMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__action__RunHBA_FeedbackMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__action__RunHBA_FeedbackMessage__Sequence__init(automap_pro__action__RunHBA_FeedbackMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_FeedbackMessage * data = NULL;

  if (size) {
    data = (automap_pro__action__RunHBA_FeedbackMessage *)allocator.zero_allocate(size, sizeof(automap_pro__action__RunHBA_FeedbackMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__action__RunHBA_FeedbackMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__action__RunHBA_FeedbackMessage__fini(&data[i - 1]);
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
automap_pro__action__RunHBA_FeedbackMessage__Sequence__fini(automap_pro__action__RunHBA_FeedbackMessage__Sequence * array)
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
      automap_pro__action__RunHBA_FeedbackMessage__fini(&array->data[i]);
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

automap_pro__action__RunHBA_FeedbackMessage__Sequence *
automap_pro__action__RunHBA_FeedbackMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__action__RunHBA_FeedbackMessage__Sequence * array = (automap_pro__action__RunHBA_FeedbackMessage__Sequence *)allocator.allocate(sizeof(automap_pro__action__RunHBA_FeedbackMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__action__RunHBA_FeedbackMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__action__RunHBA_FeedbackMessage__Sequence__destroy(automap_pro__action__RunHBA_FeedbackMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__action__RunHBA_FeedbackMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__action__RunHBA_FeedbackMessage__Sequence__are_equal(const automap_pro__action__RunHBA_FeedbackMessage__Sequence * lhs, const automap_pro__action__RunHBA_FeedbackMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__action__RunHBA_FeedbackMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__action__RunHBA_FeedbackMessage__Sequence__copy(
  const automap_pro__action__RunHBA_FeedbackMessage__Sequence * input,
  automap_pro__action__RunHBA_FeedbackMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__action__RunHBA_FeedbackMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__action__RunHBA_FeedbackMessage * data =
      (automap_pro__action__RunHBA_FeedbackMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__action__RunHBA_FeedbackMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__action__RunHBA_FeedbackMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__action__RunHBA_FeedbackMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
