// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from overlap_transformer_msgs:srv/ComputeDescriptor.idl
// generated code does not contain a copyright notice
#include "overlap_transformer_msgs/srv/detail/compute_descriptor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pointcloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Request__init(overlap_transformer_msgs__srv__ComputeDescriptor_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pointcloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->pointcloud)) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(msg);
    return false;
  }
  return true;
}

void
overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(overlap_transformer_msgs__srv__ComputeDescriptor_Request * msg)
{
  if (!msg) {
    return;
  }
  // pointcloud
  sensor_msgs__msg__PointCloud2__fini(&msg->pointcloud);
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Request__are_equal(const overlap_transformer_msgs__srv__ComputeDescriptor_Request * lhs, const overlap_transformer_msgs__srv__ComputeDescriptor_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pointcloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->pointcloud), &(rhs->pointcloud)))
  {
    return false;
  }
  return true;
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Request__copy(
  const overlap_transformer_msgs__srv__ComputeDescriptor_Request * input,
  overlap_transformer_msgs__srv__ComputeDescriptor_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pointcloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->pointcloud), &(output->pointcloud)))
  {
    return false;
  }
  return true;
}

overlap_transformer_msgs__srv__ComputeDescriptor_Request *
overlap_transformer_msgs__srv__ComputeDescriptor_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  overlap_transformer_msgs__srv__ComputeDescriptor_Request * msg = (overlap_transformer_msgs__srv__ComputeDescriptor_Request *)allocator.allocate(sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Request));
  bool success = overlap_transformer_msgs__srv__ComputeDescriptor_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
overlap_transformer_msgs__srv__ComputeDescriptor_Request__destroy(overlap_transformer_msgs__srv__ComputeDescriptor_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__init(overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  overlap_transformer_msgs__srv__ComputeDescriptor_Request * data = NULL;

  if (size) {
    data = (overlap_transformer_msgs__srv__ComputeDescriptor_Request *)allocator.zero_allocate(size, sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = overlap_transformer_msgs__srv__ComputeDescriptor_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(&data[i - 1]);
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
overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__fini(overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * array)
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
      overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(&array->data[i]);
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

overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence *
overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * array = (overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence *)allocator.allocate(sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__destroy(overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__are_equal(const overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * lhs, const overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!overlap_transformer_msgs__srv__ComputeDescriptor_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence__copy(
  const overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * input,
  overlap_transformer_msgs__srv__ComputeDescriptor_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    overlap_transformer_msgs__srv__ComputeDescriptor_Request * data =
      (overlap_transformer_msgs__srv__ComputeDescriptor_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!overlap_transformer_msgs__srv__ComputeDescriptor_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          overlap_transformer_msgs__srv__ComputeDescriptor_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!overlap_transformer_msgs__srv__ComputeDescriptor_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `descriptor`
#include "std_msgs/msg/detail/float32_multi_array__functions.h"

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Response__init(overlap_transformer_msgs__srv__ComputeDescriptor_Response * msg)
{
  if (!msg) {
    return false;
  }
  // descriptor
  if (!std_msgs__msg__Float32MultiArray__init(&msg->descriptor)) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(msg);
    return false;
  }
  return true;
}

void
overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(overlap_transformer_msgs__srv__ComputeDescriptor_Response * msg)
{
  if (!msg) {
    return;
  }
  // descriptor
  std_msgs__msg__Float32MultiArray__fini(&msg->descriptor);
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Response__are_equal(const overlap_transformer_msgs__srv__ComputeDescriptor_Response * lhs, const overlap_transformer_msgs__srv__ComputeDescriptor_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // descriptor
  if (!std_msgs__msg__Float32MultiArray__are_equal(
      &(lhs->descriptor), &(rhs->descriptor)))
  {
    return false;
  }
  return true;
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Response__copy(
  const overlap_transformer_msgs__srv__ComputeDescriptor_Response * input,
  overlap_transformer_msgs__srv__ComputeDescriptor_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // descriptor
  if (!std_msgs__msg__Float32MultiArray__copy(
      &(input->descriptor), &(output->descriptor)))
  {
    return false;
  }
  return true;
}

overlap_transformer_msgs__srv__ComputeDescriptor_Response *
overlap_transformer_msgs__srv__ComputeDescriptor_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  overlap_transformer_msgs__srv__ComputeDescriptor_Response * msg = (overlap_transformer_msgs__srv__ComputeDescriptor_Response *)allocator.allocate(sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Response));
  bool success = overlap_transformer_msgs__srv__ComputeDescriptor_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
overlap_transformer_msgs__srv__ComputeDescriptor_Response__destroy(overlap_transformer_msgs__srv__ComputeDescriptor_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__init(overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  overlap_transformer_msgs__srv__ComputeDescriptor_Response * data = NULL;

  if (size) {
    data = (overlap_transformer_msgs__srv__ComputeDescriptor_Response *)allocator.zero_allocate(size, sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = overlap_transformer_msgs__srv__ComputeDescriptor_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(&data[i - 1]);
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
overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__fini(overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * array)
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
      overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(&array->data[i]);
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

overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence *
overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * array = (overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence *)allocator.allocate(sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__destroy(overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__are_equal(const overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * lhs, const overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!overlap_transformer_msgs__srv__ComputeDescriptor_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence__copy(
  const overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * input,
  overlap_transformer_msgs__srv__ComputeDescriptor_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(overlap_transformer_msgs__srv__ComputeDescriptor_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    overlap_transformer_msgs__srv__ComputeDescriptor_Response * data =
      (overlap_transformer_msgs__srv__ComputeDescriptor_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!overlap_transformer_msgs__srv__ComputeDescriptor_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          overlap_transformer_msgs__srv__ComputeDescriptor_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!overlap_transformer_msgs__srv__ComputeDescriptor_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
