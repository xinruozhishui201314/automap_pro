// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from automap_pro:msg/GPSMeasurementMsg.idl
// generated code does not contain a copyright notice
#include "automap_pro/msg/detail/gps_measurement_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
automap_pro__msg__GPSMeasurementMsg__init(automap_pro__msg__GPSMeasurementMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    automap_pro__msg__GPSMeasurementMsg__fini(msg);
    return false;
  }
  // position_enu
  // quality
  // hdop
  // num_satellites
  // covariance
  // is_valid
  // latitude
  // longitude
  // altitude
  return true;
}

void
automap_pro__msg__GPSMeasurementMsg__fini(automap_pro__msg__GPSMeasurementMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position_enu
  // quality
  // hdop
  // num_satellites
  // covariance
  // is_valid
  // latitude
  // longitude
  // altitude
}

bool
automap_pro__msg__GPSMeasurementMsg__are_equal(const automap_pro__msg__GPSMeasurementMsg * lhs, const automap_pro__msg__GPSMeasurementMsg * rhs)
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
  // position_enu
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->position_enu[i] != rhs->position_enu[i]) {
      return false;
    }
  }
  // quality
  if (lhs->quality != rhs->quality) {
    return false;
  }
  // hdop
  if (lhs->hdop != rhs->hdop) {
    return false;
  }
  // num_satellites
  if (lhs->num_satellites != rhs->num_satellites) {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->covariance[i] != rhs->covariance[i]) {
      return false;
    }
  }
  // is_valid
  if (lhs->is_valid != rhs->is_valid) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  return true;
}

bool
automap_pro__msg__GPSMeasurementMsg__copy(
  const automap_pro__msg__GPSMeasurementMsg * input,
  automap_pro__msg__GPSMeasurementMsg * output)
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
  // position_enu
  for (size_t i = 0; i < 3; ++i) {
    output->position_enu[i] = input->position_enu[i];
  }
  // quality
  output->quality = input->quality;
  // hdop
  output->hdop = input->hdop;
  // num_satellites
  output->num_satellites = input->num_satellites;
  // covariance
  for (size_t i = 0; i < 9; ++i) {
    output->covariance[i] = input->covariance[i];
  }
  // is_valid
  output->is_valid = input->is_valid;
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  return true;
}

automap_pro__msg__GPSMeasurementMsg *
automap_pro__msg__GPSMeasurementMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__GPSMeasurementMsg * msg = (automap_pro__msg__GPSMeasurementMsg *)allocator.allocate(sizeof(automap_pro__msg__GPSMeasurementMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(automap_pro__msg__GPSMeasurementMsg));
  bool success = automap_pro__msg__GPSMeasurementMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
automap_pro__msg__GPSMeasurementMsg__destroy(automap_pro__msg__GPSMeasurementMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    automap_pro__msg__GPSMeasurementMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
automap_pro__msg__GPSMeasurementMsg__Sequence__init(automap_pro__msg__GPSMeasurementMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__GPSMeasurementMsg * data = NULL;

  if (size) {
    data = (automap_pro__msg__GPSMeasurementMsg *)allocator.zero_allocate(size, sizeof(automap_pro__msg__GPSMeasurementMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = automap_pro__msg__GPSMeasurementMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        automap_pro__msg__GPSMeasurementMsg__fini(&data[i - 1]);
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
automap_pro__msg__GPSMeasurementMsg__Sequence__fini(automap_pro__msg__GPSMeasurementMsg__Sequence * array)
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
      automap_pro__msg__GPSMeasurementMsg__fini(&array->data[i]);
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

automap_pro__msg__GPSMeasurementMsg__Sequence *
automap_pro__msg__GPSMeasurementMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  automap_pro__msg__GPSMeasurementMsg__Sequence * array = (automap_pro__msg__GPSMeasurementMsg__Sequence *)allocator.allocate(sizeof(automap_pro__msg__GPSMeasurementMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = automap_pro__msg__GPSMeasurementMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
automap_pro__msg__GPSMeasurementMsg__Sequence__destroy(automap_pro__msg__GPSMeasurementMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    automap_pro__msg__GPSMeasurementMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
automap_pro__msg__GPSMeasurementMsg__Sequence__are_equal(const automap_pro__msg__GPSMeasurementMsg__Sequence * lhs, const automap_pro__msg__GPSMeasurementMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!automap_pro__msg__GPSMeasurementMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
automap_pro__msg__GPSMeasurementMsg__Sequence__copy(
  const automap_pro__msg__GPSMeasurementMsg__Sequence * input,
  automap_pro__msg__GPSMeasurementMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(automap_pro__msg__GPSMeasurementMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    automap_pro__msg__GPSMeasurementMsg * data =
      (automap_pro__msg__GPSMeasurementMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!automap_pro__msg__GPSMeasurementMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          automap_pro__msg__GPSMeasurementMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!automap_pro__msg__GPSMeasurementMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
