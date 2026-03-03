// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from automap_pro:msg/MappingStatusMsg.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__FUNCTIONS_H_
#define AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "automap_pro/msg/rosidl_generator_c__visibility_control.h"

#include "automap_pro/msg/detail/mapping_status_msg__struct.h"

/// Initialize msg/MappingStatusMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * automap_pro__msg__MappingStatusMsg
 * )) before or use
 * automap_pro__msg__MappingStatusMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__msg__MappingStatusMsg__init(automap_pro__msg__MappingStatusMsg * msg);

/// Finalize msg/MappingStatusMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__msg__MappingStatusMsg__fini(automap_pro__msg__MappingStatusMsg * msg);

/// Create msg/MappingStatusMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * automap_pro__msg__MappingStatusMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
automap_pro__msg__MappingStatusMsg *
automap_pro__msg__MappingStatusMsg__create();

/// Destroy msg/MappingStatusMsg message.
/**
 * It calls
 * automap_pro__msg__MappingStatusMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__msg__MappingStatusMsg__destroy(automap_pro__msg__MappingStatusMsg * msg);

/// Check for msg/MappingStatusMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__msg__MappingStatusMsg__are_equal(const automap_pro__msg__MappingStatusMsg * lhs, const automap_pro__msg__MappingStatusMsg * rhs);

/// Copy a msg/MappingStatusMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__msg__MappingStatusMsg__copy(
  const automap_pro__msg__MappingStatusMsg * input,
  automap_pro__msg__MappingStatusMsg * output);

/// Initialize array of msg/MappingStatusMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * automap_pro__msg__MappingStatusMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__msg__MappingStatusMsg__Sequence__init(automap_pro__msg__MappingStatusMsg__Sequence * array, size_t size);

/// Finalize array of msg/MappingStatusMsg messages.
/**
 * It calls
 * automap_pro__msg__MappingStatusMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__msg__MappingStatusMsg__Sequence__fini(automap_pro__msg__MappingStatusMsg__Sequence * array);

/// Create array of msg/MappingStatusMsg messages.
/**
 * It allocates the memory for the array and calls
 * automap_pro__msg__MappingStatusMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
automap_pro__msg__MappingStatusMsg__Sequence *
automap_pro__msg__MappingStatusMsg__Sequence__create(size_t size);

/// Destroy array of msg/MappingStatusMsg messages.
/**
 * It calls
 * automap_pro__msg__MappingStatusMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__msg__MappingStatusMsg__Sequence__destroy(automap_pro__msg__MappingStatusMsg__Sequence * array);

/// Check for msg/MappingStatusMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__msg__MappingStatusMsg__Sequence__are_equal(const automap_pro__msg__MappingStatusMsg__Sequence * lhs, const automap_pro__msg__MappingStatusMsg__Sequence * rhs);

/// Copy an array of msg/MappingStatusMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__msg__MappingStatusMsg__Sequence__copy(
  const automap_pro__msg__MappingStatusMsg__Sequence * input,
  automap_pro__msg__MappingStatusMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__MSG__DETAIL__MAPPING_STATUS_MSG__FUNCTIONS_H_
