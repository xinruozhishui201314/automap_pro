// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from automap_pro:srv/TriggerOptimize.idl
// generated code does not contain a copyright notice

#ifndef AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__FUNCTIONS_H_
#define AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "automap_pro/msg/rosidl_generator_c__visibility_control.h"

#include "automap_pro/srv/detail/trigger_optimize__struct.h"

/// Initialize srv/TriggerOptimize message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * automap_pro__srv__TriggerOptimize_Request
 * )) before or use
 * automap_pro__srv__TriggerOptimize_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Request__init(automap_pro__srv__TriggerOptimize_Request * msg);

/// Finalize srv/TriggerOptimize message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Request__fini(automap_pro__srv__TriggerOptimize_Request * msg);

/// Create srv/TriggerOptimize message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * automap_pro__srv__TriggerOptimize_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
automap_pro__srv__TriggerOptimize_Request *
automap_pro__srv__TriggerOptimize_Request__create();

/// Destroy srv/TriggerOptimize message.
/**
 * It calls
 * automap_pro__srv__TriggerOptimize_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Request__destroy(automap_pro__srv__TriggerOptimize_Request * msg);

/// Check for srv/TriggerOptimize message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Request__are_equal(const automap_pro__srv__TriggerOptimize_Request * lhs, const automap_pro__srv__TriggerOptimize_Request * rhs);

/// Copy a srv/TriggerOptimize message.
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
automap_pro__srv__TriggerOptimize_Request__copy(
  const automap_pro__srv__TriggerOptimize_Request * input,
  automap_pro__srv__TriggerOptimize_Request * output);

/// Initialize array of srv/TriggerOptimize messages.
/**
 * It allocates the memory for the number of elements and calls
 * automap_pro__srv__TriggerOptimize_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Request__Sequence__init(automap_pro__srv__TriggerOptimize_Request__Sequence * array, size_t size);

/// Finalize array of srv/TriggerOptimize messages.
/**
 * It calls
 * automap_pro__srv__TriggerOptimize_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Request__Sequence__fini(automap_pro__srv__TriggerOptimize_Request__Sequence * array);

/// Create array of srv/TriggerOptimize messages.
/**
 * It allocates the memory for the array and calls
 * automap_pro__srv__TriggerOptimize_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
automap_pro__srv__TriggerOptimize_Request__Sequence *
automap_pro__srv__TriggerOptimize_Request__Sequence__create(size_t size);

/// Destroy array of srv/TriggerOptimize messages.
/**
 * It calls
 * automap_pro__srv__TriggerOptimize_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Request__Sequence__destroy(automap_pro__srv__TriggerOptimize_Request__Sequence * array);

/// Check for srv/TriggerOptimize message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Request__Sequence__are_equal(const automap_pro__srv__TriggerOptimize_Request__Sequence * lhs, const automap_pro__srv__TriggerOptimize_Request__Sequence * rhs);

/// Copy an array of srv/TriggerOptimize messages.
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
automap_pro__srv__TriggerOptimize_Request__Sequence__copy(
  const automap_pro__srv__TriggerOptimize_Request__Sequence * input,
  automap_pro__srv__TriggerOptimize_Request__Sequence * output);

/// Initialize srv/TriggerOptimize message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * automap_pro__srv__TriggerOptimize_Response
 * )) before or use
 * automap_pro__srv__TriggerOptimize_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Response__init(automap_pro__srv__TriggerOptimize_Response * msg);

/// Finalize srv/TriggerOptimize message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Response__fini(automap_pro__srv__TriggerOptimize_Response * msg);

/// Create srv/TriggerOptimize message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * automap_pro__srv__TriggerOptimize_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
automap_pro__srv__TriggerOptimize_Response *
automap_pro__srv__TriggerOptimize_Response__create();

/// Destroy srv/TriggerOptimize message.
/**
 * It calls
 * automap_pro__srv__TriggerOptimize_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Response__destroy(automap_pro__srv__TriggerOptimize_Response * msg);

/// Check for srv/TriggerOptimize message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Response__are_equal(const automap_pro__srv__TriggerOptimize_Response * lhs, const automap_pro__srv__TriggerOptimize_Response * rhs);

/// Copy a srv/TriggerOptimize message.
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
automap_pro__srv__TriggerOptimize_Response__copy(
  const automap_pro__srv__TriggerOptimize_Response * input,
  automap_pro__srv__TriggerOptimize_Response * output);

/// Initialize array of srv/TriggerOptimize messages.
/**
 * It allocates the memory for the number of elements and calls
 * automap_pro__srv__TriggerOptimize_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Response__Sequence__init(automap_pro__srv__TriggerOptimize_Response__Sequence * array, size_t size);

/// Finalize array of srv/TriggerOptimize messages.
/**
 * It calls
 * automap_pro__srv__TriggerOptimize_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Response__Sequence__fini(automap_pro__srv__TriggerOptimize_Response__Sequence * array);

/// Create array of srv/TriggerOptimize messages.
/**
 * It allocates the memory for the array and calls
 * automap_pro__srv__TriggerOptimize_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
automap_pro__srv__TriggerOptimize_Response__Sequence *
automap_pro__srv__TriggerOptimize_Response__Sequence__create(size_t size);

/// Destroy array of srv/TriggerOptimize messages.
/**
 * It calls
 * automap_pro__srv__TriggerOptimize_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
void
automap_pro__srv__TriggerOptimize_Response__Sequence__destroy(automap_pro__srv__TriggerOptimize_Response__Sequence * array);

/// Check for srv/TriggerOptimize message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_automap_pro
bool
automap_pro__srv__TriggerOptimize_Response__Sequence__are_equal(const automap_pro__srv__TriggerOptimize_Response__Sequence * lhs, const automap_pro__srv__TriggerOptimize_Response__Sequence * rhs);

/// Copy an array of srv/TriggerOptimize messages.
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
automap_pro__srv__TriggerOptimize_Response__Sequence__copy(
  const automap_pro__srv__TriggerOptimize_Response__Sequence * input,
  automap_pro__srv__TriggerOptimize_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AUTOMAP_PRO__SRV__DETAIL__TRIGGER_OPTIMIZE__FUNCTIONS_H_
