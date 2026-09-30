// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from jit_msgs:msg/EstopStatus.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__FUNCTIONS_H_
#define JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "jit_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "jit_msgs/msg/detail/estop_status__struct.h"

/// Initialize msg/EstopStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jit_msgs__msg__EstopStatus
 * )) before or use
 * jit_msgs__msg__EstopStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
bool
jit_msgs__msg__EstopStatus__init(jit_msgs__msg__EstopStatus * msg);

/// Finalize msg/EstopStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
void
jit_msgs__msg__EstopStatus__fini(jit_msgs__msg__EstopStatus * msg);

/// Create msg/EstopStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jit_msgs__msg__EstopStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
jit_msgs__msg__EstopStatus *
jit_msgs__msg__EstopStatus__create();

/// Destroy msg/EstopStatus message.
/**
 * It calls
 * jit_msgs__msg__EstopStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
void
jit_msgs__msg__EstopStatus__destroy(jit_msgs__msg__EstopStatus * msg);

/// Check for msg/EstopStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
bool
jit_msgs__msg__EstopStatus__are_equal(const jit_msgs__msg__EstopStatus * lhs, const jit_msgs__msg__EstopStatus * rhs);

/// Copy a msg/EstopStatus message.
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
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
bool
jit_msgs__msg__EstopStatus__copy(
  const jit_msgs__msg__EstopStatus * input,
  jit_msgs__msg__EstopStatus * output);

/// Initialize array of msg/EstopStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * jit_msgs__msg__EstopStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
bool
jit_msgs__msg__EstopStatus__Sequence__init(jit_msgs__msg__EstopStatus__Sequence * array, size_t size);

/// Finalize array of msg/EstopStatus messages.
/**
 * It calls
 * jit_msgs__msg__EstopStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
void
jit_msgs__msg__EstopStatus__Sequence__fini(jit_msgs__msg__EstopStatus__Sequence * array);

/// Create array of msg/EstopStatus messages.
/**
 * It allocates the memory for the array and calls
 * jit_msgs__msg__EstopStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
jit_msgs__msg__EstopStatus__Sequence *
jit_msgs__msg__EstopStatus__Sequence__create(size_t size);

/// Destroy array of msg/EstopStatus messages.
/**
 * It calls
 * jit_msgs__msg__EstopStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
void
jit_msgs__msg__EstopStatus__Sequence__destroy(jit_msgs__msg__EstopStatus__Sequence * array);

/// Check for msg/EstopStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
bool
jit_msgs__msg__EstopStatus__Sequence__are_equal(const jit_msgs__msg__EstopStatus__Sequence * lhs, const jit_msgs__msg__EstopStatus__Sequence * rhs);

/// Copy an array of msg/EstopStatus messages.
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
ROSIDL_GENERATOR_C_PUBLIC_jit_msgs
bool
jit_msgs__msg__EstopStatus__Sequence__copy(
  const jit_msgs__msg__EstopStatus__Sequence * input,
  jit_msgs__msg__EstopStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__FUNCTIONS_H_
