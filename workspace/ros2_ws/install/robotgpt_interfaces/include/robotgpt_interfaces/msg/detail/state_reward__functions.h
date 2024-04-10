// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robotgpt_interfaces:msg/StateReward.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__FUNCTIONS_H_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robotgpt_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "robotgpt_interfaces/msg/detail/state_reward__struct.h"

/// Initialize msg/StateReward message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robotgpt_interfaces__msg__StateReward
 * )) before or use
 * robotgpt_interfaces__msg__StateReward__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
bool
robotgpt_interfaces__msg__StateReward__init(robotgpt_interfaces__msg__StateReward * msg);

/// Finalize msg/StateReward message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
void
robotgpt_interfaces__msg__StateReward__fini(robotgpt_interfaces__msg__StateReward * msg);

/// Create msg/StateReward message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robotgpt_interfaces__msg__StateReward__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
robotgpt_interfaces__msg__StateReward *
robotgpt_interfaces__msg__StateReward__create();

/// Destroy msg/StateReward message.
/**
 * It calls
 * robotgpt_interfaces__msg__StateReward__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
void
robotgpt_interfaces__msg__StateReward__destroy(robotgpt_interfaces__msg__StateReward * msg);

/// Check for msg/StateReward message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
bool
robotgpt_interfaces__msg__StateReward__are_equal(const robotgpt_interfaces__msg__StateReward * lhs, const robotgpt_interfaces__msg__StateReward * rhs);

/// Copy a msg/StateReward message.
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
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
bool
robotgpt_interfaces__msg__StateReward__copy(
  const robotgpt_interfaces__msg__StateReward * input,
  robotgpt_interfaces__msg__StateReward * output);

/// Initialize array of msg/StateReward messages.
/**
 * It allocates the memory for the number of elements and calls
 * robotgpt_interfaces__msg__StateReward__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
bool
robotgpt_interfaces__msg__StateReward__Sequence__init(robotgpt_interfaces__msg__StateReward__Sequence * array, size_t size);

/// Finalize array of msg/StateReward messages.
/**
 * It calls
 * robotgpt_interfaces__msg__StateReward__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
void
robotgpt_interfaces__msg__StateReward__Sequence__fini(robotgpt_interfaces__msg__StateReward__Sequence * array);

/// Create array of msg/StateReward messages.
/**
 * It allocates the memory for the array and calls
 * robotgpt_interfaces__msg__StateReward__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
robotgpt_interfaces__msg__StateReward__Sequence *
robotgpt_interfaces__msg__StateReward__Sequence__create(size_t size);

/// Destroy array of msg/StateReward messages.
/**
 * It calls
 * robotgpt_interfaces__msg__StateReward__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
void
robotgpt_interfaces__msg__StateReward__Sequence__destroy(robotgpt_interfaces__msg__StateReward__Sequence * array);

/// Check for msg/StateReward message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
bool
robotgpt_interfaces__msg__StateReward__Sequence__are_equal(const robotgpt_interfaces__msg__StateReward__Sequence * lhs, const robotgpt_interfaces__msg__StateReward__Sequence * rhs);

/// Copy an array of msg/StateReward messages.
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
ROSIDL_GENERATOR_C_PUBLIC_robotgpt_interfaces
bool
robotgpt_interfaces__msg__StateReward__Sequence__copy(
  const robotgpt_interfaces__msg__StateReward__Sequence * input,
  robotgpt_interfaces__msg__StateReward__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__FUNCTIONS_H_
