// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotgpt_interfaces:msg/Action.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__ACTION__STRUCT_H_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'action'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/Action in the package robotgpt_interfaces.
typedef struct robotgpt_interfaces__msg__Action
{
  rosidl_runtime_c__float__Sequence action;
} robotgpt_interfaces__msg__Action;

// Struct for a sequence of robotgpt_interfaces__msg__Action.
typedef struct robotgpt_interfaces__msg__Action__Sequence
{
  robotgpt_interfaces__msg__Action * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotgpt_interfaces__msg__Action__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__ACTION__STRUCT_H_
