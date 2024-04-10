// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robotgpt_interfaces:msg/StateReward.idl
// generated code does not contain a copyright notice

#ifndef ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__STRUCT_H_
#define ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'state'
// Member 'info'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'info_keys'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/StateReward in the package robotgpt_interfaces.
typedef struct robotgpt_interfaces__msg__StateReward
{
  rosidl_runtime_c__float__Sequence state;
  rosidl_runtime_c__String__Sequence info_keys;
  rosidl_runtime_c__float__Sequence info;
  float reward;
  bool terminal;
} robotgpt_interfaces__msg__StateReward;

// Struct for a sequence of robotgpt_interfaces__msg__StateReward.
typedef struct robotgpt_interfaces__msg__StateReward__Sequence
{
  robotgpt_interfaces__msg__StateReward * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robotgpt_interfaces__msg__StateReward__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOTGPT_INTERFACES__MSG__DETAIL__STATE_REWARD__STRUCT_H_
