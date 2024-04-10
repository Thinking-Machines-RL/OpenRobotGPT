// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:srv/CodeExecution.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__SRV__DETAIL__CODE_EXECUTION__STRUCT_H_
#define MESSAGES__SRV__DETAIL__CODE_EXECUTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'code'
// Member 'evaluation_code'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/CodeExecution in the package messages.
typedef struct messages__srv__CodeExecution_Request
{
  rosidl_runtime_c__String code;
  rosidl_runtime_c__String evaluation_code;
} messages__srv__CodeExecution_Request;

// Struct for a sequence of messages__srv__CodeExecution_Request.
typedef struct messages__srv__CodeExecution_Request__Sequence
{
  messages__srv__CodeExecution_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__srv__CodeExecution_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/CodeExecution in the package messages.
typedef struct messages__srv__CodeExecution_Response
{
  bool completion_flag;
} messages__srv__CodeExecution_Response;

// Struct for a sequence of messages__srv__CodeExecution_Response.
typedef struct messages__srv__CodeExecution_Response__Sequence
{
  messages__srv__CodeExecution_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__srv__CodeExecution_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__SRV__DETAIL__CODE_EXECUTION__STRUCT_H_
