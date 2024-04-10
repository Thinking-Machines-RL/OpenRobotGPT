// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robotgpt_interfaces:msg/Action.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robotgpt_interfaces/msg/detail/action__rosidl_typesupport_introspection_c.h"
#include "robotgpt_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robotgpt_interfaces/msg/detail/action__functions.h"
#include "robotgpt_interfaces/msg/detail/action__struct.h"


// Include directives for member types
// Member `action`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Action__rosidl_typesupport_introspection_c__Action_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robotgpt_interfaces__msg__Action__init(message_memory);
}

void Action__rosidl_typesupport_introspection_c__Action_fini_function(void * message_memory)
{
  robotgpt_interfaces__msg__Action__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Action__rosidl_typesupport_introspection_c__Action_message_member_array[1] = {
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robotgpt_interfaces__msg__Action, action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Action__rosidl_typesupport_introspection_c__Action_message_members = {
  "robotgpt_interfaces__msg",  // message namespace
  "Action",  // message name
  1,  // number of fields
  sizeof(robotgpt_interfaces__msg__Action),
  Action__rosidl_typesupport_introspection_c__Action_message_member_array,  // message members
  Action__rosidl_typesupport_introspection_c__Action_init_function,  // function to initialize message memory (memory has to be allocated)
  Action__rosidl_typesupport_introspection_c__Action_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Action__rosidl_typesupport_introspection_c__Action_message_type_support_handle = {
  0,
  &Action__rosidl_typesupport_introspection_c__Action_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robotgpt_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robotgpt_interfaces, msg, Action)() {
  if (!Action__rosidl_typesupport_introspection_c__Action_message_type_support_handle.typesupport_identifier) {
    Action__rosidl_typesupport_introspection_c__Action_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Action__rosidl_typesupport_introspection_c__Action_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
