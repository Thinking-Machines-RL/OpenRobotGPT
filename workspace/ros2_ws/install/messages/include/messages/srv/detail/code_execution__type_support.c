// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from messages:srv/CodeExecution.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "messages/srv/detail/code_execution__rosidl_typesupport_introspection_c.h"
#include "messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "messages/srv/detail/code_execution__functions.h"
#include "messages/srv/detail/code_execution__struct.h"


// Include directives for member types
// Member `code`
// Member `evaluation_code`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  messages__srv__CodeExecution_Request__init(message_memory);
}

void CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_fini_function(void * message_memory)
{
  messages__srv__CodeExecution_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_member_array[2] = {
  {
    "code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__srv__CodeExecution_Request, code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "evaluation_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__srv__CodeExecution_Request, evaluation_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_members = {
  "messages__srv",  // message namespace
  "CodeExecution_Request",  // message name
  2,  // number of fields
  sizeof(messages__srv__CodeExecution_Request),
  CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_member_array,  // message members
  CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_type_support_handle = {
  0,
  &CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution_Request)() {
  if (!CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_type_support_handle.typesupport_identifier) {
    CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CodeExecution_Request__rosidl_typesupport_introspection_c__CodeExecution_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "messages/srv/detail/code_execution__rosidl_typesupport_introspection_c.h"
// already included above
// #include "messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "messages/srv/detail/code_execution__functions.h"
// already included above
// #include "messages/srv/detail/code_execution__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  messages__srv__CodeExecution_Response__init(message_memory);
}

void CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_fini_function(void * message_memory)
{
  messages__srv__CodeExecution_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_member_array[1] = {
  {
    "completion_flag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(messages__srv__CodeExecution_Response, completion_flag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_members = {
  "messages__srv",  // message namespace
  "CodeExecution_Response",  // message name
  1,  // number of fields
  sizeof(messages__srv__CodeExecution_Response),
  CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_member_array,  // message members
  CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_type_support_handle = {
  0,
  &CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution_Response)() {
  if (!CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_type_support_handle.typesupport_identifier) {
    CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CodeExecution_Response__rosidl_typesupport_introspection_c__CodeExecution_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "messages/srv/detail/code_execution__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_members = {
  "messages__srv",  // service namespace
  "CodeExecution",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_Request_message_type_support_handle,
  NULL  // response message
  // messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_Response_message_type_support_handle
};

static rosidl_service_type_support_t messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_type_support_handle = {
  0,
  &messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_messages
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution)() {
  if (!messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_type_support_handle.typesupport_identifier) {
    messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, messages, srv, CodeExecution_Response)()->data;
  }

  return &messages__srv__detail__code_execution__rosidl_typesupport_introspection_c__CodeExecution_service_type_support_handle;
}
