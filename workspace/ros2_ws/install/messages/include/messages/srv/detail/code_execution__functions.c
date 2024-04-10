// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:srv/CodeExecution.idl
// generated code does not contain a copyright notice
#include "messages/srv/detail/code_execution__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `code`
// Member `evaluation_code`
#include "rosidl_runtime_c/string_functions.h"

bool
messages__srv__CodeExecution_Request__init(messages__srv__CodeExecution_Request * msg)
{
  if (!msg) {
    return false;
  }
  // code
  if (!rosidl_runtime_c__String__init(&msg->code)) {
    messages__srv__CodeExecution_Request__fini(msg);
    return false;
  }
  // evaluation_code
  if (!rosidl_runtime_c__String__init(&msg->evaluation_code)) {
    messages__srv__CodeExecution_Request__fini(msg);
    return false;
  }
  return true;
}

void
messages__srv__CodeExecution_Request__fini(messages__srv__CodeExecution_Request * msg)
{
  if (!msg) {
    return;
  }
  // code
  rosidl_runtime_c__String__fini(&msg->code);
  // evaluation_code
  rosidl_runtime_c__String__fini(&msg->evaluation_code);
}

bool
messages__srv__CodeExecution_Request__are_equal(const messages__srv__CodeExecution_Request * lhs, const messages__srv__CodeExecution_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // code
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->code), &(rhs->code)))
  {
    return false;
  }
  // evaluation_code
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->evaluation_code), &(rhs->evaluation_code)))
  {
    return false;
  }
  return true;
}

bool
messages__srv__CodeExecution_Request__copy(
  const messages__srv__CodeExecution_Request * input,
  messages__srv__CodeExecution_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // code
  if (!rosidl_runtime_c__String__copy(
      &(input->code), &(output->code)))
  {
    return false;
  }
  // evaluation_code
  if (!rosidl_runtime_c__String__copy(
      &(input->evaluation_code), &(output->evaluation_code)))
  {
    return false;
  }
  return true;
}

messages__srv__CodeExecution_Request *
messages__srv__CodeExecution_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__srv__CodeExecution_Request * msg = (messages__srv__CodeExecution_Request *)allocator.allocate(sizeof(messages__srv__CodeExecution_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__srv__CodeExecution_Request));
  bool success = messages__srv__CodeExecution_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
messages__srv__CodeExecution_Request__destroy(messages__srv__CodeExecution_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    messages__srv__CodeExecution_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
messages__srv__CodeExecution_Request__Sequence__init(messages__srv__CodeExecution_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__srv__CodeExecution_Request * data = NULL;

  if (size) {
    data = (messages__srv__CodeExecution_Request *)allocator.zero_allocate(size, sizeof(messages__srv__CodeExecution_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__srv__CodeExecution_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__srv__CodeExecution_Request__fini(&data[i - 1]);
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
messages__srv__CodeExecution_Request__Sequence__fini(messages__srv__CodeExecution_Request__Sequence * array)
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
      messages__srv__CodeExecution_Request__fini(&array->data[i]);
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

messages__srv__CodeExecution_Request__Sequence *
messages__srv__CodeExecution_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__srv__CodeExecution_Request__Sequence * array = (messages__srv__CodeExecution_Request__Sequence *)allocator.allocate(sizeof(messages__srv__CodeExecution_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = messages__srv__CodeExecution_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
messages__srv__CodeExecution_Request__Sequence__destroy(messages__srv__CodeExecution_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    messages__srv__CodeExecution_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
messages__srv__CodeExecution_Request__Sequence__are_equal(const messages__srv__CodeExecution_Request__Sequence * lhs, const messages__srv__CodeExecution_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__srv__CodeExecution_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__srv__CodeExecution_Request__Sequence__copy(
  const messages__srv__CodeExecution_Request__Sequence * input,
  messages__srv__CodeExecution_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__srv__CodeExecution_Request);
    messages__srv__CodeExecution_Request * data =
      (messages__srv__CodeExecution_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__srv__CodeExecution_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__srv__CodeExecution_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!messages__srv__CodeExecution_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
messages__srv__CodeExecution_Response__init(messages__srv__CodeExecution_Response * msg)
{
  if (!msg) {
    return false;
  }
  // completion_flag
  return true;
}

void
messages__srv__CodeExecution_Response__fini(messages__srv__CodeExecution_Response * msg)
{
  if (!msg) {
    return;
  }
  // completion_flag
}

bool
messages__srv__CodeExecution_Response__are_equal(const messages__srv__CodeExecution_Response * lhs, const messages__srv__CodeExecution_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // completion_flag
  if (lhs->completion_flag != rhs->completion_flag) {
    return false;
  }
  return true;
}

bool
messages__srv__CodeExecution_Response__copy(
  const messages__srv__CodeExecution_Response * input,
  messages__srv__CodeExecution_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // completion_flag
  output->completion_flag = input->completion_flag;
  return true;
}

messages__srv__CodeExecution_Response *
messages__srv__CodeExecution_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__srv__CodeExecution_Response * msg = (messages__srv__CodeExecution_Response *)allocator.allocate(sizeof(messages__srv__CodeExecution_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__srv__CodeExecution_Response));
  bool success = messages__srv__CodeExecution_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
messages__srv__CodeExecution_Response__destroy(messages__srv__CodeExecution_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    messages__srv__CodeExecution_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
messages__srv__CodeExecution_Response__Sequence__init(messages__srv__CodeExecution_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__srv__CodeExecution_Response * data = NULL;

  if (size) {
    data = (messages__srv__CodeExecution_Response *)allocator.zero_allocate(size, sizeof(messages__srv__CodeExecution_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__srv__CodeExecution_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__srv__CodeExecution_Response__fini(&data[i - 1]);
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
messages__srv__CodeExecution_Response__Sequence__fini(messages__srv__CodeExecution_Response__Sequence * array)
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
      messages__srv__CodeExecution_Response__fini(&array->data[i]);
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

messages__srv__CodeExecution_Response__Sequence *
messages__srv__CodeExecution_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  messages__srv__CodeExecution_Response__Sequence * array = (messages__srv__CodeExecution_Response__Sequence *)allocator.allocate(sizeof(messages__srv__CodeExecution_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = messages__srv__CodeExecution_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
messages__srv__CodeExecution_Response__Sequence__destroy(messages__srv__CodeExecution_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    messages__srv__CodeExecution_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
messages__srv__CodeExecution_Response__Sequence__are_equal(const messages__srv__CodeExecution_Response__Sequence * lhs, const messages__srv__CodeExecution_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!messages__srv__CodeExecution_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
messages__srv__CodeExecution_Response__Sequence__copy(
  const messages__srv__CodeExecution_Response__Sequence * input,
  messages__srv__CodeExecution_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(messages__srv__CodeExecution_Response);
    messages__srv__CodeExecution_Response * data =
      (messages__srv__CodeExecution_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!messages__srv__CodeExecution_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          messages__srv__CodeExecution_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!messages__srv__CodeExecution_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
