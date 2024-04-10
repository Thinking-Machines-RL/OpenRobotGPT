// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robotgpt_interfaces:msg/StateReward.idl
// generated code does not contain a copyright notice
#include "robotgpt_interfaces/msg/detail/state_reward__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `state`
// Member `info`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `info_keys`
#include "rosidl_runtime_c/string_functions.h"

bool
robotgpt_interfaces__msg__StateReward__init(robotgpt_interfaces__msg__StateReward * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__float__Sequence__init(&msg->state, 0)) {
    robotgpt_interfaces__msg__StateReward__fini(msg);
    return false;
  }
  // info_keys
  if (!rosidl_runtime_c__String__Sequence__init(&msg->info_keys, 0)) {
    robotgpt_interfaces__msg__StateReward__fini(msg);
    return false;
  }
  // info
  if (!rosidl_runtime_c__float__Sequence__init(&msg->info, 0)) {
    robotgpt_interfaces__msg__StateReward__fini(msg);
    return false;
  }
  // reward
  // terminal
  return true;
}

void
robotgpt_interfaces__msg__StateReward__fini(robotgpt_interfaces__msg__StateReward * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__float__Sequence__fini(&msg->state);
  // info_keys
  rosidl_runtime_c__String__Sequence__fini(&msg->info_keys);
  // info
  rosidl_runtime_c__float__Sequence__fini(&msg->info);
  // reward
  // terminal
}

bool
robotgpt_interfaces__msg__StateReward__are_equal(const robotgpt_interfaces__msg__StateReward * lhs, const robotgpt_interfaces__msg__StateReward * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // info_keys
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->info_keys), &(rhs->info_keys)))
  {
    return false;
  }
  // info
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // reward
  if (lhs->reward != rhs->reward) {
    return false;
  }
  // terminal
  if (lhs->terminal != rhs->terminal) {
    return false;
  }
  return true;
}

bool
robotgpt_interfaces__msg__StateReward__copy(
  const robotgpt_interfaces__msg__StateReward * input,
  robotgpt_interfaces__msg__StateReward * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // info_keys
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->info_keys), &(output->info_keys)))
  {
    return false;
  }
  // info
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // reward
  output->reward = input->reward;
  // terminal
  output->terminal = input->terminal;
  return true;
}

robotgpt_interfaces__msg__StateReward *
robotgpt_interfaces__msg__StateReward__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotgpt_interfaces__msg__StateReward * msg = (robotgpt_interfaces__msg__StateReward *)allocator.allocate(sizeof(robotgpt_interfaces__msg__StateReward), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robotgpt_interfaces__msg__StateReward));
  bool success = robotgpt_interfaces__msg__StateReward__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robotgpt_interfaces__msg__StateReward__destroy(robotgpt_interfaces__msg__StateReward * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robotgpt_interfaces__msg__StateReward__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robotgpt_interfaces__msg__StateReward__Sequence__init(robotgpt_interfaces__msg__StateReward__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotgpt_interfaces__msg__StateReward * data = NULL;

  if (size) {
    data = (robotgpt_interfaces__msg__StateReward *)allocator.zero_allocate(size, sizeof(robotgpt_interfaces__msg__StateReward), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robotgpt_interfaces__msg__StateReward__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robotgpt_interfaces__msg__StateReward__fini(&data[i - 1]);
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
robotgpt_interfaces__msg__StateReward__Sequence__fini(robotgpt_interfaces__msg__StateReward__Sequence * array)
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
      robotgpt_interfaces__msg__StateReward__fini(&array->data[i]);
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

robotgpt_interfaces__msg__StateReward__Sequence *
robotgpt_interfaces__msg__StateReward__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robotgpt_interfaces__msg__StateReward__Sequence * array = (robotgpt_interfaces__msg__StateReward__Sequence *)allocator.allocate(sizeof(robotgpt_interfaces__msg__StateReward__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robotgpt_interfaces__msg__StateReward__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robotgpt_interfaces__msg__StateReward__Sequence__destroy(robotgpt_interfaces__msg__StateReward__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robotgpt_interfaces__msg__StateReward__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robotgpt_interfaces__msg__StateReward__Sequence__are_equal(const robotgpt_interfaces__msg__StateReward__Sequence * lhs, const robotgpt_interfaces__msg__StateReward__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robotgpt_interfaces__msg__StateReward__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robotgpt_interfaces__msg__StateReward__Sequence__copy(
  const robotgpt_interfaces__msg__StateReward__Sequence * input,
  robotgpt_interfaces__msg__StateReward__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robotgpt_interfaces__msg__StateReward);
    robotgpt_interfaces__msg__StateReward * data =
      (robotgpt_interfaces__msg__StateReward *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robotgpt_interfaces__msg__StateReward__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          robotgpt_interfaces__msg__StateReward__fini(&data[i]);
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
    if (!robotgpt_interfaces__msg__StateReward__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
