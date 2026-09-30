// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jit_msgs:msg/Health.idl
// generated code does not contain a copyright notice
#include "jit_msgs/msg/detail/health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `detail`
#include "rosidl_runtime_c/string_functions.h"

bool
jit_msgs__msg__Health__init(jit_msgs__msg__Health * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    jit_msgs__msg__Health__fini(msg);
    return false;
  }
  // ok
  // faults
  // detail
  if (!rosidl_runtime_c__String__init(&msg->detail)) {
    jit_msgs__msg__Health__fini(msg);
    return false;
  }
  return true;
}

void
jit_msgs__msg__Health__fini(jit_msgs__msg__Health * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // ok
  // faults
  // detail
  rosidl_runtime_c__String__fini(&msg->detail);
}

bool
jit_msgs__msg__Health__are_equal(const jit_msgs__msg__Health * lhs, const jit_msgs__msg__Health * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // ok
  if (lhs->ok != rhs->ok) {
    return false;
  }
  // faults
  if (lhs->faults != rhs->faults) {
    return false;
  }
  // detail
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->detail), &(rhs->detail)))
  {
    return false;
  }
  return true;
}

bool
jit_msgs__msg__Health__copy(
  const jit_msgs__msg__Health * input,
  jit_msgs__msg__Health * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // ok
  output->ok = input->ok;
  // faults
  output->faults = input->faults;
  // detail
  if (!rosidl_runtime_c__String__copy(
      &(input->detail), &(output->detail)))
  {
    return false;
  }
  return true;
}

jit_msgs__msg__Health *
jit_msgs__msg__Health__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jit_msgs__msg__Health * msg = (jit_msgs__msg__Health *)allocator.allocate(sizeof(jit_msgs__msg__Health), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jit_msgs__msg__Health));
  bool success = jit_msgs__msg__Health__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jit_msgs__msg__Health__destroy(jit_msgs__msg__Health * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jit_msgs__msg__Health__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jit_msgs__msg__Health__Sequence__init(jit_msgs__msg__Health__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jit_msgs__msg__Health * data = NULL;

  if (size) {
    data = (jit_msgs__msg__Health *)allocator.zero_allocate(size, sizeof(jit_msgs__msg__Health), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jit_msgs__msg__Health__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jit_msgs__msg__Health__fini(&data[i - 1]);
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
jit_msgs__msg__Health__Sequence__fini(jit_msgs__msg__Health__Sequence * array)
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
      jit_msgs__msg__Health__fini(&array->data[i]);
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

jit_msgs__msg__Health__Sequence *
jit_msgs__msg__Health__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jit_msgs__msg__Health__Sequence * array = (jit_msgs__msg__Health__Sequence *)allocator.allocate(sizeof(jit_msgs__msg__Health__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jit_msgs__msg__Health__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jit_msgs__msg__Health__Sequence__destroy(jit_msgs__msg__Health__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jit_msgs__msg__Health__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jit_msgs__msg__Health__Sequence__are_equal(const jit_msgs__msg__Health__Sequence * lhs, const jit_msgs__msg__Health__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jit_msgs__msg__Health__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jit_msgs__msg__Health__Sequence__copy(
  const jit_msgs__msg__Health__Sequence * input,
  jit_msgs__msg__Health__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jit_msgs__msg__Health);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jit_msgs__msg__Health * data =
      (jit_msgs__msg__Health *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jit_msgs__msg__Health__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jit_msgs__msg__Health__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jit_msgs__msg__Health__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
