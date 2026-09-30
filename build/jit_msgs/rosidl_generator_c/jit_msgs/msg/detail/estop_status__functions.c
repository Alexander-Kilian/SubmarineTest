// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jit_msgs:msg/EstopStatus.idl
// generated code does not contain a copyright notice
#include "jit_msgs/msg/detail/estop_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
jit_msgs__msg__EstopStatus__init(jit_msgs__msg__EstopStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    jit_msgs__msg__EstopStatus__fini(msg);
    return false;
  }
  // commanded
  // feedback
  // mismatch
  // settled
  return true;
}

void
jit_msgs__msg__EstopStatus__fini(jit_msgs__msg__EstopStatus * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // commanded
  // feedback
  // mismatch
  // settled
}

bool
jit_msgs__msg__EstopStatus__are_equal(const jit_msgs__msg__EstopStatus * lhs, const jit_msgs__msg__EstopStatus * rhs)
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
  // commanded
  if (lhs->commanded != rhs->commanded) {
    return false;
  }
  // feedback
  if (lhs->feedback != rhs->feedback) {
    return false;
  }
  // mismatch
  if (lhs->mismatch != rhs->mismatch) {
    return false;
  }
  // settled
  if (lhs->settled != rhs->settled) {
    return false;
  }
  return true;
}

bool
jit_msgs__msg__EstopStatus__copy(
  const jit_msgs__msg__EstopStatus * input,
  jit_msgs__msg__EstopStatus * output)
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
  // commanded
  output->commanded = input->commanded;
  // feedback
  output->feedback = input->feedback;
  // mismatch
  output->mismatch = input->mismatch;
  // settled
  output->settled = input->settled;
  return true;
}

jit_msgs__msg__EstopStatus *
jit_msgs__msg__EstopStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jit_msgs__msg__EstopStatus * msg = (jit_msgs__msg__EstopStatus *)allocator.allocate(sizeof(jit_msgs__msg__EstopStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jit_msgs__msg__EstopStatus));
  bool success = jit_msgs__msg__EstopStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jit_msgs__msg__EstopStatus__destroy(jit_msgs__msg__EstopStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jit_msgs__msg__EstopStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jit_msgs__msg__EstopStatus__Sequence__init(jit_msgs__msg__EstopStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jit_msgs__msg__EstopStatus * data = NULL;

  if (size) {
    data = (jit_msgs__msg__EstopStatus *)allocator.zero_allocate(size, sizeof(jit_msgs__msg__EstopStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jit_msgs__msg__EstopStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jit_msgs__msg__EstopStatus__fini(&data[i - 1]);
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
jit_msgs__msg__EstopStatus__Sequence__fini(jit_msgs__msg__EstopStatus__Sequence * array)
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
      jit_msgs__msg__EstopStatus__fini(&array->data[i]);
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

jit_msgs__msg__EstopStatus__Sequence *
jit_msgs__msg__EstopStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jit_msgs__msg__EstopStatus__Sequence * array = (jit_msgs__msg__EstopStatus__Sequence *)allocator.allocate(sizeof(jit_msgs__msg__EstopStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jit_msgs__msg__EstopStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jit_msgs__msg__EstopStatus__Sequence__destroy(jit_msgs__msg__EstopStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jit_msgs__msg__EstopStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jit_msgs__msg__EstopStatus__Sequence__are_equal(const jit_msgs__msg__EstopStatus__Sequence * lhs, const jit_msgs__msg__EstopStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jit_msgs__msg__EstopStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jit_msgs__msg__EstopStatus__Sequence__copy(
  const jit_msgs__msg__EstopStatus__Sequence * input,
  jit_msgs__msg__EstopStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jit_msgs__msg__EstopStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jit_msgs__msg__EstopStatus * data =
      (jit_msgs__msg__EstopStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jit_msgs__msg__EstopStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jit_msgs__msg__EstopStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jit_msgs__msg__EstopStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
