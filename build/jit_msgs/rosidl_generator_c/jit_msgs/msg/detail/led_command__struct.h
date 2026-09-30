// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jit_msgs:msg/LedCommand.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__LED_COMMAND__STRUCT_H_
#define JIT_MSGS__MSG__DETAIL__LED_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ESTOP'.
/**
  * red    - motor rail is dead (relay open)
 */
static const char * const jit_msgs__msg__LedCommand__ESTOP = "ESTOP";

/// Constant 'MANUAL'.
/**
  * yellow - rail live, manual control
 */
static const char * const jit_msgs__msg__LedCommand__MANUAL = "MANUAL";

/// Constant 'AUTO'.
/**
  * green  - rail live, a guided mode is active
 */
static const char * const jit_msgs__msg__LedCommand__AUTO = "AUTO";

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'pattern'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LedCommand in the package jit_msgs.
/**
  * Indicator-panel command, published by system_monitor_node on led/command.
  *
  * The panel's drive mechanism is not yet known, so this carries a pattern name
  * rather than anything hardware-specific. led_driver_node is a stub that logs
  * the string; expanding the message is expected once the panel is specified.
 */
typedef struct jit_msgs__msg__LedCommand
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String pattern;
} jit_msgs__msg__LedCommand;

// Struct for a sequence of jit_msgs__msg__LedCommand.
typedef struct jit_msgs__msg__LedCommand__Sequence
{
  jit_msgs__msg__LedCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jit_msgs__msg__LedCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JIT_MSGS__MSG__DETAIL__LED_COMMAND__STRUCT_H_
