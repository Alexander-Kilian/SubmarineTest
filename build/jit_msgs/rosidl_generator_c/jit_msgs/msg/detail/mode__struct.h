// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jit_msgs:msg/Mode.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__MODE__STRUCT_H_
#define JIT_MSGS__MSG__DETAIL__MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SAFE'.
/**
  * nothing may arm or command motion
 */
enum
{
  jit_msgs__msg__Mode__SAFE = 0
};

/// Constant 'MANUAL'.
/**
  * sticks -> MANUAL_CONTROL
 */
enum
{
  jit_msgs__msg__Mode__MANUAL = 1
};

/// Constant 'LOCAL_GUIDED'.
/**
  * waypoints against /mavros/local_position/pose
 */
enum
{
  jit_msgs__msg__Mode__LOCAL_GUIDED = 2
};

/// Constant 'GLOBAL_GUIDED'.
/**
  * waypoints against a GPS fix (not implemented)
 */
enum
{
  jit_msgs__msg__Mode__GLOBAL_GUIDED = 3
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/Mode in the package jit_msgs.
/**
  * The granted operating mode, published by system_monitor_node on jit/mode.
  *
  * This is what the system has GRANTED, not what was requested. The switch (or,
  * later, the autonomy layer) requests a mode via ModeRequest; the monitor grants
  * one after applying the health gate.
 */
typedef struct jit_msgs__msg__Mode
{
  builtin_interfaces__msg__Time stamp;
  uint8_t mode;
} jit_msgs__msg__Mode;

// Struct for a sequence of jit_msgs__msg__Mode.
typedef struct jit_msgs__msg__Mode__Sequence
{
  jit_msgs__msg__Mode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jit_msgs__msg__Mode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JIT_MSGS__MSG__DETAIL__MODE__STRUCT_H_
