// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jit_msgs:msg/EstopStatus.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__STRUCT_H_
#define JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/EstopStatus in the package jit_msgs.
/**
  * Relay state as reported by gpio_estop_node on estop/status.
  *
  * Two independent facts, because they can disagree and the disagreement is the
  * most safety-relevant signal in the system:
  *
  *   commanded - what we drove the output pin to
  *   feedback  - what the LDO on the switched battery rail says is actually
  *               happening (HIGH = relay closed = rail live)
  *
  * Published with transient-local ("latched") QoS plus a heartbeat, so a late
  * subscriber gets the state immediately and a live one can apply a staleness
  * timeout.
 */
typedef struct jit_msgs__msg__EstopStatus
{
  builtin_interfaces__msg__Time stamp;
  /// true = we asked for the rail to be live
  bool commanded;
  /// true = the rail actually is live
  bool feedback;
  /// true = settled and disagreeing (see relay_settle_ms)
  bool mismatch;
  /// Set once the boot-time self-test has run. Before that, `mismatch` is not
  /// meaningful because the relay has not had time to settle.
  bool settled;
} jit_msgs__msg__EstopStatus;

// Struct for a sequence of jit_msgs__msg__EstopStatus.
typedef struct jit_msgs__msg__EstopStatus__Sequence
{
  jit_msgs__msg__EstopStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jit_msgs__msg__EstopStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__STRUCT_H_
