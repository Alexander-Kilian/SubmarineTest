// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jit_msgs:msg/ModeRequest.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__MODE_REQUEST__STRUCT_H_
#define JIT_MSGS__MSG__DETAIL__MODE_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SOURCE_RC'.
enum
{
  jit_msgs__msg__ModeRequest__SOURCE_RC = 0
};

/// Constant 'SOURCE_AUTONOMY'.
enum
{
  jit_msgs__msg__ModeRequest__SOURCE_AUTONOMY = 1
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/ModeRequest in the package jit_msgs.
/**
  * A request for an operating mode.
  *
  * NOTE-2: there is exactly ONE publisher of this message today
  * (crsf_channel_node, from the ch8 detent), so system_monitor_node grants
  * whatever it is asked for. Before an autonomy layer publishes here as well,
  * the monitor needs source priority (RC must outrank autonomy) and request
  * expiry (a source that stops publishing loses its claim). The `source` field
  * is reserved for that and is currently ignored.
 */
typedef struct jit_msgs__msg__ModeRequest
{
  builtin_interfaces__msg__Time stamp;
  /// values are Mode.msg's constants - Mode::SAFE, Mode::MANUAL,
  /// Mode::LOCAL_GUIDED, Mode::GLOBAL_GUIDED. They are deliberately
  /// NOT redeclared here: one definition, in Mode.msg.
  uint8_t mode;
  /// reserved - see NOTE-2
  uint8_t source;
} jit_msgs__msg__ModeRequest;

// Struct for a sequence of jit_msgs__msg__ModeRequest.
typedef struct jit_msgs__msg__ModeRequest__Sequence
{
  jit_msgs__msg__ModeRequest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jit_msgs__msg__ModeRequest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JIT_MSGS__MSG__DETAIL__MODE_REQUEST__STRUCT_H_
