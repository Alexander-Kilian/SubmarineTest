// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jit_msgs:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__HEALTH__STRUCT_H_
#define JIT_MSGS__MSG__DETAIL__HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'RC_LINK_LOST'.
/**
  * ---------------------------------------------------------------------------
  * Fault bits
  * ---------------------------------------------------------------------------
  * crsf/link_ok is false
 */
enum
{
  jit_msgs__msg__Health__RC_LINK_LOST = 1ul
};

/// Constant 'CRSF_NODE_DEAD'.
/**
  * crsf/link_ok has gone stale (that node publishes
  * every poll cycle, so silence means it died)
 */
enum
{
  jit_msgs__msg__Health__CRSF_NODE_DEAD = 2ul
};

/// Constant 'MAVROS_LOST'.
/**
  * /mavros/state.connected false, or stale
 */
enum
{
  jit_msgs__msg__Health__MAVROS_LOST = 4ul
};

/// Constant 'ESTOP_NODE_DEAD'.
/**
  * estop/status has gone stale
 */
enum
{
  jit_msgs__msg__Health__ESTOP_NODE_DEAD = 8ul
};

/// Constant 'ESTOP_OPEN'.
/**
  * relay feedback says the motor rail is dead
 */
enum
{
  jit_msgs__msg__Health__ESTOP_OPEN = 16ul
};

/// Constant 'RELAY_NOT_CLOSED'.
/**
  * commanded closed, feedback says open
 */
enum
{
  jit_msgs__msg__Health__RELAY_NOT_CLOSED = 32ul
};

/// Constant 'RELAY_WELDED'.
/**
  * commanded open, feedback says CLOSED - the
  * hardware e-stop is not working
 */
enum
{
  jit_msgs__msg__Health__RELAY_WELDED = 64ul
};

/// Constant 'MISSION_COMPLETE'.
/**
  * guided mission finished and held long enough
 */
enum
{
  jit_msgs__msg__Health__MISSION_COMPLETE = 128ul
};

/// Constant 'FAULT_SAFE_MASK'.
/**
  * ---------------------------------------------------------------------------
  * FAULT_SAFE_MASK - the faults that open the motor relay
  * ---------------------------------------------------------------------------
  * system_monitor_node publishes sys/health_ok = ((faults & FAULT_SAFE_MASK) == 0),
  * and gpio_estop_node requires that to be true before it will close the relay.
  *
  * Membership of this mask is NOT a severity judgement. Two rules decide it:
  *
  *  1. A fault derived from the relay's own feedback must NEVER be in the mask.
  *     Otherwise: relay open -> feedback low -> ESTOP_OPEN -> sys/health_ok false
  *     -> relay commanded open -> feedback stays low. That is a deadlock the
  *     system cannot leave, and since the vehicle boots with the relay open it
  *     would never become drivable at all. ESTOP_OPEN, RELAY_NOT_CLOSED and
  *     RELAY_WELDED are therefore excluded.
  *
  *  2. MISSION_COMPLETE is excluded because a successful mission must not cycle
  *     the hardware e-stop. It is a "mission-safe": disarm only, relay untouched.
  *
  * Every excluded fault still sets `ok` false and still forces
  * vehicle_interface_node to disarm. They just do not gate the relay.
  * RC_LINK_LOST | CRSF_NODE_DEAD | MAVROS_LOST | ESTOP_NODE_DEAD
 */
enum
{
  jit_msgs__msg__Health__FAULT_SAFE_MASK = 15ul
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'detail'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Health in the package jit_msgs.
/**
  * The single ground-truth health verdict for the vehicle, published by
  * system_monitor_node at a fixed rate on jit/health.
  *
  * Consumers must apply their own staleness check against `stamp` in addition to
  * reading `ok` - this message being old is itself a fault condition, and the bus
  * is not a substitute for a freshness timeout.
 */
typedef struct jit_msgs__msg__Health
{
  builtin_interfaces__msg__Time stamp;
  /// True when `faults` is zero. Nothing may arm or command motion when false.
  bool ok;
  /// Bitfield of every currently-asserted fault. Rebuilt from scratch on every
  /// evaluation tick, so a fault clears the moment its cause clears.
  uint32_t faults;
  /// Human-readable summary of the asserted faults, for logs and diagnosis.
  rosidl_runtime_c__String detail;
} jit_msgs__msg__Health;

// Struct for a sequence of jit_msgs__msg__Health.
typedef struct jit_msgs__msg__Health__Sequence
{
  jit_msgs__msg__Health * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jit_msgs__msg__Health__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JIT_MSGS__MSG__DETAIL__HEALTH__STRUCT_H_
