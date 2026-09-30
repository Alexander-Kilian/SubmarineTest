// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jit_msgs:msg/Mode.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jit_msgs/msg/detail/mode__rosidl_typesupport_introspection_c.h"
#include "jit_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jit_msgs/msg/detail/mode__functions.h"
#include "jit_msgs/msg/detail/mode__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jit_msgs__msg__Mode__init(message_memory);
}

void jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_fini_function(void * message_memory)
{
  jit_msgs__msg__Mode__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_member_array[2] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jit_msgs__msg__Mode, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jit_msgs__msg__Mode, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_members = {
  "jit_msgs__msg",  // message namespace
  "Mode",  // message name
  2,  // number of fields
  sizeof(jit_msgs__msg__Mode),
  jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_member_array,  // message members
  jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_init_function,  // function to initialize message memory (memory has to be allocated)
  jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_type_support_handle = {
  0,
  &jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jit_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jit_msgs, msg, Mode)() {
  jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_type_support_handle.typesupport_identifier) {
    jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jit_msgs__msg__Mode__rosidl_typesupport_introspection_c__Mode_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
