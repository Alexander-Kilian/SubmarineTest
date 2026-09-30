// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jit_msgs:msg/LedCommand.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__LED_COMMAND__TRAITS_HPP_
#define JIT_MSGS__MSG__DETAIL__LED_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jit_msgs/msg/detail/led_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace jit_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LedCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: pattern
  {
    out << "pattern: ";
    rosidl_generator_traits::value_to_yaml(msg.pattern, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LedCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: pattern
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pattern: ";
    rosidl_generator_traits::value_to_yaml(msg.pattern, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LedCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace jit_msgs

namespace rosidl_generator_traits
{

[[deprecated("use jit_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jit_msgs::msg::LedCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  jit_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jit_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jit_msgs::msg::LedCommand & msg)
{
  return jit_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jit_msgs::msg::LedCommand>()
{
  return "jit_msgs::msg::LedCommand";
}

template<>
inline const char * name<jit_msgs::msg::LedCommand>()
{
  return "jit_msgs/msg/LedCommand";
}

template<>
struct has_fixed_size<jit_msgs::msg::LedCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jit_msgs::msg::LedCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jit_msgs::msg::LedCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JIT_MSGS__MSG__DETAIL__LED_COMMAND__TRAITS_HPP_
