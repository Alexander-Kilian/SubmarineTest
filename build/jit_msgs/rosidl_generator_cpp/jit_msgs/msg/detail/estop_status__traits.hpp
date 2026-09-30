// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jit_msgs:msg/EstopStatus.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__TRAITS_HPP_
#define JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jit_msgs/msg/detail/estop_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace jit_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EstopStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: commanded
  {
    out << "commanded: ";
    rosidl_generator_traits::value_to_yaml(msg.commanded, out);
    out << ", ";
  }

  // member: feedback
  {
    out << "feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback, out);
    out << ", ";
  }

  // member: mismatch
  {
    out << "mismatch: ";
    rosidl_generator_traits::value_to_yaml(msg.mismatch, out);
    out << ", ";
  }

  // member: settled
  {
    out << "settled: ";
    rosidl_generator_traits::value_to_yaml(msg.settled, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EstopStatus & msg,
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

  // member: commanded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "commanded: ";
    rosidl_generator_traits::value_to_yaml(msg.commanded, out);
    out << "\n";
  }

  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback, out);
    out << "\n";
  }

  // member: mismatch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mismatch: ";
    rosidl_generator_traits::value_to_yaml(msg.mismatch, out);
    out << "\n";
  }

  // member: settled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "settled: ";
    rosidl_generator_traits::value_to_yaml(msg.settled, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EstopStatus & msg, bool use_flow_style = false)
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
  const jit_msgs::msg::EstopStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  jit_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jit_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const jit_msgs::msg::EstopStatus & msg)
{
  return jit_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jit_msgs::msg::EstopStatus>()
{
  return "jit_msgs::msg::EstopStatus";
}

template<>
inline const char * name<jit_msgs::msg::EstopStatus>()
{
  return "jit_msgs/msg/EstopStatus";
}

template<>
struct has_fixed_size<jit_msgs::msg::EstopStatus>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<jit_msgs::msg::EstopStatus>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<jit_msgs::msg::EstopStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__TRAITS_HPP_
