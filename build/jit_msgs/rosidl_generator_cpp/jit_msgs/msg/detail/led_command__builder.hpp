// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jit_msgs:msg/LedCommand.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__LED_COMMAND__BUILDER_HPP_
#define JIT_MSGS__MSG__DETAIL__LED_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jit_msgs/msg/detail/led_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jit_msgs
{

namespace msg
{

namespace builder
{

class Init_LedCommand_pattern
{
public:
  explicit Init_LedCommand_pattern(::jit_msgs::msg::LedCommand & msg)
  : msg_(msg)
  {}
  ::jit_msgs::msg::LedCommand pattern(::jit_msgs::msg::LedCommand::_pattern_type arg)
  {
    msg_.pattern = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jit_msgs::msg::LedCommand msg_;
};

class Init_LedCommand_stamp
{
public:
  Init_LedCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LedCommand_pattern stamp(::jit_msgs::msg::LedCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_LedCommand_pattern(msg_);
  }

private:
  ::jit_msgs::msg::LedCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jit_msgs::msg::LedCommand>()
{
  return jit_msgs::msg::builder::Init_LedCommand_stamp();
}

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__LED_COMMAND__BUILDER_HPP_
