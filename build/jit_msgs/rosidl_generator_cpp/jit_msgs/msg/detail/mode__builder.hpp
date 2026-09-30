// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jit_msgs:msg/Mode.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__MODE__BUILDER_HPP_
#define JIT_MSGS__MSG__DETAIL__MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jit_msgs/msg/detail/mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jit_msgs
{

namespace msg
{

namespace builder
{

class Init_Mode_mode
{
public:
  explicit Init_Mode_mode(::jit_msgs::msg::Mode & msg)
  : msg_(msg)
  {}
  ::jit_msgs::msg::Mode mode(::jit_msgs::msg::Mode::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jit_msgs::msg::Mode msg_;
};

class Init_Mode_stamp
{
public:
  Init_Mode_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Mode_mode stamp(::jit_msgs::msg::Mode::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Mode_mode(msg_);
  }

private:
  ::jit_msgs::msg::Mode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jit_msgs::msg::Mode>()
{
  return jit_msgs::msg::builder::Init_Mode_stamp();
}

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__MODE__BUILDER_HPP_
