// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jit_msgs:msg/ModeRequest.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__MODE_REQUEST__BUILDER_HPP_
#define JIT_MSGS__MSG__DETAIL__MODE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jit_msgs/msg/detail/mode_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jit_msgs
{

namespace msg
{

namespace builder
{

class Init_ModeRequest_source
{
public:
  explicit Init_ModeRequest_source(::jit_msgs::msg::ModeRequest & msg)
  : msg_(msg)
  {}
  ::jit_msgs::msg::ModeRequest source(::jit_msgs::msg::ModeRequest::_source_type arg)
  {
    msg_.source = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jit_msgs::msg::ModeRequest msg_;
};

class Init_ModeRequest_mode
{
public:
  explicit Init_ModeRequest_mode(::jit_msgs::msg::ModeRequest & msg)
  : msg_(msg)
  {}
  Init_ModeRequest_source mode(::jit_msgs::msg::ModeRequest::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_ModeRequest_source(msg_);
  }

private:
  ::jit_msgs::msg::ModeRequest msg_;
};

class Init_ModeRequest_stamp
{
public:
  Init_ModeRequest_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ModeRequest_mode stamp(::jit_msgs::msg::ModeRequest::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ModeRequest_mode(msg_);
  }

private:
  ::jit_msgs::msg::ModeRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jit_msgs::msg::ModeRequest>()
{
  return jit_msgs::msg::builder::Init_ModeRequest_stamp();
}

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__MODE_REQUEST__BUILDER_HPP_
