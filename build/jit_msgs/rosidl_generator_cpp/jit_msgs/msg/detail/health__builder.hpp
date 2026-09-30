// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jit_msgs:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__HEALTH__BUILDER_HPP_
#define JIT_MSGS__MSG__DETAIL__HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jit_msgs/msg/detail/health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jit_msgs
{

namespace msg
{

namespace builder
{

class Init_Health_detail
{
public:
  explicit Init_Health_detail(::jit_msgs::msg::Health & msg)
  : msg_(msg)
  {}
  ::jit_msgs::msg::Health detail(::jit_msgs::msg::Health::_detail_type arg)
  {
    msg_.detail = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jit_msgs::msg::Health msg_;
};

class Init_Health_faults
{
public:
  explicit Init_Health_faults(::jit_msgs::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_detail faults(::jit_msgs::msg::Health::_faults_type arg)
  {
    msg_.faults = std::move(arg);
    return Init_Health_detail(msg_);
  }

private:
  ::jit_msgs::msg::Health msg_;
};

class Init_Health_ok
{
public:
  explicit Init_Health_ok(::jit_msgs::msg::Health & msg)
  : msg_(msg)
  {}
  Init_Health_faults ok(::jit_msgs::msg::Health::_ok_type arg)
  {
    msg_.ok = std::move(arg);
    return Init_Health_faults(msg_);
  }

private:
  ::jit_msgs::msg::Health msg_;
};

class Init_Health_stamp
{
public:
  Init_Health_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Health_ok stamp(::jit_msgs::msg::Health::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Health_ok(msg_);
  }

private:
  ::jit_msgs::msg::Health msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jit_msgs::msg::Health>()
{
  return jit_msgs::msg::builder::Init_Health_stamp();
}

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__HEALTH__BUILDER_HPP_
