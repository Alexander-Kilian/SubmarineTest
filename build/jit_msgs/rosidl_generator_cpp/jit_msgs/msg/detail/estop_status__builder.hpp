// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jit_msgs:msg/EstopStatus.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__BUILDER_HPP_
#define JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jit_msgs/msg/detail/estop_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jit_msgs
{

namespace msg
{

namespace builder
{

class Init_EstopStatus_settled
{
public:
  explicit Init_EstopStatus_settled(::jit_msgs::msg::EstopStatus & msg)
  : msg_(msg)
  {}
  ::jit_msgs::msg::EstopStatus settled(::jit_msgs::msg::EstopStatus::_settled_type arg)
  {
    msg_.settled = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jit_msgs::msg::EstopStatus msg_;
};

class Init_EstopStatus_mismatch
{
public:
  explicit Init_EstopStatus_mismatch(::jit_msgs::msg::EstopStatus & msg)
  : msg_(msg)
  {}
  Init_EstopStatus_settled mismatch(::jit_msgs::msg::EstopStatus::_mismatch_type arg)
  {
    msg_.mismatch = std::move(arg);
    return Init_EstopStatus_settled(msg_);
  }

private:
  ::jit_msgs::msg::EstopStatus msg_;
};

class Init_EstopStatus_feedback
{
public:
  explicit Init_EstopStatus_feedback(::jit_msgs::msg::EstopStatus & msg)
  : msg_(msg)
  {}
  Init_EstopStatus_mismatch feedback(::jit_msgs::msg::EstopStatus::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return Init_EstopStatus_mismatch(msg_);
  }

private:
  ::jit_msgs::msg::EstopStatus msg_;
};

class Init_EstopStatus_commanded
{
public:
  explicit Init_EstopStatus_commanded(::jit_msgs::msg::EstopStatus & msg)
  : msg_(msg)
  {}
  Init_EstopStatus_feedback commanded(::jit_msgs::msg::EstopStatus::_commanded_type arg)
  {
    msg_.commanded = std::move(arg);
    return Init_EstopStatus_feedback(msg_);
  }

private:
  ::jit_msgs::msg::EstopStatus msg_;
};

class Init_EstopStatus_stamp
{
public:
  Init_EstopStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EstopStatus_commanded stamp(::jit_msgs::msg::EstopStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_EstopStatus_commanded(msg_);
  }

private:
  ::jit_msgs::msg::EstopStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jit_msgs::msg::EstopStatus>()
{
  return jit_msgs::msg::builder::Init_EstopStatus_stamp();
}

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__ESTOP_STATUS__BUILDER_HPP_
