// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jit_msgs:msg/ModeRequest.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__MODE_REQUEST__STRUCT_HPP_
#define JIT_MSGS__MSG__DETAIL__MODE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jit_msgs__msg__ModeRequest __attribute__((deprecated))
#else
# define DEPRECATED__jit_msgs__msg__ModeRequest __declspec(deprecated)
#endif

namespace jit_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ModeRequest_
{
  using Type = ModeRequest_<ContainerAllocator>;

  explicit ModeRequest_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->source = 0;
    }
  }

  explicit ModeRequest_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->source = 0;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _source_type =
    uint8_t;
  _source_type source;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__source(
    const uint8_t & _arg)
  {
    this->source = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SOURCE_RC =
    0u;
  static constexpr uint8_t SOURCE_AUTONOMY =
    1u;

  // pointer types
  using RawPtr =
    jit_msgs::msg::ModeRequest_<ContainerAllocator> *;
  using ConstRawPtr =
    const jit_msgs::msg::ModeRequest_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jit_msgs::msg::ModeRequest_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jit_msgs::msg::ModeRequest_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jit_msgs__msg__ModeRequest
    std::shared_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jit_msgs__msg__ModeRequest
    std::shared_ptr<jit_msgs::msg::ModeRequest_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ModeRequest_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->source != other.source) {
      return false;
    }
    return true;
  }
  bool operator!=(const ModeRequest_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ModeRequest_

// alias to use template instance with default allocator
using ModeRequest =
  jit_msgs::msg::ModeRequest_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeRequest_<ContainerAllocator>::SOURCE_RC;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ModeRequest_<ContainerAllocator>::SOURCE_AUTONOMY;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__MODE_REQUEST__STRUCT_HPP_
