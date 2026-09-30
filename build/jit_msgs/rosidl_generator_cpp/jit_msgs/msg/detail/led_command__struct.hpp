// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jit_msgs:msg/LedCommand.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__LED_COMMAND__STRUCT_HPP_
#define JIT_MSGS__MSG__DETAIL__LED_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__jit_msgs__msg__LedCommand __attribute__((deprecated))
#else
# define DEPRECATED__jit_msgs__msg__LedCommand __declspec(deprecated)
#endif

namespace jit_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LedCommand_
{
  using Type = LedCommand_<ContainerAllocator>;

  explicit LedCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pattern = "";
    }
  }

  explicit LedCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    pattern(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pattern = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _pattern_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pattern_type pattern;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__pattern(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pattern = _arg;
    return *this;
  }

  // constant declarations
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> ESTOP;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MANUAL;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> AUTO;

  // pointer types
  using RawPtr =
    jit_msgs::msg::LedCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const jit_msgs::msg::LedCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jit_msgs::msg::LedCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jit_msgs::msg::LedCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jit_msgs__msg__LedCommand
    std::shared_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jit_msgs__msg__LedCommand
    std::shared_ptr<jit_msgs::msg::LedCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LedCommand_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->pattern != other.pattern) {
      return false;
    }
    return true;
  }
  bool operator!=(const LedCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LedCommand_

// alias to use template instance with default allocator
using LedCommand =
  jit_msgs::msg::LedCommand_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
LedCommand_<ContainerAllocator>::ESTOP = "ESTOP";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
LedCommand_<ContainerAllocator>::MANUAL = "MANUAL";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
LedCommand_<ContainerAllocator>::AUTO = "AUTO";

}  // namespace msg

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__LED_COMMAND__STRUCT_HPP_
