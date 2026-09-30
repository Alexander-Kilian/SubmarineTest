// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jit_msgs:msg/Health.idl
// generated code does not contain a copyright notice

#ifndef JIT_MSGS__MSG__DETAIL__HEALTH__STRUCT_HPP_
#define JIT_MSGS__MSG__DETAIL__HEALTH__STRUCT_HPP_

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
# define DEPRECATED__jit_msgs__msg__Health __attribute__((deprecated))
#else
# define DEPRECATED__jit_msgs__msg__Health __declspec(deprecated)
#endif

namespace jit_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Health_
{
  using Type = Health_<ContainerAllocator>;

  explicit Health_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ok = false;
      this->faults = 0ul;
      this->detail = "";
    }
  }

  explicit Health_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    detail(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ok = false;
      this->faults = 0ul;
      this->detail = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _ok_type =
    bool;
  _ok_type ok;
  using _faults_type =
    uint32_t;
  _faults_type faults;
  using _detail_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _detail_type detail;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__ok(
    const bool & _arg)
  {
    this->ok = _arg;
    return *this;
  }
  Type & set__faults(
    const uint32_t & _arg)
  {
    this->faults = _arg;
    return *this;
  }
  Type & set__detail(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->detail = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t RC_LINK_LOST =
    1u;
  static constexpr uint32_t CRSF_NODE_DEAD =
    2u;
  static constexpr uint32_t MAVROS_LOST =
    4u;
  static constexpr uint32_t ESTOP_NODE_DEAD =
    8u;
  static constexpr uint32_t ESTOP_OPEN =
    16u;
  static constexpr uint32_t RELAY_NOT_CLOSED =
    32u;
  static constexpr uint32_t RELAY_WELDED =
    64u;
  static constexpr uint32_t MISSION_COMPLETE =
    128u;
  static constexpr uint32_t FAULT_SAFE_MASK =
    15u;

  // pointer types
  using RawPtr =
    jit_msgs::msg::Health_<ContainerAllocator> *;
  using ConstRawPtr =
    const jit_msgs::msg::Health_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jit_msgs::msg::Health_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jit_msgs::msg::Health_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jit_msgs::msg::Health_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jit_msgs::msg::Health_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jit_msgs::msg::Health_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jit_msgs::msg::Health_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jit_msgs::msg::Health_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jit_msgs::msg::Health_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jit_msgs__msg__Health
    std::shared_ptr<jit_msgs::msg::Health_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jit_msgs__msg__Health
    std::shared_ptr<jit_msgs::msg::Health_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Health_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->ok != other.ok) {
      return false;
    }
    if (this->faults != other.faults) {
      return false;
    }
    if (this->detail != other.detail) {
      return false;
    }
    return true;
  }
  bool operator!=(const Health_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Health_

// alias to use template instance with default allocator
using Health =
  jit_msgs::msg::Health_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::RC_LINK_LOST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::CRSF_NODE_DEAD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::MAVROS_LOST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::ESTOP_NODE_DEAD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::ESTOP_OPEN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::RELAY_NOT_CLOSED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::RELAY_WELDED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::MISSION_COMPLETE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t Health_<ContainerAllocator>::FAULT_SAFE_MASK;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace jit_msgs

#endif  // JIT_MSGS__MSG__DETAIL__HEALTH__STRUCT_HPP_
