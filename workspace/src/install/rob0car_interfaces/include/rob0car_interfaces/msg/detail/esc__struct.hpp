// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice

#ifndef ROB0CAR_INTERFACES__MSG__DETAIL__ESC__STRUCT_HPP_
#define ROB0CAR_INTERFACES__MSG__DETAIL__ESC__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__rob0car_interfaces__msg__ESC __attribute__((deprecated))
#else
# define DEPRECATED__rob0car_interfaces__msg__ESC __declspec(deprecated)
#endif

namespace rob0car_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ESC_
{
  using Type = ESC_<ContainerAllocator>;

  explicit ESC_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      this->throttle = 0;
      this->arm = false;
    }
  }

  explicit ESC_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->channel = 0;
      this->throttle = 0;
      this->arm = false;
    }
  }

  // field types and members
  using _channel_type =
    uint8_t;
  _channel_type channel;
  using _throttle_type =
    int8_t;
  _throttle_type throttle;
  using _arm_type =
    bool;
  _arm_type arm;

  // setters for named parameter idiom
  Type & set__channel(
    const uint8_t & _arg)
  {
    this->channel = _arg;
    return *this;
  }
  Type & set__throttle(
    const int8_t & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__arm(
    const bool & _arg)
  {
    this->arm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rob0car_interfaces::msg::ESC_<ContainerAllocator> *;
  using ConstRawPtr =
    const rob0car_interfaces::msg::ESC_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rob0car_interfaces::msg::ESC_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rob0car_interfaces::msg::ESC_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rob0car_interfaces__msg__ESC
    std::shared_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rob0car_interfaces__msg__ESC
    std::shared_ptr<rob0car_interfaces::msg::ESC_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ESC_ & other) const
  {
    if (this->channel != other.channel) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->arm != other.arm) {
      return false;
    }
    return true;
  }
  bool operator!=(const ESC_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ESC_

// alias to use template instance with default allocator
using ESC =
  rob0car_interfaces::msg::ESC_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rob0car_interfaces

#endif  // ROB0CAR_INTERFACES__MSG__DETAIL__ESC__STRUCT_HPP_
