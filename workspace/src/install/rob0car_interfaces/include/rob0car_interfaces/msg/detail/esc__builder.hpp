// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice

#ifndef ROB0CAR_INTERFACES__MSG__DETAIL__ESC__BUILDER_HPP_
#define ROB0CAR_INTERFACES__MSG__DETAIL__ESC__BUILDER_HPP_

#include "rob0car_interfaces/msg/detail/esc__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace rob0car_interfaces
{

namespace msg
{

namespace builder
{

class Init_ESC_arm
{
public:
  explicit Init_ESC_arm(::rob0car_interfaces::msg::ESC & msg)
  : msg_(msg)
  {}
  ::rob0car_interfaces::msg::ESC arm(::rob0car_interfaces::msg::ESC::_arm_type arg)
  {
    msg_.arm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rob0car_interfaces::msg::ESC msg_;
};

class Init_ESC_throttle
{
public:
  explicit Init_ESC_throttle(::rob0car_interfaces::msg::ESC & msg)
  : msg_(msg)
  {}
  Init_ESC_arm throttle(::rob0car_interfaces::msg::ESC::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_ESC_arm(msg_);
  }

private:
  ::rob0car_interfaces::msg::ESC msg_;
};

class Init_ESC_channel
{
public:
  Init_ESC_channel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ESC_throttle channel(::rob0car_interfaces::msg::ESC::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_ESC_throttle(msg_);
  }

private:
  ::rob0car_interfaces::msg::ESC msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rob0car_interfaces::msg::ESC>()
{
  return rob0car_interfaces::msg::builder::Init_ESC_channel();
}

}  // namespace rob0car_interfaces

#endif  // ROB0CAR_INTERFACES__MSG__DETAIL__ESC__BUILDER_HPP_
