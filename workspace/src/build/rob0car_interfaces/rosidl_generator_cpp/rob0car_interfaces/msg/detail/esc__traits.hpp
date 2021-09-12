// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice

#ifndef ROB0CAR_INTERFACES__MSG__DETAIL__ESC__TRAITS_HPP_
#define ROB0CAR_INTERFACES__MSG__DETAIL__ESC__TRAITS_HPP_

#include "rob0car_interfaces/msg/detail/esc__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<rob0car_interfaces::msg::ESC>()
{
  return "rob0car_interfaces::msg::ESC";
}

template<>
inline const char * name<rob0car_interfaces::msg::ESC>()
{
  return "rob0car_interfaces/msg/ESC";
}

template<>
struct has_fixed_size<rob0car_interfaces::msg::ESC>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rob0car_interfaces::msg::ESC>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rob0car_interfaces::msg::ESC>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROB0CAR_INTERFACES__MSG__DETAIL__ESC__TRAITS_HPP_
