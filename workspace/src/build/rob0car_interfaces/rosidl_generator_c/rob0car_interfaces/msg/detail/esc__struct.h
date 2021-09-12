// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice

#ifndef ROB0CAR_INTERFACES__MSG__DETAIL__ESC__STRUCT_H_
#define ROB0CAR_INTERFACES__MSG__DETAIL__ESC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ESC in the package rob0car_interfaces.
typedef struct rob0car_interfaces__msg__ESC
{
  uint8_t channel;
  int8_t throttle;
  bool arm;
} rob0car_interfaces__msg__ESC;

// Struct for a sequence of rob0car_interfaces__msg__ESC.
typedef struct rob0car_interfaces__msg__ESC__Sequence
{
  rob0car_interfaces__msg__ESC * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rob0car_interfaces__msg__ESC__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROB0CAR_INTERFACES__MSG__DETAIL__ESC__STRUCT_H_
