// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rob0car_interfaces/msg/detail/esc__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rob0car_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ESC_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rob0car_interfaces::msg::ESC(_init);
}

void ESC_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rob0car_interfaces::msg::ESC *>(message_memory);
  typed_message->~ESC();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ESC_message_member_array[3] = {
  {
    "channel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rob0car_interfaces::msg::ESC, channel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "throttle",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rob0car_interfaces::msg::ESC, throttle),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "arm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rob0car_interfaces::msg::ESC, arm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ESC_message_members = {
  "rob0car_interfaces::msg",  // message namespace
  "ESC",  // message name
  3,  // number of fields
  sizeof(rob0car_interfaces::msg::ESC),
  ESC_message_member_array,  // message members
  ESC_init_function,  // function to initialize message memory (memory has to be allocated)
  ESC_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ESC_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ESC_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rob0car_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rob0car_interfaces::msg::ESC>()
{
  return &::rob0car_interfaces::msg::rosidl_typesupport_introspection_cpp::ESC_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rob0car_interfaces, msg, ESC)() {
  return &::rob0car_interfaces::msg::rosidl_typesupport_introspection_cpp::ESC_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
