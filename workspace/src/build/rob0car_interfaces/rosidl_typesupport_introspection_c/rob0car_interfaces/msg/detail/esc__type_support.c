// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rob0car_interfaces/msg/detail/esc__rosidl_typesupport_introspection_c.h"
#include "rob0car_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rob0car_interfaces/msg/detail/esc__functions.h"
#include "rob0car_interfaces/msg/detail/esc__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ESC__rosidl_typesupport_introspection_c__ESC_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rob0car_interfaces__msg__ESC__init(message_memory);
}

void ESC__rosidl_typesupport_introspection_c__ESC_fini_function(void * message_memory)
{
  rob0car_interfaces__msg__ESC__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ESC__rosidl_typesupport_introspection_c__ESC_message_member_array[3] = {
  {
    "channel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rob0car_interfaces__msg__ESC, channel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "throttle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rob0car_interfaces__msg__ESC, throttle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "arm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rob0car_interfaces__msg__ESC, arm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ESC__rosidl_typesupport_introspection_c__ESC_message_members = {
  "rob0car_interfaces__msg",  // message namespace
  "ESC",  // message name
  3,  // number of fields
  sizeof(rob0car_interfaces__msg__ESC),
  ESC__rosidl_typesupport_introspection_c__ESC_message_member_array,  // message members
  ESC__rosidl_typesupport_introspection_c__ESC_init_function,  // function to initialize message memory (memory has to be allocated)
  ESC__rosidl_typesupport_introspection_c__ESC_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ESC__rosidl_typesupport_introspection_c__ESC_message_type_support_handle = {
  0,
  &ESC__rosidl_typesupport_introspection_c__ESC_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rob0car_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rob0car_interfaces, msg, ESC)() {
  if (!ESC__rosidl_typesupport_introspection_c__ESC_message_type_support_handle.typesupport_identifier) {
    ESC__rosidl_typesupport_introspection_c__ESC_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ESC__rosidl_typesupport_introspection_c__ESC_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
