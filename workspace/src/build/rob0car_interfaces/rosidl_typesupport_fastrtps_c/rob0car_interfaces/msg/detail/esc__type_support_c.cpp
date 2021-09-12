// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice
#include "rob0car_interfaces/msg/detail/esc__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rob0car_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rob0car_interfaces/msg/detail/esc__struct.h"
#include "rob0car_interfaces/msg/detail/esc__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ESC__ros_msg_type = rob0car_interfaces__msg__ESC;

static bool _ESC__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ESC__ros_msg_type * ros_message = static_cast<const _ESC__ros_msg_type *>(untyped_ros_message);
  // Field name: channel
  {
    cdr << ros_message->channel;
  }

  // Field name: throttle
  {
    cdr << ros_message->throttle;
  }

  // Field name: arm
  {
    cdr << (ros_message->arm ? true : false);
  }

  return true;
}

static bool _ESC__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ESC__ros_msg_type * ros_message = static_cast<_ESC__ros_msg_type *>(untyped_ros_message);
  // Field name: channel
  {
    cdr >> ros_message->channel;
  }

  // Field name: throttle
  {
    cdr >> ros_message->throttle;
  }

  // Field name: arm
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->arm = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rob0car_interfaces
size_t get_serialized_size_rob0car_interfaces__msg__ESC(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ESC__ros_msg_type * ros_message = static_cast<const _ESC__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name channel
  {
    size_t item_size = sizeof(ros_message->channel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name throttle
  {
    size_t item_size = sizeof(ros_message->throttle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arm
  {
    size_t item_size = sizeof(ros_message->arm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ESC__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rob0car_interfaces__msg__ESC(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rob0car_interfaces
size_t max_serialized_size_rob0car_interfaces__msg__ESC(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: channel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: throttle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _ESC__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_rob0car_interfaces__msg__ESC(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_ESC = {
  "rob0car_interfaces::msg",
  "ESC",
  _ESC__cdr_serialize,
  _ESC__cdr_deserialize,
  _ESC__get_serialized_size,
  _ESC__max_serialized_size
};

static rosidl_message_type_support_t _ESC__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ESC,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rob0car_interfaces, msg, ESC)() {
  return &_ESC__type_support;
}

#if defined(__cplusplus)
}
#endif
