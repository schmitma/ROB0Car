// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice
#include "rob0car_interfaces/msg/detail/esc__rosidl_typesupport_fastrtps_cpp.hpp"
#include "rob0car_interfaces/msg/detail/esc__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace rob0car_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rob0car_interfaces
cdr_serialize(
  const rob0car_interfaces::msg::ESC & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: channel
  cdr << ros_message.channel;
  // Member: throttle
  cdr << ros_message.throttle;
  // Member: arm
  cdr << (ros_message.arm ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rob0car_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  rob0car_interfaces::msg::ESC & ros_message)
{
  // Member: channel
  cdr >> ros_message.channel;

  // Member: throttle
  cdr >> ros_message.throttle;

  // Member: arm
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.arm = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rob0car_interfaces
get_serialized_size(
  const rob0car_interfaces::msg::ESC & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: channel
  {
    size_t item_size = sizeof(ros_message.channel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: throttle
  {
    size_t item_size = sizeof(ros_message.throttle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arm
  {
    size_t item_size = sizeof(ros_message.arm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_rob0car_interfaces
max_serialized_size_ESC(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: channel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: throttle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _ESC__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const rob0car_interfaces::msg::ESC *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ESC__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rob0car_interfaces::msg::ESC *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ESC__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rob0car_interfaces::msg::ESC *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ESC__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ESC(full_bounded, 0);
}

static message_type_support_callbacks_t _ESC__callbacks = {
  "rob0car_interfaces::msg",
  "ESC",
  _ESC__cdr_serialize,
  _ESC__cdr_deserialize,
  _ESC__get_serialized_size,
  _ESC__max_serialized_size
};

static rosidl_message_type_support_t _ESC__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ESC__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace rob0car_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_rob0car_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<rob0car_interfaces::msg::ESC>()
{
  return &rob0car_interfaces::msg::typesupport_fastrtps_cpp::_ESC__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, rob0car_interfaces, msg, ESC)() {
  return &rob0car_interfaces::msg::typesupport_fastrtps_cpp::_ESC__handle;
}

#ifdef __cplusplus
}
#endif
