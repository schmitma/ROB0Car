// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rob0car_interfaces:msg/ESC.idl
// generated code does not contain a copyright notice
#include "rob0car_interfaces/msg/detail/esc__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
rob0car_interfaces__msg__ESC__init(rob0car_interfaces__msg__ESC * msg)
{
  if (!msg) {
    return false;
  }
  // channel
  // throttle
  // arm
  return true;
}

void
rob0car_interfaces__msg__ESC__fini(rob0car_interfaces__msg__ESC * msg)
{
  if (!msg) {
    return;
  }
  // channel
  // throttle
  // arm
}

rob0car_interfaces__msg__ESC *
rob0car_interfaces__msg__ESC__create()
{
  rob0car_interfaces__msg__ESC * msg = (rob0car_interfaces__msg__ESC *)malloc(sizeof(rob0car_interfaces__msg__ESC));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rob0car_interfaces__msg__ESC));
  bool success = rob0car_interfaces__msg__ESC__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
rob0car_interfaces__msg__ESC__destroy(rob0car_interfaces__msg__ESC * msg)
{
  if (msg) {
    rob0car_interfaces__msg__ESC__fini(msg);
  }
  free(msg);
}


bool
rob0car_interfaces__msg__ESC__Sequence__init(rob0car_interfaces__msg__ESC__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rob0car_interfaces__msg__ESC * data = NULL;
  if (size) {
    data = (rob0car_interfaces__msg__ESC *)calloc(size, sizeof(rob0car_interfaces__msg__ESC));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rob0car_interfaces__msg__ESC__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rob0car_interfaces__msg__ESC__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rob0car_interfaces__msg__ESC__Sequence__fini(rob0car_interfaces__msg__ESC__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rob0car_interfaces__msg__ESC__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rob0car_interfaces__msg__ESC__Sequence *
rob0car_interfaces__msg__ESC__Sequence__create(size_t size)
{
  rob0car_interfaces__msg__ESC__Sequence * array = (rob0car_interfaces__msg__ESC__Sequence *)malloc(sizeof(rob0car_interfaces__msg__ESC__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = rob0car_interfaces__msg__ESC__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
rob0car_interfaces__msg__ESC__Sequence__destroy(rob0car_interfaces__msg__ESC__Sequence * array)
{
  if (array) {
    rob0car_interfaces__msg__ESC__Sequence__fini(array);
  }
  free(array);
}
