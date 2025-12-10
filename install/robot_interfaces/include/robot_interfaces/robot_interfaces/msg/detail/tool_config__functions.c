// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/ToolConfig.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/tool_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `parent_frame`
#include "rosidl_runtime_c/string_functions.h"
// Member `tcp_offset`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `com`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
robot_interfaces__msg__ToolConfig__init(robot_interfaces__msg__ToolConfig * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    robot_interfaces__msg__ToolConfig__fini(msg);
    return false;
  }
  // parent_frame
  if (!rosidl_runtime_c__String__init(&msg->parent_frame)) {
    robot_interfaces__msg__ToolConfig__fini(msg);
    return false;
  }
  // tcp_offset
  if (!geometry_msgs__msg__Pose__init(&msg->tcp_offset)) {
    robot_interfaces__msg__ToolConfig__fini(msg);
    return false;
  }
  // mass
  // com
  if (!geometry_msgs__msg__Vector3__init(&msg->com)) {
    robot_interfaces__msg__ToolConfig__fini(msg);
    return false;
  }
  return true;
}

void
robot_interfaces__msg__ToolConfig__fini(robot_interfaces__msg__ToolConfig * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // parent_frame
  rosidl_runtime_c__String__fini(&msg->parent_frame);
  // tcp_offset
  geometry_msgs__msg__Pose__fini(&msg->tcp_offset);
  // mass
  // com
  geometry_msgs__msg__Vector3__fini(&msg->com);
}

bool
robot_interfaces__msg__ToolConfig__are_equal(const robot_interfaces__msg__ToolConfig * lhs, const robot_interfaces__msg__ToolConfig * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // parent_frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->parent_frame), &(rhs->parent_frame)))
  {
    return false;
  }
  // tcp_offset
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->tcp_offset), &(rhs->tcp_offset)))
  {
    return false;
  }
  // mass
  if (lhs->mass != rhs->mass) {
    return false;
  }
  // com
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->com), &(rhs->com)))
  {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__ToolConfig__copy(
  const robot_interfaces__msg__ToolConfig * input,
  robot_interfaces__msg__ToolConfig * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // parent_frame
  if (!rosidl_runtime_c__String__copy(
      &(input->parent_frame), &(output->parent_frame)))
  {
    return false;
  }
  // tcp_offset
  if (!geometry_msgs__msg__Pose__copy(
      &(input->tcp_offset), &(output->tcp_offset)))
  {
    return false;
  }
  // mass
  output->mass = input->mass;
  // com
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->com), &(output->com)))
  {
    return false;
  }
  return true;
}

robot_interfaces__msg__ToolConfig *
robot_interfaces__msg__ToolConfig__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__ToolConfig * msg = (robot_interfaces__msg__ToolConfig *)allocator.allocate(sizeof(robot_interfaces__msg__ToolConfig), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__ToolConfig));
  bool success = robot_interfaces__msg__ToolConfig__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__ToolConfig__destroy(robot_interfaces__msg__ToolConfig * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__ToolConfig__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__ToolConfig__Sequence__init(robot_interfaces__msg__ToolConfig__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__ToolConfig * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__ToolConfig *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__ToolConfig), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__ToolConfig__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__ToolConfig__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_interfaces__msg__ToolConfig__Sequence__fini(robot_interfaces__msg__ToolConfig__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_interfaces__msg__ToolConfig__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_interfaces__msg__ToolConfig__Sequence *
robot_interfaces__msg__ToolConfig__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__ToolConfig__Sequence * array = (robot_interfaces__msg__ToolConfig__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__ToolConfig__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__ToolConfig__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__ToolConfig__Sequence__destroy(robot_interfaces__msg__ToolConfig__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__ToolConfig__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__ToolConfig__Sequence__are_equal(const robot_interfaces__msg__ToolConfig__Sequence * lhs, const robot_interfaces__msg__ToolConfig__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__ToolConfig__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__ToolConfig__Sequence__copy(
  const robot_interfaces__msg__ToolConfig__Sequence * input,
  robot_interfaces__msg__ToolConfig__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__ToolConfig);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__ToolConfig * data =
      (robot_interfaces__msg__ToolConfig *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__ToolConfig__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__ToolConfig__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__ToolConfig__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
