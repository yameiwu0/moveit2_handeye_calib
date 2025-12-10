// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/ToolConfig.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'parent_frame'
#include "rosidl_runtime_c/string.h"
// Member 'tcp_offset'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'com'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ToolConfig in the package robot_interfaces.
/**
  * ToolConfig.msg
 */
typedef struct robot_interfaces__msg__ToolConfig
{
  /// 工具名称
  rosidl_runtime_c__String name;
  /// TCP 所相对的父坐标系
  rosidl_runtime_c__String parent_frame;
  /// TCP 在父坐标系下的位姿
  geometry_msgs__msg__Pose tcp_offset;
  /// 工具质量
  double mass;
  /// 工具质心位置，相对 TCP 原点
  geometry_msgs__msg__Vector3 com;
} robot_interfaces__msg__ToolConfig;

// Struct for a sequence of robot_interfaces__msg__ToolConfig.
typedef struct robot_interfaces__msg__ToolConfig__Sequence
{
  robot_interfaces__msg__ToolConfig * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__ToolConfig__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__STRUCT_H_
