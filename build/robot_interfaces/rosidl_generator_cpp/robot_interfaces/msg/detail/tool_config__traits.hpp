// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/ToolConfig.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/tool_config__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'tcp_offset'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'com'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ToolConfig & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: parent_frame
  {
    out << "parent_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_frame, out);
    out << ", ";
  }

  // member: tcp_offset
  {
    out << "tcp_offset: ";
    to_flow_style_yaml(msg.tcp_offset, out);
    out << ", ";
  }

  // member: mass
  {
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << ", ";
  }

  // member: com
  {
    out << "com: ";
    to_flow_style_yaml(msg.com, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ToolConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: parent_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_frame, out);
    out << "\n";
  }

  // member: tcp_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_offset:\n";
    to_block_style_yaml(msg.tcp_offset, out, indentation + 2);
  }

  // member: mass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mass: ";
    rosidl_generator_traits::value_to_yaml(msg.mass, out);
    out << "\n";
  }

  // member: com
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "com:\n";
    to_block_style_yaml(msg.com, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ToolConfig & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::ToolConfig & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::ToolConfig & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::ToolConfig>()
{
  return "robot_interfaces::msg::ToolConfig";
}

template<>
inline const char * name<robot_interfaces::msg::ToolConfig>()
{
  return "robot_interfaces/msg/ToolConfig";
}

template<>
struct has_fixed_size<robot_interfaces::msg::ToolConfig>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::msg::ToolConfig>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::msg::ToolConfig>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__TRAITS_HPP_
