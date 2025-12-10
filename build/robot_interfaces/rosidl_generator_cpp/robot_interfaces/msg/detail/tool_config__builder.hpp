// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/ToolConfig.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/tool_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ToolConfig_com
{
public:
  explicit Init_ToolConfig_com(::robot_interfaces::msg::ToolConfig & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::ToolConfig com(::robot_interfaces::msg::ToolConfig::_com_type arg)
  {
    msg_.com = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::ToolConfig msg_;
};

class Init_ToolConfig_mass
{
public:
  explicit Init_ToolConfig_mass(::robot_interfaces::msg::ToolConfig & msg)
  : msg_(msg)
  {}
  Init_ToolConfig_com mass(::robot_interfaces::msg::ToolConfig::_mass_type arg)
  {
    msg_.mass = std::move(arg);
    return Init_ToolConfig_com(msg_);
  }

private:
  ::robot_interfaces::msg::ToolConfig msg_;
};

class Init_ToolConfig_tcp_offset
{
public:
  explicit Init_ToolConfig_tcp_offset(::robot_interfaces::msg::ToolConfig & msg)
  : msg_(msg)
  {}
  Init_ToolConfig_mass tcp_offset(::robot_interfaces::msg::ToolConfig::_tcp_offset_type arg)
  {
    msg_.tcp_offset = std::move(arg);
    return Init_ToolConfig_mass(msg_);
  }

private:
  ::robot_interfaces::msg::ToolConfig msg_;
};

class Init_ToolConfig_parent_frame
{
public:
  explicit Init_ToolConfig_parent_frame(::robot_interfaces::msg::ToolConfig & msg)
  : msg_(msg)
  {}
  Init_ToolConfig_tcp_offset parent_frame(::robot_interfaces::msg::ToolConfig::_parent_frame_type arg)
  {
    msg_.parent_frame = std::move(arg);
    return Init_ToolConfig_tcp_offset(msg_);
  }

private:
  ::robot_interfaces::msg::ToolConfig msg_;
};

class Init_ToolConfig_name
{
public:
  Init_ToolConfig_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ToolConfig_parent_frame name(::robot_interfaces::msg::ToolConfig::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_ToolConfig_parent_frame(msg_);
  }

private:
  ::robot_interfaces::msg::ToolConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::ToolConfig>()
{
  return robot_interfaces::msg::builder::Init_ToolConfig_name();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__BUILDER_HPP_
