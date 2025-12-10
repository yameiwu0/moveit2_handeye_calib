// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/ToolConfig.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tcp_offset'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'com'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__ToolConfig __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__ToolConfig __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ToolConfig_
{
  using Type = ToolConfig_<ContainerAllocator>;

  explicit ToolConfig_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : tcp_offset(_init),
    com(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->parent_frame = "";
      this->mass = 0.0;
    }
  }

  explicit ToolConfig_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    parent_frame(_alloc),
    tcp_offset(_alloc, _init),
    com(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->parent_frame = "";
      this->mass = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _parent_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _parent_frame_type parent_frame;
  using _tcp_offset_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _tcp_offset_type tcp_offset;
  using _mass_type =
    double;
  _mass_type mass;
  using _com_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _com_type com;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__parent_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->parent_frame = _arg;
    return *this;
  }
  Type & set__tcp_offset(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->tcp_offset = _arg;
    return *this;
  }
  Type & set__mass(
    const double & _arg)
  {
    this->mass = _arg;
    return *this;
  }
  Type & set__com(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->com = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::ToolConfig_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::ToolConfig_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::ToolConfig_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::ToolConfig_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__ToolConfig
    std::shared_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__ToolConfig
    std::shared_ptr<robot_interfaces::msg::ToolConfig_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ToolConfig_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->parent_frame != other.parent_frame) {
      return false;
    }
    if (this->tcp_offset != other.tcp_offset) {
      return false;
    }
    if (this->mass != other.mass) {
      return false;
    }
    if (this->com != other.com) {
      return false;
    }
    return true;
  }
  bool operator!=(const ToolConfig_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ToolConfig_

// alias to use template instance with default allocator
using ToolConfig =
  robot_interfaces::msg::ToolConfig_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__TOOL_CONFIG__STRUCT_HPP_
