#include "robot_kinematics.hpp"
void RobotKinematics::speed_config_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    RCLCPP_INFO(node_->get_logger(),
                "Updated speed_scaling,accel_scaling");
    double speed_scaling_ = std::clamp(msg->data[0], 0.1, 1.0);
    double accel_scaling_ = std::clamp(msg->data[1], 0.1, 1.0);

    arm_->setMaxVelocityScalingFactor(speed_scaling_);
    arm_->setMaxAccelerationScalingFactor(accel_scaling_);

    RCLCPP_INFO(node_->get_logger(),
                "Updated speed_scaling=%.2f, accel_scaling=%.2f",
                speed_scaling_, accel_scaling_);
}
void RobotKinematics::tool_config_callback(const robot_interfaces::msg::ToolConfig::SharedPtr msg)
{
    tcp_offset = msg->tcp_offset;

    RCLCPP_INFO(node_->get_logger(),
                "收到工具配置: 名称='%s', TCP偏移位置=[%.3f, %.3f, %.3f], 姿态=[%.3f, %.3f, %.3f, %.3f], 质量=%.3f kg",
                msg->name.c_str(),
                msg->tcp_offset.position.x, msg->tcp_offset.position.y, msg->tcp_offset.position.z,
                msg->tcp_offset.orientation.x, msg->tcp_offset.orientation.y,
                msg->tcp_offset.orientation.z, msg->tcp_offset.orientation.w,
                msg->mass);
}