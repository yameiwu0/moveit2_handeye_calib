from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_virtual_fence'),
        'config',
        'obstacles_example.yaml'
    ])

    # 虚拟围栏节点
    virtual_fence_node = Node(
        package='robot_virtual_fence',
        executable='virtual_fence_node',
        name='virtual_fence_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        virtual_fence_node
    ])
