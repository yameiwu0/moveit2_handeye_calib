from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 多障碍物配置文件
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_virtual_fence'),
        'config',
        'multi_obstacles.yaml'
    ])

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
