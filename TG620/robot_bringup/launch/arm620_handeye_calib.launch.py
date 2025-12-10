#!/usr/bin/env python3
"""TG620 ARM620 手眼标定启动文件 (Eye-in-Hand 配置)"""

import os
from math import pi
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # 默认 rviz 配置文件路径
    default_rviz_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'rviz', 'arm620_handeye_calib.rviz'
    )

    # 1. 启动机械臂 CAN 驱动
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_driver'),
                        'launch', 'robot_driver.launch.py')
        )
    )

    # 2. 启动机械臂 SDK 功能包
    robot_func = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sdk'),
                        'launch', 'robot_sdk.launch.py')
        )
    )

    # 3. 启动机械臂描述 (URDF + TF) - 使用 DH 模型
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_description'),
                        'launch', 'arm620_dh_display.launch.py')
        )
    )

    # 4. 启动机械臂控制
    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_control'),
                        'launch', 'robot_control.launch.py')
        )
    )

    # 5. 启动 MoveIt2 (move_group + rviz)
    robot_moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('arm620_config'),
                        'launch', 'real_moveit_demo.launch.py')
        ),
        launch_arguments={
            'rviz_config': rviz_config,
        }.items()
    )

    # 6. 启动相机驱动 (RealSense D435i)
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'),
                        'launch', 'rs_launch.py')
        ),
        launch_arguments=[
            ('enable_color', 'true'),
            ('enable_depth', 'true'),
            ('pointcloud.enable', 'false'),
            ('align_depth.enable', 'true'),
            ('publish_tf', 'true'),
            ('tf_publish_rate', '10.0'),
        ]
    )

    # 7. 相机安装位置静态 TF 发布 (eye-in-hand: 相机装在末端连接板上)
    # 将 RealSense 的 camera_link 连接到机械臂 Link6
    # 相机与夹爪在同一平面 (z=0.02, 连接板高度)，Y方向偏移5.5cm
    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        arguments=[
            "--x", "0.0",        # 相机在 Link6 X方向偏移
            "--y", "0.058",      # 相机在 Link6 Y方向偏移 5.5cm
            "--z", "0.02",       # 相机在 Link6 Z方向偏移 2cm (连接板高度)
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "Link6",
            "--child-frame-id", "camera_link",  # RealSense 的 camera_link
        ],
    )

    # 8. 相机光学坐标系变换 (ROS 相机坐标系约定)
    # RealSense 相机会自动发布这个变换，如果没有则需要手动发布
    # camera_optical_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="camera_optical_tf",
    #     arguments=[
    #         "--roll", str(-pi / 2),
    #         "--pitch", "0.0",
    #         "--yaw", str(-pi / 2),
    #         "--frame-id", "camera_color_optical_frame",
    #         "--child-frame-id", "camera_link",
    #     ],
    # )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz config file for hand-eye calibration'
        ),

        # 启动节点
        robot_driver,
        robot_func,
        robot_description,
        robot_control,
        robot_moveit_config,
        camera_node,
        camera_mount_tf,
    ])
