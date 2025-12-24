""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: Link6 -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "Link6",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.0319045",
                "--y",
                "0.0530007",
                "--z",
                "-0.0141292",
                "--qx",
                "0.0315969",
                "--qy",
                "0.00402934",
                "--qz",
                "-0.0201579",
                "--qw",
                "0.999289",
                # "--roll",
                # "0.0633551",
                # "--pitch",
                # "0.00677914",
                # "--yaw",
                # "-0.0405538",
            ],
        ),
    ]
    return LaunchDescription(nodes)
