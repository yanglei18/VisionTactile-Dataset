from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vt_vive_tracker_gui",
                executable="vt_vive_tracker_gui",
                output="screen",
            )
        ]
    )
