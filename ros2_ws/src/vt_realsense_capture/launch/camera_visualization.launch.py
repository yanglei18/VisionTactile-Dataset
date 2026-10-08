"""Read-only RViz workspace for an already-running D436 camera.

This launch neither starts the camera nor changes its stream/filter settings.
The camera owner must explicitly enable the driver's derived point cloud.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory("vt_realsense_capture"))
        / "rviz"
        / "head_camera.rviz"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_config,
                description="RViz configuration for the existing camera topics",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="vt_head_camera_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                output="screen",
            ),
        ]
    )
