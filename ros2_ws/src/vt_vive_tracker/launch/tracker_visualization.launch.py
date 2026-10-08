from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = str(
        Path(get_package_share_directory('vt_vive_tracker'))
        / 'rviz'
        / 'triple_tracker.rviz'
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rviz_config', default_value=default_config
            ),
            Node(
                package='vt_vive_tracker',
                executable='vt_vive_tracker_visualizer',
                name='vt_vive_tracker_visualizer',
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='vt_vive_tracker_rviz',
                arguments=[
                    '-d', LaunchConfiguration('rviz_config')
                ],
                output='screen',
            ),
        ]
    )
