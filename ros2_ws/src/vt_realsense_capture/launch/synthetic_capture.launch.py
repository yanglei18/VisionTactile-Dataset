"""TEST ONLY: minimal Recorder lifecycle graph."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SYNTHETIC_TEST_ONLY = True


def generate_launch_description() -> LaunchDescription:
    output_root = LaunchConfiguration('output_root')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'output_root',
                description='TEST ONLY: absolute temporary Recorder output root',
            ),
            Node(
                package='vt_realsense_capture',
                executable='synthetic_capture_test_support',
                name='synthetic_capture_support',
                output='screen',
            ),
            Node(
                package='vt_realsense_capture',
                executable='capture_controller',
                name='capture_controller',
                parameters=[{'output_root': output_root}],
                output='screen',
            ),
        ]
    )
