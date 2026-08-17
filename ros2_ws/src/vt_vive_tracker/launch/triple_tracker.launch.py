from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('bundle_path'),
            DeclareLaunchArgument('role_map_path'),
            DeclareLaunchArgument('frame_id', default_value='vive_map'),
            DeclareLaunchArgument('status_rate_hz', default_value='10.0'),
            DeclareLaunchArgument('read_timeout_ms', default_value='100'),
            DeclareLaunchArgument(
                'disconnect_timeout_ms', default_value='1000'
            ),
            DeclareLaunchArgument('queue_capacity', default_value='4096'),
            Node(
                package='vt_vive_tracker',
                executable='vt_vive_tracker_node',
                name='vt_vive_tracker',
                output='screen',
                parameters=[
                    {
                        'bundle_path': LaunchConfiguration('bundle_path'),
                        'role_map_path': LaunchConfiguration('role_map_path'),
                        'frame_id': LaunchConfiguration('frame_id'),
                        'status_rate_hz': ParameterValue(
                            LaunchConfiguration('status_rate_hz'),
                            value_type=float,
                        ),
                        'read_timeout_ms': ParameterValue(
                            LaunchConfiguration('read_timeout_ms'),
                            value_type=int,
                        ),
                        'disconnect_timeout_ms': ParameterValue(
                            LaunchConfiguration('disconnect_timeout_ms'),
                            value_type=int,
                        ),
                        'queue_capacity': ParameterValue(
                            LaunchConfiguration('queue_capacity'),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
