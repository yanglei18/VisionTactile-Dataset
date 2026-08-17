from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vt_realsense_capture.config import load_config
from vt_realsense_capture.launch_arguments import camera_launch_arguments


def _launch_setup(context: LaunchContext):
    capture_share = Path(get_package_share_directory("vt_realsense_capture"))
    realsense_share = Path(get_package_share_directory("realsense2_camera"))
    config_path = Path(LaunchConfiguration("config_path").perform(context))
    output_root = LaunchConfiguration("output_root").perform(context)
    options_path = capture_share / "config" / "realsense_options.yaml"
    official_launch = realsense_share / "launch" / "rs_launch.py"
    config = load_config(config_path)

    actions = []
    for camera in config.cameras:
        arguments = camera_launch_arguments(
            camera, config, config_file=str(options_path)
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(official_launch)),
                launch_arguments=arguments.items(),
            )
        )

    actions.append(
        Node(
            package="vt_realsense_capture",
            executable="timing_normalizer",
            name="timing_normalizer",
            parameters=[
                {
                    "camera_names": [camera.name for camera in config.cameras],
                    "camera_models": [
                        camera.model_token for camera in config.cameras
                    ],
                    "serial_numbers": [camera.serial for camera in config.cameras],
                    "max_wait_ns": 150_000_000,
                    "max_newer_stamps": 4,
                }
            ],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="vt_realsense_capture",
            executable="capture_controller",
            name="capture_controller",
            parameters=[
                {
                    "config_path": str(config_path),
                    "output_root": output_root,
                }
            ],
            output="screen",
        )
    )
    return actions


def generate_launch_description():
    capture_share = Path(get_package_share_directory("vt_realsense_capture"))
    default_config = capture_share / "config" / "cameras.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=str(default_config),
                description="Path to the fixed three-camera capture configuration",
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value="",
                description=(
                    "Absolute existing writable capture root outside the repository"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
