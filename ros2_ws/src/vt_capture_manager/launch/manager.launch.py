"""Launch only the manager: read-only by default, managed recording opt-in.

This launch file does not start acquisition nodes or automatically record.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _manager(context):
    arguments = []
    for argument, flag in (("config", "--config"), ("socket_path", "--socket")):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            arguments.extend((flag, value))
    state_dir = LaunchConfiguration("state_dir").perform(context)
    if state_dir:
        arguments.extend(("--state-dir", state_dir))
    for name, flag in (("enable_recording", "--enable-recording"), ("enable_device_control", "--enable-device-control")):
        enabled = LaunchConfiguration(name).perform(context).lower()
        if enabled not in {"true", "false"}:
            raise ValueError(f"{name} must be true or false")
        if enabled == "true":
            arguments.append(flag)
    return [Node(package="vt_capture_manager", executable="vt_capture_manager", output="screen",
                 arguments=arguments)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value="", description="Optional manager JSON/YAML; defaults to the 13 known streams."),
        DeclareLaunchArgument("socket_path", default_value="", description="Empty uses the per-user private manager socket."),
        DeclareLaunchArgument("enable_recording", default_value="false", description="Explicitly enable managed recorder service commands; never starts devices."),
        DeclareLaunchArgument("enable_device_control", default_value="false", description="Allow explicit device start/stop actions from configured launch profiles; no autostart."),
        DeclareLaunchArgument("state_dir", default_value="", description="Private durable management state directory; unused in read-only mode."),
        OpaqueFunction(function=_manager),
    ])
