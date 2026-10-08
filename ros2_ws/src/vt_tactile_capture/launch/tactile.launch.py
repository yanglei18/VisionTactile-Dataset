"""Live acquisition plus an optional independently subscribed viewer."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _capture_node(context):
    overrides = {
        "publish_heatmap": ParameterValue(LaunchConfiguration("publish_heatmap"), value_type=bool),
        "use_sim_time": False,
    }
    # Empty launch defaults must not erase explicit values in the YAML file.
    for name in ("port", "hand"):
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            overrides[name] = value
    return [Node(
        package="vt_tactile_capture", executable="vt_tactile_node",
        name="tactile_capture", namespace=LaunchConfiguration("namespace"),
        output="screen", parameters=[LaunchConfiguration("config"), overrides],
    )]


def _viewer_node(context):
    display = {
        key: ParameterValue(LaunchConfiguration(key), value_type=str)
        for key in ("hand_layout_path", "baseline_path", "initial_view")
    }
    hand = LaunchConfiguration("hand").perform(context).strip()
    if hand:
        display["hand"] = hand
    return [Node(
        package="vt_tactile_capture", executable="vt_tactile_viewer",
        name="tactile_viewer", namespace=LaunchConfiguration("namespace"),
        condition=IfCondition(LaunchConfiguration("gui")), output="screen",
        prefix=[LaunchConfiguration("gui_python")],
        parameters=[LaunchConfiguration("config"), display],
    )]


def generate_launch_description():
    config = get_package_share_directory("vt_tactile_capture") + "/config/right_tactile.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="/gloves/right/tactile"),
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("port", default_value="",
                              description="Override tactile device; empty uses config port (both empty = disconnected)."),
        DeclareLaunchArgument("hand", default_value="",
                              description="Override config hand; empty uses YAML (default right). Set matching namespace for left."),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("gui_python", default_value="/usr/bin/python3",
                              description="Python with PyQt5/pyqtgraph and ROS imports."),
        DeclareLaunchArgument("hand_layout_path", default_value="",
                              description="Viewer-only per-hand candidate JSON; empty uses right layout or left raw matrix only."),
        DeclareLaunchArgument("baseline_path", default_value="",
                              description="Viewer-only unpressed reference NPZ with companion JSON; must match session and epoch."),
        DeclareLaunchArgument("initial_view", default_value="hand",
                              description="Viewer initial tab: hand or matrix. Does not change acquisition."),
        DeclareLaunchArgument("publish_heatmap", default_value="false"),
        LogInfo(msg="Tactile acquisition uses the explicit port in config or port:=...; neither set = disconnected, never guessed."),
        OpaqueFunction(function=_capture_node),
        OpaqueFunction(function=_viewer_node),
    ])
