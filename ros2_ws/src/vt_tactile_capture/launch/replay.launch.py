"""Hardware-free reconstruction. Run ros2 bag play BAG --clock separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="/gloves/right/tactile"),
        DeclareLaunchArgument("hand", default_value="right"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("gui_python", default_value="/usr/bin/python3"),
        DeclareLaunchArgument("hand_layout_path", default_value="",
                              description="Viewer-only per-hand candidate JSON; empty uses right layout or left raw matrix only."),
        DeclareLaunchArgument("baseline_path", default_value="",
                              description="Viewer-only reference NPZ with companion JSON; must match the replay session and epoch."),
        DeclareLaunchArgument("initial_view", default_value="hand",
                              description="Viewer initial tab: hand or matrix."),
        DeclareLaunchArgument("max_cell_age_ms", default_value="100.0"),
        DeclareLaunchArgument("publish_heatmap", default_value="false"),
        Node(package="vt_tactile_capture", executable="vt_tactile_replay",
             namespace=LaunchConfiguration("namespace"), output="screen",
             parameters=[{"use_sim_time": True, "hand": LaunchConfiguration("hand"),
                          "max_cell_age_ms": ParameterValue(LaunchConfiguration("max_cell_age_ms"), value_type=float),
                          "publish_heatmap": ParameterValue(LaunchConfiguration("publish_heatmap"), value_type=bool)}]),
        Node(package="vt_tactile_capture", executable="vt_tactile_viewer",
             namespace=LaunchConfiguration("namespace"), output="screen",
             condition=IfCondition(LaunchConfiguration("gui")),
             prefix=[LaunchConfiguration("gui_python")], parameters=[{
                 "use_sim_time": True,
                 "hand": ParameterValue(LaunchConfiguration("hand"), value_type=str),
                 "hand_layout_path": ParameterValue(LaunchConfiguration("hand_layout_path"), value_type=str),
                 "baseline_path": ParameterValue(LaunchConfiguration("baseline_path"), value_type=str),
                 "initial_view": ParameterValue(LaunchConfiguration("initial_view"), value_type=str),
             }]),
    ])
