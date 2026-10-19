"""Read-only live ROS-to-GUI IPC bridge; does not start acquisition or a GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hand", default_value="right"),
        DeclareLaunchArgument("namespace", default_value=["/gloves/", LaunchConfiguration("hand"), "/tactile"]),
        DeclareLaunchArgument("topic", default_value="",
                              description="Empty uses <namespace>/state."),
        DeclareLaunchArgument("socket_path", default_value="",
                              description="Empty derives a hand/topic-namespace-specific private socket."),
        Node(package="vt_tactile_capture", executable="vt_tactile_display_bridge",
             name="tactile_display_bridge", namespace=LaunchConfiguration("namespace"), output="screen", parameters=[{
                 "topic": ParameterValue(LaunchConfiguration("topic"), value_type=str),
                 "socket_path": ParameterValue(LaunchConfiguration("socket_path"), value_type=str),
                 "hand": ParameterValue(LaunchConfiguration("hand"), value_type=str),
                 "use_sim_time": False,
             }]),
    ])
