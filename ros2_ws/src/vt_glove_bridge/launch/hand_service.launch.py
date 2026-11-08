"""One managed headless hand producer and its existing result-only ROS bridge."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnExecutionComplete
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vt_glove_bridge.launch_support import gui_environment, pose_options


DEFAULTS = {"hand": "right", "glove_python": "", "resources": "", "control_socket": "",
            "fsglove_source": "", "mano_assets": "", "config": "", "port": "",
            "socket_path": "", "output_root": str(Path.home() / "visiontactile-data" / "glove_sessions"),
            "output_rate_hz": ""}


def _start(context):
    from vt_capture_core.hand_paths import default_hand_control_socket
    values = {key: LaunchConfiguration(key, default=value).perform(context) for key, value in DEFAULTS.items()}
    if values["resources"]:
        resource_config = Path(values["resources"]).expanduser() / "config.json"
        if values["config"] and Path(values["config"]).expanduser().resolve() != resource_config.resolve():
            raise ValueError("resource bundle and explicit configuration must identify the same file")
        values["config"] = str(resource_config)
    pose = pose_options(values)
    hand = pose["hand"]
    python = Path(values["glove_python"]).expanduser()
    if not python.is_absolute() or not python.is_file() or not os.access(python, os.X_OK):
        raise ValueError("glove_python must identify the prepared numerical interpreter")
    control = values["control_socket"] or str(default_hand_control_socket(hand))
    if Path(control).resolve() == Path(pose["socket_path"]).resolve():
        raise ValueError("hand control and pose result sockets must differ")
    command = [str(python), "-I", "-m", "vt_fsglove12", "hand-service", "--hand", hand,
               "--control-socket", control, "--ros-socket", pose["socket_path"],
               "--output-root", values["output_root"]]
    if values["resources"]:
        command += ["--resources", values["resources"]]
    else:
        for key in ("fsglove_source", "mano_assets", "config"):
            if not values[key]:
                raise ValueError(f"managed hand requires {key} or a resource bundle")
            command += ["--" + key.replace("_", "-"), values[key]]
    if values["port"]:
        command += ["--port", values["port"]]
    if values["output_rate_hz"]:
        command += ["--output-rate-hz", values["output_rate_hz"]]
    service = ExecuteProcess(cmd=command, env=gui_environment(), name=f"hand_{hand}_service", output="screen")
    bridge = Node(package="vt_glove_bridge", executable="vt_glove_bridge_node", name="vt_glove_bridge",
        namespace=pose["namespace"], output="screen", parameters=[{
            "hand": hand, "reference_node_id": pose["reference_node_id"],
            "socket_path": pose["socket_path"], "topic": pose["topic"], "use_sim_time": False}])
    processes = (service, bridge)
    lifecycle = [RegisterEventHandler(OnExecutionComplete(target_action=owner,
        on_completion=[EmitEvent(event=ShutdownProcess(process_matcher=matches_action(peer)))
                       for peer in processes if peer is not owner])) for owner in processes]
    return [*lifecycle, *processes]


def generate_launch_description():
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k, v in DEFAULTS.items()],
                              OpaqueFunction(function=_start)])
