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
            "fsglove_source": "", "mano_assets": "", "config": "", "rest": "", "port": "",
            "socket_path": "", "output_root": str(Path.home() / "visiontactile-data" / "glove_sessions"),
            "output_rate_hz": ""}


def _resource_arguments(values):
    """Resolve one complete resource selection before constructing a process.

    Legacy profiles may carry paths already inside their selected bundle. They
    are accepted only when every nonempty path identifies that exact bundle
    member. Different installations must never be silently combined.
    """
    members = {"config": "config.json", "rest": "rest.json",
               "mano_assets": "mano", "fsglove_source": "upstream"}
    if values["resources"]:
        root = Path(values["resources"]).expanduser().resolve(strict=True)
        for key, relative in members.items():
            expected = root / relative
            if values[key] and Path(values[key]).expanduser().resolve(strict=True) != expected.resolve(strict=True):
                raise ValueError(f"resource bundle conflicts with explicit {key}")
        # The result-only ROS bridge uses the same installation map as the
        # worker. The worker receives the bundle alone and validates its hashes.
        values["config"] = str(root / "config.json")
        return ["--resources", str(root)]
    missing = [key for key in members if not values[key]]
    if missing:
        raise ValueError("managed hand explicit resources require " + ", ".join(missing)
                         + "; select a complete hand resource bundle or provide all four paths")
    arguments = []
    for key in members:
        path = Path(values[key]).expanduser().resolve(strict=True)
        expected_type = path.is_file() if key in {"config", "rest"} else path.is_dir()
        if not expected_type:
            raise ValueError(f"invalid {key} resource path")
        values[key] = str(path)
        arguments += ["--" + key.replace("_", "-"), str(path)]
    return arguments


def _start(context):
    from vt_capture_core.hand_paths import default_hand_control_socket
    values = {key: LaunchConfiguration(key, default=value).perform(context) for key, value in DEFAULTS.items()}
    resource_arguments = _resource_arguments(values)
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
    command += resource_arguments
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
