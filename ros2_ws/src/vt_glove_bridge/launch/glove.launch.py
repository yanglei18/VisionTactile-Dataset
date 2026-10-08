"""One independent hand, with backwards-compatible right-hand defaults."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vt_glove_bridge.launch_support import gui_command, gui_environment, pose_options


DEFAULTS = {
    "hand": "", "namespace": "", "launch_gui": "true",
    "glove_python": "", "fsglove_source": "", "mano_assets": "",
    "socket_path": "", "topic": "", "config": "", "port": "",
    "output_root": str(Path.home() / "visiontactile-data" / "glove_sessions"),
    "max_input_age_s": "0.15", "allow_replay": "false", "replay_raw": "",
    "replay_speed": "1.0", "output_rate_hz": "", "tactile_overlay": "false",
    "tactile_socket": "", "tactile_topic": "", "tactile_layout": "",
}

DESCRIPTIONS = {
    "hand": "left/right; empty reads config hand, otherwise defaults to right",
    "namespace": "Node namespace; empty derives /gloves/<hand>",
    "launch_gui": "Open the calibration GUI without automatically connecting hardware",
    "glove_python": "Python executable in the independent pose environment",
    "fsglove_source": "Trusted pinned official FSGlove checkout",
    "mano_assets": "User-provided MANO asset directory for the selected hand",
    "socket_path": "Pose-result Unix socket; empty derives the selected hand's path",
    "topic": "Solved-state output topic; empty derives /gloves/<hand>/state",
    "config": "Optional per-hand solver JSON; must match an explicitly selected hand",
    "port": "IMU serial path prefill only; connect manually in the GUI",
    "output_root": "Local session/debug directory, separate from ROS bags",
    "max_input_age_s": "Maximum source-input age before marking a received pose invalid",
    "allow_replay": "Explicit opt-in to replay-labelled packets, not live synchronization",
    "replay_raw": "Historical CSV input, never opens hardware",
    "replay_speed": "Historical CSV replay multiplier",
    "output_rate_hz": "Optional solve-rate target; no publication-frequency guarantee",
    "tactile_overlay": "Start a read-only display bridge; does not start tactile serial acquisition",
    "tactile_socket": "Independent ROS-to-GUI tactile socket; defaults by hand",
    "tactile_topic": "Existing TactileState topic; defaults by hand",
    "tactile_layout": "Explicit per-hand surface layout JSON; required for left 3D tactile overlay",
}


def _options(context):
    return pose_options({key: LaunchConfiguration(key, default=value).perform(context)
                         for key, value in DEFAULTS.items()})


def _enabled(value, name):
    if value.lower() not in ("true", "false"):
        raise ValueError(f"{name} must be true or false")
    return value.lower() == "true"


def _start_gui(context):
    values = _options(context)
    if not _enabled(values["launch_gui"], "launch_gui"):
        return []
    names = ("glove_python", "fsglove_source", "mano_assets", "socket_path", "hand",
             "output_root", "config", "port", "replay_raw", "replay_speed", "output_rate_hz")
    options = {name: values[name] for name in names}
    if _enabled(values["tactile_overlay"], "tactile_overlay"):
        options.update(tactile_socket=values["tactile_socket"], tactile_layout=values["tactile_layout"])
    return [ExecuteProcess(cmd=gui_command(**options), env=gui_environment(),
                           name=f"glove_{values['hand']}_calibration_gui", output="screen")]


def _start_tactile_display(context):
    values = _options(context)
    if not _enabled(values["tactile_overlay"], "tactile_overlay"):
        return []
    if values["replay_raw"]:
        raise ValueError("Live tactile overlay cannot be combined with historical pose replay")
    if Path(values["tactile_socket"]).expanduser().resolve() == Path(values["socket_path"]).expanduser().resolve():
        raise ValueError("tactile_socket must differ from the pose result socket_path")
    return [Node(package="vt_tactile_capture", executable="vt_tactile_display_bridge",
                 name="vt_tactile_display_bridge", namespace=values["namespace"],
                 output="screen", parameters=[{
                     "socket_path": values["tactile_socket"], "topic": values["tactile_topic"],
                     "hand": values["hand"], "use_sim_time": False,
                 }])]


def _start(context):
    values = _options(context)
    # Validate GUI settings before returning any process action.
    gui = _start_gui(context)
    display = _start_tactile_display(context)
    bridge = Node(package="vt_glove_bridge", executable="vt_glove_bridge_node",
                  name="vt_glove_bridge", namespace=values["namespace"], output="screen",
                  parameters=[{
                      "hand": values["hand"], "reference_node_id": values["reference_node_id"],
                      "socket_path": values["socket_path"], "topic": values["topic"],
                      "max_input_age_s": float(values["max_input_age_s"]),
                      "allow_replay": _enabled(values["allow_replay"], "allow_replay"),
                      "use_sim_time": False,
                  }])
    return [bridge, *display, *gui]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value, description=DESCRIPTIONS[name])
          for name, value in DEFAULTS.items()],
        OpaqueFunction(function=_start),
    ])
