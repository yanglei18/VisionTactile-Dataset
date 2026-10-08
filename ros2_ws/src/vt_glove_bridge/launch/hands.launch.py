"""Compose any subset of left/right pose and tactile without automatic recording."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from vt_glove_bridge.hands import load_hands, hand_plan, RUNTIME_DEFAULTS, SIDES
from vt_glove_bridge.launch_support import gui_command


OVERRIDES = [*RUNTIME_DEFAULTS, "pose_hands", "tactile_hands",
             *(f"{side}_{field}" for side in SIDES for field in
               ("imu_port", "tactile_port", "pose_config", "tactile_layout"))]


def _description(key):
    descriptions = {
        "pose_hands": "Override enabled pose hands: none, left, right, both; empty keeps YAML",
        "tactile_hands": "Override enabled tactile hands independently; empty keeps YAML",
        "glove_python": "Pose GUI Python executable; unused in tactile-only mode",
        "fsglove_source": "Trusted official FSGlove checkout; unused in tactile-only mode",
        "mano_assets": "Directory containing models/MANO_LEFT.pkl and/or MANO_RIGHT.pkl",
        "tactile_gui_python": "Tactile viewer Python with PyQt5, pyqtgraph and ROS imports",
        "output_root": "Parent of separate left/right local pose session directories, not ROS bags",
    }
    if key in descriptions:
        return descriptions[key]
    side, field = key.split("_", 1)
    return f"{side}: " + {
        "imu_port": "explicit IMU device prefill; manual connection in this hand's GUI",
        "tactile_port": "explicit tactile serial device; never share with another reader",
        "pose_config": "per-hand IMU mapping/solver JSON; must match the selected hand",
        "tactile_layout": "explicit anatomical layout JSON; left defaults to matrix without one",
    }[field]


def _start(context):
    profile = LaunchConfiguration("config").perform(context)
    overrides = {key: LaunchConfiguration(key).perform(context) for key in OVERRIDES}
    plan = hand_plan(load_hands(profile, overrides=overrides))
    # Preflight every enabled GUI before returning any action. Tactile-only
    # never resolves a MANO asset or imports the pose implementation.
    for entry in plan:
        pose = entry["pose"]
        if pose is not None and pose["launch_gui"] == "true":
            keys = ("glove_python", "fsglove_source", "mano_assets", "socket_path",
                    "output_root", "config", "port", "output_rate_hz", "hand")
            options = {key: pose[key] for key in keys}
            if pose["tactile_overlay"] == "true":
                options.update(tactile_socket=pose["tactile_socket"], tactile_layout=pose["tactile_layout"])
            gui_command(**options)
    actions = []
    topics = []
    for entry in plan:
        for key, package, launch in (
            ("tactile", "vt_tactile_capture", "tactile.launch.py"),
            ("pose", "vt_glove_bridge", "glove.launch.py"),
        ):
            options = entry[key]
            if options is None:
                continue
            source = Path(get_package_share_directory(package)) / "launch" / launch
            # Do not leak the parent hands-profile `config` into the child's
            # ROS-parameter `config`, or pose options into the next tactile node.
            actions.append(GroupAction(scoped=True, forwarding=False, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(source)), launch_arguments=options.items())]))
        topics.extend(entry["record_topics"])
        if entry["hand"] == "left" and entry["tactile"] and not entry["tactile"]["hand_layout_path"]:
            actions.append(LogInfo(msg="Left tactile: raw matrix view; supply a left layout to enable anatomical/3D mapping."))
    actions.insert(0, LogInfo(msg="Enabled recording topics (not automatically recorded): " + ", ".join(topics)))
    if not plan:
        actions.append(LogInfo(msg="All hand modalities are disabled; no hardware processes started."))
    return actions


def generate_launch_description():
    config = Path(get_package_share_directory("vt_glove_bridge")) / "config" / "hands.example.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(config),
                              description="Hands profile YAML; independent modality flags and per-hand settings"),
        *[DeclareLaunchArgument(key, default_value="", description=_description(key)) for key in OVERRIDES],
        OpaqueFunction(function=_start),
    ])
