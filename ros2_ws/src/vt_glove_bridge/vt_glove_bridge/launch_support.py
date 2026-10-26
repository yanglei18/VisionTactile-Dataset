"""Build the independent GUI process without importing its optional dependencies."""
from __future__ import annotations

import os
import json
from pathlib import Path

from .transport import default_socket_path


def pose_options(options):
    """Resolve one hand without importing FSGlove, Qt, torch, or ROS.

    Only the producer selects its default solver configuration. Without an
    explicit config, use the node's packet-defined reference mode: the protocol
    validates the producer's palm ID and its source timestamps. An explicit
    config additionally pins that ID for this bridge instance.
    """
    values = dict(options)
    requested = values.get("hand", "")
    config = None
    if values.get("config"):
        location = Path(values["config"]).expanduser().resolve()
        config = json.loads(location.read_text(encoding="utf-8"))
        if not isinstance(config, dict):
            raise ValueError("glove config must be an object")
        values["config"] = str(location)
    hand = requested or (config.get("hand") if config is not None else "right")
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    if config is not None and config.get("hand") != hand:
        raise ValueError("hand does not match glove config")
    reference = 0  # GloveBridge's packet-defined mode; never a physical IMU ID.
    if config is not None:
        mapping = config.get("node_to_slot", {})
        if not isinstance(mapping, dict):
            raise ValueError("glove config node_to_slot must be an object")
        candidates = [key for key, slot in mapping.items() if type(slot) is int and slot == 0]
        if len(candidates) != 1 or candidates[0] not in {str(i) for i in range(1, 13)}:
            raise ValueError("glove config must identify exactly one palm node in 1..12")
        reference = int(candidates[0])
    values["hand"] = hand
    values["reference_node_id"] = reference
    for key, default in {
        "topic": f"/gloves/{hand}/state",
        "namespace": f"/gloves/{hand}",
        "socket_path": default_socket_path(hand),
        "tactile_socket": f"/tmp/vt-tactile-{os.getuid()}/{hand}.sock",
        "tactile_topic": f"/gloves/{hand}/tactile/state",
    }.items():
        values[key] = values.get(key) or default
    other = "left" if hand == "right" else "right"
    for key in ("topic", "tactile_topic"):
        if values[key].startswith(f"/gloves/{other}/"):
            raise ValueError(f"{key} namespace conflicts with hand")
    return values


def gui_command(*, glove_python, fsglove_source, mano_assets, socket_path,
                output_root, config="", port="", replay_raw="", replay_speed="1.0", output_rate_hz="",
                tactile_socket="", hand="right", tactile_layout=""):
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    for name, value in (("glove_python", glove_python), ("fsglove_source", fsglove_source),
                        ("mano_assets", mano_assets)):
        if not value or not Path(value).expanduser().is_absolute():
            raise ValueError(f"{name} is required and must be an absolute path when launch_gui=true")
    interpreter = Path(glove_python).expanduser()
    if not interpreter.is_file() or not os.access(interpreter, os.X_OK):
        raise ValueError("glove_python must be an executable Python in the glove virtual environment")
    for name, value in (("fsglove_source", fsglove_source), ("mano_assets", mano_assets)):
        if not Path(value).expanduser().is_dir():
            raise ValueError(f"{name} directory does not exist")
    command = [str(interpreter), "-I", "-m", "vt_fsglove12", "upstream-live",
               "--fsglove-source", str(Path(fsglove_source).expanduser()),
               "--mano-assets", str(Path(mano_assets).expanduser()),
               "--ros-socket", socket_path, "--output-root", output_root,
               "--hand", hand]
    for option, value in (("--config", config), ("--port", port), ("--replay-raw", replay_raw)):
        if value:
            command += [option, value]
    if replay_raw:
        command += ["--replay-speed", replay_speed]
    if output_rate_hz:
        command += ["--output-rate-hz", str(output_rate_hz)]
    if tactile_socket:
        if Path(tactile_socket).expanduser().absolute() == Path(socket_path).expanduser().absolute():
            raise ValueError("tactile_socket must differ from the pose result socket_path")
        if replay_raw:
            raise ValueError("Live tactile overlay cannot be combined with historical pose replay")
        command += ["--tactile-socket", str(tactile_socket)]
    if tactile_layout:
        command += ["--tactile-layout", str(Path(tactile_layout).expanduser().resolve())]
    # No auto-connect/auto-calibrate options: those actions remain in the GUI.
    return command


def gui_environment(environment=None):
    env = dict(os.environ if environment is None else environment)
    # Tiny matrix inference must not spawn a large BLAS pool per operation.
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        env.setdefault(key, "1")
    # -I also ignores Python env injection. Remove ROS search paths from GUI/worker
    # child processes while retaining DISPLAY, XAUTHORITY, Wayland/XDG and Qt platform.
    for key in ("PYTHONPATH", "PYTHONHOME", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH",
                "CMAKE_PREFIX_PATH"):
        env.pop(key, None)
    if "LD_LIBRARY_PATH" in env:
        env["LD_LIBRARY_PATH"] = ":".join(
            part for part in env["LD_LIBRARY_PATH"].split(":")
            if part and "/opt/ros/" not in part and "/ros2_ws/install/" not in part)
    return env
