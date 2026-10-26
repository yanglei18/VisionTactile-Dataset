import sys
import importlib.util
import json
from pathlib import Path
import subprocess
import pytest

from vt_glove_bridge.launch_support import gui_command, gui_environment, pose_options
from vt_glove_bridge.protocol import decode_state
from test_protocol import packet


@pytest.mark.parametrize("hand", ["left", "right"])
@pytest.mark.parametrize("palm", [11, 12])
@pytest.mark.parametrize("launch_gui", ["false", "true"])
def test_no_config_consumes_the_producers_effective_mapping(hand, palm, launch_gui):
    """A producer's default may change without a second ROS-side mapping edit."""
    values = pose_options({"hand": hand, "launch_gui": launch_gui})
    data = packet(hand=hand, frame_id=f"glove_{hand}_reference",
                  reference_node_id=palm, source_node_ids=[5, palm])
    state = decode_state(data, expected_hand=values["hand"],
                         reference_node_id=values["reference_node_id"] or None)
    assert state["reference_node_id"] == palm
    assert state["host_monotonic_ns"] == state["source_monotonic_ns"][1]
    assert state["hand"] == hand


@pytest.mark.parametrize("change,reason", [
    ({"reference_node_id": 11}, "valid time requires the palm source"),
    ({"source_monotonic_ns": [9_997_000_000, 9_999_999_999]}, "reference stamp"),
    ({"reference_node_id": 13, "source_node_ids": [5, 13]}, "source node IDs"),
    ({"hand": "left", "frame_id": "glove_left_reference"}, "bridge instance"),
])
def test_packet_defined_reference_still_checks_identity_and_source_consistency(change, reason):
    values = pose_options({})
    with pytest.raises(ValueError, match=reason):
        decode_state(packet(**change), expected_hand=values["hand"],
                     reference_node_id=values["reference_node_id"] or None)


def test_explicit_new_layout_pins_both_gui_config_and_bridge_reference(tmp_path):
    path = tmp_path / "right-distal.json"
    path.write_text(json.dumps({"hand": "right", "node_to_slot": {"1": 15, "11": 0}}))
    values = pose_options({"config": str(path)})
    command = gui_command(glove_python=sys.executable, fsglove_source=str(tmp_path),
                          mano_assets=str(tmp_path), output_root=str(tmp_path),
                          **{key: values[key] for key in ("config", "hand", "socket_path")})
    assert command[command.index("--config") + 1] == str(path.resolve())
    state = decode_state(packet(reference_node_id=11, source_node_ids=[5, 11]),
                         expected_hand=values["hand"], reference_node_id=values["reference_node_id"])
    assert state["reference_node_id"] == 11
    with pytest.raises(ValueError, match="configured mapping"):
        decode_state(packet(), expected_hand=values["hand"], reference_node_id=values["reference_node_id"])
    with pytest.raises(ValueError, match="does not match"):
        pose_options({"hand": "left", "config": str(path)})


@pytest.mark.parametrize("rate_override", ["", "30"])
def test_new_glove_config_shares_palm_and_inherits_rate_unless_explicitly_overridden(tmp_path, rate_override):
    path = (Path(__file__).resolve().parents[4] / "tools/fsglove12/src/vt_fsglove12/configs/right_new_imu.json")
    config = json.loads(path.read_text(encoding="utf-8"))
    values = pose_options({"hand": "right", "config": str(path)})
    command = gui_command(glove_python=sys.executable, fsglove_source=str(tmp_path),
                          mano_assets=str(tmp_path), output_root=str(tmp_path),
                          output_rate_hz=rate_override,
                          **{key: values[key] for key in ("config", "hand", "socket_path")})
    assert values["reference_node_id"] == 11
    assert config["node_to_slot"]["11"] == 0
    assert command[command.index("--config") + 1] == str(path)
    assert config["sample_rate_hz"] == config["output_rate_hz"] == 104
    if rate_override:
        assert command[command.index("--output-rate-hz") + 1] == rate_override
    else:
        assert "--output-rate-hz" not in command


def test_missing_explicit_config_fails_instead_of_falling_back(tmp_path):
    with pytest.raises(FileNotFoundError):
        pose_options({"config": str(tmp_path / "missing.json"), "launch_gui": "false"})


def test_default_launch_resolution_does_not_import_solver_or_numeric_dependencies():
    source = Path(__file__).resolve().parents[1]
    script = """
import sys
sys.path.insert(0, sys.argv[1])
from vt_glove_bridge.launch_support import pose_options
assert pose_options({})['reference_node_id'] == 0
for name in ('vt_fsglove12', 'torch', 'numpy', 'scipy', 'PyQt5', 'PySide6', 'rclpy'):
    assert name not in sys.modules, name
"""
    subprocess.run([sys.executable, "-I", "-c", script, str(source)], check=True)


def test_command_keeps_gui_in_own_python_without_auto_connection(tmp_path):
    command = gui_command(glove_python=sys.executable, fsglove_source=str(tmp_path),
                          mano_assets=str(tmp_path), socket_path="/tmp/private/g.sock",
                          output_root=str(tmp_path / "out"), port="/dev/serial/by-id/example")
    assert command[:5] == [sys.executable, "-I", "-m", "vt_fsglove12", "upstream-live"]
    assert "--ros-socket" in command
    assert "--port" in command
    assert not any("connect" in option or "calibrat" in option for option in command)


def test_environment_preserves_desktop_but_removes_ros_python():
    env = gui_environment({"DISPLAY": ":1", "XAUTHORITY": "/tmp/auth", "XDG_RUNTIME_DIR": "/run/user/1000",
                           "QT_QPA_PLATFORM": "xcb", "PYTHONPATH": "/opt/ros/jazzy/lib/python3.12/site-packages",
                           "LD_LIBRARY_PATH": "/opt/ros/jazzy/lib:/usr/local/cuda/lib64",
                           "AMENT_PREFIX_PATH": "/opt/ros/jazzy"})
    assert "PYTHONPATH" not in env and "AMENT_PREFIX_PATH" not in env
    assert env["DISPLAY"] == ":1" and env["QT_QPA_PLATFORM"] == "xcb"
    assert env["LD_LIBRARY_PATH"] == "/usr/local/cuda/lib64"
    assert env["OMP_NUM_THREADS"] == env["OPENBLAS_NUM_THREADS"] == env["MKL_NUM_THREADS"] == "1"


def test_command_explicit_high_rate_is_a_solver_option_not_bridge_repetition(tmp_path):
    command = gui_command(glove_python=sys.executable, fsglove_source=str(tmp_path),
                          mano_assets=str(tmp_path), socket_path="/tmp/private/g.sock",
                          output_root=str(tmp_path / "out"), output_rate_hz="120")
    assert command[-2:] == ["--output-rate-hz", "120"]


def test_tactile_overlay_uses_separate_readonly_socket(tmp_path):
    options = dict(glove_python=sys.executable, fsglove_source=str(tmp_path),
                   mano_assets=str(tmp_path), socket_path="/tmp/private/g.sock",
                   output_root=str(tmp_path), tactile_socket="/tmp/private/touch.sock")
    command = gui_command(**options)
    assert command[-2:] == ["--tactile-socket", "/tmp/private/touch.sock"]
    assert command[command.index("--ros-socket") + 1] == "/tmp/private/g.sock"
    assert "--tactile-socket" not in gui_command(**{**options, "tactile_socket": ""})
    with pytest.raises(ValueError, match="historical pose replay"):
        gui_command(**options, replay_raw="/recorded/raw.csv")
    with pytest.raises(ValueError, match="must differ"):
        gui_command(**{**options, "tactile_socket": options["socket_path"]})


def test_launch_tactile_branch_only_starts_the_readonly_bridge():
    from launch import LaunchContext
    path = Path(__file__).parents[1] / "launch/glove.launch.py"
    spec = importlib.util.spec_from_file_location("glove_tactile_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(tactile_overlay="false")
    assert module._start_tactile_display(context) == []
    context.launch_configurations.update(tactile_overlay="true", replay_raw="",
        tactile_socket="/tmp/test-tactile/right.sock", socket_path="/tmp/test-pose/right.sock",
        tactile_topic="/gloves/right/tactile/state")
    actions = module._start_tactile_display(context)
    assert len(actions) == 1
    node = actions[0]
    assert node.node_package == "vt_tactile_capture"
    assert node.node_executable == "vt_tactile_display_bridge"
    context.launch_configurations["replay_raw"] = "/old/raw.csv"
    with pytest.raises(ValueError, match="historical pose replay"):
        module._start_tactile_display(context)
