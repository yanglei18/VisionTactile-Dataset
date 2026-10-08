import sys
import importlib.util
from pathlib import Path
import pytest

from vt_glove_bridge.launch_support import gui_command, gui_environment


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
