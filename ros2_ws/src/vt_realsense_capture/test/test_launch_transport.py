"""Direct camera launch applies the same transport default before any child."""
import importlib.util
import os
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, GroupAction


ROOT = Path(__file__).parents[1]


def load_launch():
    spec = importlib.util.spec_from_file_location("triple_realsense_transport", ROOT / "launch/triple_realsense.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("custom", [False, True])
def test_transport_delta_precedes_camera_includes(tmp_path, monkeypatch, custom):
    monkeypatch.chdir(tmp_path)
    module = load_launch()
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(ROOT))
    # LaunchContext.environment exposes os.environ, not a context-owned copy.
    # Replace the mapping for this test instead of clearing the test process.
    monkeypatch.setattr(os, "environ", {"UNCHANGED": "preserve"})
    context = LaunchContext()
    if custom:
        context.environment["FASTRTPS_DEFAULT_PROFILES_FILE"] = "/operator/profile.xml"
    context.launch_configurations.update(config_path=str(ROOT / "config/cameras.yaml"), output_root="")
    before = dict(context.environment)
    actions = module._launch_setup(context)
    first_camera = next(i for i, item in enumerate(actions) if isinstance(item, TimerAction))
    changes = [i for i, item in enumerate(actions) if isinstance(item, SetEnvironmentVariable)]
    assert len(changes) == (0 if custom else 1)
    assert all(index < first_camera for index in changes)
    assert context.environment == before  # Launch actions, not global os.environ mutation.
    for index in changes:
        actions[index].execute(context)
    assert context.environment["UNCHANGED"] == "preserve"
    if custom:
        assert context.environment == before
    else:
        assert Path(context.environment["FASTRTPS_DEFAULT_PROFILES_FILE"]).is_file()


def test_capture_declares_transport_package_dependency():
    assert "<exec_depend>vt_capture_core</exec_depend>" in (ROOT / "package.xml").read_text()


def test_camera_launches_are_staggered_and_scoped(monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(ROOT))
    context = LaunchContext()
    context.launch_configurations.update(config_path=str(ROOT / "config/cameras.yaml"), output_root="")
    timers = [a for a in module._launch_setup(context) if isinstance(a, TimerAction)]
    assert len(timers) == 3
    for index, timer in enumerate(timers):
        assert timer.period == float(index * 3)
        assert len(timer.actions) == 1
        assert isinstance(timer.actions[0], GroupAction)
        assert any(isinstance(a, IncludeLaunchDescription) for a in timer.actions[0].get_sub_entities())
