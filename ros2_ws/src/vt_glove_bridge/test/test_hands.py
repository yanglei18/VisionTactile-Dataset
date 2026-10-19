import importlib.util
import itertools
import json
from pathlib import Path

import pytest
import yaml

from vt_glove_bridge.hands import load_hands, hand_plan, selected_hands
from vt_glove_bridge.launch_support import pose_options, gui_command
from vt_glove_bridge.transport import default_socket_path


@pytest.mark.parametrize("pose,tactile", itertools.product(("none", "left", "right", "both"), repeat=2))
def test_all_modality_combinations_are_independent(pose, tactile):
    config = load_hands(overrides={"pose_hands": pose, "tactile_hands": tactile})
    plan = hand_plan(config)
    pose_sides, tactile_sides = selected_hands(pose), selected_hands(tactile)
    assert {item["hand"] for item in plan} == pose_sides | tactile_sides
    topics, sockets = [], []
    for item in plan:
        side = item["hand"]
        assert (item["pose"] is not None) == (side in pose_sides)
        assert (item["tactile"] is not None) == (side in tactile_sides)
        topics.extend(item["record_topics"])
        if item["pose"]:
            sockets.append(item["pose"]["socket_path"])
            assert item["pose"]["hand"] == side
            assert item["pose"]["topic"] == f"/gloves/{side}/state"
        if item["tactile"]:
            assert item["tactile"]["hand"] == side
            assert item["tactile"]["namespace"] == f"/gloves/{side}/tactile"
    assert len(topics) == len(set(topics)) == len(pose_sides) + len(tactile_sides)
    assert len(sockets) == len(set(sockets))


def test_left_without_anatomical_layout_defaults_to_raw_matrix_not_fake_overlay():
    plan = hand_plan(load_hands(overrides={"pose_hands": "both", "tactile_hands": "both"}))
    by_hand = {entry["hand"]: entry for entry in plan}
    assert by_hand["left"]["tactile"]["initial_view"] == "matrix"
    assert by_hand["left"]["pose"]["tactile_overlay"] == "false"
    assert by_hand["right"]["pose"]["tactile_overlay"] == "true"


def test_profile_relative_paths_and_explicit_overrides(tmp_path):
    config = tmp_path / "hands.yaml"
    config.write_text(yaml.safe_dump({"schema_version": 1, "runtime": {"output_root": "sessions"},
        "hands": {"left": {"tactile_enabled": True, "tactile_layout": "layout.json"}}}))
    loaded = load_hands(config, overrides={"pose_hands": "none", "tactile_hands": "both"})
    assert loaded["runtime"]["output_root"] == str(tmp_path / "sessions")
    assert loaded["hands"]["left"]["tactile_layout"] == str(tmp_path / "layout.json")
    assert len(hand_plan(loaded)) == 2


def test_reject_same_serial_port_even_across_modalities_and_aliases(tmp_path):
    device = tmp_path / "device"
    device.touch()
    alias = tmp_path / "alias"
    alias.symlink_to(device)
    with pytest.raises(ValueError, match="serial device reused"):
        load_hands(overrides={"pose_hands": "right", "tactile_hands": "left",
                             "right_imu_port": str(device), "left_tactile_port": str(alias)})
    # Disabled left pose must not reserve its configured device.
    load_hands(overrides={"pose_hands": "right", "tactile_hands": "none",
                         "right_imu_port": str(device), "left_imu_port": str(alias)})


@pytest.mark.parametrize("document", [
    {"schema_version": 2}, {"schema_version": True},
    {"schema_version": 1, "hands": {"middle": {}}},
    {"schema_version": 1, "hands": {"right": {"pose_enabled": "false"}}},
    {"schema_version": 1, "hands": {"right": {"output_rate_hz": float("nan")}}},
    {"schema_version": 1, "runtime": {"invented": "path"}},
])
def test_invalid_profile_is_rejected_before_any_process(tmp_path, document):
    path = tmp_path / "hands.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_hands(path)


def test_hand_config_derives_model_side_and_palm_without_importing_solver(tmp_path):
    path = tmp_path / "left.json"
    path.write_text(json.dumps({"hand": "left", "node_to_slot": {"11": 0, "1": 1}}))
    values = pose_options({"config": str(path)})
    assert values["hand"] == "left" and values["reference_node_id"] == 11
    assert values["socket_path"] == default_socket_path("left")
    assert values["topic"] == "/gloves/left/state"
    with pytest.raises(ValueError, match="does not match"):
        pose_options({"hand": "right", "config": str(path)})
    with pytest.raises(ValueError, match="conflicts"):
        pose_options({"hand": "left", "topic": "/gloves/right/state"})


def test_gui_command_passes_hand_and_optional_anatomical_layout(tmp_path):
    import sys
    command = gui_command(glove_python=sys.executable, fsglove_source=str(tmp_path),
                          mano_assets=str(tmp_path), socket_path="/tmp/left.sock",
                          output_root=str(tmp_path), hand="left", tactile_layout=str(tmp_path / "layout.json"))
    assert command[command.index("--hand") + 1] == "left"
    assert command[command.index("--tactile-layout") + 1] == str(tmp_path / "layout.json")


def test_tactile_only_launch_does_not_require_pose_assets_or_start_pose_processes(tmp_path, monkeypatch):
    from launch import LaunchContext
    from launch.actions import GroupAction
    path = Path(__file__).parents[1] / "launch/hands.launch.py"
    spec = importlib.util.spec_from_file_location("test_hands_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(tmp_path / name))
    monkeypatch.setattr(module, "gui_command", lambda **kwargs: pytest.fail("tactile-only imported pose GUI"))
    context = LaunchContext()
    context.launch_configurations.update({key: "" for key in module.OVERRIDES})
    context.launch_configurations.update(config="", pose_hands="none", tactile_hands="both")
    actions = module._start(context)
    groups = [action for action in actions if isinstance(action, GroupAction)]
    assert len(groups) == 2


def test_existing_single_right_defaults_remain_right():
    values = pose_options({})
    assert values["hand"] == "right" and values["reference_node_id"] == 12
    assert values["topic"] == "/gloves/right/state"
    assert values["socket_path"] == default_socket_path()


def test_node_specific_tactile_ports_require_explicit_device_for_composition(tmp_path):
    config = tmp_path / "touch.yaml"
    config.write_text(yaml.safe_dump({
        "/gloves/left/tactile/tactile_capture": {"ros__parameters": {"port": "/dev/shared"}},
        "/gloves/right/tactile/tactile_capture": {"ros__parameters": {"port": "/dev/shared"}},
    }))
    profile = tmp_path / "hands.yaml"
    profile.write_text(yaml.safe_dump({"schema_version": 1, "hands": {
        "left": {"tactile_enabled": True, "tactile_config": str(config)},
        "right": {"tactile_enabled": True, "tactile_config": str(config)},
    }}))
    with pytest.raises(ValueError, match="explicit tactile_port"):
        load_hands(profile)
    selected = load_hands(profile, overrides={"left_tactile_port": "/dev/left_touch",
                                             "right_tactile_port": "/dev/right_touch"})
    assert len(hand_plan(selected)) == 2


def test_wildcard_tactile_yaml_device_participates_in_alias_check(tmp_path):
    config = tmp_path / "touch.yaml"
    config.write_text(yaml.safe_dump({"/**": {"ros__parameters": {"port": "/dev/shared"}}}))
    profile = tmp_path / "hands.yaml"
    profile.write_text(yaml.safe_dump({"schema_version": 1, "hands": {
        "left": {"tactile_enabled": True, "tactile_config": str(config)},
        "right": {"pose_enabled": True, "imu_port": "/dev/shared"},
    }}))
    with pytest.raises(ValueError, match="serial device reused"):
        load_hands(profile)


def test_included_tactile_launch_does_not_inherit_parent_config_or_hand(tmp_path, monkeypatch):
    """Expand includes and scoping only: never execute a Node/process action."""
    from launch import LaunchContext, LaunchDescription
    from launch.actions import ExecuteProcess, LogInfo
    from launch_ros.actions import Node

    packages = Path(__file__).resolve().parents[2]
    shares = {name: str(packages / name) for name in ("vt_glove_bridge", "vt_tactile_capture")}
    monkeypatch.setattr("ament_index_python.packages.get_package_share_directory", shares.__getitem__)
    path = packages / "vt_glove_bridge/launch/hands.launch.py"
    spec = importlib.util.spec_from_file_location("test_scoped_hands_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "gui_command", lambda **kwargs: pytest.fail("pose GUI must remain disabled"))
    profile = tmp_path / "hands.yaml"
    profile.write_text("schema_version: 1\nhands: {}\n")
    context = LaunchContext()
    context.launch_configurations.update({key: "" for key in module.OVERRIDES})
    context.launch_configurations.update(config=str(profile), pose_hands="none", tactile_hands="both",
                                         hand="parent_wrong_hand", namespace="/parent_wrong_namespace")
    parent_configurations = dict(context.launch_configurations)
    nodes = []

    def expand(entity):
        if isinstance(entity, Node):
            nodes.append({key: context.launch_configurations[key] for key in ("config", "hand", "namespace")})
            return  # Critical: construction is safe; Node.execute would start a process.
        if isinstance(entity, ExecuteProcess):
            pytest.fail("unexpected non-node process in tactile-only composition")
        if isinstance(entity, LogInfo):
            return
        children = entity.entities if isinstance(entity, LaunchDescription) else entity.execute(context)
        for child in children or []:
            expand(child)

    for action in module._start(context):
        expand(action)
    expected_config = str(packages / "vt_tactile_capture/config/right_tactile.yaml")
    assert nodes == [
        {"config": expected_config, "hand": hand, "namespace": f"/gloves/{hand}/tactile"}
        for hand in ("left", "right") for _ in range(2)
    ]
    assert context.launch_configurations == parent_configurations
