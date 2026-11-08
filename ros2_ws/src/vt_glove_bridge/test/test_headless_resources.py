"""Resource selection is complete before ROS creates any owned processes."""
import importlib.util
import json
from pathlib import Path

import pytest

pytest.importorskip("launch")
pytest.importorskip("launch_ros")


@pytest.fixture
def launcher():
    path = Path(__file__).parents[1] / "launch/hand_service.launch.py"
    spec = importlib.util.spec_from_file_location("tested_hand_service_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def selection(tmp_path, launcher):
    (tmp_path / "config.json").write_text(json.dumps({"hand": "right", "node_to_slot": {"12": 0}}))
    (tmp_path / "rest.json").write_text("{}")
    (tmp_path / "mano").mkdir()
    (tmp_path / "upstream").mkdir()
    return dict(launcher.DEFAULTS, config=str(tmp_path / "config.json"),
                rest=str(tmp_path / "rest.json"), mano_assets=str(tmp_path / "mano"),
                fsglove_source=str(tmp_path / "upstream"))


@pytest.mark.parametrize("key", ["config", "rest", "mano_assets", "fsglove_source"])
def test_missing_explicit_member_is_rejected_without_guessing_adjacent_files(launcher, selection, key):
    selection[key] = ""
    with pytest.raises(ValueError, match=key):
        launcher._resource_arguments(selection)


def test_explicit_worker_arguments_include_rest(launcher, selection):
    result = launcher._resource_arguments(selection)
    for key in ("config", "rest", "mano_assets", "fsglove_source"):
        assert result[result.index("--" + key.replace("_", "-")) + 1] == selection[key]
    assert "--resources" not in result


@pytest.mark.parametrize("key", ["config", "rest", "mano_assets", "fsglove_source"])
def test_bundle_cannot_silently_override_other_installation(launcher, selection, tmp_path, key):
    selection["resources"] = str(tmp_path)
    unrelated = tmp_path / "unrelated"
    unrelated.touch()
    selection[key] = str(unrelated)
    with pytest.raises(ValueError, match="conflicts with explicit " + key):
        launcher._resource_arguments(selection)


def test_matching_legacy_fields_resolve_to_bundle_only(launcher, selection, tmp_path):
    selection["resources"] = str(tmp_path)
    assert launcher._resource_arguments(selection) == ["--resources", str(tmp_path)]
    assert selection["config"] == str(tmp_path / "config.json")


@pytest.mark.parametrize("key", ["config", "rest", "mano_assets", "fsglove_source"])
def test_missing_explicit_file_or_directory_is_rejected(launcher, selection, tmp_path, key):
    selection[key] = str(tmp_path / "absent")
    with pytest.raises(FileNotFoundError):
        launcher._resource_arguments(selection)


@pytest.mark.parametrize("mode", ["bundle", "explicit"])
def test_wrong_installation_hand_fails_before_process_construction(launcher, selection, tmp_path, monkeypatch, mode):
    from launch import LaunchContext
    selection["hand"] = "left"
    if mode == "bundle":
        selection["resources"] = str(tmp_path)
    context = LaunchContext()
    context.launch_configurations.update(selection)
    monkeypatch.setattr(launcher, "ExecuteProcess", lambda **_: pytest.fail("No worker may be constructed"))
    with pytest.raises(ValueError, match="hand does not match"):
        launcher._start(context)
