from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_PATH = (
    PACKAGE_ROOT / "launch" / "tracker_visualization.launch.py"
)
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "triple_tracker.rviz"


def launch_description():
    spec = importlib.util.spec_from_file_location(
        "tracker_visualization_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_starts_only_visualizer_and_rviz():
    description = launch_description()
    nodes = [
        entity
        for entity in description.entities
        if isinstance(entity, Node)
    ]

    assert [
        (
            vars(node)["_Node__package"],
            vars(node)["_Node__node_executable"],
        )
        for node in nodes
    ] == [
        ("vt_vive_tracker", "vt_vive_tracker_visualizer"),
        ("rviz2", "rviz2"),
    ]


def test_launch_declares_only_optional_rviz_config():
    description = launch_description()
    arguments = [
        entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert [
        vars(argument)["_DeclareLaunchArgument__name"]
        for argument in arguments
    ] == ["rviz_config"]
    assert vars(arguments[0])["_DeclareLaunchArgument__default_value"]


def test_rviz_config_uses_vive_map_grid_and_exact_marker_topic():
    config = yaml.safe_load(RVIZ_PATH.read_text(encoding="utf-8"))
    manager = config["Visualization Manager"]
    displays = manager["Displays"]

    assert manager["Global Options"]["Fixed Frame"] == "vive_map"
    assert any(
        display["Class"] == "rviz_default_plugins/Grid"
        and display["Enabled"] is True
        for display in displays
    )
    marker_display = next(
        display
        for display in displays
        if display["Class"] == "rviz_default_plugins/MarkerArray"
    )
    assert marker_display["Enabled"] is True
    assert marker_display["Topic"]["Value"] == (
        "/vive/visualization/markers"
    )
    assert marker_display["Queue Size"] >= 10


def test_setup_installs_rviz_and_manifest_declares_runtime():
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    manifest = (PACKAGE_ROOT / "package.xml").read_text(
        encoding="utf-8"
    )

    assert 'glob("rviz/*.rviz")' in setup
    assert '<exec_depend>rviz2</exec_depend>' in manifest
