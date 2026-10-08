"""The camera workbench is a subscriber, not a second hardware owner."""
import importlib.util
from pathlib import Path

import yaml
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


ROOT = Path(__file__).parents[1]


def config():
    return yaml.safe_load((ROOT / "rviz" / "head_camera.rviz").read_text())


def test_workbench_launch_only_starts_rviz():
    spec = importlib.util.spec_from_file_location(
        "camera_visualization_launch", ROOT / "launch" / "camera_visualization.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    nodes = [item for item in entities if isinstance(item, Node)]
    assert len(nodes) == 1
    assert vars(nodes[0])["_Node__package"] == "rviz2"
    assert vars(nodes[0])["_Node__node_executable"] == "rviz2"
    assert [vars(item)["_DeclareLaunchArgument__name"] for item in entities
            if isinstance(item, DeclareLaunchArgument)] == ["rviz_config"]


def test_images_and_cloud_use_latest_camera_topics_and_compatible_qos():
    manager = config()["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "d436_link"
    assert 1 <= manager["Global Options"]["Frame Rate"] <= 30
    inputs = [item for item in manager["Displays"]
              if item["Class"] in ("rviz_default_plugins/Image", "rviz_default_plugins/PointCloud2")]
    assert {item["Topic"]["Value"] for item in inputs} == {
        "/d436/color/image_raw", "/d436/depth/image_rect_raw", "/d436/depth/color/points"
    }
    for item in inputs:
        assert item["Enabled"] is True
        assert item["Topic"]["Reliability Policy"] == "Best Effort"
        assert item["Topic"]["Durability Policy"] == "Volatile"
        assert 1 <= item["Topic"]["Depth"] <= 5
    cloud = next(item for item in inputs if item["Class"] == "rviz_default_plugins/PointCloud2")
    assert cloud["Decay Time"] == 0
    assert cloud["Color Transformer"] == "RGB8"


def test_inspection_tools_do_not_publish_pose_or_goal_commands():
    tools = {item["Class"] for item in config()["Visualization Manager"]["Tools"]}
    assert "rviz_default_plugins/Measure" in tools
    assert "rviz_default_plugins/MoveCamera" in tools
    assert "rviz_default_plugins/SetInitialPose" not in tools
    assert "rviz_default_plugins/SetGoal" not in tools


def test_config_and_launch_are_installed_and_rviz_dependency_is_explicit():
    cmake = (ROOT / "CMakeLists.txt").read_text()
    assert "install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})" in cmake
    assert "launch/camera_visualization.launch.py" in cmake
    assert "<exec_depend>rviz2</exec_depend>" in (ROOT / "package.xml").read_text()
