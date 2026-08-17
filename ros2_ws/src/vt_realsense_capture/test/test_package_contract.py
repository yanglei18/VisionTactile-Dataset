from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).parents[1]
ROS_SHARE = Path("/opt/ros/jazzy/share")
RUNTIME_DEPENDENCY_TAGS = ("depend", "exec_depend")


def _runtime_dependencies(package_manifest: Path) -> set[str]:
    root = ElementTree.parse(package_manifest).getroot()
    return {
        element.text.strip()
        for tag in RUNTIME_DEPENDENCY_TAGS
        for element in root.findall(tag)
        if element.text
    }


def _runtime_dependency_closure(dependencies: set[str]) -> set[str]:
    closure: set[str] = set()
    pending = list(dependencies)
    while pending:
        dependency = pending.pop()
        if dependency in closure:
            continue
        closure.add(dependency)
        dependency_manifest = ROS_SHARE / dependency / "package.xml"
        if dependency_manifest.exists():
            pending.extend(
                _runtime_dependencies(dependency_manifest) - closure
            )
    return closure


def test_bag_validator_is_not_shipped() -> None:
    assert not (ROOT / "vt_realsense_capture" / "bag_validate.py").exists()
    assert not (ROOT / "scripts" / "bag_validate.py").exists()
    assert not (ROOT / "test" / "test_bag_validate_v2.py").exists()
    assert "bag_validate" not in (ROOT / "CMakeLists.txt").read_text()


def test_manifest_requires_rosbag_cli_without_compression_plugins() -> None:
    manifest_path = ROOT / "package.xml"
    manifest_root = ElementTree.parse(manifest_path).getroot()
    declared_dependencies = {
        element.text.strip()
        for element in manifest_root
        if (
            element.text
            and (
                element.tag == "depend"
                or element.tag.endswith("_depend")
            )
        )
    }
    runtime_dependencies = _runtime_dependencies(manifest_path)

    direct_rosbag_dependencies = {
        dependency
        for dependency in declared_dependencies
        if dependency == "ros2bag" or dependency.startswith("rosbag2")
    }
    assert direct_rosbag_dependencies == {"ros2bag"}
    assert (ROS_SHARE / "ros2bag" / "package.xml").is_file()

    runtime_dependency_closure = _runtime_dependency_closure(
        runtime_dependencies
    )
    assert "rosbag2" not in runtime_dependency_closure
    assert "rosbag2_compression" in runtime_dependency_closure
    assert "rosbag2_compression_zstd" not in runtime_dependency_closure


def test_manifest_declares_ros2_launch_cli_for_synthetic_tests() -> None:
    manifest_root = ElementTree.parse(ROOT / "package.xml").getroot()
    test_dependencies = {
        element.text.strip()
        for element in manifest_root.findall("test_depend")
        if element.text
    }

    assert "ros2launch" in test_dependencies
