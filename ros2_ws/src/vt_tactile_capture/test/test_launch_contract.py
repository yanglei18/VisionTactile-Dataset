import ast
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).parents[1]


def test_message_package_registers_ament_build_type_for_rosbag_discovery():
    package = ET.parse(ROOT.parent / "vt_tactile_msgs/package.xml").getroot()
    # Python imports alone can work for a generic CMake build while rosbag's
    # ament index cannot find the type. The package must source ament hooks.
    assert package.findtext("export/build_type") == "ament_cmake"


def test_launch_has_headless_explicit_port_and_namespace_controls():
    tree = ast.parse((ROOT / "launch/tactile.launch.py").read_text())
    arguments = {
        call.args[0].value: next((kw.value.value for kw in call.keywords
                                if kw.arg == "default_value" and isinstance(kw.value, ast.Constant)), None)
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
    }
    assert arguments["port"] == ""
    assert arguments["hand"] == ""
    assert arguments["namespace"] == "/gloves/right/tactile"
    assert arguments["gui"] == "true"
    assert arguments["publish_heatmap"] == "false"
    assert arguments["gui_python"] == "/usr/bin/python3"
    assert arguments["hand_layout_path"] == ""
    assert arguments["baseline_path"] == ""
    assert arguments["initial_view"] == "hand"
    assert 'IfCondition(LaunchConfiguration("gui"))' in (ROOT / "launch/tactile.launch.py").read_text()


def test_empty_launch_port_and_hand_preserve_yaml_values(monkeypatch):
    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location("tactile_launch", ROOT / "launch/tactile.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(port="", hand="")
    parameters = module._capture_node(context)[0]["parameters"]
    assert len(parameters) == 2  # YAML stays first; only explicit CLI values override it.
    assert "port" not in parameters[1]
    assert "hand" not in parameters[1]
    context.launch_configurations.update(port="/dev/serial/by-id/explicit-tactile", hand="left")
    overrides = module._capture_node(context)[0]["parameters"][1]
    assert overrides["port"] == "/dev/serial/by-id/explicit-tactile"
    assert overrides["hand"] == "left"


def test_package_and_node_do_not_import_pose_or_gui_dependencies():
    node_text = (ROOT / "vt_tactile_capture/node.py").read_text()
    viewer_text = (ROOT / "vt_tactile_capture/viewer.py").read_text()
    for forbidden in ("torch", "MANO", "PyQt", "pyqtgraph"):
        assert forbidden not in node_text
    assert "import serial" not in viewer_text
    assert "serial_source" not in viewer_text
    assert "vt_tactile_capture.node" not in viewer_text
    package = ET.parse(ROOT / "package.xml").getroot()
    dependencies = {child.text for child in package if child.tag.endswith("depend")}
    assert "vt_tactile_msgs" in dependencies
    assert not dependencies & {"torch", "vt_glove_bridge", "vt_glove_msgs"}


@pytest.mark.parametrize("filename", ["tactile.launch.py", "replay.launch.py"])
@pytest.mark.parametrize("layout,baseline,view", [
    ("", "", "hand"),
    ("/tmp/layout file.json", "/tmp/reference file.npz", "matrix"),
])
def test_display_arguments_are_typed_strings_and_only_reach_viewer(monkeypatch, filename, layout, baseline, view):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.parameter_descriptions import ParameterValue

    spec = importlib.util.spec_from_file_location("tactile_display_contract", ROOT / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    if filename == "tactile.launch.py":
        monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(ROOT))
    description = module.generate_launch_description()
    declarations = {entity.name: entity for entity in description.entities
                    if isinstance(entity, DeclareLaunchArgument)}
    context = LaunchContext()
    context.launch_configurations.update(
        hand_layout_path=layout, baseline_path=baseline, initial_view=view,
        port="", hand="", publish_heatmap="false",
    )
    defaults = {key: "".join(part.perform(context) for part in declarations[key].default_value)
                for key in ("hand_layout_path", "baseline_path", "initial_view")}
    assert defaults == {"hand_layout_path": "", "baseline_path": "", "initial_view": "hand"}
    viewer = (module._viewer_node(context)[0] if filename == "tactile.launch.py" else
              next(entity for entity in description.entities if isinstance(entity, dict)
                   and entity["executable"] == "vt_tactile_viewer"))
    values = viewer["parameters"][-1]
    for key, expected in (("hand_layout_path", layout), ("baseline_path", baseline), ("initial_view", view)):
        assert isinstance(values[key], ParameterValue)
        assert values[key].evaluate(context) == expected
        assert isinstance(values[key].evaluate(context), str)
    if filename == "tactile.launch.py":
        source = module._capture_node(context)[0]
    else:
        source = next(entity for entity in description.entities if isinstance(entity, dict)
                      and entity["executable"] == "vt_tactile_replay")
        assert values["use_sim_time"] is True
    for parameter_set in source["parameters"]:
        if isinstance(parameter_set, dict):
            assert not ({"hand_layout_path", "baseline_path", "initial_view"} & parameter_set.keys())


def test_candidate_layout_is_included_in_python_package_data():
    tree = ast.parse((ROOT / "setup.py").read_text())
    setup = next(call for call in ast.walk(tree) if isinstance(call, ast.Call)
                 and isinstance(call.func, ast.Name) and call.func.id == "setup")
    data = next(keyword.value for keyword in setup.keywords if keyword.arg == "package_data")
    assert isinstance(data, ast.Dict)
    assert any(isinstance(pattern, ast.Constant) and pattern.value == "layouts/*.json"
               for value in data.values for pattern in value.elts)
    assert (ROOT / "vt_tactile_capture/layouts/right_glove_candidate.json").is_file()


def test_viewer_receives_same_explicit_hand_or_yaml_as_capture(monkeypatch):
    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location("tactile_launch_hand", ROOT / "launch/tactile.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    context = LaunchContext()
    for hand in ("", "left", "right"):
        context.launch_configurations.update(hand=hand, port="", config="/tmp/device.yaml")
        viewer = module._viewer_node(context)[0]
        assert viewer["parameters"][0].perform(context) == "/tmp/device.yaml"
        assert viewer["parameters"][-1].get("hand", "") == hand
