from __future__ import annotations

import ast
import importlib.util
import runpy
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).parents[1]
MODULE_ROOT = PACKAGE_ROOT / "vt_vive_tracker_gui"


def _load_launch_description():
    path = PACKAGE_ROOT / "launch" / "tracker_gui.launch.py"
    spec = importlib.util.spec_from_file_location("tracker_gui_launch", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _substitution_text(value):
    if isinstance(value, str):
        return value
    return perform_substitutions(LaunchContext(), value)


def test_launch_starts_exactly_one_gui_node_and_no_publisher():
    description = _load_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert [
        (
            _substitution_text(node.node_package),
            _substitution_text(node.node_executable),
        )
        for node in nodes
    ] == [("vt_vive_tracker_gui", "vt_vive_tracker_gui")]


def test_setup_exports_and_installs_gui_product(monkeypatch):
    captured = {}
    monkeypatch.chdir(PACKAGE_ROOT)
    monkeypatch.setattr("setuptools.setup", lambda **values: captured.update(values))

    runpy.run_path(str(PACKAGE_ROOT / "setup.py"), run_name="__main__")

    assert captured["entry_points"]["console_scripts"] == [
        "vt_vive_tracker_gui = vt_vive_tracker_gui.main:main"
    ]
    installed = {
        destination: tuple(files)
        for destination, files in captured["data_files"]
    }
    assert installed["share/vt_vive_tracker_gui"] == (
        "package.xml",
        "README.md",
    )
    assert installed["share/vt_vive_tracker_gui/launch"] == (
        "launch/tracker_gui.launch.py",
    )


def test_gui_sources_have_no_write_hardware_or_process_dependencies():
    forbidden_calls = {"create_publisher", "create_service", "create_client"}
    forbidden_imports = {"subprocess", "pyvut", "hid", "hidapi"}

    for path in MODULE_ROOT.glob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        calls = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        imported_roots = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported_roots.update(
                    alias.name.split(".", 1)[0] for alias in node.names
                )
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported_roots.add(node.module.split(".", 1)[0])

        assert calls.isdisjoint(forbidden_calls), path
        assert imported_roots.isdisjoint(forbidden_imports), path


def test_main_composes_owned_runtime_and_closes_it_headlessly(monkeypatch):
    import vt_vive_tracker_gui.main as main_module

    events = []

    class FakeRoot:
        def title(self, value):
            events.append(("title", value))

        def geometry(self, value):
            events.append(("geometry", value))

        def minsize(self, width, height):
            events.append(("minsize", width, height))

        def mainloop(self):
            events.append("mainloop")

    class FakeRuntime:
        def start(self):
            events.append("runtime_start")

        def stop(self):
            events.append("runtime_stop")

    runtime = FakeRuntime()

    class FakeRosRuntime:
        @classmethod
        def from_node(cls, node, shutdown_context):
            events.append(("runtime", node, shutdown_context))
            return runtime

    class FakeApplication:
        def __init__(self, root, store, view, shutdown):
            events.append(("app", root, store, view, shutdown))
            self.shutdown = shutdown

        def start(self):
            events.append("app_start")

        def close(self):
            events.append("app_close")
            self.shutdown()

    store = object()
    node = object()
    view = object()
    root = FakeRoot()
    shutdown_context = lambda: events.append("rclpy_shutdown")
    monkeypatch.setattr(
        main_module.rclpy,
        "init",
        lambda *, args: events.append(("rclpy_init", args)),
    )
    monkeypatch.setattr(main_module.rclpy, "shutdown", shutdown_context)
    monkeypatch.setattr(main_module, "LatestSnapshotStore", lambda: store)
    monkeypatch.setattr(main_module, "TrackerGuiNode", lambda value: node)
    monkeypatch.setattr(main_module.tk, "Tk", lambda: root)
    monkeypatch.setattr(main_module, "TrackerDashboard", lambda value: view)
    monkeypatch.setattr(main_module, "TrackerApplication", FakeApplication)
    monkeypatch.setattr(main_module, "RosRuntime", FakeRosRuntime)

    main_module.main(args=["--ros-args"])

    assert events == [
        ("rclpy_init", ["--ros-args"]),
        ("runtime", node, shutdown_context),
        ("title", "VIVE Ultimate Tracker Monitor"),
        ("geometry", "1280x800"),
        ("minsize", 1000, 650),
        ("app", root, store, view, runtime.stop),
        "runtime_start",
        "app_start",
        "mainloop",
        "app_close",
        "runtime_stop",
    ]


def test_readme_documents_launch_roles_and_read_only_boundary():
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    assert "ros2 launch vt_vive_tracker_gui tracker_gui.launch.py" in readme
    assert all(role in readme for role in ("left_wrist", "right_wrist", "torso"))
    assert "read-only ROS consumer" in readme
