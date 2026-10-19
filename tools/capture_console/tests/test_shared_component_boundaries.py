"""Shared presentation widgets must be usable without the application shell."""

import ast
from importlib.util import resolve_name
import os
from pathlib import Path
import subprocess
import sys
import textwrap
from types import SimpleNamespace

import pytest


WIDGETS = (Path(__file__).resolve().parents[1] / "src" / "vt_capture_console"
           / "ui" / "widgets")
APPLICATION_SHELL = "vt_capture_console.ui.main_window"
SOURCE_ROOT = WIDGETS.parents[1]
SHARED_COMPONENTS = {
    WIDGETS / "capture_stage.py": "CaptureStage",
    WIDGETS / "device_controls.py": "DeviceControls",
    WIDGETS / "device_card.py": "DeviceCard",
    WIDGETS / "preview.py": "PreviewWidget",
    WIDGETS / "spatial_scene.py": "SpatialScene",
    SOURCE_ROOT / "viewmodels" / "spatial.py": "SpatialViewModel",
}


def _private_owners():
    """Derive protected state/methods from shared component implementations."""
    owners = {}
    for path, class_name in SHARED_COMPONENTS.items():
        tree = ast.parse(path.read_text(encoding="utf-8"))
        component = next(node for node in ast.walk(tree)
                         if isinstance(node, ast.ClassDef) and node.name == class_name)
        for node in ast.walk(component):
            name = None
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                name = node.name
            elif (isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name)
                  and node.value.id == "self"):
                name = node.attr
            if name and name.startswith("_") and not name.startswith("__"):
                owners.setdefault(name, set()).add((path, class_name))
    return owners


def _foreign_private_accesses(source, path, owners):
    """Conservatively guard shared private names, without inferring object types.

    Another object's matching private name is rejected even if its type is
    unrelated; this is a name-based boundary guard, not a Python type analysis.
    Copying instance caches is allowed only inside the owning component class
    and module. Ordinary self/cls internals and Qt superclass calls remain legal.
    """
    violations = []

    class BoundaryVisitor(ast.NodeVisitor):
        def __init__(self):
            self.classes = []

        def visit_ClassDef(self, node):
            self.classes.append(node.name)
            self.generic_visit(node)
            self.classes.pop()

        def record(self, receiver, name, line):
            owner = (path, ".".join(self.classes))
            if not isinstance(name, str) or name not in owners or owner in owners[name]:
                return
            if isinstance(receiver, ast.Name) and receiver.id in {"self", "cls"}:
                return
            if (isinstance(receiver, ast.Call) and isinstance(receiver.func, ast.Name)
                    and receiver.func.id == "super"):
                return
            violations.append((line, name))

        def visit_Attribute(self, node):
            self.record(node.value, node.attr, node.lineno)
            self.generic_visit(node)

        def visit_Call(self, node):
            if (isinstance(node.func, ast.Name)
                    and node.func.id in {"getattr", "setattr", "hasattr", "delattr"}
                    and len(node.args) >= 2 and isinstance(node.args[1], ast.Constant)):
                self.record(node.args[0], node.args[1].value, node.lineno)
            self.generic_visit(node)

    BoundaryVisitor().visit(ast.parse(source))
    return sorted(violations)


def test_all_component_consumers_use_public_presentation_boundaries():
    owners = _private_owners()
    violations = []
    for path in sorted(SOURCE_ROOT.rglob("*.py")):
        violations.extend(f"{path.relative_to(SOURCE_ROOT)}:{line}: {name}"
                          for line, name in _foreign_private_accesses(
                              path.read_text(encoding="utf-8"), path, owners))
    assert not violations, "Cross-component private access: " + ", ".join(violations)


def test_boundary_checker_distinguishes_consumers_from_component_internals():
    owners = _private_owners()
    consumer = SOURCE_ROOT / "controllers" / "example.py"
    source = textwrap.dedent("""
        self._frame = None
        self._position_overlays()
        super().closeEvent(event)
        self.preview.device_snapshot
        source._frame
        self.stage._position_overlays()
        getattr(self.controls, "_available")
        setattr(self.model, "_held", None)
    """)
    assert [name for _line, name in _foreign_private_accesses(source, consumer, owners)] == [
        "_frame", "_position_overlays", "_available", "_held",
    ]
    assert not _foreign_private_accesses(textwrap.dedent("""
        class PreviewWidget:
            def copy_from(self, source):
                self._frame = source._frame
                self._reset_frame()
                super().closeEvent(event)
    """), WIDGETS / "preview.py", owners)


def test_boundary_checker_does_not_exempt_another_class_in_the_same_module():
    source = textwrap.dedent("""
        class PreviewInspector:
            def inspect(self, preview):
                return preview._frame
    """)
    assert [name for _line, name in _foreign_private_accesses(
        source, WIDGETS / "preview.py", _private_owners())] == ["_frame"]


def test_stage_public_layout_refresh_reflows_changed_presentation(qapp):
    from vt_capture_console.ui.widgets.capture_stage import CaptureStage

    stage = CaptureStage()
    try:
        stage.resize(850, 480)
        stage.show()
        qapp.processEvents()
        before = stage.hero_title.height()
        stage.hero_title.setText("One\nTwo\nThree\nFour")
        stage.refresh_layout()
        assert stage.hero_title.height() > before
        assert stage.canvas.rect().contains(stage.hero.geometry())
        assert stage.source_preview is None
    finally:
        stage.close()


def test_device_control_availability_is_read_only_and_tracks_refresh(qapp):
    from vt_capture_console.ui.widgets.device_controls import DeviceControls

    backend = SimpleNamespace(mode="live", supports_device_control=True, latest_operation=None)
    controls = DeviceControls(backend)
    try:
        assert controls.control_available is False
        snapshot = {"process_control": {"enabled": True, "groups": []},
                    "capabilities": {"device_control": True}}
        controls.refresh(snapshot)
        assert controls.control_available is True
        snapshot["capabilities"]["device_control"] = False
        controls.refresh(snapshot)
        assert controls.control_available is False
        with pytest.raises(AttributeError):
            controls.control_available = True
    finally:
        controls.close()


def test_camera_canvas_can_paint_history_through_public_preview_api(qapp):
    from PySide6.QtCore import QRectF
    from PySide6.QtGui import QColor, QImage, QPainter
    from vt_capture_console.ui.widgets.preview import PreviewWidget

    preview = PreviewWidget("camera")
    image = QImage(240, 120, QImage.Format.Format_ARGB32)
    image.fill(QColor("white"))
    painter = QPainter(image)
    try:
        preview.paint_history_overlay(painter, QRectF(0, 0, 240, 120))
    finally:
        painter.end()
        preview.close()
    assert image.pixelColor(0, 0) != QColor("white")


def _imported_modules(node, package):
    if isinstance(node, ast.Import):
        return [alias.name for alias in node.names]
    if isinstance(node, ast.ImportFrom):
        module = "." * node.level + (node.module or "")
        base = resolve_name(module, package) if node.level else module
        return [base, *(f"{base}.{alias.name}" for alias in node.names)]
    if isinstance(node, ast.Call) and node.args:
        function = node.func
        name = function.id if isinstance(function, ast.Name) else (
            function.attr if isinstance(function, ast.Attribute) else "")
        argument = node.args[0]
        if (name in {"import_module", "__import__"}
                and isinstance(argument, ast.Constant)
                and isinstance(argument.value, str)):
            module = argument.value
            return [resolve_name(module, package) if module.startswith(".") else module]
    return []


def test_shared_widgets_do_not_import_the_application_shell():
    violations = []
    for path in sorted(WIDGETS.rglob("*.py")):
        relative = path.relative_to(WIDGETS)
        package = ".".join(("vt_capture_console", "ui", "widgets", *relative.parts[:-1]))
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if any(module == APPLICATION_SHELL or module.startswith(APPLICATION_SHELL + ".")
                   for module in _imported_modules(node, package)):
                violations.append(f"{relative}:{node.lineno}")
    assert not violations, "Shared widgets depend on MainWindow: " + ", ".join(violations)


def test_device_card_can_be_used_without_importing_main_window():
    # The suite's autouse settings fixture imports MainWindow. A fresh process
    # makes this a real import-boundary check, independent of test order.
    script = textwrap.dedent("""
        import sys
        from PySide6.QtCore import QCoreApplication, QEvent
        from PySide6.QtTest import QSignalSpy
        from PySide6.QtWidgets import QApplication
        from shiboken6 import isValid
        from vt_capture_console.ui.widgets.device_card import DeviceCard

        shell = "vt_capture_console.ui.main_window"
        assert DeviceCard.__module__ == "vt_capture_console.ui.widgets.device_card"
        assert shell not in sys.modules
        app = QApplication([])
        device = dict(id="camera:test", kind="camera", label="Test camera",
                      status="offline", receive_hz=0.0, preview={})
        card = DeviceCard(device)
        initial_status = card.state.text()
        card.update_device(dict(device, status="online", receive_hz=12.5))
        assert card.state.text() != initial_status
        assert "12.5" in card.metrics.text()

        replay = DeviceCard(device, embedded_replay=True)
        selected = QSignalSpy(replay.selected)
        windows_before = {id(widget) for widget in app.topLevelWidgets()}
        replay.open_detail()
        assert selected.count() == 1
        assert selected.at(0) == [device["id"]]
        assert {id(widget) for widget in app.topLevelWidgets()} == windows_before
        for widget in (card, replay):
            widget.close_details()
            widget.close()
            widget.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        app.processEvents()
        assert not isValid(card) and not isValid(replay)
        assert shell not in sys.modules
    """)
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=30,
        env={**os.environ, "QT_QPA_PLATFORM": "offscreen"}, check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
