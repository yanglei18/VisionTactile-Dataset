"""Read-only installation checks for the packaged static tactile viewer.

No application, sensor connection, model worker, or user preference is created.
The only output is the explicitly requested JSON report. Importing the renderer
checks its native dependencies; an actual OpenGL context still needs a desktop
smoke test, and serial hardware still needs a separate acceptance test.
"""
from __future__ import annotations

import importlib
from importlib.resources import files
import json
import os
from pathlib import Path
import platform
import re
import sys
import tempfile


def _runtime_details():
    # Match the application: establish PySide6 before importing pyqtgraph.
    versions = {}
    for name in ("PySide6", "PySide6.QtCore", "PySide6.QtGui", "PySide6.QtWidgets",
                 "numpy", "OpenGL", "OpenGL.GL", "serial", "pyqtgraph"):
        module = importlib.import_module(name)
        version = getattr(module, "__version__", None)
        if version is not None:
            versions[name] = str(version)
    graph = importlib.import_module("pyqtgraph")
    if graph.Qt.QT_LIB != "PySide6":
        raise RuntimeError(f"pyqtgraph 使用了 {graph.Qt.QT_LIB}，需要 PySide6。")
    renderer = importlib.import_module("vt_fsglove12.hand_view")
    if not callable(getattr(renderer, "HandView", None)):
        raise RuntimeError("三维手部渲染组件 HandView 不可用。")
    # Import the real receive pipeline without starting it. Native Windows
    # precise-clock binding must also work before an installer is accepted.
    source = importlib.import_module("vt_capture_console.backend.tactile_serial")
    clock = importlib.import_module("vt_tactile_capture.host_clock")
    if (not callable(getattr(source, "TactileSerialSource", None))
            or clock.realtime_ns() <= 0 or clock.monotonic_ns() <= 0):
        raise RuntimeError("串口读取组件或主机接收时钟不可用。")
    return {"versions": versions, "qt_binding": graph.Qt.QT_LIB,
            "renderer": "vt_fsglove12.hand_view.HandView",
            "receive_clock": "precise_utc_qpc" if sys.platform == "win32" else "system_realtime_monotonic",
            "opengl_context": "not_checked", "serial_device": "not_opened"}


def _resource_details():
    theme_root = files("vt_fsglove12.ui_theme")
    stylesheet = theme_root.joinpath("theme.qss").read_text(encoding="utf-8")
    if not stylesheet.strip():
        raise ValueError("主题样式资源为空。")
    icons = sorted(set(re.findall(r"@THEME_DIR@/([^\"')\s]+)", stylesheet)))
    if not icons:
        raise ValueError("主题缺少图标资源引用。")
    for name in icons:
        if not theme_root.joinpath(name).read_text(encoding="utf-8").strip():
            raise ValueError(f"主题图标为空：{name}")
    catalog_root = files("vt_capture_console.i18n")
    catalogs = sorted((item for item in catalog_root.iterdir()
                       if item.name.startswith("catalog_") and item.name.endswith(".json")),
                      key=lambda item: item.name)
    if not catalogs:
        raise ValueError("缺少界面语言资源。")
    translations = {}
    for path in catalogs:
        entries = json.loads(path.read_text(encoding="utf-8"))
        if (not isinstance(entries, dict) or not entries
                or any(not isinstance(key, str) or not isinstance(value, str)
                       for key, value in entries.items())):
            raise ValueError(f"界面语言资源格式无效：{path.name}")
        if translations.keys() & entries.keys():
            raise ValueError(f"界面语言资源包含重复条目：{path.name}")
        translations.update(entries)
    from vt_tactile_capture.hand_layout import load_layout

    layout = load_layout(expected_hand="right")
    return {"theme_icons": len(icons), "translation_files": len(catalogs),
            "translation_entries": len(translations), "layout": layout.name,
            "layout_addresses": len(layout.covered_indices)}


def _model_details(model_bundle):
    from .tactile_settings import TactileSettings
    from ..ui.widgets.tactile_model_loader import load_bundle_file
    from vt_fsglove12.tactile_surface import TactileSurfaceBinding

    settings = TactileSettings(model_bundle=model_bundle)
    if not settings.model_bundle:
        raise ValueError("缺少安装包静态手模型资源。")
    path = Path(settings.model_bundle).expanduser()
    bundle = load_bundle_file(path, "right")
    neutral, axes = bundle["neutral_geometry"], bundle["model_axes"]
    binding = TactileSurfaceBinding(neutral, axes, hand="right")
    metadata = binding.metadata()
    if metadata["bound_count"] < 1:
        raise ValueError("静态手模型没有可用的触觉表面映射。")
    # Report metadata only; never copy licensed geometry into diagnostics.
    return {"path": str(path), "hand": "right", "vertices": len(neutral["vertices_m"]),
            "faces": len(neutral["faces"]), "address_count": metadata["address_count"],
            "bound_count": metadata["bound_count"], "unbound_count": metadata["unbound_count"],
            "region_counts": metadata["region_counts"],
            "verified_skin_coordinates": metadata["verified_skin_coordinates"]}


def _write_report(path, report):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=path.parent, prefix=".tactile-check-",
                                         suffix=".json", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(report, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def check_installation(report_path, *, model_bundle=None) -> int:
    """Write a bounded diagnostic report; return 0 only when all checks pass.

    This verifies the packaged static model, never the optional Python/MANO
    exporter configuration. Missing resources fail instead of falling back to
    a model environment installed on the build machine.
    """
    report = {"schema_version": 1, "status": "ok", "issues": [], "details": {
        "python": platform.python_version(), "platform": platform.system(),
        "machine": platform.machine(), "frozen": bool(getattr(sys, "frozen", False))}}
    for name, check in (("runtime", _runtime_details), ("resources", _resource_details),
                        ("model", lambda: _model_details(model_bundle))):
        try:
            report["details"][name] = check()
        except Exception as exc:
            # The diagnostic command must produce a report for broken installs,
            # including unexpected import/native-extension failures.
            report["issues"].append({"check": name, "error": type(exc).__name__,
                                     "message": str(exc)[:4096]})
    if report["issues"]:
        report["status"] = "error"
    try:
        _write_report(report_path, report)
    except (OSError, ValueError, TypeError) as exc:
        print(f"无法写入安装自检报告：{exc}", file=sys.stderr)
        return 2
    return 1 if report["issues"] else 0
