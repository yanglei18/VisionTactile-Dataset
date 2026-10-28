"""Packaged-install checks use synthetic geometry and never acquire hardware."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from PySide6.QtCore import QCoreApplication, QSettings
import pytest

from vt_capture_console.backend import tactile_installation as module
from vt_capture_console.backend import tactile_settings


def _synthetic_bundle():
    """Independent flat strips with valid fingers; no licensed model data."""
    joints = np.zeros((21, 3))
    vertices, faces = [], []

    def strip(origin, direction, length, width, rows):
        direction = np.asarray(direction)
        lateral = np.cross(direction, [0., 0., 1.])
        if lateral[0] > 0:
            lateral = -lateral
        for z, reverse in ((-.004, False), (.004, True)):
            start = len(vertices)
            for along in np.linspace(0., length, rows):
                for across in np.linspace(-width / 2, width / 2, 5):
                    vertices.append(np.asarray(origin) + direction * along
                                    + lateral * across + [0., 0., z])
            for row in range(rows - 1):
                for col in range(4):
                    first = start + row * 5 + col
                    for face in ([first, first + 1, first + 5],
                                 [first + 1, first + 6, first + 5]):
                        faces.append(face[::-1] if reverse else face)

    strip([0., .006, 0.], [0., 1., 0.], .050, .077, 8)
    for finger, (x, y, direction) in enumerate((
            (-.044, .027, [-.8, .6, 0.]), (-.028, .070, [0., 1., 0.]),
            (-.009, .075, [0., 1., 0.]), (.012, .070, [0., 1., 0.]),
            (.031, .058, [0., 1., 0.]))):
        for offset, along in enumerate((0., .020, .038, .053)):
            joints[1 + 4 * finger + offset] = [x, y, 0.] + np.asarray(direction) * along
        strip([x, y, 0.], direction, .053, .014, 13)
    vertices.extend([np.zeros(3)] * (778 - len(vertices)))
    return {"schema_version": 1, "neutral_geometry": {
        "vertices_m": np.asarray(vertices).tolist(), "faces": faces,
        "joints_m": joints.tolist(), "provenance": {"hand": "right"}},
        "model_axes": {"hand": "right", "pitch_axis": [-1, 0, 0],
                       "roll_axis": [0, 1, 0], "palm_up_axis": [0, 0, 1]}}


@pytest.fixture
def isolated_settings(tmp_path, monkeypatch):
    path = tmp_path / "已有设置 中文.ini"
    settings = QSettings(str(path), QSettings.Format.IniFormat)
    settings.setValue("unrelated/keep", "保持原样")
    settings.sync()
    monkeypatch.setattr(tactile_settings, "QSettings", lambda *args, **kwargs: settings)
    # The constructor uses QSettings enum attributes as well as the factory.
    tactile_settings.QSettings.Format = QSettings.Format
    tactile_settings.QSettings.Scope = QSettings.Scope
    yield path


@pytest.fixture
def model_path(tmp_path):
    path = tmp_path / "安装 目录" / "右手 模型.json"
    path.parent.mkdir()
    path.write_text(json.dumps(_synthetic_bundle()), encoding="utf-8")
    return path


def _report(path):
    return json.loads(path.read_text(encoding="utf-8"))


@pytest.fixture
def default_reference_path(tmp_path):
    path = tmp_path / "通用参考.json"
    path.write_text(json.dumps({
        "schema_version": 2, "scope": "layout", "layout": "right_adc20x20_v1",
        "protocol": "vt_tactile_adc400_v1", "hand": "right",
        "reference": {"schema_version": 1, "baseline_adc": [18000.] * 400,
                      "activation_threshold_adc": [30.] * 400, "valid_mask": [True] * 400},
        "settings": {"display_range_adc": 1000., "direction": "both"},
        "captured_at_utc": "2026-10-27T00:00:00+00:00",
    }), encoding="utf-8")
    return path


@pytest.mark.parametrize("scope,expected_mode", [
    ("layout", "layout_default"), ("device", "device_matched_preset"),
])
def test_reference_self_check_reports_scope_without_requiring_device_access(
        tmp_path, model_path, isolated_settings, default_reference_path, monkeypatch, scope, expected_mode):
    import serial
    from serial.tools import list_ports

    def forbidden(*_args, **_kwargs):
        pytest.fail("Reference installation checks must not access serial hardware")

    monkeypatch.setattr(serial, "Serial", forbidden)
    monkeypatch.setattr(list_ports, "comports", forbidden)
    if scope == "device":
        value = _report(default_reference_path)
        del value["scope"], value["layout"]
        value.update(schema_version=1, device={"vendor_id": 1234, "product_id": 5678,
                                               "serial_number": "private-test-identity"})
        default_reference_path.write_text(json.dumps(value), encoding="utf-8")
    path = tmp_path / "reference-check.json"
    before = isolated_settings.read_bytes()
    assert module.check_installation(path, model_bundle=model_path,
                                     response_profile=default_reference_path) == 0
    details = _report(path)["details"]["response_profile"]
    assert details == {"included": True, "mode": expected_mode, "reference_cells": 400,
                       "display_range_adc": 1000., "direction": "both"}
    assert "private-test-identity" not in path.read_text(encoding="utf-8")
    assert isolated_settings.read_bytes() == before


@pytest.mark.parametrize("fault", ["scope", "layout", "protocol", "baseline", "missing"])
def test_invalid_explicit_reference_fails_installation_check(
        tmp_path, model_path, isolated_settings, default_reference_path, fault):
    value = _report(default_reference_path)
    if fault == "missing":
        default_reference_path.unlink()
    else:
        if fault == "baseline":
            value["reference"]["baseline_adc"][0] = -1
        else:
            value[fault] = "unsupported"
        default_reference_path.write_text(json.dumps(value), encoding="utf-8")
    path = tmp_path / "invalid-reference.json"
    assert module.check_installation(path, model_bundle=model_path,
                                     response_profile=default_reference_path) == 1
    report = _report(path)
    assert report["status"] == "error"
    assert [issue["check"] for issue in report["issues"]] == ["response_profile"]


def test_real_imports_binding_resources_and_unicode_paths_preserve_settings(
        tmp_path, model_path, isolated_settings):
    report_path = tmp_path / "诊断 结果" / "安装自检.json"
    settings_before = isolated_settings.read_bytes()
    application_before = QCoreApplication.instance()
    assert module.check_installation(report_path, model_bundle=model_path) == 0
    report = _report(report_path)
    assert report["schema_version"] == 1
    assert report["status"] == "ok" and report["issues"] == []
    assert report["details"]["runtime"]["qt_binding"] == "PySide6"
    assert report["details"]["runtime"]["opengl_context"] == "not_checked"
    assert report["details"]["resources"]["layout_addresses"] == 400
    model = report["details"]["model"]
    assert model["vertices"] == 778 and model["address_count"] == 400
    assert model["bound_count"] > 300
    assert model["bound_count"] + model["unbound_count"] == 400
    assert model["verified_skin_coordinates"] is False
    assert "vertices_m" not in report_path.read_text(encoding="utf-8")
    assert isolated_settings.read_bytes() == settings_before
    assert QCoreApplication.instance() is application_before
    assert not list(report_path.parent.glob(".tactile-check-*"))


def test_missing_model_fails_with_report_instead_of_starting_exporter(tmp_path, isolated_settings):
    path = tmp_path / "missing-report.json"
    assert module.check_installation(path, model_bundle=tmp_path / "缺失 模型.json") == 1
    report = _report(path)
    assert report["status"] == "error"
    assert any(issue["check"] == "model" for issue in report["issues"])


def test_no_packaged_model_fails_without_using_external_model_settings(
        tmp_path, isolated_settings, monkeypatch):
    monkeypatch.setattr(tactile_settings, "bundled_model_path", lambda: "")
    path = tmp_path / "no-bundle.json"
    assert module.check_installation(path) == 1
    issue, = _report(path)["issues"]
    assert issue["check"] == "model" and "静态手模型" in issue["message"]


@pytest.mark.parametrize("kind", ["json", "duplicate", "oversized", "degenerate_joints", "unbound"])
def test_invalid_static_models_fail_including_unusable_surface_binding(
        tmp_path, model_path, isolated_settings, kind):
    bundle = deepcopy(_synthetic_bundle())
    if kind == "json":
        document = "not JSON"
    elif kind == "duplicate":
        document = '{"schema_version":1,' + json.dumps(bundle)[1:]
    elif kind == "oversized":
        document = " " * (512 * 1024 + 1)
    else:
        if kind == "degenerate_joints":
            bundle["neutral_geometry"]["joints_m"] = np.zeros((21, 3)).tolist()
        else:
            # A valid triangle with a dorsal normal, so no tactile surface exists.
            bundle["neutral_geometry"]["faces"] = [[0, 5, 1]]
        document = json.dumps(bundle)
    model_path.write_text(document, encoding="utf-8")
    report_path = tmp_path / "bad-model.json"
    assert module.check_installation(report_path, model_bundle=model_path) == 1
    assert [issue["check"] for issue in _report(report_path)["issues"]] == ["model"]


def test_wrong_qt_binding_is_reported(tmp_path, model_path, isolated_settings, monkeypatch):
    import pyqtgraph

    monkeypatch.setattr(pyqtgraph.Qt, "QT_LIB", "PyQt5")
    path = tmp_path / "wrong-qt.json"
    assert module.check_installation(path, model_bundle=model_path) == 1
    issue, = _report(path)["issues"]
    assert issue["check"] == "runtime" and "PySide6" in issue["message"]


def test_unavailable_receive_clock_prevents_accepting_an_installation(
        tmp_path, model_path, isolated_settings, monkeypatch):
    from vt_tactile_capture import host_clock

    monkeypatch.setattr(host_clock, "realtime_ns", lambda: 0)
    path = tmp_path / "bad-clock.json"
    assert module.check_installation(path, model_bundle=model_path) == 1
    issue, = _report(path)["issues"]
    assert issue["check"] == "runtime" and "时钟" in issue["message"]


def test_missing_translation_resources_are_reported(
        tmp_path, model_path, isolated_settings, monkeypatch):
    original = module.files
    empty = tmp_path / "missing-catalogs"
    empty.mkdir()
    monkeypatch.setattr(module, "files", lambda name:
                        empty if name == "vt_capture_console.i18n" else original(name))
    path = tmp_path / "missing-resources.json"
    assert module.check_installation(path, model_bundle=model_path) == 1
    issue, = _report(path)["issues"]
    assert issue["check"] == "resources" and "语言资源" in issue["message"]


def test_report_write_failure_has_nonzero_exit(tmp_path, model_path, isolated_settings, capsys):
    assert module.check_installation(tmp_path, model_bundle=model_path) == 2
    assert "无法写入" in capsys.readouterr().err
    assert not list(tmp_path.parent.glob(".tactile-check-*"))


def test_fresh_process_has_no_app_heavy_runtime_settings_writes_or_device_access(tmp_path, model_path):
    path = tmp_path / "isolated-check.json"
    source = r'''
import importlib.abc
import sys
class ForbiddenModules(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"torch", "manotorch", "rclpy"}:
            raise AssertionError("Unexpected runtime import: " + fullname)
sys.meta_path.insert(0, ForbiddenModules())
from PySide6.QtCore import QCoreApplication, QSettings, QProcess
import serial
def forbidden(*args, **kwargs):
    raise AssertionError("Installation checks cannot write settings or open devices/processes")
QSettings.setValue = forbidden
QSettings.sync = forbidden
QProcess.start = forbidden
serial.Serial = forbidden
from vt_capture_console.backend.tactile_installation import check_installation
assert QCoreApplication.instance() is None
result = check_installation(sys.argv[1], model_bundle=sys.argv[2])
assert QCoreApplication.instance() is None
assert not {"torch", "manotorch", "rclpy"}.intersection(sys.modules)
raise SystemExit(result)
'''
    result = subprocess.run([sys.executable, "-I", "-c", source, str(path), str(model_path)],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr + (_report(path).__repr__() if path.exists() else "")
    assert _report(path)["status"] == "ok"
