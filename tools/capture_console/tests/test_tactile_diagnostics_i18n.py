"""Translate authored tactile diagnostics only at their display boundary."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_capture_console import i18n
from vt_capture_console.backend.tactile_serial import _CLOCK_REASONS, _WAITING_REASONS, _serial_error
from vt_capture_console.backend.tactile_source import TactileDisplayModel
from vt_capture_console.i18n import messages


def clear_diagnostic_cache():
    messages._index.cache_clear()
    messages._alias_index.cache_clear()
    messages._resolve.cache_clear()


@pytest.fixture(autouse=True)
def diagnostic_language():
    before = i18n.language()
    clear_diagnostic_cache()
    i18n.set_language("en", persist=False)
    clear_diagnostic_cache()
    yield
    i18n.set_language(before, persist=False)
    clear_diagnostic_cache()


def test_tactile_diagnostics_are_registered_and_retranslate_without_losing_fields():
    path = Path(messages.__file__).with_name("catalog_diagnostics_tactile.json")
    entries = json.loads(path.read_text(encoding="utf-8"))
    exact, templates = messages._index()
    assert entries.keys() <= exact
    assert len(exact) <= messages.MAX_SOURCES
    assert len(templates) <= messages.MAX_TEMPLATES
    sample = {"count": 217, "name": "valid_mask", "v0": "vertices_m",
              "detail": r"COM12: C:\操作者\模型{v0}\'右手'.json"}
    for source, english in entries.items():
        raw = source.format(**sample)
        expected = english.format(**sample)
        display = messages.message(raw)
        assert str(display) == expected, source
        i18n.set_language("zh_CN", persist=False)
        assert str(display) == raw, source
        i18n.set_language("en", persist=False)
        assert str(display) == expected, source


@pytest.mark.parametrize("detail", [*_WAITING_REASONS.values(), *_CLOCK_REASONS.values()])
def test_serial_stream_conditions_have_complete_translations(detail):
    display = messages.message(detail)
    assert str(display) != detail
    assert not any("\u4e00" <= char <= "\u9fff" for char in str(display))
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == detail


@pytest.mark.parametrize("reason", [
    "PortInUseError: COM12 is already in use",
    "ClearCommError failed: WinError 1167",
    "Access is denied: COM12",
    "PermissionError: /dev/ttyACM0",
    "FileNotFoundError: COM12",
    "Input/output error",
    "ModuleNotFoundError: serial",
    "Unknown serial error: COM12",
])
def test_serial_error_translation_preserves_backend_diagnostics(reason):
    authored = _serial_error(reason)
    payload = {"reason": reason, "detail": authored, "port": "COM12"}
    original = payload.copy()
    display = messages.message(payload["detail"])
    assert str(display) != authored
    assert not any("\u4e00" <= char <= "\u9fff" for char in str(display))
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == authored
    assert payload == original


def test_grid_validation_error_is_localized_without_changing_source_state():
    model = TactileDisplayModel()
    model.observe(SimpleNamespace(schema_version="invalid", rows=20, cols=20), now_ns=1)
    state = model.poll(now_ns=1)
    assert state.detail == "Invalid tactile schema version"
    display = messages.message(state.detail)
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == "触觉数据版本无效"
    assert state.detail == "Invalid tactile schema version"
    i18n.set_language("en", persist=False)
    assert str(display) == state.detail


@pytest.mark.parametrize("prefix,english", [
    ("静态手模型加载失败：", "Static hand model could not be loaded: "),
    ("3D 模型数据无效：", "Invalid 3D model data: "),
    ("3D 模型进程无法运行；请检查手套 Python 环境：",
     "The 3D model process cannot run; check the glove Python environment: "),
])
def test_model_error_keeps_unknown_path_port_and_embedded_messages_opaque(prefix, english):
    detail = r"COM12: C:\用户\等待数据\模型{v0}\'右手'.json; Access denied"
    raw = prefix + detail
    display = messages.message(raw)
    assert str(display) == english + detail
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == raw


def test_known_nested_static_model_error_translates_the_entire_diagnostic(tmp_path):
    from vt_capture_console.ui.widgets.tactile_model_loader import load_bundle_file

    invalid = tmp_path / "右手{v0}.json"
    invalid.write_text('{"schema_version": 1, "schema_version": 1}', encoding="utf-8")
    i18n.set_language("zh_CN", persist=False)
    with pytest.raises(ValueError) as failure:
        load_bundle_file(invalid)
    raw = "静态手模型加载失败：" + str(failure.value)
    display = messages.message(raw)
    i18n.set_language("en", persist=False)
    assert str(display) == (
        "Static hand model could not be loaded: Static hand model JSON contains duplicate fields")
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == raw


def test_existing_model_error_templates_are_registered_for_later_language_changes():
    raw = "3D 模型 vertices_m 的形状或数值无效"
    display = messages.message(raw)
    assert str(display) == "Invalid shape or values in 3D model vertices_m"
    i18n.set_language("zh_CN", persist=False)
    assert str(display) == raw


@pytest.mark.parametrize("raw", [
    "COM12 · 操作者手套", r"C:\模型\等待数据\COM12.json",
    "用户笔记：请选择串口并连接。", "Unknown device detail: COM12",
])
def test_user_port_labels_paths_and_unknown_text_are_not_translated(raw):
    assert str(messages.message(raw)) == raw
    i18n.set_language("zh_CN", persist=False)
    assert str(messages.message(raw)) == raw
