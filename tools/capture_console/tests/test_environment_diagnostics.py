"""Local deployment diagnostics translate without modifying opaque path data."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from vt_capture_console import i18n
from vt_capture_console.backend.environment_discovery import default_data_root
from vt_capture_console.backend.workstation_settings import prepare_default_output
from vt_capture_console.i18n import messages


@pytest.fixture(autouse=True)
def diagnostic_language():
    previous = i18n.language()
    messages._index.cache_clear()
    messages._alias_index.cache_clear()
    messages._resolve.cache_clear()
    i18n.set_language("en", persist=False)
    yield
    i18n.set_language(previous, persist=False)
    messages._index.cache_clear()
    messages._alias_index.cache_clear()
    messages._resolve.cache_clear()


@pytest.mark.parametrize("source,english", [
    ("VT_DATA_ROOT 必须是绝对目录路径；可使用 ~，不执行 shell 变量展开。",
     "VT_DATA_ROOT must be an absolute directory path. You may use ~; shell variables are not expanded."),
    ("默认数据目录不能是符号链接：{path}",
     "The default data directory must not be a symbolic link: {path}"),
    ("默认数据目录必须为真实目录：{path}",
     "The default data directory must be a real directory: {path}"),
    ("默认数据目录不可写：{path}",
     "The default data directory is not writable: {path}"),
    ("无法创建默认数据目录：{path}；{detail}",
     "Could not create the default data directory: {path}; {detail}"),
])
def test_environment_diagnostics_retranslate_while_paths_and_error_details_stay_opaque(source, english):
    values = {
        "path": "/用户/默认数据目录不可写：{path}/'数据目录'",
        "detail": "Permission denied: '/用户/默认数据目录{detail}'",
    }
    raw = source.format(**values)
    expected = english.format(**values)
    payload = {"detail": raw, "path": values["path"]}
    before = payload.copy()
    displayed = messages.message(payload["detail"])
    assert str(displayed) == expected
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == raw
    assert str(messages.message(expected)) == raw
    i18n.set_language("en", persist=False)
    assert str(displayed) == expected
    assert payload == before


def test_catalog_registers_all_new_backend_errors_within_diagnostic_limits():
    entries = json.loads(Path(messages.__file__).with_name(
        "catalog_diagnostics_environment.json").read_text(encoding="utf-8"))
    exact, templates = messages._index()
    assert len(entries) == 5
    assert entries.keys() <= exact
    assert entries.keys() <= i18n.catalog().keys()
    assert len(exact) <= messages.MAX_SOURCES
    assert len(templates) <= messages.MAX_TEMPLATES


def test_actual_invalid_environment_exception_is_translated_without_rewriting_exception(tmp_path):
    with pytest.raises(ValueError) as caught:
        default_data_root(home=tmp_path, environ={"VT_DATA_ROOT": "$HOME/data"})
    raw = str(caught.value)
    assert str(messages.message(raw)) == (
        "VT_DATA_ROOT must be an absolute directory path. You may use ~; shell variables are not expanded.")
    assert str(caught.value) == raw


@pytest.mark.parametrize("failure,english", [
    ("symlink", "The default data directory must not be a symbolic link: "),
    ("file", "The default data directory must be a real directory: "),
    ("permission", "The default data directory is not writable: "),
    ("creation", "Could not create the default data directory: "),
])
def test_actual_settings_directory_errors_preserve_paths(tmp_path, monkeypatch, failure, english):
    output = tmp_path / "用户{path}" / "默认数据目录"
    output.parent.mkdir()
    monkeypatch.setenv("VT_DATA_ROOT", str(output))
    if failure == "symlink":
        target = tmp_path / "real"
        target.mkdir()
        output.symlink_to(target, target_is_directory=True)
    elif failure == "file":
        output.write_text("keep")
    elif failure == "permission":
        output.mkdir()
        monkeypatch.setattr("os.access", lambda *_: False)
    else:
        def denied(*args, **kwargs):
            raise PermissionError("Permission denied: '保留{detail}'")
        monkeypatch.setattr(Path, "mkdir", denied)
    configuration = {"process_groups": [{
        "preset": "cameras", "parameters": {"output_root": str(output)},
    }]}
    with pytest.raises(ValueError) as caught:
        prepare_default_output(configuration)
    raw = str(caught.value)
    expected = english + str(output)
    if failure == "creation":
        expected += "; Permission denied: '保留{detail}'"
    displayed = messages.message(raw)
    assert str(displayed) == expected
    i18n.set_language("zh_CN", persist=False)
    assert str(displayed) == raw
    assert str(caught.value) == raw


def test_ambiguous_path_delimiters_and_unrelated_user_text_are_not_modified():
    raw = "无法创建默认数据目录：/用户/包含；分隔符；Permission denied"
    assert str(messages.message(raw)) == raw
    note = "/用户/默认数据目录不可写：{path}/我的备注"
    assert str(messages.message(note)) == note
