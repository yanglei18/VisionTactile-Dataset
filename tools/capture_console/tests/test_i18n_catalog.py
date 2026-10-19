"""Translation coverage checks against checked-in application source and catalogs.

These checks do not change the interface language or contact a backend. Run with
``pytest -s`` to see the advisory report for unwrapped Chinese literals; the
same report is included as a JUnit property when XML reporting is enabled.
"""

from __future__ import annotations

import ast
from collections import Counter
import json
from pathlib import Path
import re
from string import Formatter

import pytest


_PACKAGE = Path(__file__).resolve().parents[1] / "src" / "vt_capture_console"
_CHINESE = re.compile(r"[\u3400-\u4dbf\u4e00-\u9fff\uf900-\ufaff]")
_FORMATTER = Formatter()


def _unique_mapping(pairs):
    result = {}
    for source, english in pairs:
        if source in result:
            raise ValueError(f"Duplicate translation key: {source!r}")
        result[source] = english
    return result


@pytest.fixture(scope="module")
def catalogs():
    paths = sorted((_PACKAGE / "i18n").glob("catalog_*.json"))
    assert {"catalog_a.json", "catalog_b.json", "catalog_c.json"} <= {
        path.name for path in paths
    }, "All three base English catalogs must be checked in"
    merged = {}
    origins = {}
    for path in paths:
        try:
            entries = json.loads(
                path.read_text(encoding="utf-8"), object_pairs_hook=_unique_mapping
            )
        except (ValueError, UnicodeError) as exc:
            pytest.fail(f"Invalid catalog {path.name}: {exc}")
        assert isinstance(entries, dict), f"{path.name} must contain a JSON object"
        assert entries, f"{path.name} must not be empty"
        for source, english in entries.items():
            assert isinstance(source, str) and source.strip(), (
                f"{path.name} has an empty or non-string source"
            )
            assert source not in merged, (
                f"Translation {source!r} appears in both "
                f"{origins.get(source)} and {path.name}"
            )
            merged[source] = english
            origins[source] = path.name
    return merged, origins


@pytest.fixture(scope="module")
def presentation_sources():
    sources = []
    for directory in ("ui", "viewmodels"):
        paths = sorted((_PACKAGE / directory).rglob("*.py"))
        assert paths, f"No presentation sources found in {directory}"
        for path in paths:
            relative = path.relative_to(_PACKAGE)
            sources.append((relative, ast.parse(path.read_text(encoding="utf-8"))))
    return sources


def _translation_source(node):
    if not isinstance(node, ast.Call):
        return None
    function = node.func
    is_translation = (
        isinstance(function, ast.Name) and function.id == "tr"
    ) or (isinstance(function, ast.Attribute) and function.attr == "tr")
    if not is_translation:
        return None
    source = node.args[0] if node.args else next(
        (keyword.value for keyword in node.keywords if keyword.arg == "source"), None
    )
    if isinstance(source, ast.Constant) and isinstance(source.value, str):
        return source
    return None


def _format_fields(text):
    # Counter preserves multiplicity: silently removing a repeated parameter is
    # a data loss even when the set of field names remains unchanged.
    return Counter(
        (field, specification, conversion)
        for _, field, specification, conversion in _FORMATTER.parse(text)
        if field is not None
    )


def test_all_catalog_entries_have_english_display_text(catalogs):
    entries, origins = catalogs
    invalid = []
    for source, english in entries.items():
        if not isinstance(english, str) or not english.strip():
            invalid.append(f"{origins[source]}: {source!r} has no English translation")
        elif _CHINESE.search(english):
            invalid.append(f"{origins[source]}: {source!r} retains Chinese: {english!r}")
    assert not invalid, "\n".join(invalid)


def test_catalogs_preserve_format_arguments_and_line_breaks(catalogs):
    entries, origins = catalogs
    invalid = []
    for source, english in entries.items():
        if not isinstance(english, str):
            invalid.append(f"{origins[source]}: {source!r} has a non-string translation")
            continue
        try:
            expected = _format_fields(source)
            actual = _format_fields(english)
        except ValueError as exc:
            invalid.append(f"{origins[source]}: {source!r}: {exc}")
            continue
        if actual != expected:
            invalid.append(
                f"{origins[source]}: {source!r}\n"
                f"  missing fields: {expected - actual}\n"
                f"  extra fields: {actual - expected}"
            )
        if source.count("\n") != english.count("\n"):
            invalid.append(f"{origins[source]}: {source!r} changes the line break count")
    assert not invalid, "\n".join(invalid)


def test_translated_ui_and_viewmodel_messages_are_in_catalogs(catalogs, presentation_sources):
    entries, _ = catalogs
    missing = []
    checked = 0
    for path, tree in presentation_sources:
        for node in ast.walk(tree):
            source = _translation_source(node)
            if source is None or not _CHINESE.search(source.value):
                continue
            checked += 1
            if source.value not in entries:
                missing.append(f"{path}:{source.lineno}: {source.value!r}")
    assert checked, "The coverage scan did not find any translated Chinese UI messages"
    assert not missing, "UI messages missing English translations:\n" + "\n".join(missing)


def _unwrapped_chinese_literals(tree):
    translated = {
        id(source)
        for node in ast.walk(tree)
        if (source := _translation_source(node)) is not None
    }
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            if node.body and isinstance(node.body[0], ast.Expr):
                value = node.body[0].value
                if isinstance(value, ast.Constant) and isinstance(value.value, str):
                    docstrings.add(id(value))
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and _CHINESE.search(node.value)
        and id(node) not in translated
        and id(node) not in docstrings
    ]


def test_unwrapped_chinese_literals_are_reported(presentation_sources, record_property):
    # This is advisory: an untranslated literal may be a protocol lookup, a
    # Chinese-language selector label, or another intentional non-display value.
    findings = [
        f"{path}:{node.lineno}: {node.value!r}"
        for path, tree in presentation_sources
        for node in _unwrapped_chinese_literals(tree)
    ]
    record_property("unwrapped_chinese_literals", json.dumps(findings, ensure_ascii=False))
    print(f"\nUnwrapped Chinese literals for review ({len(findings)}):")
    for finding in findings:
        print(finding)


def test_advisory_scan_distinguishes_docs_and_translated_messages():
    source = '''"""中文模块说明。"""
class Example:
    """中文类说明。"""
    def method(self):
        """中文方法说明。"""
        tr("已翻译消息")
        i18n.tr(source="另一条已翻译消息")
        # 中文注释不是显示文案。
        self.setText("未翻译标题")
        return "中文协议值"
'''
    findings = _unwrapped_chinese_literals(ast.parse(source))
    assert [node.value for node in findings] == ["中文协议值", "未翻译标题"]


def test_format_comparison_detects_lost_repeated_fields():
    source = "{v0:.1f} / {v1!s} / {v0:.1f}"
    assert _format_fields(source) != _format_fields("{v0:.1f} / {v1!s}")
    assert _format_fields(source) != _format_fields("{v0:.0f} / {v1!s} / {v0:.1f}")
    assert _format_fields(source) != _format_fields("{v0:.1f} / {v1!r} / {v0:.1f}")
    assert _format_fields(source) == _format_fields("{v1!s} / {v0:.1f} / {v0:.1f}")
