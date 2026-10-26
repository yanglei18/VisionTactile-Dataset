"""The shared appearance is usable without either Qt binding or device runtime."""
from pathlib import Path
import re
import subprocess
import sys

import pytest

from vt_fsglove12 import ui_theme


def test_shared_theme_is_cwd_independent_and_icons_exist(tmp_path, monkeypatch):
    original = ui_theme.stylesheet()
    monkeypatch.chdir(tmp_path)
    assert ui_theme.stylesheet() == original
    assert "@THEME_DIR@" not in original
    paths = set(re.findall(r'url\("([^\"]+)"\)', original))
    assert len(paths) >= 5
    for path in paths:
        assert Path(path).is_absolute()
        assert "<svg" in Path(path).read_text(encoding="utf-8")


def test_calibration_adds_only_its_controls_to_shared_appearance():
    style = ui_theme.calibration_stylesheet()
    assert style.startswith(ui_theme.stylesheet())
    assert "QProgressBar#calibrationProgress" in style
    assert "@background@" not in style and "@emphasis@" not in style
    assert ui_theme.CANVAS_BACKGROUND == "#000000"
    assert "#f2f5f7" not in style


def test_theme_import_does_not_choose_a_qt_binding_or_load_hardware():
    source = str(Path(__file__).resolve().parents[1] / "src")
    script = f"""
import importlib.abc
import sys
sys.path.insert(0, {source!r})
class BlockRuntime(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {{'PyQt5', 'PySide6', 'pyqtgraph', 'serial', 'rclpy', 'numpy'}}:
            raise AssertionError('Shared theme imported runtime: ' + fullname)
sys.meta_path.insert(0, BlockRuntime())
from vt_fsglove12.ui_theme import calibration_stylesheet, status_style
assert calibration_stylesheet()
assert status_style('warning')
"""
    result = subprocess.run([sys.executable, "-I", "-c", script],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("state", ["neutral", "info", "success", "warning", "error", "muted"])
def test_semantic_status_styles_use_only_neutral_shared_palette(state):
    for compact in (False, True):
        style = ui_theme.status_style(state, compact=compact)
        assert set(re.findall(r"#[0-9a-f]{6}", style)) <= set(ui_theme.PALETTE.values())
    assert ui_theme.status_style("warning") != ui_theme.status_style("success")


def test_invalid_semantic_roles_fail_instead_of_silently_hiding_status():
    with pytest.raises(ValueError, match="status"):
        ui_theme.status_style("unrecognized")
    with pytest.raises(ValueError, match="text role"):
        ui_theme.text_style("unrecognized")
