import os
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_capture_console.backend.tool_launcher import ToolLauncher


def deployed(tmp_path):
    root = tmp_path / "workspace"
    for relative in ("tools/tracker_camera_calibration/run_gui.sh", ".venv-calibration/bin/python",
                     ".venv-calibration/bin/vt-tracker-camera-calibrate-gui"):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture")
        path.chmod(0o700)
    return root


def test_existing_gui_recipe_no_shell_interpolation_or_sensor_launch(tmp_path, monkeypatch):
    calls, state = [], {"code": None}
    def launch(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(poll=lambda: state["code"])
    monkeypatch.setattr("subprocess.Popen", launch)
    monkeypatch.setenv("QT_PLUGIN_PATH", "/wrong/pyside")
    root = deployed(tmp_path)
    launcher = ToolLauncher(root, tmp_path / "private")
    assert calls == []
    assert launcher.start_calibration()
    assert not launcher.start_calibration()
    assert len(calls) == 1
    argv, kwargs = calls[0]
    assert argv == ("/bin/bash", str(root / "tools/tracker_camera_calibration/run_gui.sh"))
    assert kwargs["start_new_session"] and "shell" not in kwargs
    assert "QT_PLUGIN_PATH" not in kwargs["env"]
    assert launcher.log_path.stat().st_mode & 0o777 == 0o600
    assert launcher.take_exit() is None
    state["code"] = 2
    assert launcher.take_exit() == 2
    assert launcher.take_exit() is None


def test_missing_calibration_environment_never_spawns(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("No process when the GUI is not installed")
    monkeypatch.setattr("subprocess.Popen", forbidden)
    launcher = ToolLauncher(tmp_path, tmp_path / "private")
    with pytest.raises(ValueError, match="一次性安装"):
        launcher.start_calibration()
    assert not (tmp_path / "private").exists()
