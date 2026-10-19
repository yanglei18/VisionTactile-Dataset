"""Standalone model preferences do not read or change workstation profiles."""
from pathlib import Path
import subprocess
import sys

import pytest
from PySide6.QtCore import QSettings

from vt_capture_console.backend.tactile_settings import TactileSettings
from vt_capture_console.backend import tactile_settings as module


@pytest.fixture
def store(tmp_path):
    return QSettings(str(tmp_path / "tactile.ini"), QSettings.Format.IniFormat)


@pytest.fixture
def assets(tmp_path):
    root = tmp_path / "trusted assets"
    (root / "models").mkdir(parents=True)
    (root / "models/MANO_RIGHT.pkl").touch()
    return root


def test_empty_settings_are_explicit_and_configuration_is_independent(store):
    settings = TactileSettings(store)
    assert settings.glove_python == settings.mano_assets == ""
    config = settings.configuration
    assert config == {"process_groups": [{"id": "right_glove", "parameters": {
        "hand": "right", "glove_python": "", "mano_assets": ""}}]}
    config["process_groups"][0]["parameters"]["glove_python"] = "/changed"
    assert settings.configuration["process_groups"][0]["parameters"]["glove_python"] == ""
    assert store.allKeys() == []


def test_cli_model_overrides_do_not_save_or_modify_existing_preferences(store):
    store.setValue("model/python", "/saved/python")
    store.setValue("model/assets", "/saved/assets")
    store.sync()
    before = Path(store.fileName()).read_bytes()
    settings = TactileSettings(store, glove_python="/override/python", mano_assets="/override/assets")
    assert settings.glove_python == "/override/python"
    assert settings.mano_assets == "/override/assets"
    assert Path(store.fileName()).read_bytes() == before
    reloaded = TactileSettings(store)
    assert reloaded.glove_python == "/saved/python" and reloaded.mano_assets == "/saved/assets"


def test_installed_static_model_takes_precedence_without_rewriting_user_settings(store, tmp_path, monkeypatch):
    resource = tmp_path / "安装目录" / "right_hand.json"
    monkeypatch.setattr(module, "bundled_model_path", lambda: str(resource))
    store.setValue("model/python", "/old-linux-environment/bin/python")
    store.sync()
    before = Path(store.fileName()).read_bytes()
    settings = TactileSettings(store)
    assert settings.uses_static_model
    assert settings.configuration["process_groups"][0]["parameters"]["preview_bundle"] == str(resource)
    override = tmp_path / "explicit.json"
    selected = TactileSettings(store, model_bundle=override)
    assert selected.model_bundle == str(override)
    assert Path(store.fileName()).read_bytes() == before


def test_missing_frozen_model_keeps_static_mode_instead_of_requesting_python(store, tmp_path, monkeypatch):
    monkeypatch.setattr(module, "__file__", str(tmp_path / "package/backend/tactile_settings.py"))
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    settings = TactileSettings(store)
    assert settings.uses_static_model
    assert settings.model_bundle == str(tmp_path / "package/assets/tactile/right_hand.json")
    assert settings.glove_python == settings.mano_assets == ""
    assert not store.allKeys()


def test_explicit_save_preserves_venv_symlink_and_only_writes_tool_store(store, assets, tmp_path):
    executable = tmp_path / "python-real"
    executable.touch()
    executable.chmod(0o700)
    venv = tmp_path / "venv python"
    venv.symlink_to(executable)
    workstation = tmp_path / "workstation.json"
    workstation.write_text('{"untouched": true}', encoding="utf-8")
    settings = TactileSettings(store)
    settings.save_model(str(venv), str(assets))
    assert settings.glove_python == str(venv) != str(executable)
    reloaded = TactileSettings(QSettings(store.fileName(), QSettings.Format.IniFormat))
    assert reloaded.glove_python == str(venv)
    assert reloaded.mano_assets == str(assets)
    assert workstation.read_text(encoding="utf-8") == '{"untouched": true}'
    assert set(store.allKeys()) == {"model/python", "model/assets"}


@pytest.mark.parametrize("bad", ["empty_python", "empty_assets", "missing_python",
                                  "nonexecutable_python", "missing_model"])
def test_invalid_model_paths_do_not_replace_previous_preferences(store, assets, tmp_path, bad):
    settings = TactileSettings(store, glove_python="/existing/python", mano_assets="/existing/assets")
    store.setValue("keep", "unchanged")
    store.sync()
    before = Path(store.fileName()).read_bytes()
    interpreter = tmp_path / "python"
    interpreter.touch()
    interpreter.chmod(0o700)
    python, models = str(interpreter), str(assets)
    if bad == "empty_python":
        python = " "
    elif bad == "empty_assets":
        models = " "
    elif bad == "missing_python":
        python = str(tmp_path / "missing-python")
    elif bad == "nonexecutable_python":
        interpreter.chmod(0o600)
    else:
        (assets / "models/MANO_RIGHT.pkl").unlink()
    with pytest.raises(ValueError):
        settings.save_model(python, models)
    assert settings.glove_python == "/existing/python" and settings.mano_assets == "/existing/assets"
    assert Path(store.fileName()).read_bytes() == before


def test_standalone_configuration_import_has_no_workstation_ros_or_device_side_effects():
    code = """import sys
from vt_capture_console.backend.tactile_settings import TactileSettings
from vt_capture_console import tactile_app
args = tactile_app.parser().parse_args(['--port', '/dev/MUST_NOT_OPEN'])
assert args.port == '/dev/MUST_NOT_OPEN'
for name in ('rclpy', 'serial', 'vt_capture_console.backend.workstation_settings',
             'vt_capture_console.backend.local_runtime', 'vt_capture_console.backend.tactile_serial'):
    assert name not in sys.modules, name
"""
    result = subprocess.run([sys.executable, "-I", "-c", code],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
