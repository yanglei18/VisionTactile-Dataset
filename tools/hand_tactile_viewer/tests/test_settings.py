"""Real settings persistence and isolation from the existing tactile product."""
import json
import os
from pathlib import Path
import subprocess
import sys

from PySide6.QtCore import QSettings
import pytest

from vt_hand_tactile_viewer.settings import Settings


def test_language_and_device_preferences_persist_and_invalid_language_is_not_saved(tmp_path):
    path = tmp_path / "hand-motion-touch.ini"
    first = Settings(QSettings(str(path), QSettings.IniFormat))
    first.save_language("en")
    first.set("imu/port", "COM7")
    before = path.read_bytes()
    with pytest.raises(ValueError, match="Unsupported language"):
        first.save_language("fr")
    assert path.read_bytes() == before
    second = Settings(QSettings(str(path), QSettings.IniFormat))
    assert second.language == "en"
    assert second.get("imu/port") == "COM7"
    second.save_language("zh_CN")
    assert Settings(QSettings(str(path), QSettings.IniFormat)).language == "zh_CN"
    second.set("language", "invalid external preference")
    assert second.language in ("zh_CN", "en")


def test_preferences_write_failure_is_reported(tmp_path):
    blocked = tmp_path / "blocked-parent"
    blocked.write_text("a file cannot hold a settings directory", encoding="utf-8")
    settings = Settings(QSettings(str(blocked / "preferences.ini"), QSettings.IniFormat))
    with pytest.raises(OSError, match="Cannot save"):
        settings.save_language("en")


def test_default_product_preferences_and_response_files_do_not_modify_tactile_3d(tmp_path):
    # A fresh process gives QSettings the new user's actual config directory,
    # avoiding global Qt path changes in the window test process.
    script = r'''
import json
from pathlib import Path
from vt_hand_tactile_viewer.settings import Settings
from vt_capture_console.backend.tactile_settings import TactileSettings
from vt_tactile_capture.display_response import ResponseSettings
old = TactileSettings()
old.store.setValue("language", "zh_CN")
old.store.setValue("serial/port", "old-port")
old.store.sync()
old_path = Path(old.store.fileName())
old_bytes = old_path.read_bytes()
old_references = old.reference_store()
old_references.save_default_settings(ResponseSettings(display_range_adc=2048.))
old_reference = old_references.directory / "default-settings.json"
old_reference_bytes = old_reference.read_bytes()
new = Settings()
new.save_language("en")
new.set("imu/port", "new-port")
new_references = new.reference_store(Path("not-loaded-preset.json"))
new_references.save_default_settings(ResponseSettings(display_range_adc=4096.))
assert old_path.read_bytes() == old_bytes
assert old_reference.read_bytes() == old_reference_bytes
assert new.get("serial/port") is None
assert old.store.value("imu/port") is None
assert Path(new.store.fileName()) != old_path
assert new_references.directory != old_references.directory
assert new_references.directory.is_relative_to(new.data_root)
print(json.dumps(dict(old=str(old_path), new=new.store.fileName(), language=Settings().language,
                     new_reference=str(new_references.directory / "default-settings.json"))))
'''
    # QSettings uses an INI format here on both platforms; setPath in the child
    # also keeps this test isolated on Windows where XDG_CONFIG_HOME is unused.
    script = ("from PySide6.QtCore import QSettings\n"
              f"QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, {str(tmp_path)!r})\n" + script)
    result = subprocess.run([sys.executable, "-c", script], text=True, capture_output=True,
                            env={**os.environ, "XDG_CONFIG_HOME": str(tmp_path)}, timeout=20)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert Path(report["old"]).stem == "tactile-3d"
    assert Path(report["new"]).stem == "hand-motion-touch"
    assert report["language"] == "en"
    assert Path(report["new_reference"]).is_file()
