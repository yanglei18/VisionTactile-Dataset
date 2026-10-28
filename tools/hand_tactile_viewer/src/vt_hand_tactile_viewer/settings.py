"""Preferences and references isolated from the pure tactile viewer."""
from pathlib import Path

from PySide6.QtCore import QLocale, QSettings


class Settings:
    def __init__(self, store=None):
        self.store = store if store is not None else QSettings(
            QSettings.IniFormat, QSettings.UserScope, "visiontactile", "hand-motion-touch")
        self.data_root = Path(self.store.fileName()).parent / "hand-motion-touch"

    @property
    def language(self):
        default = "zh_CN" if QLocale.system().language() == QLocale.Chinese else "en"
        value = self.get("language", default)
        return value if value in ("zh_CN", "en") else default

    def save_language(self, value):
        if value not in ("zh_CN", "en"):
            raise ValueError("Unsupported language")
        self.set("language", value)

    def get(self, key, default=None):
        return self.store.value(key, default)

    def set(self, key, value):
        self.store.setValue(key, value)
        self.store.sync()
        if self.store.status() != QSettings.NoError:
            raise OSError("Cannot save Hand Motion & Touch preferences")

    def reference_store(self, preset_path):
        from vt_capture_console.backend.tactile_reference_store import TactileReferenceStore
        return TactileReferenceStore(self.data_root / "tactile-references", preset_path=preset_path)
