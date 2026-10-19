"""Preferences belonging only to the standalone tactile tool, not the workbench."""
from __future__ import annotations

import os
from pathlib import Path
import sys

from PySide6.QtCore import QSettings


def bundled_model_path():
    """Find package data from wheels/frozen apps, independent of cwd or PATH."""
    path = Path(__file__).resolve().parents[1] / "assets" / "tactile" / "right_hand.json"
    # An incomplete installation must show a repairable resource error rather
    # than asking its user to install a separate Python/model environment.
    return str(path) if path.is_file() or getattr(sys, "frozen", False) else ""


class TactileSettings:
    def __init__(self, settings=None, *, glove_python=None, mano_assets=None, model_bundle=None):
        self.store = settings if settings is not None else QSettings(
            QSettings.Format.IniFormat, QSettings.Scope.UserScope, "visiontactile", "tactile-3d")
        self.glove_python = str(glove_python if glove_python is not None
                                else self.store.value("model/python", ""))
        self.mano_assets = str(mano_assets if mano_assets is not None
                              else self.store.value("model/assets", ""))
        self.model_bundle = (str(Path(model_bundle).expanduser().absolute()) if model_bundle is not None
                             else bundled_model_path())

    @property
    def uses_static_model(self):
        return bool(self.model_bundle)

    @property
    def configuration(self):
        # Adapt to the existing static model loader without reading workstation
        # files, loading a device profile, or requiring a repository checkout.
        parameters = {"hand": "right", "glove_python": self.glove_python, "mano_assets": self.mano_assets}
        if self.uses_static_model:
            parameters["preview_bundle"] = self.model_bundle
        return {"process_groups": [{"id": "right_glove", "parameters": parameters}]}

    def save_model(self, glove_python, mano_assets):
        if not glove_python.strip() or not mano_assets.strip():
            raise ValueError("请选择模型 Python 和 MANO 资源目录。")
        interpreter = Path(glove_python).expanduser().absolute()
        assets = Path(mano_assets).expanduser().absolute()
        if not interpreter.is_file() or not os.access(interpreter, os.X_OK):
            raise ValueError("模型 Python 不存在或不可执行。")
        if not (assets / "models/MANO_RIGHT.pkl").is_file():
            raise ValueError("资源目录中缺少 models/MANO_RIGHT.pkl。")
        # Retain the interpreter symlink so its venv remains selected.
        self.glove_python, self.mano_assets = str(interpreter), str(assets)
        self.store.setValue("model/python", self.glove_python)
        self.store.setValue("model/assets", self.mano_assets)
        self.store.sync()
        if self.store.status() != QSettings.Status.NoError:
            raise OSError("无法保存触觉工具的模型设置。")
