"""Native runtime smoke checks, isolated from Linux workstation test fixtures.

This also runs on Linux to detect accidental coupling early. The Windows CI
job executes it against regular package installations, without MANO or torch.
No real serial device or external model process is opened.
"""
from __future__ import annotations

import os
import subprocess
import sys
import textwrap


def test_installed_runtime_resources_and_static_mode_need_no_workstation(tmp_path):
    code = textwrap.dedent(r'''
        import importlib.abc
        import sys
        from pathlib import Path

        class RuntimeBoundary(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                excluded = (
                    "rclpy", "vt_capture_manager", "vt_capture_core", "torch", "manotorch",
                    "vt_capture_console.ui.main_window",
                    "vt_capture_console.backend.local_runtime",
                    "vt_capture_console.backend.workstation_settings",
                    "vt_capture_console.backend.pose_preview",
                    "vt_capture_console.ui.widgets.glove_3d",
                    "vt_capture_console.ui.widgets.camera_gl",
                    "vt_capture_console.ui.gpu_status",
                )
                if sys.platform == "win32":
                    excluded += ("fcntl", "termios", "pty", "resource")
                if any(fullname == name or fullname.startswith(name + ".") for name in excluded):
                    raise AssertionError("standalone runtime imported " + fullname)
                return None

        sys.meta_path.insert(0, RuntimeBoundary())

        import serial
        def no_serial(*args, **kwargs):
            raise AssertionError("runtime smoke must not open physical devices")
        serial.Serial = no_serial

        from PySide6.QtCore import QSettings
        from vt_capture_console import tactile_app
        from vt_capture_console.backend.tactile_installation import _resource_details, _runtime_details
        from vt_capture_console.backend.tactile_settings import TactileSettings
        from vt_capture_console.ui.tactile_window import TactileWindow

        runtime = _runtime_details()
        resources = _resource_details()
        assert runtime["qt_binding"] == "PySide6"
        assert runtime["renderer"] == "vt_fsglove12.hand_view.HandView"
        assert runtime["serial_device"] == "not_opened"
        assert resources["theme_icons"] > 0
        assert resources["translation_entries"] > 0
        assert resources["layout_addresses"] > 0
        assert callable(TactileWindow)

        root = Path(sys.argv[1])
        store = QSettings(str(root / "viewer.ini"), QSettings.Format.IniFormat)
        model = root / "静态资源 with spaces" / "right_hand.json"
        settings = TactileSettings(store, model_bundle=model)
        assert settings.uses_static_model
        assert settings.glove_python == settings.mano_assets == ""
        assert not store.allKeys()
        params = settings.configuration["process_groups"][0]["parameters"]
        assert params["preview_bundle"] == str(model)
        args = tactile_app.parser().parse_args(["--port", "COM12"])
        assert args.port == "COM12"
        print("standalone runtime resources verified")
    ''')
    environment = dict(os.environ, PYTHONUTF8="1", PYQTGRAPH_QT_LIB="PySide6",
                       QT_QPA_PLATFORM="offscreen")
    result = subprocess.run([sys.executable, "-X", "utf8", "-I", "-c", code, str(tmp_path)],
                            cwd=tmp_path, env=environment, capture_output=True,
                            text=True, encoding="utf-8", timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "standalone runtime resources verified" in result.stdout
