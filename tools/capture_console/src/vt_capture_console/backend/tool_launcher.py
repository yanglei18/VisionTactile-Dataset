"""Launch existing calibration GUIs separately, never import their Qt or ROS."""
from __future__ import annotations

import os
from pathlib import Path
import subprocess
import time
import uuid

from .workstation_settings import private_directory


class ToolLauncher:
    def __init__(self, repo_root, log_dir):
        self.repo_root = Path(repo_root)
        self.log_dir = Path(log_dir)
        self._process = None
        self.log_path = None

    def start_calibration(self):
        if self._process is not None and self._process.poll() is None:
            return False
        script = self.repo_root / "tools/tracker_camera_calibration/run_gui.sh"
        python = self.repo_root / ".venv-calibration/bin/python"
        entry = self.repo_root / ".venv-calibration/bin/vt-tracker-camera-calibrate-gui"
        if not script.is_file() or not all(path.is_file() and os.access(path, os.X_OK) for path in (python, entry)):
            raise ValueError("相机–Tracker 标定环境尚未部署，请完成该工具的一次性安装；不会自动安装或启动设备。")
        directory = private_directory(self.log_dir)
        self.log_path = directory / f"extrinsics-{time.time_ns()}-{uuid.uuid4().hex[:8]}.log"
        fd = os.open(self.log_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        env = dict(os.environ)
        # Inherited PySide plugin paths must not leak into the older GUI's Qt.
        for name in ("PYTHONHOME", "PYTHONPATH", "VIRTUAL_ENV", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
            env.pop(name, None)
        try:
            self._process = subprocess.Popen(
                ("/bin/bash", str(script)), cwd=self.repo_root, env=env,
                stdin=subprocess.DEVNULL, stdout=fd, stderr=subprocess.STDOUT,
                start_new_session=True, close_fds=True,
            )
        finally:
            os.close(fd)
        return True

    def take_exit(self):
        if self._process is None:
            return None
        code = self._process.poll()
        if code is None:
            return None
        self._process = None
        return code

    # No close/kill hook: closing the workbench must not kill a calibration
    # window, which may now own a user-started recording or computation.
