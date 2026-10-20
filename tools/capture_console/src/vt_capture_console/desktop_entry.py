"""Install a per-user application launcher; no autostart or system service."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys


def _quoted(value):
    value = str(value)
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ValueError("launcher paths must not contain control characters")
    # Exec argument quoting is decoded after desktop key-file string escapes.
    # Both layers are required; percent signs are desktop field codes.
    for char in ("\\", '"', "`", "$"):
        value = value.replace(char, "\\" + char)
    return '"' + value.replace("\\", "\\\\").replace("%", "%%") + '"'


def launcher_text(repo_root, *, python=None):
    root = Path(repo_root).resolve()
    script = root / "tools/capture_deploy/start_workbench.sh"
    if not script.is_file():
        raise ValueError("selected folder does not contain the desktop workbench launcher")
    command = f"/bin/bash {_quoted(script)}"
    if python is not None:
        # Preserve a venv's invocation path: resolving its symlink may select
        # the system interpreter, which does not have the installed packages.
        interpreter = Path(python).expanduser()
        if not interpreter.is_absolute() or not interpreter.is_file() or not os.access(interpreter, os.X_OK):
            raise ValueError("installed Python must be an existing executable absolute path")
        command = f"/usr/bin/env {_quoted('VT_CONSOLE_PYTHON=' + str(interpreter))} {command}"
    icon = root / "tools/capture_console/src/vt_capture_console/theme/icons/orbit.svg"
    icon_value = str(icon).replace("\\", "\\\\")
    return ("[Desktop Entry]\nType=Application\nVersion=1.0\n"
            "Name=Vision Tactile Workbench\nName[zh_CN]=Vision Tactile 采集工作台\n"
            "Comment=Configure devices, preview sensors and manage recordings\n"
            "Comment[zh_CN]=图形化设备配置、传感器预览与录制管理\n"
            f"Exec={command}\nIcon={icon_value}\n"
            "Terminal=false\nCategories=Science;\nStartupNotify=true\n"
            "StartupWMClass=Vision Tactile Capture Console\n")


def install(repo_root, *, applications_dir=None, desktop_dir=None, python=None):
    applications = Path(applications_dir) if applications_dir else Path.home() / ".local/share/applications"
    content = launcher_text(repo_root, python=python)
    targets = [applications / "visiontactile-capture.desktop"]
    if desktop_dir is not None:
        desktop = Path(desktop_dir)
        if not desktop.is_dir():
            raise ValueError("desktop directory does not exist")
        targets.append(desktop / "Vision Tactile.desktop")
    for target in targets:
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.is_symlink():
            raise ValueError("refusing to replace a symbolic-link launcher")
        flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW
        fd = os.open(target, flags, 0o755)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(content)
        target.chmod(0o755)
    return targets


def main(argv=None):
    parser = argparse.ArgumentParser(description="Install the desktop workbench entry for the current user")
    parser.add_argument("--repo-root", required=True)
    parser.add_argument("--desktop", action="store_true", help="Also place a launcher on the configured desktop")
    args = parser.parse_args(argv)
    desktop = None
    if args.desktop:
        try:
            result = subprocess.run(["xdg-user-dir", "DESKTOP"], capture_output=True, text=True, timeout=3)
            candidate = Path(result.stdout.strip())
            if result.returncode == 0 and candidate.is_dir() and candidate != Path.home():
                desktop = candidate
        except (OSError, subprocess.TimeoutExpired):
            pass
    for path in install(args.repo_root, desktop_dir=desktop, python=sys.executable):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
