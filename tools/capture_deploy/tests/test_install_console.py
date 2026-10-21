"""Installer contract tests use inert interpreters, never install packages."""
import json
import os
from pathlib import Path
import subprocess

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "install.sh"


@pytest.mark.parametrize("value", ["relative/bin/python", "/missing-environment/bin/python"])
def test_invalid_explicit_python_stops_before_installation(tmp_path, value):
    result = subprocess.run(
        ["/bin/bash", str(SCRIPT)], cwd=tmp_path,
        env={**os.environ, "VT_CONSOLE_PYTHON": value},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 2
    assert "VT_CONSOLE_PYTHON" in result.stderr
    assert list(tmp_path.iterdir()) == []


def test_custom_interpreter_is_used_for_packages_and_desktop_registration(tmp_path):
    log = tmp_path / "calls.jsonl"
    python = tmp_path / "环境 with spaces/bin/python"
    python.parent.mkdir(parents=True)
    python.write_text(
        "#!/usr/bin/python3\nimport json, os, sys\n"
        "with open(os.environ['INSTALL_TEST_CALLS'], 'a') as stream:\n"
        "    stream.write(json.dumps(sys.argv[1:]) + '\\n')\n"
    )
    python.chmod(0o700)
    result = subprocess.run(
        ["/bin/bash", str(SCRIPT)], cwd=tmp_path,
        env={**os.environ, "VT_CONSOLE_PYTHON": str(python), "INSTALL_TEST_CALLS": str(log)},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(calls) == 2
    assert calls[0][:4] == ["-I", "-m", "pip", "install"]
    assert calls[1][:3] == ["-I", "-m", "vt_capture_console.desktop_entry"]


@pytest.mark.parametrize("arguments", [["--tracker-package"], ["--tracker-package", "/missing.vttracker"], ["--unknown"]])
def test_bad_tracker_package_arguments_stop_before_installation(tmp_path, arguments):
    result = subprocess.run(
        ["/bin/bash", str(SCRIPT), *arguments], cwd=tmp_path,
        env={**os.environ, "VT_CONSOLE_PYTHON": "/missing-interpreter"},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 2
    assert "VT_CONSOLE_PYTHON" not in result.stderr
    assert list(tmp_path.iterdir()) == []


def test_tracker_import_uses_literal_path_before_desktop_registration(tmp_path):
    log = tmp_path / "calls.jsonl"
    python = tmp_path / "python"
    python.write_text(
        "#!/usr/bin/python3\nimport json, os, sys\n"
        "with open(os.environ['INSTALL_TEST_CALLS'], 'a') as stream:\n"
        "    stream.write(json.dumps(sys.argv[1:]) + '\\n')\n"
    )
    python.chmod(0o700)
    package = tmp_path / "设备 $(do-not-run).vttracker"
    package.write_bytes(b"test fixture")
    result = subprocess.run(
        ["/bin/bash", str(SCRIPT), "--tracker-package", str(package)],
        env={**os.environ, "VT_CONSOLE_PYTHON": str(python), "INSTALL_TEST_CALLS": str(log)},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(calls) == 3
    assert calls[1] == ["-I", str(SCRIPT.parents[1] / "tracker_deploy/manage.py"),
                         "install", "--package", str(package)]
    assert calls[2][:3] == ["-I", "-m", "vt_capture_console.desktop_entry"]


def test_no_desktop_installs_selected_dependencies_without_switching_launcher(tmp_path):
    log = tmp_path / "calls.jsonl"
    python = tmp_path / "python"
    python.write_text(
        "#!/usr/bin/python3\nimport json, os, sys\n"
        "with open(os.environ['INSTALL_TEST_CALLS'], 'a') as stream:\n"
        "    stream.write(json.dumps(sys.argv[1:]) + '\\n')\n"
    )
    python.chmod(0o700)
    package = tmp_path / "fixture.vttracker"
    package.write_bytes(b"inert test package")
    result = subprocess.run(
        ["/bin/bash", str(SCRIPT), "--no-desktop", "--tracker-package", str(package)],
        env={**os.environ, "VT_CONSOLE_PYTHON": str(python), "INSTALL_TEST_CALLS": str(log)},
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 0, result.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    assert len(calls) == 2
    assert calls[0][:4] == ["-I", "-m", "pip", "install"]
    assert calls[1][-3:] == ["install", "--package", str(package)]
    assert "桌面入口等待" in result.stdout
