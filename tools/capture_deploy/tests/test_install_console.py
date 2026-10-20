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
