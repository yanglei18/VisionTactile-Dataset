"""The source launcher must reject the other checkout before starting the UI."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv

import pytest


@pytest.mark.skipif(sys.platform != "linux", reason="Linux source launcher")
@pytest.mark.parametrize("hand,resource", [("right", "oct29-right"), ("left", "left-hand")])
def test_launcher_ignores_shell_pythonpath_and_rejects_mixed_checkout(tmp_path, hand, resource):
    tool = tmp_path / "tools/hand_tactile_viewer"
    tool.mkdir(parents=True)
    launcher = tool / f"start_{hand}_hand.sh"
    shutil.copy2(Path(__file__).parents[1] / launcher.name, launcher)

    shutil.copy2(Path(__file__).parents[1] / "start_hand.sh", tool)
    env_dir = tmp_path / f".venv-{hand}-hand"
    venv.EnvBuilder(with_pip=False).create(env_dir)
    site = env_dir / f"lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages"
    source_dirs = []
    for name, base in (
        ("vt_hand_tactile_viewer", tool / "src"),
        ("vt_fsglove12", tmp_path / "tools/fsglove12/src"),
        ("vt_capture_console", tmp_path / "tools/capture_console/src"),
        ("vt_capture_core", tmp_path / "ros2_ws/src/vt_capture_core"),
        ("vt_tactile_capture", tmp_path / "ros2_ws/src/vt_tactile_capture"),
    ):
        package = base / name
        package.mkdir(parents=True)
        (package / "__init__.py").touch()
        source_dirs.append(str(base))
    entry = "__main__.py" if hand == "right" else "left.py"
    (tool / "src/vt_hand_tactile_viewer" / entry).write_text(
        "import sys\nprint(sys.flags.isolated, *sys.argv[1:])\n")
    other = tmp_path / "other"
    (other / "vt_fsglove12").mkdir(parents=True)
    (other / "vt_fsglove12/__init__.py").touch()
    editable = site / "checkout.pth"
    editable.write_text("\n".join(source_dirs) + "\n")
    env = {**os.environ, "PYTHONPATH": str(other)}
    args = [str(launcher), "--language", "en"]
    result = subprocess.run(args, env=env, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == f"1 --resources {tmp_path}/.runtime/{resource} --language en"

    editable.write_text(str(other) + "\n" + "\n".join(source_dirs) + "\n")
    result = subprocess.run(args, env=env, capture_output=True, text=True)
    assert result.returncode != 0
    assert "vt_fsglove12" in result.stderr
    assert not result.stdout
