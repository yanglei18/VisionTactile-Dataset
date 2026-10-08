"""The preview exporter only materializes neutral model geometry."""
import json
import subprocess
import sys

import numpy as np

from vt_fsglove12 import tactile_preview_export as module


def test_module_import_does_not_load_gui_serial_ros_or_torch():
    script = """
import sys
import vt_fsglove12.tactile_preview_export
for prefix in ('torch', 'manotorch', 'PyQt5', 'PySide6', 'rclpy', 'serial'):
    assert not any(name == prefix or name.startswith(prefix + '.') for name in sys.modules), prefix
"""
    subprocess.run([sys.executable, "-I", "-c", script], check=True, timeout=10)


def test_export_uses_only_explicit_hand_neutral_pose_and_json_stdout(monkeypatch, capsys):
    from vt_fsglove12 import guided_calibration, mano

    calls = []

    class Backend:
        def __init__(self, assets, hand):
            calls.append((assets, hand))
            print("backend diagnostic")

        def render(self, pose):
            np.testing.assert_array_equal(pose, np.zeros((16, 3)))
            return {"vertices_m": [[0, 0, 0]], "faces": [[0, 0, 0]], "joints_m": [[1, 2, 3]],
                    "provenance": {"private_path": "must not leak"}, "mano_pose_rotvec": "not exported"}

    def axes(joints, hand):
        assert joints == [[1, 2, 3]]
        return {"hand": hand}

    monkeypatch.setattr(mano, "ManoBackend", Backend)
    monkeypatch.setattr(guided_calibration, "model_motion_axes", axes)
    assert module.main(["--mano-assets", "trusted-assets", "--hand", "left"]) == 0
    output = capsys.readouterr()
    value = json.loads(output.out)
    assert calls == [("trusted-assets", "left")]
    assert value == {"schema_version": 1, "neutral_geometry": {
        "vertices_m": [[0, 0, 0]], "faces": [[0, 0, 0]], "joints_m": [[1, 2, 3]],
        "provenance": {"hand": "left"}}, "model_axes": {"hand": "left"}}
    assert "backend diagnostic" in output.err
    assert "must not leak" not in output.out


def test_export_failure_does_not_emit_partial_json(monkeypatch, capsys):
    def broken(*args):
        raise RuntimeError("unavailable model")

    monkeypatch.setattr(module, "export_preview", broken)
    assert module.main(["--mano-assets", "missing", "--hand", "right"]) == 1
    output = capsys.readouterr()
    assert output.out == ""
    assert "unavailable model" in output.err


def test_export_nonfinite_payload_is_rejected(monkeypatch, capsys):
    monkeypatch.setattr(module, "export_preview", lambda *args: {"bad": float("nan")})
    assert module.main(["--mano-assets", "missing", "--hand", "right"]) == 1
    assert capsys.readouterr().out == ""
