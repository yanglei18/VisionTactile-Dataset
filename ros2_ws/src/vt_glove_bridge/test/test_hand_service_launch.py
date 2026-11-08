"""Optional real-model launch smoke test; no ports or physical devices opened."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


@pytest.mark.parametrize("hand", ["left", "right"])
def test_headless_launch_starts_before_calibration_and_cleans_up(tmp_path, hand):
    resources = os.environ.get(f"VT_TEST_{hand.upper()}_RESOURCES")
    python = os.environ.get(f"VT_TEST_{hand.upper()}_PYTHON")
    if not resources or not python:
        pytest.skip("requires explicit real hand resources and numerical interpreter")
    from vt_fsglove12.hand_service_protocol import request
    control, pose = tmp_path / "control.sock", tmp_path / "pose.sock"
    command = ["ros2", "launch", "vt_glove_bridge", "hand_service.launch.py",
               f"hand:={hand}", f"glove_python:={python}", f"resources:={resources}",
               f"control_socket:={control}", f"socket_path:={pose}",
               f"output_root:={tmp_path / 'sessions'}"]
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    log = tmp_path / "launch.log"
    with log.open("w") as output:
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=environment)
    try:
        deadline, reply = time.monotonic() + 45, {}
        while time.monotonic() < deadline:
            assert process.poll() is None, log.read_text()
            try:
                reply = request(str(control), dict(schema_version=1, op="snapshot", hand=hand), timeout=.5)
            except (OSError, ValueError):
                pass
            if reply.get("service_state") == "ready":
                break
            time.sleep(.1)
        assert reply.get("service_state") == "ready", log.read_text()
        assert reply["pose_valid"] is False
        assert reply["calibration_busy"] is False
        runtime = reply["snapshot"]["runtime"]
        assert runtime["connected"] is False
        assert runtime["received_samples"] == 0
        assert pose.exists()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
                pytest.fail("headless hand launch did not shut down within 12 s")
    assert process.returncode == 0, log.read_text()
    assert not control.exists()
    assert not pose.exists()
