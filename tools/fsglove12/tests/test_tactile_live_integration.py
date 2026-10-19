"""Real Qt + Unix IPC + licensed MANO display integration, without devices/ROS.

Opt in with FSGLOVE12_MANO_ASSETS. Only the pose-worker acquisition boundary is
stubbed; the model, surface binding, transport, baseline, and window are real.
These tests inspect actual GL mesh data, not GPU pixels or tactile accuracy.
"""

from copy import deepcopy
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("PyQt5")
pytest.importorskip("pyqtgraph")
pytest.importorskip("OpenGL")
from PyQt5 import QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ros2_ws/src/vt_tactile_capture"))

from vt_fsglove12 import live_ui, tactile_surface
from vt_fsglove12.config import load_config
from vt_fsglove12.mano import ManoBackend
from vt_fsglove12.tactile_overlay import TactileOverlayController
from vt_tactile_capture.display_transport import LocalTactileSender


pytestmark = pytest.mark.skipif(not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="Real MANO + Qt integration requires explicit licensed FSGLOVE12_MANO_ASSETS")
NS = 1_000_000_000


class PoseWorkerBoundary:
    """Inject snapshots without starting a process or touching any device."""

    renders_geometry = True

    def __init__(self, stamp):
        self.started = self.stopped = False
        self.commands = []
        self.state = dict(runtime=dict(connected=False, received_samples=0,
            last_sample_age_s=None, published_monotonic_s=stamp, message="synthetic", error=None),
            calibration=dict(active=None, captured_steps=[], calibrated_nodes=[],
                rejected_nodes={}, warnings=[]), orientation=None, pose=None)

    def start(self): self.started = True
    def stop(self): self.stopped = True
    def join(self, timeout): return True
    def latest_snapshot(self): return deepcopy(self.state)
    def submit(self, name, **args):
        self.commands.append((name, args))
        return True


def wire_state(stamp_ns, sequence, *, pressed=False):
    adc = np.full((20, 20), 20_000, dtype=np.uint16)
    if pressed:
        adc[8:20, 12:16] += 4000  # actual index candidate addresses, not rescaled
    return SimpleNamespace(schema_version=2, rows=20, cols=20, hand="right",
        session_id="synthetic-tactile", stream_epoch=0, source_sequence=sequence,
        host_realtime_ns=1_700_000_000 * NS + stamp_ns,
        host_monotonic_ns=stamp_ns, snapshot_monotonic_ns=stamp_ns,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        connected=True, time_valid=True, max_age_ns=250_000_000,
        adc=adc.reshape(-1), valid_mask=np.ones(400, dtype=bool),
        block_monotonic_ns=np.full(160, stamp_ns, dtype=np.uint64))


def publish_pose(worker, stamp_ns, geometry, *, index=1):
    stamp = stamp_ns / NS
    worker.state["runtime"].update(connected=True, last_sample_age_s=.005,
        published_monotonic_s=stamp, received_samples=index * 12, session_id="synthetic-imu")
    worker.state["calibration"]["calibrated_nodes"] = [str(i) for i in range(1, 13)]
    pose = dict(timestamp_s=stamp, source_timestamp_s=stamp, frame_index=index,
        pose_available=True, solver_success=True, quality="experimental_partial",
        session_id="synthetic-imu", profile_version=1,
        anatomical_euler_rad=np.zeros((16, 3)).tolist(), root_quat_xyzw=[0., 0., 0., 1.],
        joint_valid_mask=[True] * 16, joint_display_available_mask=[True] * 16,
        joint_quality=["valid"] * 16, held_slots=[], unknown_slots=[], provisional_slots=[])
    worker.state.update(pose=pose, display_pose=dict(pose, display_only=True),
                        display_geometry=geometry)


@pytest.fixture(scope="module")
def real_backend():
    # A configured but broken real model is an error, never silently skipped.
    model = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    model.export_rest()
    return model


@pytest.fixture
def window_factory(monkeypatch, tmp_path, real_backend):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(live_ui, "list_serial_ports", lambda: [])
    clock = [100 * NS]
    monkeypatch.setattr(live_ui.time, "monotonic", lambda: clock[0] / NS)
    monkeypatch.setattr(live_ui.time, "monotonic_ns", lambda: clock[0])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    windows = []
    # Unix socket paths are deliberately short and the directory is mode 0700.
    with tempfile.TemporaryDirectory(prefix="vt-live-") as directory:
        endpoint = Path(directory) / "right.sock"

        def make(**kwargs):
            config = load_config()
            config["input_health"] = {"enabled": False}
            worker = PoseWorkerBoundary(clock[0] / NS)
            window = live_ui.LiveWindow(config, real_backend, output_root=tmp_path,
                port="", controller=worker, tactile_socket=str(endpoint), **kwargs)
            window.timer.stop()
            if window.tactile_timer is not None:
                window.tactile_timer.stop()
            windows.append(window)
            return window, worker

        try:
            yield make, clock, endpoint, real_backend
        finally:
            for window in windows:
                window.close()
            app.processEvents()


def test_real_live_unloaded_reference_contact_pose_update_and_stale_cleanup(window_factory):
    make, clock, endpoint, backend = window_factory
    window, worker = make()
    assert worker.started and isinstance(window.tactile_controller, TactileOverlayController)
    assert isinstance(window.view.tactile_binding, tactile_surface.TactileSurfaceBinding)
    assert window.view.tactile_binding.valid.sum() >= 360
    assert endpoint.is_socket()
    sender = LocalTactileSender(endpoint)
    try:
        window.refresh()
        assert sender.send(wire_state(clock[0], 1))
        window.refresh_tactile()
        window.tactile_reference.click()
        assert window.tactile_controller.baseline.collecting
        assert not window._tactile_pose_current and worker.commands == []
        for index in range(1, 31):
            clock[0] = 100 * NS + index * 100_000_000
            assert sender.send(wire_state(clock[0], index + 1))
            window.refresh_tactile()
        baseline = window.tactile_controller.baseline
        assert baseline.values is not None and not baseline.collecting
        assert baseline.samples_count == 31
        np.testing.assert_array_equal(baseline.values, 20_000.)
        assert worker.state["calibration"]["calibrated_nodes"] == []
        assert "参考就绪" in window.tactile_status.text()
        np.testing.assert_allclose(window.view.mesh_data.vertexColors(), window.view.colors, atol=1e-7)

        clock[0] += 10_000_000
        publish_pose(worker, clock[0], window.neutral)
        assert sender.send(wire_state(clock[0], 32, pressed=True))
        window.refresh()
        window.refresh_tactile()
        assert window._tactile_pose_current and window.view.mesh.visible()
        mesh, faces = window.view.mesh, window.view.mesh_data.faces()
        hot = window.view.mesh_data.vertexColors().copy()
        changed = np.any(np.abs(hot - window.view.colors) > 1e-5, axis=1)
        assert changed.sum() >= 10, window.tactile_status.text()
        binding = window.view.tactile_binding
        address = np.flatnonzero((binding.addresses[:, 0] == 19) & (binding.addresses[:, 1] == 12))[0]
        assert np.all(binding.vertex_regions[changed] == binding.regions[address])
        assert "非压力" in window.tactile_status.text() and "有效表面点" in window.tactile_status.text()

        # A genuine MANO pose deformation updates the existing mesh, preserving
        # contact colors on its fixed topology rather than recreating an overlay.
        angles = np.zeros((16, 3))
        angles[1:4, 2] = [.35, .6, .3]
        bent = backend.render(angles)
        previous_vertices = window.view.mesh_data.vertexes().copy()
        clock[0] += 40_000_000
        publish_pose(worker, clock[0], bent, index=2)
        window.refresh()
        np.testing.assert_array_equal(window.view.mesh_data.vertexColors(), hot)
        window.refresh_tactile()
        np.testing.assert_array_equal(window.view.mesh_data.vertexColors(), hot)
        assert not np.array_equal(window.view.mesh_data.vertexes(), previous_vertices)
        assert window.view.mesh is mesh and window.view.mesh_data.faces() is faces

        # IMU pose stays fresh while tactile delivery stops: no contact survives
        # beyond the actual tactile source age, and the complete hand stays up.
        clock[0] += 300_000_000
        publish_pose(worker, clock[0], bent, index=3)
        window.refresh()
        window.refresh_tactile()
        assert window._tactile_pose_current and window.view.mesh.visible()
        np.testing.assert_allclose(window.view.mesh_data.vertexColors(), window.view.colors, atol=1e-7)
        assert "过期" in window.tactile_status.text()
        assert window.tactile_controller.receiver.received == 32
        assert worker.commands == []  # in particular no IMU clear/calibrate/reset
        window.close()
        assert worker.stopped and not endpoint.exists()
    finally:
        sender.close()


@pytest.mark.parametrize("failure", ["replay", "binding"])
def test_optional_tactile_failure_does_not_stop_real_pose_window(window_factory, monkeypatch, failure):
    make, clock, endpoint, backend = window_factory
    kwargs = {}
    if failure == "replay":
        kwargs["replay_raw"] = "synthetic-worker-boundary.jsonl"
    else:
        def fail_binding(*_args, **_kwargs):
            raise ValueError("synthetic unsupported surface binding")
        monkeypatch.setattr(tactile_surface, "TactileSurfaceBinding", fail_binding)
    window, worker = make(**kwargs)
    assert worker.started and window.tactile_controller is None
    assert not endpoint.exists() and not window.tactile_enabled.isEnabled()
    assert "不影响手势" in window.tactile_status.text()
    publish_pose(worker, clock[0], window.neutral)
    window.refresh()
    assert window._has_display_cache and window.view.mesh.visible()
    old = window.view.mesh_data.vertexes().copy()
    angles = np.zeros((16, 3))
    angles[4:7, 2] = [.3, .5, .2]
    clock[0] += 50_000_000
    publish_pose(worker, clock[0], backend.render(angles), index=2)
    window.refresh()
    assert window.view.mesh.visible() and not np.array_equal(old, window.view.mesh_data.vertexes())
    assert worker.commands == []
