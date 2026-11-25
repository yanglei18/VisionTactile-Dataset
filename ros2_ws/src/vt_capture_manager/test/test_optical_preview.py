"""Exact-stamp optical overlays on a live camera preview."""
from collections import deque
from types import SimpleNamespace

import pytest


@pytest.fixture
def viewer():
    pytest.importorskip("PyQt5")
    pytest.importorskip("rclpy")
    from vt_capture_manager.optical import viewer
    return viewer


def test_preview_prefers_completed_image_and_limits_wait(viewer):
    frames = deque([(10., 101), (10.03, 102), (10.06, 103)], maxlen=7)
    fused = {102: {"left": object()}}
    pick = viewer.select_preview_frame
    assert pick(frames, fused, 10.07, wait_for_fusion=False) == frames[-1]
    assert pick(frames, fused, 10.07, wait_for_fusion=True) is None
    fused[102]["right"] = object()
    assert pick(frames, fused, 10.08, wait_for_fusion=True) == frames[1]
    assert pick(frames, fused, 10.20, wait_for_fusion=True) == frames[-1]
    assert pick(frames, fused, 10.61, wait_for_fusion=True) is None
    busy = deque([(10 + i / 120, i) for i in range(7)], maxlen=7)
    assert pick(busy, {}, 10.06, wait_for_fusion=True) == busy[0]


def test_late_hands_can_enrich_current_image_without_rewinding(viewer, monkeypatch):
    # Exercise the actual display timer, including its monotonic drawing decision.
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    app = viewer.QtWidgets.QApplication.instance() or viewer.QtWidgets.QApplication([])
    window = viewer.Viewer(camera_name="d436", layout_path="", language="en")
    window.timer.stop()
    window.node = window.fusion_node = object()
    frame = viewer.np.zeros((64, 64, 3), dtype="uint8")
    states = dict.fromkeys(("left", "right"), SimpleNamespace(observation=None, status="unbound"))
    now = [10.]
    monkeypatch.setattr(viewer.time, "monotonic", lambda: now[0])
    msg = SimpleNamespace(joints_valid=False, wrist_valid=False, status="cube_lost")
    try:
        window.receive(frame, None, None, [], states, 1)
        now[0] = 10.13
        window.refresh()
        assert window.preview.source_geometry[-1] == 1
        window.fused[1] = {"left": msg, "right": msg}
        window.refresh()
        assert window.readout.text().count("cube_lost") == 2
        now[0] = 10.14
        window.receive(frame, None, None, [], states, 2)
        now[0] = 10.18
        window.receive(frame, None, None, [], states, 3)
        window.fused[2] = {"left": msg, "right": msg}
        window.refresh()
        assert window.preview.source_geometry[-1] == 2
        now[0] = 10.32
        window.refresh()
        assert window.preview.source_geometry[-1] == 3
        assert "cube_lost" not in window.readout.text()
        now[0] = 10.70
        window.refresh()
        assert window.preview.source_geometry is None
    finally:
        window.node = window.fusion_node = None
        window.close()
        app.processEvents()
