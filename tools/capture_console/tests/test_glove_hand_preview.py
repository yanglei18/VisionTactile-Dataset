"""Gesture UI integration without model inference, OpenGL, ROS, or hardware."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtTest import QSignalSpy, QTest

from vt_capture_console.backend.pose_preview import key_for
from vt_capture_console.ui import main_window
from vt_capture_console.ui.main_window import DeviceCard
from vt_capture_console.ui.widgets import glove_3d
from vt_capture_console.ui.widgets.preview import PreviewWidget


def mano(hand="right", **changes):
    result = dict(pose=[.12, -.23, .34] + [.025] * 45, shape=[.15] * 10, hand=hand,
                  session_id="session-1", calibration_version=3, frame_index=20,
                  source_realtime_ns=10_000_000_000, source_mode="live",
                  model_rest_sha256="a" * 64, config_sha256="b" * 64)
    result.update(changes)
    return result


def glove_device(hand="right", *, payload=None, **changes):
    result = dict(id=f"{hand}_glove", kind="glove", side=hand, label=f"{hand} hand",
                  topic=f"/gloves/{hand}/state", status="online", receive_hz=100., valid_hz=100.,
                  age_s=.01, glove_action_enabled=True, glove_hint="使用原标定工具",
                  preview=payload if payload is not None else {
                      "joints": [[i * .005, i * .002, i % 4 * .003] for i in range(21)],
                      "valid": True, "mano": mano(hand)})
    result.update(changes)
    return result


@pytest.fixture
def fake_canvas(monkeypatch):
    class Canvas(QtWidgets.QWidget):
        def __init__(self, hand, parent=None):
            super().__init__(parent)
            self.hand = hand
            self.samples = []
            self.disposed = False

        def set_sample(self, frame, **context):
            self.samples.append((frame, context))

        def dispose(self):
            self.disposed = True
            self.hide()
            self.deleteLater()

    monkeypatch.setattr(glove_3d, "Glove3DWidget", Canvas)
    return Canvas


@pytest.fixture
def widgets(qapp):
    created = []

    def create(widget):
        created.append(widget)
        return widget

    yield create
    for widget in created:
        if isinstance(widget, DeviceCard):
            widget.close_glove_detail()
        widget.close()
        widget.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.Type.DeferredDelete)
    qapp.processEvents()


def click(button):
    QTest.mouseClick(button, QtCore.Qt.MouseButton.LeftButton)


def render(widget):
    widget.resize(500, 420)
    image = QtGui.QImage(widget.size(), QtGui.QImage.Format.Format_ARGB32)
    image.fill(QtGui.QColor("white"))
    widget.render(image)
    return image


def test_card_default_real_3d_and_mode_switch_leave_received_pose_unchanged(fake_canvas, widgets):
    raw = glove_device()
    card = widgets(DeviceCard(raw))
    assert card.preview.glove_view == "3d"
    assert card.glove_controls.three_d_button.isChecked()
    assert card.preview.height() == 256
    frame = card.preview._frame
    raw["preview"]["mano"]["pose"][0] = 999
    raw["preview"]["mano"]["shape"][0] = 999
    raw["preview"]["joints"][1][0] = 999
    assert frame["mano"]["pose"][0] == .12
    assert frame["mano"]["shape"][0] == .15
    assert frame["joints"][1][0] == .005
    for button, expected in ((card.glove_controls.skeleton_button, "skeleton"),
                             (card.glove_controls.three_d_button, "3d")):
        click(button)
        assert card.preview.glove_view == expected and card.preview._frame is frame
    with pytest.raises(ValueError):
        card.preview.set_glove_view("matrix")


def test_demo_card_stays_skeleton_and_never_fabricates_mano(fake_canvas, widgets):
    card = widgets(DeviceCard(glove_device(payload={"demo": True, "phase": .2})))
    assert card.preview.glove_view == "skeleton" and card.preview._glove_3d is None
    assert "mano" not in card.preview._frame
    click(card.glove_controls.three_d_button)
    _, context = card.preview._glove_3d.samples[-1]
    assert context["mode"] == "demo"
    assert "mano" not in card.preview._frame


def test_card_and_detail_are_singleton_synced_with_explicit_hand_calibration_signal(fake_canvas, widgets):
    card = widgets(DeviceCard(glove_device("left")))
    requested = QSignalSpy(card.calibration_requested)
    card.open_glove_detail()
    dialog = card.glove_dialog
    assert dialog.preview.glove_view == card.preview.glove_view == "3d"
    assert dialog.preview._frame == card.preview._frame
    assert dialog.preview._frame is not card.preview._frame
    card.open_glove_detail()
    assert card.glove_dialog is dialog
    click(dialog.controls.skeleton_button)
    assert dialog.preview.glove_view == card.preview.glove_view == "skeleton"
    assert card.glove_controls.skeleton_button.isChecked()
    click(card.glove_controls.three_d_button)
    assert dialog.preview.glove_view == card.preview.glove_view == "3d"
    assert dialog.controls.three_d_button.isChecked()
    click(card.glove_controls.calibration_button)
    click(dialog.controls.calibration_button)
    assert requested.count() == 2
    assert requested.at(0) == requested.at(1) == ["left"]
    card.update_device(glove_device("left", glove_action_enabled=False))
    assert not card.glove_controls.calibration_button.isEnabled()
    assert not dialog.controls.calibration_button.isEnabled()


@pytest.mark.parametrize("status", ["stale", "offline", "unknown"])
def test_held_snapshot_survives_detail_open_and_mode_changes_without_new_pose(fake_canvas, widgets, status):
    card = widgets(DeviceCard(glove_device()))
    frame = card.preview._frame
    newer = glove_device(status=status)
    newer["preview"]["mano"]["pose"][0] = .9
    card.update_device(newer)
    assert card.preview._held and card.preview._frame is frame
    card.open_glove_detail()
    detail = card.glove_dialog.preview
    assert detail._held and detail._frame == frame
    _, context = detail._glove_3d.samples[-1]
    assert context["held"] is True
    click(card.glove_dialog.controls.skeleton_button)
    click(card.glove_controls.three_d_button)
    assert detail._held and card.preview._held and detail._frame["mano"]["pose"][0] == .12


def test_invalid_pose_retains_history_and_mano_only_pose_has_no_fake_skeleton(fake_canvas, widgets):
    preview = widgets(PreviewWidget("glove"))
    preview.set_device(glove_device())
    previous = preview._frame
    preview.set_device(glove_device(payload={"valid": False, "joints": [], "reason": "not_calibrated"}))
    assert preview._held and preview._frame is previous
    preview.set_device(glove_device(payload={"valid": True, "joints": [], "mano": mano(frame_index=21)}))
    assert not preview._held
    assert preview._frame["joints"] == preview._frame["relative"] == ()
    assert not render(preview).isNull()
    preview.set_glove_view("3d")
    assert preview._glove_3d.samples[-1][0]["joints"] == ()


def test_legacy_joint_only_data_remains_renderable_without_requesting_a_model(fake_canvas, widgets):
    raw = glove_device()
    del raw["preview"]["mano"]
    card = widgets(DeviceCard(raw))
    click(card.glove_controls.skeleton_button)
    assert card.preview.glove_view == "skeleton"
    assert len(card.preview._frame["relative"]) == 21
    assert "mano" not in card.preview._frame and not card.preview._held
    assert not render(card.preview).isNull()


def action_window(monkeypatch, *, can_focus=False, can_start=False, runtime=True, accept=True):
    """Invoke the real UI action method on an inert ownership/capability seam."""
    calls, messages = [], []
    snapshot = {"synthetic_test_only": True}
    status = {"can_focus": can_focus, "can_start": can_start, "message": "不可启动",
              "group": {"id": "right_glove", "pid": 12345}}
    monkeypatch.setattr(main_window, "glove_status", lambda value, hand: status)
    monkeypatch.setattr(main_window, "focus_owned_calibration",
                        lambda group, hand: (calls.append(("focus", group["id"], hand)) or True, "已切换标定窗口"))
    backend = SimpleNamespace(snapshot=lambda: snapshot)
    if runtime:
        backend.runtime = SimpleNamespace(configuration={"process_groups": [{"id": "right_glove",
            "parameters": {"hand": "right", "launch_gui": "true"}}]})
    controls = SimpleNamespace(pending_request=None, selected_id=None,
        operation_notice=SimpleNamespace(text=lambda: "队列拒绝"),
        refresh=lambda value: calls.append(("refresh", value)))

    def submit(action):
        calls.append(("submit", controls.selected_id, action))
        if accept:
            controls.pending_request = "queued-request"

    controls.submit = submit
    window = SimpleNamespace(demo=False, backend=backend, device_controls=controls,
                             statusBar=lambda: SimpleNamespace(showMessage=lambda text, duration: messages.append(text)))
    return window, calls, messages


def test_calibration_action_focuses_owned_window_without_second_start(monkeypatch):
    window, calls, messages = action_window(monkeypatch, can_focus=True, can_start=True)
    main_window.MainWindow.open_glove_calibration(window, "right")
    assert calls == [("focus", "right_glove", "right")]
    assert messages == ["已切换标定窗口"]


@pytest.mark.parametrize("accept", [True, False])
def test_calibration_action_uses_explicit_managed_start_and_reports_actual_queue_result(monkeypatch, accept):
    window, calls, messages = action_window(monkeypatch, can_start=True, accept=accept)
    main_window.MainWindow.open_glove_calibration(window, "right")
    assert calls[-1] == ("submit", "right_glove", "start")
    assert len(calls) == 2
    assert ("已请求" in messages[-1]) is accept
    if accept:
        assert "不会自动标定" in messages[-1]
    else:
        assert messages[-1] == "队列拒绝"


@pytest.mark.parametrize("blocked", ["demo", "readonly", "no_capability", "wrong_hand", "headless"])
def test_calibration_action_does_not_launch_without_owned_desktop_configuration(monkeypatch, blocked):
    window, calls, messages = action_window(monkeypatch, can_start=blocked != "no_capability",
                                            runtime=blocked != "readonly")
    if blocked == "demo":
        window.demo = True
    elif blocked in ("wrong_hand", "headless"):
        parameters = window.backend.runtime.configuration["process_groups"][0]["parameters"]
        parameters["hand" if blocked == "wrong_hand" else "launch_gui"] = "left" if blocked == "wrong_hand" else "false"
    main_window.MainWindow.open_glove_calibration(window, "right")
    assert calls == [] and messages


@pytest.mark.parametrize("change", [
    lambda p: p["mano"].update(hand="left"),
    lambda p: p["mano"].update(pose=[float("nan")] * 48),
    lambda p: p["mano"].update(shape=[0.] * 9),
    lambda p: p["mano"].update(source_mode="invented"),
])
def test_malformed_mano_never_replaces_last_valid_ui_snapshot(fake_canvas, widgets, change):
    preview = widgets(PreviewWidget("glove"))
    preview.set_device(glove_device())
    previous = preview._frame
    bad = glove_device()
    change(bad["preview"])
    preview.set_device(bad)
    assert preview._held and preview._preview_error
    assert preview._frame is previous


def test_side_change_disposes_renderer_clears_old_pose_and_closes_detail(fake_canvas, widgets):
    card = widgets(DeviceCard(glove_device()))
    old = card.preview._glove_3d
    card.open_glove_detail()
    card.update_device(glove_device("left", payload={}))
    assert old.disposed
    assert card.glove_dialog is None
    assert card.preview._frame is None and card.preview._glove_3d.hand == "left"
    assert card.preview._glove_3d.samples[-1][0] is None


def test_copy_requires_same_device_hand_and_preserves_replay_marker(fake_canvas, widgets):
    source, target = (widgets(PreviewWidget("glove")) for _ in range(2))
    data = glove_device()
    data["preview"].update(replay=True)
    data["preview"]["mano"]["source_mode"] = "replay"
    source.set_device(data)
    target.copy_glove_state_from(source)
    assert target._mode == "replay" and target._frame == source._frame
    target.set_device(glove_device("left"))
    with pytest.raises(ValueError, match="same device"):
        target.copy_glove_state_from(source)


class FakePosePreview(QtCore.QObject):
    initialized = QtCore.Signal(object)
    ready = QtCore.Signal(object)
    failed = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.bundle = self.geometry = None
        self.submissions = []
        self.clears = self.retries = 0

    def submit(self, mano, joints=None):
        self.submissions.append((deepcopy(mano), deepcopy(joints)))

    def clear(self):
        self.clears += 1
        self.geometry = None

    def retry(self):
        self.retries += 1
        self.clear()


class FakeHandView(QtWidgets.QWidget):
    """Geometry sink: any transform/neutral substitution remains observable."""
    def __init__(self, parent):
        super().__init__(parent)
        self.vertices = np.array([[0., 0., 0.], [.05, .1, .02], [-.03, .14, -.04]])
        self.mesh_data = SimpleNamespace(vertexes=lambda: self.vertices)
        self.opts = {"fov": 60.}
        self.received = []
        self.hand_visible = False

    def set_geometry(self, geometry):
        self.received.append(geometry)
        self.vertices = geometry["vertices_m"]
        self.hand_visible = True

    def set_hand_visible(self, value):
        self.hand_visible = value

    def set_show_joints(self, checked):
        self.show_joints = checked


@pytest.fixture
def renderer(qapp, monkeypatch, widgets):
    worker = FakePosePreview()
    monkeypatch.setattr(glove_3d, "pose_preview", lambda hand: worker)
    widget = widgets(glove_3d.Glove3DWidget("right"))
    return widget, worker


def sample(source=None):
    return {"mano": source or mano(), "joints": glove_device()["preview"]["joints"]}


def result_for(source, offset=0.):
    return {"key": key_for(source), "geometry": {
        "vertices_m": np.array([[.1, .2, -.05], [-.2, .3, .1], [.15, -.1, .3]]) + offset,
        "joints_m": np.arange(63, dtype=float).reshape(21, 3) / 1000.}}


def attach_view(widget):
    view = FakeHandView(widget)
    widget.view = view
    widget.layout.addWidget(view)
    return view


@pytest.mark.parametrize("frame, held, mode", [
    (None, False, "live"), (None, True, "live"),
    ({"joints": [(0., 0., 0.)] * 21}, False, "live"),
    (sample(), True, "live"), (sample(), False, "demo"),
])
def test_no_inference_or_neutral_hand_before_valid_current_mano(renderer, frame, held, mode):
    widget, worker = renderer
    widget.set_sample(frame, held=held, mode=mode, device=glove_device())
    assert worker.submissions == []
    assert widget.view is None and widget._display_key is None
    assert widget._invalid and widget._desired_key is None
    assert not widget.message.isHidden()
    assert not widget.findChildren(QtCore.QProcess)


def test_valid_request_preserves_root_encoded_pose_and_received_geometry(renderer):
    widget, worker = renderer
    view = attach_view(widget)
    source = mano()
    before = deepcopy(source)
    frame = sample(source)
    widget.set_sample(frame, held=False, mode="live", device=glove_device())
    assert worker.submissions == [(before, frame["joints"])]
    assert view.received == []  # No manufactured neutral result while pending.
    result = result_for(source)
    vertices = result["geometry"]["vertices_m"].copy()
    worker.ready.emit(result)
    assert view.received == [result["geometry"]]
    np.testing.assert_array_equal(view.vertices, vertices)
    assert source == before and widget._display_key == key_for(source)
    assert widget.message.isHidden()


def test_late_result_after_hold_is_ignored_and_recovery_waits_for_current_key(renderer):
    widget, worker = renderer
    view = attach_view(widget)
    first, second = mano(), mano(frame_index=21, source_realtime_ns=10_010_000_000)
    widget.set_sample(sample(first), held=False, mode="live", device=glove_device())
    worker.ready.emit(result_for(first))
    widget.set_sample(sample(first), held=True, mode="live", device=glove_device(status="stale"))
    assert worker.clears == 1 and widget._desired_key is None
    assert "历史姿态" in widget.message.text()
    worker.ready.emit(result_for(first, offset=.4))
    assert len(view.received) == 1
    widget.set_sample(sample(second), held=False, mode="live", device=glove_device())
    worker.ready.emit(result_for(first, offset=.4))
    assert len(view.received) == 1
    worker.ready.emit(result_for(second, offset=.2))
    assert len(view.received) == 2 and widget._display_key == key_for(second)


@pytest.mark.parametrize("change", [dict(session_id="session-2"), dict(calibration_version=4),
    dict(model_rest_sha256="c" * 64), dict(config_sha256="d" * 64), dict(source_mode="replay")])
def test_new_session_or_calibration_hides_old_mesh_until_matching_result(renderer, change):
    widget, worker = renderer
    view = attach_view(widget)
    first = mano()
    widget.set_sample(sample(first), held=False, mode="live", device=glove_device())
    worker.ready.emit(result_for(first))
    assert view.hand_visible
    newer = mano(**change)
    widget.set_sample(sample(newer), held=False, mode=newer["source_mode"], device=glove_device())
    assert not view.hand_visible and widget._display_key is None
    assert not widget.message.isHidden()
    worker.ready.emit(result_for(first))
    assert not view.hand_visible and len(view.received) == 1
    worker.ready.emit(result_for(newer))
    assert view.hand_visible and widget._display_key == key_for(newer)


@pytest.mark.parametrize("change", [dict(frame_index=None), dict(hand="left"), dict(session_id=4)])
def test_invalid_mano_identity_fences_pending_result_instead_of_recovering_from_old_frame(renderer, change):
    widget, worker = renderer
    view = attach_view(widget)
    first = mano()
    widget.set_sample(sample(first), held=False, mode="live", device=glove_device())
    widget.set_sample(sample(mano(**change)), held=False, mode="live", device=glove_device())
    assert widget._invalid and widget._desired_key is None
    assert widget._current is None
    assert len(worker.submissions) == 1
    worker.ready.emit(result_for(first))
    assert not view.received and not widget.message.isHidden()


def test_missing_model_error_can_be_retried_without_fake_geometry(renderer):
    widget, worker = renderer
    widget.set_sample(sample(), held=False, mode="live", device=glove_device())
    worker.failed.emit("缺少 MANO_RIGHT.pkl")
    assert "缺少 MANO_RIGHT.pkl" in widget.message.text()
    assert not widget.retry_button.isHidden() and widget.view is None
    click(widget.retry_button)
    assert worker.retries == 1 and len(worker.submissions) == 2
    assert widget.view is None and widget._display_key is None


def test_offscreen_initialization_keeps_model_hidden_with_actionable_skeleton_fallback(renderer):
    widget, worker = renderer
    widget.set_sample(sample(), held=False, mode="live", device=glove_device())
    worker.initialized.emit({"neutral_geometry": {}, "model_axes": {}})
    assert widget.view is None and widget._display_key is None
    assert "OpenGL" in widget.message.text() and "骨架" in widget.message.text()
    assert not widget.retry_button.isHidden()


def test_disposed_view_ignores_worker_signals(renderer):
    widget, worker = renderer
    view = attach_view(widget)
    widget.set_sample(sample(), held=False, mode="live", device=glove_device())
    widget.dispose()
    worker.ready.emit(result_for(mano()))
    worker.failed.emit("late")
    assert widget._disposed and not view.received
