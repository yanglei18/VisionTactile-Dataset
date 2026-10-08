"""Offscreen tests use fake sources only; never enumerate or open real hardware."""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("PyQt5")
from PyQt5 import QtWidgets

from vt_glove_pose import gui
from vt_glove_pose.canvas import HandCanvas


@pytest.fixture(scope="module")
def app():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["gui-tests"])
    application.setQuitOnLastWindowClosed(False)
    yield application


class FakeEngine:
    def __init__(self, config):
        self.last_time = None
        self.samples = 0
        self.neutral_ready = False
        self.active = None
        self.loaded = []
        self.saved = []
        self.snapshot_now = None

    def process(self, sample):
        self.samples += 1
        self.last_time = sample.host_monotonic_s

    def get_snapshot(self, now=None):
        self.snapshot_now = now
        return {"model": {"fingers": {}, "palm_points": [], "warnings": []}, "nodes": {},
                "calibration": {"neutral_ready": self.neutral_ready, "active": self.active, "message": "", "axis_nodes": []}, "warnings": []}

    def begin_neutral(self, duration=5, confirm_mount=False):
        if not confirm_mount:
            raise ValueError("安装确认尚未完成")
        self.neutral_ready = True

    def begin_axis(self, target, duration=8):
        if not self.neutral_ready:
            raise ValueError("请先静止标定")
        self.active = {"kind": "axis", "target": target, "remaining_s": duration}

    def cancel_calibration(self):
        self.active = None

    def load_calibration(self, path):
        self.loaded.append(path)
        self.neutral_ready = False

    def save_calibration(self, path):
        self.saved.append(path)


class FakeSource:
    instances = []

    def __init__(self, path, speed=1):
        self.path = path
        self.opened = False
        self.closed = False
        self.eof = False
        self.reads = 0
        self.__class__.instances.append(self)

    def open(self):
        self.opened = True

    def read(self):
        self.reads += 1
        return [SimpleNamespace(host_monotonic_s=float(self.reads))]

    def close(self):
        self.closed = True


class FakeReplay(FakeSource):
    def read(self):
        self.eof = True
        return [SimpleNamespace(host_monotonic_s=123.0)]


class FakeRecorder:
    instances = []

    def __init__(self, path):
        self.path = path
        self.closed = False
        self.opened = False
        self.written = 0
        self.__class__.instances.append(self)

    def open(self):
        self.opened = True

    def write(self, samples):
        self.written += len(samples)

    def close(self):
        self.closed = True


@pytest.fixture(autouse=True)
def no_real_serial(monkeypatch):
    FakeSource.instances = []
    FakeReplay.instances = []
    FakeRecorder.instances = []
    monkeypatch.setattr(gui, "list_serial_ports", lambda: [("/dev/fake-imu", "not a real port")])


def wait_for(app, predicate, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return
        time.sleep(0.005)
    assert predicate(), "condition timed out"


def worker(mode="live", **kwargs):
    return gui.PoseWorker({"hand": "right"}, mode=mode, engine_factory=FakeEngine,
                          serial_factory=FakeSource, replay_factory=FakeReplay,
                          recorder_factory=FakeRecorder, **kwargs)


def stop(thread):
    thread.stop()
    assert thread.wait(1500)


def test_worker_does_not_auto_connect(app):
    thread = worker()
    thread.start()
    try:
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("running"))
        assert FakeSource.instances == []
        assert FakeRecorder.instances == []
        assert not thread.latest_snapshot()["runtime"]["connected"]
        first = thread.latest_snapshot()
        first["runtime"]["error"] = "mutation"
        assert thread.latest_snapshot()["runtime"]["error"] == ""
    finally:
        stop(thread)


def test_manual_commands_operator_error_and_clean_shutdown(app):
    thread = worker()
    thread.start()
    try:
        thread.submit("connect", source="/dev/fake-imu")
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("connected"))
        thread.submit("neutral", confirm_mount=False)
        wait_for(app, lambda: "安装确认" in thread.latest_snapshot()["runtime"]["error"])
        assert thread.isRunning()
        assert thread.latest_snapshot()["runtime"]["connected"]
        thread.submit("neutral", confirm_mount=True)
        thread.submit("axis", target="index")
        wait_for(app, lambda: thread.latest_snapshot().get("calibration", {}).get("active") is not None)
        thread.submit("cancel")
        thread.submit("record_start", path="fake.csv")
        wait_for(app, lambda: bool(FakeRecorder.instances) and FakeRecorder.instances[0].written > 0)
        thread.submit("disconnect")
        wait_for(app, lambda: not thread.latest_snapshot()["runtime"]["connected"])
        assert FakeSource.instances[0].closed
        assert FakeRecorder.instances[0].closed
    finally:
        stop(thread)


def test_replay_uses_recorded_time_and_displays_eof(app):
    thread = worker(mode="replay")
    thread.start()
    try:
        thread.submit("connect", source="fake.csv")
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("eof"))
        assert thread._engine.snapshot_now == 123.0
        assert "不是实时" in thread.latest_snapshot()["runtime"]["message"]
        assert FakeRecorder.instances == []
    finally:
        stop(thread)
    assert FakeReplay.instances[0].closed


def test_demo_never_opens_or_records(app):
    def demo(t, config):
        return {"model": {"fingers": {}}, "nodes": {}, "calibration": {}, "warnings": ["模拟数据"]}
    thread = worker(mode="demo", demo_factory=demo)
    thread.start()
    try:
        thread.submit("connect", source="/dev/fake-imu")
        thread.submit("record_start", path="fake.csv")
        wait_for(app, lambda: "模拟模式" in thread.latest_snapshot().get("runtime", {}).get("error", ""))
        assert FakeSource.instances == []
        assert FakeRecorder.instances == []
    finally:
        stop(thread)


def test_window_prefill_is_not_auto_connection_or_calibration(app):
    thread = worker(calibration_path="explicit-calibration.json")
    window = gui.HandPoseWindow({"hand": "right"}, source="/dev/fake-prefilled", calibration_path="explicit-calibration.json", worker=thread)
    window.show()
    try:
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("running"))
        assert window.source_combo.currentText() == "/dev/fake-prefilled"
        assert not window.mount_check.isChecked()
        assert thread._engine.loaded == []
        assert FakeSource.instances == []
        window.connect_button.click()
        wait_for(app, lambda: thread.latest_snapshot()["runtime"]["connected"])
        assert FakeSource.instances[0].path == "/dev/fake-prefilled"
        assert thread._engine.loaded == ["explicit-calibration.json"]
        assert not thread._engine.neutral_ready
    finally:
        window.close()
        app.processEvents()
    assert not thread.isRunning()
    assert FakeSource.instances[0].closed


def test_save_dialog_rejects_existing_file(app, monkeypatch, tmp_path):
    existing = tmp_path / "existing.json"
    existing.write_text("preserve", encoding="utf-8")
    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", lambda *args, **kwargs: (str(existing), "JSON"))
    thread = worker()
    window = gui.HandPoseWindow({}, worker=thread)
    try:
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("running"))
        window._save_calibration()
        window._refresh_snapshot()
        assert "不会覆盖" in window.status_label.text()
        assert thread._engine.saved == []
        assert existing.read_text(encoding="utf-8") == "preserve"
    finally:
        window.close()


def test_canvas_offscreen_with_estimated_and_invalid_geometry(app):
    canvas = HandCanvas()
    canvas.resize(600, 500)
    canvas.set_model({"fingers": {"index": {"points": [[0, 0, 0], [0, 35, 0], [0, 60, 5], [0, 75, 8]], "estimated_segments": [2], "valid": True}}, "palm_points": [[-20, 0, 0], [20, 0, 0], [20, -35, 0], [-20, -35, 0]]}, mode="demo")
    assert not canvas.grab().isNull()
    canvas.set_model({"fingers": {"index": {"points": [[float("nan"), 0, 0], [0, 2, 0]]}}})
    assert not canvas.grab().isNull()


def test_command_queue_is_bounded():
    thread = worker()
    assert all(thread.submit("cancel") for _ in range(64))
    assert not thread.submit("cancel")


def test_axis_progress_requires_both_actual_finger_nodes():
    from vt_glove_pose.config import load_config
    config = load_config()
    index_nodes = [config["fingers"]["index"][key] for key in ("proximal", "distal")]
    thumb = config["fingers"]["thumb"]
    completed_axes = [config["palm_node"], thumb["proximal"], *index_nodes, config["auxiliary_node"]]
    text, details = gui.axis_progress(config, {"axis_nodes": completed_axes})
    assert "2/6 完成" in text  # Palm and index only; auxiliary node adds no step.
    assert "手掌 ✓" in text
    assert "食指 ✓" in text
    assert "拇指 1/2" in text
    assert "中指 0/2" in text
    assert f"Node {thumb['proximal']} 已有轴标定" in details
    assert f"Node {thumb['distal']} 待标定" in details
    # A changed but valid physical mapping must not retain default node pairs.
    config["fingers"]["index"], config["fingers"]["middle"] = config["fingers"]["middle"], config["fingers"]["index"]
    remapped, _ = gui.axis_progress(config, {"axis_nodes": completed_axes})
    assert "食指 0/2" in remapped
    assert "中指 ✓" in remapped
    simulated, _ = gui.axis_progress(config, {"axis_nodes": list(range(1, 13))}, demo=True)
    assert "不表示完成真实标定" in simulated
    assert "6/6" not in simulated


@pytest.mark.parametrize("palm,auxiliary", [(12, 11), (11, 12)])
def test_config_drives_reference_labels_and_axis_progress(app, palm, auxiliary):
    from vt_glove_pose.config import load_config
    config = load_config()
    config.update(palm_node=palm, auxiliary_node=auxiliary)
    thread = worker()
    window = gui.HandPoseWindow(config, worker=thread)
    try:
        assert f"Node {palm} 是待验证的手掌参考" in window.explanation_label.text()
        assert f"辅助 Node {auxiliary} 不预设为前臂" in window.explanation_label.text()
        assert f"手掌参考 Node {palm} 贴合" in window.mount_check.text()
        assert "功能轴 0/6 完成" in window.axes_progress_label.text()
        assert f"手掌：Node {palm} 待标定" in window.axes_progress_label.toolTip()
    finally:
        window.close()


def test_fatal_worker_disables_actions_preserves_error_and_stales_canvas(app, monkeypatch):
    thread = worker()
    window = gui.HandPoseWindow({}, worker=thread)
    try:
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("running"))
        # Stop the fake thread before supplying immutable terminal snapshots.
        stop(thread)
        fatal = {"runtime": {"running": False, "connected": False, "error": "fatal engine failure: retained detail"},
                 "calibration": {"neutral_ready": True, "active": None},
                 "model": {"fingers": {}}, "nodes": {"1": {"fresh": True}}}
        monkeypatch.setattr(thread, "latest_snapshot", lambda: fatal)
        window._ui_notice = "older operator notice must not hide the fatal error"
        window._refresh_snapshot()
        for widget in (window.connect_button, window.disconnect_button, window.source_combo,
                       window.refresh_button, window.hand_combo, window.hand_check,
                       window.neutral_button, window.axis_combo, window.axis_button,
                       window.cancel_button, window.save_button, window.load_button,
                       window.mount_check, window.record_button, window.stop_record_button):
            assert not widget.isEnabled()
        assert window.canvas.stale
        assert "关闭本窗口后重新启动" in window.status_label.text()
        assert "fatal engine failure: retained detail" in window.status_label.toolTip()
        queued = thread._commands.qsize()
        window._submit("connect", source="/dev/fake")
        assert thread._commands.qsize() == queued
        assert "fatal engine failure" in window.status_label.text()
        assert FakeSource.instances == []
        # The initial absent running flag must not be treated as terminal.
        monkeypatch.setattr(thread, "latest_snapshot", lambda: {"runtime": {"connected": False}})
        window._ui_notice = ""
        window._refresh_snapshot()
        assert window.connect_button.isEnabled()
        assert window.source_combo.isEnabled()
        assert "关闭本窗口后重新启动" not in window.status_label.text()
    finally:
        window.close()


def test_demo_hides_hardware_calibration_and_recording_but_keeps_view_controls(app):
    from vt_glove_pose.config import load_config

    def demo(t, config):
        return {"model": {"fingers": {}, "hand": config["hand"]}, "nodes": {},
                "calibration": {}, "warnings": ["模拟数据 / DEMO · 不代表真机精度"]}

    thread = worker(mode="demo", demo_factory=demo)
    window = gui.HandPoseWindow(load_config(), mode="demo", worker=thread)
    window.show()
    try:
        wait_for(app, lambda: thread.latest_snapshot().get("runtime", {}).get("running"))
        window._refresh_snapshot()
        for widget in (window.source_combo, window.refresh_button, window.connect_button,
                       window.disconnect_button, window.hand_check, window.calibration_group,
                       window.recording_controls, window.neutral_button, window.record_button):
            assert not widget.isVisible()
        for widget in (window.source_mode_label, window.hand_combo, window.render_combo,
                       window.root_follow_check, window.canvas, window.performance_label,
                       window.nodes_table, window.angles_table):
            assert widget.isVisible()
        assert "模拟数据 / DEMO" in window.source_mode_label.text()
        assert "不代表真机精度" in window.source_mode_label.text()
        assert "非传感器端到端延迟" in window.performance_label.text()
        assert "快照年龄不含串口等待、融合、模型计算" in window.performance_label.toolTip()
        assert window.styleSheet() == ""
        assert window.nodes_table.showGrid()
        assert window.angles_table.showGrid()
        assert not window.angles_table.alternatingRowColors()
        assert window.hand_combo.isEnabled()
        window.hand_combo.setCurrentIndex(1)
        wait_for(app, lambda: thread.config["hand"] == "left")
        wait_for(app, lambda: window.canvas.model.get("hand") == "left")
        assert FakeSource.instances == []
        assert FakeRecorder.instances == []
    finally:
        window.close()


def test_demo_reclaims_vertical_space_for_canvas_without_changing_live_controls(app):
    from vt_glove_pose.config import load_config
    config = load_config()
    live_thread = worker()
    live = gui.HandPoseWindow(config, worker=live_thread)
    live.show()
    try:
        wait_for(app, lambda: live_thread.latest_snapshot().get("runtime", {}).get("running"))
        app.processEvents()
        live_height = live.canvas.height()
        assert live.calibration_group.isVisible()
        assert live.recording_controls.isVisible()
        assert live.source_combo.isVisible()
        assert live.nodes_table.isVisible()
        assert live.explanation_label.isVisible()
        assert "六轴航向可能漂移" in live.explanation_label.text()
    finally:
        live.close()

    demo_thread = worker(mode="demo", demo_factory=lambda t, c: {
        "model": {"fingers": {}}, "nodes": {}, "calibration": {}, "warnings": ["模拟数据"]})
    demo = gui.HandPoseWindow(config, mode="demo", worker=demo_thread)
    demo.show()
    try:
        wait_for(app, lambda: demo_thread.latest_snapshot().get("runtime", {}).get("running"))
        app.processEvents()
        assert demo.canvas.height() >= live_height + 150
        assert demo.canvas.height() >= 600
        assert demo.canvas.width() > demo.nodes_table.width()
    finally:
        demo.close()
