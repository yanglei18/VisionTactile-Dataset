"""Quick-hand UI uses fake data only; no hardware opening or enumeration."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from copy import deepcopy
import time
import pytest
from PyQt5 import QtCore, QtWidgets
from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose import quick_hand_live as live


@pytest.fixture(scope="module")
def app():
    value = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    value.setQuitOnLastWindowClosed(False)
    return value


class FakeEngine:
    def __init__(self, config):
        self.config = config
        self.received = []
        self.reference_calls = []
        self.learn_calls = []
        self.alignment_calls = []
        self.active = None
        self.ready = False
        self.cancelled = 0

    def process(self, sample):
        self.received.append(sample)

    def begin_reference(self, duration=2):
        self.reference_calls.append(duration)
        self.active = dict(kind="reference", remaining_s=duration)

    def begin_motion_learning(self, duration=6):
        self.learn_calls.append(duration)
        self.active = dict(kind="motion_learning", remaining_s=duration)

    def begin_frame_alignment(self, duration=6):
        self.alignment_calls.append(duration)
        self.active = dict(kind="frame_alignment", remaining_s=duration)

    def cancel_calibration(self):
        self.active = None
        self.cancelled += 1

    def get_snapshot(self, now=None):
        return dict(model=dict(hand="right", fingers={}), nodes={str(s.node_id): dict(fresh=True) for s in self.received},
                    calibration=dict(neutral_ready=self.ready, active=deepcopy(self.active), message=""),
                    warnings=["六轴航向仍会漂移"], quick_mode=True)


class FakeSource:
    instances = []
    def __init__(self, port):
        self.port = port
        self.opened = self.closed = False
        self.reads = 0
        self.instances.append(self)

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True

    def read(self):
        self.reads += 1
        time.sleep(.002)
        return frame(time.monotonic())


def frame(now=100.):
    return [Sample(node, (0, 0, 0, 0, 0, 8197), now, 1000.) for node in range(1, 13)]


@pytest.fixture
def worker():
    FakeSource.instances = []
    value = live.QuickHandWorker(load_config(), engine_factory=FakeEngine, source_factory=FakeSource, clock=lambda: 100.)
    yield value
    if value.isRunning():
        value.stop()
        assert value.wait(1500)
    if value.source is not None:
        value.disconnect()


def test_no_reader_until_explicit_connect_and_all12_processed_once(worker):
    worker.publish()
    assert not FakeSource.instances and worker.source is None
    worker.command("connect", dict(port="fake"))
    rows = frame()
    worker.feed_samples(rows)
    worker.publish()
    assert len(FakeSource.instances) == 1
    assert worker.engine.received == rows
    assert worker.latest_snapshot()["runtime"]["received_samples"] == 12
    for _ in range(8):
        worker.publish()
    assert len(worker.engine.received) == 12


def test_reference_and_learning_are_separate_fixed_duration_commands(worker):
    with pytest.raises(ValueError, match="先连接"):
        worker.command("reference", {})
    worker.command("connect", dict(port="fake"))
    worker.command("reference", {})
    assert worker.engine.reference_calls == []
    worker.publish(103.)
    assert worker.engine.reference_calls == [2]
    worker.engine.ready = True
    worker.command("learn", {})
    worker.feed_samples(frame())
    assert worker.engine.learn_calls == [6]
    assert len(worker.engine.received) == 12
    worker.command("cancel", {})
    assert worker.engine.active is None


def test_reconnect_resets_reference_and_closes_single_previous_source(worker):
    worker.command("connect", dict(port="fake"))
    engine, source = worker.engine, worker.source
    worker.engine.ready = True
    with pytest.raises(ValueError, match="已经连接"):
        worker.command("connect", dict(port="other"))
    assert len(FakeSource.instances) == 1
    worker.command("disconnect", {})
    assert source.closed and not worker.engine.ready
    worker.command("connect", dict(port="fake"))
    assert worker.engine is not engine and not worker.engine.ready


def test_latest_snapshot_is_a_copy(worker):
    worker.publish()
    state = worker.latest_snapshot()
    state["runtime"]["error"] = "mutated"
    assert worker.latest_snapshot()["runtime"]["error"] == ""


def test_disconnected_gui_is_prefill_only_and_no_auto_enumeration(app, worker, monkeypatch):
    monkeypatch.setattr(live, "list_serial_ports", lambda: pytest.fail("unexpected enumeration"))
    window = live.QuickHandWindow(load_config(), port="/dev/fake", worker=worker, start_worker=False)
    try:
        assert window.port_box.currentText() == "/dev/fake"
        assert not FakeSource.instances and worker._commands.empty()
        assert not window.reference_button.isEnabled()
        assert not window.learn_button.isEnabled()
        assert "整手坐标对齐" in window.learn_button.text()
        assert "准备 3 秒" in window.reference_button.text()
        assert "采集 10 秒" in window.alignment_button.text()
        assert "握拳" not in window.learn_button.text()
        assert window.timer.interval() == 33
        assert window.canvas._geometry_config == load_config()
        window.connect_button.click()
        assert worker._commands.get_nowait() == ("connect", {"port": "/dev/fake"})
    finally:
        window.close()


def test_partial_missing_finger_grey_only_and_table_unavailable(app, worker):
    worker.command("connect", dict(port="fake"))
    worker.engine.ready = True
    worker.feed_samples(frame())
    worker.publish()
    model = dict(hand="right", root_orientation_wxyz=[1, 0, 0, 0], root_orientation_valid=True,
                 fingers=dict(index=dict(valid=True, geometry_live=True, quality="quick_estimate", mcp_deg=23., pip_deg=45.),
                              ring=dict(valid=False, geometry_live=False, quality="stale", mcp_deg=99.)))
    worker._snapshot["model"] = deepcopy(model)
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    try:
        assert not window.canvas.stale
        assert window.canvas.model["fingers"]["index"]["valid"]
        assert window.canvas.model["fingers"]["ring"]["quality"] == "unavailable"
        assert worker._snapshot["model"] == model
        assert window.angles_table.item(1, 1).text() == "23.0°"
        assert window.angles_table.item(3, 1).text() == "—"
        assert window.learn_button.isEnabled()
        window.learn_button.click()
        assert worker._commands.get_nowait()[0] == "align"
    finally:
        window.close()


def test_guidance_countdown_without_enforcing_optional_learning(app, worker):
    worker.command("connect", dict(port="fake"))
    worker.command("reference", {})
    worker.publish()
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    try:
        assert "准备剩余 3.0 秒" in window.guidance_label.text()
        assert "尚未采集" in window.guidance_label.text()
        assert "#fff1b8" in window.guidance_label.styleSheet()
        assert "掌心朝下" in window.guidance_label.text()
        assert not window.learn_button.isEnabled()
        assert window.cancel_button.isEnabled()
        worker.publish(103.)
        window.refresh()
        assert "采集剩余 2.0 秒" in window.guidance_label.text()
        assert "正在采集参考" in window.guidance_label.text()
        assert "#daf5df" in window.guidance_label.styleSheet()
        worker.engine.active = None
        worker.engine.ready = True
        worker.publish()
        window.refresh()
        assert "五指姿态正在显示" in window.guidance_label.text()
        assert not worker.engine.learn_calls
    finally:
        window.close()


def test_alignment_has_three_second_preparation_then_ten_second_collection(app, worker):
    worker.command("connect", dict(port="fake"))
    worker.engine.ready = True
    worker.command("align", {})
    worker.feed_samples(frame())
    worker.publish()
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    try:
        assert worker.engine.alignment_calls == []
        assert "准备" in window.guidance_label.text()
        assert "不要等采集结束才动作" in window.guidance_label.text()
        assert "#fff1b8" in window.guidance_label.styleSheet()
        worker.publish(103.)
        window.refresh()
        assert worker.engine.alignment_calls == [10]
        assert not worker.engine.learn_calls
        assert "现在开始转动，正在采集" in window.guidance_label.text()
        assert "整个手掌向下倾斜约 30°再回水平" in window.guidance_label.text()
        assert "不要握拳" in window.guidance_label.text()
        assert "上下平移" in window.guidance_label.text()
        assert "10.0 秒" in window.guidance_label.text()
        assert "#daf5df" in window.guidance_label.styleSheet()
        assert not window.alignment_button.isEnabled()
        assert window.cancel_button.isEnabled()
        assert not window.save_profile_button.isEnabled()
        assert not window.load_profile_button.isEnabled()
        assert not window.flip_button.isEnabled()
        assert worker.received_samples == 12
    finally:
        window.close()


@pytest.mark.parametrize("command,field,duration,kind", [
    ("reference", "reference_calls", 2, "reference"),
    ("align", "alignment_calls", 10, "frame_alignment"),
])
def test_fake_clock_preparation_does_not_capture_and_starts_exactly_once(worker, command, field, duration, kind):
    now = [100.]
    worker.clock = lambda: now[0]
    worker.command("connect", dict(port="fake"))
    worker.engine.ready = True
    worker.command(command, {})
    for stamp in (100., 101., 102.999):
        now[0] = stamp
        worker.feed_samples(frame(stamp))
        worker.publish()
        active = worker.latest_snapshot()["calibration"]["active"]
        assert active["kind"] == kind and active["phase"] == "preparing"
        assert active["capture_duration_s"] == duration
        assert active["remaining_s"] == pytest.approx(103. - stamp)
        assert worker.engine.active is None
        assert getattr(worker.engine, field) == []
    now[0] = 103.
    worker.publish()
    assert worker.latest_snapshot()["calibration"]["active"]["phase"] == "collecting"
    assert getattr(worker.engine, field) == [duration]
    for stamp in (103., 103.2, 104., 105.):
        now[0] = stamp
        worker.publish()
    assert getattr(worker.engine, field) == [duration]
    assert worker.pending is None
    assert worker.received_samples == 36


@pytest.mark.parametrize("stop_command", ["cancel", "disconnect"])
def test_cancel_or_disconnect_during_preparation_never_starts_capture(worker, stop_command):
    now = [100.]
    worker.clock = lambda: now[0]
    worker.command("connect", dict(port="fake"))
    engine = worker.engine
    worker.command("align", {})
    now[0] = 102.
    worker.command(stop_command, {})
    now[0] = 110.
    worker.publish()
    assert worker.pending is None
    assert not engine.alignment_calls
    assert not worker.latest_snapshot()["calibration"]["active"]
    if stop_command == "disconnect":
        assert worker.source is None


def test_preparation_cannot_be_replaced_and_failed_start_does_not_retry(worker):
    worker.command("connect", dict(port="fake"))
    worker.command("align", {})
    with pytest.raises(ValueError, match="准备或采集"):
        worker.command("reference", {})
    attempts = []
    def failing_start(duration):
        attempts.append(duration)
        raise ValueError("没有新鲜数据")
    worker.engine.begin_frame_alignment = failing_start
    worker.publish(103.)
    worker.publish(104.)
    assert attempts == [10]
    assert worker.pending is None
    assert "采集未开始" in worker.latest_snapshot()["runtime"]["error"]
    assert worker.source is not None


def test_large_preparation_and_collection_instructions_fit_visible_label(app, worker):
    worker.command("connect", dict(port="fake"))
    worker.engine.ready = True
    worker.command("align", {})
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    window.show()
    app.processEvents()
    try:
        for now in (100., 103.):
            worker.publish(now)
            window.refresh()
            app.processEvents()
            label = window.guidance_label
            bounds = label.fontMetrics().boundingRect(
                QtCore.QRect(0, 0, label.contentsRect().width() - 20, 10000),
                QtCore.Qt.TextWordWrap, label.text())
            assert label.height() >= bounds.height() + 20
            assert window.angles_table.geometry().top() > label.geometry().bottom()
    finally:
        window.close()


def test_released_workflow_hides_unfinished_tools_and_never_sends_old_fist_command(app, worker):
    worker.command("connect", dict(port="fake"))
    worker.engine.ready = True
    worker.publish()
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    try:
        window.show()
        app.processEvents()
        assert window.parameters_group.isHidden()
        assert not window.flip_button.isVisible()
        assert not window.save_profile_button.isVisible()
        assert not window.load_profile_button.isVisible()
        window.reference_button.click()
        window.alignment_button.click()
        commands = [worker._commands.get_nowait()[0] for _ in range(2)]
        assert commands == ["reference", "align"]
        assert "learn" not in commands
    finally:
        window.close()


def test_worker_thread_single_reader_shutdown(app):
    FakeSource.instances = []
    worker = live.QuickHandWorker(load_config(), engine_factory=FakeEngine, source_factory=FakeSource)
    worker.start()
    try:
        worker.submit("connect", port="fake")
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline and worker.latest_snapshot()["runtime"].get("received_samples", 0) < 12:
            app.processEvents()
            time.sleep(.01)
        assert len(FakeSource.instances) == 1
        assert worker.latest_snapshot()["runtime"]["received_samples"] >= 12
    finally:
        worker.stop()
        assert worker.wait(1500)
    assert FakeSource.instances[0].closed
    assert not worker.latest_snapshot()["runtime"]["connected"]


def test_cli_loads_config_and_prefills_port_without_connection(monkeypatch):
    calls = []
    monkeypatch.setattr(live, "launch_quick_hand", lambda config, **kwargs: calls.append((config, kwargs)) or 7)
    assert live.main(["--port", "/dev/fake", "--smoke-seconds", "0.2"]) == 7
    assert calls[0][1] == dict(port="/dev/fake", smoke_seconds=.2)


def test_invalid_smoke_time_rejected_without_ui():
    with pytest.raises(ValueError, match="positive"):
        live.launch_quick_hand(load_config(), smoke_seconds=0)


def test_port_refresh_uses_device_path_from_source_tuple(app, worker, monkeypatch):
    monkeypatch.setattr(live, "list_serial_ports", lambda: [("/dev/fake0", "Fake board")])
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    try:
        window.refresh_ports()
        assert window.port_box.itemText(0) == "/dev/fake0"
        assert not FakeSource.instances
    finally:
        window.close()


def test_completed_action_status_and_appended_warnings_are_visible(app, worker):
    worker.command("connect", dict(port="fake"))
    worker.engine.ready = True
    worker.message = "保持 2 秒"
    worker.publish()
    worker._snapshot["calibration"]["message"] = "参考已设置，可以活动五指"
    worker._snapshot["warnings"] = ["航向会漂移", "固定提示2", "固定提示3", "固定提示4", "节点 8 缺失", "请检查固定"]
    window = live.QuickHandWindow(load_config(), worker=worker, start_worker=False)
    try:
        assert "可以活动五指" in window.status_label.text()
        assert "保持 2 秒" not in window.status_label.text()
        assert "节点 8 缺失" in window.warnings_label.text()
        assert "请检查固定" in window.warnings_label.text()
        assert "航向会漂移" in window.warnings_label.text()
    finally:
        window.close()


def test_real_engine_reference_whole_hand_renders_without_learning(app):
    from vt_glove_pose.quick_hand import QuickHandEngine
    engine = QuickHandEngine(load_config())
    for sample in frame(100.):
        engine.process(sample)
    engine.begin_reference(duration=2)
    for i in range(1, 250):
        for sample in frame(100. + i / 120.):
            engine.process(sample)
    state = engine.get_snapshot(100. + 249 / 120.)
    assert state["calibration"]["neutral_ready"]
    assert all(item["geometry_live"] for item in state["model"]["fingers"].values())
    canvas = live.QuickHandCanvas(config=load_config())
    canvas.resize(650, 600)
    canvas.set_model(live.display_model(state["model"]), stale=False)
    canvas.show()
    app.processEvents()
    assert not canvas.grab().isNull()
    assert canvas.drawn_triangles > 0
    canvas.close()
