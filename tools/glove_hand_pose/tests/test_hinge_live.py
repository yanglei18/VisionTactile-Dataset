"""No real ports: exercise the new UI, background work and exported evidence."""
import copy
import json
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import threading
import time

import pytest
from PyQt5 import QtWidgets

from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_export import export_hinge_session
from vt_glove_pose.hinge_worker import HingeWorker
from vt_glove_pose.records import Sample
from vt_glove_pose.pair_capture import load_pair_capture
from vt_glove_pose import hinge_gui
from vt_glove_pose.hinge_session import HingeSession


def test_hinge_ui_displays_effective_per_node_units_without_connecting(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .00875}
    config["gyro_scale_provenance"] = {"kind": "empirical_candidate_not_firmware_readback"}
    monkeypatch.setattr(hinge_gui, "list_serial_ports", lambda: [])
    window = hinge_gui.HingeWindow(config)
    try:
        assert "节点 5 = 0.0175" in window.unit_label.text()
        assert "节点 6 = 0.00875" in window.unit_label.text()
        assert "实测推断候选" in window.unit_label.text()
        assert window.worker._source is None
    finally:
        window.worker.stop()
        window.worker.wait(2000)
        window.close()
        app.processEvents()


def raw_frame(t=10., first=1):
    return [Sample(node, (0, 0, 0, 0, 0, 8197), t, 1000+t) for node in range(first, first+6)]


def test_export_roundtrip_retains_cancelled_full_wire_group_and_direction(tmp_path):
    config = load_config()
    samples = raw_frame()+raw_frame(10.01)
    bundle = dict(attempts=[dict(kind="motion", samples=samples, status="cancelled", reason="operator")],
                  profile=dict(offline_only=True, usable_as_live_calibration=False),
                  neutral_samples=samples, motion_samples=samples, direction_samples=samples,
                  direction=dict(status="confirmed", multiplier=-1))
    folder = tmp_path/"experiment"
    export_hinge_session(folder, bundle, config=config)
    restored = load_pair_capture(folder/"001_motion.csv", proximal=2, distal=4)
    assert restored["metadata"]["pairs"] == 2
    manifest = json.loads((folder/"manifest.json").read_text())
    assert manifest["export_complete"]
    assert manifest["attempts"][0]["status"] == "cancelled"
    assert manifest["attempts"][0]["raw_sample_count"] == 12
    assert manifest["session"]["direction"]["multiplier"] == -1
    assert "001_motion.csv" in manifest["files"]
    assert len(bundle["attempts"][0]["samples"]) == 12
    with pytest.raises(FileExistsError):
        export_hinge_session(folder, bundle, config=config)


def test_export_marks_partial_directory_when_json_metadata_fails(tmp_path):
    folder = tmp_path/"partial"
    with pytest.raises(ValueError):
        export_hinge_session(folder, dict(attempts=[], direction={"bad": float("nan")}), config=load_config())
    assert (folder/"export_started.json").exists()
    assert not (folder/"manifest.json").exists()


def test_export_validation_snapshots_labelled_display_rate(tmp_path):
    folder = tmp_path/"validation"
    export_hinge_session(folder, {}, config=load_config(), validation_attempts=[dict(
        kind="validation", status="complete", samples=raw_frame(), series=[dict(math_angle_deg=12.)])])
    preview = json.loads((folder/"001_validation_preview.json").read_text())
    assert preview["scope"] == "display_rate_snapshots_not_every_imu_sample"
    assert load_pair_capture(folder/"001_validation.csv")["metadata"]["pairs"] == 1


def test_real_session_bundle_exports_all_retained_windows(tmp_path):
    session = HingeSession(load_config())
    session.connect(10.)
    session.start_neutral(10.)
    for index in range(961):
        stamp = 10.+index/120.
        session.feed(raw_frame(stamp), now_s=stamp)
    assert session.snapshot()["state"] == "neutral_ready"
    session.start_motion(18.)
    for index in range(481):
        stamp = 18.+index/120.
        session.feed(raw_frame(stamp), now_s=stamp)
    session.cancel()
    bundle = session.export_bundle()
    folder = tmp_path/"real-controller-export"
    export_hinge_session(folder, bundle, config=load_config())
    manifest = json.loads((folder/"manifest.json").read_text())
    assert manifest["export_complete"]
    assert len(manifest["attempts"]) == 2
    assert load_pair_capture(folder/"001_neutral.csv")["metadata"]["pairs"] == 600
    assert manifest["attempts"][1]["status"] == "cancelled"
    assert "pairs" not in manifest["attempts"][0]


class FakeSession:
    def __init__(self, config, proximal, distal):
        self.connected = False
        self.count = 0
        self.generation = 0
        self.request = None
        self.accepted = []
        self.rejected = []
        self.state = "disconnected"
        self.outputs = []

    def connect(self, now_s):
        self.connected = True
        self.state = "needs_neutral"
        self.generation += 1

    def disconnect(self):
        self.connected = False
        self.state = "disconnected"
        self.generation += 1

    def feed(self, samples, now_s=None):
        self.count += len(samples)
        for sample in samples:
            if sample.node_id == 5:
                self.outputs.append(dict(host_monotonic_s=sample.host_monotonic_s, angle_deg=12.))

    def take_pair_outputs(self):
        rows, self.outputs = self.outputs, []
        return dict(rows=rows, discarded_rows=0)

    def tick(self, now_s):
        pass

    def snapshot(self, now_s=None):
        return dict(state=self.state, connected=self.connected, preview_valid=self.connected,
                    observer=dict(samples=self.count), math_angle_deg=12., anatomical_angle_deg=None)

    def start_neutral(self, now_s):
        self.state = "neutral_ready"

    def start_motion(self, now_s):
        self.state = "fitting"
        self.generation += 1
        self.request = dict(token=self.generation, neutral_capture={}, motion_capture={}, accel_g_per_lsb=.000122)

    def start_direction(self, now_s):
        self.state = "candidate_ready"

    def cancel(self):
        self.generation += 1
        self.state = "needs_neutral"

    def take_fit_request(self):
        result, self.request = self.request, None
        return result

    def accept_fit(self, token, profile):
        if token == self.generation:
            self.accepted.append(profile)
            self.state = "candidate_ready"

    def reject_fit(self, token, error):
        if token == self.generation:
            self.rejected.append(error)
            self.state = "neutral_ready"

    def export_bundle(self):
        return dict(attempts=[], profile=None)

    def validation_context(self):
        return dict(profile={"test_profile": True}, direction={"status": "unconfirmed"})


class FakeSource:
    instances = []

    def __init__(self, port):
        self.port = port
        self.opened = self.closed = False
        self.reads = 0
        self.instances.append(self)

    def open(self):
        self.opened = True

    def read(self):
        self.reads += 1
        return raw_frame(time.monotonic())

    def close(self):
        self.closed = True


def wait_until(predicate, seconds=3.):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.005)
    assert predicate()


def make_worker(**kwargs):
    return HingeWorker(load_config(), source_factory=FakeSource, session_factory=FakeSession, **kwargs)


def test_no_autoconnect_and_manual_connect_disconnect_preserves_single_owner():
    FakeSource.instances.clear()
    worker = make_worker()
    worker.start()
    try:
        wait_until(lambda: worker.latest_snapshot())
        assert not FakeSource.instances
        assert worker.submit("connect", port="/fake/only")
        wait_until(lambda: worker.latest_snapshot().get("runtime", {}).get("connected"))
        assert len(FakeSource.instances) == 1
        assert worker.submit("connect", port="/fake/other")
        wait_until(lambda: "已连接" in worker.latest_snapshot()["runtime"]["error"])
        assert len(FakeSource.instances) == 1
        assert worker.submit("disconnect")
        wait_until(lambda: not worker.latest_snapshot()["runtime"]["connected"])
        assert FakeSource.instances[0].closed
    finally:
        worker.stop()
        assert worker.wait(3000)


def test_background_fit_does_not_block_serial_and_late_cancelled_result_is_ignored():
    entered, release = threading.Event(), threading.Event()
    def fit(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return {"candidate": True}
    worker = make_worker(fit_function=fit)
    worker.start()
    try:
        worker.submit("connect", port="/fake")
        worker.submit("motion")
        assert entered.wait(2)
        first = worker.session.count
        wait_until(lambda: worker.session.count > first+30)
        worker.submit("cancel")
        wait_until(lambda: worker.latest_snapshot().get("state") == "needs_neutral")
        release.set()
        wait_until(lambda: worker.latest_snapshot()["runtime"]["background_job"] is None)
        assert worker.session.accepted == []
    finally:
        release.set()
        worker.stop()
        assert worker.wait(3000)


def test_fit_failure_is_reported_without_closing_stream():
    def fit(*args, **kwargs):
        raise ValueError("poor motion")
    worker = make_worker(fit_function=fit)
    worker.start()
    try:
        worker.submit("connect", port="/fake")
        worker.submit("motion")
        wait_until(lambda: worker.session.rejected)
        wait_until(lambda: "poor motion" in worker.latest_snapshot()["runtime"]["error"])
        assert worker.latest_snapshot()["runtime"]["connected"]
    finally:
        worker.stop()
        assert worker.wait(3000)


def test_disconnect_closes_source_immediately_while_fit_still_finishing():
    entered, release = threading.Event(), threading.Event()
    def fit(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return {}
    worker = make_worker(fit_function=fit)
    worker.start()
    try:
        worker.submit("connect", port="/fake")
        worker.submit("motion")
        assert entered.wait(2)
        worker.stop()
        wait_until(lambda: FakeSource.instances[-1].closed)
        assert worker.isRunning()
    finally:
        release.set()
        assert worker.wait(3000)
    assert not worker.latest_snapshot()["runtime"]["running"]


def test_validation_exports_raw_not_just_display_and_disconnect_retains_partial(tmp_path):
    worker = make_worker()
    worker.start()
    try:
        worker.submit("connect", port="/fake")
        worker.submit("direction")
        worker.submit("validation")
        wait_until(lambda: worker._validation_samples >= 36)
        worker.submit("disconnect")
        wait_until(lambda: bool(worker._validation_attempts))
        folder = tmp_path/"partial-validation"
        worker.submit("export", directory=str(folder))
        wait_until(lambda: worker.latest_snapshot().get("runtime", {}).get("last_export"))
        capture = load_pair_capture(folder/"001_validation.csv")
        assert capture["metadata"]["pairs"] >= 6
        assert json.loads((folder/"manifest.json").read_text())["attempts"][0]["status"] == "cancelled"
        assert not worker.latest_snapshot()["runtime"]["unsaved_experiment"]
    finally:
        worker.stop()
        assert worker.wait(3000)


def test_export_continues_acquisition_and_does_not_clear_new_unsaved_actions(tmp_path):
    entered, release = threading.Event(), threading.Event()
    def export(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return str(tmp_path)
    worker = make_worker(export_function=export)
    worker.start()
    try:
        worker.submit("connect", port="/fake")
        worker.submit("neutral")
        worker.submit("export", directory=str(tmp_path))
        assert entered.wait(2)
        first = worker.session.count
        wait_until(lambda: worker.session.count > first+30)
        worker.submit("cancel")
        wait_until(lambda: worker.latest_snapshot().get("state") == "needs_neutral")
        release.set()
        wait_until(lambda: worker.latest_snapshot()["runtime"]["last_export"])
        assert worker.latest_snapshot()["runtime"]["unsaved_experiment"]
    finally:
        release.set()
        worker.stop()
        assert worker.wait(3000)


@pytest.fixture(scope="module")
def app():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication(["hinge-tests"])
    application.setQuitOnLastWindowClosed(False)
    return application


class WindowWorker:
    def __init__(self, config, proximal, distal):
        self.pair = (proximal, distal)
        self.commands = []
        self.stopped = False
        self.state = dict(state="disconnected", runtime=dict(connected=False))
    def start(self): pass
    def submit(self, command, **args):
        self.commands.append((command,args))
        return True
    def latest_snapshot(self): return copy.deepcopy(self.state)
    def stop(self): self.stopped = True
    def isRunning(self): return False


def test_window_only_connects_on_button_and_renders_unconfirmed_angle(app):
    window = hinge_gui.HingeWindow(load_config(), port="/fake/prefill", worker_factory=WindowWorker)
    try:
        window.show()
        app.processEvents()
        assert window.worker.pair == (5, 6)
        assert window.worker.commands == []
        window.refresh()
        window.connect_button.click()
        assert window.worker.commands == [("connect", {"port": "/fake/prefill"})]
        window.worker.state.update(state="candidate_ready", preview_valid=True, math_angle_deg=-40.,
            observer=dict(samples=1), runtime=dict(connected=True))
        window.refresh()
        assert window.buttons["direction"].isEnabled()
        assert window.canvas.state.get("anatomical_angle_deg") is None
        assert not window.canvas.grab().isNull()
        window.worker.state.update(preview_valid=False, pair_age_s=2.)
        window.refresh()
        assert not window.buttons["direction"].isEnabled()
        assert not window.buttons["validation"].isEnabled()
    finally:
        window.close()
    assert window.worker.stopped


def test_cli_hinge_live_is_lazy_and_prefill_not_auto_open(monkeypatch):
    from vt_glove_pose.cli import main
    calls = []
    monkeypatch.setattr(hinge_gui, "launch_hinge_gui", lambda config, **kw: calls.append(kw) or 0)
    assert main(["hinge-live", "--finger", "ring", "--port", "/fake", "--smoke-seconds", ".2"]) == 0
    assert calls == [dict(finger="ring", port="/fake", smoke_seconds=.2)]


def test_markers_have_preparation_and_do_not_modify_estimator_or_zero():
    now = [100.]
    worker = make_worker(clock=lambda: now[0])
    worker._command("connect", {"port": "/fake"})
    worker._command("direction", {})
    worker._command("validation", {"duration_s": 180.})
    before = worker.session.snapshot()
    worker._command("marker", {"kind": "reference"})
    marker = worker._validation["markers"][0]
    assert marker["start_host_monotonic_s"] == 101.
    assert marker["end_host_monotonic_s"] == 103.
    with pytest.raises(ValueError, match="当前标记"):
        worker._command("marker", {"kind": "hold"})
    now[0] = 104.
    worker._command("marker", {"kind": "return_pose"})
    assert marker["status"] == "completed"
    assert worker._validation["markers"][1]["kind"] == "return"
    worker._command("cancel_marker", {})
    assert worker._validation["markers"][1]["status"] == "cancelled"
    assert worker.session.snapshot() == before
    worker._command("marker", {"kind": "hold"})
    assert worker._validation["markers"][-1]["end_host_monotonic_s"] == 165.
    worker._disconnect()
    assert worker._validation_attempts[0]["markers"][-1]["status"] == "interrupted"


@pytest.mark.parametrize("duration", [0., 181., float("nan"), True, "60"])
def test_invalid_validation_duration_does_not_start_recording(duration):
    worker = make_worker()
    worker._command("connect", {"port": "/fake"})
    worker._command("direction", {})
    try:
        with pytest.raises(ValueError, match="时长"):
            worker._command("validation", {"duration_s": duration})
        assert worker._validation is None
    finally:
        worker._disconnect()


def test_marker_outside_remaining_time_is_not_started():
    worker = make_worker(clock=lambda: 100.)
    worker._command("connect", {"port": "/fake"})
    worker._command("direction", {})
    worker._command("validation", {"duration_s": 30.})
    try:
        with pytest.raises(ValueError, match="剩余录制时间不足"):
            worker._command("marker", {"kind": "hold"})
        assert worker._validation["markers"] == []
    finally:
        worker._disconnect()


def test_export_freezes_per_validation_profile_and_csv_time_origin(tmp_path):
    folder = tmp_path/"marked"
    export_hinge_session(folder, {"profile": {"newer": True}}, config=load_config(), validation_attempts=[dict(
        kind="validation", started_s=9.9, status="partial", samples=raw_frame(10.), series=[],
        profile={"during_validation": True}, markers=[dict(kind="reference", pose_id="ref", status="completed",
            start_host_monotonic_s=11., end_host_monotonic_s=13.)])])
    protocol = json.loads((folder/"001_validation_protocol.json").read_text())
    assert protocol["windows"][0]["start_s"] == 1.
    assert protocol["windows"][0]["end_s"] == 3.
    assert protocol["csv_origin_host_monotonic_s"] == 10.
    assert json.loads((folder/protocol["profile_file"]).read_text()) == {"during_validation": True}
    assert json.loads((folder/"candidate_profile.json").read_text()) == {"newer": True}
    assert len(protocol["capture_sha256"]) == 64


def test_window_validation_duration_and_manual_marker_controls(app):
    window = hinge_gui.HingeWindow(load_config(), worker_factory=WindowWorker)
    try:
        window.worker.state.update(state="candidate_ready", preview_valid=True,
            runtime=dict(connected=True, validation_remaining_s=None))
        window.refresh()
        assert all(not button.isEnabled() for button in window.marker_buttons.values())
        window.buttons["validation"].click()
        assert window.worker.commands[-1] == ("validation", {"duration_s": 180})
        window.worker.state["runtime"].update(validation_remaining_s=175., validation_markers=[])
        window.refresh()
        window.marker_buttons["reference"].click()
        assert window.worker.commands[-1] == ("marker", {"kind": "reference"})
    finally:
        window.close()


def test_worker_records_every_online_pair_independent_of_ui_publishing():
    now = [100.]
    worker = make_worker(clock=lambda: now[0])
    worker._command("connect", {"port": "/fake"})
    worker._command("direction", {})
    try:
        worker._feed(raw_frame(99.9), 99.9)
        worker._command("validation", {"duration_s": 30.})
        # Several complete wire frames can share one USB-read timestamp.
        worker._feed(raw_frame(100.) + raw_frame(100.) + raw_frame(100.01), 100.02)
        worker._publish(100.02)
        assert len(worker._validation["online"]) == 3
        assert len(worker._validation["series"]) == 1
        assert len(worker._validation["samples"]) == 18
        assert worker.latest_snapshot()["runtime"]["validation_online_rows"] == 3
        worker._feed(raw_frame(130.), 130.)
        assert len(worker._validation["online"]) == 3
        assert worker.session.take_pair_outputs()["rows"] == []
    finally:
        worker._disconnect()


def test_worker_online_retention_is_bounded_and_loss_is_reported():
    worker = make_worker(clock=lambda: 100.)
    worker._command("connect", {"port": "/fake"})
    worker._command("direction", {})
    worker._command("validation", {"duration_s": 30.})
    try:
        worker._validation["online"] = [dict(host_monotonic_s=100.)] * 29999
        worker._feed(raw_frame(100.) * 3, 100.)
        assert len(worker._validation["online"]) == 30000
        assert worker._validation["online_discarded_rows"] == 2
    finally:
        worker._disconnect()


def test_guided_quickcheck_schedules_only_declared_reference_and_return_without_zeroing():
    now = [100.]
    worker = make_worker(clock=lambda: now[0])
    worker._command("connect", {"port": "/fake"})
    worker._command("direction", {})
    before = worker.session.snapshot()
    worker._command("validation", {"duration_s": 30., "guided_quickcheck": True})
    markers = worker._validation["markers"]
    assert [(m["kind"], m["start_host_monotonic_s"], m["end_host_monotonic_s"])
            for m in markers] == [("reference", 103., 105.), ("return", 128., 130.)]
    assert all(m["status"] == "scheduled" for m in markers)
    now[0] = 105.
    worker._advance_marker(now[0])
    assert markers[0]["status"] == "completed"
    assert markers[1]["status"] == "scheduled"
    assert worker.session.snapshot() == before
    with pytest.raises(ValueError, match="向导"):
        worker._command("marker", {"kind": "hold"})
    now[0] = 110.
    worker._finish_validation("partial", "operator_stopped_early")
    assert markers[1]["status"] == "interrupted"
    assert worker._validation_for_current_candidate
    worker._command("neutral", {})
    assert not worker._validation_for_current_candidate
    worker._disconnect()


def test_guided_quickcheck_protocol_is_not_labelled_as_detected_motion(tmp_path):
    folder = tmp_path/"guided"
    export_hinge_session(folder, {}, config=load_config(), validation_attempts=[dict(
        kind="validation", started_s=10., status="complete", samples=raw_frame(10.), series=[],
        profile={"during_validation": True}, guided_quickcheck=True, markers=[])])
    protocol = json.loads((folder/"001_validation_protocol.json").read_text())
    assert protocol["guided_quickcheck"]
    assert protocol["annotation_source"] == "timed_guidance_not_detected_pose_or_independent_truth"


def test_guided_window_primary_action_and_advanced_controls_are_separate(app, monkeypatch):
    window = hinge_gui.HingeWindow(load_config(), worker_factory=WindowWorker)
    try:
        window.show()
        app.processEvents()
        assert window.advanced_panel.isHidden()
        window.worker.state.update(state="needs_neutral", pair_age_s=None, runtime=dict(connected=True))
        window.refresh()
        assert not window.primary_button.isEnabled()
        window.worker.state.update(state="neutral_ready", pair_age_s=.01)
        window.refresh()
        window.primary_button.click()
        assert window.worker.commands[-1] == ("motion", {})
        window.worker.state.update(state="candidate_ready", preview_valid=True,
            observer=dict(current_angle_observed=True), direction=dict(status="confirmed"))
        window.refresh()
        window.primary_button.click()
        assert window.worker.commands[-1] == ("validation", {"duration_s": 30., "guided_quickcheck": True})
        calls = []
        monkeypatch.setattr(window, "export", lambda: calls.append("export"))
        window.worker.state["runtime"].update(validation_for_current_candidate=True, unsaved_experiment=True)
        window.refresh()
        window.primary_button.click()
        assert calls == ["export"]
        window.advanced_toggle.setChecked(True)
        assert not window.advanced_panel.isHidden()
        window.worker.state["runtime"]["unsaved_experiment"] = False
    finally:
        window.close()


@pytest.mark.parametrize("reason,action", [
    ("explicit_neutral_gravity_nearly_parallel_to_hinge", "neutral"),
    ("explicit_neutral_acceleration_norm_inconsistent", "export"),
    ("explicit_neutral_axis_gravity_mismatch", "export"),
])
def test_window_keeps_failure_reason_and_routes_its_actual_primary_button(app, monkeypatch, reason, action):
    from vt_glove_pose.hinge_recovery import classify_hinge_failure
    recovery = classify_hinge_failure("frames_inconsistent", [reason])
    window = hinge_gui.HingeWindow(load_config(), worker_factory=WindowWorker)
    exported = []
    monkeypatch.setattr(window, "export", lambda: exported.append(True))
    try:
        window.worker.state.update(state="needs_neutral" if recovery["requires_new_neutral"] else "neutral_ready",
            recovery=recovery, pair_age_s=.01, runtime=dict(connected=True, unsaved_experiment=False))
        window.refresh()
        assert window.guide_title.text() == recovery["title"]
        assert window.instructions.text() == recovery["instruction"]
        assert window.guide_detail.text() == recovery["detail"]
        assert window.primary_button.isEnabled()
        window.primary_button.click()
        if action == "export":
            assert exported == [True] and not window.worker.commands
        else:
            assert window.worker.commands == [("neutral", {})]
    finally:
        window.close()
