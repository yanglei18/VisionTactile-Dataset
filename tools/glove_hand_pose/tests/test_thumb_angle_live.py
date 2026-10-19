"""Single-reader angle preview integration, with a fake candidate processor."""
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from copy import deepcopy
import json
import math
import sys
from types import SimpleNamespace

import pytest
from PyQt5 import QtWidgets

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose import thumb_preview_live


class FakeSource:
    instances = []
    def __init__(self, port):
        self.port, self.opened, self.closed = port, False, False
        self.instances.append(self)
    def open(self): self.opened = True
    def close(self): self.closed = True


class FakeCandidate:
    instances = []
    fail_init = False
    def __init__(self, profile, config=None):
        if self.fail_init: raise ValueError("candidate init test failure")
        self.profile, self.config = deepcopy(profile), deepcopy(config)
        self.count, self.stamp, self.received = 0, None, []
        self.fail_process = self.fail_snapshot = False
        self.instances.append(self)
    def process(self, sample):
        if self.fail_process: raise ValueError("candidate process test failure")
        self.received.append(sample)
        if sample.node_id != 6: return None
        self.count += 1; self.stamp = sample.host_monotonic_s
        return self.snapshot(now=self.stamp)
    def snapshot(self, now=None):
        if self.fail_snapshot: raise ValueError("candidate snapshot test failure")
        fresh = self.stamp is not None and (now is None or now-self.stamp < .5)
        return dict(angle_deg=12.+self.count if self.count else None,
                    quality="tracking", current_angle_observed=True, paired_fresh=fresh,
                    history_loss=False, reasons=[], host_monotonic_s=self.stamp,
                    pair_count=self.count, node_ids=[5, 6])


@pytest.fixture
def candidate_module(monkeypatch):
    FakeSource.instances = []
    FakeCandidate.instances = []
    FakeCandidate.fail_init = False
    def validate(profile):
        if profile.get("kind") != "test_candidate": raise ValueError("invalid candidate")
        return deepcopy(profile)
    module = SimpleNamespace(CandidateAngleProcessor=FakeCandidate, validate_candidate=validate)
    monkeypatch.setitem(sys.modules, "vt_glove_pose.thumb_angle_stream", module)
    return module


@pytest.fixture
def worker(candidate_module):
    now = [100.]
    value = thumb_preview_live.OrientationWorker(load_config(), source_factory=FakeSource,
        clock=lambda: now[0], angle_candidate=dict(kind="test_candidate", node_ids=[5, 6]))
    yield value, now
    value.disconnect()
    value.save_pool.shutdown(wait=True)


def frame(t=100.):
    return [Sample(node, (0, 0, 0, 0, 0, 8197), t, 1000+t) for node in range(1, 13)]


def connect(worker):
    worker.command("connect", dict(port="test"))


def test_no_reader_until_explicit_connect_and_both_processors_share_samples(worker):
    value, now = worker
    assert value.source is None and FakeSource.instances == []
    connect(value)
    records = frame()
    value.feed_samples(records)
    assert len(FakeSource.instances) == 1
    assert value.angle_processor.received == records
    assert value.processor.snapshot()["pair_count"] == 1
    value.publish(now[0])
    assert value.latest_snapshot()["angle_candidate"]["angle_deg"] == 13.
    assert len(value.angle_history) == 1


def test_each_new_pair_is_recorded_once_even_with_equal_host_stamps(worker):
    value, now = worker
    connect(value)
    value.feed_samples(frame()+frame())
    for _ in range(5): value.publish(now[0])
    assert len(value.angle_history) == 2
    assert set(value.angle_history[0]) == {"time_s", "angle_deg", "current_angle_observed", "paired_fresh",
                                          "history_loss", "pair_count", "quality"}
    assert [row["pair_count"] for row in value.angle_history] == [1, 2]
    assert [row["time_s"] for row in value.angle_history] == [0., 0.]


def test_display_reference_does_not_recreate_or_zero_candidate(worker):
    value, now = worker
    connect(value); value.feed_samples(frame())
    candidate = value.angle_processor
    before = deepcopy(list(value.angle_history))
    origin = value._angle_started_at
    value.command("reference", {})
    assert value.angle_processor is candidate
    assert list(value.angle_history) == before and value._angle_started_at == origin
    assert candidate.count == 1 and "未归零" in value.notice


@pytest.mark.parametrize("failure", ["process", "snapshot"])
def test_candidate_failures_do_not_close_serial_or_stop_orientation(worker, failure):
    value, now = worker
    connect(value)
    setattr(value.angle_processor, "fail_"+failure, True)
    value.feed_samples(frame())
    value.publish(now[0])
    assert value.source is not None and not value.source.closed
    assert value.processor.snapshot()["pair_count"] == 1
    state = value.latest_snapshot()
    assert state["angle_candidate"]["angle_deg"] is None
    assert state["angle_candidate"]["quality"] == "candidate_error"
    assert state["error"] == ""
    value.feed_samples(frame(100.01))
    assert value.processor.snapshot()["pair_count"] == 2


def test_candidate_initialization_failure_is_isolated(candidate_module):
    FakeCandidate.fail_init = True
    value = thumb_preview_live.OrientationWorker(load_config(), source_factory=FakeSource,
        clock=lambda: 100., angle_candidate=dict(kind="test_candidate", node_ids=[5, 6]))
    try:
        connect(value); value.feed_samples(frame()); value.publish(100.)
        assert value.source is not None and value.processor.snapshot()["paired_fresh"]
        assert value.latest_snapshot()["angle_candidate"]["quality"] == "candidate_error"
    finally:
        value.disconnect(); value.save_pool.shutdown(wait=True)


def test_candidate_reset_only_on_connection_lifecycle_and_history_is_bounded(worker):
    value, now = worker
    connect(value); value.feed_samples(frame())
    previous = value.angle_processor
    for i in range(3700):
        value._feed_angle_sample(Sample(6, (0, 0, 0, 0, 0, 8197), 100.+i/120., 1000.))
    assert len(value.angle_history) == 3600
    value.disconnect()
    assert value.angle_processor is not previous and not value.angle_history
    value.publish(now[0])
    assert value.latest_snapshot()["angle_candidate"]["paired_fresh"] is False
    connect(value)
    assert value.angle_processor.count == 0


def test_default_without_candidate_keeps_orientation_behavior(candidate_module):
    value = thumb_preview_live.OrientationWorker(load_config(), source_factory=FakeSource, clock=lambda: 100.)
    try:
        connect(value); value.feed_samples(frame()); value.publish(100.)
        assert value.angle_processor is None and not FakeCandidate.instances
        assert value.latest_snapshot()["angle_candidate"] is None
        assert value.processor.snapshot()["paired_fresh"]
    finally:
        value.disconnect(); value.save_pool.shutdown(wait=True)


class FakeWorker:
    def __init__(self): self.state={}; self.commands=[]; self.started=False
    def start(self): self.started=True
    def stop(self): pass
    def wait(self, timeout): return True
    def latest_snapshot(self): return deepcopy(self.state)
    def submit(self, name, **kwargs): self.commands.append((name, kwargs))


def test_window_adds_experimental_tab_without_changing_controls_or_auto_connect(candidate_module):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    fake = FakeWorker()
    window = thumb_preview_live.LiveOrientationWindow(load_config(), worker=fake,
                angle_candidate=dict(kind="test_candidate", node_ids=[5, 6]))
    try:
        assert fake.started and not fake.commands
        assert [window.tabs.tabText(i) for i in range(window.tabs.count())] == ["独立朝向", "拇指弯曲（实验）"]
        fake.state = dict(connected=True, snapshot=dict(paired_fresh=True, reference_ready=True, valid=True),
                          angle_candidate=dict(angle_deg=31., paired_fresh=True, current_angle_observed=False,
                                               node_ids=[5, 6], history_loss=False, reasons=[]), angle_history=[])
        window.refresh()
        assert window.angle_widget.mode == "predicted"
        assert window.angle_widget.readout.text() == "+31.0°"
        window.reference_button.click()
        assert fake.commands == [("reference", {})]
    finally:
        window.close(); app.processEvents()


def test_cli_rejects_invalid_candidate_before_creating_window(tmp_path, monkeypatch, candidate_module):
    config, candidate = tmp_path/"config.json", tmp_path/"candidate.json"
    config.write_text(json.dumps(load_config()), encoding="utf-8")
    candidate.write_text(json.dumps(dict(kind="invalid")), encoding="utf-8")
    def forbidden(*args, **kwargs): raise AssertionError("must not start UI or serial")
    monkeypatch.setattr(thumb_preview_live, "LiveOrientationWindow", forbidden)
    with pytest.raises(SystemExit) as caught:
        thumb_preview_live.main(["--config", str(config), "--angle-candidate", str(candidate)])
    assert caught.value.code == 2


def test_real_candidate_processes_bent_initial_pose_without_orientation_zero_reset():
    """Real scalar observer plus real orientation processor on synthetic raw."""
    config = load_config()
    identity = [[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]
    candidate = dict(kind="vt_glove_pose.thumb_angle_candidate", schema_version=1,
        scope="conditional_scalar_preview", certified_calibration=False, anatomical_accuracy_verified=False,
        config=config, node_ids=[5,6], bias_dps={"5":[0.,0.,0.], "6":[0.,0.,0.]},
        frames=dict(status="frames_ready", prox_sensor_to_functional=identity, dist_sensor_to_functional=identity,
                    functional_neutral_prox_acc_g=[0.,0.,1.], functional_neutral_dist_acc_g=[0.,0.,1.]),
        axis_fit=dict(candidate_status="inconsistent", reason_codes=["synthetic_test_fixture"]))
    now=[100.]
    value = thumb_preview_live.OrientationWorker(config, source_factory=FakeSource,
        clock=lambda: now[0], angle_candidate=candidate)
    try:
        connect(value)
        theta = math.radians(35.)
        records = [Sample(node, (0,0,0,0,round(math.sin(theta)/.000122) if node==6 else 0,
                                 round((math.cos(theta) if node==6 else 1.)/.000122)), 100., 1100.)
                   for node in range(1,13)]
        value.feed_samples(records); value.publish(now[0])
        before = value.latest_snapshot()["angle_candidate"]
        assert before["paired_fresh"] and abs(before["angle_deg"]) > 30.
        processor = value.angle_processor
        value.command("reference", {}); value.publish(now[0])
        after = value.latest_snapshot()["angle_candidate"]
        assert value.angle_processor is processor and after["angle_deg"] == before["angle_deg"]
        assert len(value.angle_history) == 1
        now[0] += 1.; value.publish(now[0])
        assert not value.latest_snapshot()["angle_candidate"]["paired_fresh"]
        assert value.source is not None
    finally:
        value.disconnect(); value.save_pool.shutdown(wait=True)
