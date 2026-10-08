"""Actual candidate pair outputs are saved, never reconstructed from a poll."""
from copy import deepcopy
import json
import math
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import iter_csv
from vt_glove_pose.thumb_trial import ThumbTrial
from vt_glove_pose.thumb_preview_live import OrientationWorker


def profile():
    identity = [[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]
    return dict(kind="vt_glove_pose.thumb_angle_candidate", schema_version=1,
        scope="conditional_scalar_preview", certified_calibration=False, anatomical_accuracy_verified=False,
        config=load_config(), node_ids=[5,6], bias_dps={"5":[0.,0.,0.], "6":[0.,0.,0.]},
        frames=dict(status="frames_ready", prox_sensor_to_functional=identity, dist_sensor_to_functional=identity,
                    functional_neutral_prox_acc_g=[0.,0.,1.], functional_neutral_dist_acc_g=[0.,0.,1.]),
        axis_fit=dict(candidate_status="inconsistent", reason_codes=["synthetic_test_fixture"]))


def orientation(t=100., count=1):
    return dict(host_monotonic_s=t, pair_count=count, node_ids=[5,6], paired_fresh=True,
                reference_ready=True, valid=True, reasons=[], nodes={str(n):dict(fresh=True,
                    sensor_quat_wxyz=[1.,0.,0.,0.],relative_quat_wxyz=[1.,0.,0.,0.],
                    gyro_dps=[0.,0.,0.],accel_g=[0.,0.,1.]) for n in (5,6)})


def angle(t=100., count=1, value=23.):
    return dict(host_monotonic_s=t, pair_count=count, node_ids=[5,6], angle_deg=value,
                current_angle_observed=True, paired_fresh=True, quality="tracking", history_loss=False,
                reasons=[], relative_bias_dps=.7, observer_details={"state_origin":"actual_online_test"})


def samples(t=100., degrees=0.):
    theta = math.radians(degrees)
    return [Sample(node, (0,0,0,0,round(math.sin(theta)/.000122) if node==6 else 0,
                          round((math.cos(theta) if node==6 else 1.)/.000122)),t,t+1000.)
            for node in range(1,13)]


def rows(trial):
    return [json.loads(line) for line in (trial.directory/"online.jsonl").read_text().splitlines()]


def test_trial_freezes_profile_and_preserves_exact_actual_online_snapshot(tmp_path):
    candidate = profile()
    frozen = deepcopy(candidate)
    trial = ThumbTrial(load_config(), tmp_path, 100., orientation(), angle_candidate=candidate)
    try:
        candidate["bias_dps"]["5"][0] = 9.
        actual = angle()
        expected = deepcopy(actual)
        assert trial.feed_snapshot(orientation(), angle_snapshot=actual)
        actual["angle_deg"] = 99.; actual["observer_details"]["state_origin"] = "mutated"
        metadata = json.loads((trial.directory/"metadata.json").read_text())
        assert metadata["angle_candidate_profile"] == frozen
        assert "not_replay" in metadata["angle_candidate_online_scope"]
        assert metadata["angle_candidate_accuracy_verified"] is False
        assert rows(trial)[0]["angle_candidate"] == expected
        assert rows(trial)[0]["angle_candidate_record_status"] == "actual_online_pair_output"
    finally:
        trial.finish(100., "test_cleanup")


def test_repeated_timestamp_keeps_two_distinct_candidate_outputs_and_raw(tmp_path):
    trial = ThumbTrial(load_config(), tmp_path, 100., orientation(), angle_candidate=profile())
    try:
        raw = samples(degrees=15.)+samples(degrees=25.)
        trial.feed_raw(raw)
        trial.feed_snapshot(orientation(count=1), angle_snapshot=angle(count=17, value=15.))
        trial.feed_snapshot(orientation(count=2), angle_snapshot=angle(count=18, value=25.))
        saved = rows(trial)
        assert [r["angle_candidate"]["angle_deg"] for r in saved] == [15.,25.]
        assert [r["angle_candidate"]["pair_count"] for r in saved] == [17,18]
        assert saved[0]["host_monotonic_s"] == saved[1]["host_monotonic_s"]
        assert list(iter_csv(trial.directory/"raw.csv")) == raw
    finally:
        trial.finish(100., "test_cleanup")


@pytest.mark.parametrize("missing", [None, dict(angle(), host_monotonic_s=99.), dict(angle(), node_ids=[3,4])])
def test_missing_or_mismatched_candidate_is_explicit_null_not_old_angle(tmp_path, missing):
    trial = ThumbTrial(load_config(), tmp_path, 100., orientation(), angle_candidate=profile())
    try:
        trial.feed_snapshot(orientation(), angle_snapshot=angle())
        trial.feed_snapshot(orientation(count=2), angle_snapshot=missing)
        assert rows(trial)[1]["angle_candidate"] is None
        assert rows(trial)[1]["angle_candidate_record_status"] != "actual_online_pair_output"
    finally:
        trial.finish(100., "test_cleanup")


def test_no_candidate_keeps_old_constructor_and_row_contract(tmp_path):
    trial = ThumbTrial(load_config(), tmp_path, 100., orientation())
    try:
        assert trial.feed_snapshot(orientation())
        assert "angle_candidate" not in rows(trial)[0]
        assert "angle_candidate_profile" not in json.loads((trial.directory/"metadata.json").read_text())
    finally:
        trial.finish(100., "test_cleanup")


class Source:
    def __init__(self, port): self.closed=False
    def open(self): pass
    def close(self): self.closed=True


def test_worker_records_same_process_returns_not_polled_or_replayed_angles(tmp_path):
    now=[100.]
    worker=OrientationWorker(load_config(), source_factory=Source, clock=lambda:now[0], angle_candidate=profile())
    trial = None
    try:
        worker.command("connect", dict(port="fake"))
        worker.feed_samples(samples())
        worker.command("reference", {})
        worker.command("start_trial", dict(output_root=str(tmp_path)))
        trial=worker.trial
        raw=samples(100.01,15.)+samples(100.01,25.)
        processor=worker.angle_processor
        real_process=processor.process
        actual=[]
        def observe_process(sample):
            result=real_process(sample)
            if result is not None: actual.append(deepcopy(result))
            return result
        processor.process=observe_process
        worker.feed_samples(raw)
        assert len(actual)==2
        assert [row["angle_candidate"] for row in rows(trial)]==actual
        assert list(iter_csv(trial.directory/"raw.csv"))==raw
        # Polls can change freshness/state but must never add or replace saved
        # completed-pair outputs. No background UI reader is involved.
        for _ in range(5): worker.publish(100.01)
        assert [row["angle_candidate"] for row in rows(trial)]==actual
        with pytest.raises(ValueError,match="测试期间"):
            worker.command("reference",{})
        assert worker.angle_processor is processor and worker.source is not None
        worker.finish_trial("operator_cancelled")
        worker.save_pool.shutdown(wait=True); worker.poll_trial_save()
        assert worker.last_trial["export_complete"]
        assert worker.last_trial["status"]=="aborted"
    finally:
        worker.disconnect();worker.save_pool.shutdown(wait=True)


def test_worker_missing_return_does_not_reuse_previous_same_stamp_candidate(tmp_path):
    worker=OrientationWorker(load_config(),source_factory=Source,clock=lambda:100.,angle_candidate=profile())
    try:
        worker.command("connect",dict(port="fake"));worker.feed_samples(samples())
        worker.command("reference",{});worker.command("start_trial",dict(output_root=str(tmp_path)))
        worker.feed_samples(samples(100.01,20.))
        assert rows(worker.trial)[0]["angle_candidate"] is not None
        worker.angle_processor.process=lambda sample:None
        worker.feed_samples(samples(100.01,30.))
        assert rows(worker.trial)[1]["angle_candidate"] is None
        assert rows(worker.trial)[1]["angle_candidate_record_status"]=="no_output_for_this_pair"
    finally:
        worker.disconnect();worker.save_pool.shutdown(wait=True)


def test_candidate_ui_does_not_call_missing_orientation_reference_failed_calibration():
    from PyQt5 import QtWidgets
    from vt_glove_pose.thumb_preview_live import LiveOrientationWindow
    class Worker:
        def start(self): pass
        def stop(self): pass
        def wait(self, timeout): return True
        def latest_snapshot(self):
            return dict(connected=True, snapshot=dict(paired_fresh=True, reference_ready=False,
                reasons=["display_reference_not_set"]), angle_candidate=angle(), angle_history=[])
        def submit(self, *args, **kwargs): pass
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window=LiveOrientationWindow(load_config(),worker=Worker(),angle_candidate=profile())
    try:
        window.refresh()
        assert "拇指实时弯曲" in window.windowTitle()
        assert "不影响弯曲页" in window.status.text()
        assert window.angle_widget.readout.text()=="+23.0°"
        assert window.reference_button.isEnabled()
    finally:
        window.close();app.processEvents()
