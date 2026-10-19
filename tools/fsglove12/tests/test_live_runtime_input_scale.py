"""Automatic evidence and explicit configuration transaction; no hardware."""
from collections import deque
from copy import deepcopy
import io
from types import SimpleNamespace
import threading

import pytest

from test_live_runtime import Clock, Fusion, Solver, harness, wait_for
from vt_fsglove12.config import PARENTS, digest, load_config
from vt_fsglove12.live_runtime import LiveController, _Session, STEP_KEYS, INPUT_SCALE_MAX_SAMPLES
from vt_fsglove12.live_stream import RawSample


class Backend:
    def __init__(self, config):
        self.config, self.resets = deepcopy(config), 0

    def reset_calibration(self):
        self.resets += 1

    def with_config(self, config):
        return Backend(config)


class Job:
    def __init__(self, *args):
        self.args, self.result = args, None
        self.running, self.cancelled = True, False

    def is_alive(self):
        return self.running

    def cancel(self):
        self.cancelled = True

    def join(self, timeout=None):
        pass


def setup(tmp_path):
    config = load_config()
    config["input_health"] = {"enabled": False}
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    clock = Clock()
    backend = Backend(config)
    controller = LiveController(config, rest, {}, output_root=tmp_path,
        clock=clock, pose_backend=backend, input_scale_job_factory=Job)
    session = _Session("scale-test", tmp_path, "NEVER_OPEN", object(), Fusion(config), object(), clock())
    session.opened = True
    session.streams = {name: io.StringIO() for name in ("events", "orientations", "poses")}
    controller._session = session
    return controller, session, clock


def populate(controller, session):
    session.scale_samples = deque([RawSample(12, (0, 0, 0, 0, 0, 8192), 100., 200.)],
                                  maxlen=INPUT_SCALE_MAX_SAMPLES)
    controller._scale_holds = {key: dict(label=key, start_s=100. + index*6,
        end_s=102. + index*6, time_basis="absolute_monotonic") for index, key in enumerate(STEP_KEYS)}
    controller._queue_scale_check(session)
    controller._poll_scale_check()
    return controller._scale_job


def candidate(controller):
    config = deepcopy(controller.config)
    config.setdefault("gyro_dps_per_lsb_by_node", {})["5"] = .0175
    config["units_verified"] = False
    report = dict(status="completed", recommended_nodes=["5"], candidate_config=config)
    controller._scale_report = report
    controller._scale_state = dict(status="completed", candidate_available=True,
        config_sha256=controller._config_sha, candidate_path="candidate.json")
    return config


def test_ring_is_opt_in_bounded_and_retains_unfiltered_immutable_samples(tmp_path):
    controller, session, _ = setup(tmp_path)
    sample = RawSample(5, (32767, 0, 0, 0, 0, 8192), 100., 200.)
    controller._retain_scale_samples(session, [sample])
    assert session.scale_samples is None
    session.scale_samples = deque(maxlen=3)
    controller._retain_scale_samples(session, [sample] * 5)
    assert len(session.scale_samples) == 3 and session.scale_samples[0] is sample
    newer = RawSample(5, sample.values, 191., 291.)
    controller._retain_scale_samples(session, [newer])
    assert list(session.scale_samples) == [newer]


def test_three_completed_holds_queue_even_when_pose_fit_fails(tmp_path):
    controller, session, clock = setup(tmp_path)
    def fit(_):
        controller._profile_status = "fit_failed"
    controller._fit_profile = fit
    for index, key in enumerate(STEP_KEYS):
        clock.value = 100. + index * 6
        controller._command("capture", {"step": key})
        controller._retain_scale_samples(session, [RawSample(12, (0, 0, 0, 0, 0, 8192), clock(), 200.)])
        active = controller._active
        active["rows"] = [{"nodes": {"12": {"sample_index": index + 1}}}]
        clock.value = active["end_at"]
        controller._capture(session, {"nodes": {}}, clock())
    assert controller._profile_status == "fit_failed"
    assert controller._scale_state["status"] == "running"
    assert controller._scale_job is None  # No numerical work inside capture.
    controller._poll_scale_check()
    job = controller._scale_job
    assert len(job.args[2]) == 3
    assert all(w["end_s"] - w["start_s"] == 2. for w in job.args[2])
    assert len(job.args[0]) == 3
    assert job.args[1] is not controller.config
    assert controller._profile is None


@pytest.mark.parametrize("action", ["clear", "redo"])
def test_completed_old_job_cannot_commit_after_generation_change(tmp_path, action):
    controller, session, _ = setup(tmp_path)
    job = populate(controller, session)
    if action == "clear":
        controller._clear_calibration()
        assert session.scale_samples is None
    else:
        controller._command("capture", {"step": "neutral"})
        assert "neutral" not in controller._scale_holds
        assert len(controller._scale_holds) == 2
    assert job.cancelled
    job.running = False
    job.result = {"report": {"status": "completed", "candidate_config": {"danger": True}}, "error": None}
    controller._poll_scale_check()
    assert controller._scale_report is None
    assert not controller._scale_state["candidate_available"]


def test_new_trial_waits_for_cancelled_job_cleanup(tmp_path):
    controller, session, _ = setup(tmp_path)
    old = populate(controller, session)
    controller._invalidate_scale_check()
    controller._queue_scale_check(session)
    controller._poll_scale_check()
    assert controller._scale_job is old and old.cancelled
    assert controller._scale_pending is not None
    old.running = False
    controller._poll_scale_check()
    assert controller._scale_job is not old
    assert controller._scale_pending is None


def test_check_failure_is_nonblocking_and_preserves_live_parameters(tmp_path):
    controller, session, _ = setup(tmp_path)
    original = deepcopy(controller.config)
    job = populate(controller, session)
    job.running, job.result = False, {"report": None, "error": "disk not writable"}
    controller._poll_scale_check()
    assert controller._session is session and session.opened
    assert controller.config == original and controller._scale_state["status"] == "failed"
    assert not session.stop.is_set()


@pytest.mark.parametrize("fault", ["none", "digest", "active", "extra_change", "backend"])
def test_invalid_adoption_acknowledges_without_changing_or_closing_session(tmp_path, fault):
    controller, session, _ = setup(tmp_path)
    if fault != "none":
        candidate(controller)
    if fault == "digest":
        controller._scale_state["config_sha256"] = "wrong"
    elif fault == "active":
        controller._active = {"step": "neutral"}
    elif fault == "extra_change":
        controller._scale_report["candidate_config"]["output_rate_hz"] = 30.
        controller.config["output_rate_hz"] = 120.
    elif fault == "backend":
        controller._pose_backend = object()
    original = deepcopy(controller.config)
    with pytest.raises(ValueError):
        controller._command("use_input_scale_candidate", {})
    assert controller.config == original and controller._session is session
    assert not session.stop.is_set()
    assert controller._snapshot["runtime"]["input_scale_generation"] == 1
    assert controller._input_snapshot["runtime"]["input_scale_generation"] == 1


def test_adoption_disconnects_first_creates_new_backend_and_requires_explicit_reconnect(harness):
    h = harness
    h.connect()
    original = deepcopy(h.controller.config)
    old_backend = Backend(original)
    h.controller._pose_backend = old_backend
    new_config = candidate(h.controller)
    old_session = h.controller._session
    h.controller._profile = {"profile_version": 1}
    assert h.controller.submit("use_input_scale_candidate")
    wait_for(lambda: h.controller.latest_snapshot()["runtime"]["input_scale_generation"] == 1)
    snapshot = h.controller.latest_snapshot()
    assert not snapshot["runtime"]["connected"] and len(h.sources) == 1
    assert old_session.done.is_set() and h.sources[0].closed
    assert h.controller.config == new_config and old_backend.config == original
    assert h.controller._pose_backend is not old_backend
    assert h.controller._profile is None and not h.controller._captures
    assert h.controller._config_sha == digest(new_config)
    assert snapshot["calibration"]["input_scale_check"]["status"] == "applied"
    assert snapshot["runtime"]["active_input_contract"]["nodes"]["5"]["gyro_dps_per_lsb"] == .0175
    import json
    assert json.loads((old_session.directory / "config.json").read_text()) == original
    assert json.loads((old_session.directory / "summary.json").read_text())["config_sha256"] == digest(original)
    assert h.controller.submit("connect", port="EXPLICIT_FAKE")
    wait_for(lambda: len(h.sources) == 2 and h.controller.latest_snapshot()["runtime"]["connected"])
    new_session = h.controller._session
    assert new_session.session_id != old_session.session_id
    assert json.loads((new_session.directory / "config.json").read_text()) == new_config
    assert h.controller._profile is None


def test_real_spawn_evidence_to_explicit_adoption_uses_frozen_candidate_not_editable_file(harness):
    from test_input_scale_check import synthetic
    from pathlib import Path
    h = harness
    samples, _, holds = synthetic(gains={5: 2.}, nodes=(5, 12))
    h.connect()
    h.controller._pose_backend = Backend(h.controller.config)
    session = h.controller._session
    session.scale_samples = deque(samples, maxlen=INPUT_SCALE_MAX_SAMPLES)
    h.controller._scale_holds = {w["label"]: w for w in holds}
    h.controller._queue_scale_check(session)
    wait_for(lambda: h.controller.latest_snapshot()["calibration"]["input_scale_check"]["status"]
             == "completed", timeout=30.)
    state = h.controller.latest_snapshot()["calibration"]["input_scale_check"]
    assert state["recommended_nodes"] == ["5"] and state["supported_nodes"] == ["12"]
    assert state["candidate_available"]
    assert len(state["inconclusive_nodes"]) == 10
    frozen = deepcopy(h.controller._scale_report["candidate_config"])
    # A third party editing an exported file cannot alter this in-memory
    # session-bound transaction. The separate launch CLI would validate a file.
    Path(state["candidate_path"]).write_text("not JSON; never reload this for adoption")
    assert h.controller.submit("use_input_scale_candidate")
    wait_for(lambda: h.controller.latest_snapshot()["runtime"]["input_scale_generation"] == 1)
    assert h.controller.config == frozen
    assert h.controller._scale_state["status"] == "applied"
    assert h.controller._session is None and len(h.sources) == 1


def test_join_inventories_job_registered_during_stop(tmp_path):
    controller, session, _ = setup(tmp_path)
    entered, release = threading.Event(), threading.Event()
    job = Job()
    def delayed_factory(*args):
        entered.set()
        release.wait(2.)
        return job
    controller._scale_job_factory = delayed_factory
    session.scale_samples = deque([RawSample(12, (0, 0, 0, 0, 0, 8192), 100., 200.)])
    controller._scale_holds = {key: dict(label=key, start_s=100.+i*6, end_s=102.+i*6,
        time_basis="absolute_monotonic") for i, key in enumerate(STEP_KEYS)}
    controller._queue_scale_check(session)
    # Isolate only the processor/job handoff (no fake source lifecycle needed).
    def processor():
        controller._poll_scale_check()
        controller._invalidate_scale_check()
    controller._thread = threading.Thread(target=processor)
    controller._thread.start()
    assert entered.wait(1.)
    controller.stop()
    assert not controller.join(.01)
    release.set()
    assert not controller.join(1.), "unjoined background analysis must not count as shutdown"
    assert job.cancelled
    job.running = False
    assert controller.join(1.)


def test_rechecking_adopted_candidate_does_not_multiply_scales_again(tmp_path):
    from test_input_scale_check import synthetic
    from vt_fsglove12.input_scale_check import run_input_scale_check
    samples, config, holds = synthetic(gains={5: 2., 6: .5}, nodes=(5, 6, 12))
    first = run_input_scale_check(samples, config, holds, tmp_path / "before")
    applied = deepcopy(first["candidate_config"])
    second = run_input_scale_check(samples, applied, holds, tmp_path / "after")
    assert first["recommended_nodes"] == ["5", "6"]
    assert second["nominal_supported_nodes"] == ["5", "6", "12"]
    assert second["recommended_nodes"] == [] and second["candidate_config"] is None
    assert not second["candidate_available"]
    assert not (tmp_path / "after" / "config.candidate.json").exists()
    assert applied == first["candidate_config"]
