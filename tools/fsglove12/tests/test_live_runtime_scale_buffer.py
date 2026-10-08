"""Slow interactive calibration retains first-hold raw evidence safely."""
from copy import deepcopy

from test_live_runtime_input_scale import setup
from vt_fsglove12.live_runtime import STEP_KEYS
from vt_fsglove12.live_stream import RawSample
from vt_fsglove12.scale_capture_buffer import DEFAULT_MAX_SAMPLES, ScaleCaptureBuffer


def sample(stamp):
    return RawSample(12, (0, 0, 0, 0, 0, 8192), float(stamp), float(stamp + 1000))


def test_interactive_three_steps_longer_than_90_seconds_keep_first_hold_and_transitions(tmp_path):
    controller, session, clock = setup(tmp_path)
    controller._fit_profile = lambda session: None
    evidence = []
    for index, step in enumerate(STEP_KEYS):
        clock.value = 100. + index * 140.
        controller._command("capture", {"step": step})
        assert isinstance(session.scale_samples, ScaleCaptureBuffer)
        assert session.scale_samples.maxlen == DEFAULT_MAX_SAMPLES
        active = controller._active
        group = [sample(stamp) for stamp in (active["collect_at"], active["collect_at"] + 1., active["end_at"])]
        controller._retain_scale_samples(session, group)
        evidence.extend(group)
        active["rows"] = [{"nodes": {"12": {"sample_index": index + 1}}}]
        clock.value = active["end_at"]
        controller._capture(session, {"nodes": {}}, clock())
        if index < 2:
            transition = sample(clock() + 50.)
            controller._retain_scale_samples(session, [transition])
            evidence.append(transition)
    assert controller._scale_state["status"] == "running"
    assert controller._scale_job is None
    frozen = controller._scale_pending["samples"]
    assert frozen == tuple(evidence)
    assert frozen[0] is evidence[0]
    assert frozen[-1].host_monotonic_s - frozen[0].host_monotonic_s > 280.
    assert [hold["start_s"] for hold in controller._scale_pending["holds"]] == [103., 243., 383.]
    # After the request, incoming data cannot mutate the job's frozen prefix.
    controller._retain_scale_samples(session, [sample(1000.)])
    controller._poll_scale_check()
    job = controller._scale_job
    assert job.args[0] == frozen
    assert job.args[0][0] is evidence[0]
    assert all(row.host_monotonic_s < 1000. for row in job.args[0])
    assert not session.scale_samples.history_evicted


def test_eviction_is_explicit_failed_state_without_job_guessing_or_configuration_change(tmp_path):
    controller, session, _ = setup(tmp_path)
    original = deepcopy(controller.config)
    session.scale_samples = ScaleCaptureBuffer(maxlen=3)
    controller._retain_scale_samples(session, [sample(stamp) for stamp in range(100, 108)])
    controller._scale_holds = {key: dict(label=key, start_s=100. + i * 140.,
        end_s=102. + i * 140., time_basis="absolute_monotonic") for i, key in enumerate(STEP_KEYS)}
    controller._queue_scale_check(session)
    assert controller._scale_state["status"] == "failed"
    assert not controller._scale_state["candidate_available"]
    assert controller._scale_state["retention"]["status"] == "history_evicted"
    assert controller._scale_state["retention"]["history_evicted"] is True
    assert controller._scale_state["retention"]["evicted_samples"] == 5
    assert controller._scale_pending is None
    controller._poll_scale_check()
    assert controller._scale_job is None and controller._scale_report is None
    assert controller.config == original and controller._session is session
    assert not session.stop.is_set()


def test_clear_discards_old_trial_buffer_and_next_capture_gets_a_fresh_budget(tmp_path):
    controller, session, _ = setup(tmp_path)
    old = ScaleCaptureBuffer([sample(100), sample(101)], maxlen=1)
    session.scale_samples = old
    controller._clear_calibration()
    assert session.scale_samples is None and old.history_evicted
    controller._command("capture", {"step": "neutral"})
    assert isinstance(session.scale_samples, ScaleCaptureBuffer)
    assert session.scale_samples is not old
    assert not session.scale_samples.history_evicted
    assert session.scale_samples.maxlen == DEFAULT_MAX_SAMPLES
