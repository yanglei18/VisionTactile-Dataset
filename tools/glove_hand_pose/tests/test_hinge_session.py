"""Pure manual-session workflow; no Qt, serial, filesystem writes or threads."""
import json
import math

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_session import HingeSession
from vt_glove_pose.hinge_replay import calibrate_hinge_pair
from vt_glove_pose.math3d import from_axis_angle, to_matrix
from vt_glove_pose.records import Sample


RATE = 30.
GYRO = .00875
ACC = .000122


def config():
    value = load_config()
    value["sample_rate_hz"] = RATE
    return value


def frame(t, angle=0., speed=0., *, proximal=5, distal=6, other_noise=False):
    group_start = (proximal - 1) // 6 * 6 + 1
    gravity = [0., 0., 1.]
    a = math.radians(angle)
    distal_gravity = [0., -math.sin(a), math.cos(a)]
    rows = []
    for node in range(group_start, group_start + 6):
        if node == proximal:
            values = (0, 0, 0, 0, 0, round(1. / ACC))
        elif node == distal:
            values = (round(-speed / GYRO), 0, 0, *np.rint(np.asarray(distal_gravity) / ACC).astype(int))
        else:
            values = (int(25000 * math.sin(t * 9 + node)) if other_noise else 0, 0, 0, 0, 0, 8197)
        rows.append(Sample(node, tuple(values), t, 1000. + t))
    return rows


def pump(session, end, movement=None, *, other_noise=False):
    first = math.floor(session._now * RATE + 1e-8) + 1
    last = math.floor(end * RATE + 1e-8)
    for tick in range(first, last + 1):
        t = tick / RATE
        angle, speed = movement(t) if movement else (0., 0.)
        session.feed(frame(t, angle, speed, proximal=session.proximal, distal=session.distal,
                           other_noise=other_noise), now_s=t)
    return session.snapshot()


def neutral_session(*, proximal=5, distal=6):
    session = HingeSession(config(), proximal, distal)
    session.connect(0.)
    session.feed(frame(0., proximal=proximal, distal=distal))
    session.start_neutral(0.)
    result = pump(session, 8., other_noise=True)
    assert result["state"] == "neutral_ready", result
    return session


def fitting_session():
    session = neutral_session()
    session.start_motion(8.)
    pump(session, 31.)
    request = session.take_fit_request()
    assert request is not None
    assert session.snapshot()["state"] == "fitting"
    return session, request


def profile():
    return dict(kind="vt_glove_pose.offline_hinge_profile", schema_version=1,
                offline_only=True, usable_as_live_calibration=False,
                profile_status="offline_candidate_ready",
                settings=dict(node_ids=[5, 6], sample_rate_hz=RATE, gyro_dps_per_lsb=GYRO, accel_g_per_lsb=ACC),
                axis_fit=dict(candidate_status="candidate_supported"),
                frames=dict(status="frames_ready", prox_sensor_to_functional=np.eye(3).tolist(),
                            dist_sensor_to_functional=np.eye(3).tolist()),
                neutral=dict(prox_bias_dps=[0., 0., 0.], dist_bias_dps=[0., 0., 0.],
                             prox_acc_g=[0., 0., 1.], dist_acc_g=[0., 0., 1.]),
                anatomical_flexion_sign_verified=False)


def ready_session(angle=0.):
    session, request = fitting_session()
    session.feed(frame(31. + 1. / RATE, angle=angle))
    original = profile()
    assert session.accept_fit(request["token"], original)
    assert original == profile()
    return session


def test_initial_state_does_nothing_without_explicit_requests():
    session = HingeSession(config())
    assert session.snapshot()["state"] == "disconnected"
    assert session.take_fit_request() is None
    session.feed(frame(0.))
    assert session.snapshot()["matched_pairs"] == 0
    session.connect(0.)
    pump(session, 10.)
    assert session.snapshot()["state"] == "needs_neutral"
    assert session.export_bundle()["attempts"] == []
    with pytest.raises(ValueError, match="neutral"):
        session.start_motion(10.)


def test_neutral_only_checks_selected_pair_and_retains_full_six_node_group():
    session = neutral_session(proximal=2, distal=4)
    bundle = session.export_bundle()
    record = bundle["attempts"][0]
    assert record["kind"] == "neutral" and record["status"] == "completed"
    assert {sample.node_id for sample in record["samples"]} == set(range(1, 7))
    assert record["matched_pairs"] == 150
    assert len(record["samples"]) == 900
    assert not session.snapshot()["needs_neutral"]
    assert record["metrics"]["gyro_vector_std_dps"] == [0., 0.]


def test_motion_request_contract_and_one_shot_token_delivery():
    session, request = fitting_session()
    assert session.take_fit_request() is None
    assert request["motion_capture"]["prox_gyro_dps"].shape == (600, 3)
    assert request["neutral_capture"]["prox_acc_raw"].shape == (150, 3)
    metadata = request["motion_capture"]["metadata"]
    assert metadata["attempt_id"] == 2 and metadata["connection_id"] == 1
    assert metadata["capture_start_s"] == 11. and metadata["capture_end_s"] == 31.
    assert metadata["proximal"] == 5 and metadata["distal"] == 6
    assert request["accel_g_per_lsb"] == ACC


def test_real_calibrator_request_to_frames_to_current_pose_and_streaming_angle():
    """Analytic two-body motion, quantized packets, real solver, no fake profile."""
    session = HingeSession(config())
    mp = to_matrix(from_axis_angle([.3, -.4, .8], .9))
    md = to_matrix(from_axis_angle([.8, .2, -.3], -1.1))
    gravity = np.array([.25, .2, np.sqrt(1. - .25**2 - .2**2)])
    bp, bd = np.array([.6, -.3, .2]), np.array([-.2, .4, .1])
    def physical(t):
        e = max(0., t - 11.)
        if t <= 8.:
            theta, theta_rate = 0., 0.
        elif t < 11.:
            theta, theta_rate = .55 * (t - 8.) / 3., .55 / 3.
        else:
            theta, theta_rate = .55 + .5 * np.sin(1.8 * e), .9 * np.cos(1.8 * e)
        roll, pitch, yaw = .3 * np.sin(1.4 * e), .45 * np.sin(1.1 * e), .4 * np.sin(.7 * e)
        rx = to_matrix(from_axis_angle([1, 0, 0], roll))
        ry = to_matrix(from_axis_angle([0, 1, 0], pitch))
        rz = to_matrix(from_axis_angle([0, 0, 1], yaw))
        h = to_matrix(from_axis_angle([1, 0, 0], theta))
        omega = np.zeros(3) if t < 11. else (rx.T @ ry.T @ [0, 0, .28 * np.cos(.7 * e)]
                  + rx.T @ [0, .495 * np.cos(1.1 * e), 0] + [.42 * np.cos(1.4 * e), 0, 0])
        pg = np.degrees(omega) @ mp + bp
        dg = np.degrees(h.T @ omega + [theta_rate, 0, 0]) @ md + bd
        ap = (rz @ ry @ rx).T @ gravity
        da = h.T @ ap
        values = {
            5: tuple(np.rint(np.r_[pg / GYRO, (ap @ mp) / ACC]).astype(int)),
            6: tuple(np.rint(np.r_[dg / GYRO, (da @ md) / ACC]).astype(int)),
        }
        rows = [Sample(node, values.get(node, (0, 0, 0, 0, 0, 8197)), t, 1000. + t) for node in range(1, 7)]
        return rows, np.degrees(theta)
    session.connect(0.)
    session.start_neutral(0.)
    for tick in range(241):
        session.feed(physical(tick / RATE)[0])
    assert session.snapshot()["state"] == "neutral_ready"
    session.start_motion(8.)
    for tick in range(241, 931):
        session.feed(physical(tick / RATE)[0])
    request = session.take_fit_request()
    assert request is not None
    fitted = calibrate_hinge_pair(request["neutral_capture"], request["motion_capture"],
                                  accel_g_per_lsb=request["accel_g_per_lsb"])
    assert fitted["profile_status"] == "offline_candidate_ready", fitted["reason_codes"]
    assert session.accept_fit(request["token"], fitted)
    sign = np.sign(np.dot(fitted["axis_fit"]["axis_prox"], mp.T @ [1, 0, 0]))
    assert session.snapshot()["math_angle_deg"] == pytest.approx(sign * physical(31.)[1], abs=.15)
    errors = []
    for tick in range(931, 991):
        rows, truth = physical(tick / RATE)
        current = session.feed(rows)
        assert current["preview_valid"]
        errors.append(abs(current["math_angle_deg"] - sign * truth))
    assert max(errors) < 1.5  # Coarse 30 Hz analytic-rate integration, not hardware accuracy.
    assert session.export_bundle()["profile"]["usable_as_live_calibration"] is False


def test_candidate_initializes_at_current_bent_pose_not_a_fictional_zero():
    session = ready_session(angle=37.)
    state = session.snapshot()
    assert state["state"] == "candidate_ready" and state["preview_valid"]
    assert state["math_angle_deg"] == pytest.approx(37., abs=.02)
    assert state["anatomical_angle_deg"] is None
    assert state["direction"]["status"] == "unconfirmed"
    assert session.export_bundle()["profile"]["usable_as_live_calibration"] is False
    sample_count = state["observer"]["samples"]
    angle = state["math_angle_deg"]
    session.tick(session._now + .001)
    for _ in range(10):
        assert session.snapshot()["observer"]["samples"] == sample_count
        assert session.snapshot()["math_angle_deg"] == angle


def test_pair_outputs_are_one_per_pair_even_with_identical_host_stamps():
    session = ready_session(angle=37.)
    session.take_pair_outputs()
    base = session.snapshot()["matched_pairs"]
    t = session._now + 1. / RATE
    inputs = [frame(t, angle=angle, speed=3.) for angle in (37.1, 37.2, 37.3)]
    session.feed([sample for group in inputs for sample in group], now_s=t)
    result = session.take_pair_outputs()
    assert result["discarded_rows"] == 0
    assert [row["pair_sequence"] for row in result["rows"]] == [base + 1, base + 2, base + 3]
    assert [row["host_monotonic_s"] for row in result["rows"]] == [t] * 3
    assert [row["host_wall_s"] for row in result["rows"]] == [1000. + t] * 3
    for output, raw in zip(result["rows"], inputs):
        assert output["proximal_raw"] == list(raw[4].values)
        assert output["distal_raw"] == list(raw[5].values)
        assert output["angle_deg"] > 30. and output["current_angle_observed"]
    assert session.take_pair_outputs() == {"rows": [], "discarded_rows": 0}


def test_accept_fit_and_ui_reads_never_add_pair_outputs_or_relabel_old_pair():
    session, request = fitting_session()
    session.take_pair_outputs()
    session.feed(frame(session._now + 1. / RATE, angle=37.))
    assert session.accept_fit(request["token"], profile())
    rows = session.take_pair_outputs()["rows"]
    # The raw pair arrived BEFORE its profile: preserve its actual unavailable output.
    assert len(rows) == 1 and rows[0]["angle_deg"] is None
    assert rows[0]["valid"] is False and rows[0]["current_angle_observed"] is False
    assert "no_current_candidate" in rows[0]["reason_codes"]
    assert session.snapshot()["math_angle_deg"] == pytest.approx(37., abs=.02)
    for _ in range(4):
        session.tick(session._now)
        session.snapshot()
        assert session.take_pair_outputs() == {"rows": [], "discarded_rows": 0}
    session.feed(frame(session._now + 1. / RATE, angle=37.))
    actual = session.take_pair_outputs()["rows"]
    assert len(actual) == 1 and actual[0]["angle_deg"] == pytest.approx(37., abs=.02)


def test_pair_collection_preserves_online_bias_nonzero_pose_and_snapshot_independence():
    session = ready_session(angle=37.)
    session.take_pair_outputs()
    for _ in range(450):
        rows = frame(session._now + 1. / RATE, angle=37.)
        p = rows[4]
        rows[4] = Sample(5, (103, *p.values[1:]), p.host_monotonic_s, p.host_wall_s)
        session.feed(rows)
    before = session.snapshot()["observer"]
    assert before["relative_bias_dps"] > .3
    batch = session.take_pair_outputs()
    actual = batch["rows"][-1]
    assert len(batch["rows"]) == 450 and batch["discarded_rows"] == 0
    for field in ("angle_deg", "relative_bias_dps", "angle_std_deg", "rate_dps",
                  "valid", "current_angle_observed", "quality", "history_loss"):
        assert actual[field] == before[field]
    assert actual["angle_deg"] == pytest.approx(37., abs=.2)
    assert session.snapshot()["observer"] == before
    actual["angle_deg"] = -999
    actual["proximal_raw"][0] = -999
    actual["observation_sources"].append("mutated")
    actual["rejection_counts"]["mutated"] = 123
    assert session.snapshot()["observer"] == before
    assert session.export_bundle()["runtime_results"][-1]["mathematical_angle_deg"] == before["angle_deg"]
    session.feed(frame(session._now + 1. / RATE, angle=37.))
    following = session.take_pair_outputs()["rows"][0]
    assert "mutated" not in following["observation_sources"]
    assert "mutated" not in following["rejection_counts"]
    json.dumps(batch, allow_nan=False)


def test_matched_pair_after_continuity_loss_has_explicit_unavailable_output():
    session = ready_session(angle=37.)
    session.take_pair_outputs()
    rows = frame(session._now + 1. / RATE, angle=37.)
    session.feed([rows[4], rows[0], rows[5]])
    assert session.take_pair_outputs() == {"rows": [], "discarded_rows": 0}
    session.feed(frame(session._now + 1. / RATE, angle=37.))
    output = session.take_pair_outputs()["rows"][0]
    assert output["angle_deg"] is None and output["relative_bias_dps"] is None
    assert output["angle_std_deg"] is None and output["history_loss"]
    assert not output["valid"] and not output["current_angle_observed"]
    assert output["quality"] == "needs_neutral"
    assert "session_needs_explicit_neutral" in output["reason_codes"]
    assert output["reason"]


def test_pair_output_queue_is_bounded_and_overflow_count_is_drain_local():
    session = HingeSession(config())
    session.connect(0.)
    pair = frame(0.)[4:]
    session.feed(pair * 30000)
    session.feed(pair * 2)
    result = session.take_pair_outputs()
    assert len(result["rows"]) == 30000
    assert result["discarded_rows"] == 2
    assert result["rows"][0]["pair_sequence"] == 3
    assert result["rows"][-1]["pair_sequence"] == 30002
    assert result["rows"][-1]["angle_deg"] is None
    assert session.take_pair_outputs() == {"rows": [], "discarded_rows": 0}
    session.feed(pair)
    next_batch = session.take_pair_outputs()
    assert next_batch["discarded_rows"] == 0
    assert next_batch["rows"][0]["pair_sequence"] == 30003


def test_actual_session_outputs_export_and_auto_validate_without_replay(tmp_path, monkeypatch):
    """Session -> raw/online/profile export -> protocol, without device or rerun."""
    from vt_glove_pose import hinge_validation_io
    from vt_glove_pose.hinge_export import export_hinge_session

    session = ready_session(angle=37.)

    def biased_frame():
        rows = frame(session._now + 1. / RATE, angle=37.)
        proximal = rows[4]
        rows[4] = Sample(5, (103, *proximal.values[1:]),
                         proximal.host_monotonic_s, proximal.host_wall_s)
        session.feed(rows)
        return rows

    # Let the real online observer accumulate bias BEFORE validation begins.
    for _ in range(120):
        biased_frame()
    session.take_pair_outputs()
    samples = []
    for _ in range(45):
        samples.extend(biased_frame())
    captured = session.take_pair_outputs()
    online = captured["rows"]
    assert len(online) == 45 and captured["discarded_rows"] == 0
    assert online[0]["angle_deg"] > 30.
    assert online[0]["relative_bias_dps"] > .01
    start, end = samples[0].host_monotonic_s, samples[-1].host_monotonic_s + 1. / RATE
    frozen_profile = session.export_bundle()["profile"]
    folder = tmp_path / "actual_online_session"
    export_hinge_session(folder, {}, config=session.config, validation_attempts=[dict(
        kind="validation", started_s=start, ended_s=end, duration_s=end-start, status="complete",
        samples=samples, series=[], online=online, online_discarded_rows=captured["discarded_rows"],
        profile=frozen_profile, markers=[dict(kind="reference", pose_id="held_bent_37deg",
            window_id=1, status="completed", start_host_monotonic_s=start,
            end_host_monotonic_s=end)])])

    def forbidden_replay(*args, **kwargs):
        pytest.fail("auto must use actual online output, not reinitialize or replay the observer")

    monkeypatch.setattr(hinge_validation_io, "replay_hinge_pair", forbidden_replay)
    report = hinge_validation_io.validate_hinge_protocol(folder / "001_validation_protocol.json", source="auto")
    provenance = report["provenance"]
    assert provenance["evaluation_source"] == "online"
    assert provenance["observer_state_basis"] == "actual_recorded_per_pair_online_output"
    assert provenance["exported_hash_verified"] == {"capture": True, "profile": True, "online": True}
    summary = provenance["replay_summary"]
    assert summary["matched_online_rows"] == 45 and summary["missing_online_rows"] == 0
    assert summary["final_angle_deg"] == online[-1]["angle_deg"]
    assert summary["final_relative_bias_dps"] == online[-1]["relative_bias_dps"]
    assert summary["final_model_angle_std_deg"] == online[-1]["angle_std_deg"]
    assert summary["angle_range_deg"] == [min(row["angle_deg"] for row in online),
                                          max(row["angle_deg"] for row in online)]
    reference = report["windows"][0]
    assert reference["sample_count"] == 45 and reference["status"] == "reference_ready"
    assert reference["angle"]["median_deg"] == np.median([row["angle_deg"] for row in online])
    assert reference["angle"]["min_deg"] > 30.  # No artificial neutral initialization.
    assert report["absolute_accuracy"]["status"] == "not_evaluated"


@pytest.mark.parametrize("failure", ["order", "timestamp", "host_gap", "stale"])
def test_pair_continuity_failure_invalidates_preview_without_crashing(failure):
    session = ready_session()
    t = session._now
    if failure == "order":
        rows = frame(t + 1. / RATE)
        session.feed([rows[4], rows[0], rows[5]])
    elif failure == "timestamp":
        rows = frame(t + 1. / RATE)
        different = Sample(6, rows[5].values, t + 2. / RATE, 1000. + t + 2. / RATE)
        session.feed([rows[4], different])
    elif failure == "host_gap":
        session.feed(frame(t + 1.))
    else:
        session.tick(t + 1.)
    state = session.snapshot()
    assert state["connected"]
    assert state["needs_neutral"] and not state["preview_valid"]
    assert state["math_angle_deg"] is None
    assert "失效" in state["status_message"]
    assert len(session.export_bundle()["attempts"]) == 2


def test_other_wire_group_never_satisfies_pair_freshness():
    session = HingeSession(config())
    session.connect(0.)
    session.start_neutral(0.)
    for tick in range(31):
        t = tick / RATE
        session.feed(frame(t, proximal=7, distal=8))
    state = session.snapshot()
    assert state["needs_neutral"]
    assert state["matched_pairs"] == 0
    assert session.export_bundle()["attempts"][0]["status"] == "interrupted"


def test_cancel_and_stale_background_results_do_not_replace_newer_session():
    session, request = fitting_session()
    session.cancel()
    assert not session.accept_fit(request["token"], profile())
    assert not session.reject_fit(request["token"], "late failure")
    assert session.export_bundle()["profile"] is None
    assert len(session.export_bundle()["motion_samples"]) == 3600
    session.start_motion(31.)
    pump(session, 54.)
    second = session.take_fit_request()
    assert second["token"] != request["token"]
    assert not session.accept_fit(request["token"], profile())
    assert session.accept_fit(second["token"], profile())


def test_distinct_controllers_have_distinct_fit_tokens():
    first, a = fitting_session()
    second, b = fitting_session()
    assert a["token"] != b["token"]
    assert not second.accept_fit(a["token"], profile())


def test_disconnect_retains_raw_and_reconnect_requires_fresh_explicit_neutral():
    session = ready_session()
    original = session.export_bundle()
    session.disconnect()
    assert len(session.export_bundle()["motion_samples"]) == len(original["motion_samples"])
    session.connect(100.)
    session.feed(frame(100.))
    assert session.snapshot()["needs_neutral"]
    assert session.snapshot()["observer"] is None
    with pytest.raises(ValueError, match="neutral"):
        session.start_motion(100.)
    assert len(session.export_bundle()["attempts"]) == 2


def test_cancelled_capture_kept_and_export_does_not_alias_internal_lists():
    session = HingeSession(config())
    session.connect(0.)
    session.start_neutral(0.)
    pump(session, 4.)
    session.cancel()
    bundle = session.export_bundle()
    assert bundle["attempts"][0]["status"] == "cancelled"
    assert len(bundle["neutral_samples"]) > 0
    raw = bundle["neutral_samples"][0]
    assert raw is bundle["attempts"][0]["samples"][0]  # Immutable Sample is shared safely.
    bundle["attempts"][0]["samples"].clear()
    bundle["neutral_samples"].clear()
    assert len(session.export_bundle()["neutral_samples"]) > 0


def test_raw_retention_limit_stops_capture_instead_of_overwriting_evidence():
    session = HingeSession(config(), max_samples=60)
    session.connect(0.)
    session.start_neutral(0.)
    pump(session, 4.)
    bundle = session.export_bundle()
    assert len(bundle["neutral_samples"]) == 60
    assert bundle["attempts"][0]["status"] == "interrupted"
    assert bundle["attempts"][0]["reason"] == "retained_raw_buffer_full_export_required"
    with pytest.raises(ValueError, match="buffer is full"):
        session.start_neutral(4.)


def test_noisy_neutral_is_rejected_but_its_raw_window_survives():
    session = HingeSession(config())
    session.connect(0.)
    session.start_neutral(0.)
    pump(session, 8., movement=lambda t: (0., 15. * math.sin(10. * t)))
    state = session.snapshot()
    assert state["state"] == "needs_neutral"
    assert "neutral_gyro_not_quiet" in state["status_message"]
    assert len(session.export_bundle()["neutral_samples"]) == 900


def test_rejected_profile_has_actionable_reason_and_previous_success_is_not_current():
    session = ready_session()
    session.start_motion(session._now)
    assert session.export_bundle()["profile"] is None
    pump(session, session._now + 23.)
    request = session.take_fit_request()
    failed = dict(profile_status="axis_insufficient_excitation", reason_codes=["training_tangent_sensitivity_too_small"])
    assert not session.accept_fit(request["token"], failed)
    state = session.snapshot()
    assert state["state"] == "neutral_ready"
    assert "axis_insufficient_excitation" in state["status_message"]
    assert "training_tangent_sensitivity_too_small" in state["status_message"]
    assert "轻缓" in state["status_message"]
    bundle = session.export_bundle()
    assert bundle["profile"] is None
    assert bundle["fit_reports"][0]["profile"]["profile_status"] == "offline_candidate_ready"
    assert bundle["fit_reports"][-1]["profile"] == failed


@pytest.mark.parametrize("status,reason,action,needs_neutral", [
    ("frames_neutral_unobservable", "explicit_neutral_gravity_nearly_parallel_to_hinge", "neutral", True),
    ("frames_inconsistent", "explicit_neutral_acceleration_norm_inconsistent", "export", True),
    ("frames_inconsistent", "explicit_neutral_axis_gravity_mismatch", "export", False),
    ("frames_sign_unobservable", "insufficient_parallel_gravity_component_to_resolve_axis_sign", "motion", False),
    ("axis_insufficient_excitation", "training_tangent_sensitivity_too_small", "motion", False),
    ("frames_inconsistent", "insufficient_norm_consistent_acceleration", "export", False),
])
def test_fit_recovery_routes_by_exact_reason_and_preserves_failed_evidence(status, reason, action, needs_neutral):
    session, request = fitting_session()
    failed = dict(profile_status=status, reason_codes=[reason])
    assert not session.accept_fit(request["token"], failed)
    state = session.snapshot()
    assert state["needs_neutral"] is needs_neutral
    assert state["state"] == ("needs_neutral" if needs_neutral else "neutral_ready")
    recovery = state["recovery"]
    assert recovery["suggested_action"] == action
    assert recovery["requires_new_neutral"] is needs_neutral
    bundle = session.export_bundle()
    assert bundle["fit_reports"][-1]["profile"] == failed
    assert bundle["fit_reports"][-1]["recovery"] == recovery
    assert len(bundle["neutral_samples"]) == 900 and len(bundle["motion_samples"]) == 3600
    recovery["reason_codes"].append("caller_mutation")
    recovery["instruction"] = "caller_mutation"
    assert "caller_mutation" not in session.snapshot()["recovery"]["reason_codes"]
    assert session.export_bundle()["fit_reports"][-1]["recovery"]["instruction"] != "caller_mutation"
    # A review recommendation adds no separate authority gate to advanced actions.
    if needs_neutral:
        session.start_neutral(session._now)
    else:
        session.start_motion(session._now)
    assert session.snapshot()["recovery"] is None
    assert session.export_bundle()["fit_reports"][-1]["recovery"] is not None


def test_unknown_fit_exception_is_preserved_for_review_and_recovery_clears_on_reconnect():
    session, request = fitting_session()
    assert session.reject_fit(request["token"], "unexpected solver configuration")
    recovery = session.snapshot()["recovery"]
    assert recovery["suggested_action"] == "export" and recovery["needs_review"]
    session.disconnect()
    assert session.snapshot()["recovery"] is None
    session.connect(session._now)
    assert session.snapshot()["recovery"] is None
    history = session.export_bundle()["fit_reports"][-1]
    assert history["recovery"] == recovery and "configuration" in history["error"]


def test_retry_success_clears_recovery_without_erasing_previous_failure():
    session, request = fitting_session()
    assert not session.accept_fit(request["token"], dict(profile_status="axis_insufficient_excitation",
        reason_codes=["training_motion_below_noise"]))
    session.start_motion(session._now)
    assert session.snapshot()["recovery"] is None
    pump(session, session._now + 23.)
    retry = session.take_fit_request()
    assert session.accept_fit(retry["token"], profile())
    assert session.snapshot()["recovery"] is None
    history = session.export_bundle()["fit_reports"]
    assert history[-2]["recovery"]["suggested_action"] == "motion"
    assert history[-1]["error"] is None and history[-1]["recovery"] is None


def test_neutral_variation_recovery_uses_metrics_and_keeps_failed_capture():
    session = HingeSession(config())
    session.connect(0.)
    session.start_neutral(0.)
    for tick in range(241):
        t = tick / RATE
        rows = frame(t)
        # Alternating +/- 0.08 g changes the current std, not its mean norm.
        for index in (4, 5):
            old = rows[index]
            ax = round((.08 if tick % 2 else -.08) / ACC)
            rows[index] = Sample(old.node_id, (*old.values[:3], ax, *old.values[4:]),
                                 old.host_monotonic_s, old.host_wall_s)
        session.feed(rows)
    state = session.snapshot()
    assert state["state"] == "needs_neutral"
    assert state["recovery"]["suggested_action"] == "neutral"
    assert "加速度有波动" in state["recovery"]["title"]
    failed = session.export_bundle()["attempts"][0]
    assert failed["status"] == "rejected" and len(failed["samples"]) == 900
    assert failed["recovery"] == state["recovery"]
    session.start_neutral(session._now)
    assert session.snapshot()["recovery"] is None
    pump(session, session._now + 8.)
    assert session.snapshot()["state"] == "neutral_ready"
    assert session.snapshot()["recovery"] is None
    assert session.export_bundle()["attempts"][0]["recovery"] == failed["recovery"]


def test_failed_calibrator_neutral_metrics_reach_recovery_classifier():
    session, request = fitting_session()
    failed = dict(profile_status="neutral_invalid", reason_codes=["neutral_not_quiet_or_gravity_like"],
                  neutral=dict(mean_accel_norm_g=[.99, 1.01], accel_vector_std_g=[.04, .02]))
    assert not session.accept_fit(request["token"], failed)
    recovery = session.snapshot()["recovery"]
    assert recovery["suggested_action"] == "neutral" and not recovery["needs_review"]


def test_pair_loss_clears_obsolete_motion_recovery_but_preserves_failure_history():
    session, request = fitting_session()
    assert not session.accept_fit(request["token"], dict(profile_status="axis_insufficient_excitation",
        reason_codes=["training_motion_below_noise"]))
    assert session.snapshot()["recovery"]["suggested_action"] == "motion"
    session.tick(session._now + 1.)
    assert session.snapshot()["needs_neutral"] and session.snapshot()["recovery"] is None
    session.feed(frame(session._now + 1. / RATE))
    state = session.snapshot()
    assert state["needs_neutral"] and state["state"] == "needs_neutral"
    assert state["recovery"] is None
    assert session.export_bundle()["fit_reports"][-1]["recovery"]["suggested_action"] == "motion"


def test_direction_uses_first_bout_not_endpoint_and_can_start_bent():
    session = ready_session(angle=37.)
    started = session._now
    session.start_direction(started)
    capture_start = started + 3.
    def movement(t):
        e = t - capture_start
        if e <= 0:
            return 37., 0.
        if e <= 1.:
            return 37. + 20. * e, 20.
        if e <= 2.:
            return 57. - 20. * (e - 1.), -20.
        return 37. - 30. * (e - 2.), -30.
    pump(session, started + 7., movement=movement)
    state = session.snapshot()
    assert state["state"] == "candidate_ready"
    assert state["direction"]["status"] == "confirmed", state["direction"]
    assert state["direction"]["multiplier"] == 1
    assert state["direction"]["first_bout_delta_deg"] > 15.
    assert state["math_angle_deg"] < -15.
    assert state["anatomical_angle_deg"] == state["math_angle_deg"]
    assert session.export_bundle()["profile"]["anatomical_flexion_sign_verified"] is False


def test_tiny_first_bout_is_not_replaced_by_later_large_opposite_excursion():
    session = ready_session(angle=20.)
    started = session._now
    session.start_direction(started)
    capture_start = started + 3.
    def movement(t):
        e = t - capture_start
        if e <= 0:
            return 20., 0.
        if e <= 1.:
            return 20. + 4. * e, 4.
        return 24. - 20. * (e - 1.), -20.
    pump(session, started + 7., movement=movement)
    state = session.snapshot()
    assert state["direction"]["status"] == "failed"
    assert state["direction"]["multiplier"] is None
    assert state["anatomical_angle_deg"] is None
    assert abs(state["math_angle_deg"]) > 15.
    assert state["preview_valid"]


def test_bundle_metadata_is_json_safe_after_immutable_samples_are_removed():
    session = ready_session()
    bundle = session.export_bundle()
    for record in bundle["attempts"]:
        record.pop("samples")
    for key in ("neutral_samples", "motion_samples", "direction_samples"):
        bundle.pop(key)
    json.dumps(bundle, allow_nan=False)


@pytest.mark.parametrize("nodes", [(5, 7), (1, 1), (0, 2), (True, 2)])
def test_invalid_pair_configuration_is_rejected(nodes):
    with pytest.raises(ValueError):
        HingeSession(config(), *nodes)


def test_invalid_feed_is_transactional_and_profile_input_is_not_mutated():
    session = HingeSession(config())
    session.connect(0.)
    before = session.snapshot()
    with pytest.raises(ValueError):
        session.feed([object()])
    assert session.snapshot() == before
    with pytest.raises(ValueError):
        session.tick(float("nan"))
    assert session.snapshot() == before
