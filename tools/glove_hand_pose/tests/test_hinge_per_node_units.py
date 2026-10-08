"""Explicit mixed sensitivities propagate without rewriting saved evidence."""
from copy import deepcopy
import json

import numpy as np
import pytest

from vt_glove_pose.cli import inspect_csv, main
from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_export import export_hinge_session
from vt_glove_pose.hinge_online import online_hinge_report
from vt_glove_pose.hinge_replay import _profile, replay_hinge_pair, calibrate_hinge_pair
from vt_glove_pose.hinge_session import HingeSession
from vt_glove_pose.hinge_settings import hinge_settings_match
from vt_glove_pose.hinge_validation_io import validate_hinge_protocol
from vt_glove_pose.pair_capture import load_pair_capture
from vt_glove_pose.records import Sample
from vt_glove_pose.stream import RawRecorder


OVERRIDES = {"5": .0175}


def config():
    value = load_config()
    value.update(sample_rate_hz=30., gyro_dps_per_lsb_by_node=dict(OVERRIDES))
    return value


def profile():
    identity = np.eye(3).tolist()
    return dict(kind="vt_glove_pose.offline_hinge_profile", schema_version=1,
        offline_only=True, usable_as_live_calibration=False, profile_status="offline_candidate_ready",
        settings=dict(node_ids=[5, 6], sample_rate_hz=30., gyro_dps_per_lsb=.00875,
                      gyro_dps_per_lsb_by_node=dict(OVERRIDES), accel_g_per_lsb=.000122),
        axis_fit=dict(candidate_status="candidate_supported"),
        frames=dict(status="frames_ready", prox_sensor_to_functional=identity, dist_sensor_to_functional=identity),
        neutral=dict(prox_bias_dps=[0., 0., 0.], dist_bias_dps=[0., 0., 0.],
                     prox_acc_g=[0., 0., 1.], dist_acc_g=[0., 0., 1.]))


def samples(n=3, *, raw=100, start=10.):
    return [Sample(node, (raw, 0, 0, 0, 0, 8197), start+i/30., 1000.+start+i/30.)
            for i in range(n) for node in range(1, 7)]


def capture_file(tmp_path, *, n=3, raw=100):
    path = tmp_path/"capture.csv"
    with RawRecorder(path) as recorder:
        recorder.write(samples(n, raw=raw))
    return path


def capture(path, overrides=OVERRIDES):
    return load_pair_capture(path, sample_rate_hz=30., gyro_dps_per_lsb_by_node=overrides)


def test_pair_reader_freezes_effective_selected_scales_and_preserves_raw(tmp_path):
    path = capture_file(tmp_path)
    old = capture(path, None)
    data = capture(path)
    assert "gyro_dps_per_lsb_by_node" not in old["metadata"]
    assert data["metadata"]["gyro_dps_per_lsb_by_node"] == {"5": .0175, "6": .00875}
    np.testing.assert_allclose(data["prox_gyro_dps"], old["prox_gyro_dps"]*2)
    np.testing.assert_allclose(data["dist_gyro_dps"], old["dist_gyro_dps"])
    np.testing.assert_array_equal(data["prox_gyro_raw"], old["prox_gyro_raw"])
    assert data["metadata"]["sha256_read_prefix"] == old["metadata"]["sha256_read_prefix"]
    reverse = load_pair_capture(path, proximal=6, distal=5, gyro_dps_per_lsb_by_node=OVERRIDES)
    np.testing.assert_allclose(reverse["prox_gyro_dps"], data["dist_gyro_dps"])
    np.testing.assert_allclose(reverse["dist_gyro_dps"], data["prox_gyro_dps"])


@pytest.mark.parametrize("overrides", [{5: .0175}, {"05": .0175}, {"13": .0175},
                                      {"1": True}, {"1": float("nan")}, []])
def test_pair_reader_rejects_invalid_maps_even_for_unselected_nodes(tmp_path, overrides):
    with pytest.raises(ValueError, match="gyro_dps_per_lsb_by_node"):
        capture(capture_file(tmp_path), overrides)


def test_settings_match_effective_units_not_fallback_or_dictionary_shape():
    left = profile()["settings"]
    same = dict(left, gyro_dps_per_lsb=.0175, gyro_dps_per_lsb_by_node={"6": .00875, "1": .07})
    assert hinge_settings_match(left, same)
    assert not hinge_settings_match(left, dict(left, gyro_dps_per_lsb_by_node={"6": .0175}))
    legacy = dict(left)
    del legacy["gyro_dps_per_lsb_by_node"]
    assert hinge_settings_match(legacy, dict(legacy, gyro_dps_per_lsb_by_node={"5": .00875}))


def test_legacy_profile_still_loads_and_mixed_profile_rejects_uniform_capture(tmp_path):
    candidate = profile()
    old = deepcopy(candidate)
    del old["settings"]["gyro_dps_per_lsb_by_node"]
    assert "gyro_dps_per_lsb_by_node" not in _profile(old)[0]
    path = capture_file(tmp_path)
    assert replay_hinge_pair(candidate, capture(path))["replay_status"] == "completed"
    with pytest.raises(ValueError, match="differ"):
        replay_hinge_pair(candidate, capture(path, None))
    recorded = capture(path)
    recorded["metadata"]["source_configuration"] = load_config()
    recorded["metadata"]["source_configuration"]["sample_rate_hz"] = 30.
    with pytest.raises(ValueError, match="recorded"):
        replay_hinge_pair(candidate, recorded)


def test_calibration_keeps_mixed_units_and_rejects_changed_reference_units(tmp_path):
    path = capture_file(tmp_path, n=120, raw=0)
    data = capture(path)
    result = calibrate_hinge_pair(data, data, accel_g_per_lsb=.000122)
    assert result["settings"]["gyro_dps_per_lsb_by_node"] == {"5": .0175, "6": .00875}
    assert result["profile_status"] == "insufficient_motion"
    assert result["gyro_scale_diagnostics"]["proximal"]["status"] == "insufficient_information"
    assert result["gyro_scale_diagnostics"]["proximal"]["configured_gyro_dps_per_lsb"] == .0175
    assert result["gyro_scale_diagnostics"]["distal"]["configured_gyro_dps_per_lsb"] == .00875
    json.dumps(result, allow_nan=False)
    with pytest.raises(ValueError, match="same nodes, rate and units"):
        calibrate_hinge_pair(data, capture(path, None), accel_g_per_lsb=.000122)


def test_calibration_records_scale_suspicion_without_correcting_or_accepting_axes(tmp_path):
    path = capture_file(tmp_path, n=300, raw=0)
    still = capture(path, None)
    motion = deepcopy(still)
    t = np.arange(300)/30.
    angle = .5*np.sin(t)
    # Physical angle derivative is twice the configured gyro. Both nodes rotate
    # together around one axis, so motion is still insufficient for an axis fit.
    gyro = np.column_stack((np.rad2deg(.25*np.cos(t)), np.zeros((300, 2))))
    acc = np.column_stack((np.zeros(300), np.sin(angle), np.cos(angle)))
    for role in ("prox", "dist"):
        motion[role+"_gyro_dps"] = gyro.copy()
        motion[role+"_acc_raw"] = np.rint(acc/.000122).astype(np.int16)
    result = calibrate_hinge_pair(still, motion, accel_g_per_lsb=.000122)
    for item in result["gyro_scale_diagnostics"].values():
        assert item["status"] == "possible_scale_mismatch"
        assert item["suggested_multiplier"] == pytest.approx(2., rel=.05)
        assert not item["changes_configuration"]
    assert result["profile_status"] != "offline_candidate_ready"
    assert result["settings"]["gyro_dps_per_lsb"] == .00875
    assert "gyro_dps_per_lsb_by_node" not in result["settings"]
    json.dumps(result, allow_nan=False)


def test_session_capture_and_observer_both_use_selected_node_units():
    session = HingeSession(config())
    pair = tuple(sample for sample in samples(1) if sample.node_id in (5, 6))
    data = session._capture(dict(pairs=[pair], kind="motion", attempt_id=1, connection=1,
                                capture_start_s=10., capture_end_s=11.))
    assert data["prox_gyro_dps"][0, 0] == pytest.approx(1.75)
    assert data["dist_gyro_dps"][0, 0] == pytest.approx(.875)
    assert data["metadata"]["gyro_dps_per_lsb_by_node"] == {"5": .0175, "6": .00875}
    settings, matrices, vectors = _profile(profile())
    session._transforms = (matrices, vectors)
    received = []
    class Observer:
        def update(self, pg, pa, dg, da, **kwargs):
            received.append((pg.copy(), dg.copy()))
            return dict(angle_deg=0., valid=True, quality="tracking", history_loss=False,
                        current_angle_observed=True, angle_std_deg=.5, relative_bias_dps=0.)
    session._observer = Observer()
    session._update_observer(pair)
    assert received[0][0][0] == pytest.approx(1.75)
    assert received[0][1][0] == pytest.approx(.875)


@pytest.mark.parametrize("wrong", [False, True])
def test_session_rejects_profile_with_wrong_effective_scale_only(wrong):
    session = HingeSession(config())
    session._state, session._connected, session._needs_neutral = "fitting", True, False
    session._fit_token = "unit-test"
    candidate = profile()
    if wrong:
        del candidate["settings"]["gyro_dps_per_lsb_by_node"]
    assert session.accept_fit("unit-test", candidate) is not wrong


@pytest.mark.parametrize("source", ["replay", "online"])
def test_validation_export_and_reader_keep_frozen_mixed_units(tmp_path, source):
    raw = samples(n=120, raw=0)
    rows = [dict(pair_sequence=i+1, connection_id=1, host_monotonic_s=pair[0].host_monotonic_s,
                 host_wall_s=pair[0].host_wall_s, proximal_raw=list(pair[0].values),
                 distal_raw=list(pair[1].values), angle_deg=17., relative_bias_dps=0., angle_std_deg=.5,
                 valid=True, current_angle_observed=True, history_loss=False,
                 quality="tracking", raw_near_limit=False, host_gap_before=False)
            for i, pair in enumerate(zip(raw[4::6], raw[5::6]))]
    attempt = dict(kind="validation", started_s=10., ended_s=14., status="complete", samples=raw,
                   profile=profile(), markers=[], series=[])
    if source == "online":
        attempt.update(online=rows, online_discarded_rows=0)
    folder = tmp_path/"export"
    export_hinge_session(folder, {}, config=config(), validation_attempts=[attempt])
    report = validate_hinge_protocol(folder/"001_validation_protocol.json", source=source)
    assert report["provenance"]["profile_settings"]["gyro_dps_per_lsb_by_node"] == OVERRIDES
    assert report["provenance"]["source"]["gyro_dps_per_lsb_by_node"] == {"5": .0175, "6": .00875}


def test_online_trace_settings_cannot_silently_change_mixed_units(tmp_path):
    data = capture(capture_file(tmp_path))
    trace = dict(kind="vt_glove_pose.hinge_online_trace", schema_version=1,
                 settings=profile()["settings"], discarded_rows=0, series=[])
    wrong = deepcopy(trace)
    wrong["settings"]["gyro_dps_per_lsb_by_node"] = {"6": .0175}
    with pytest.raises(ValueError, match="trace settings differ"):
        online_hinge_report(wrong, data, profile()["settings"])


def test_cli_replay_uses_frozen_map_and_rejects_incompatible_explicit_config(tmp_path):
    path = capture_file(tmp_path)
    profile_path, config_path, output = tmp_path/"profile.json", tmp_path/"config.json", tmp_path/"result.json"
    profile_path.write_text(json.dumps(profile()), encoding="utf-8")
    assert main(["replay-hinge", str(path), "--profile", str(profile_path), "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert report["settings"]["gyro_dps_per_lsb_by_node"] == OVERRIDES
    wrong = config()
    del wrong["gyro_dps_per_lsb_by_node"]
    config_path.write_text(json.dumps(wrong), encoding="utf-8")
    assert main(["replay-hinge", str(path), "--profile", str(profile_path), "--config", str(config_path)]) == 2


def test_inspect_report_labels_effective_node_units_without_changing_raw_statistics(tmp_path):
    path = capture_file(tmp_path)
    mixed = inspect_csv(path, config())
    legacy = inspect_csv(path, load_config())
    assert mixed["gyro_dps_per_lsb_by_node"] == OVERRIDES
    assert mixed["nodes"]["5"]["configured_gyro_dps_per_lsb"] == .0175
    assert mixed["nodes"]["6"]["configured_gyro_dps_per_lsb"] == .00875
    assert mixed["nodes"]["5"]["raw_mean"] == legacy["nodes"]["5"]["raw_mean"]
    assert "gyro_dps_per_lsb_by_node" not in legacy
