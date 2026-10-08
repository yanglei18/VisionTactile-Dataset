"""v3 contract: calibration-relative projection, evidence-gated QMT heading.

The removed v2 assumptions (instant correction of an arbitrary first-frame
yaw and no output after one second without heading evidence) were the bugs,
not accuracy requirements. Real input/context checks and read-only semantics
remain enforced below; accuracy claims are limited to known synthetic motion.
"""
from copy import deepcopy
import importlib

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

pytest.importorskip("qmt")

from test_heading_correction import PAIRS, W, fixture_model, make_frame
from vt_fsglove12.hinge_estimator import FunctionalHingeEstimator


SLOTS = (2, 5, 8, 11, 15)
ROOTS = ((0., 0., 0.), (20., -15., 10.), (-30., 40., 90.),
         (100., 20., -130.), (-55., -20., 60.))


def frame_for_bend(model, stamp, bend, drift=0., *, root=ROOTS[0], vertical=False, tilt=0., offaxis=0.):
    _, _, _, k, local, rest, a0, axis = model
    frame, _ = make_frame(model, stamp)
    for data in frame["nodes"].values():
        data["sample_index"] = int(round(stamp * 120)) + 1
    for _, (pn, cn, parent_slot, child_slot) in PAIRS.items():
        world = (R.align_vectors(np.array([[0., 0., 1.]]),
                  (rest[child_slot] * a0).apply(axis)[None, :])[0] if vertical else
                 R.from_euler("xyz", root, degrees=True))
        parent = W.inv() * world
        relative = R.from_rotvec(axis * bend)
        if offaxis:
            orth = np.cross(axis, np.eye(3)[np.argmin(np.abs(axis))])
            orth /= np.linalg.norm(orth)
            relative = relative * R.from_rotvec(orth * np.deg2rad(offaxis))
        child = parent * rest[child_slot] * a0 * relative
        if tilt:
            tilt_axis = np.cross((world * rest[child_slot] * a0).apply(axis), [0., 0., 1.])
            tilt_axis /= np.linalg.norm(tilt_axis)
            child = W.inv() * R.from_rotvec(tilt_axis * np.deg2rad(tilt)) * W * child
        frame["nodes"][pn]["quat_xyzw"] = (k[parent_slot].inv() * parent * local[parent_slot]).as_quat().tolist()
        frame["nodes"][cn]["quat_xyzw"] = (R.from_euler("z", drift, degrees=True)
            * k[child_slot].inv() * child * local[child_slot]).as_quat().tolist()
    return frame


def apply_read_only(estimator, slot, frame, model):
    calibrator, binding, profile, *_ = model
    originals = deepcopy((frame, binding, profile, vars(calibrator)))
    angle, detail = estimator.apply(slot, frame, profile, calibrator, binding)
    assert (frame, binding, profile) == originals[:3]
    for name, value in originals[3].items():
        np.testing.assert_array_equal(getattr(calibrator, name), value)
    assert detail["valid"] == (detail["quality"] == "valid")
    assert detail["display_available"] == (angle is not None)
    assert detail["quality"] in ("valid", "degraded", "unavailable")
    return angle, detail


@pytest.mark.parametrize("slot", SLOTS)
def test_calibrated_projection_preserves_fast_bends_and_common_root_motion(slot):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    for index, (bend, root) in enumerate(zip((.1, 1.25, .35, 1.05, .2), ROOTS)):
        angle, detail = apply_read_only(estimator, slot,
            frame_for_bend(model, 1. + index / 30., bend, root=root), model)
        assert angle == pytest.approx(bend, abs=1e-7)
        assert detail["display_available"] and not detail["valid"]
        assert detail["heading_quality"] == "calibration_relative"
        assert detail["delta_rad"] == 0.


@pytest.mark.parametrize("slot", SLOTS)
def test_fixed_three_degree_model_error_and_turning_palm_cannot_create_heading_or_flexion(slot):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    actual = []
    for index in range(301):
        t = index / 30.
        root = (65*np.sin(t), 45*np.sin(.7*t), 75*np.cos(.3*t))
        angle, d = apply_read_only(estimator, slot,
            frame_for_bend(model, 1.+t, .6, root=root, offaxis=3.), model)
        actual.append(angle)
        assert d["delta_rad"] == 0. and not d["heading_updated"]
        assert d["display_available"] and d["quality"] == "degraded"
    np.testing.assert_allclose(actual, actual[0], atol=1e-10)
    assert d["heading_window"]["status"] in ("insufficient_local_articulation", "insufficient_heading_geometry")


@pytest.mark.parametrize("slot", SLOTS)
def test_pi_branch_and_quaternion_sign_changes_do_not_change_angle_definition(slot):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    for index, degrees in enumerate((170., 179., 181., 190., 179., 170.)):
        frame = frame_for_bend(model, 1. + index / 30., np.deg2rad(degrees))
        if index % 2:
            for node in frame["nodes"].values():
                node["quat_xyzw"] = (-np.array(node["quat_xyzw"])).tolist()
        angle, detail = apply_read_only(estimator, slot, frame, model)
        assert angle == pytest.approx(np.deg2rad(degrees), abs=1e-7)
        assert detail["continuity"]["branch_corrected"] == (degrees > 180.)


def test_unrelated_profile_and_finger_changes_do_not_reset_branch():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    apply_read_only(estimator, 8, frame_for_bend(model, 1., np.deg2rad(179.)), model)
    model[2]["profile_version"] = 100
    model[2]["functional_fingers"]["thumb"]["slot_refs"]["15"]["axis_local"] = [1., 0., 0.]
    model[1]["nodes_epochs"]["5"] = 100
    angle, detail = apply_read_only(estimator, 8,
        frame_for_bend(model, 1.+1/30, np.deg2rad(181.), vertical=True), model)
    assert angle == pytest.approx(np.deg2rad(181.), abs=1e-7)
    assert detail["continuity"]["reset_reason"] is None


@pytest.mark.parametrize("invalid", ["missing", "stale", "skew", "orientation", "quaternion", "epoch", "sample_reversed"])
def test_real_sensor_faults_are_unavailable_and_cannot_be_hidden_by_heading_hold(invalid):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    apply_read_only(estimator, 8, frame_for_bend(model, 1., .4), model)
    bad = frame_for_bend(model, 1. + 1. / 30., .7)
    node = bad["nodes"]["10"]
    if invalid == "missing":
        del bad["nodes"]["10"]
    elif invalid in ("stale", "skew"):
        node["age_s"] = .2 if invalid == "stale" else .04
    elif invalid == "orientation":
        node["orientation_valid"] = False
    elif invalid == "quaternion":
        node["quat_xyzw"] = [0., 0., 0., 0.]
    elif invalid == "epoch":
        node["epoch"] = 9
    else:
        node["sample_index"] = 1
    angle, detail = apply_read_only(estimator, 8, bad, model)
    assert angle is None and not detail["display_available"] and not detail["valid"]
    assert detail["hard_reset"] == (invalid not in ("missing", "stale", "skew"))


@pytest.mark.parametrize("change", ["reset", "reference", "installation", "epoch", "session", "gap", "clock"])
def test_context_or_source_discontinuity_clears_window_and_angle_branch(change):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    apply_read_only(estimator, 8, frame_for_bend(model, 1., np.deg2rad(179.)), model)
    frame = frame_for_bend(model, 1. + 1. / 30., np.deg2rad(181.))
    binding, profile = model[1:3]
    saved = profile["functional_fingers"]["little"]["binding"]
    if change == "reset":
        estimator.reset()
    elif change == "reference":
        profile["functional_fingers"]["little"]["slot_refs"]["8"]["axis_local"] = [1., 0., 0.]
    elif change == "installation":
        binding["upstream_state_sha256"] = saved["upstream_state_sha256"] = "new"
    elif change == "epoch":
        frame["nodes"]["10"]["epoch"] = binding["nodes_epochs"]["10"] = saved["nodes_epochs"]["10"] = 2
    elif change == "session":
        frame["session_id"] = profile["session_id"] = binding["session_id"] = saved["session_id"] = "new"
    elif change == "gap":
        frame["timestamp_s"] = 1.3
    else:
        frame["timestamp_s"] = .99
    angle, detail = apply_read_only(estimator, 8, frame, model)
    assert angle is not None and detail["quality"] == "degraded"
    assert detail["continuity"]["reset_reason"] is not None
    assert not detail["continuity"]["branch_corrected"]
    assert detail["heading_window"]["window_samples"] == 1
    assert detail["delta_rad"] == 0.


@pytest.mark.parametrize("mode", ["vertical", "protected", "ordinary"])
def test_thirty_seconds_without_heading_evidence_keeps_current_projection_not_last_angle(mode):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    for index in range(301):
        bend = .5 + .3 * np.sin(index / 10)
        frame = frame_for_bend(model, 1.+index/10, bend, vertical=mode == "vertical")
        if mode == "protected":
            frame["nodes"]["10"]["input_filter"] = {"gyro_rejected": True}
        angle, d = apply_read_only(estimator, 8, frame, model)
        assert angle == pytest.approx(bend, abs=1e-7)
        assert d["quality"] == "degraded" and d["display_available"]
        assert not d["heading_updated"]


def test_protection_blocks_qmt_update_but_not_current_fast_flexion(monkeypatch):
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    module = importlib.import_module("qmt.functions.heading_correction")
    def forbidden(*args, **kwargs):
        pytest.fail("protected samples cannot update relative heading")
    monkeypatch.setattr(module, "estimateDelta1d", forbidden)
    for index, bend in enumerate((1.2, .1, 1.4), 1):
        frame = frame_for_bend(model, 1.+index/30, bend, root=ROOTS[index])
        frame["nodes"]["10"]["input_filter"] = {"gyro_rejected": True}
        angle, d = apply_read_only(estimator, 8, frame, model)
        assert angle == pytest.approx(bend, abs=1e-7)
        assert not d["heading_updated"]


def test_duplicate_pairs_never_fill_history_or_become_heading_evidence():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    frame = frame_for_bend(model, 1., .4)
    apply_read_only(estimator, 8, frame, model)
    for stamp in np.arange(1.1, 5., .1):
        frame["timestamp_s"] = float(stamp)
        angle, d = apply_read_only(estimator, 8, frame, model)
        assert not d["fresh_pair"] and not d["heading_updated"] and not d["valid"]
        assert angle == pytest.approx(.4, abs=1e-7)
        assert len(estimator._states[8].observer.history) == 1


def test_current_residual_is_degraded_not_a_switch_to_another_angle_definition():
    model, estimator = fixture_model(), FunctionalHingeEstimator()
    actual = []
    for index, tilt in enumerate((14.5, 15., 15.5, 14.9, 35., 14.8)):
        angle, d = apply_read_only(estimator, 8,
            frame_for_bend(model, 1.+index/30, .8, tilt=tilt), model)
        actual.append(angle)
        assert d["delta_rad"] == 0. and d["quality"] == "degraded"
        assert d["residual_deg"] == pytest.approx(tilt, abs=1e-7)
    np.testing.assert_allclose(actual, .8, atol=1e-7)


@pytest.mark.parametrize("options", [{"unknown": 1}, {"rating_min": 2}, {"max_age_s": 0},
    {"heading_window_s": float("inf")}, {"smoothing_tau_s": True},
    {"heading_min_window_s": 6}, {"heading_estimate_rate_hz": 31}])
def test_invalid_options_are_rejected(options):
    with pytest.raises(ValueError):
        FunctionalHingeEstimator(options)
