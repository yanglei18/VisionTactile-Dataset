"""Sparse adapter algebra and opt-in parity with the real official calibrator.

Set FSGLOVE12_UPSTREAM_CHECKOUT and FSGLOVE12_MANO_ASSETS to run the external
source/model tests. Synthetic motions test implementation, not glove accuracy.
"""
from copy import deepcopy
import json
import os
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12.config import FINGER_DIPS, PARENTS, digest, load_config
from vt_fsglove12.upstream_backend import (
    ALGORITHM, COUPLED_POLICY, LEGACY_ALGORITHM, OFFICIAL_STEPS, ZERO_POLICY,
    OfficialCalibrationError, OfficialFSGloveBackend, calibration_motion_diagnostics,
    complete_missing_quaternions,
)


def synthetic_state():
    rng = np.random.default_rng(42)
    return SimpleNamespace(
        _base_quat=Rotation.random(16, random_state=rng).as_quat(),
        _delta_rot_fa_rotvec=Rotation.random(16, random_state=rng).as_rotvec(),
        _global_installation_error=Rotation.random(16, random_state=rng).as_rotvec(),
        _local_installation_error=Rotation.random(16, random_state=rng).as_rotvec(),
        _joint_fa=np.array([0] + list(PARENTS[1:])),
        _fk_seq=[1, 4, 7, 10, 13, 2, 5, 8, 11, 14, 3, 6, 9, 12, 15],
    )


def test_full16_completion_is_exact_identity_without_quaternion_reencoding():
    raw = Rotation.random(16, random_state=np.random.default_rng(2)).as_quat()
    raw[[2, 5]] *= -1  # Even the input hemisphere is preserved byte for byte.
    actual = complete_missing_quaternions(synthetic_state(), raw, np.ones(16, dtype=bool))
    np.testing.assert_array_equal(actual, raw)


def test_missing_completion_is_inverse_of_official_formula_not_copied_observation():
    state = synthetic_state()
    raw = Rotation.random(16, random_state=np.random.default_rng(3)).as_quat()
    observed = np.ones(16, dtype=bool)
    observed[[1, 2, 3, 6, 9, 12, 13]] = False
    complete = complete_missing_quaternions(state, raw, observed)
    np.testing.assert_array_equal(complete[observed], raw[observed])
    global_error = Rotation.from_rotvec(state._global_installation_error)
    delta = Rotation.from_quat(state._base_quat).inv() * Rotation.from_quat(complete)
    local_error = Rotation.from_rotvec(state._local_installation_error)
    fk = global_error * delta * local_error.inv()
    rest = Rotation.from_rotvec(state._delta_rot_fa_rotvec)
    for slot in np.flatnonzero(~observed):
        expected = fk[state._joint_fa[slot]] * rest[slot]
        assert (expected.inv() * fk[slot]).magnitude() < 1e-12
    assert not np.allclose(complete[1], complete[0])
    assert not np.allclose(complete[13], complete[14])


@pytest.mark.parametrize("mask", [[True] * 15, [1] * 16, [False] + [True] * 15])
def test_missing_completion_requires_boolean_mask_and_measured_root(mask):
    with pytest.raises(ValueError):
        complete_missing_quaternions(synthetic_state(), np.tile([0., 0., 0., 1.], (16, 1)), mask)


@pytest.fixture(scope="module")
def official_core():
    checkout = os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT")
    assets = os.environ.get("FSGLOVE12_MANO_ASSETS")
    if not checkout or not assets:
        pytest.skip("explicit official checkout and licensed MANO model opt-in required")
    from vt_fsglove12.upstream_core import load_upstream_core
    core = load_upstream_core(checkout, assets)
    core.torch.set_num_threads(1)
    return core


def make_data(core):
    config = load_config()
    rest = {"parents": list(PARENTS), "rest_global_xyzw": np.tile([0., 0., 0., 1.], (16, 1)).tolist(),
            "provenance": {"kind": "synthetic_test_rest_only"}}
    backend = OfficialFSGloveBackend(config, rest, checkout="unused", mano_assets_root="unused", core=core)
    base = Rotation.random(16, random_state=np.random.default_rng(9))
    poses = [base, Rotation.from_rotvec([.6, 0., 0.]) * base, Rotation.from_rotvec([-.6, 0., 0.]) * base]
    captures = {}
    for pose_index, (step, pose) in enumerate(zip(OFFICIAL_STEPS, poses)):
        captures[step["key"]] = [
            {"timestamp_s": 100. + pose_index * 3 + i * .1, "session_id": "test-session", "nodes": {
                node: {"quat_xyzw": pose[slot].as_quat().tolist(), "age_s": .01,
                       "epoch": int(node) % 3, "sample_index": 1 + pose_index * 20 + i}
                for node, slot in config["node_to_slot"].items()}}
            for i in range(5)]
    return backend, captures


def fit_data(core):
    backend, captures = make_data(core)
    profile = backend.fit_calibration(captures, session_id="test-session")
    frame = deepcopy(captures["base_pose"][-1])
    return backend, profile, frame


def test_real_core_full16_inference_equals_direct_upstream(official_core):
    base = Rotation.random(16, random_state=np.random.default_rng(8))
    calib = official_core.create_calibrator(base.as_quat(),
        (Rotation.from_rotvec([.6, 0., 0.]) * base).as_quat(),
        (Rotation.from_rotvec([-.6, 0., 0.]) * base).as_quat())
    raw = Rotation.random(16, random_state=np.random.default_rng(7)).as_quat()
    expected = calib.get_mano_param(Rotation.from_quat(raw)).numpy()
    completed = complete_missing_quaternions(calib, raw, np.ones(16, dtype=bool))
    actual = calib.get_mano_param(Rotation.from_quat(completed)).numpy()
    np.testing.assert_array_equal(completed, raw)
    np.testing.assert_array_equal(actual, expected)


def test_real_sparse_zero_prior_and_honest_masks(official_core):
    backend, profile, frame = fit_data(official_core)
    # This is specifically the historical v1 zero-prior contract. New v2
    # profiles intentionally couple currently observable PIP flexion to DIP.
    profile["method"], profile["missing_slot_policy"] = LEGACY_ALGORITHM, ZERO_POLICY
    profile.pop("missing_slot_options")
    # Apply a synthetic rigid movement, then remove one observed parent.
    delta = Rotation.from_rotvec([.2, -.1, .3])
    for data in frame["nodes"].values():
        data["quat_xyzw"] = (delta * Rotation.from_quat(data["quat_xyzw"])).as_quat().tolist()
    del frame["nodes"]["1"]  # Slot 4 absent; slot 5 is observed but its joint is inferred.
    pose = backend.solve(frame, profile)
    assert pose["pose_available"] and pose["solver_success"]
    missing = pose["virtual_input_slots"]
    assert missing == [3, 4, 6, 9, 12, 13]
    np.testing.assert_allclose(np.asarray(pose["anatomical_euler_rad"])[missing], 0., atol=1e-12)
    assert pose["observed_mask"][5] and not pose["joint_observed_mask"][5]
    assert pose["inferred_mask"][5] and pose["inferred_mask"][13]
    assert pose["source_labels"][4] == "explicit_fixed_zero_anatomical_prior"
    assert pose["source_labels"][5] == "official_measured_segment_with_inferred_parent"
    assert pose["input_contract"] == "explicit_sparse_to16_adapter"
    assert not pose["anatomical_accuracy_verified"] and not pose["yaw_drift_corrected"]
    assert "residual_deg_by_slot" not in pose
    assert profile["measured_calibration_slots"] == sorted(backend.config["node_to_slot"].values())
    assert profile["virtual_calibration_slots"] == [3, 6, 9, 12, 13]


def test_real_calibration_neutral_pose_and_official_result_path(official_core):
    backend, profile, frame = fit_data(official_core)
    pose = backend.solve(frame, profile)
    assert pose["algorithm"] == ALGORITHM
    assert pose["upstream_fsglove_solver_executed"] is True
    assert pose["upstream_commit"] == official_core.provenance["upstream_commit"]
    np.testing.assert_allclose(pose["anatomical_euler_rad"], 0., atol=1e-12)
    assert np.asarray(pose["mano_pose_rotvec"]).shape == (16, 3)
    assert pose["shape_params"] == [0.] * 10 and not pose["shape_calibrated"]
    assert profile["upstream_provenance"]["numerical_ast_modified"] is False


def test_configured_official_shape_parameters_are_not_silently_zeroed(official_core):
    old, captures = make_data(official_core)
    config = deepcopy(old.config)
    config.update(upstream_shape_params=[.01 * i for i in range(10)],
                  upstream_shape_calibrated=True,
                  upstream_shape_provenance={"algorithm": "official_pinch_shape_optimizer"})
    backend = OfficialFSGloveBackend(config, old.rest, checkout="unused", mano_assets_root="unused", core=official_core)
    profile = backend.fit_calibration(captures, session_id="test-session")
    pose = backend.solve(captures["base_pose"][-1], json.loads(json.dumps(profile)))
    assert profile["shape_params"] == pose["shape_params"] == config["upstream_shape_params"]
    assert profile["shape_calibrated"] is True and pose["shape_calibrated"] is True
    assert profile["shape_provenance"] == config["upstream_shape_provenance"]
    assert any("已载入" in warning for warning in profile["warnings"])


@pytest.mark.parametrize("shape", [[0.] * 9, [0.] * 11, [float("nan")] * 10])
def test_configured_shape_must_have_ten_finite_parameters(shape):
    config = load_config()
    config["upstream_shape_params"] = shape
    rest = {"parents": list(PARENTS), "rest_global_xyzw": np.tile([0., 0., 0., 1.], (16, 1)).tolist()}
    with pytest.raises(ValueError):
        OfficialFSGloveBackend(config, rest, checkout="unused", mano_assets_root="unused", core=object())


def test_saved_matrices_override_reinitialization_and_shape_survives_json(official_core, monkeypatch):
    backend, profile, frame = fit_data(official_core)
    frame["nodes"]["6"]["quat_xyzw"] = (Rotation.from_rotvec([.1, -.2, .3]) *
        Rotation.from_quat(frame["nodes"]["6"]["quat_xyzw"])).as_quat().tolist()
    profile["shape_params"] = [.01 * i for i in range(10)]
    expected = backend.solve(frame, profile)
    saved = json.loads(json.dumps(profile, allow_nan=False))
    create = official_core.create_calibrator

    def changed_initialization(*args, **kwargs):
        obj = create(*args, **kwargs)
        obj._delta_rot_fa_rotvec[:] += .1
        obj._global_installation_error[:] -= .2
        obj._local_installation_error[:] += .3
        return obj

    monkeypatch.setattr(official_core, "create_calibrator", changed_initialization)
    restored, _ = make_data(official_core)
    actual = restored.solve(frame, saved)
    np.testing.assert_array_equal(actual["mano_pose_rotvec"], expected["mano_pose_rotvec"])
    assert actual["shape_params"] == saved["shape_params"]
    assert saved["upstream_state_sha256"] == digest(saved["upstream_state"])


@pytest.mark.parametrize("change,reason", [("session", "calibration_session_mismatch"),
                                          ("root_epoch", "calibration_epoch_changed"),
                                          ("root_stale", "missing_or_stale_root")])
def test_real_runtime_binding_rejects_stale_or_restarted_root(official_core, change, reason):
    backend, profile, frame = fit_data(official_core)
    if change == "session":
        frame["session_id"] = "another-session"
    elif change == "root_epoch":
        frame["nodes"]["12"]["epoch"] += 1
    else:
        frame["nodes"]["12"]["age_s"] = 1.
    pose = backend.solve(frame, profile)
    assert not pose["pose_available"] and pose["reason"] == reason
    assert pose["mano_pose_rotvec"] is None


def test_real_finger_epoch_change_is_not_a_whole_hand_reset(official_core):
    backend, profile, frame = fit_data(official_core)
    frame["nodes"]["6"]["epoch"] += 1
    pose = backend.solve(frame, profile)
    assert pose["pose_available"]
    assert not pose["observed_mask"][15] and pose["inferred_mask"][15]
    assert pose["calibration_rejections"]["6"] == "filter_epoch_changed_recalibration_required"


@pytest.mark.parametrize("change", ["method", "config", "state", "source"])
def test_real_no_legacy_fallback_or_silent_profile_tampering(official_core, change):
    backend, profile, frame = fit_data(official_core)
    if change == "method":
        profile["method"] = "guided_rigid_pose_hand_eye_neutral_anchor_v2"
    elif change == "config":
        profile["config_sha256"] = "0" * 64
    elif change == "state":
        profile["upstream_state"]["_global_installation_error"][2][0] += .1
    else:
        profile["upstream_provenance"]["upstream_commit"] = "unknown"
    with pytest.raises(ValueError):
        backend.solve(frame, profile)


def test_real_calibration_bad_finger_is_explicitly_virtual_and_bad_root_fails(official_core):
    backend, captures = make_data(official_core)
    for frame in captures["z_rot_pose_2"]:
        frame["nodes"]["6"]["epoch"] += 1
    profile = backend.fit_calibration(captures, session_id="test-session")
    assert "6" in profile["rejected_nodes"] and "6" not in profile["nodes"]
    assert 15 in profile["virtual_calibration_slots"]
    for frame in captures["z_rot_pose_2"]:
        frame["nodes"]["12"]["epoch"] += 1
    with pytest.raises(OfficialCalibrationError, match="手掌标定失败"):
        backend.fit_calibration(captures, session_id="test-session")


def frame_for_anatomical_angles(backend, profile, template, angles):
    """Synthetic test oracle, not an estimate of recorded physical motion."""
    calibrator = backend._calibrator(profile)
    local = Rotation.from_euler("XYZ", angles)
    rest = Rotation.from_rotvec(calibrator._delta_rot_fa_rotvec)
    fk = [None] * 16
    fk[0] = local[0]
    for slot in calibrator._fk_seq:
        fk[slot] = fk[calibrator._joint_fa[slot]] * rest[slot] * local[slot]
    fk = Rotation.from_quat([rotation.as_quat() for rotation in fk])
    base = Rotation.from_quat(calibrator._base_quat)
    installation = Rotation.from_rotvec(calibrator._global_installation_error)
    local_installation = Rotation.from_rotvec(calibrator._local_installation_error)
    raw = (base * installation.inv() * fk * local_installation).as_quat()
    frame = deepcopy(template)
    for node, entry in profile["nodes"].items():
        frame["nodes"][node]["quat_xyzw"] = raw[entry["slot"]].tolist()
    return frame


def test_v2_real_nonzero_pip_couples_dip_without_claiming_measurement(official_core):
    backend, profile, template = fit_data(official_core)
    angles = np.zeros((16, 3))
    angles[0] = [.2, -.1, .3]
    angles[[2, 5, 8, 11], 2] = [.5, .7, 1., 1.4]
    frame = frame_for_anatomical_angles(backend, profile, template, angles)
    pose = backend.solve(frame, profile)
    output = np.asarray(pose["anatomical_euler_rad"])
    assert profile["method"] == pose["algorithm"] == ALGORITHM
    assert profile["missing_slot_policy"] == pose["missing_slot_policy"] == COUPLED_POLICY
    assert profile["missing_slot_options"] == {
        "dip_ratio": .65, "dip_ratio_by_finger": {}, "upstream_dip_max_rad": 1.65}
    for finger, slot in FINGER_DIPS.items():
        np.testing.assert_allclose(output[slot], [0., 0., .65 * angles[slot - 1, 2]], atol=1e-12)
        np.testing.assert_allclose(output[slot - 1], angles[slot - 1], atol=1e-12)
        assert not pose["observed_mask"][slot] and not pose["segment_observed_mask"][slot]
        assert not pose["joint_observed_mask"][slot] and pose["inferred_mask"][slot]
        assert pose["source_labels"][slot] == "pip_coupled_dip_prior_not_observation"
        entry = pose["sparse_prior"]["dip_by_finger"][finger]
        assert entry["applied"] and entry["source"] == "inferred_not_measured"
    assert sum(pose["segment_observed_mask"]) == 11
    np.testing.assert_allclose(output[13], 0., atol=1e-12)
    assert pose["source_labels"][13] == "explicit_fixed_zero_anatomical_prior"
    assert pose["upstream_fsglove_solver_executed"] is True
    assert not pose["sparse_prior"]["inference_is_measurement"]
    assert not pose["anatomical_accuracy_verified"] and not pose["yaw_drift_corrected"]


def test_v2_configured_ratio_and_limit_reach_actual_official_output(official_core):
    original, captures = make_data(official_core)
    config = {**original.config, "dip_ratio": .4, "dip_ratio_by_finger": {"index": 1.},
              "upstream_dip_max_rad": 1.1}
    backend = OfficialFSGloveBackend(config, original.rest, checkout="unused", mano_assets_root="unused", core=official_core)
    profile = backend.fit_calibration(captures, session_id="test-session")
    angles = np.zeros((16, 3))
    angles[[2, 5, 8, 11], 2] = [2.2, 1., .5, -.2]
    frame = frame_for_anatomical_angles(backend, profile, captures["base_pose"][-1], angles)
    pose = backend.solve(frame, json.loads(json.dumps(profile)))
    np.testing.assert_allclose(np.asarray(pose["anatomical_euler_rad"])[[3, 6, 9, 12], 2],
                               [1.1, .4, .2, 0.], atol=1e-12)
    assert pose["sparse_prior"]["dip_by_finger"]["index"]["clip_applied"]
    assert pose["sparse_prior"]["dip_by_finger"]["ring"]["clip_applied"]


def test_v1_json_replay_retains_zero_dip_under_v2_default_backend(official_core):
    backend, new_profile, template = fit_data(official_core)
    angles = np.zeros((16, 3))
    angles[[2, 5, 8, 11], 2] = [.8, 1., .5, .7]
    frame = frame_for_anatomical_angles(backend, new_profile, template, angles)
    old_profile = deepcopy(new_profile)
    old_profile.update(method=LEGACY_ALGORITHM, missing_slot_policy=ZERO_POLICY)
    old_profile.pop("missing_slot_options")
    old_profile.pop("calibration_motion_diagnostics")
    old_profile = json.loads(json.dumps(old_profile))
    original_profile = deepcopy(old_profile)
    restored, _ = make_data(official_core)
    legacy = restored.solve(frame, old_profile)
    np.testing.assert_allclose(np.asarray(legacy["anatomical_euler_rad"])[[3, 6, 9, 12, 13]], 0., atol=1e-12)
    assert legacy["algorithm"] == LEGACY_ALGORITHM
    assert legacy["missing_slot_policy"] == ZERO_POLICY
    assert legacy["sparse_prior"]["coupled_dip_slots"] == []
    assert all(legacy["source_labels"][slot] == "explicit_fixed_zero_anatomical_prior"
               for slot in FINGER_DIPS.values())
    new_pose = restored.solve(frame, new_profile)
    np.testing.assert_allclose(np.asarray(new_pose["anatomical_euler_rad"])[[3, 6, 9, 12], 2],
                               .65 * angles[[2, 5, 8, 11], 2], atol=1e-12)
    assert old_profile == original_profile
    old_profile["missing_slot_policy"] = COUPLED_POLICY
    with pytest.raises(ValueError, match="v1 profiles must retain"):
        restored.solve(frame, old_profile)


@pytest.mark.parametrize("change", ["policy", "ratio", "per_finger", "maximum", "missing_options"])
def test_v2_profile_policy_and_options_mismatch_rejected_before_cached_calibrator(official_core, change):
    backend, profile, frame = fit_data(official_core)
    assert backend.solve(frame, profile)["pose_available"]
    if change == "policy":
        profile["missing_slot_policy"] = ZERO_POLICY
    elif change == "ratio":
        profile["missing_slot_options"]["dip_ratio"] = .2
    elif change == "per_finger":
        profile["missing_slot_options"]["dip_ratio_by_finger"] = {"little": .2}
    elif change == "maximum":
        profile["missing_slot_options"]["upstream_dip_max_rad"] = 1.
    else:
        profile.pop("missing_slot_options")
    with pytest.raises(ValueError, match="saved sparse prior policy does not match"):
        backend.solve(frame, profile)


@pytest.mark.parametrize("maximum", [-.1, 0., np.pi, 4., True, "1.0", float("nan"), float("inf")])
def test_backend_rejects_invalid_dip_max_without_loading_models(maximum):
    old, _ = make_data(object())
    config = {**old.config, "upstream_dip_max_rad": maximum}
    with pytest.raises(ValueError):
        OfficialFSGloveBackend(config, old.rest, checkout="unused", mano_assets_root="unused", core=object())


def motion_inputs(up_deg, down_deg):
    base = Rotation.random(16, random_state=np.random.default_rng(831))
    return dict(base_pose=base.as_quat().tolist(),
                z_rot_pose_1=(Rotation.from_rotvec(np.deg2rad(up_deg)) * base).as_quat().tolist(),
                z_rot_pose_2=(Rotation.from_rotvec(np.deg2rad(down_deg)) * base).as_quat().tolist())


def test_motion_diagnostics_ideal_opposite_50_degree_poses_are_nonblocking():
    inputs = motion_inputs([50., 0., 0.], [-50., 0., 0.])
    diagnostic = calibration_motion_diagnostics(inputs, [0, 1, 2, 4], "12")
    root = diagnostic["root"]
    np.testing.assert_allclose([root["up_from_base_deg"], root["down_from_base_deg"], root["up_to_down_deg"]],
                               [50., 50., 100.], atol=1e-12)
    assert root["opposite_axis_error_deg"] < 1e-5
    assert root["rotation_axis_gravity_tilt_deg"] < 1e-12
    assert diagnostic["warnings"] == [] and diagnostic["blocking"] is False
    assert diagnostic["kind"] == "three_pose_motion_consistency_not_accuracy"
    assert diagnostic["thresholds_are_capture_guidance_not_anatomical_limits"] is True
    assert set(diagnostic["by_slot"]) == {"0", "1", "2", "4"}


@pytest.mark.parametrize("up,down,warning", [
    ([50., 0., 0.], [0., -50., 0.], "不是反向同轴"),
    ([40., 0., 0.], [-40., 0., 0.], "小于官方建议的 90"),
    ([5., 0., 0.], [-50., 0., 0.], "不足 15"),
])
def test_motion_diagnostics_warn_without_turning_guidance_into_failure(up, down, warning):
    diagnostic = calibration_motion_diagnostics(motion_inputs(up, down), [0, 1, 2], "12")
    assert diagnostic["blocking"] is False
    assert any(warning in text for text in diagnostic["warnings"])
    if warning == "不是反向同轴":
        assert abs(diagnostic["root"]["opposite_axis_error_deg"] - 90.) < 1e-10
    elif warning == "小于官方建议的 90":
        assert abs(diagnostic["root"]["up_to_down_deg"] - 80.) < 1e-10


@pytest.mark.parametrize("up,down,expect_warning", [
    ([50., 0., 0.], [-50., 0., 0.], False),
    ([40., 0., 0.], [-40., 0., 0.], True),
    ([50., 0., 0.], [0., -50., 0.], True),
])
def test_real_motion_diagnostics_persist_and_never_block_valid_numeric_calibration(
        official_core, up, down, expect_warning):
    backend, captures = make_data(official_core)
    rotations = motion_inputs(up, down)
    for key, rows in captures.items():
        for frame in rows:
            for node, slot in backend.config["node_to_slot"].items():
                frame["nodes"][node]["quat_xyzw"] = rotations[key][slot]
    profile = backend.fit_calibration(captures, session_id="test-session")
    diagnostics = profile["calibration_motion_diagnostics"]
    assert diagnostics["blocking"] is False
    assert bool(diagnostics["warnings"]) == expect_warning
    assert all(text in profile["warnings"] for text in diagnostics["warnings"])
    assert profile["status"] == "official_core_sparse_adapter_not_accuracy_validated"
    assert backend.solve(captures["base_pose"][-1], json.loads(json.dumps(profile)))["pose_available"]


def test_legacy_heading_options_do_not_create_a_second_input_correction(official_core, monkeypatch):
    from vt_fsglove12.heading_correction import CausalHeadingCorrector
    def forbidden(*args, **kwargs):
        pytest.fail("live backend must not run a second pre-inference heading correction")
    monkeypatch.setattr(CausalHeadingCorrector, "apply", forbidden)
    original, captures = make_data(official_core)
    profile = original.fit_calibration(captures, session_id="test-session")
    sample = captures["base_pose"][-1]
    saved = deepcopy(sample)
    expected = original.solve(sample, profile)
    for enabled in (False, True):
        config = {**original.config, "relative_heading_correction": {"enabled": enabled}}
        backend = OfficialFSGloveBackend(config, original.rest, checkout="unused", mano_assets_root="unused", core=official_core)
        fitted = backend.fit_calibration(captures, session_id="test-session")
        actual = backend.solve(sample, fitted)
        np.testing.assert_array_equal(actual["mano_pose_rotvec"], expected["mano_pose_rotvec"])
        assert actual["relative_heading_correction"]["reason"] == "superseded_by_unified_continuous_functional_hinges"
        with pytest.raises(ValueError, match="configuration or MANO model digest"):
            backend.solve(sample, profile)
    assert sample == saved


def test_uncorrected_calibration_capture_never_clears_live_hinge_history(official_core):
    from test_upstream_functional_integration import trial, _excited_calibrated_heading_window
    backend, profile, frame, neutral, motion = trial(official_core, "little")
    # Exercise a genuinely qualified window, not the removed first-frame
    # heading bootstrap. Historical calibration fitting must retain both its
    # accepted evidence and the still-live source/continuity history.
    pose = _excited_calibrated_heading_window(backend, profile, frame, 8)
    assert pose["joint_valid_mask"][8]
    state = backend._functional_pipeline.hinges._states[8]
    saved = deepcopy(vars(state))
    observer = state.observer
    saved_observer = deepcopy(vars(observer))
    assert len(observer.history) >= 20 and observer.evidence_time is not None
    before_profile = deepcopy(profile)
    backend._functional_capture_results([neutral, motion], profile, session_id=profile["session_id"])
    assert backend._functional_pipeline.hinges._states[8] is state
    for name in ('delta', 'source_time', 'angle', 'angle_time', 'identity', 'frame_time'):
        assert getattr(state, name) == saved[name]
    assert state.observer is observer
    assert len(observer.history) == len(saved_observer['history'])
    for old, current in zip(saved_observer['history'], observer.history):
        np.testing.assert_equal(old, current)
    for name in ('last_sample', 'last_fit', 'last_source', 'target', 'evidence_time', 'diagnostic'):
        assert getattr(observer, name) == saved_observer[name]
    assert profile == before_profile
    current = backend.solve(frame(.7, root=(1.1, -.8, .3), sequence=400), profile)
    assert current['joint_display_available_mask'][8] and current['joint_valid_mask'][8]
    assert current['anatomical_euler_rad'][8][2] == pytest.approx(.7, abs=2e-7)
    assert current['joint_estimation']['8']['continuity']['reset_reason'] is None
    # Explicit user disabling still resets; fitting another finger does not.
    backend.solve(frame(.6, sequence=401), {**profile, "functional_enabled": False})
    assert backend._functional_pipeline.hinges._states == {}
