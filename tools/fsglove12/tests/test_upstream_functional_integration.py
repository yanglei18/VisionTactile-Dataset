"""Actual upstream + QMT + MANO integration, not a real-hand accuracy test."""
from copy import deepcopy
import json
import os

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_upstream_backend import official_core, fit_data, frame_for_anatomical_angles
from vt_fsglove12.upstream_backend import ALGORITHM, FUNCTIONAL_ALGORITHM


pytestmark = pytest.mark.skipif(
    not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
    reason="requires explicitly configured external official source and licensed model",
)


def trial(core, finger="index"):
    pytest.importorskip("qmt")
    backend, profile, template = fit_data(core)
    slots = {"index": (1, 2), "middle": (4, 5), "ring": (10, 11),
             "little": (7, 8), "thumb": (15,)}[finger]
    # An explicit residual frame error after the upstream installation stage.
    # This fixture does not assume a single motion identifies arbitrary 3D
    # sensor mounts or test independent yaw drift as if it were rigid mounting.
    bases = {slot: Rotation.from_rotvec([.6, -.25, 0.]) for slot in slots}
    offsets = {slot: Rotation.from_euler("XYZ", [.12, -.09, .04]) for slot in slots}

    def frame(bend=0., root=(0., 0., 0.), spread=0., sequence=100):
        angles = np.zeros((16, 3))
        angles[0] = root
        for slot in slots:
            target = Rotation.from_euler("XYZ", [0., spread if slot in (1, 4, 7, 10) else 0., bend])
            angles[slot] = (offsets[slot] * bases[slot] * target * bases[slot].inv()).as_euler("XYZ")
        row = frame_for_anatomical_angles(backend, profile, template, angles)
        row["timestamp_s"] = 200. + sequence / 30.
        for data in row["nodes"].values():
            data["sample_index"] = sequence
        return row

    neutral = [frame(sequence=100 + i) for i in range(6)]
    # Three declared flexion cycles; no absolute 90-degree label is supplied.
    motion = [frame(float(angle), sequence=110 + i)
              for i, angle in enumerate(np.tile(np.r_[np.linspace(.05, 1.2, 12), np.linspace(1.2, .05, 12)], 3))]
    entry = backend.fit_functional_finger(neutral, motion, profile, finger=finger, session_id=profile["session_id"])
    profile = {**profile, "functional_fingers": {finger: entry}, "functional_enabled": True}
    return backend, profile, frame, neutral, motion


@pytest.mark.parametrize("finger,slots", [("index", (1, 2)), ("middle", (4, 5)),
                                        ("ring", (10, 11)), ("little", (7, 8))])
def test_real_pipeline_axis_correction_dip_root_and_unmodified_upstream_state(official_core, finger, slots):
    backend, profile, frame, _, _ = trial(official_core, finger)
    original = deepcopy(profile)
    sample = frame(.8, root=(.2, -.3, .4), spread=.18)
    before_sample = deepcopy(sample)
    raw = backend.solve(sample, {**profile, "functional_enabled": False})
    # The bound neutral supplies the initial projection, not a single-frame
    # heading fit. Current geometry remains visible before window evidence.
    warm = backend.solve(frame(.8, root=(1.1, -.8, .3), sequence=99), profile)
    assert warm["joint_display_available_mask"][slots[1]]
    assert warm["joint_quality"][slots[1]] == "degraded"
    assert warm["joint_estimation"][str(slots[1])]["delta_rad"] == 0.
    corrected = backend.solve(sample, profile)
    assert corrected["algorithm"] == FUNCTIONAL_ALGORITHM
    assert corrected['pose_processing_revision'] == 'unified_functional_calibration_window_v3'
    assert corrected["upstream_fsglove_solver_executed"]
    angles = np.asarray(corrected["anatomical_euler_rad"])
    np.testing.assert_allclose(angles[slots[0]], [0., .18, .8], atol=2e-7)
    np.testing.assert_allclose(angles[slots[1]], [0., 0., .8], atol=2e-7)
    dip = slots[1] + 1
    np.testing.assert_allclose(angles[dip], [0., 0., .52], atol=2e-7)
    unchanged = [slot for slot in range(16) if slot not in (*slots, dip)]
    np.testing.assert_allclose(angles[unchanged], np.asarray(raw["anatomical_euler_rad"])[unchanged], atol=1e-12)
    np.testing.assert_array_equal(corrected["mano_pose_rotvec"][0], raw["mano_pose_rotvec"][0])
    assert corrected["root_quat_xyzw"] == raw["root_quat_xyzw"]
    assert corrected["upstream_raw_mano_pose_rotvec"] == raw["mano_pose_rotvec"]
    assert corrected["upstream_raw_anatomical_euler_rad"] == raw["anatomical_euler_rad"]
    assert corrected["segment_observed_mask"] == raw["segment_observed_mask"]
    for slot in slots:
        assert not corrected["joint_observed_mask"][slot] and corrected["inferred_mask"][slot]
        assert corrected["unconstrained_joint_observed_mask"][slot]
    assert not corrected["observed_mask"][dip]
    assert sample == before_sample and profile == original


def test_real_common_root_rotation_leaves_corrected_local_angles_and_return_neutral(official_core):
    backend, profile, frame, _, _ = trial(official_core)
    identity = backend.solve(frame(.9), profile)
    turned = backend.solve(frame(.9, root=(.7, -.2, .5)), profile)
    np.testing.assert_allclose(np.asarray(identity["anatomical_euler_rad"])[1:],
                               np.asarray(turned["anatomical_euler_rad"])[1:], atol=1e-12)
    neutral = backend.solve(frame(0.), profile)
    np.testing.assert_allclose(np.asarray(neutral["anatomical_euler_rad"])[[1, 2, 3]], 0., atol=1e-12)


def test_real_thumb_ip_only_does_not_invent_cmc_or_change_thumb_mcp(official_core):
    backend, profile, frame, _, _ = trial(official_core, "thumb")
    sample = frame(.7)
    raw = backend.solve(sample, {**profile, "functional_enabled": False})
    warm = backend.solve(frame(.7, root=(1.1, -.8, .3), sequence=99), profile)
    assert warm["joint_display_available_mask"][15]
    assert not warm["joint_valid_mask"][15]
    actual = backend.solve(sample, profile)
    assert actual["functional_adapter"]["applied_slots"] == [15]
    np.testing.assert_allclose(actual["anatomical_euler_rad"][15], [0., 0., .7], atol=1e-12)
    np.testing.assert_array_equal(np.asarray(actual["anatomical_euler_rad"])[:15], np.asarray(raw["anatomical_euler_rad"])[:15])


@pytest.mark.parametrize("finger,slots", [("index", (1, 2)), ("middle", (4, 5)),
                                        ("ring", (10, 11)), ("little", (7, 8)),
                                        ("thumb", (15,))])
def test_real_pipeline_small_initial_reverse_motion_cannot_invert_finger(official_core, finger, slots):
    backend, profile, frame, neutral, motion = trial(official_core, finger)
    early_reverse = frame(np.deg2rad(-9.), sequence=109)
    entry = backend.fit_functional_finger(neutral, [early_reverse] + motion,
        profile, finger=finger, session_id=profile["session_id"])
    updated = {**profile, "functional_fingers": {finger: entry}}
    # Use an observable pose, distinct live identities, and the actual official
    # solver/QMT/continuous-hinge/MANO composition, not stateless projection only.
    for i, bend in enumerate((0., .8, 0., -.1)):
        pose = backend.solve(frame(bend, root=(1.1, -.8, .3), sequence=400 + i), updated)
        assert all(pose["joint_display_available_mask"][slot] for slot in slots)
        hinge = slots[-1]
        assert pose["joint_quality"][hinge] == "degraded"
        assert pose["joint_estimation"][str(hinge)]["delta_rad"] == 0.
        np.testing.assert_allclose(np.asarray(pose["anatomical_euler_rad"])[list(slots), 2],
                                   bend, atol=2e-7)
        if finger != "thumb":
            assert pose["anatomical_euler_rad"][slots[1] + 1][2] == pytest.approx(
                max(0., bend) * .65, abs=2e-7)


def test_real_bad_dependency_does_not_apply_old_correction_or_stale_dip(official_core):
    backend, profile, frame, _, _ = trial(official_core)
    sample = frame(.8)
    sample["nodes"]["4"]["epoch"] += 1
    pose = backend.solve(sample, profile)
    assert 2 not in pose["functional_adapter"]["applied_slots"]
    assert 1 in pose["functional_adapter"]["applied_slots"]
    assert not pose["segment_observed_mask"][2]
    np.testing.assert_allclose(pose["anatomical_euler_rad"][3], 0., atol=1e-12)


def test_real_json_replay_disable_and_failure_preserve_previous_calibration(official_core):
    backend, profile, frame, neutral, motion = trial(official_core)
    saved = deepcopy(profile)
    sample = frame(.8)
    expected = backend.solve(sample, profile)
    restored = json.loads(json.dumps(profile, allow_nan=False))
    actual = backend.solve(sample, restored)
    np.testing.assert_array_equal(actual["mano_pose_rotvec"], expected["mano_pose_rotvec"])
    raw = backend.solve(sample, {**restored, "functional_enabled": False})
    assert raw["algorithm"] == ALGORITHM
    np.testing.assert_array_equal(raw["mano_pose_rotvec"], actual["upstream_raw_mano_pose_rotvec"])
    # Frozen repeated rows are not counted as new calibration evidence.
    with pytest.raises(ValueError):
        backend.fit_functional_finger(neutral[:1] * 8, motion[:1] * 16, profile,
                                       finger="index", session_id=profile["session_id"])
    assert profile == saved


def test_real_functional_mesh_uses_original_mano_composer(official_core):
    from vt_fsglove12.mano import ManoBackend
    backend, profile, frame, _, _ = trial(official_core)
    pose = backend.solve(frame(.75, root=(.3, -.1, .2)), profile)
    mano = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"])
    geometry = mano.render_mano(pose["mano_pose_rotvec"], pose["shape_params"])
    assert np.asarray(geometry["vertices_m"]).shape == (778, 3)
    assert np.isfinite(geometry["vertices_m"]).all()
    assert pose["pose_postprocessing"] == "qmt_functional_axis_and_joint_constraint"
    assert not pose["yaw_drift_corrected"] and not pose["physical_joint_axes_verified"]


def test_missing_qmt_marks_managed_joint_unavailable_without_raw_fallback(official_core, monkeypatch):
    from vt_fsglove12 import upstream_functional
    backend, profile, frame, neutral, motion = trial(official_core)
    saved = deepcopy(profile)
    sample = frame(.8)
    raw = backend.solve(sample, {**profile, "functional_enabled": False})

    def unavailable():
        raise upstream_functional.FunctionalDependencyError("qmt unavailable in this test")

    monkeypatch.setattr(upstream_functional, "_qmt", unavailable)
    result = backend.solve(sample, profile)
    assert result["algorithm"] == FUNCTIONAL_ALGORITHM
    assert result["upstream_fsglove_solver_executed"]
    assert result["functional_adapter"]["applied_slots"] == []
    assert result["functional_adapter"]["warnings"]
    assert not result["joint_valid_mask"][2] and not result["joint_display_available_mask"][2]
    assert not result["joint_valid_mask"][3]
    assert result["source_labels"][2] == "unavailable_geometry_placeholder_not_measurement"
    np.testing.assert_array_equal(result["upstream_raw_mano_pose_rotvec"], raw["mano_pose_rotvec"])
    with pytest.raises(ValueError, match="qmt unavailable"):
        backend.fit_functional_finger(neutral, motion, profile,
            finger="index", session_id=profile["session_id"])
    assert profile == saved


def _official_calibrated_projection(raw, profile, finger, slot, previous=None):
    """Independent MIT QMT projection of official local output at delta=0.

    This does not call the stateful estimator or its virtual-frame geometry.
    An injected yaw error must NOT disappear just because a first frame exists.
    """
    import qmt
    reference = profile['functional_fingers'][finger]['slot_refs'][str(slot)]
    neutral = Rotation.from_quat(reference['neutral_local_xyzw'])
    basis = Rotation.from_quat(reference['axis_basis_xyzw'])
    local = Rotation.from_euler('XYZ', raw['anatomical_euler_rad'][slot])
    retargeted = basis.inv() * neutral.inv() * local * basis
    projected = qmt.quatProject(retargeted.as_quat()[[3, 0, 1, 2]], [0., 0., 1.])
    angle = float(projected['projAngle'])
    if previous is not None:
        angle += round((previous - angle) / (2 * np.pi)) * (2 * np.pi)
    return angle


def test_little_unobserved_yaw_preserves_calibrated_projection_not_instant_repair(official_core):
    backend, profile, frame, _, _ = trial(official_core, "little")
    raw_backend = type(backend)(backend.config, backend.rest,
                               checkout="unused", mano_assets_root="unused", core=official_core)
    saved = deepcopy(profile)
    previous = None
    for i, root in enumerate(((0., 0., 0.), (.4, -.3, .2), (-.5, .2, -.3))):
        sample = frame(.6, root=root, sequence=400 + i)
        # Each fixture sensor has its own calibrated initial yaw. An extra
        # child yaw is a residual six-axis heading error, not physical bending.
        child = Rotation.from_quat(sample["nodes"]["10"]["quat_xyzw"])
        sample["nodes"]["10"]["quat_xyzw"] = (
            Rotation.from_euler("z", 170., degrees=True) * child).as_quat().tolist()
        original_sample = deepcopy(sample)
        raw = raw_backend.solve(sample, {**profile, "functional_enabled": False})
        corrected = backend.solve(sample, profile)
        detail = corrected["little_relative_heading"]
        assert detail["applied"] and detail['display_available'] and not detail['valid']
        assert not detail['heading_updated'] and detail['delta_rad'] == 0.
        expected = _official_calibrated_projection(raw, profile, 'little', 8, previous)
        previous = expected
        np.testing.assert_allclose(corrected["anatomical_euler_rad"][8], [0., 0., expected], atol=2e-7)
        np.testing.assert_allclose(corrected["anatomical_euler_rad"][9],
            [0., 0., min(1.65, .65 * max(0., expected))], atol=2e-7)
        np.testing.assert_array_equal(corrected["mano_pose_rotvec"][0], raw["mano_pose_rotvec"][0])
        np.testing.assert_array_equal(corrected["upstream_raw_anatomical_euler_rad"], raw["anatomical_euler_rad"])
        for slot in (1, 2, 3, 4, 5, 6, 10, 11, 12, 13, 14, 15):
            np.testing.assert_array_equal(corrected["anatomical_euler_rad"][slot], raw["anatomical_euler_rad"][slot])
        assert original_sample == sample and saved == profile


def test_positive_hinge_pi_crossing_no_longer_resets_coupled_dip(official_core):
    backend, profile, frame, _, _ = trial(official_core)
    for i, degrees in enumerate((170., 179.99, 180.01, 185.)):
        pose = backend.solve(frame(np.deg2rad(degrees), sequence=400 + i), profile)
        assert pose["anatomical_euler_rad"][2][2] == pytest.approx(np.deg2rad(degrees), abs=1e-12)
        assert pose["anatomical_euler_rad"][3][2] == pytest.approx(1.65)
        assert pose["sparse_prior"]["dip_by_finger"]["index"]["target_flexion_rad"] == pytest.approx(1.65)
        assert pose["functional_adapter"]["angle_continuity_by_slot"]["2"]["branch_corrected"] == (degrees > 180.)


def test_little_short_input_protection_preserves_current_calibrated_projection(official_core):
    backend, profile, frame, _, _ = trial(official_core, "little")
    raw_backend = type(backend)(backend.config, backend.rest,
        checkout="unused", mano_assets_root="unused", core=official_core)
    previous = None
    for i, bend in enumerate((.4, .7, .2, .6)):
        sample = frame(bend, sequence=400 + i)
        child = sample["nodes"]["10"]
        child["quat_xyzw"] = (Rotation.from_euler("z", 170., degrees=True)
                              * Rotation.from_quat(child["quat_xyzw"])).as_quat().tolist()
        child["orientation_valid"] = True
        if i in (1, 2):
            child["input_filter"] = {"gyro_rejected": True}
        pose = backend.solve(sample, profile)
        raw = raw_backend.solve(sample, {**profile, 'functional_enabled': False})
        expected = _official_calibrated_projection(raw, profile, 'little', 8, previous)
        previous = expected
        assert pose["little_relative_heading"]["applied"]
        if i in (1, 2):
            detail = pose['little_relative_heading']
            # A large injected yaw can also violate the residual threshold;
            # that geometric failure takes precedence over protection status.
            expected_status = ('high_residual_using_last_heading' if detail['residual_deg'] > 15.
                               else 'input_protection_using_calibrated_heading')
            assert detail['status'] == expected_status
            assert not detail['heading_updated'] and not detail['heading_window']
        assert pose['joint_quality'][8] == 'degraded'
        assert pose['joint_estimation']['8']['delta_rad'] == 0.
        assert pose["anatomical_euler_rad"][8][2] == pytest.approx(expected, abs=2e-7)
        assert pose["anatomical_euler_rad"][9][2] == pytest.approx(min(1.65, .65 * max(0., expected)), abs=2e-7)


def _with_virtual_hinge_error(backend, profile, sample, finger, slot, *, yaw_deg=70., tilt_deg=0.):
    """Perturb the actual calibrated virtual hinge, not a guessed IMU axis."""
    from vt_fsglove12.heading_correction import CausalHeadingCorrector
    row = deepcopy(sample)
    binding = backend._functional_binding(profile)
    parent_node, child_node = binding["dependency_nodes_by_slot"][str(slot)]
    reference = profile["functional_fingers"][finger]["slot_refs"][str(slot)]
    hp, hc, rp, rc, axis = CausalHeadingCorrector._geometry(
        backend._calibrator(profile), slot - 1, slot, reference)
    q1 = hp * Rotation.from_quat(row["nodes"][parent_node]["quat_xyzw"]) * rp
    q2 = hc * Rotation.from_quat(row["nodes"][child_node]["quat_xyzw"]) * rc
    if tilt_deg:
        tilt_axis = np.cross(q1.apply(axis), [0., 0., 1.])
        tilt_axis /= np.linalg.norm(tilt_axis)
        q2 = Rotation.from_rotvec(tilt_axis * np.deg2rad(tilt_deg)) * q2
    child = hc.inv() * q2 * rc.inv()
    row["nodes"][child_node]["quat_xyzw"] = (
        Rotation.from_euler("z", yaw_deg, degrees=True) * child).as_quat().tolist()
    return row


def _excited_calibrated_heading_window(backend, profile, frame, slot):
    # Use real source identities/times, multiple flexion cycles and parent
    # inclination changes. Neither a long static hold nor world yaw suffices.
    for sequence in range(300, 399):
        t = (sequence - 300) / 30.
        root = (1.1 + .3 * np.sin(2 * np.pi * t / 1.6),
                -.8 + .25 * np.cos(2 * np.pi * t / 2.), .3)
        backend.solve(frame(.6 + .3 * np.sin(2 * np.pi * t / .9),
                            root=root, sequence=sequence), profile)
    result = backend.solve(frame(.6, root=(1.1, -.8, .3), sequence=399), profile)
    assert result['joint_valid_mask'][slot], result['joint_estimation'][str(slot)]
    assert result['joint_estimation'][str(slot)]['heading_window']['evidence_age_s'] < 1.
    return result


@pytest.mark.parametrize('finger,slot', [('index', 2), ('little', 8), ('thumb', 15)])
def test_no_flexion_excitation_keeps_real_projection_visible_after_window_age(official_core, finger, slot):
    backend, profile, frame, _, _ = trial(official_core, finger)
    original = deepcopy(profile)
    # More than a complete four-second window, with palm tilt but fixed local
    # articulation: time passing is not heading evidence and must not freeze
    # or hide the current calibrated projection.
    for sequence in range(300, 451):
        t = (sequence - 300) / 30.
        root = (1.1 + .3 * np.sin(t), -.8 + .25 * np.cos(t), .3)
        result = backend.solve(frame(.6, root=root, sequence=sequence), profile)
        detail = result['joint_estimation'][str(slot)]
        assert detail['display_available'] and not detail['valid']
        assert not detail['heading_updated'] and detail['delta_rad'] == 0.
        assert detail['heading_reference_age_s'] is None
        assert result['joint_calibration_status'][slot] == 'degraded'
        coverage = result['functional_calibration_coverage']
        assert coverage['correction_applied_mask'][slot]
        assert not coverage['correction_valid_mask'][slot]
        np.testing.assert_allclose(result['anatomical_euler_rad'][slot], [0., 0., .6], atol=2e-7)
        if slot != 15:
            np.testing.assert_allclose(result['anatomical_euler_rad'][slot + 1], [0., 0., .39], atol=2e-7)
    assert profile == original


@pytest.mark.parametrize("finger,slot", [("index", 2), ("little", 8)])
def test_unified_real_pipeline_residual_threshold_is_quality_only_with_dip_inheritance(
        official_core, finger, slot):
    backend, profile, frame, _, _ = trial(official_core, finger)
    raw_backend = type(backend)(backend.config, backend.rest,
        checkout="unused", mano_assets_root="unused", core=official_core)
    saved = deepcopy(profile)
    root = (1.1, -.8, .3)
    initial = _excited_calibrated_heading_window(backend, profile, frame, slot)
    first_delta = initial["joint_estimation"][str(slot)]["delta_rad"]
    for i, tilt in enumerate((14.5, 15., 15.5, 14.8, 15.5, 14.9)):
        sample = _with_virtual_hinge_error(backend, profile,
            frame(.6, root=root, sequence=400 + i), finger, slot, yaw_deg=0., tilt_deg=tilt)
        before = deepcopy(sample)
        raw = raw_backend.solve(sample, {**profile, "functional_enabled": False})
        pose = backend.solve(sample, profile)
        detail = pose["joint_estimation"][str(slot)]
        quality = "valid" if tilt <= 15. else "degraded"
        assert pose["joint_quality"][slot] == pose["joint_quality"][slot + 1] == quality
        assert pose["joint_valid_mask"][slot] == (tilt <= 15.)
        assert pose["joint_valid_mask"][slot + 1] == (tilt <= 15.)
        assert pose["joint_display_available_mask"][slot]
        assert pose["joint_display_available_mask"][slot + 1]
        assert detail["residual_deg"] == pytest.approx(tilt, abs=2e-6)
        assert detail["delta_rad"] == pytest.approx(first_delta, abs=2e-7)
        assert detail["continuity"]["reset_reason"] is None
        assert pose["functional_adapter"]["no_uncorrected_hinge_fallback"]
        np.testing.assert_allclose(pose["anatomical_euler_rad"][slot], [0., 0., .6], atol=2e-7)
        np.testing.assert_allclose(pose["anatomical_euler_rad"][slot + 1], [0., 0., .39], atol=2e-7)
        # Child-only corruption must not change this finger's MCP or others.
        np.testing.assert_allclose(pose["anatomical_euler_rad"][slot - 1],
            initial["anatomical_euler_rad"][slot - 1], atol=2e-7)
        unchanged = [s for s in range(16) if s not in (slot - 1, slot, slot + 1)]
        np.testing.assert_array_equal(np.asarray(pose["anatomical_euler_rad"])[unchanged],
            np.asarray(raw["anatomical_euler_rad"])[unchanged])
        np.testing.assert_array_equal(pose["upstream_raw_anatomical_euler_rad"], raw["anatomical_euler_rad"])
        assert sample == before and profile == saved


@pytest.mark.parametrize("finger,slot", [("index", 2), ("little", 8), ("thumb", 15)])
def test_unified_real_pipeline_has_no_flexion_filter_during_protected_heading(
        official_core, finger, slot):
    backend, profile, frame, _, _ = trial(official_core, finger)
    root = (1.1, -.8, .3)
    initial = backend.solve(_with_virtual_hinge_error(backend, profile,
        frame(.4, root=root, sequence=399), finger, slot, yaw_deg=0.), profile)
    assert initial["joint_display_available_mask"][slot]
    assert not initial["joint_valid_mask"][slot]
    delta = initial["joint_estimation"][str(slot)]["delta_rad"]
    child_node = backend._functional_binding(profile)["dependency_nodes_by_slot"][str(slot)][1]
    for i, bend in enumerate((1.2, .1, 1.1, .3)):
        sample = _with_virtual_hinge_error(backend, profile,
            frame(bend, root=root, sequence=400 + i), finger, slot, yaw_deg=0.)
        sample["nodes"][child_node].update(orientation_valid=True, input_filter={"gyro_rejected": True})
        pose = backend.solve(sample, profile)
        assert pose["anatomical_euler_rad"][slot][2] == pytest.approx(bend, abs=2e-7)
        assert pose["joint_quality"][slot] == "degraded"
        assert not pose["joint_valid_mask"][slot] and pose["joint_display_available_mask"][slot]
        assert pose["joint_estimation"][str(slot)]["delta_rad"] == delta
        if slot != 15:
            assert pose["anatomical_euler_rad"][slot + 1][2] == pytest.approx(.65 * bend, abs=2e-7)
            assert pose["joint_quality"][slot + 1] == "degraded"


@pytest.mark.parametrize("invalid", ["orientation", "epoch", "missing", "skew"])
def test_unified_real_pipeline_invalid_dependency_never_falls_back_to_official_angle(
        official_core, invalid):
    backend, profile, frame, _, _ = trial(official_core, "little")
    root = (1.1, -.8, .3)
    warm = _with_virtual_hinge_error(backend, profile,
        frame(.4, root=root, sequence=399), "little", 8)
    assert backend.solve(warm, profile)["joint_display_available_mask"][8]
    bad = _with_virtual_hinge_error(backend, profile,
        frame(.9, root=root, sequence=400), "little", 8)
    if invalid == "orientation":
        bad["nodes"]["10"]["orientation_valid"] = False
    elif invalid == "epoch":
        bad["nodes"]["10"]["epoch"] += 1
    elif invalid == "missing":
        del bad["nodes"]["10"]
    else:
        bad["nodes"]["10"]["age_s"] = .06
    pose = backend.solve(bad, profile)
    assert pose["pose_available"] and pose["algorithm"] == FUNCTIONAL_ALGORITHM
    assert 8 not in pose["functional_adapter"]["applied_slots"]
    for slot in (8, 9):
        assert not pose["joint_valid_mask"][slot]
        assert not pose["joint_display_available_mask"][slot]
        assert pose["joint_quality"][slot] == "unavailable"
        assert pose["source_labels"][slot] == "unavailable_geometry_placeholder_not_measurement"
        assert not pose["joint_observed_mask"][slot]
        np.testing.assert_array_equal(pose["anatomical_euler_rad"][slot], [0., 0., 0.])
    assert not pose["sparse_prior"]["dip_by_finger"]["little"]["applied"]


def test_unified_real_pipeline_calibration_capture_does_not_reset_live_heading(official_core):
    backend, profile, frame, neutral, motion = trial(official_core, "little")
    warm = _with_virtual_hinge_error(backend, profile,
        frame(.4, root=(1.1, -.8, .3), sequence=399), "little", 8, yaw_deg=0.)
    initial = backend.solve(warm, profile)
    assert initial["joint_display_available_mask"][8]
    assert not initial["joint_valid_mask"][8]
    estimator = backend._functional_pipeline.hinges
    state = estimator._states[8]
    delta, stamp, angle = state.delta, state.source_time, state.angle
    history = deepcopy(list(state.observer.history))
    # Historical capture rows, not live rows, are sent through the official
    # solver to fit a new functional reference. This must not touch live state.
    backend._functional_capture_results([neutral, motion], profile, session_id=profile["session_id"])
    assert estimator._states[8] is state
    assert (state.delta, state.source_time, state.angle) == (delta, stamp, angle)
    for old, current in zip(history, state.observer.history):
        np.testing.assert_equal(old, current)
    changed = deepcopy(profile)
    changed["profile_version"] = changed.get("profile_version", 1) + 1
    fresh = _with_virtual_hinge_error(backend, changed,
        frame(1.1, root=(1.1, -.8, .3), sequence=400), "little", 8, yaw_deg=0.)
    fresh["nodes"]["10"]["input_filter"] = {"gyro_rejected": True}
    pose = backend.solve(fresh, changed)
    assert pose["anatomical_euler_rad"][8][2] == pytest.approx(1.1, abs=2e-7)
    assert pose["joint_quality"][8] == "degraded"
    assert pose["joint_estimation"]["8"]["continuity"]["reset_reason"] is None


def test_unified_real_pipeline_respects_stricter_backend_dependency_freshness(official_core, monkeypatch):
    import test_upstream_backend as fixtures
    config = fixtures.load_config()
    config["stale_after_s"] = .04
    monkeypatch.setattr(fixtures, "load_config", lambda: deepcopy(config))
    backend, profile, frame, _, _ = trial(official_core, "little")
    warm = _with_virtual_hinge_error(backend, profile,
        frame(.4, root=(1.1, -.8, .3), sequence=399), "little", 8)
    assert backend.solve(warm, profile)["joint_display_available_mask"][8]
    bad = _with_virtual_hinge_error(backend, profile,
        frame(.9, root=(1.1, -.8, .3), sequence=400), "little", 8)
    # Both ages satisfy the estimator's .15s ceiling and pair skew, but fail
    # the explicitly tighter input contract chosen for this backend.
    bad["nodes"]["9"]["age_s"] = bad["nodes"]["10"]["age_s"] = .06
    pose = backend.solve(bad, profile)
    assert not pose["segment_observed_mask"][7] and not pose["segment_observed_mask"][8]
    for slot in (8, 9):
        assert not pose["joint_valid_mask"][slot]
        assert not pose["joint_display_available_mask"][slot]
        assert pose["joint_quality"][slot] == "unavailable"
