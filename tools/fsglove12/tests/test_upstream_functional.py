"""Synthetic functional directions, not labels for real glove anatomy.

These tests call the installed MIT QMT quaternion projection. Synthetic local
rotations represent the explicitly unadapted official solver interface; whole
pipeline, real MANO, common-root and freshness tests live in the integration
suite. No calibration optimizer or IMU solver is reimplemented here.
"""
from copy import deepcopy
import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from vt_fsglove12 import upstream_functional as functional


qmt = pytest.importorskip("qmt", reason="functional adapter requires actual qmt.quatProject")


def binding():
    return dict(session_id="synthetic-functional-session", upstream_state_sha256="official-installation",
                config_sha256="node-map", rest_sha256="actual-mano-rest",
                nodes_epochs={str(n): 2 for n in range(1, 13)},
                dependency_nodes_by_slot={"1": ["12", "3"], "2": ["3", "4"],
                    "4": ["12", "1"], "5": ["1", "2"], "7": ["12", "9"],
                    "8": ["9", "10"], "10": ["12", "7"], "11": ["7", "8"],
                    "15": ["5", "6"]})


def result(angles, timestamp):
    return dict(anatomical_euler_rad=np.asarray(angles).tolist(), timestamp_s=float(timestamp),
                joint_observed_mask=[True] * 16, observed_mask=[True] * 16,
                calibration_rejections={}, pose_available=True)


def synthetic(seed=20261001, finger="index", maximum=135.):
    """Known arbitrary neutral mounts and functional axes (not measured anatomy)."""
    rng = np.random.default_rng(seed)
    slots = functional.FINGER_SLOTS[finger]
    references = {slot: Rotation.random(random_state=rng) for slot in slots}
    bases = {}
    for slot in slots:
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        bases[slot] = Rotation.align_vectors(axis, [0., 0., 1.])[0]

    def angles_by_target(targets):
        angles = np.zeros((16, 3))
        for slot in slots:
            target = np.asarray(targets.get(slot, [0., 0., 0.]))
            raw = references[slot] * bases[slot] * Rotation.from_euler("XYZ", target) * bases[slot].inv()
            angles[slot] = raw.as_euler("XYZ")
        return angles

    neutral = [result(angles_by_target({}), i) for i in range(8)]
    movement = np.r_[np.linspace(0., maximum, 24), np.linspace(maximum, 0., 24)]
    motion = [result(angles_by_target({s: [0., 0., np.deg2rad(angle)] for s in slots}), 10 + i)
              for i, angle in enumerate(movement)]
    return neutral, motion, angles_by_target, bases


def fit(seed=20261001, finger="index"):
    neutral, motion, make_angles, bases = synthetic(seed, finger)
    entry = functional.fit_functional_finger(neutral, motion, finger=finger, binding=binding())
    return entry, make_angles, bases


@pytest.mark.parametrize("seed", [1, 19, 101, 349])
def test_arbitrary_mounts_and_wrong_axes_recover_signed_unscaled_flexion(seed):
    entry, angles, bases = fit(seed)
    assert entry["kind"] == functional.KIND
    assert entry["rejected_slots"] == {}
    for slot in (1, 2):
        reference = entry["slot_refs"][str(slot)]
        np.testing.assert_allclose(reference["axis_local"], bases[slot].apply([0., 0., 1.]), atol=2e-14)
        assert entry["diagnostics"][str(slot)]["principal_axis_energy"] == pytest.approx(1.)
        assert entry["diagnostics"][str(slot)]["holdout_p95_residual_deg"] < 1e-10
        assert entry["diagnostics"][str(slot)]["holdout_used_for_axis_fitting"] is False
    for degree in (-35., 0., 65., 135.):
        target = np.deg2rad(degree)
        output, diagnostic = functional.apply_functional_fingers(
            angles({1: [0., 0., target], 2: [0., 0., target]}), {"index": entry},
            [True] * 16, binding=binding())
        np.testing.assert_allclose(output[[1, 2]], [[0., 0., target]] * 2, atol=2e-14)
        assert diagnostic["applied_slots"] == [1, 2]
        assert max(diagnostic["residual_deg_by_slot"].values()) < 1e-10
    assert entry["angle_gain"] == 1.
    assert not entry["basis_twist_observed"] and not entry["anatomical_accuracy_verified"]
    assert not entry["relative_yaw_drift_corrected"]
    assert json.loads(json.dumps(entry)) == entry
    assert entry["qmt"]["function"] == "qmt.quatProject"
    assert len(entry["qmt"]["source_sha256"]) == 64


def test_neutral_offset_removed_and_mcp_spread_bend_preserved_without_twist():
    entry, angles, _ = fit()
    target = np.deg2rad([27., -18., 72.])
    raw = angles({1: target})
    output, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=binding())
    np.testing.assert_allclose(output[1], [0., target[1], target[2]], atol=2e-14)
    np.testing.assert_allclose(output[2], 0., atol=2e-14)
    assert diagnostic["residual_deg_by_slot"]["1"] == pytest.approx(27.)
    assert diagnostic["warnings"] and not diagnostic["residual_is_anatomical_error"]


@pytest.mark.parametrize("finger", list(functional.FINGER_SLOTS))
def test_initial_small_reverse_excursion_does_not_invert_later_flexion(finger):
    neutral, motion, angles, bases = synthetic(finger=finger)
    slots = functional.FINGER_SLOTS[finger]
    early = result(angles({s: [0., 0., np.deg2rad(-9.)] for s in slots}), 9.)
    entry = functional.fit_functional_finger(neutral, [early] + motion,
        finger=finger, binding=binding())
    for slot in slots:
        np.testing.assert_allclose(entry["slot_refs"][str(slot)]["axis_local"],
                                   bases[slot].apply([0., 0., 1.]), atol=2e-14)
        support = entry["diagnostics"][str(slot)]["direction_support"]
        assert support["positive_energy_fraction"] > .99
        assert support["opposite_energy_fraction"] > 0.
        assert not support["uses_first_sample_only"]
    # A real reverse angle remains negative; this is not a clamp or gain fix.
    for degree in (-9., 0., 60., 135., 0.):
        expected = np.deg2rad(degree)
        output, _ = functional.apply_functional_fingers(
            angles({s: [0., 0., expected] for s in slots}), {finger: entry},
            [True] * 16, binding=binding())
        np.testing.assert_allclose(output[list(slots), 2], expected, atol=2e-14)


def test_direction_support_is_order_and_svd_sign_invariant():
    axis = np.array([0., 0., -1.])
    vectors = np.array([[0., 0., a] for a in (-.16, .4, 1.2, .9, .5)])
    first, support = functional._orient_flexion_axis(axis, vectors)
    for alternative in (vectors[::-1], np.roll(vectors, 2, axis=0)):
        for sign in (-1., 1.):
            oriented, info = functional._orient_flexion_axis(axis * sign, alternative)
            np.testing.assert_array_equal(oriented, first)
            assert info["positive_energy_fraction"] == pytest.approx(support["positive_energy_fraction"])


def test_ambiguous_opposite_excursions_do_not_overwrite_previous_reference():
    previous, _, _ = fit()
    saved = deepcopy(previous)
    neutral, _, angles, _ = synthetic()
    motion = [result(angles({s: [0., 0., a] for s in (1, 2)}), 30 + i)
              for i, a in enumerate([-.7, .7, -.7, .7, -.7, .7])]
    with pytest.raises(ValueError, match="正方向不明确"):
        functional.fit_functional_finger(neutral, motion, finger="index",
                                         binding=binding(), previous=previous)
    assert previous == saved


def test_qmt_real_function_called_scalar_first_and_residual_is_not_silently_hidden(monkeypatch):
    entry, angles, _ = fit()
    original = qmt.quatProject
    calls = []

    def spy(quaternion, axis, **kwargs):
        calls.append((np.array(quaternion), np.array(axis)))
        return original(quaternion, axis, **kwargs)

    monkeypatch.setattr(qmt, "quatProject", spy)
    target = np.deg2rad([25., 0., 40.])
    output, diagnostic = functional.apply_functional_fingers(angles({2: target}), {"index": entry},
                                                             [True] * 16, binding=binding())
    assert len(calls) == 1
    assert calls[0][0].shape == (2, 4)
    expected = Rotation.from_euler("XYZ", target).as_quat()[[3, 0, 1, 2]]
    assert abs(np.dot(calls[0][0][1], expected)) == pytest.approx(1.)
    np.testing.assert_allclose(calls[0][1], [0., 0., 1.])
    np.testing.assert_allclose(output[2], np.deg2rad([0., 0., 40.]), atol=2e-14)
    assert diagnostic["residual_deg_by_slot"]["2"] == pytest.approx(25.)
    assert any("25.0" in message for message in diagnostic["warnings"])


@pytest.mark.parametrize("finger", ["index", "middle", "ring", "little", "thumb"])
def test_only_selected_measured_joints_changed_and_inputs_not_mutated(finger):
    entry, make_angles, _ = fit(finger=finger)
    raw = make_angles({slot: [.15, -.2, .7] for slot in functional.FINGER_SLOTS[finger]})
    others = [slot for slot in range(16) if slot not in functional.FINGER_SLOTS[finger]]
    raw[others] = [.11, -.09, .23]
    entries, current_binding, mask = {finger: entry}, binding(), [True] * 16
    old_raw, old_entries, old_binding = raw.copy(), deepcopy(entries), deepcopy(current_binding)
    output, diagnostic = functional.apply_functional_fingers(raw, entries, mask, binding=current_binding)
    np.testing.assert_array_equal(output[others], raw[others])
    np.testing.assert_array_equal(raw, old_raw)
    assert entries == old_entries and current_binding == old_binding and mask == [True] * 16
    assert diagnostic["applied_slots"] == sorted(functional.FINGER_SLOTS[finger])
    if finger == "thumb":
        np.testing.assert_array_equal(output[[13, 14]], raw[[13, 14]])
        assert entry["slot_refs"]["15"]["mode"] == "hinge_projection"


def test_missing_parent_or_child_mask_passes_current_input_and_never_uses_previous_output():
    entry, angles, _ = fit()
    raw = angles({1: [0., 0., .8], 2: [0., 0., 1.2]})
    first, _ = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=binding())
    mask = [True] * 16
    mask[2] = False
    current = raw.copy()
    current[2] = [.2, -.6, .1]
    dropped, diagnostic = functional.apply_functional_fingers(current, {"index": entry}, mask, binding=binding())
    np.testing.assert_array_equal(dropped[2], current[2])
    assert not np.allclose(dropped[2], first[2])
    assert diagnostic["applied_slots"] == [1]
    assert diagnostic["skipped_slots"]["2"] == "parent_or_child_not_currently_observed"
    assert not diagnostic["depends_on_previous_frame"]
    restored, _ = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=binding())
    np.testing.assert_array_equal(restored, first)


@pytest.mark.parametrize("field", ["session_id", "upstream_state_sha256", "config_sha256", "rest_sha256"])
def test_global_binding_change_skips_saved_adapter(field):
    entry, angles, _ = fit()
    current_binding = binding()
    current_binding[field] += "-changed"
    raw = angles({1: [0., 0., .5], 2: [0., 0., .8]})
    output, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=current_binding)
    np.testing.assert_array_equal(output, raw)
    assert diagnostic["applied_slots"] == []
    assert set(diagnostic["skipped_slots"]) == {"1", "2"}


def test_only_this_joint_dependency_epochs_invalidate_it():
    entry, angles, _ = fit()
    raw = angles({1: [0., 0., .5], 2: [0., 0., .8]})
    current_binding = binding()
    current_binding["nodes_epochs"]["10"] += 1  # Unrelated little-finger node.
    _, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=current_binding)
    assert diagnostic["applied_slots"] == [1, 2]
    current_binding["nodes_epochs"]["4"] += 1  # Index PIP child only.
    output, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=current_binding)
    assert diagnostic["applied_slots"] == [1]
    np.testing.assert_array_equal(output[2], raw[2])


def test_joint_rejection_does_not_discard_other_success_or_same_binding_previous():
    previous, _, _ = fit()
    old_copy = deepcopy(previous)
    neutral, motion, make_angles, _ = synthetic(seed=17)
    # Strong three-axis motion is not a usable hinge direction. MCP remains valid.
    for index, sample in enumerate(motion):
        rotation = Rotation.from_rotvec(np.eye(3)[index % 3])
        reference = Rotation.from_euler("XYZ", neutral[0]["anatomical_euler_rad"][2])
        sample["anatomical_euler_rad"][2] = (reference * rotation).as_euler("XYZ").tolist()
    new = functional.fit_functional_finger(neutral, motion, finger="index", binding=binding(), previous=previous)
    assert new["diagnostics"]["1"]["status"] == "fitted"
    assert new["diagnostics"]["2"]["status"] == "retained_previous"
    assert "混合转轴" in new["rejected_slots"]["2"]
    assert new["slot_refs"]["2"] == previous["slot_refs"]["2"]
    assert previous == old_copy
    changed = binding()
    changed["nodes_epochs"]["4"] += 1
    new = functional.fit_functional_finger(neutral, motion, finger="index", binding=changed, previous=previous)
    assert set(new["slot_refs"]) == {"1"}
    assert new["diagnostics"]["2"]["status"] == "rejected"


@pytest.mark.parametrize("problem", ["small_motion", "unstable_neutral", "near_pi", "missing_joint",
                                     "base_rejected", "duplicate_time", "unavailable_pose"])
def test_unusable_capture_keeps_previous_untouched_and_reports_reason(problem):
    previous, _, _ = fit()
    old = deepcopy(previous)
    neutral, motion, angles, _ = synthetic(maximum=6. if problem == "small_motion" else 135.)
    if problem == "unstable_neutral":
        neutral = [result(angles({1: [0., 0., (-1) ** i * .3], 2: [0., 0., (-1) ** i * .3]}), i)
                   for i in range(8)]
    elif problem == "near_pi":
        motion[-1] = result(angles({1: [0., 0., np.deg2rad(175.)], 2: [0., 0., np.deg2rad(175.)]}), 100.)
    elif problem == "missing_joint":
        for sample in motion:
            sample["joint_observed_mask"][1:3] = [False, False]
    elif problem == "base_rejected":
        for sample in motion:
            sample["calibration_rejections"] = {"3": "wrong_epoch"}
    elif problem == "duplicate_time":
        motion = [motion[5]] * 40
    elif problem == "unavailable_pose":
        for sample in motion:
            sample["pose_available"] = False
    with pytest.raises(ValueError, match="没有可更新"):
        functional.fit_functional_finger(neutral, motion, finger="index", binding=binding(), previous=previous)
    assert previous == old


def test_three_moving_samples_are_enough_and_135_degrees_not_clipped():
    neutral, _, make_angles, _ = synthetic()
    motion = [result(make_angles({1: [0., 0., a], 2: [0., 0., a]}), 30 + i)
              for i, a in enumerate(np.deg2rad([20., 90., 135.]))]
    entry = functional.fit_functional_finger(neutral[:3], motion, finger="index", binding=binding())
    assert entry["diagnostics"]["2"]["observed_range_deg"][1] == pytest.approx(135.)
    assert entry["diagnostics"]["2"]["holdout_samples"] == 1


def _mixed_axis_capture(finger="index", off_axis_deg=18.):
    neutral, _, make_angles, _ = synthetic(finger=finger)
    motion = [result(make_angles({slot: np.deg2rad([(-1) ** i * off_axis_deg, 0., angle])
                                         for slot in functional.FINGER_SLOTS[finger]}), 30 + i)
              for i, angle in enumerate(np.linspace(60., 110., 36))]
    return neutral, motion


def test_dominant_axis_with_high_residual_is_rejected_for_hinge_but_not_two_dof_mcp():
    # A dominant SVD axis alone does not establish a usable single-axis model.
    # Previously both joints were saved with warnings, even though the PIP
    # capture already violated the online hinge's existing 15-degree limit.
    neutral, motion = _mixed_axis_capture()
    entry = functional.fit_functional_finger(neutral, motion, finger="index", binding=binding())
    assert set(entry["slot_refs"]) == {"1"}
    assert set(entry["rejected_slots"]) == {"2"}
    mcp = entry["diagnostics"]["1"]
    pip = entry["diagnostics"]["2"]["candidate"]
    for diagnostic in (mcp, pip):
        assert diagnostic["principal_axis_energy"] > .75
        assert diagnostic["holdout_p95_residual_deg"] > 15.
        assert not diagnostic["anatomical_angle_ground_truth_available"]
    assert mcp["acceptance"]["status"] == "accepted"
    assert mcp["acceptance"]["criterion"] == "axis_direction_only"
    assert mcp["acceptance"]["residual_limit_deg"] is None
    assert pip["acceptance"]["status"] == "rejected"
    assert pip["acceptance"]["reason_codes"] == ["hinge_model_residual_too_large"]
    assert "单指补采" in entry["rejected_slots"]["2"]


def test_rejected_hinge_candidate_keeps_previous_axis_and_its_evidence_separate():
    previous, _, _ = fit()
    saved = deepcopy(previous)
    neutral, motion = _mixed_axis_capture()
    entry = functional.fit_functional_finger(neutral, motion, finger="index",
                                             binding=binding(), previous=previous)
    assert entry["slot_refs"]["2"] == saved["slot_refs"]["2"]
    assert entry["diagnostics"]["2"]["status"] == "retained_previous"
    assert entry["diagnostics"]["2"]["candidate"]["acceptance"]["status"] == "rejected"
    assert previous == saved


def test_entire_finger_rejection_retains_numeric_candidate_evidence_and_previous():
    previous, _, _ = fit(finger="thumb")
    saved = deepcopy(previous)
    neutral, motion = _mixed_axis_capture("thumb")
    with pytest.raises(functional.FunctionalCalibrationRejected, match="模型验收") as error:
        functional.fit_functional_finger(neutral, motion, finger="thumb",
                                         binding=binding(), previous=previous)
    diagnostic = error.value.diagnostics["15"]
    assert diagnostic["status"] == "retained_previous"
    assert diagnostic["candidate"]["holdout_p95_residual_deg"] > 15.
    assert diagnostic["candidate"]["acceptance"]["status"] == "rejected"
    assert set(error.value.rejected_slots) == {"15"}
    assert previous == saved


def test_usable_hinge_candidate_above_warning_level_still_passes_online_model_limit():
    neutral, motion = _mixed_axis_capture(off_axis_deg=12.)
    entry = functional.fit_functional_finger(neutral, motion, finger="index", binding=binding())
    assert set(entry["slot_refs"]) == {"1", "2"}
    pip = entry["diagnostics"]["2"]
    assert 10. < pip["holdout_p95_residual_deg"] < 15.
    assert pip["warnings"]
    assert pip["acceptance"]["status"] == "accepted"


def test_legacy_reference_retention_does_not_invent_acceptance_evidence():
    previous, _, _ = fit()
    for value in previous["diagnostics"].values():
        value.pop("acceptance")
    saved = deepcopy(previous)
    neutral, motion = _mixed_axis_capture()
    entry = functional.fit_functional_finger(neutral, motion, finger="index",
                                             binding=binding(), previous=previous)
    assert entry["slot_refs"]["2"] == saved["slot_refs"]["2"]
    assert "acceptance" not in entry["diagnostics"]["2"]
    assert entry["diagnostics"]["2"]["candidate"]["acceptance"]["status"] == "rejected"
    assert previous == saved


def test_mcp_two_dof_capture_does_not_claim_hinge_validation():
    neutral, _, make_angles, _ = synthetic()
    motion = [result(make_angles({slot: np.deg2rad([(-1) ** i * 18., 0., angle])
                                         for slot in (1, 2)}), 30 + i)
              for i, angle in enumerate(np.linspace(60., 110., 36))]
    entry = functional.fit_functional_finger(neutral, motion, finger="index", binding=binding())
    diagnostic = entry["diagnostics"]["1"]
    assert diagnostic["warnings"]
    assert diagnostic["acceptance"]["criterion"] == "axis_direction_only"
    assert not diagnostic["acceptance"]["anatomical_accuracy_verified"]


def test_no_entries_needs_no_qmt_and_missing_qmt_never_uses_replacement(monkeypatch):
    entry, angles, _ = fit()
    real_import = functional.importlib.import_module

    def unavailable(name, *args, **kwargs):
        if name == "qmt":
            raise ImportError("test intentionally unavailable qmt")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(functional.importlib, "import_module", unavailable)
    raw = angles({})
    output, diagnostic = functional.apply_functional_fingers(raw, {}, [True] * 16, binding=binding())
    np.testing.assert_array_equal(output, raw)
    assert not diagnostic["applied_slots"]
    with pytest.raises(functional.FunctionalDependencyError, match="qmt.quatProject"):
        functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=binding())


def test_orthogonal_180_degree_projection_is_ambiguous_and_skips_only_that_joint():
    entry, angles, _ = fit()
    raw = angles({1: [0., 0., .7], 2: [np.pi, 0., 0.]})
    output, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=binding())
    np.testing.assert_array_equal(output[2], raw[2])
    assert diagnostic["applied_slots"] == [1]
    assert "投影不唯一" in diagnostic["skipped_slots"]["2"]


@pytest.mark.parametrize("selected", [None, {1, 4, 7, 10}, {2, 5, 8, 11, 15}])
def test_batched_official_projection_matches_scalar_for_all_fingers(monkeypatch, selected):
    entries, raw = {}, np.zeros((16, 3))
    for index, finger in enumerate(functional.FINGER_SLOTS):
        entry, angles, _ = fit(seed=19 + index, finger=finger)
        entries[finger] = entry
        slots = functional.FINGER_SLOTS[finger]
        sample = angles({slot: [.17, -.13, .31 + .03 * slot] for slot in slots})
        raw[list(slots)] = sample[list(slots)]
    actual, diagnostic = functional.apply_functional_fingers(raw, entries, [True] * 16,
                                                             binding=binding(), slots=selected)
    project = functional._project

    def scalar_only(rotations, axis):
        if not rotations.single:
            raise ValueError("force compatibility scalar fallback")
        return project(rotations, axis)

    monkeypatch.setattr(functional, "_project", scalar_only)
    expected, reference = functional.apply_functional_fingers(raw, entries, [True] * 16,
                                                             binding=binding(), slots=selected)
    np.testing.assert_allclose(actual, expected, rtol=0., atol=2e-14)
    for field in ("projection_deg_by_slot", "residual_deg_by_slot"):
        assert diagnostic.pop(field) == pytest.approx(reference.pop(field), abs=1e-10)
    assert diagnostic == reference


def test_nonfinite_batch_falls_back_per_slot_without_losing_healthy_joint(monkeypatch):
    entry, angles, _ = fit()
    original = qmt.quatProject
    calls = []

    def one_bad_result(quaternion, axis, **kwargs):
        calls.append(np.asarray(quaternion).shape)
        value = original(quaternion, axis, **kwargs)
        value["projAngle"] = (np.full(2, np.nan) if np.ndim(quaternion) == 2 else value["projAngle"])
        return value

    monkeypatch.setattr(qmt, "quatProject", one_bad_result)
    raw = angles({1: [0., 0., .4], 2: [0., 0., .8]})
    output, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16,
                                                             binding=binding())
    assert calls == [(2, 4), (4,), (4,)]
    assert diagnostic["applied_slots"] == [1, 2]
    np.testing.assert_allclose(output[[1, 2], 2], [.4, .8], atol=2e-14)


def test_static_reference_cache_detects_mutation_and_revalidates_only_affected_slot():
    entry, angles, _ = fit()
    raw = angles({1: [0., 0., .4], 2: [0., 0., .8]})
    entries = {"index": entry}
    first, _ = functional.apply_functional_fingers(raw, entries, [True] * 16, binding=binding())
    reference = entry["slot_refs"]["2"]
    reference["neutral_local_xyzw"][:] = Rotation.from_euler("XYZ", raw[2]).as_quat().tolist()
    changed, _ = functional.apply_functional_fingers(raw, entries, [True] * 16, binding=binding())
    functional._reference_rotations.cache_clear()
    fresh, _ = functional.apply_functional_fingers(raw, entries, [True] * 16, binding=binding())
    np.testing.assert_array_equal(changed, fresh)
    np.testing.assert_allclose(changed[2], 0., atol=2e-14)
    assert not np.allclose(first[2], changed[2])
    reference["neutral_local_xyzw"][:] = [0., 0., 0., 2.]
    bad, diagnostic = functional.apply_functional_fingers(raw, entries, [True] * 16, binding=binding())
    assert diagnostic["applied_slots"] == [1]
    assert "单位 xyzw quaternion" in diagnostic["skipped_slots"]["2"]
    np.testing.assert_array_equal(bad[2], raw[2])
    assert functional._reference_rotations.cache_info().maxsize == 128


def test_axis_sign_comes_from_declared_flexion_not_nearness_to_model_z():
    neutral, motion, make_angles, _ = synthetic()
    for sample in motion:
        for slot in (1, 2):
            ref = Rotation.from_euler("XYZ", neutral[0]["anatomical_euler_rad"][slot])
            delta = ref.inv() * Rotation.from_euler("XYZ", sample["anatomical_euler_rad"][slot])
            sample["anatomical_euler_rad"][slot] = (ref * delta.inv()).as_euler("XYZ").tolist()
    entry = functional.fit_functional_finger(neutral, motion, finger="index", binding=binding())
    raw = make_angles({2: [0., 0., -.9]})
    output, _ = functional.apply_functional_fingers(raw, {"index": entry}, [True] * 16, binding=binding())
    assert output[2, 2] == pytest.approx(.9)


@pytest.mark.parametrize("invalid", ["mask", "nan", "binding", "entry"])
def test_invalid_contract_is_reported_or_skipped_without_mutating_input(invalid):
    entry, angles, _ = fit()
    raw, mask, current_binding = angles({}), [True] * 16, binding()
    if invalid == "mask":
        mask = [1] * 16
    elif invalid == "nan":
        raw[2, 2] = np.nan
    elif invalid == "binding":
        del current_binding["session_id"]
    else:
        entry["binding"] = None
        output, diagnostic = functional.apply_functional_fingers(raw, {"index": entry}, mask, binding=current_binding)
        np.testing.assert_array_equal(output, raw)
        assert not diagnostic["applied_slots"]
        return
    with pytest.raises(ValueError):
        functional.apply_functional_fingers(raw, {"index": entry}, mask, binding=current_binding)
