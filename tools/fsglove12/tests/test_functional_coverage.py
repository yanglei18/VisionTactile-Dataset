"""Coverage/provenance contract tests, not physical calibration validation."""
from contextlib import nullcontext
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from vt_fsglove12 import functional_pose
from vt_fsglove12.config import PARENTS
from vt_fsglove12.upstream_functional import FINGER_SLOTS


FIELDS = ("joint_calibration_status", "functional_calibration_coverage", "joint_output_provenance")
EXPECTED = [1, 2, 4, 5, 7, 8, 10, 11, 15]


def base_result():
    observed = [slot not in (3, 6, 9, 12, 13) for slot in range(16)]
    joints = [observed[slot] and observed[PARENTS[slot]] for slot in range(16)]
    return dict(pose_available=True, solver_success=True,
        anatomical_euler_rad=(np.arange(48).reshape(16, 3) / 100).tolist(),
        mano_pose_rotvec=(np.arange(48).reshape(16, 3) / 200).tolist(),
        observed_mask=observed, segment_observed_mask=list(observed),
        joint_observed_mask=joints, inferred_mask=[not v for v in joints],
        source_labels=["official_measured_root" if slot == 0 else
            "official_measured_parent_child" if joints[slot] else
            "official_measured_segment_with_inferred_parent" if observed[slot] else
            "explicit_fixed_zero_anatomical_prior" for slot in range(16)],
        sparse_prior={})


class _Tensor:
    """Minimal compositor stub; tests below assert provenance, not MANO math."""
    def __init__(self, values):
        self.values = values

    def float(self):
        return self

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def apply_fixture(monkeypatch, slots, *, enabled=True, states=None, result=None):
    entries = {finger: dict(slot_refs={str(slot): {} for slot in joints if slot in slots})
               for finger, joints in FINGER_SLOTS.items() if any(slot in slots for slot in joints)}
    profile = dict(functional_fingers=entries, functional_enabled=enabled)
    untouched_profile = deepcopy(profile)
    result = base_result() if result is None else result
    original = deepcopy(result)
    states = states or {}

    def mcp(angles, _entries, _observed, *, binding, slots):
        adjusted = np.array(angles, copy=True)
        applied, skipped = [], {}
        for slot in slots & set(int(k) for e in entries.values() for k in e['slot_refs']):
            if states.get(slot) == "unavailable":
                skipped[str(slot)] = "test_reference_unavailable"
            else:
                adjusted[slot] = [0., angles[slot, 1], .4 + slot / 100]
                applied.append(slot)
        return adjusted, dict(applied_slots=applied, skipped_slots=skipped, warnings=[],
            residual_deg_by_slot={str(slot): 16. if states.get(slot) == 'degraded' else 1.
                                  for slot in applied}, projection_deg_by_slot={})

    def hinge(slot, *_args):
        quality = states.get(slot, "valid")
        angle = None if quality == "unavailable" else .5 + slot / 100
        return angle, dict(valid=quality == "valid", display_available=angle is not None,
            status="test_projection", quality=quality, applied=angle is not None,
            heading_updated=False, residual_deg=1.)

    monkeypatch.setattr(functional_pose, 'apply_functional_fingers', mcp)
    pipeline = functional_pose.FunctionalPosePipeline()
    pipeline.hinges = SimpleNamespace(apply=hinge, reset=lambda *_args: None)
    core = SimpleNamespace(torch=SimpleNamespace(no_grad=nullcontext, from_numpy=_Tensor))
    calibrator = SimpleNamespace(_axislayer=SimpleNamespace(
        compose=lambda tensor: [_Tensor(tensor.values[0])]))
    output = pipeline.apply(result, profile, calibrator, {}, {}, core,
        coupled=False, algorithm="test_functional_algorithm")
    assert profile == untouched_profile
    return original, output


def test_complete_functional_coverage_is_not_physical_accuracy_or_fresh_heading(monkeypatch):
    original, result = apply_fixture(monkeypatch, EXPECTED)
    coverage = result['functional_calibration_coverage']
    assert coverage['version'] == 1 and coverage['order'] == 'MANO16'
    assert coverage['status'] == coverage['reference_coverage'] == 'complete'
    assert coverage['expected_slots'] == EXPECTED
    for name in ('reference_present_mask', 'correction_applied_mask', 'correction_valid_mask'):
        assert coverage[name] == [slot in EXPECTED for slot in range(16)]
    for slot in EXPECTED:
        assert result['joint_calibration_status'][slot] == 'corrected'
        assert result['joint_output_provenance'][slot]['source_kind'] == 'functional'
        assert result['joint_output_provenance'][slot]['base_joint_observed']
    for slot in (3, 6, 9, 12, 13, 14):
        assert result['joint_calibration_status'][slot] == 'not_applicable'
        assert result['joint_output_provenance'][slot]['source_kind'] == 'prior'
        assert not result['joint_output_provenance'][slot]['base_joint_observed']
    assert result['joint_output_provenance'][0]['source_kind'] == 'base'
    assert all(not item['anatomical_accuracy_verified'] for item in result['joint_output_provenance'])
    assert not coverage['anatomical_accuracy_verified']
    assert all(not detail['heading_updated'] for detail in result['joint_estimation'].values())
    assert result['upstream_raw_anatomical_euler_rad'] == original['anatomical_euler_rad']
    assert result['upstream_raw_mano_pose_rotvec'] == original['mano_pose_rotvec']


def test_real_session_shaped_partial_coverage_preserves_uncorrected_base_values(monkeypatch):
    # Same coverage shape as the reported partial hand fit, not its sensor data.
    original, result = apply_fixture(monkeypatch, [5, 7, 10, 15])
    coverage = result['functional_calibration_coverage']
    assert coverage['status'] == coverage['reference_coverage'] == 'partial'
    for slot in (1, 2, 4, 8, 11):
        assert result['joint_calibration_status'][slot] == 'reference_missing'
        assert result['joint_output_provenance'][slot]['source_kind'] == 'base'
        assert result['joint_output_provenance'][slot]['base_output_valid']
        assert result['joint_valid_mask'][slot] and result['joint_display_available_mask'][slot]
        assert result['anatomical_euler_rad'][slot] == original['anatomical_euler_rad'][slot]
        assert result['source_labels'][slot] == original['source_labels'][slot]
        assert not coverage['correction_valid_mask'][slot]
    assert result['joint_calibration_status'][15] == 'corrected'
    assert result['pose_available'] and result['solver_success']


@pytest.mark.parametrize('slots,enabled,status,reference', [
    ([], True, 'no_references', 'none'),
    (EXPECTED, False, 'disabled', 'complete'),
    ([], False, 'disabled', 'none'),
])
def test_disabled_and_no_references_do_not_change_base_result(monkeypatch, slots, enabled, status, reference):
    original, result = apply_fixture(monkeypatch, slots, enabled=enabled)
    coverage = result['functional_calibration_coverage']
    assert coverage['status'] == status and coverage['reference_coverage'] == reference
    assert not any(coverage['correction_applied_mask'])
    assert not any(coverage['correction_valid_mask'])
    for key, value in original.items():
        assert result[key] == value
    assert 'joint_valid_mask' not in result  # Retain legacy base validity, not invented failure.
    assert all(result['joint_output_provenance'][slot]['base_output_valid'] for slot in range(16))


def test_present_reference_is_not_a_current_valid_correction(monkeypatch):
    _, result = apply_fixture(monkeypatch, EXPECTED, states={2: 'degraded', 8: 'unavailable', 10: 'unavailable'})
    coverage = result['functional_calibration_coverage']
    assert coverage['reference_coverage'] == 'complete'
    assert coverage['status'] == 'partial'
    assert result['joint_calibration_status'][2] == 'degraded'
    assert coverage['correction_applied_mask'][2] and not coverage['correction_valid_mask'][2]
    for slot in (8, 10):
        assert result['joint_calibration_status'][slot] == 'unavailable'
        assert not coverage['correction_applied_mask'][slot]
        assert not coverage['correction_valid_mask'][slot]
        assert result['joint_output_provenance'][slot]['source_kind'] == 'unavailable_geometry'
        assert result['joint_output_provenance'][slot]['base_output_valid']


def test_no_usable_correction_does_not_report_complete(monkeypatch):
    _, result = apply_fixture(monkeypatch, [1], states={1: 'unavailable'})
    assert result['functional_calibration_coverage']['status'] == 'unavailable'


def test_mcp_residual_warning_degrades_coverage_without_changing_base_validity_or_angle(monkeypatch):
    _, result = apply_fixture(monkeypatch, [1], states={1: 'degraded'})
    assert result['functional_adapter']['residual_deg_by_slot']['1'] == 16.
    assert result['joint_valid_mask'][1] and result['joint_display_available_mask'][1]
    np.testing.assert_allclose(result['anatomical_euler_rad'][1], [0., .04, .41], atol=1e-15)
    assert result['joint_calibration_status'][1] == 'degraded'
    coverage = result['functional_calibration_coverage']
    assert coverage['correction_applied_mask'][1] and not coverage['correction_valid_mask'][1]
    assert result['joint_output_provenance'][1]['source_kind'] == 'functional'
    assert result['joint_output_provenance'][1]['base_output_valid']


def test_existing_base_quality_is_not_rehabilitated_by_partial_functional_layer(monkeypatch):
    base = base_result()
    base.update(joint_valid_mask=[True] * 16, joint_display_available_mask=[True] * 16,
                joint_quality=['valid'] * 16)
    base['joint_valid_mask'][4] = base['joint_display_available_mask'][4] = False
    base['joint_quality'][4] = 'unavailable'
    _, result = apply_fixture(monkeypatch, [7, 8], result=base)
    assert not result['joint_valid_mask'][4] and not result['joint_display_available_mask'][4]
    assert result['joint_quality'][4] == 'unavailable'
    assert not result['joint_output_provenance'][4]['base_output_valid']
    assert result['joint_calibration_status'][4] == 'reference_missing'


@pytest.mark.parametrize('slots,enabled', [(EXPECTED, True), ([5, 7, 10, 15], True), (EXPECTED, False), ([], True)])
def test_coverage_exports_as_independent_quality_without_changing_pose_validity(monkeypatch, slots, enabled):
    import json
    from pathlib import Path
    from test_pose_export import inputs, state
    _, output = apply_fixture(monkeypatch, slots, enabled=enabled)
    frame, base, geometry = inputs()
    pose = {**base, **output}
    before = deepcopy(pose)
    exported = state(frame, pose, geometry)
    without_coverage = state(frame, {k: v for k, v in pose.items() if k not in FIELDS}, geometry)
    assert exported['pose_valid'] == without_coverage['pose_valid']
    for key in ('mano_pose_rotvec', 'root_quat_xyzw', 'joint_positions_m', 'source_labels',
                'joint_observed_mask', 'inferred_mask', 'segment_observed_mask'):
        assert exported[key] == without_coverage[key]
    quality = json.loads(exported['quality_json'])
    for key in FIELDS:
        assert quality[key] == pose[key]
    assert not exported['anatomical_accuracy_verified']
    assert len(json.dumps(exported).encode()) < 60_000
    assert 'raw_values' not in json.dumps(exported)
    assert pose == before
    # The existing ROS JSON ABI accepts these additive results-only fields;
    # no ROS process, generated message, device, or new .msg is required.
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[3] / 'ros2_ws/src/vt_glove_bridge'))
    from vt_glove_bridge.protocol import decode_state
    decoded = decode_state(json.dumps({**exported, 'producer_dropped_total': 0}).encode())
    assert decoded['pose_valid'] == exported['pose_valid']
    assert json.loads(decoded['quality_json']) == quality


# Optional real-core integration: these fixtures require explicit external
# source/model paths, and never connect hardware or write saved profiles.
from test_upstream_backend import official_core
from test_upstream_functional_hand import batch_data, fit_batch


def test_real_official_qmt_profile_coverage_preserves_missing_base_and_coupled_provenance(batch_data):
    backend, base_profile, _, fingers, _ = batch_data
    fitted = fit_batch(batch_data)
    assert fitted['errors'] == {}
    full = {**base_profile, 'functional_fingers': fitted['entries'], 'functional_enabled': True}
    original = deepcopy(full)
    complete = backend.solve(fingers[5], full)
    coverage = complete['functional_calibration_coverage']
    assert coverage['reference_coverage'] == 'complete'
    assert all(coverage['reference_present_mask'][slot] for slot in EXPECTED)
    for slot in (3, 6, 9, 12):
        assert complete['joint_calibration_status'][slot] == 'not_applicable'
        assert complete['joint_output_provenance'][slot]['source_kind'] in ('prior', 'unavailable_geometry')
        assert not coverage['correction_valid_mask'][slot]
        assert not complete['joint_output_provenance'][slot]['base_joint_observed']
    partial = deepcopy(full)
    partial['functional_fingers'].pop('index')
    for finger, slot in [('middle', 4), ('ring', 11), ('little', 8)]:
        partial['functional_fingers'][finger]['slot_refs'].pop(str(slot))
    result = backend.solve(fingers[6], partial)
    assert result['functional_calibration_coverage']['reference_coverage'] == 'partial'
    for slot in (1, 2, 4, 8, 11):
        assert result['joint_calibration_status'][slot] == 'reference_missing'
        assert result['joint_valid_mask'][slot]
        assert result['joint_output_provenance'][slot]['source_kind'] == 'base'
        np.testing.assert_array_equal(result['anatomical_euler_rad'][slot],
                                      result['upstream_raw_anatomical_euler_rad'][slot])
    assert full == original
