"""Unloaded-reference and live-response behavior, without hardware or Qt."""
from dataclasses import FrozenInstanceError
import json

import numpy as np
import pytest

from vt_tactile_capture.display_response import (
    ReferenceProfile, ResponseSettings, TactileResponseProcessor,
)


IDENTITY = ("right-tactile", 1)
SECOND = 1_000_000_000


def inputs(value=1000., *, stamp=SECOND, cells=400):
    return (np.full(400, value), np.arange(400) < cells,
            np.full(400, stamp, dtype=np.int64))


def reference(*, settings=None, cells=400, noise=None):
    response = TactileResponseProcessor(settings)
    response.begin_reference(IDENTITY, 0)
    for index in range(1, 31):
        now = index * SECOND // 10
        values, mask, stamps = inputs(stamp=now, cells=cells)
        if noise is not None:
            values += noise[index % len(noise)]
        response.observe(values, mask, stamps, IDENTITY, now)
    assert response.status.phase == "ready"
    return response


def render(response, value=1000., *, now=4 * SECOND, identity=IDENTITY, cells=400):
    values, mask, _ = inputs(value, cells=cells)
    return response.transform(values, mask, identity, now)


def test_no_reference_and_collection_show_neutral_but_are_not_ready():
    response = TactileResponseProcessor()
    for value in (0., 1000., 65000.):
        frame = render(response, value)
        assert frame["values"] == [0.] * 400
        assert all(frame["valid_mask"])
        assert frame["levels"] == [0., 1000.]
        assert frame["value_kind"] == "tactile_response"
        assert response.status.phase == "unreferenced"
    response.begin_reference(IDENTITY, 5 * SECOND)
    assert render(response, 2000., now=6 * SECOND)["values"] == [0.] * 400
    assert response.status.phase == "collecting"
    assert response.status.progress == pytest.approx(1 / 3)


def test_zeroing_removes_per_cell_offsets_and_retains_raw_input():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    values = np.arange(400, dtype=np.uint16) * 100
    original = values.copy()
    for index in range(1, 31):
        now = index * SECOND // 10
        _, mask, stamps = inputs(stamp=now)
        response.observe(values, mask, stamps, IDENTITY, now)
    frame = response.transform(values, mask, IDENTITY, 4 * SECOND)
    assert frame["values"] == [0.] * 400
    assert response.status.reference_cells == 400
    np.testing.assert_array_equal(values, original)


def test_local_press_hold_release_and_fixed_scale_without_baseline_tracking():
    response = reference()
    values, mask, _ = inputs()
    values[21] += 80
    original = values.copy()
    first = response.transform(values, mask, IDENTITY, 4 * SECOND)
    assert first["values"][21] > 0
    assert sum(value > 0 for value in first["values"]) == 1
    for now in (10 * SECOND, 60 * SECOND, 600 * SECOND):
        assert response.transform(values, mask, IDENTITY, now) == first
    assert response.status.active_cells == 1
    assert render(response, now=601 * SECOND)["values"] == [0.] * 400
    assert response.status.active_cells == 0
    np.testing.assert_array_equal(values, original)


def test_noise_is_measured_per_cell_instead_of_a_global_sensitivity_cutoff():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    noise = [-5., -4., -3., -2., -1., 0., 1., 2., 3., 4., 5.]
    for index in range(1, 61):
        now = index * SECOND // 20
        values, mask, stamps = inputs(stamp=now)
        values[0] += noise[index % len(noise)]
        response.observe(values, mask, stamps, IDENTITY, now)
    values, mask, _ = inputs()
    values[:2] += 8
    frame = response.transform(values, mask, IDENTITY, 4 * SECOND)
    assert frame["values"][0] == 0  # This cell had substantially more idle noise.
    assert frame["values"][1] > 0  # A quiet cell still detects the same small change.


def test_hysteresis_prevents_repeated_contact_toggle_at_activation_threshold():
    response = reference()
    assert response.status.active_cells == 0
    assert render(response, 1002.)["values"] == [0.] * 400
    assert all(value > 0 for value in render(response, 1004.)["values"])
    assert all(value > 0 for value in render(response, 1002.)["values"])
    assert response.status.active_cells == 400
    assert render(response, 1001.)["values"] == [0.] * 400
    assert render(response, 1002.)["values"] == [0.] * 400
    assert response.status.active_cells == 0


@pytest.mark.parametrize("direction,positive,negative", [
    ("both", True, True), ("increase", True, False), ("decrease", False, True),
])
def test_signal_polarity_is_explicit_and_not_pressure(direction, positive, negative):
    response = reference(settings=ResponseSettings(direction=direction))
    assert bool(render(response, 1100.)["values"][0]) is positive
    assert bool(render(response, 900.)["values"][0]) is negative
    assert render(response)["values"] == [0.] * 400


def test_settings_change_keeps_reference_and_does_not_derive_scale_from_data():
    response = reference()
    render(response, 1100.)
    response.configure(direction="decrease", display_range_adc=500.)
    assert response.status.phase == "ready"
    assert response.status.reference_cells == 400
    assert response.status.active_cells == 0
    frame = render(response, 900.)
    assert frame["levels"] == [0., 500.]
    assert frame["values"][0] > 0
    assert render(response, 60000.)["levels"] == [0., 500.]
    previous = response.settings
    with pytest.raises(ValueError):
        response.configure(direction="invalid", display_range_adc=200.)
    assert response.settings == previous
    with pytest.raises(FrozenInstanceError):
        response.settings.direction = "both"


def test_only_independently_sampled_cells_receive_a_reference():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    for index in range(1, 31):
        now = index * SECOND // 10
        values, mask, stamps = inputs(stamp=now)
        stamps[0] = SECOND // 10  # Repeated rolling cache, never a new sample.
        mask[1] = False
        response.observe(values, mask, stamps, IDENTITY, now)
    assert response.status.phase == "ready"
    assert response.status.reference_cells == 398
    frame = render(response)
    assert frame["valid_mask"][:3] == [False, False, True]


def test_samples_cached_before_start_do_not_count_and_polled_frames_do_not_count():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, SECOND)
    for index in range(1, 31):
        now = SECOND + index * SECOND // 10
        values, mask, stamps = inputs(stamp=SECOND)
        response.observe(values, mask, stamps, IDENTITY, now)
        response.transform(values, mask, IDENTITY, now)
    assert response.status.phase == "failed"
    assert response.status.reference_cells == 0
    assert response.status.reason == "insufficient_samples"


def test_future_and_stale_source_samples_cannot_build_a_reference():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, SECOND)
    for index in range(1, 31):
        now = SECOND + index * SECOND // 10
        values, mask, stamps = inputs(stamp=now)
        stamps[:100] = now + 1
        stamps[100:200] = now - SECOND
        response.observe(values, mask, stamps, IDENTITY, now)
    assert response.status.reference_cells == 200
    assert render(response, now=5 * SECOND)["valid_mask"] == [False] * 200 + [True] * 200


def test_reference_requires_fresh_per_cell_data_at_completion():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    for index in range(1, 21):
        now = index * SECOND // 10
        response.observe(*inputs(stamp=now), IDENTITY, now)
    response.poll(3 * SECOND)
    assert response.status.phase == "failed"
    assert response.status.reason == "stale_reference"


def test_delayed_new_sample_cannot_retroactively_rescue_expired_reference():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    for index in range(1, 21):
        now = index * SECOND // 10
        response.observe(*inputs(stamp=now), IDENTITY, now)
    response.observe(*inputs(stamp=4 * SECOND), IDENTITY, 4 * SECOND)
    assert response.status.phase == "failed"
    assert response.status.reference_cells == 0


def test_invalid_cells_release_active_state_before_reappearing():
    response = reference()
    assert render(response, 1004.)["values"][0] > 0
    assert not any(render(response, 1004., cells=0)["valid_mask"])
    assert response.status.active_cells == 0
    # Between the off/on thresholds must not resurrect the former contact.
    assert render(response, 1002.)["values"] == [0.] * 400


def test_clearing_activity_after_an_outage_keeps_reference_but_not_hysteresis():
    response = reference()
    assert all(value > 0 for value in render(response, 1004.)["values"])
    settings = response.settings
    response.clear_activity()
    assert response.status.phase == "ready"
    assert response.status.reference_cells == 400
    assert response.status.active_cells == 0
    assert response.settings == settings
    assert render(response, 1002.)["values"] == [0.] * 400
    assert render(response, 1010.)["values"][0] > 0


def test_clearing_activity_does_not_cancel_reference_collection():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    for index in range(1, 31):
        now = index * SECOND // 10
        response.observe(*inputs(stamp=now), IDENTITY, now)
        response.clear_activity()
    assert response.status.phase == "ready"
    assert response.status.reference_cells == 400


def test_identity_change_invalidates_reference_and_held_activity():
    response = reference()
    render(response, 1100.)
    frame = render(response, 1100., identity=(IDENTITY[0], 2))
    assert frame["values"] == [0.] * 400
    assert response.status.phase == "unreferenced"
    assert response.status.reason == "stream_changed"
    assert response.status.reference_cells == 0


def test_restart_and_clock_reversal_never_reuse_previous_reference():
    response = reference()
    response.begin_reference(IDENTITY, 4 * SECOND)
    assert response.status.reference_cells == 0
    response.poll(3 * SECOND)
    assert response.status.phase == "unreferenced"
    assert response.status.reason == "clock_backwards"
    assert render(response, 1100., now=4 * SECOND)["values"] == [0.] * 400


def test_transport_generation_is_part_of_reference_identity():
    response = TactileResponseProcessor()
    identity = (*IDENTITY, 2, 7)
    response.begin_reference(identity, 0)
    for index in range(1, 31):
        now = index * SECOND // 10
        response.observe(*inputs(stamp=now), identity, now)
    assert response.status.phase == "ready"
    frame = render(response, 1100., identity=(*IDENTITY, 2, 8))
    assert frame["reference_ready"] is False
    assert response.status.reason == "stream_changed"


def test_cancel_only_affects_pending_reference():
    response = reference()
    response.cancel_reference("source_interrupted")
    assert response.status.phase == "ready"
    response.begin_reference(IDENTITY, 4 * SECOND)
    response.cancel_reference("source_interrupted")
    assert response.status.phase == "failed"
    assert response.status.reason == "source_interrupted"
    assert response.status.reference_cells == 0
    assert render(response, 1100., now=5 * SECOND)["reference_ready"] is False


def test_high_rate_repeats_cannot_replace_duration_with_sample_count():
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    # One thousand distinct source samples in 1 ms still occupy one time bin.
    for index in range(1, 1001):
        response.observe(*inputs(stamp=index * 1000), IDENTITY, index * 1000)
    response.observe(*inputs(stamp=3 * SECOND), IDENTITY, 3 * SECOND)
    assert response.status.phase == "failed"
    assert response.status.reason == "insufficient_samples"


def test_reference_and_output_do_not_promote_invalid_nan_to_an_active_cell():
    response = reference(cells=399)
    values, mask, _ = inputs(cells=399)
    values[399] = np.nan
    frame = response.transform(values, mask, IDENTITY, 4 * SECOND)
    assert frame["values"] == [0.] * 400
    assert frame["valid_mask"][-1] is False
    assert response.status.active_cells == 0


@pytest.mark.parametrize("changed", ["adc_length", "valid_nan", "mask_length", "mask_value", "stamp_length"])
def test_malformed_measurements_fail_explicitly(changed):
    response = TactileResponseProcessor()
    response.begin_reference(IDENTITY, 0)
    values, mask, stamps = inputs()
    if changed == "adc_length":
        values = values[:-1]
    elif changed == "valid_nan":
        values[0] = np.nan
    elif changed == "mask_length":
        mask = mask[:-1]
    elif changed == "mask_value":
        mask = mask.astype(int)
        mask[0] = 2
    else:
        stamps = stamps[:-1]
    with pytest.raises(ValueError):
        response.observe(values, mask, stamps, IDENTITY, SECOND)


@pytest.mark.parametrize("changes", [
    {"display_range_adc": 0}, {"display_range_adc": -1},
    {"display_range_adc": True}, {"display_range_adc": float("inf")},
    {"display_range_adc": 65536},
    {"direction": "positive"},
])
def test_invalid_settings_are_rejected(changes):
    with pytest.raises(ValueError):
        ResponseSettings(**changes)


def test_reference_round_trip_preserves_response_without_collecting_new_data():
    measured = reference(noise=[-5., -2., 0., 2., 5.])
    profile = ReferenceProfile.from_dict(json.loads(json.dumps(
        measured.export_reference().to_dict(), allow_nan=False)))
    restored = TactileResponseProcessor()
    restored.load_reference(profile, IDENTITY, 4 * SECOND, source="preset")
    assert measured.status.reference_source == "measured"
    assert restored.status.reference_source == "preset"
    assert restored.status.reference_cells == 400
    for index, value in enumerate((1000., 1100., 1100., 1000., 990., 1000.)):
        now = (index + 5) * SECOND
        assert render(restored, value, now=now) == render(measured, value, now=now)


def test_export_and_load_do_not_persist_contact_or_invent_missing_references():
    measured = reference(cells=398)
    render(measured, 1100.)
    profile = measured.export_reference()
    assert profile.valid_mask[-2:] == (False, False)
    assert profile.baseline_adc[-2:] == (0., 0.)
    assert profile.activation_threshold_adc[-2:] == (0., 0.)
    restored = TactileResponseProcessor()
    restored.load_reference(profile, ("new-process", 1), 0)
    assert restored.status.reference_source == "saved"
    assert restored.status.active_cells == 0
    # A delta between release and activation must not retain previous contact.
    frame = render(restored, 1002., identity=("new-process", 1), now=SECOND)
    assert frame["values"] == [0.] * 400
    assert frame["valid_mask"] == [True] * 398 + [False] * 2


def test_profile_owns_immutable_data_and_serialized_values_are_independent():
    baseline = np.full(400, 1000.)
    thresholds = [3.] * 400
    mask = [True] * 400
    profile = ReferenceProfile(baseline, thresholds, mask)
    baseline[0], thresholds[0], mask[0] = 0., 600., False
    document = profile.to_dict()
    document["baseline_adc"][1] = 0.
    document["valid_mask"][1] = False
    assert profile.baseline_adc[:2] == (1000., 1000.)
    assert profile.activation_threshold_adc[0] == 3.
    assert profile.valid_mask[:2] == (True, True)
    with pytest.raises(FrozenInstanceError):
        profile.schema_version = 2
    with pytest.raises(TypeError):
        profile.baseline_adc[0] = 1.


@pytest.mark.parametrize("field,value", [
    ("schema_version", True), ("schema_version", 2),
    ("baseline_adc", [1000.] * 399),
    ("baseline_adc", [True] + [1000.] * 399),
    ("baseline_adc", [float("nan")] + [1000.] * 399),
    ("baseline_adc", [65536.] + [1000.] * 399),
    ("baseline_adc", [10 ** 100] + [1000.] * 399),
    ("baseline_adc", {i: 1000. for i in range(400)}),
    ("activation_threshold_adc", [float("inf")] + [3.] * 399),
    ("activation_threshold_adc", [-1.] + [3.] * 399),
    ("activation_threshold_adc", [2.] + [3.] * 399),
    ("activation_threshold_adc", [1e9] + [3.] * 399),
    ("valid_mask", [True] * 399),
    ("valid_mask", [False] * 400),
    ("valid_mask", [1] * 400),
])
def test_malformed_profiles_cannot_change_a_ready_processor(field, value):
    response = reference()
    before = render(response, 1100.)
    previous = response.status
    document = response.export_reference().to_dict()
    document[field] = value
    with pytest.raises(ValueError):
        profile = ReferenceProfile.from_dict(document)
        response.load_reference(profile, IDENTITY, 4 * SECOND)
    assert response.status == previous
    assert render(response, 1100.) == before


@pytest.mark.parametrize("value", [None, [], {}, {"schema_version": 1}])
def test_profile_requires_a_complete_versioned_document(value):
    with pytest.raises(ValueError):
        ReferenceProfile.from_dict(value)


def test_profile_rejects_unknown_fields_instead_of_ignoring_them():
    document = reference().export_reference().to_dict()
    document["release_ratio"] = .99
    with pytest.raises(ValueError):
        ReferenceProfile.from_dict(document)


@pytest.mark.parametrize("options", [
    {"identity": ("", 1)}, {"identity": ("new", -1)},
    {"now_ns": True}, {"now_ns": -1}, {"now_ns": 3 * SECOND},
    {"source": "unknown"}, {"profile": {}},
])
def test_invalid_load_arguments_leave_reference_and_contact_intact(options):
    response = reference()
    before = render(response, 1100., now=4 * SECOND)
    previous = response.status
    arguments = dict(profile=response.export_reference(), identity=IDENTITY,
                     now_ns=4 * SECOND, source="saved")
    arguments.update(options)
    with pytest.raises(ValueError):
        response.load_reference(**arguments)
    assert response.status == previous
    assert render(response, 1100.) == before


def test_large_measured_noise_threshold_is_not_clipped_by_export_or_load():
    response = reference(noise=[-1000., 64535.])
    profile = response.export_reference()
    assert min(profile.activation_threshold_adc) > 65535.
    restored = TactileResponseProcessor()
    restored.load_reference(ReferenceProfile.from_dict(profile.to_dict()), IDENTITY, 0)
    assert render(restored, 0.) == render(response, 0.)
    assert render(restored, 65535.) == render(response, 65535.)
    assert restored.status.active_cells == 0


def test_loading_is_explicit_and_does_not_survive_unrelated_stream_or_recalibration():
    profile = reference().export_reference()
    response = TactileResponseProcessor()
    response.load_reference(profile, IDENTITY, 0)
    render(response, 1100., now=SECOND, identity=("other", 1))
    assert response.status.phase == "unreferenced"
    assert response.status.reference_source == ""
    with pytest.raises(ValueError):
        response.export_reference()
    response.load_reference(profile, IDENTITY, SECOND, source="preset")
    response.begin_reference(IDENTITY, 2 * SECOND)
    assert response.status.reference_source == ""
    with pytest.raises(ValueError):
        response.export_reference()
    response.cancel_reference()
    with pytest.raises(ValueError):
        response.export_reference()
