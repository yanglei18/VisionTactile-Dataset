import json
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest

from vt_tactile_capture.display_baseline import DisplayBaseline


IDENTITY = ("tactile-session", 1)


def state(sequence=1, **changes):
    message = SimpleNamespace(
        schema_version=2, rows=20, cols=20, adc=[18000] * 400,
        valid_mask=[True] * 400, connected=True, time_valid=True,
        session_id=IDENTITY[0], stream_epoch=IDENTITY[1], source_sequence=sequence,
    )
    for key, value in changes.items():
        setattr(message, key, value)
    return message


def completed():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 31):
        baseline.observe(state(sequence), sequence * 100_000_000)
    assert baseline.values is not None
    return baseline


def archive(tmp_path, *, identity=IDENTITY, frames=12):
    adc = np.full((frames, 20, 20), 18000, dtype=np.uint16)
    valid = np.ones(adc.shape, dtype=bool)
    adc[:, 0, 0] = 0
    valid[:, 0, 1] = False
    path = tmp_path / "baseline.npz"
    # The loader must not access this object array; allow_pickle=False protects it.
    np.savez_compressed(path, adc=adc, valid_mask=valid, unrelated=np.array([{}], dtype=object))
    path.with_suffix(".json").write_text(json.dumps({
        "session_epoch_frame_counts": {f"{identity[0]}:{identity[1]}": frames},
    }))
    return path


def test_three_second_reference_and_signed_delta_without_mutating_input():
    baseline = completed()
    assert not baseline.collecting
    assert baseline.samples_count == 30
    assert baseline.identity == IDENTITY
    assert baseline.metadata()["valid_cells"] == 400
    values = np.full((20, 20), 17900, dtype=np.uint16)
    result = baseline.subtract(values)
    assert np.all(result == -100)
    assert np.all(values == 17900)
    assert "not pressure" in baseline.status


def test_hand_specific_references_cannot_cross_contaminate_even_with_matching_session(tmp_path):
    left, right = (DisplayBaseline(expected_hand=hand) for hand in ("left", "right"))
    for baseline in (left, right):
        baseline.start(IDENTITY, 0)
    for sequence in range(1, 31):
        left.observe(state(sequence, hand="left", adc=[100] * 400), sequence * 100_000_000)
        right.observe(state(sequence, hand="right", adc=[200] * 400), sequence * 100_000_000)
    assert np.all(left.values == 100) and np.all(right.values == 200)
    left.observe(state(31, hand="right"), 3_100_000_000)
    assert left.values is None and np.all(right.values == 200)
    path = archive(tmp_path)
    metadata = json.loads(path.with_suffix(".json").read_text())
    metadata["hand"] = "right"
    path.with_suffix(".json").write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="Reference hand"):
        left.load(path, IDENTITY)


def test_valid_zero_and_missing_cells_stay_distinct():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 31):
        message = state(sequence)
        message.adc[0] = 0
        message.valid_mask[1] = False
        baseline.observe(message, sequence * 100_000_000)
    assert baseline.values[0, 0] == 0
    assert np.isnan(baseline.values[0, 1])
    values = np.zeros((20, 20))
    values[0, 2] = np.nan
    difference = baseline.subtract(values)
    assert difference[0, 0] == 0
    assert np.isnan(difference[0, 1])
    assert np.isnan(difference[0, 2])
    assert baseline.metadata()["valid_cells"] == 399


def test_repeated_or_reordered_sequences_do_not_become_new_samples():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    baseline.observe(state(3), 100_000_000)
    for sequence in (3, 2, 1, 3):
        baseline.observe(state(sequence), 200_000_000)
    assert baseline.samples_count == 1
    baseline.poll(3_000_000_000)
    assert baseline.values is None
    assert not baseline.collecting
    assert "at least 10" in baseline.status


def test_source_epoch_change_invalidates_even_empty_event():
    baseline = completed()
    baseline.observe(state(31, stream_epoch=2, valid_mask=[False] * 400), 3_100_000_000)
    assert baseline.values is None
    assert baseline.identity is None
    assert "Session/epoch changed" in baseline.status


@pytest.mark.parametrize("changes,reason", [
    ({"connected": False}, "disconnected"),
    ({"time_valid": False}, "invalid source time"),
])
def test_unhealthy_state_cancels_collection_but_retains_completed_reference(changes, reason):
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    baseline.observe(state(**changes), 100_000_000)
    assert not baseline.collecting
    assert baseline.values is None
    assert reason in baseline.status
    baseline = completed()
    previous = baseline.values.copy()
    baseline.observe(state(31, **changes), 3_100_000_000)
    np.testing.assert_array_equal(baseline.values, previous)
    assert "retained" in baseline.status


def test_no_message_timeout_and_stale_last_sample_fail():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    baseline.poll(3_000_000_000)
    assert not baseline.collecting
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 21):
        baseline.observe(state(sequence), sequence * 100_000_000)
    baseline.poll(3_000_000_000)
    assert baseline.values is None
    assert "200 ms" in baseline.status


def test_delayed_message_cannot_rescue_expired_collection():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 21):
        baseline.observe(state(sequence), sequence * 100_000_000)
    baseline.observe(state(100), 20_000_000_000)
    assert baseline.values is None
    assert not baseline.collecting


def test_poll_succeeds_with_recent_sample_and_partial_masks():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 30):
        baseline.observe(state(sequence, valid_mask=[False] + [True] * 399), sequence * 100_000_000)
    assert baseline.collecting
    baseline.poll(3_000_000_000)
    assert baseline.values is not None
    assert np.isnan(baseline.values[0, 0])


def test_empty_masks_never_count_as_valid_samples():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 31):
        baseline.observe(state(sequence, valid_mask=[False] * 400), sequence * 100_000_000)
    assert baseline.values is None
    assert "at least 10" in baseline.status


def test_memory_bounded_and_refresh_clears_previous_reference():
    baseline = completed()
    baseline.start(IDENTITY, 4_000_000_000)
    assert baseline.values is None
    for sequence in range(1, 301):
        baseline.observe(state(sequence), 4_000_000_000 + sequence * 10_000_000)
        assert baseline.samples_count <= 120
    assert baseline.samples_count == 120
    assert baseline.values is not None
    assert not baseline._samples


@pytest.mark.parametrize("changes", [
    {"schema_version": 3}, {"rows": 21}, {"adc": [1]},
    {"valid_mask": [True]}, {"adc": [65536] * 400},
    {"valid_mask": [2] * 400}, {"source_sequence": -1}, {"connected": "yes"},
])
def test_invalid_state_cancels_reference_collection(changes):
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    baseline.observe(state(**changes), 1)
    assert baseline.values is None
    assert not baseline.collecting
    assert "invalid state" in baseline.status


def test_backwards_local_clock_cancels_collection():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 1_000_000_000)
    baseline.observe(state(), 0)
    assert "backwards" in baseline.status


def test_load_same_identity_preserves_valid_zero_missing_and_metadata(tmp_path):
    baseline = DisplayBaseline()
    path = archive(tmp_path)
    baseline.load(path, IDENTITY)
    assert baseline.identity == IDENTITY
    assert baseline.values[0, 0] == 0
    assert np.isnan(baseline.values[0, 1])
    metadata = baseline.metadata()
    assert metadata["samples_count"] == 12
    assert metadata["source"] == str(path)
    assert metadata["identity"] == {"session_id": IDENTITY[0], "stream_epoch": 1}
    assert metadata["valid_cells"] == 399
    json.dumps(metadata)


def test_load_identity_mismatch_clears_old_reference(tmp_path):
    baseline = completed()
    with pytest.raises(ValueError, match="does not match"):
        baseline.load(archive(tmp_path), (IDENTITY[0], 2))
    assert baseline.values is None
    assert "failed" in baseline.status


def test_load_rejects_mixed_identity_or_mismatched_frame_count(tmp_path):
    baseline = DisplayBaseline()
    path = archive(tmp_path)
    path.with_suffix(".json").write_text(json.dumps({
        "session_epoch_frame_counts": {f"{IDENTITY[0]}:1": 6, f"{IDENTITY[0]}:2": 6},
    }))
    with pytest.raises(ValueError, match="exactly one"):
        baseline.load(path, IDENTITY)
    path.with_suffix(".json").write_text(json.dumps({
        "session_epoch_frame_counts": {f"{IDENTITY[0]}:1": 11},
    }))
    with pytest.raises(ValueError, match="shape"):
        baseline.load(path, IDENTITY)


def test_load_oversize_header_rejected_without_loading_array(tmp_path):
    path = tmp_path / "large.npz"
    with zipfile.ZipFile(path, "w") as target:
        with target.open("adc.npy", "w") as member:
            np.lib.format.write_array_header_1_0(member, {
                "descr": "<u2", "fortran_order": False, "shape": (10001, 20, 20),
            })
    path.with_suffix(".json").write_text(json.dumps({
        "session_epoch_frame_counts": {f"{IDENTITY[0]}:1": 10001},
    }))
    with pytest.raises(ValueError, match="10000"):
        DisplayBaseline().load(path, IDENTITY)


def test_load_bad_sidecar_and_empty_reference_give_value_error(tmp_path):
    path = archive(tmp_path)
    path.with_suffix(".json").write_text("[]")
    with pytest.raises(ValueError, match="JSON object"):
        DisplayBaseline().load(path, IDENTITY)
    path = archive(tmp_path)
    np.savez_compressed(path, adc=np.zeros((12, 20, 20), dtype=np.uint16),
                        valid_mask=np.zeros((12, 20, 20), dtype=bool))
    with pytest.raises(ValueError, match="no valid ADC"):
        DisplayBaseline().load(path, IDENTITY)


def test_median_rejects_single_outlier_and_copies_input():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 31):
        message = state(sequence)
        if sequence == 10:
            message.adc[0] = 65000
        baseline.observe(message, sequence * 100_000_000)
        message.adc[0] = 60000
    assert baseline.values[0, 0] == 18000


def test_subtract_requires_reference_and_correct_shape():
    with pytest.raises(ValueError, match="No ADC"):
        DisplayBaseline().subtract(np.zeros((20, 20)))
    with pytest.raises(ValueError, match="shape"):
        completed().subtract(np.zeros((80, 80)))


def test_legacy_80_reference_load_crops_by_coordinates_and_preserves_identity(tmp_path):
    path = tmp_path / "legacy.npz"
    adc = np.broadcast_to(np.arange(6400, dtype=np.uint16).reshape(80, 80), (12, 80, 80)).copy()
    valid = np.ones((12, 80, 80), dtype=bool)
    valid[:, 2, 3] = False
    np.savez_compressed(path, adc=adc, valid_mask=valid)
    path.with_suffix(".json").write_text(json.dumps({
        "session_epoch_frame_counts": {f"{IDENTITY[0]}:1": 12},
    }))
    baseline = DisplayBaseline()
    baseline.load(path, IDENTITY)
    assert baseline.values.shape == (20, 20)
    assert baseline.identity == IDENTITY
    assert baseline.values[0, 0] == 0
    assert baseline.values[1, 0] == 80
    assert baseline.values[19, 19] == 1539
    assert np.isnan(baseline.values[2, 3])
    assert baseline.metadata()["valid_cells"] == 399
    assert "Legacy 80x80" in baseline.status


def test_legacy_80_states_can_collect_reference_without_clock_fields():
    baseline = DisplayBaseline()
    baseline.start(IDENTITY, 0)
    for sequence in range(1, 31):
        message = state(sequence, schema_version=1, rows=80, cols=80,
                        adc=np.arange(6400, dtype=np.uint16), valid_mask=[True] * 6400)
        baseline.observe(message, sequence * 100_000_000)
    assert baseline.values.shape == (20, 20)
    assert baseline.values[1, 0] == 80
    assert baseline.values[19, 19] == 1539


def test_reference_load_rejects_mixed_compact_and_legacy_shapes(tmp_path):
    path = archive(tmp_path)
    np.savez_compressed(path, adc=np.ones((12, 20, 20), dtype=np.uint16),
                        valid_mask=np.ones((12, 80, 80), dtype=bool))
    with pytest.raises(ValueError, match="shapes must match"):
        DisplayBaseline().load(path, IDENTITY)
