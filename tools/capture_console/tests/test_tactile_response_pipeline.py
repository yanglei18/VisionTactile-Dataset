"""Unloaded tactile display through the real source and shared preview boundaries."""
from types import SimpleNamespace

import numpy as np
import pytest

from vt_capture_console.backend.tactile_source import TactileDisplayModel


NOW = 10_000_000_000


def sample(sequence, stamp, *, values=None, **changes):
    fields = dict(schema_version=2, hand="right", rows=20, cols=20,
                  session_id="response-test", stream_epoch=1, source_sequence=sequence,
                  host_realtime_ns=stamp, host_monotonic_ns=stamp,
                  snapshot_monotonic_ns=stamp, max_age_ns=100_000_000,
                  clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                  unit="adc_counts", layout_id="row20_col_group_stride8_v2",
                  connected=True, time_valid=True, reason="synthetic",
                  adc=np.full(400, 1000, dtype=np.uint16) if values is None else values,
                  valid_mask=np.ones(400, dtype=bool), updated_mask=np.ones(400, dtype=bool),
                  block_monotonic_ns=np.full(160, stamp, dtype=np.uint64),
                  block_realtime_ns=np.full(160, stamp, dtype=np.uint64))
    return SimpleNamespace(**(fields | changes))


def calibrated_model():
    model = TactileDisplayModel(response_enabled=True)
    model.observe(sample(0, NOW), now_ns=NOW)
    model.start_reference(now_ns=NOW)
    for sequence in range(1, 31):
        stamp = NOW + sequence * 100_000_000
        model.observe(sample(sequence, stamp), now_ns=stamp)
        result = model.poll(now_ns=stamp)
    assert result.response.phase == "ready"
    return model


def test_source_does_not_present_initial_offsets_as_touch_or_change_raw_input():
    values = np.arange(400, dtype=np.uint16) + 1000
    message = sample(1, NOW, values=values)
    model = TactileDisplayModel(response_enabled=True)
    raw_model = TactileDisplayModel()
    for target in (model, raw_model):
        target.observe(message, now_ns=NOW)
    response = model.poll(now_ns=NOW)
    assert response.response.phase == "unreferenced"
    assert response.frame["values"] == [0.] * 400
    assert response.frame["levels"] == [0., 1000.]
    assert response.frame["value_kind"] == "tactile_response"
    assert raw_model.poll(now_ns=NOW).frame["values"] == values.tolist()
    np.testing.assert_array_equal(message.adc, np.arange(400) + 1000)


def test_sustained_local_touch_and_release_through_source():
    model = calibrated_model()
    values = np.full(400, 1000, dtype=np.uint16)
    values[12] += 100
    for sequence in range(31, 91):
        stamp = NOW + sequence * 100_000_000
        model.observe(sample(sequence, stamp, values=values), now_ns=stamp)
        result = model.poll(now_ns=stamp)
        assert result.frame["values"][12] > 0
        assert sum(v > 0 for v in result.frame["values"]) == 1
    stamp += 100_000_000
    model.observe(sample(91, stamp), now_ns=stamp)
    assert model.poll(now_ns=stamp).frame["values"] == [0.] * 400


@pytest.mark.parametrize("changes", [{"session_id": "new"}, {"stream_epoch": 2},
                                      {"transport_health_generation": 1}])
def test_new_stream_never_uses_previous_reference(changes):
    model = calibrated_model()
    stamp = NOW + 3_100_000_000
    model.observe(sample(31, stamp, values=np.full(400, 1400), **changes), now_ns=stamp)
    result = model.poll(now_ns=stamp)
    assert result.response.phase == "unreferenced"
    assert result.frame["values"] == [0.] * 400


def test_reference_requires_live_data_and_cannot_finish_from_repaints():
    model = TactileDisplayModel(response_enabled=True)
    with pytest.raises(ValueError):
        model.start_reference(now_ns=NOW)
    model.observe(sample(1, NOW), now_ns=NOW)
    model.start_reference(now_ns=NOW)
    for step in range(1, 32):
        result = model.poll(now_ns=NOW + step * 100_000_000)
    assert result.response.phase == "failed"
    assert result.frame is None


def test_configuring_display_does_not_reacquire_or_alter_reference():
    model = calibrated_model()
    model.configure_response(display_range_adc=200., direction="decrease")
    stamp = NOW + 3_100_000_000
    values = np.full(400, 1000, dtype=np.uint16)
    values[0], values[1] = 900, 1100
    model.observe(sample(31, stamp, values=values), now_ns=stamp)
    result = model.poll(now_ns=stamp)
    assert result.response.phase == "ready"
    assert result.frame["values"][0] > 0 and result.frame["values"][1] == 0
    assert result.frame["levels"] == [0., 200.]


def test_stale_cells_release_activity_without_discarding_completed_reference():
    model = calibrated_model()
    stamp = NOW + 3_100_000_000
    model.observe(sample(31, stamp, values=np.full(400, 1010)), now_ns=stamp)
    assert model.poll(now_ns=stamp).response.active_cells == 400
    missing = model.poll(now_ns=stamp + 100_000_001)
    assert missing.frame is None and missing.response.phase == "ready"
    assert missing.response.active_cells == 0
    # Between the on/off thresholds: a lost contact must not remain latched.
    stamp += 150_000_000
    model.observe(sample(32, stamp, values=np.full(400, 1002)), now_ns=stamp)
    assert model.poll(now_ns=stamp).frame["values"] == [0.] * 400
