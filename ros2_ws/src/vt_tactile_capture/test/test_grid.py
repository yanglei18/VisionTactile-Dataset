"""Compact physical grid migration never reshapes the wrong 400 wire cells."""
from types import SimpleNamespace

import numpy as np
import pytest

from vt_tactile_capture.grid import normalize_state


def legacy(**changes):
    values = dict(schema_version=1, rows=80, cols=80,
                  adc=np.arange(6400, dtype=np.uint16),
                  valid_mask=np.ones(6400, dtype=bool),
                  updated_mask=np.zeros(6400, dtype=bool),
                  block_monotonic_ns=np.arange(640, dtype=np.uint64),
                  block_realtime_ns=np.arange(640, dtype=np.uint64) + 1234,
                  host_realtime_ns=9988, host_monotonic_ns=100,
                  snapshot_monotonic_ns=150, source_sequence=12,
                  transport_reset_generation=3, transport_health_only=False)
    values.update(changes)
    return SimpleNamespace(**values)


def test_legacy_conversion_is_row_column_crop_not_flat_prefix_and_keeps_clocks():
    source = legacy()
    result = normalize_state(source)
    assert (result.schema_version, result.rows, result.cols) == (2, 20, 20)
    np.testing.assert_array_equal(result.adc.reshape(20, 20),
                                  source.adc.reshape(80, 80)[:20, :20])
    assert result.adc[20] == 80 and result.adc[-1] == 1539
    np.testing.assert_array_equal(result.block_monotonic_ns, source.block_monotonic_ns[:160])
    np.testing.assert_array_equal(result.block_realtime_ns, source.block_realtime_ns[:160])
    assert result.host_realtime_ns == 9988 and result.host_monotonic_ns == 100
    assert result.snapshot_monotonic_ns == 150 and result.source_sequence == 12
    assert result.transport_reset_generation == 3 and result.transport_health_only is False
    result.adc[0] = 99
    result.valid_mask[0] = False
    assert source.adc[0] == 0 and source.valid_mask[0]
    assert normalize_state(result) is result


@pytest.mark.parametrize("changes", [
    dict(schema_version=True), dict(schema_version=1.0), dict(schema_version=3),
    dict(rows=80.0), dict(cols=True), dict(rows=20, cols=20),
    dict(adc=[0] * 400), dict(valid_mask=[True] * 400),
    dict(block_monotonic_ns=[0] * 160),
])
def test_wrong_schema_or_legacy_lengths_rejected(changes):
    with pytest.raises(ValueError):
        normalize_state(legacy(**changes))


def test_legacy_required_missing_array_raises_value_error_not_uncaught_key_error():
    source = legacy()
    del source.adc
    with pytest.raises(ValueError, match="missing adc"):
        normalize_state(source)


def test_baseline_only_legacy_state_does_not_invent_optional_timestamps():
    source = legacy()
    del source.block_monotonic_ns
    del source.block_realtime_ns
    result = normalize_state(source)
    assert not hasattr(result, "block_monotonic_ns")
    assert len(result.adc) == 400
