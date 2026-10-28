"""Pure tactile display validation; no Qt, ROS or device access."""
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from vt_capture_console.backend import tactile_source as module


NOW = 10_000_000_000


def state(sequence=1, now=NOW, **changes):
    fields = dict(schema_version=2, hand="right", rows=20, cols=20,
                  session_id="synthetic", stream_epoch=1, source_sequence=sequence,
                  host_realtime_ns=now, host_monotonic_ns=now,
                  snapshot_monotonic_ns=now, max_age_ns=100_000_000,
                  clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                  unit="adc_counts", layout_id="row20_col_group_stride8_v2",
                  connected=True, time_valid=True, reason="synthetic",
                  adc=np.arange(400, dtype=np.uint16), valid_mask=np.ones(400, dtype=bool),
                  updated_mask=np.ones(400, dtype=bool),
                  block_monotonic_ns=np.full(160, now, dtype=np.uint64),
                  block_realtime_ns=np.full(160, now, dtype=np.uint64))
    return SimpleNamespace(**(fields | changes))


def test_zero_adc_is_valid_and_returns_independent_python_lists():
    source = state()
    model = module.TactileDisplayModel()
    model.observe(source, now_ns=NOW)
    result = model.poll(now_ns=NOW)
    assert result.status == "实时" and result.valid_cells == 400
    assert result.frame["values"][0] == 0 and result.frame["valid_mask"][0] is True
    assert isinstance(result.frame["values"], list)
    assert all(type(value) is bool for value in result.frame["valid_mask"])
    source.adc[:] = 999
    result.frame["values"][0] = 666
    assert model.poll(now_ns=NOW).frame["values"][0] == 0


def test_public_timing_keeps_source_provenance_across_repaints_and_is_immutable():
    from dataclasses import FrozenInstanceError

    sample = state(snapshot_monotonic_ns=NOW + 1_000_000)
    model = module.TactileDisplayModel()
    model.observe(sample, now_ns=NOW + 1_000_000)
    first = model.poll(now_ns=NOW + 2_000_000)
    later = model.poll(now_ns=NOW + 20_000_000)
    assert first.timing == later.timing
    assert first.timing.session_id == "synthetic"
    assert first.timing.source_monotonic_ns == NOW
    assert first.timing.snapshot_monotonic_ns == NOW + 1_000_000
    assert first.timing.source_sequence == 1
    assert first.timing.block_monotonic_ns == (NOW,) * 160
    sample.block_monotonic_ns[:] = 0
    assert first.timing.block_monotonic_ns == (NOW,) * 160
    with pytest.raises(FrozenInstanceError):
        first.timing.source_sequence = 2
    assert model.poll(now_ns=NOW + 100_000_001).timing is None


def test_public_timing_exposes_hidden_health_and_reference_boundaries():
    model = module.TactileDisplayModel(response_enabled=True)
    model.observe(state(), now_ns=NOW)
    first = model.poll(now_ns=NOW).timing
    model.invalidate("等待数据", "暂时中断", reset_reference=True)
    model.observe(state(2, NOW + 1), now_ns=NOW + 1)
    recovered = model.poll(now_ns=NOW + 1).timing
    assert recovered.display_generation > first.display_generation
    assert recovered.session_id == first.session_id
    model.start_reference(now_ns=NOW + 2)
    collecting = model.poll(now_ns=NOW + 2).timing
    assert collecting.display_generation > recovered.display_generation
    model.configure_response(display_range_adc=500)
    changed = model.poll(now_ns=NOW + 3).timing
    assert changed.display_generation > collecting.display_generation
    model.configure_response(display_range_adc=500)
    assert model.poll(now_ns=NOW + 4).timing == changed


def test_timing_normalizes_block_storage_and_rejects_invalid_clock_contract():
    blocks = [NOW] * 160
    timing = module.TactileTiming("one", 1, 1, NOW, NOW, blocks, 100_000_000)
    blocks[0] = 0
    assert timing.block_monotonic_ns[0] == NOW
    with pytest.raises(ValueError):
        module.TactileTiming("one", 1, 1, NOW, NOW - 1, blocks, 100_000_000)


def test_cell_freshness_uses_stride8_and_rechecks_without_new_packets():
    sample = state()
    sample.block_monotonic_ns[3 * 8 + 2] = NOW - 100_000_001
    model = module.TactileDisplayModel()
    model.observe(sample, now_ns=NOW)
    result = model.poll(now_ns=NOW)
    assert result.valid_cells == 397
    assert [i for i, valid in enumerate(result.frame["valid_mask"]) if not valid] == [62, 70, 78]
    assert model.poll(now_ns=NOW + 100_000_000).valid_cells == 397
    expired = model.poll(now_ns=NOW + 100_000_001)
    assert expired.frame is None and expired.status == "数据过期"
    assert model.poll(now_ns=NOW + 250_000_001).frame is None


@pytest.mark.parametrize("changes", [
    {"hand": "left"}, {"clock_domain": "CLOCK_REALTIME"}, {"timing_source": "device"},
    {"time_valid": False}, {"time_valid": "true"}, {"connected": 1},
    {"unit": "newton"}, {"session_id": ""}, {"host_monotonic_ns": 0},
    {"host_monotonic_ns": NOW + 1}, {"snapshot_monotonic_ns": NOW + 1},
    {"snapshot_monotonic_ns": NOW - 1}, {"max_age_ns": -1},
    {"adc": [1] * 399}, {"adc": [65536] * 400}, {"valid_mask": [2] * 400},
    {"block_monotonic_ns": [NOW] * 159},
    {"transport_reset_generation": -1}, {"transport_health_generation": "invalid"},
])
def test_bad_input_clears_previously_good_frame(changes):
    model = module.TactileDisplayModel()
    model.observe(state(), now_ns=NOW)
    assert model.poll(now_ns=NOW).frame is not None
    model.observe(state(2, **changes), now_ns=NOW)
    result = model.poll(now_ns=NOW)
    assert result.frame is None and result.status == "数据无效"


def test_disconnect_future_blocks_and_source_expiry_never_reuse_old_data():
    model = module.TactileDisplayModel()
    model.observe(state(), now_ns=NOW)
    model.observe(state(2, connected=False), now_ns=NOW)
    assert model.poll(now_ns=NOW).status == "连接断开"
    model.observe(state(3, block_monotonic_ns=np.full(160, NOW + 1, dtype=np.uint64)), now_ns=NOW)
    assert model.poll(now_ns=NOW).frame is None
    model.observe(state(4, max_age_ns=1_000_000_000), now_ns=NOW)
    assert model.poll(now_ns=NOW + module.MAX_AGE_NS).valid_cells == 400
    assert model.poll(now_ns=NOW + module.MAX_AGE_NS + 1).frame is None
    model.observe(state(5), now_ns=NOW + module.MAX_AGE_NS + 1)
    assert model.poll(now_ns=NOW + module.MAX_AGE_NS + 1).status == "数据过期"


def test_distinct_source_rate_does_not_count_repaints_or_duplicates():
    model = module.TactileDisplayModel()
    model.observe(state(), now_ns=NOW)
    for offset in range(10):
        assert model.poll(now_ns=NOW + offset).receive_hz == 0
    later = NOW + 50_000_000
    model.observe(state(2, later), now_ns=later)
    assert model.poll(now_ns=later).receive_hz == pytest.approx(20.)
    model.observe(state(2, later), now_ns=later + 10_000_000)
    assert model.poll(now_ns=later + 10_000_000).receive_hz == pytest.approx(20.)
    model.observe(state(1, later), now_ns=later + 10_000_000)
    assert model.poll(now_ns=later + 10_000_000).frame is None


def test_identity_and_hidden_health_boundary_reset_rate_and_old_cells():
    model = module.TactileDisplayModel()
    model.observe(state(), now_ns=NOW)
    later = NOW + 50_000_000
    model.observe(state(2, later), now_ns=later)
    assert model.poll(now_ns=later).receive_hz == 20
    mask = np.zeros(400, dtype=bool)
    mask[12] = True
    model.observe(state(1, later, session_id="second", valid_mask=mask), now_ns=later)
    result = model.poll(now_ns=later)
    assert result.valid_cells == 1 and result.receive_hz == 0
    model.observe(state(2, later + 20_000_000, session_id="second", valid_mask=mask), now_ns=later + 20_000_000)
    assert model.poll(now_ns=later + 20_000_000).receive_hz == 50
    model.observe(state(3, later + 30_000_000, session_id="second", valid_mask=mask,
                        transport_health_generation=1), now_ns=later + 30_000_000)
    assert model.poll(now_ns=later + 30_000_000).receive_hz == 0


def test_local_clock_regression_clears_display():
    model = module.TactileDisplayModel()
    model.observe(state(), now_ns=NOW)
    assert model.poll(now_ns=NOW).frame is not None
    assert model.poll(now_ns=NOW - 1).status == "数据无效"


def test_health_only_same_sequence_recovery_is_not_a_new_snapshot():
    model = module.TactileDisplayModel()
    model.observe(state(), now_ns=NOW)
    model.observe(state(2, NOW + 50_000_000), now_ns=NOW + 50_000_000)
    assert model.poll(now_ns=NOW + 50_000_000).receive_hz == 20
    transition = state(2, NOW + 50_000_000, connected=False,
                       transport_health_generation=1, transport_health_only=True)
    model.observe(transition, now_ns=NOW + 60_000_000)
    assert model.poll(now_ns=NOW + 60_000_000).status == "连接断开"
    transition.connected = True
    model.observe(transition, now_ns=NOW + 70_000_000)
    assert model.poll(now_ns=NOW + 70_000_000).frame is None
    assert model.poll(now_ns=NOW + 70_000_000).receive_hz == 0
    for sequence, offset in [(3, 100_000_000), (4, 150_000_000)]:
        model.observe(state(sequence, NOW + offset, transport_health_generation=1), now_ns=NOW + offset)
        assert model.poll(now_ns=NOW + offset).receive_hz == (0 if sequence == 3 else 20)


def test_model_import_has_no_qt_ros_serial_or_pose_side_effects():
    script = """import sys
from vt_capture_console.backend.tactile_source import TactileDisplayModel
for name in ('PySide6', 'rclpy', 'serial', 'torch', 'vt_fsglove12'):
    assert name not in sys.modules, name
assert TactileDisplayModel().poll().frame is None
"""
    result = subprocess.run([sys.executable, "-I", "-c", script],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
