"""Validity is independent of ADC magnitude and display refresh frequency."""

from dataclasses import fields, replace
from pathlib import Path

import pytest

from vt_tactile_capture.assembler import Assembler, State
from vt_tactile_capture.protocol import Block
from vt_tactile_capture.serial_source import Batch


def batch(sequence=0, row=2, **kwargs):
    values = dict(session_id="session-a", stream_epoch=1, sequence=sequence,
                  first_packet_index=sequence, host_realtime_ns=1000 + sequence * 10,
                  host_monotonic_ns=100 + sequence * 10, connected=True,
                  time_valid=True, reason="ok", blocks=(Block(row, tuple(range(20))),))
    values.update(kwargs)
    return Batch(**values)


def test_initial_state_has_no_samples():
    state = Assembler().snapshot(100, 50)
    assert not state.connected and not state.time_valid
    assert not any(state.valid_mask) and not any(state.updated_mask)
    assert state.schema_version == 3 and (state.rows, state.cols) == (20, 20)
    assert state.layout_id == "row20_col20_v3"
    assert len(state.adc) == len(state.valid_mask) == len(state.updated_mask) == 400
    assert len(state.row_realtime_ns) == len(state.row_monotonic_ns) == 20
    assert state.host_realtime_ns == state.host_monotonic_ns == 0


def test_mapping_zero_validity_update_consumption_and_stale_stamp():
    assembler = Assembler()
    assembler.apply(batch())
    state = assembler.snapshot(100, 5)
    indices = list(range(2 * 20, 3 * 20))
    assert [state.adc[index] for index in indices] == list(range(20))
    assert [index for index, valid in enumerate(state.valid_mask) if valid] == indices
    assert state.valid_mask[indices[0]]  # ADC zero is a measurement.
    assert sum(state.updated_mask) == 20
    assert state.row_realtime_ns[2] == 1000
    assert state.row_monotonic_ns[2] == 100
    refreshed = assembler.snapshot(101, 5)
    assert sum(refreshed.valid_mask) == 20
    assert not any(refreshed.updated_mask)
    stale = assembler.snapshot(106, 5)
    assert not any(stale.valid_mask) and not any(stale.adc)
    assert stale.reason == "stale"
    assert stale.time_valid  # The clock is valid, but these samples are too old.
    assert stale.host_realtime_ns == state.host_realtime_ns
    assert stale.host_monotonic_ns == state.host_monotonic_ns
    assert stale.snapshot_monotonic_ns == 106


def test_rows_have_independent_age_and_last_update_wins():
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(batch(1, row=4))
    state = assembler.snapshot(115, 10)
    assert sum(state.valid_mask) == 20
    assert state.valid_mask[4 * 20 + 4]
    assert not state.valid_mask[2 * 20 + 3]
    repeated = batch(2, row=4, blocks=(Block(4, (9,) * 20),
                                        Block(4, (0,) * 20)))
    assembler.apply(repeated)
    state = assembler.snapshot(120, 10)
    assert state.adc[4 * 20 + 4] == 0
    assert state.valid_mask[4 * 20 + 4]


@pytest.mark.parametrize("changes,reason", [
    ({"sequence": 2, "first_packet_index": 2}, "sequence_gap"),
    ({"first_packet_index": 7}, "packet_index_gap"),
    ({"session_id": "session-b"}, "session_changed"),
    ({"stream_epoch": 2}, "epoch_changed"),
    ({"producer_dropped_total": 1}, "producer_drop"),
    ({"discarded_bytes_total": 1}, "serial_corruption"),
    ({"malformed_packets_total": 1}, "serial_corruption"),
])
def test_discontinuities_clear_old_rows_but_accept_new_measurements(changes, reason):
    assembler = Assembler()
    assembler.apply(batch())
    second = replace(batch(1, row=4), **changes)
    assembler.apply(second)
    state = assembler.snapshot(second.host_monotonic_ns, 100)
    assert sum(state.valid_mask) == 20
    assert not state.valid_mask[2 * 20 + 3]
    assert state.valid_mask[4 * 20 + 3]
    assert state.reason == reason


@pytest.mark.parametrize("changes", [
    {"time_valid": False}, {"host_realtime_ns": 0}, {"host_monotonic_ns": 0},
    {"host_realtime_ns": 999}, {"host_monotonic_ns": 99},
    {"host_realtime_ns": 1000}, {"host_monotonic_ns": 100},
])
def test_invalid_or_backward_clocks_cannot_create_valid_points(changes):
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(replace(batch(1), **changes))
    state = assembler.snapshot(110, 100)
    assert not state.time_valid and not any(state.valid_mask)
    assert not any(state.row_monotonic_ns)
    assembler.apply(batch(2, row=4))
    assert sum(assembler.snapshot(120, 100).valid_mask) == 20


@pytest.mark.parametrize("connected,reason", [(False, "serial_error"), (True, "no_data_timeout")])
def test_empty_faults_invalidate_without_retimestamping(connected, reason):
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(batch(1, blocks=(), connected=connected, time_valid=False, reason=reason))
    state = assembler.snapshot(200, 100)
    assert state.connected == connected
    assert not state.time_valid and not any(state.valid_mask)
    assert not any(state.adc) and not any(state.row_monotonic_ns)
    assert state.host_realtime_ns == 1000 and state.host_monotonic_ns == 100
    assert state.source_sequence == 1
    assert state.reason == reason


def test_snapshot_clock_regression_clears_cache():
    assembler = Assembler()
    assembler.apply(batch())
    assembler.snapshot(110, 100)
    state = assembler.snapshot(109, 100)
    assert not any(state.valid_mask)
    assert state.reason == "snapshot_clock_backwards"
    assert state.host_monotonic_ns == 100


def test_future_sample_is_not_valid():
    assembler = Assembler()
    assembler.apply(batch())
    state = assembler.snapshot(99, 100)
    assert not state.time_valid and not any(state.valid_mask)


def test_bad_block_cannot_partially_apply_batch():
    assembler = Assembler()
    assembler.apply(batch(blocks=(Block(2, (1,) * 20), Block(80, (1,) * 20))))
    state = assembler.snapshot(100, 100)
    assert not state.time_valid and not any(state.valid_mask)
    assert state.reason == "invalid_block"


@pytest.mark.parametrize("changes,reason", [
    ({"schema_version": 1}, "invalid_schema"),
    ({"hand": "left"}, "invalid_hand"),
    ({"hand": "typo"}, "invalid_hand"),
    ({"session_id": ""}, "missing_session"),
    ({"clock_domain": "ros_sim_time"}, "invalid_clock_domain"),
    ({"timing_source": "publish_time"}, "invalid_timing_source"),
    ({"time_valid": 1}, "invalid_flags"),
    ({"sequence": -1}, "invalid_sequence"),
])
def test_contract_errors_cannot_become_valid_samples(changes, reason):
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(replace(batch(1), **changes))
    state = assembler.snapshot(110, 100)
    assert not state.time_valid and not any(state.valid_mask)
    assert state.reason == reason
    assert state.host_realtime_ns == (1010 if "clock_domain" in changes or "timing_source" in changes else 1000)


@pytest.mark.parametrize("changes,reason", [
    ({"sequence": 0}, "nonincreasing_sequence"),
    ({"first_packet_index": 0}, "packet_index_regression"),
    ({"stream_epoch": 0}, "epoch_regression"),
])
def test_old_or_duplicate_packets_are_not_reaccepted_as_new(changes, reason):
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(replace(batch(1), **changes))
    state = assembler.snapshot(110, 100)
    assert not any(state.valid_mask)
    assert state.reason == reason
    assert state.host_realtime_ns == 1000
    # Rejected metadata must not lower any admission high-water marks.
    assembler.apply(batch(2, row=4, host_monotonic_ns=120, host_realtime_ns=1020))
    assert sum(assembler.snapshot(120, 100).valid_mask) == 20


@pytest.mark.parametrize("changes", [{"session_id": "session-b"}, {"stream_epoch": 2}])
def test_new_stream_can_have_lower_monotonic_origin_than_previous_snapshot(changes):
    assembler = Assembler()
    assembler.apply(batch())
    assembler.snapshot(120, 100)
    assembler.apply(batch(1, host_monotonic_ns=10, **changes))
    state = assembler.snapshot(10, 100)
    assert state.time_valid and sum(state.valid_mask) == 20
    assert state.host_monotonic_ns == 10


def test_forward_wallclock_jump_is_invalid_and_tolerance_is_configurable():
    first = batch()
    jumped = batch(1, host_realtime_ns=6_001_010)
    assembler = Assembler()
    assembler.apply(first)
    assembler.apply(jumped)
    state = assembler.snapshot(110, 100)
    assert not state.time_valid and not any(state.valid_mask)
    assert state.reason == "clock_step"
    tolerant = Assembler(max_clock_step_ns=7_000_000)
    tolerant.apply(first)
    tolerant.apply(jumped)
    assert sum(tolerant.snapshot(110, 100).valid_mask) == 20


def test_realtime_highwater_survives_a_new_session_or_epoch():
    for changes in ({"session_id": "new-session"}, {"stream_epoch": 2}):
        assembler = Assembler()
        assembler.apply(batch())
        assembler.apply(batch(1, host_realtime_ns=999, host_monotonic_ns=10, **changes))
        state = assembler.snapshot(10, 100)
        assert not state.time_valid and not any(state.valid_mask)
        assert state.reason == "nonincreasing_realtime"


def test_clock_regression_cannot_lower_realtime_admission_highwater():
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(batch(1, host_realtime_ns=900))
    assembler.apply(batch(2, host_realtime_ns=950))
    assert not any(assembler.snapshot(120, 100).valid_mask)
    assembler.apply(batch(3, host_realtime_ns=1030))
    assert sum(assembler.snapshot(130, 100).valid_mask) == 20


def test_rejected_packet_regression_cannot_lower_packet_admission_highwater():
    assembler = Assembler()
    assembler.apply(batch(5))
    assembler.apply(batch(6, first_packet_index=0))
    assembler.apply(batch(7, first_packet_index=1))
    state = assembler.snapshot(170, 100)
    assert not any(state.valid_mask) and state.reason == "packet_index_regression"


def test_native_ros_numpy_adc_scalars_are_accepted_if_numpy_available():
    numpy = pytest.importorskip("numpy")
    assembler = Assembler()
    assembler.apply(batch(blocks=(Block(2, tuple(numpy.arange(20, dtype=numpy.uint16))),)))
    assert sum(assembler.snapshot(100, 100).valid_mask) == 20


def test_complete_native_scan_preserves_every_adc_and_zero():
    assembler = Assembler()
    blocks = tuple(Block(row, tuple(row * 20 + col for col in range(20))) for row in range(20))
    assembler.apply(batch(blocks=blocks))
    state = assembler.snapshot(100, 100)
    assert sum(state.valid_mask) == sum(state.updated_mask) == 400
    assert list(state.adc) == list(range(400))
    assert state.adc[0] == 0 and state.valid_mask[0]
    assert len(state.row_monotonic_ns) == 20
    assembler.apply(batch(1, first_packet_index=20))
    assert assembler.snapshot(110, 100).reason == "ok"


@pytest.mark.parametrize("row", [20, 79, -1, True])
def test_outside_rows_are_invalid_packets_not_silent_updates(row):
    assembler = Assembler()
    assembler.apply(batch())
    assembler.apply(batch(1, row=row))
    state = assembler.snapshot(110, 100)
    assert not state.time_valid and state.reason == "invalid_block"
    assert not any(state.valid_mask)


@pytest.mark.parametrize("now,age", [(-1, 10), (100, -1)])
def test_negative_age_parameters_rejected(now, age):
    with pytest.raises(ValueError):
        Assembler().snapshot(now, age)


@pytest.mark.parametrize("cls,message", [(Batch, "TactileRow44Array"), (State, "TactileGridState")])
def test_dataclasses_match_authoritative_ros_messages(cls, message):
    message_path = Path(__file__).resolve().parents[2] / "vt_tactile_msgs" / "msg" / f"{message}.msg"
    names = [line.split()[1] for line in message_path.read_text().splitlines()
             if line.strip() and not line.startswith("#")]
    assert [field.name for field in fields(cls)] == [name for name in names if name != "header"]
