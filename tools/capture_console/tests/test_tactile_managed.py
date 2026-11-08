"""Managed tactile presentation shares the real response core, never a serial reader."""
from types import SimpleNamespace

import pytest

from vt_capture_console.backend.tactile_managed import ManagedTactileSource
from vt_tactile_capture.display_transport import LocalTactileSender, default_socket_path, default_workbench_socket


NOW = 10_000_000_000


def sample(index=1, now=NOW, hand="right", value=1000, **changes):
    return SimpleNamespace(**(dict(schema_version=3, hand=hand, rows=20, cols=20,
        session_id="managed-test", stream_epoch=1, source_sequence=index,
        host_realtime_ns=now, host_monotonic_ns=now, snapshot_monotonic_ns=now,
        max_age_ns=100_000_000, clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        unit="adc_counts", layout_id="row20_col20_v3", connected=True, time_valid=True,
        reason="ok", adc=[value] * 400, valid_mask=[True] * 400, updated_mask=[True] * 400,
        row_monotonic_ns=[now] * 20, row_realtime_ns=[now] * 20, producer_dropped_total=0) | changes))


@pytest.mark.parametrize("hand", ["left", "right"])
def test_three_second_shared_reference_does_not_mutate_raw(hand):
    source = ManagedTactileSource(hand=hand)
    first = sample(hand=hand)
    source.observe(first, now_ns=NOW)
    source.start_reference(now_ns=NOW)
    for i in range(1, 31):
        now = NOW + i * 100_000_000
        incoming = sample(i + 1, now, hand)
        source.observe(incoming, now_ns=now)
        source.poll(now_ns=now)
    assert source.response_status.phase == "ready"
    later = NOW + 3_010_000_000
    pressed = sample(32, later, hand, value=1500)
    source.observe(pressed, now_ns=later)
    shown = source.poll(now_ns=later)
    assert source.response_status.active_cells == 400
    assert shown.raw_frame["values"] == [1500] * 400
    assert shown.frame["value_kind"] == "tactile_response"
    assert shown.frame["values"][0] > 0
    assert first.adc == [1000] * 400 and pressed.adc == [1500] * 400
    assert source.poll(now_ns=later + 100_000_001).frame is None


def test_repeated_source_does_not_complete_reference_or_refresh_age():
    source = ManagedTactileSource()
    incoming = sample()
    source.observe(incoming, now_ns=NOW)
    source.start_reference(now_ns=NOW)
    for i in range(31):
        now = NOW + i * 100_000_000
        source.observe(incoming, now_ns=now)
        source.poll(now_ns=now)
    assert source.response_status.phase != "ready"
    assert source.poll(now_ns=NOW + 3_000_000_000).frame is None


def test_callbacks_are_requests_and_only_manager_receipt_confirms_stop():
    calls = []
    source = ManagedTactileSource(request_connect=lambda p, b: calls.append((p, b)),
                                  request_disconnect=lambda: calls.append("stop"))
    source.start()
    source.connect("/configured/port", 115200)
    assert source.connection_state == "connecting" and source.can_disconnect
    source.observe(sample(), now_ns=NOW)
    assert source.is_connected
    source.disconnect()
    assert source.connection_state == "disconnecting"
    source.observe(sample(2, NOW + 1), now_ns=NOW + 1)
    assert source.connection_state == "disconnecting"
    assert source.poll(now_ns=NOW + 1).frame is None
    source.set_connection_state("disconnected")
    assert source.can_connect
    source.close()
    assert calls == [("/configured/port", 115200), "stop"]


def test_read_only_close_never_stops_managed_acquisition():
    calls = []
    source = ManagedTactileSource(request_disconnect=lambda: calls.append("stop"))
    source.observe(sample(), now_ns=NOW)
    assert source.close()
    assert calls == []


def test_full_state_socket_is_private_to_each_hand_and_not_overlay(tmp_path):
    assert default_workbench_socket("right") != default_workbench_socket("left")
    assert default_workbench_socket("right") != default_socket_path("right")
    directory = tmp_path / "private"
    directory.mkdir(mode=0o700)
    path = directory / "right.sock"
    source = ManagedTactileSource(socket_path=path)
    source.start()
    sender = LocalTactileSender(path)
    try:
        assert sender.send(sample())
        result = source.poll(now_ns=NOW)
        assert source.is_connected and result.valid_cells == 400
        assert result.raw_frame["values"] == [1000] * 400
        assert result.timing.row_monotonic_ns == (NOW,) * 20
    finally:
        sender.close()
        source.close()


def test_wrong_hand_and_old_schema_are_not_promoted_to_live():
    source = ManagedTactileSource(hand="left")
    source.observe(vars(sample()), now_ns=NOW)
    assert source.poll(now_ns=NOW).frame is None and not source.is_connected
    source.observe(sample(hand="left", schema_version=2), now_ns=NOW)
    assert source.poll(now_ns=NOW).raw_frame is None
