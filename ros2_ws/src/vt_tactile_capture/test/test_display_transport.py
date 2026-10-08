import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
from types import SimpleNamespace

import numpy as np
import pytest

from vt_tactile_capture.display_baseline import DisplayBaseline
from vt_tactile_capture.display_transport import (
    LocalTactileReceiver, LocalTactileSender, MAX_DRAIN_PACKETS, MAX_PACKET_BYTES,
    decode_state, default_socket_path, encode_state,
)


def state(sequence=1, **changes):
    message = SimpleNamespace(
        schema_version=2, rows=20, cols=20, stream_epoch=1, hand="right", session_id="synthetic",
        source_sequence=sequence, host_realtime_ns=1_700_000_000_123_456_789,
        host_monotonic_ns=123_456_789_123, snapshot_monotonic_ns=123_456_799_123,
        max_age_ns=100_000_000, producer_dropped_total=7, connected=True, time_valid=True,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive", layout_id="row20_col_group_stride8_v2",
        unit="adc_counts", reason="ok", header=SimpleNamespace(frame_id="right_tactile"),
        adc=np.arange(400, dtype=np.uint16), valid_mask=np.ones(400, dtype=bool),
        updated_mask=np.arange(400) % 3 == 0,
        block_monotonic_ns=np.arange(160, dtype=np.uint64) + 123_450_000_000,
        block_realtime_ns=np.arange(160, dtype=np.uint64) + 1_700_000_000_120_000_000,
    )
    message.valid_mask[1] = False
    for key, value in changes.items():
        setattr(message, key, value)
    return message


@pytest.fixture
def endpoint():
    # pytest's long temporary paths can exceed Unix-domain socket's 107 bytes.
    with tempfile.TemporaryDirectory(prefix="vtt-") as directory:
        yield str(Path(directory) / "right.sock")


def test_roundtrip_preserves_all_values_masks_source_clocks_and_identity():
    message = state()
    message.adc[-1] = 65535
    message.block_monotonic_ns[-1] = np.iinfo(np.uint64).max
    payload = encode_state(message)
    assert len(payload) < MAX_PACKET_BYTES
    decoded = decode_state(payload)
    for field in ("adc", "valid_mask", "updated_mask", "block_monotonic_ns", "block_realtime_ns"):
        np.testing.assert_array_equal(getattr(decoded, field), getattr(message, field))
    for field in ("hand", "session_id", "stream_epoch", "source_sequence", "host_realtime_ns",
                  "host_monotonic_ns", "snapshot_monotonic_ns", "max_age_ns", "clock_domain",
                  "timing_source", "producer_dropped_total", "reason"):
        assert getattr(decoded, field) == getattr(message, field)
    assert decoded.adc[0] == 0 and decoded.valid_mask[0]
    assert not decoded.valid_mask[1]
    assert decoded.header.stamp.sec * 10**9 + decoded.header.stamp.nanosec == message.host_realtime_ns
    assert decoded.header.frame_id == "right_tactile"
    decoded.adc[0] = 500
    assert message.adc[0] == 0


def test_default_endpoints_are_hand_and_namespace_specific():
    assert default_socket_path("right") == default_socket_path()
    assert default_socket_path("left").endswith("/left.sock")
    assert default_socket_path("left", "/gloves/left/tactile/") == default_socket_path("left")
    assert default_socket_path("left", "/operator_a/left/tactile") != default_socket_path("left", "/operator_b/left/tactile")
    with pytest.raises(ValueError):
        default_socket_path("both")
    with pytest.raises(ValueError):
        default_socket_path("left", "relative")


def test_two_private_receivers_keep_histories_and_shutdown_independent(endpoint):
    paths = [str(Path(endpoint).with_name(hand + ".sock")) for hand in ("left", "right")]
    receivers = [LocalTactileReceiver(path) for path in paths]
    senders = [LocalTactileSender(path) for path in paths]
    try:
        for sender, hand, adc in zip(senders, ("left", "right"), (111, 222)):
            assert sender.send(state(hand=hand, adc=np.full(400, adc, dtype=np.uint16)))
        for receiver, hand, adc in zip(receivers, ("left", "right"), (111, 222)):
            message = receiver.receive_latest()
            assert message.hand == hand and np.all(message.adc == adc)
            assert receiver.reset_generation == 1
        receivers[0].close()
        assert Path(paths[1]).is_socket()
        assert senders[1].send(state(sequence=2))
        assert receivers[1].receive_latest().source_sequence == 2
        assert receivers[1].reset_generation == 1
    finally:
        for endpoint in receivers + senders:
            endpoint.close()


def test_payload_is_compatible_with_reference_without_ros():
    baseline = DisplayBaseline()
    baseline.start(("synthetic", 1), 0)
    for sequence in range(1, 31):
        baseline.observe(decode_state(encode_state(state(sequence))), sequence * 100_000_000)
    assert baseline.values[0, 0] == 0
    assert np.isnan(baseline.values[0, 1])
    assert baseline.values[19, 19] == 399


def legacy_state():
    return state(schema_version=1, rows=80, cols=80, layout_id="row80_col_group_stride8_v1",
                 adc=np.arange(6400, dtype=np.uint16),
                 valid_mask=np.arange(6400) % 5 != 0,
                 updated_mask=np.arange(6400) % 3 == 0,
                 block_monotonic_ns=np.arange(640, dtype=np.uint64) + 120_000_000_000,
                 block_realtime_ns=np.arange(640, dtype=np.uint64) + 1_700_000_000_000_000_000)


def legacy_packet(message):
    # Build the released wire-v1 format independently of the new encoder.
    names = ("schema_version", "rows", "cols", "stream_epoch", "hand", "session_id",
             "clock_domain", "timing_source", "connected", "time_valid", "source_sequence",
             "host_realtime_ns", "host_monotonic_ns", "snapshot_monotonic_ns", "max_age_ns",
             "producer_dropped_total", "layout_id", "unit", "reason")
    header = json.dumps(dict({key: getattr(message, key) for key in names},
                             frame_id=message.header.frame_id)).encode()
    return (struct.pack("!4sHH", b"VTTD", 1, len(header)) + header
            + message.adc.astype("<u2").tobytes()
            + np.packbits(message.valid_mask, bitorder="little").tobytes()
            + np.packbits(message.updated_mask, bitorder="little").tobytes()
            + message.block_monotonic_ns.astype("<u8").tobytes()
            + message.block_realtime_ns.astype("<u8").tobytes())


@pytest.mark.parametrize("path", ["legacy_wire", "legacy_ros_to_new_wire"])
def test_legacy_80_state_crops_exact_addresses_masks_and_clocks(path):
    message = legacy_state()
    payload = legacy_packet(message) if path == "legacy_wire" else encode_state(message)
    decoded = decode_state(payload)
    assert (decoded.schema_version, decoded.rows, decoded.cols) == (2, 20, 20)
    assert decoded.layout_id == "row20_col_group_stride8_v2"
    assert decoded.metadata["rows"] == decoded.metadata["cols"] == 20
    assert decoded.host_monotonic_ns == message.host_monotonic_ns
    assert decoded.snapshot_monotonic_ns == message.snapshot_monotonic_ns
    assert decoded.header.frame_id == message.header.frame_id
    for field in ("adc", "valid_mask", "updated_mask"):
        np.testing.assert_array_equal(getattr(decoded, field).reshape(20, 20),
                                      getattr(message, field).reshape(80, 80)[:20, :20])
    for field in ("block_monotonic_ns", "block_realtime_ns"):
        np.testing.assert_array_equal(getattr(decoded, field), getattr(message, field)[:160])
    assert decoded.adc[20] == 80  # Crop by row stride, never take first 400 raw cells.
    decoded.adc[0] = 65535
    assert message.adc[0] == 0
    if path == "legacy_ros_to_new_wire":
        assert struct.unpack("!H", payload[4:6])[0] == 2
        assert len(payload) < 5000


def test_wire_v1_cannot_claim_v2_shape():
    message = legacy_state()
    message.schema_version = 2
    with pytest.raises(ValueError, match="schema"):
        decode_state(legacy_packet(message))


def test_optional_display_fields_have_unambiguous_defaults():
    message = state()
    for field in ("updated_mask", "block_realtime_ns", "producer_dropped_total", "layout_id", "unit", "reason", "header"):
        delattr(message, field)
    decoded = decode_state(encode_state(message))
    assert not decoded.updated_mask.any()
    assert not decoded.block_realtime_ns.any()
    assert decoded.producer_dropped_total == 0
    assert decoded.unit == "adc_counts"


@pytest.mark.parametrize("changes", [
    {"adc": [-1] * 400}, {"adc": [65536] * 400}, {"adc": [0.5] * 400},
    {"adc": [0] * 399}, {"valid_mask": [2] * 400}, {"updated_mask": [False]},
    {"block_monotonic_ns": [0] * 10}, {"block_realtime_ns": [-1] * 160},
    {"source_sequence": True}, {"source_sequence": -1}, {"source_sequence": 1 << 64},
    {"stream_epoch": 1 << 32}, {"connected": 1}, {"time_valid": "true"},
    {"hand": "unknown"}, {"session_id": ""}, {"schema_version": 3}, {"rows": 21},
    {"unit": "pressure"}, {"reason": "x" * 1025}, {"session_id": "\x00"},
])
def test_malformed_fields_cannot_be_encoded(changes):
    with pytest.raises(ValueError):
        encode_state(state(**changes))


@pytest.mark.parametrize("mutate", [
    lambda p: b"", lambda p: p[:-1], lambda p: p + b"x", lambda p: b"BAD!" + p[4:],
    lambda p: p[:4] + b"\x00\x03" + p[6:],
    lambda p: p[:6] + b"\xff\xff" + p[8:],
    lambda p: p[:8] + b"!" + p[9:], lambda p: b"x" * (MAX_PACKET_BYTES + 1),
])
def test_wire_corruption_is_rejected_without_pickle_or_partial_arrays(mutate):
    with pytest.raises(ValueError):
        decode_state(mutate(encode_state(state())))


def test_decoded_header_is_validated_independently():
    payload = encode_state(state())
    size = struct.unpack("!H", payload[6:8])[0]
    header = json.loads(payload[8:8 + size])
    header["time_valid"] = 1
    replacement = json.dumps(header).encode()
    damaged = payload[:6] + struct.pack("!H", len(replacement)) + replacement + payload[8 + size:]
    with pytest.raises(ValueError, match="boolean"):
        decode_state(damaged)


def test_independent_import_has_no_ros_qt_serial_or_pose_dependencies():
    source = """import sys
import vt_tactile_capture.display_transport
assert not any(name in sys.modules for name in ('rclpy','PyQt5','pyqtgraph','serial','torch'))
"""
    result = subprocess.run([sys.executable, "-c", source], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_nonblocking_empty_receiver_and_roundtrip_cleanup(endpoint):
    receiver, sender = LocalTactileReceiver(endpoint), LocalTactileSender(endpoint)
    try:
        assert receiver.receive_latest() is None
        assert os.stat(endpoint).st_mode & 0o777 == 0o600
        assert sender.send(state())
        decoded = receiver.receive_latest()
        assert decoded.source_sequence == 1
        assert decoded.transport_reset_generation == receiver.reset_generation == 1
        assert receiver.received == 1 and receiver.rejected == 0
        assert receiver.receive_latest() is None
    finally:
        sender.close()
        receiver.close()
        receiver.close()
    assert not Path(endpoint).exists()
    assert receiver.receive_latest() is None


def test_latest_only_rejects_duplicate_reverse_and_preserves_reset_boundaries(endpoint):
    receiver, sender = LocalTactileReceiver(endpoint), LocalTactileSender(endpoint)
    try:
        for sequence in (2, 2, 1, 3):
            assert sender.send(state(sequence))
        assert receiver.receive_latest().source_sequence == 3
        assert receiver.received == 2 and receiver.rejected == 2
        assert sender.send(state(1, stream_epoch=2))
        assert sender.send(state(4))  # Return to an old identity remains a reset, never a silent mix.
        message = receiver.receive_latest()
        assert message.stream_epoch == 1 and message.source_sequence == 4
        assert receiver.reset_generation == message.transport_reset_generation == 3
    finally:
        sender.close()
        receiver.close()


def test_intermediate_health_loss_survives_latest_only_drain(endpoint):
    receiver, sender = LocalTactileReceiver(endpoint), LocalTactileSender(endpoint)
    try:
        assert sender.send(state(1))
        assert sender.send(state(2, connected=False))
        assert sender.send(state(3))
        message = receiver.receive_latest()
        assert message.connected
        assert message.transport_health_generation == receiver.health_generation == 1
        assert receiver.reset_generation == 1
    finally:
        sender.close()
        receiver.close()


def test_same_sequence_health_events_preserve_interruption_without_new_sample(endpoint):
    receiver, sender = LocalTactileReceiver(endpoint), LocalTactileSender(endpoint)
    try:
        assert sender.send(state(1))
        assert sender.send(state(1, connected=False))
        assert sender.send(state(1))
        message = receiver.receive_latest()
        assert message.connected and message.source_sequence == 1
        assert message.transport_health_only
        assert receiver.health_generation == message.transport_health_generation == 1
        assert receiver.received == 3 and receiver.rejected == 0
        # Old-source health events cannot invalidate a newer sample.
        assert sender.send(state(2))
        assert sender.send(state(1, connected=False))
        assert receiver.receive_latest().source_sequence == 2
        assert receiver.health_generation == 1 and receiver.rejected == 1
    finally:
        sender.close()
        receiver.close()


def test_oversized_malformed_datagrams_are_counted_and_do_not_kill_receiver(endpoint):
    receiver = LocalTactileReceiver(endpoint)
    sender = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sender.sendto(b"x" * (MAX_PACKET_BYTES + 10), endpoint)
        sender.sendto(b"nonsense", endpoint)
        sender.sendto(encode_state(state()), endpoint)
        assert receiver.receive_latest().source_sequence == 1
        assert receiver.rejected == 2 and receiver.received == 1
    finally:
        sender.close()
        receiver.close()


def test_drain_is_bounded_even_if_socket_is_always_readable(endpoint):
    receiver = LocalTactileReceiver(endpoint)
    real_socket = receiver.socket

    class BusySocket:
        calls = 0

        def recvmsg(self, maximum):
            assert maximum == MAX_PACKET_BYTES + 1
            self.calls += 1
            return encode_state(state(self.calls)), [], 0, None

    busy = BusySocket()
    receiver.socket = busy
    try:
        assert receiver.receive_latest().source_sequence == MAX_DRAIN_PACKETS
        assert busy.calls == MAX_DRAIN_PACKETS
    finally:
        receiver.socket = real_socket
        receiver.close()


def test_gui_absent_full_and_restarted_never_blocks_sender(endpoint):
    sender = LocalTactileSender(endpoint)
    assert not sender.send(state())
    receiver = LocalTactileReceiver(endpoint)
    try:
        dropped = False
        for sequence in range(1, 100):
            if not sender.send(state(sequence)):
                dropped = True
                break
        assert dropped, "Unix datagram queue should be bounded"
        assert sender.dropped >= 2
        assert receiver.receive_latest() is not None
        receiver.close()
        assert not sender.send(state(101))
        receiver = LocalTactileReceiver(endpoint)
        assert sender.send(state(102))
        assert receiver.receive_latest().source_sequence == 102
        sender.close()
        assert not sender.send(state(103))
    finally:
        sender.close()
        receiver.close()


def test_sender_invalid_message_is_counted_not_thrown(endpoint):
    sender = LocalTactileSender(endpoint)
    try:
        assert not sender.send(state(adc=[0]))
        assert sender.rejected == 1 and sender.sent == 0
    finally:
        sender.close()


def test_receiver_does_not_steal_active_lock_or_foreign_listener(endpoint):
    receiver = LocalTactileReceiver(endpoint)
    try:
        with pytest.raises((ValueError, BlockingIOError)):
            LocalTactileReceiver(endpoint)
        assert Path(endpoint).is_socket()
    finally:
        receiver.close()
    original = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    original.bind(endpoint)
    try:
        with pytest.raises(ValueError, match="another receiver"):
            LocalTactileReceiver(endpoint)
        assert Path(endpoint).is_socket()
    finally:
        original.close()


def test_stale_endpoint_can_restart_but_files_and_symlinks_are_untouched(endpoint):
    original = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    original.bind(endpoint)
    original.close()
    receiver = LocalTactileReceiver(endpoint)
    receiver.close()
    Path(endpoint).write_text("keep")
    with pytest.raises(ValueError, match="non-socket"):
        LocalTactileReceiver(endpoint)
    assert Path(endpoint).read_text() == "keep"
    Path(endpoint).unlink()
    target = Path(endpoint).with_suffix(".target")
    target.write_text("keep too")
    Path(endpoint).symlink_to(target)
    with pytest.raises(ValueError, match="non-socket"):
        LocalTactileReceiver(endpoint)
    assert Path(endpoint).is_symlink() and target.read_text() == "keep too"


def test_receiver_close_does_not_unlink_replacement(endpoint):
    receiver = LocalTactileReceiver(endpoint)
    Path(endpoint).unlink()
    Path(endpoint).write_text("replacement")
    receiver.close()
    assert Path(endpoint).read_text() == "replacement"


def test_owned_private_directory_and_absolute_path_required(endpoint):
    with pytest.raises(ValueError, match="absolute"):
        LocalTactileReceiver("relative.sock")
    with pytest.raises(ValueError, match="107"):
        LocalTactileSender("/" + "a" * 108)
    os.chmod(Path(endpoint).parent, 0o755)
    with pytest.raises(ValueError, match="private"):
        LocalTactileReceiver(endpoint)
    with pytest.raises(ValueError, match="private"):
        LocalTactileSender(endpoint)


def test_default_path_is_per_uid():
    assert default_socket_path() == f"/tmp/vt-tactile-{os.getuid()}/right.sock"
