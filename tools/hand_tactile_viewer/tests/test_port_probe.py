"""Protocol evidence, bounded reads and ownership cleanup without hardware."""
from dataclasses import FrozenInstanceError
import errno
import random
import struct
import threading

import pytest

from vt_hand_tactile_viewer.port_probe import (
    MAX_BYTES, MAX_READS, PROBE_SECONDS, READ_TIMEOUT, ProbeResult, probe_port,
)


def imu_frame(first):
    return b"".join(struct.pack("<BB6h", node, 0, 20, -40, 70, 100, 200, 4096)
                    for node in range(first, first + 6))


IMU = (imu_frame(1) + imu_frame(7)) * 3


def touch_block(row):
    return b"\xa5\x5a" + struct.pack(">HBB10H", row, 0, 0, *range(100, 110))


TOUCH = b"".join(touch_block(row) for row in range(12))


class Clock:
    def __init__(self):
        self.value = 0.

    def __call__(self):
        return self.value


class Device:
    def __init__(self, chunks, clock, *, tick=.05, on_read=None, on_close=None):
        self.chunks = list(chunks)
        self.clock, self.tick = clock, tick
        self.on_read, self.on_close = on_read, on_close
        self.requests = []
        self.returned = 0
        self.closed = False

    def read(self, size):
        assert not self.closed
        self.requests.append(size)
        self.clock.value += self.tick
        if self.on_read:
            self.on_read()
        chunk = self.chunks.pop(0) if self.chunks else b""
        if isinstance(chunk, Exception):
            raise chunk
        if len(chunk) > size:
            self.chunks.insert(0, chunk[size:])
            chunk = chunk[:size]
        self.returned += len(chunk)
        return chunk

    def close(self):
        self.closed = True
        if self.on_close:
            self.on_close()


def run(chunks, **device_options):
    clock = Clock()
    device = Device(chunks, clock, **device_options)
    calls = []

    def open_device(*args, **kwargs):
        calls.append((args, kwargs))
        return device

    result = probe_port("COM4", opener=open_device, closer=lambda item: item.close(), clock=clock)
    assert device.closed
    assert calls == [(('COM4', 115200), dict(read_timeout=READ_TIMEOUT, passive=True))]
    return result, device, clock


@pytest.mark.parametrize("payload,role", [(IMU, "imu"), (TOUCH, "tactile")])
def test_repeated_protocol_frames_identify_supported_stream(payload, role):
    result, _, _ = run([payload])
    assert result == ProbeResult(role)
    with pytest.raises(FrozenInstanceError):
        result.role = "wrong"


@pytest.mark.parametrize("payload,role", [(IMU, "imu"), (TOUCH, "tactile")])
def test_fragmented_frames_and_leading_noise_resynchronize(payload, role):
    stream = b"\xff\x00\xa5\x00" + payload
    result, _, _ = run([stream[i:i + 7] for i in range(0, len(stream), 7)], tick=.001)
    assert result.role == role


@pytest.mark.parametrize("payload", [
    b"", imu_frame(1), imu_frame(1) + imu_frame(7), imu_frame(1) * 6,
    touch_block(0), touch_block(0) * 12,
    b"".join(touch_block(row) for row in range(7)),
    random.Random(331).randbytes(32768),
    b"\xa5\x5a\xff\xff\x00\x00" + b"\x00" * 20,
])
def test_silence_noise_invalid_addresses_and_insufficient_evidence_stay_unknown(payload):
    result, _, _ = run([payload])
    assert result.role == "unknown"


def test_both_supported_protocols_are_ambiguous_even_after_early_positive_evidence():
    result, _, _ = run([IMU, b"", TOUCH])
    assert result.role == "ambiguous"


@pytest.mark.parametrize("low_byte", range(8))
@pytest.mark.parametrize("fragment_size", [7, 4096])
def test_44_byte_rows_with_plausible_26_byte_prefixes_remain_unknown(low_byte, fragment_size):
    # A supported packet's column group is at byte 5. In this incompatible
    # 44-byte row format, byte 5 is the first ADC's low byte and can happen to
    # be 0..7. Each row then looks valid to the permissive legacy parser, even
    # though its next header is 44 rather than 26 bytes away.
    from vt_tactile_capture.protocol import Parser
    stream = b"".join(b"\xa5\x5a" + struct.pack(">H20H", row,
                     0x4700 | low_byte, *range(18000, 18019)) for row in range(20))
    parser = Parser()
    assert len(parser.feed(stream)) == 20
    assert parser.discarded_bytes_total == 20 * 18
    stream = b"\xff\x00" + stream
    result, _, _ = run([stream[i:i + fragment_size] for i in range(0, len(stream), fragment_size)], tick=.001)
    assert result.role == "unknown"
    assert "tactile_blocks=20" in result.detail


@pytest.mark.parametrize("frame", [imu_frame(1) + imu_frame(7), TOUCH[:26 * 7]])
def test_nonadjacent_valid_fragments_do_not_accumulate_into_protocol_identity(frame):
    stream = b"\xff".join([frame] * 4)
    result, _, _ = run([stream])
    assert result.role == "unknown"


@pytest.mark.parametrize("payload,role", [(IMU, "imu"), (TOUCH, "tactile")])
def test_boundary_evidence_recovers_after_an_interrupted_run(payload, role):
    stream = payload[:len(payload) // 3] + b"\xff\x00\xff" + payload
    result, _, _ = run([stream[i:i + 11] for i in range(0, len(stream), 11)], tick=.001)
    assert result.role == role


def test_two_protocols_remain_ambiguous_when_tactile_is_seen_first():
    result, _, _ = run([TOUCH, b"\xff\xff", IMU])
    assert result.role == "ambiguous"


def test_silent_stream_read_time_is_bounded_and_device_closes():
    result, device, clock = run([])
    assert result == ProbeResult("unknown", "bytes_received=0; imu_frames=0; tactile_blocks=0")
    assert PROBE_SECONDS <= clock.value <= PROBE_SECONDS + READ_TIMEOUT
    assert len(device.requests) <= MAX_READS


def test_unrecognized_bytes_report_evidence_counts_without_claiming_a_protocol():
    payload = b"\xff" * 4096
    result, device, _ = run([payload])
    assert result == ProbeResult("unknown", "bytes_received=4096; imu_frames=0; tactile_blocks=0")
    assert device.returned == len(payload)


def test_read_count_limits_empty_nonblocking_transport():
    result, device, _ = run([], tick=0.)
    assert result.role == "unknown"
    assert len(device.requests) == MAX_READS


def test_byte_budget_bounds_fast_transport():
    result, device, _ = run([b"\xff" * (MAX_BYTES * 2)], tick=0.)
    assert result.role == "unknown"
    assert device.returned == MAX_BYTES
    assert max(device.requests) <= 4096


def test_next_baudrate_uses_fresh_evidence_and_closes_each_attempt():
    clock = Clock()
    first = Device([imu_frame(1) + imu_frame(7)], clock)
    second = Device([imu_frame(1) + imu_frame(7)], clock)
    devices, calls = [first, second], []

    def opener(port, baudrate, **kwargs):
        calls.append(baudrate)
        return devices.pop(0)

    result = probe_port("COM4", baudrates=(9600, 115200), opener=opener,
                        closer=lambda item: item.close(), clock=clock)
    assert result.role == "unknown"  # four frames across two speeds must not be combined
    assert calls == [9600, 115200]
    assert first.closed and second.closed


def test_supported_second_baudrate_is_detected_without_duplicate_attempts():
    clock = Clock()
    devices = [Device([b"\xff" * 200], clock), Device([TOUCH], clock)]
    calls = []

    def opener(port, baudrate, **kwargs):
        calls.append(baudrate)
        return devices.pop(0)

    result = probe_port("COM4", baudrates=(9600, 9600, 115200), opener=opener,
                        closer=lambda item: item.close(), clock=clock)
    assert result.role == "tactile"
    assert calls == [9600, 115200]


def test_confirmed_first_baudrate_does_not_reopen_device():
    clock, calls = Clock(), []
    device = Device([IMU], clock)

    def opener(*args, **kwargs):
        calls.append(args)
        return device

    result = probe_port("COM4", baudrates=(115200, 9600), opener=opener,
                        closer=lambda item: item.close(), clock=clock)
    assert result.role == "imu" and len(calls) == 1


@pytest.mark.parametrize("exc,role", [
    (OSError(errno.EBUSY, "already in use"), "busy"),
    (OSError(errno.EAGAIN, "Could not exclusively lock port"), "busy"),
    (PermissionError(errno.EACCES, "Access is denied"), "error"),
    (FileNotFoundError(errno.ENOENT, "device removed"), "error"),
])
def test_open_failure_reports_ownership_or_error_without_trying_another_speed(exc, role):
    calls = []

    def opener(*args, **kwargs):
        calls.append(args)
        raise exc

    def closer(device):
        pytest.fail("No device was opened")

    result = probe_port("COM4", baudrates=(115200, 9600), opener=opener, closer=closer)
    assert result.role == role and result.detail
    assert len(calls) == 1


def test_disconnect_read_error_discards_prior_positive_evidence_and_closes():
    result, device, _ = run([IMU, OSError("device removed")])
    assert result.role == "error"
    assert device.closed


def test_close_failure_is_reported_instead_of_claiming_success():
    def close_error():
        raise OSError("could not close device")

    result, _, _ = run([IMU], on_close=close_error)
    assert result.role == "error"


@pytest.mark.parametrize("phase", ["before", "open", "read", "close"])
def test_cancellation_never_returns_late_positive_result_or_leaks_open_device(phase):
    cancel, clock, calls = threading.Event(), Clock(), []
    if phase == "before":
        cancel.set()
    device = Device([IMU], clock,
                    on_read=cancel.set if phase == "read" else None,
                    on_close=cancel.set if phase == "close" else None)

    def opener(*args, **kwargs):
        calls.append(args)
        if phase == "open":
            cancel.set()
        return device

    result = probe_port("COM4", baudrates=(115200, 9600), cancel=cancel,
                        opener=opener, closer=lambda item: item.close(), clock=clock)
    assert result == ProbeResult("unknown", "cancelled")
    if phase == "before":
        assert not calls and not device.closed
    else:
        assert len(calls) == 1 and device.closed


@pytest.mark.parametrize("rates", [(), (True,), (0,), (-1,), (1.5,), (4_000_001,), (9600, 57600, 115200)])
def test_invalid_baudrate_requests_do_not_open_any_device(rates):
    def opener(*args, **kwargs):
        pytest.fail("Invalid request must not open serial")

    with pytest.raises(ValueError):
        probe_port("COM4", baudrates=rates, opener=opener)


@pytest.mark.parametrize("port", ["", "  ", None])
def test_empty_port_is_rejected(port):
    with pytest.raises(ValueError):
        probe_port(port)
