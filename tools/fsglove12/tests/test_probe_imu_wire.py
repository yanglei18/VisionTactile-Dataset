"""Synthetic direct-wire probe tests; never open a device or pseudo-terminal."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import struct
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "probe_imu_wire.py"
SPEC = importlib.util.spec_from_file_location("probe_imu_wire_script", SCRIPT)
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def wire(first=1, *, zero=False, status=19):
    return b"".join(struct.pack("<BB6h", node, 0 if zero else status,
        *( (0, 0, 0, 0, 0, 0) if zero else (-node, node * 10, -32768, 32767, 0, 8197)))
        for node in range(first, first + 6))


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now


class FakeSerial:
    def __init__(self, clock, chunks, *, interval=.02, error_after=None):
        self.clock, self.chunks, self.interval = clock, list(chunks), interval
        self.calls = 0
        self.requests = []
        self.error_after = error_after
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, amount):
        self.requests.append(amount)
        self.clock.now += self.interval
        if self.error_after is not None and self.calls >= self.error_after:
            raise OSError("synthetic disconnect")
        self.calls += 1
        if not self.chunks:
            return b""
        chunk = self.chunks.pop(0)
        if len(chunk) > amount:
            self.chunks.insert(0, chunk[amount:])
        return chunk[:amount]

    def write(self, *args):
        raise AssertionError("must never write serial")

    def reset_input_buffer(self):
        raise AssertionError("must never reset input")

    def reset_output_buffer(self):
        raise AssertionError("must never reset output")

    def close(self):
        self.closed = True


def _run(tmp_path, chunks, **kwargs):
    clock = Clock()
    serial = FakeSerial(clock, chunks, interval=kwargs.pop("interval", .02),
                        error_after=kwargs.pop("error_after", None))
    report = probe.run_probe("EXPLICIT_FAKE_PORT", tmp_path / "report.json", duration_s=kwargs.pop("duration_s", .2),
        opener=lambda port, baud: serial, closer=lambda opened: opened.close(), clock=clock, **kwargs)
    return report, serial


def test_independent_decoder_all_chunk_splits_signed_values_and_zero_bank():
    data = wire(1, zero=True) + wire(7)
    for split in range(len(data) + 1):
        decoder = probe.WireAudit()
        decoder.feed(data[:split], 1.)
        decoder.feed(data[split:], 1.1)
        report = decoder.report(.1)
        assert report["frames_decoded"] == 2 and report["samples_decoded"] == 12
        assert report["bytes_discarded_during_framing"] == report["unparsed_tail_bytes"] == 0
        assert report["bank_frames"] == {"1-6": 1, "7-12": 1}
        assert bytes.fromhex(report["first_frame_evidence"]["1-6"]["hex"]) == wire(1, zero=True)
        assert report["first_frame_evidence"]["7-12"]["wire_offset"] == 84
        for node in range(1, 13):
            stats = report["nodes"][str(node)]
            assert stats["samples"] == 1
            assert stats["all_samples_zero"] is (node <= 6)
            assert stats["last_raw"] == ([0] * 6 if node <= 6 else [-node, node * 10, -32768, 32767, 0, 8197])
        assert report["status_byte_semantics"] == "unknown" and report["device_dropped_bytes"] is None
        assert "not_verified" in report["acceleration_scale"]["basis"]


def test_resynchronization_reports_only_framing_discards_and_keeps_incomplete_tail():
    malformed = bytearray(wire())
    malformed[14] = 99
    noise = b"garbage\xff" + bytes(malformed)
    decoder = probe.WireAudit()
    decoder.feed(noise + wire(7) + wire(1)[:27], 1.)
    report = decoder.report(.1)
    assert report["frames_decoded"] == 1
    assert report["bytes_discarded_during_framing"] == len(noise)
    assert report["unparsed_tail_bytes"] == 27
    assert report["first_frame_evidence"]["7-12"]["wire_offset"] == len(noise)
    assert report["device_dropped_bytes"] is None


def test_exact_wire_saved_before_decode_with_bounded_hex_and_arrival_rates(tmp_path):
    frames = [wire(1, zero=True) + wire(7, status=index) for index in range(4)]
    report, serial = _run(tmp_path, frames, interval=.1, duration_s=.4)
    raw = (tmp_path / "report.json.wire.bin").read_bytes()
    assert raw == b"".join(frames)
    assert report["wire_sha256"] == hashlib.sha256(raw).hexdigest()
    assert report["wire_complete"] and report["wire_bytes_saved"] == len(raw)
    assert report["bytes_received"] == len(raw) and report["initial_in_waiting"] == 168
    assert report["max_in_waiting"] == 168
    assert report["status"] == "duration_limit" and serial.closed
    assert report["nodes"]["1"]["observed_receive_rate_hz"] == pytest.approx(10.)
    assert report["nodes"]["1"]["all_zero_raw_samples"] == 4
    assert report["nodes"]["7"]["unique_raw_payloads"] == 1
    assert report["nodes"]["7"]["unique_status_values"] == 4
    assert report["nodes"]["7"]["consecutive_identical_payloads"] == 3
    assert sum(len(item["hex"]) for item in report["first_frame_evidence"].values()) == 336
    assert json.loads((tmp_path / "report.json").read_text()) == report
    assert report["serial_write_calls"] == report["input_buffer_resets"] == 0


def test_idle_reads_and_duration_bound_close_without_reconnect(tmp_path):
    report, serial = _run(tmp_path, [], duration_s=.08)
    assert report["status"] == "duration_limit" and serial.closed
    assert .08 <= report["elapsed_read_window_s"] <= .101
    assert report["empty_reads"] == report["read_calls"]
    assert report["empty_read_wait_s"] == report["initially_empty_read_wait_s"]
    assert report["empty_read_wait_s"] == pytest.approx(report["elapsed_read_window_s"])
    assert report["bytes_received"] == 0 and report["frames_decoded"] == 0
    assert all(value["all_samples_zero"] is None for value in report["nodes"].values())
    assert all(value["observed_receive_rate_hz"] is None for value in report["nodes"].values())
    assert set(serial.requests) == {1}


def test_byte_limit_is_exact_and_oversized_read_requests_are_never_used(tmp_path):
    data = wire() * 100
    report, serial = _run(tmp_path, [data], max_bytes=4200)
    assert report["status"] == "byte_limit"
    assert report["bytes_received"] == 4200
    assert (tmp_path / "report.json.wire.bin").read_bytes() == data[:4200]
    assert serial.requests == [4096, 104] and report["frames_decoded"] == 50


def test_sample_limit_stops_on_whole_frames_preserving_unparsed_wire(tmp_path):
    data = wire() * 4
    report, serial = _run(tmp_path, [data], max_samples=13)
    assert report["status"] == "sample_limit" and report["samples_decoded"] == 12
    assert report["unparsed_tail_bytes"] == 168
    assert (tmp_path / "report.json.wire.bin").read_bytes() == data
    assert serial.closed


def test_read_error_preserves_received_evidence_and_explicit_failure_status(tmp_path):
    report, serial = _run(tmp_path, [wire(7)], error_after=1)
    assert report["status"] == "read_error" and serial.closed
    assert report["frames_decoded"] == 1
    assert "synthetic disconnect" in report["errors"][0]
    assert (tmp_path / "report.json.wire.bin").read_bytes() == wire(7)
    assert json.loads((tmp_path / "report.json").read_text())["status"] == "read_error"


def test_existing_evidence_is_not_overwritten_and_never_opens_port(tmp_path):
    path = tmp_path / "report.json"
    wire_path = tmp_path / "report.json.wire.bin"
    wire_path.write_bytes(b"old evidence")
    with pytest.raises(FileExistsError):
        probe.run_probe("NO_OPEN", path, opener=lambda *args: pytest.fail("opened"))
    assert wire_path.read_bytes() == b"old evidence" and not path.exists()


@pytest.mark.parametrize("changes", [dict(duration_s=31), dict(duration_s=0), dict(duration_s=float("nan")),
    dict(max_bytes=probe.MAX_BYTES + 1), dict(max_bytes=0), dict(max_samples=5),
    dict(max_samples=probe.MAX_SAMPLES + 1), dict(accel_g_per_lsb=0), dict(baud=True)])
def test_invalid_bounds_never_open_or_create_outputs(tmp_path, changes):
    with pytest.raises(ValueError):
        probe.run_probe("NO_OPEN", tmp_path / "new.json", opener=lambda *args: pytest.fail("opened"), **changes)
    assert list(tmp_path.iterdir()) == []


def _fake_serial_backend(monkeypatch, *, holders=None, claim=True):
    from vt_fsglove12 import live_stream
    instances, released = [], []

    class Backend:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.closed = False
            instances.append(self)

        def open(self):
            # Mirror pyserial's implicit open hooks, whose originals must not
            # run in this probe (even though ordinary pyserial calls them).
            self._reset_input_buffer()
            self._update_dtr_state()
            self._update_rts_state()

        def _reset_input_buffer(self):
            raise AssertionError("default destructive input flush executed")

        def _update_dtr_state(self):
            raise AssertionError("explicit DTR update executed")

        def _update_rts_state(self):
            raise AssertionError("explicit RTS update executed")

        def fileno(self):
            return 123

        def close(self):
            self.closed = True

    monkeypatch.setattr(live_stream, "_serial_module", lambda: SimpleNamespace(Serial=Backend))
    monkeypatch.setattr(live_stream, "_find_serial_port_holders", holders or (lambda port: []))
    monkeypatch.setattr(live_stream, "_claim_tty_exclusive", lambda opened: claim)
    monkeypatch.setattr(live_stream, "_release_tty_exclusive", lambda opened: released.append(opened))
    return instances, released


def test_open_reuses_exclusion_but_suppresses_pyserial_flush_and_line_updates(monkeypatch):
    instances, released = _fake_serial_backend(monkeypatch)
    serial = probe._open_serial("EXPLICIT_FAKE", 115200)
    assert len(instances) == 1
    assert serial.kwargs["port"] is None and serial.port == "EXPLICIT_FAKE"
    assert serial.kwargs["exclusive"] is True and serial.kwargs["timeout"] == .020
    for name, args in (("write", (b"not sent",)), ("reset_input_buffer", ()), ("reset_output_buffer", ())):
        with pytest.raises(RuntimeError, match="forbids"):
            getattr(serial, name)(*args)
    probe._close_serial(serial)
    assert serial.closed and released == [serial]


def test_visible_existing_reader_is_rejected_before_open(monkeypatch):
    instances, _ = _fake_serial_backend(monkeypatch, holders=lambda port: [(4321, 7)])
    with pytest.raises(RuntimeError, match="already held"):
        probe._open_serial("EXPLICIT_FAKE", 115200)
    assert instances == []


def test_reader_race_after_open_refuses_reads_releases_and_closes(monkeypatch):
    calls = []

    def holders(port):
        calls.append(port)
        return [] if len(calls) == 1 else [(os.getpid(), 123), (4321, 7)]

    instances, released = _fake_serial_backend(monkeypatch, holders=holders)
    with pytest.raises(RuntimeError, match="appeared during open"):
        probe._open_serial("EXPLICIT_FAKE", 115200)
    assert instances[0].closed and released == instances


def test_missing_tiocexcl_fails_closed(monkeypatch):
    instances, released = _fake_serial_backend(monkeypatch, claim=False)
    with pytest.raises(RuntimeError, match="TIOCEXCL unavailable"):
        probe._open_serial("EXPLICIT_FAKE", 115200)
    assert instances[0].closed and released == []


def test_cli_requires_explicit_connect_before_any_output(tmp_path, monkeypatch):
    monkeypatch.setattr(probe, "_open_serial", lambda *args: pytest.fail("opened"))
    with pytest.raises(SystemExit) as exc:
        probe.main(["--port", "NO_OPEN", "--report", str(tmp_path / "new.json")])
    assert exc.value.code == 2 and list(tmp_path.iterdir()) == []


def test_open_error_is_recorded_without_claiming_received_data(tmp_path):
    def failure(*args):
        raise RuntimeError("occupied port, close GUI")

    report = probe.run_probe("NO_OPEN", tmp_path / "report.json", opener=failure)
    assert report["status"] == "open_error" and not report["serial_opened"]
    assert report["bytes_received"] == report["frames_decoded"] == 0
    assert report["errors"] == ["RuntimeError: occupied port, close GUI"]
    assert (tmp_path / "report.json.wire.bin").read_bytes() == b""
