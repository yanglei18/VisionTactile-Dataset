"""Independent input tests use fake serial/temporary CSV, never hardware."""
import csv
from itertools import groupby
import json
import math
import os
import struct
import threading
from types import SimpleNamespace

import numpy as np
import pytest
from vqf import VQF

from vt_fsglove12.config import load_config
from vt_fsglove12 import live_stream as live
from vt_fsglove12.replay import iter_orientation_frames, replay_raw


def sample(node=1, stamp=1., values=(0, 0, 0, 0, 0, 8197), status=0):
    return live.RawSample(node, values, stamp, 1000. + stamp, status)


def test_independent_input_monitor_does_not_steal_warnings_or_frame_indices():
    fusion = live.OnlineFusion({})
    fusion.update([sample(12, 1.)])
    fusion.reset_for_calibration()
    fusion.update([sample(12, 1.01)])
    monitor = fusion.input_snapshot(1.02)
    assert monitor["frame_index"] == 0
    assert "calibration_reset_filters_reinitialized" in monitor["warnings"]
    assert fusion.input_snapshot(1.03)["frame_index"] == 0
    logged = fusion.snapshot(1.03)
    assert logged["frame_index"] == 0
    assert "calibration_reset_filters_reinitialized" in logged["warnings"]
    assert logged["nodes"]["12"]["sample_index"] == 2
    assert logged["nodes"]["12"]["epoch"] == 1
    assert fusion.snapshot(1.04)["frame_index"] == 1
    assert "calibration_reset_filters_reinitialized" not in fusion.input_snapshot(1.04)["warnings"]
    # Genuine source silence still expires inputs, independently of the solver.
    assert fusion.input_snapshot(2.)["nodes"] == {}


def wire(first=1, status=255):
    return b"".join(bytes((node, status)) + struct.pack("<6h", -node, node * 10, -32768, 32767, 0, 8197)
                    for node in range(first, first + 6))


@pytest.mark.parametrize("changes", [dict(node_id=True), dict(node_id=0), dict(node_id=13),
    dict(values=[0] * 5), dict(values=[False] * 6), dict(values=[32768] * 6),
    dict(values=[1.5] * 6), dict(host_monotonic_s=-1), dict(host_monotonic_s=float("nan")),
    dict(host_wall_s=float("inf")), dict(status=True), dict(status=256),
    dict(host_monotonic_ns=True), dict(host_monotonic_ns=-1),
    dict(host_monotonic_ns=1.5), dict(host_monotonic_ns=2_000_000_000),
    dict(host_realtime_ns=False), dict(host_realtime_ns=2.),
    dict(host_realtime_ns=3_000_000_000)])
def test_raw_sample_rejects_invalid_fields(changes):
    args = dict(node_id=1, values=[0] * 6, host_monotonic_s=1., host_wall_s=2.)
    args.update(changes)
    with pytest.raises(ValueError):
        live.RawSample(**args)


def test_parser_handles_every_split_preserves_status_and_exact_int16():
    payload = wire(1) + wire(7, 17)
    for split in range(len(payload) + 1):
        parser = live.ImuParser()
        rows = parser.feed(payload[:split], 1., 2.) + parser.feed(payload[split:], 1., 2.)
        assert [r.node_id for r in rows] == list(range(1, 13))
        assert rows[0].values == (-1, 10, -32768, 32767, 0, 8197)
        assert [r.status for r in rows] == [255] * 6 + [17] * 6
        assert all(r.host_monotonic_s == 1. for r in rows)
        assert parser.frames_ok == 2 and not parser.buffer


def test_parser_resynchronizes_damaged_frame_and_bounds_noise_buffer():
    corrupt = bytearray(wire())
    corrupt[14] = 12
    parser = live.ImuParser()
    rows = parser.feed(b"noise" * 9000 + corrupt + wire(7), 1., 2.)
    assert [r.node_id for r in rows] == list(range(7, 13))
    assert parser.frames_ok == 1 and parser.bytes_discarded >= len(corrupt)
    assert len(parser.buffer) < live.FRAME_SIZE
    parser.feed(b"x" * 20000, 2., 3.)
    assert len(parser.buffer) < live.FRAME_SIZE
    parser.reset()
    assert parser.frames_ok == parser.bytes_discarded == 0 and not parser.buffer


def test_parser_uses_completion_host_time_without_deduplicating_batch():
    parser = live.ImuParser()
    assert parser.feed(wire()[:30], 1., 2.) == []
    result = parser.feed(wire()[30:] + wire(), 2., 3.)
    assert len(result) == 12 and all(r.host_monotonic_s == 2. for r in result)
    with pytest.raises(ValueError):
        parser.feed(b"", float("nan"), 1.)
    with pytest.raises(TypeError):
        parser.feed("not bytes", 1., 2.)


def test_parser_preserves_exact_ns_for_all_samples_completed_in_one_batch():
    parser = live.ImuParser()
    # Deliberately beyond the integers exactly representable by a float.
    monotonic_ns, realtime_ns = 19_876_543_210_987_653, 1_791_001_234_567_890_123
    assert int((realtime_ns / 1_000_000_000) * 1_000_000_000) != realtime_ns
    assert parser.feed(wire()[:30], host_monotonic_ns=monotonic_ns - 100_000_000,
                       host_realtime_ns=realtime_ns - 100_000_000) == []
    rows = parser.feed(wire()[30:] + wire(7) + wire(),
                       host_monotonic_ns=monotonic_ns, host_realtime_ns=realtime_ns)
    assert len(rows) == 18
    assert all(row.host_monotonic_ns == monotonic_ns for row in rows)
    assert all(row.host_realtime_ns == realtime_ns for row in rows)
    assert all(type(row.host_realtime_ns) is int for row in rows)
    assert all(row.host_monotonic_s == monotonic_ns / 1_000_000_000 for row in rows)
    assert all(row.host_wall_s == realtime_ns / 1_000_000_000 for row in rows)
    assert parser.frames_ok == 3


def test_parser_float_compatibility_does_not_invent_integer_clock_precision():
    parser = live.ImuParser()
    rows = parser.feed(wire(), 1.25, 2.5)
    assert rows[0] == live.RawSample(1, (-1, 10, -32768, 32767, 0, 8197), 1.25, 2.5, 255)
    assert all(row.host_monotonic_ns is None and row.host_realtime_ns is None for row in rows)
    mixed = parser.feed(wire(), 1.5, host_realtime_ns=2_000_000_001)
    assert mixed[0].host_monotonic_ns is None
    assert mixed[0].host_realtime_ns == 2_000_000_001
    with pytest.raises(ValueError, match="same timestamp"):
        parser.feed(wire(), 1., 2., host_monotonic_ns=3_000_000_000)


def test_parser_default_clocks_preserve_native_integer_precision(monkeypatch):
    monotonic_ns, realtime_ns = 1_234_567_891, 1_791_001_234_567_890_123
    monkeypatch.setattr(live.time, "monotonic_ns", lambda: monotonic_ns)
    monkeypatch.setattr(live.time, "time_ns", lambda: realtime_ns)
    rows = live.ImuParser().feed(wire())
    assert rows[0].host_monotonic_ns == monotonic_ns
    assert rows[0].host_realtime_ns == realtime_ns


class FakePort:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.closed = False
        self.payload = wire()
        self.read_sizes = []
        self.waiting_queries = 0
    @property
    def in_waiting(self):
        self.waiting_queries += 1
        return len(self.payload)
    def read(self, size):
        self.read_sizes.append(size)
        part, self.payload = self.payload[:size], self.payload[size:]
        return part
    def write(self, data):
        pytest.fail("device writes are forbidden")
    def close(self):
        self.closed = True


@pytest.fixture
def fake_serial(monkeypatch):
    opened, released = [], []
    def factory(*args, **kwargs):
        port = FakePort(*args, **kwargs)
        opened.append(port)
        return port
    monkeypatch.setattr(live, "_serial_module", lambda: SimpleNamespace(Serial=factory, SerialException=OSError))
    monkeypatch.setattr(live, "_find_serial_port_holders", lambda port: [])
    monkeypatch.setattr(live, "_claim_tty_exclusive", lambda port: True)
    monkeypatch.setattr(live, "_release_tty_exclusive", lambda port: released.append(port))
    return opened, released


def test_serial_manual_only_single_exclusive_reader_no_writes(fake_serial):
    opened, released = fake_serial
    source = live.SerialSource("fake")
    assert not opened
    with pytest.raises(live.StreamError, match="not open"):
        source.read_samples()
    assert source.open() is source and source.open() is source
    assert len(opened) == 1
    assert opened[0].args == ("fake", 115200)
    assert opened[0].kwargs["timeout"] == .020
    if os.name == "posix":
        assert opened[0].kwargs["exclusive"] is True
    assert len(source.read_samples()) == 6
    assert source.read_samples() == []
    assert max(opened[0].read_sizes) <= 4096
    source.close()
    source.close()
    assert opened[0].closed and released == [opened[0]]
    assert len(opened) == 1


def test_serial_timestamps_read_completion_before_parser_lock(fake_serial, monkeypatch):
    opened, _ = fake_serial
    source = live.SerialSource("fake").open()
    events = []
    monotonic_ns, realtime_ns = 1_234_567_891, 1_791_001_234_567_890_123
    read = opened[0].read
    def read_and_mark(size):
        result = read(size)
        events.append("read_complete")
        return result
    def monotonic_clock():
        events.append("monotonic_ns")
        return monotonic_ns
    def realtime_clock():
        events.append("time_ns")
        return realtime_ns
    class ObservedLock:
        def __enter__(self):
            events.append("lock")
        def __exit__(self, *args):
            return False
    monkeypatch.setattr(opened[0], "read", read_and_mark)
    monkeypatch.setattr(live.time, "monotonic_ns", monotonic_clock)
    monkeypatch.setattr(live.time, "time_ns", realtime_clock)
    monkeypatch.setattr(source, "_diagnostics_lock", ObservedLock())
    rows = source.read_samples()
    assert events == ["read_complete", "monotonic_ns", "time_ns", "lock"]
    assert all(row.host_monotonic_ns == monotonic_ns and row.host_realtime_ns == realtime_ns for row in rows)
    source.close()


def test_serial_diagnostics_are_passive_and_empty_reads_do_not_invent_activity(fake_serial, monkeypatch):
    opened, _ = fake_serial
    clock = [10.]
    monkeypatch.setattr(live.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(live.time, "monotonic_ns", lambda: int(clock[0] * 1_000_000_000))
    source = live.SerialSource("fake")
    empty = dict(source_kind="raw_serial", opened=False, bytes_received=0,
                 frames_decoded=0, samples_decoded=0, bytes_discarded=0,
                 buffered_bytes=0, last_byte_age_s=None, last_sample_age_s=None)
    assert source.snapshot_diagnostics() == empty
    assert not opened
    source.open()
    port = opened[0]
    port.payload = b""
    assert source.snapshot_diagnostics() == dict(empty, opened=True)
    assert port.read_sizes == [] and port.waiting_queries == 0
    assert source.read_samples() == []
    clock[0] = 20.
    assert source.snapshot_diagnostics() == dict(empty, opened=True)
    assert port.read_sizes == [1] and port.waiting_queries == 1
    source.close()


def test_serial_diagnostics_distinguish_noise_split_frame_and_recovery(fake_serial, monkeypatch):
    opened, _ = fake_serial
    clock = [10.]
    monkeypatch.setattr(live.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(live.time, "monotonic_ns", lambda: int(clock[0] * 1_000_000_000))
    source = live.SerialSource("fake").open()
    port = opened[0]
    port.payload = b"x" * 100
    assert source.read_samples() == []
    noisy = source.snapshot_diagnostics(10.5)
    assert noisy == dict(source_kind="raw_serial", opened=True, bytes_received=100,
                         frames_decoded=0, samples_decoded=0, bytes_discarded=17,
                         buffered_bytes=83, last_byte_age_s=.5, last_sample_age_s=None)

    clock[0] = 11.
    port.payload = wire()[:30]
    assert source.read_samples() == []
    partial = source.snapshot_diagnostics()
    assert partial["bytes_received"] == 130
    assert partial["frames_decoded"] == partial["samples_decoded"] == 0
    assert partial["bytes_discarded"] == 47 and partial["buffered_bytes"] == 83
    assert partial["last_byte_age_s"] == 0 and partial["last_sample_age_s"] is None

    clock[0] = 12.
    port.payload = wire()[30:]
    assert [row.node_id for row in source.read_samples()] == list(range(1, 7))
    recovered = source.snapshot_diagnostics(12.5)
    assert recovered == dict(source_kind="raw_serial", opened=True, bytes_received=184,
                             frames_decoded=1, samples_decoded=6, bytes_discarded=100,
                             buffered_bytes=0, last_byte_age_s=.5, last_sample_age_s=.5)

    clock[0] = 13.
    port.payload = wire(7, status=17) + b"x" * 10
    rows = source.read_samples()
    assert [row.node_id for row in rows] == list(range(7, 13))
    assert all(row.status == 17 for row in rows)  # Status is retained, not a CRC.
    clock[0] = 14.
    assert source.read_samples() == []
    final = source.snapshot_diagnostics()
    assert final["bytes_received"] == 278
    assert final["frames_decoded"] == 2 and final["samples_decoded"] == 12
    assert final["bytes_discarded"] == 100 and final["buffered_bytes"] == 10
    assert final["last_byte_age_s"] == final["last_sample_age_s"] == 1.
    assert port.read_sizes == [100, 30, 54, 94, 1] and port.waiting_queries == 5
    clock[0] = 15.
    port.payload = b"x" * 84
    assert source.read_samples() == []
    more_noise = source.snapshot_diagnostics(15.5)
    assert more_noise["last_byte_age_s"] == .5 and more_noise["last_sample_age_s"] == 2.5
    assert more_noise["frames_decoded"] == 2 and more_noise["samples_decoded"] == 12
    assert more_noise["bytes_received"] == 362
    assert more_noise["bytes_discarded"] == 111 and more_noise["buffered_bytes"] == 83
    source.close()


def test_serial_diagnostics_preserved_on_repeat_open_and_close_reset_on_reopen(fake_serial, monkeypatch):
    opened, released = fake_serial
    clock = [10.]
    monkeypatch.setattr(live.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(live.time, "monotonic_ns", lambda: int(clock[0] * 1_000_000_000))
    source = live.SerialSource("fake").open()
    opened[0].payload = wire() + b"x" * 100
    assert len(source.read_samples()) == 6
    before = source.snapshot_diagnostics()
    assert source.open() is source and len(opened) == 1
    assert source.snapshot_diagnostics() == before
    source.close()
    source.close()
    assert opened[0].closed and released == [opened[0]]
    assert source.snapshot_diagnostics() == dict(before, opened=False)
    clock[0] = 20.
    assert source.snapshot_diagnostics()["last_byte_age_s"] == 10.
    assert source.open() is source and len(opened) == 2
    assert source.snapshot_diagnostics() == dict(source_kind="raw_serial", opened=True,
        bytes_received=0, frames_decoded=0, samples_decoded=0, bytes_discarded=0,
        buffered_bytes=0, last_byte_age_s=None, last_sample_age_s=None)
    source.close()


def test_serial_diagnostics_do_not_wait_for_blocking_read(fake_serial):
    opened, _ = fake_serial
    source = live.SerialSource("fake").open()
    reading, release, snapshot_done = threading.Event(), threading.Event(), threading.Event()
    rows, snapshots = [], []
    def blocked_read(size):
        reading.set()
        if not release.wait(2.):
            raise RuntimeError("test did not release fake read")
        return wire()[:size]
    opened[0].read = blocked_read
    reader = threading.Thread(target=lambda: rows.extend(source.read_samples()), daemon=True)
    def snapshot():
        snapshots.append(source.snapshot_diagnostics())
        snapshot_done.set()
    observer = threading.Thread(target=snapshot, daemon=True)
    reader.start()
    try:
        assert reading.wait(1.)
        observer.start()
        assert snapshot_done.wait(1.), "snapshot was blocked by serial I/O"
        assert snapshots[0]["bytes_received"] == snapshots[0]["samples_decoded"] == 0
    finally:
        release.set()
        reader.join(2.)
        if observer.ident is not None:
            observer.join(2.)
        source.close()
    assert not reader.is_alive() and not observer.is_alive()
    assert len(rows) == 6
    assert source.snapshot_diagnostics()["frames_decoded"] == 1


def test_serial_diagnostics_counters_and_parser_snapshot_are_atomic(fake_serial, monkeypatch):
    source = live.SerialSource("fake").open()
    parsing, release = threading.Event(), threading.Event()
    observing, snapshot_done = threading.Event(), threading.Event()
    rows, snapshots = [], []
    feed = source.parser.feed
    def blocked_feed(*args, **kwargs):
        parsing.set()
        if not release.wait(2.):
            raise RuntimeError("test did not release parser")
        return feed(*args, **kwargs)
    monkeypatch.setattr(source.parser, "feed", blocked_feed)
    reader = threading.Thread(target=lambda: rows.extend(source.read_samples()), daemon=True)
    def snapshot():
        observing.set()
        snapshots.append(source.snapshot_diagnostics())
        snapshot_done.set()
    observer = threading.Thread(target=snapshot, daemon=True)
    reader.start()
    try:
        assert parsing.wait(1.)
        observer.start()
        assert observing.wait(1.)
        assert not snapshot_done.wait(.05), "snapshot observed a partial parser update"
    finally:
        release.set()
        reader.join(2.)
        if observer.ident is not None:
            observer.join(2.)
        source.close()
    assert not reader.is_alive() and not observer.is_alive()
    assert len(rows) == 6 and snapshot_done.is_set()
    assert snapshots[0]["bytes_received"] == 84
    assert snapshots[0]["frames_decoded"] == 1 and snapshots[0]["samples_decoded"] == 6
    assert snapshots[0]["bytes_discarded"] == snapshots[0]["buffered_bytes"] == 0


def test_existing_holder_prevents_open_and_failed_lock_closes_fd(fake_serial, monkeypatch):
    opened, _ = fake_serial
    monkeypatch.setattr(live, "_find_serial_port_holders", lambda port: [(123, 9)])
    with pytest.raises(live.StreamError, match="PID 123"):
        live.SerialSource("fake").open()
    assert not opened
    monkeypatch.setattr(live, "_find_serial_port_holders", lambda port: [])
    def reject(port):
        raise OSError("exclusive lock failed")
    monkeypatch.setattr(live, "_claim_tty_exclusive", reject)
    with pytest.raises(live.StreamError, match="exclusive lock"):
        live.SerialSource("fake").open()
    assert len(opened) == 1 and opened[0].closed


def test_read_failure_is_reported_without_reopening(fake_serial):
    opened, _ = fake_serial
    source = live.SerialSource("fake").open()
    def failed(size):
        raise OSError("unplugged")
    opened[0].read = failed
    with pytest.raises(live.StreamError, match="unplugged"):
        source.read_samples()
    assert len(opened) == 1
    source.close()


def test_visible_fd_preflight_follows_symlink_without_touching_process(tmp_path, monkeypatch):
    monkeypatch.setattr(live.sys, "platform", "linux")
    target = tmp_path / "fake_device"
    target.touch()
    alias = tmp_path / "device_alias"
    alias.symlink_to(target)
    root = tmp_path / "proc"
    descriptors = root / "991" / "fd"
    descriptors.mkdir(parents=True)
    (descriptors / "8").symlink_to(alias)
    (descriptors / "9").symlink_to(tmp_path / "different")
    assert live._find_serial_port_holders(str(target), proc_root=root) == [(991, 8)]
    assert (descriptors / "8").is_symlink()


def test_recorder_roundtrip_preserves_unknown_status_and_backward_time(tmp_path):
    path = tmp_path / "raw.csv"
    recorder = live.RawRecorder(path)
    assert not path.exists()
    rows = [sample(5, 2., status=255), sample(6, 2., status=17), sample(5, 1., status=None)]
    recorder.open().write(rows)
    recorder.close()
    assert list(live.iter_csv_samples(path)) == rows
    with pytest.raises(FileExistsError):
        live.RawRecorder(path).open()
    with pytest.raises(RuntimeError):
        recorder.write(rows)
    with pytest.raises(RuntimeError):
        recorder.open()


def test_legacy_csv_optional_phase_and_missing_status_are_not_invented(tmp_path):
    path = tmp_path / "legacy.csv"
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([*live.CSV_FIELDS[:-1], "phase"])
        writer.writerow([1., 2., 5, 1, 2, 3, 4, 5, 6, "neutral"])
    result = list(live.iter_csv_samples(path))
    assert result == [live.RawSample(5, (1, 2, 3, 4, 5, 6), 1., 2., None)]
    assert result[0].host_monotonic_ns is None and result[0].host_realtime_ns is None


def test_raw_recorder_keeps_legacy_columns_without_claiming_ns_roundtrip(tmp_path):
    path = tmp_path / "raw.csv"
    monotonic_ns, realtime_ns = 1_234_567_891, 1_791_001_234_567_890_123
    rows = live.ImuParser().feed(wire(), host_monotonic_ns=monotonic_ns,
                                host_realtime_ns=realtime_ns)
    recorder = live.RawRecorder(path).open()
    recorder.write(rows)
    recorder.close()
    with path.open(newline="") as stream:
        assert tuple(next(csv.reader(stream))) == live.CSV_FIELDS
    restored = list(live.iter_csv_samples(path))
    assert restored[0].values == rows[0].values
    assert restored[0].host_monotonic_s == rows[0].host_monotonic_s
    assert restored[0].host_wall_s == rows[0].host_wall_s
    assert restored[0].host_monotonic_ns is None and restored[0].host_realtime_ns is None


@pytest.mark.parametrize("mutation", ["duplicate_header", "missing_field", "extra_field", "bad_id", "out_of_range", "bad_timestamp", "bad_status"])
def test_csv_rejects_malformed_data(tmp_path, mutation):
    fields = list(live.CSV_FIELDS)
    row = [1., 2., 1, 0, 0, 0, 0, 0, 8197, 0]
    if mutation == "duplicate_header": fields[-1] = fields[0]
    if mutation == "missing_field": row.pop()
    if mutation == "extra_field": row.append("extra")
    if mutation == "bad_id": row[2] = "01"
    if mutation == "out_of_range": row[3] = 32768
    if mutation == "bad_timestamp": row[0] = "nan"
    if mutation == "bad_status": row[-1] = 256
    path = tmp_path / "bad.csv"
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerow(row)
    with pytest.raises(ValueError, match="CSV"):
        list(live.iter_csv_samples(path))


def test_incremental_vqf_matches_official_units_and_configuration():
    config = load_config()
    # Historical/explicit overrides remain honored even if the default changes.
    config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "6": .0175}
    fusion = live.OnlineFusion(config)
    values = (500, -250, 100, 1200, 1700, 7900)
    for _ in range(30):
        fusion.update([sample(node, 1., values) for node in (3, 5, 6)])
    frame = fusion.snapshot(1.)
    for node, scale in ((3, .00875), (5, .0175), (6, .0175)):
        official = VQF(1 / 120)
        gyro = np.array(values[:3]) * scale * math.pi / 180.
        accel = np.array(values[3:]) * .000122 * 9.80665
        for _ in range(30): official.update(gyro, accel)
        actual = frame["nodes"][str(node)]
        assert actual["quat_xyzw"] == pytest.approx(official.getQuat6D()[[1, 2, 3, 0]])
        assert actual["sample_index"] == 30 and actual["epoch"] == 0
        assert actual["gyro_rad_s"] == pytest.approx(gyro)
        assert actual["accel_m_s2"] == pytest.approx(accel)
    assert frame["orientation_frame"] == "independent_initial_heading"


def test_fusion_preserves_per_node_source_ns_without_retiming_or_reintegrating():
    fusion = live.OnlineFusion({"stale_after_s": .1})
    parser = live.ImuParser()
    mono1, wall1 = 1_000_000_001, 1_791_001_234_567_890_123
    mono2, wall2 = 1_050_000_003, 1_791_001_234_617_890_125
    fusion.update(parser.feed(wire(1), host_monotonic_ns=mono1, host_realtime_ns=wall1))
    fusion.update(parser.feed(wire(7), host_monotonic_ns=mono2, host_realtime_ns=wall2))
    for now in (1.06, 1.09):
        frame = fusion.snapshot(now)
        for key, node in frame["nodes"].items():
            expected_mono, expected_wall = (mono1, wall1) if int(key) <= 6 else (mono2, wall2)
            assert node["host_monotonic_ns"] == expected_mono
            assert node["host_realtime_ns"] == expected_wall
            assert node["age_s"] == now - expected_mono / 1_000_000_000
            assert node["sample_index"] == 1 and node["epoch"] == 0
    stale = fusion.snapshot(1.13)
    assert set(stale["nodes"]) == {str(node) for node in range(7, 13)}
    assert fusion.snapshot(1.2)["nodes"] == {}
    # The legacy path remains explicitly unavailable at native ns precision.
    fusion.update([sample(1, 1.25)])
    legacy = fusion.snapshot(1.25)["nodes"]["1"]
    assert legacy["host_monotonic_ns"] is None and legacy["host_realtime_ns"] is None
    assert legacy["sample_index"] == 2


def test_snapshot_silence_stales_nodes_without_advancing_or_resetting_filter():
    fusion = live.OnlineFusion({"stale_after_s": .1})
    fusion.update([sample(1, 1.)])
    filt = fusion.states["1"]["filter"]
    q = fusion.states["1"]["quat_xyzw"][:]
    first = fusion.snapshot(1.)
    again = fusion.snapshot(1.05)
    stale = fusion.snapshot(1.2)
    assert first["frame_index"] == 0 and again["frame_index"] == 1 and stale["frame_index"] == 2
    assert again["nodes"]["1"]["sample_index"] == 1
    assert stale["nodes"] == {} and "node_1_stale" in stale["warnings"]
    assert fusion.states["1"]["filter"] is filt and fusion.states["1"]["quat_xyzw"] == q
    assert fusion.epochs["1"] == 0
    first["nodes"]["1"]["quat_xyzw"][0] = 42
    assert fusion.states["1"]["quat_xyzw"] == q


def test_gap_resets_only_returning_node_and_clock_rollback_resets_every_epoch():
    # Preserve the old reset contract only when explicitly requested for replay.
    fusion = live.OnlineFusion({"stale_after_s": .1, "filter_reset_gap_s": .1})
    fusion.update([sample(1, 1.), sample(2, 1.)])
    second_filter = fusion.states["2"]["filter"]
    fusion.update([sample(2, 1.09), sample(2, 1.18), sample(1, 1.2)])
    frame = fusion.snapshot(1.2)
    assert frame["nodes"]["1"]["epoch"] == 1
    assert frame["nodes"]["2"]["epoch"] == 0
    assert fusion.states["2"]["filter"] is second_filter
    assert "node_1_gap_filter_reinitialized" in frame["warnings"]
    # UI time moving backward does not itself mutate VQF/epochs.
    assert fusion.snapshot(.1)["nodes"] == {}
    assert fusion.epochs["1"] == 1 and fusion.epochs["2"] == 0
    fusion.update([sample(2, .1)])
    frame = fusion.snapshot(.1)
    assert set(frame["nodes"]) == {"2"}
    assert frame["nodes"]["2"]["epoch"] == 1 and frame["nodes"]["2"]["sample_index"] == 4
    assert fusion.epochs["1"] == 2
    assert "host_clock_reset_filters_reinitialized" in frame["warnings"]


def test_incremental_same_input_matches_offline_replay_including_gaps_and_resets(tmp_path):
    config = load_config()
    rows = [sample(n, tick / 120, (0, 0, 800, 0, 0, 8197))
            for tick in range(32) for n in (1, 5, 6)]
    rows += [sample(5, .5), sample(5, .5), sample(6, .5), sample(6, 1.7), sample(1, .1), sample(5, .2)]
    raw = tmp_path / "raw.csv"
    recorder = live.RawRecorder(raw).open()
    recorder.write(rows)
    recorder.close()
    output = tmp_path / "offline.jsonl"
    replay_raw(raw, config, output)
    expected = list(iter_orientation_frames(output))
    fusion = live.OnlineFusion(config)
    actual, last_output = [], None
    for stamp, batch in groupby(live.iter_csv_samples(raw), key=lambda s: s.host_monotonic_s):
        reset = fusion.last_input is not None and stamp < fusion.last_input
        fusion.update(batch)
        if last_output is None or reset or stamp - last_output >= 1 / config["output_rate_hz"] - 1e-9:
            actual.append(fusion.snapshot(stamp))
            last_output = stamp
    assert len(actual) == len(expected)
    for result, reference in zip(actual, expected):
        assert result["frame_index"] == reference["frame_index"]
        assert result["timestamp_s"] == reference["timestamp_s"]
        assert set(result["warnings"]) == set(reference["warnings"])
        assert result["gap_diagnostics"] == reference["gap_diagnostics"]
        assert set(result["nodes"]) == set(reference["nodes"])
        for key, node in reference["nodes"].items():
            for field in ("epoch", "sample_index", "age_s"):
                assert result["nodes"][key][field] == node[field]
            assert result["nodes"][key]["quat_xyzw"] == pytest.approx(node["quat_xyzw"], abs=1e-13)
    streamed = tmp_path / "streamed.jsonl"
    streamed.write_text("".join(json.dumps(frame) + "\n" for frame in actual))
    assert len(list(iter_orientation_frames(streamed))) == len(actual)


def test_empty_fusion_has_no_invented_identity_and_rejects_bad_updates():
    fusion = live.OnlineFusion({})
    assert fusion.snapshot(1.)["nodes"] == {}
    with pytest.raises(TypeError): fusion.update([object()])
    with pytest.raises(ValueError): fusion.snapshot(float("nan"))
    with pytest.raises(ValueError): live.OnlineFusion({"gyro_dps_per_lsb_by_node": {"05": .0175}})
