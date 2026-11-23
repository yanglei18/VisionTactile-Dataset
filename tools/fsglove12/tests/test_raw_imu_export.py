"""Original synthetic IMU evidence; no hardware, calibration or model assets."""
import json
import socket
import struct
import threading
import time

from vt_fsglove12.live_stream import ImuParser, OnlineFusion, RawSample
from vt_fsglove12.raw_imu_export import RawImuDatagramExporter, raw_sample_document


def wait_for(predicate):
    end = time.monotonic() + 3.
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(.005)
    raise AssertionError("raw export did not reach expected state")


def observation(index=1):
    real, mono = 1_800_000_000_000_000_123, 10_000_000_123
    sample = RawSample(9, (1, 2, 3, 4, 5, 6), mono / 1_000_000_000, real / 1_000_000_000, 255, mono, real)
    return raw_sample_document(sample, index, 0, dict(gyro_rejected=True, accel_rejected=False), True)


def publish(exporter, rows):
    exporter.publish(rows, hand="left", session_id="session", config_sha256="a" * 64, source_mode="live")


def test_parser_and_fusion_export_exact_clocks_raw_rejections_and_reset_identity():
    parser, fusion, rows = ImuParser(), OnlineFusion({"input_filter": {"enabled": True}}), []
    real, mono = 1_800_000_000_000_000_123, 10_000_000_123
    for i, gyro in enumerate((0, 32764)):
        wire = b"".join(struct.pack("<BB6h", node, 255 - node, gyro, 0, 0, 0, 0, 8197) for node in range(7, 13))
        samples = parser.feed(wire, host_realtime_ns=real + i * 10_000_000, host_monotonic_ns=mono + i * 10_000_000)
        fusion.update(samples, raw_observer=lambda *args: rows.append(raw_sample_document(*args)))
    assert len(rows) == 12
    assert rows[0]["host_realtime_ns"] == real
    assert rows[0]["host_monotonic_ns"] == mono
    rejected = rows[6]
    assert rejected["gyro_rejected"] and rejected["raw_values"][0] == 32764
    assert rejected["status"] == 248 and rejected["status_known"]
    before = fusion.snapshot((mono + 10_000_000) / 1e9)
    assert all(before["nodes"][str(row["node_id"])]["sample_index"] == row["sample_index"] for row in rows[6:])
    fusion.reset_for_calibration()
    sample = samples[0]
    fusion.update([sample], raw_observer=lambda *args: rows.append(raw_sample_document(*args)))
    assert rows[-1]["sample_index"] == 3 and rows[-1]["filter_epoch"] == 1
    legacy = RawSample(9, (0, 0, 0, 0, 0, 0), 1., 2., None)
    row = raw_sample_document(legacy, 1, 0, dict(gyro_rejected=False, accel_rejected=False), True)
    assert not row["time_valid"] and row["host_realtime_ns"] == row["host_monotonic_ns"] == 0
    assert not row["status_known"]


def test_transport_failure_is_counted_and_delivered_after_receiver_recovers(tmp_path):
    exporter = RawImuDatagramExporter(str(tmp_path / "pose.sock"))
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.settimeout(2.)
    try:
        publish(exporter, [observation()])
        wait_for(lambda: exporter.snapshot()["dropped_samples"] == 1)
        receiver.bind(exporter.path)
        publish(exporter, [observation(2)])
        document = json.loads(receiver.recv(60_000))
        assert document["batch_sequence"] == 2
        assert document["producer_dropped_batches_total"] == document["producer_dropped_samples_total"] == 1
        assert document["samples"][0] == observation(2)
        assert exporter.snapshot()["last_error"]  # Failure evidence stays visible after recovery.
    finally:
        assert exporter.close()
        receiver.close()


def test_one_large_read_preserves_all_samples_across_bounded_datagrams(tmp_path):
    exporter = RawImuDatagramExporter(str(tmp_path / "pose.sock"))
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.bind(exporter.path)
    receiver.settimeout(2.)
    try:
        rows = [observation(index) for index in range(1, 131)]
        publish(exporter, rows)
        batches = [json.loads(receiver.recv(60_000)) for _ in range(3)]
        assert [len(batch["samples"]) for batch in batches] == [64, 64, 2]
        assert [batch["batch_sequence"] for batch in batches] == [1, 2, 3]
        assert [row for batch in batches for row in batch["samples"]] == rows
        assert exporter.snapshot()["dropped_samples"] == 0
    finally:
        assert exporter.close()
        receiver.close()


def test_bounded_queue_loses_explicit_samples_without_blocking_reader(tmp_path):
    exporter = RawImuDatagramExporter(str(tmp_path / "pose.sock"), capacity=1)
    entered, release = threading.Event(), threading.Event()
    original = exporter._socket

    class PausedSocket:
        def sendto(self, payload, path):
            entered.set()
            assert release.wait(3.)
            return len(payload)

        def close(self):
            original.close()

    exporter._socket = PausedSocket()
    try:
        publish(exporter, [observation()])
        assert entered.wait(2.)
        publish(exporter, [observation(2)])
        publish(exporter, [observation(3), observation(4)])
        status = exporter.snapshot()
        assert status["queued_batches"] == status["queue_capacity"] == 1
        assert status["dropped_batches"] == 1 and status["dropped_samples"] == 2
    finally:
        release.set()
        assert exporter.close()
