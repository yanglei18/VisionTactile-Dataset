import json
from types import SimpleNamespace

from vt_multisensor_alignment.capture_quality import generate_capture_quality
from vt_multisensor_alignment.raw_imu_quality import RawImuQuality


def test_closed_mcap_detects_dropped_tail_even_when_bag_metadata_matches(tmp_path):
    import pytest
    rosbag = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    bag = tmp_path / "bag"
    writer = rosbag.SequentialWriter()
    writer.open(rosbag.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag.ConverterOptions("", ""))
    writer.create_topic(rosbag.TopicMetadata(id=1, name="/opaque", type="std_msgs/msg/String",
                                             serialization_format="cdr"))
    for i in range(3):
        writer.write("/opaque", serialize_message(String(data=str(i))), 1000 + i)
    writer.close()
    metrics = dict(schema_version=1, bag_path=str(bag), session_id=tmp_path.name,
                   storage_write_semantics="storage_plugin_write_returned_not_fsync", pid=101, observed_monotonic_ns=512,
                   recorder_instance_id="test-recorder", writer_errors=0, storage_errors=0,
                   closed=True, topics=[dict(topic="/opaque", writer_received_messages=4,
                                             storage_returned_messages=3, writer_received_bytes=42,
                                             storage_returned_bytes=42, storage_returned_last_monotonic_ns=501)])
    counters = tmp_path / "recorder-metrics.json"
    counters.write_text(json.dumps(metrics))
    report = generate_capture_quality(bag, tmp_path / "first.json")
    row = report["recorder_reconciliation"]
    assert row["status"] == "incomplete"
    assert row["topics"]["/opaque"]["mcap_read"] == 3
    assert row["topics"]["/opaque"]["mcap_record_timestamp_timing"]["positive_interval_rate_hz"] == 1e9
    metrics["topics"][0]["writer_received_messages"] = 3
    counters.write_text(json.dumps(metrics))
    assert generate_capture_quality(bag, tmp_path / "second.json")["recorder_reconciliation"]["status"] == "verified"
    metrics["closed"] = False
    counters.write_text(json.dumps(metrics))
    assert generate_capture_quality(bag, tmp_path / "third.json")["recorder_reconciliation"]["status"] == "unverified"


def test_raw_rates_count_samples_with_duplicate_batch_timestamps_and_exclude_disconnect_gap():
    q = RawImuQuality("/gloves/left/imu/raw")
    def add(session, seq, index, times):
        samples = [SimpleNamespace(node_id=9, sample_index=index+i, raw_values=[0]*6,
                                   time_valid=True, host_monotonic_ns=stamp,
                                   gyro_rejected=False, accel_rejected=False) for i, stamp in enumerate(times)]
        q.add(SimpleNamespace(schema_version=1, hand="left", session_id=session,
                              source_mode="live", config_sha256="a"*64, batch_sequence=seq,
                              clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                              producer_dropped_batches_total=2, producer_dropped_samples_total=4,
                              bridge_sequence_gaps_total=seq, bridge_rejected_total=0,
                              samples=samples), times[0])

    add("a", 10, 50, [1_000_000, 1_000_000])
    add("a", 12, 53, [21_000_000])
    add("b", 1, 1, [800_000_000])
    r = q.document()
    assert r["recorded_samples"] == 4 and r["unobserved_batches"] == 1
    assert r["nodes"]["9"] == dict(samples=4, unobserved_sample_indices=1,
        host_timestamp_regressions=0, repeated_or_reordered_indices=0, observed_sample_hz=100.0, longest_host_gap_ns=20_000_000)
    assert r["cumulative_counters"]["bridge_sequence_gaps_total"]["observed_increase"] == 2


def test_moved_archive_requires_agreeing_original_controller_journal(tmp_path):
    import pytest
    from vt_multisensor_alignment.recorder_reconciliation import read_recorder_counters

    session = tmp_path / "archive" / "trial"
    bag = session / "bag"
    bag.mkdir(parents=True)
    original = "/original/trial/bag"
    metrics = dict(schema_version=1, session_id="trial", recorder_instance_id="process",
                   bag_path=original, closed=True, observed_monotonic_ns=100, pid=10,
                   storage_write_semantics="storage_plugin_write_returned_not_fsync",
                   topics=[], writer_errors=0, storage_errors=0)
    (session/"recorder-metrics.json").write_text(json.dumps(metrics))
    with pytest.raises(ValueError, match="Moved bag"):
        read_recorder_counters(bag)
    journal = session/"controller_session.json"
    journal.write_text(json.dumps(dict(version=1, session_id="trial", output_path=original)))
    assert read_recorder_counters(bag) == metrics
    journal.write_text(json.dumps(dict(version=1, session_id="another", output_path=original)))
    with pytest.raises(ValueError, match="Moved bag"):
        read_recorder_counters(bag)
