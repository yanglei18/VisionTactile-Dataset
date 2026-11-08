"""Real CDR/MCAP regression for frozen legacy26 and named row44 contracts."""
import json

import numpy as np
import pytest

pytest.importorskip("rclpy.serialization")
messages = pytest.importorskip("vt_tactile_msgs.msg")
if not hasattr(messages, "TactileRow44Array"):
    pytest.skip("Build the versioned tactile ROS interfaces first", allow_module_level=True)

from rclpy.serialization import serialize_message
import rosbag2_py

from vt_multisensor_alignment.bag_reader import _deserializer
from vt_multisensor_alignment.tactile_adapter import (
    TACTILE_SAMPLE_TYPE, TACTILE_ROW44_SAMPLE_TYPE, validate_serialized_schema,
)
from vt_multisensor_alignment.tactile_export import iter_tactile_events, reconstruct_frames, export_tactile
from vt_multisensor_alignment.capture_quality import TopicQualityAccumulator


def message_for(type_name, hand="right"):
    row44 = type_name == TACTILE_ROW44_SAMPLE_TYPE
    cls = messages.TactileRow44Array if row44 else messages.TactileSampleArray
    value = cls(schema_version=2 if row44 else 1, hand=hand, session_id="cdr-fixture",
                stream_epoch=1, sequence=0, first_packet_index=0,
                host_realtime_ns=1_000_000_000, host_monotonic_ns=2_000_000_000,
                clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
                connected=True, time_valid=True, reason="ok")
    value.header.stamp.sec = 1
    value.header.frame_id = hand + "_tactile"
    if row44:
        value.blocks = [messages.TactileRow44(row=r, adc=[r * 100 + c for c in range(20)])
                        for r in range(20)]
    else:
        value.blocks = [messages.TactileBlock(row=r, column_group=g, status_raw=77,
                        adc=[r * 100 + g + 8 * k for k in range(10)])
                        for r in range(20) for g in range(8)]
    return value


@pytest.mark.parametrize("type_name", [TACTILE_SAMPLE_TYPE, TACTILE_ROW44_SAMPLE_TYPE])
@pytest.mark.parametrize("hand", ["left", "right"])
def test_actual_cdr_mcap_reconstruction_preserves_adc_and_source_granularity(tmp_path, type_name, hand):
    incoming = message_for(type_name, hand)
    payload = serialize_message(incoming)
    before = bytes(payload)
    decoded = _deserializer(type_name)(payload)
    assert decoded == incoming
    topic = f"/gloves/{hand}/tactile/samples"
    bag = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=topic, type=type_name, serialization_format="cdr"))
    writer.write(topic, payload, incoming.host_realtime_ns + 12345)
    del writer
    events = list(iter_tactile_events(bag, topic=topic))
    assert len(events) == 1 and events[0].source_type == type_name
    frames = list(reconstruct_frames(events, [(0, incoming.host_realtime_ns)], max_age_ns=100_000_000))
    arrays, metadata = frames[0]
    expected = np.arange(20)[:, None] * 100 + np.arange(20)
    np.testing.assert_array_equal(arrays["adc"], expected)
    assert arrays["valid_mask"].all() and arrays["adc"][0, 0] == 0
    assert metadata["last_source_event"]["source_type"] == type_name
    prefix = "row" if type_name == TACTILE_ROW44_SAMPLE_TYPE else "block"
    shape = (20,) if prefix == "row" else (20, 8)
    assert arrays[prefix + "_monotonic_ns"].shape == shape
    assert np.all(arrays[prefix + "_monotonic_ns"] == incoming.host_monotonic_ns)
    targets = tmp_path / "aligned_frames.jsonl"
    targets.write_text(json.dumps(dict(frame_index=0, reference_time_ns=incoming.host_realtime_ns)) + "\n")
    with pytest.warns(UserWarning, match="UNVERIFIED"):
        output = export_tactile(bag_path=bag, aligned_frames=targets, output_directory=tmp_path / "export",
                                topic=topic)
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["source_type"] == type_name
    assert manifest["source_schema_version"] == incoming.schema_version
    quality = TopicQualityAccumulator(topic, type_name)
    quality.add(decoded, incoming.host_realtime_ns + 12345)
    report = quality.document()
    assert report["malformed_messages"] == 0
    assert report["tactile"]["observed_at_least_once_cells"] == 400
    assert len(report["tactile"]["per_physical_block"]) == (20 if prefix == "row" else 160)
    assert bytes(payload) == before


def test_historical_same_name_row44_payload_is_rejected_before_legacy_array_decode():
    payload = serialize_message(message_for(TACTILE_ROW44_SAMPLE_TYPE))
    # This is exactly the incompatible historical fixed-array shape. Its schema
    # cannot be repaired by installing the old package under the same type name.
    with pytest.raises(ValueError, match="type/schema mismatch"):
        _deserializer(TACTILE_SAMPLE_TYPE)(payload)


def test_truncated_or_unidentified_cdr_cannot_select_a_legacy_decoder():
    payload = serialize_message(message_for(TACTILE_SAMPLE_TYPE))
    for short in (b"", payload[:12], payload[:18]):
        with pytest.raises(ValueError):
            validate_serialized_schema(TACTILE_SAMPLE_TYPE, short)
