"""Image quality is measured from recorded headers, independently of timing topics."""
import struct
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
import yaml

from vt_multisensor_alignment import capture_quality as quality


SECOND = 1_000_000_000
IMAGE_TYPE = "sensor_msgs/msg/Image"


def cdr_header(stamp, *, endian="<", frame_id=b"camera"):
    return (b"\0\1\0\0" if endian == "<" else b"\0\0\0\0") + struct.pack(
        endian + "iII", stamp // SECOND, stamp % SECOND, len(frame_id) + 1
    ) + frame_id + b"\0"


def report_for(tmp_path, events, *, expected=None, counts=None):
    bag = tmp_path / "bag"
    bag.mkdir()
    counts = counts or {"/image": len(events)}
    info = dict(storage_identifier="mcap", duration={"nanoseconds": 2 * SECOND},
                starting_time={"nanoseconds_since_epoch": SECOND},
                message_count=sum(counts.values()), topics_with_message_count=[
                    dict(topic_metadata=dict(name=topic, type=IMAGE_TYPE), message_count=count)
                    for topic, count in counts.items()])
    (bag / "metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information": info}))
    pending = list(events)
    reader = NS(has_next=lambda: bool(pending), read_next=lambda: pending.pop(0))
    with patch.object(quality, "_open_reader", return_value=reader), \
         patch.object(quality, "_set_filter") as filtered, \
         patch.object(quality, "_deserializer", side_effect=AssertionError("Image must not deserialize")):
        report = quality.generate_capture_quality(
            bag, tmp_path / "report.json", expected_image_rates_hz=expected)
    return report, filtered


@pytest.mark.parametrize("endian", ["<", ">"])
def test_cdr_header_only_uses_a_view_of_the_payload(endian):
    from vt_multisensor_alignment.image_header import image_header_stamp_ns

    class NoFullCopy(bytes):
        def __bytes__(self):
            raise AssertionError("must not copy image payload")

    payload = NoFullCopy(cdr_header(2 * SECOND + 42, endian=endian) + b"pixel-data")
    assert image_header_stamp_ns(payload) == 2 * SECOND + 42


@pytest.mark.parametrize("payload", [
    b"\0\1", b"\0\3\0\0" + b"\0" * 20,
    b"\0\1\0\0" + struct.pack("<iII", -1, 0, 1) + b"\0",
    b"\0\1\0\0" + struct.pack("<iII", 1, SECOND, 1) + b"\0",
    b"\0\1\0\0" + struct.pack("<iII", 1, 0, 100) + b"\0",
    b"\0\1\0\0" + struct.pack("<iII", 1, 0, 0),
    cdr_header(SECOND)[:-1] + b"x",
])
def test_cdr_header_rejects_unsupported_truncated_or_invalid_header(payload):
    from vt_multisensor_alignment.image_header import image_header_stamp_ns
    with pytest.raises(ValueError):
        image_header_stamp_ns(payload)


def test_recorded_images_at_15hz_warn_even_with_no_long_gap(tmp_path):
    events = [("/image", cdr_header(SECOND + round(i * SECOND / 15)), SECOND + i)
              for i in range(31)]
    report, filtered = report_for(tmp_path, events, expected={"/image": 30.})
    image = report["topics"]["/image"]
    assert image["inspection"] == "image_headers"
    assert image["header_source_timing"]["positive_interval_rate_hz"] == 15.
    assert image["header_source_timing"]["gaps_over_threshold"] == 30
    assert image["rate_assessment"] == "below_expected"
    assert {item["code"] for item in report["warnings"]} >= {
        "image_rate_below_expected", "image_source_gaps"}
    rate_warning = next(item for item in report["warnings"] if item["code"] == "image_rate_below_expected")
    assert rate_warning["observed_rate_hz"] == 15.
    assert rate_warning["expected_rate_hz"] == 30.
    gap_warning = next(item for item in report["warnings"] if item["code"] == "image_source_gaps")
    assert gap_warning["count"] == 30
    assert gap_warning["gap_threshold_ns"] == 50_000_000
    assert report["summary"]["inspected_topic_count"] == 1
    assert report["summary"]["metadata_only_topic_count"] == 0
    assert filtered.call_args.args[1] == ["/image"]


def test_actual_30hz_jitter_uses_own_span_not_whole_bag_duration(tmp_path):
    # +/-0.2 ms jitter and 0.1% source clock drift must not masquerade as loss.
    events = [("/image", cdr_header(SECOND + round(i * SECOND / 29.97) + (i % 2) * 200_000),
               SECOND + i) for i in range(32)]
    report, _ = report_for(tmp_path, events, expected={"/image": 30.})
    image = report["topics"]["/image"]
    assert image["metadata_global_duration_rate_hz"] == 16.
    assert image["rate_assessment"] == "within_tolerance"
    assert report["status"] == "ok"


def test_healthy_average_does_not_hide_an_internal_missing_interval(tmp_path):
    stamps = [SECOND + round(i * SECOND / 30) for i in range(302) if i != 150]
    report, _ = report_for(tmp_path, [("/image", cdr_header(ns), ns) for ns in stamps],
                           expected={"/image": 30.})
    image = report["topics"]["/image"]
    assert image["rate_assessment"] == "within_tolerance"
    assert image["header_source_timing"]["gaps_over_threshold"] == 1
    assert report["status"] == "warnings"


def test_unconfigured_low_rate_image_is_observed_without_assuming_30hz(tmp_path):
    report, _ = report_for(tmp_path, [("/image", cdr_header(i * SECOND), i * SECOND)
                                     for i in (1, 2, 3)])
    image = report["topics"]["/image"]
    assert image["expected_rate_hz"] is None
    assert image["rate_assessment"] == "not_configured"
    assert image["header_source_timing"]["positive_interval_rate_hz"] == 1.
    assert report["status"] == "ok"


def test_finite_small_expected_rate_does_not_overflow_gap_threshold(tmp_path):
    report, _ = report_for(tmp_path, [("/image", cdr_header(i * SECOND), i * SECOND)
                                     for i in (1, 2, 3)], expected={"/image": 1e-300})
    assert report["topics"]["/image"]["rate_assessment"] == "within_tolerance"
    assert report["topics"]["/image"]["header_source_timing"]["gap_threshold_ns"] > 10 ** 300


@pytest.mark.parametrize("count", [0, 1, 5])
def test_empty_or_short_stream_cannot_claim_expected_rate(tmp_path, count):
    events = [("/image", cdr_header(SECOND + i * 33_333_333), SECOND + i)
              for i in range(count)]
    report, _ = report_for(tmp_path, events, expected={"/image": 30.}, counts={"/image": count})
    assert report["topics"]["/image"]["rate_assessment"] == "insufficient_samples"
    assert "image_timing_insufficient" in {w["code"] for w in report["warnings"]}


def test_bad_headers_duplicates_and_regressions_warn_without_stopping_scan(tmp_path):
    stamps = [SECOND, SECOND, SECOND - 1, 0, 2 * SECOND]
    events = [("/image", cdr_header(ns), SECOND + i) for i, ns in enumerate(stamps)]
    events.insert(2, ("/image", b"broken", SECOND + 2))
    report, _ = report_for(tmp_path, events, expected={"/image": 30.})
    image = report["topics"]["/image"]
    assert image["observed_messages"] == 6
    assert image["malformed_messages"] == 1
    assert image["invalid_time_messages"] == 1
    assert image["header_source_timing"]["duplicate_timestamps"] == 1
    assert image["header_source_timing"]["timestamp_regressions"] == 1
    assert {w["code"] for w in report["warnings"]} >= {
        "image_source_duplicates", "image_source_regressions", "malformed_messages", "invalid_time_messages"}


@pytest.mark.parametrize("expected", [{"/image": 0}, {"/image": True}, {"/image": float("nan")},
                                      {"/image": float("inf")}, {"/image": "30"}])
def test_invalid_expected_rates_fail_before_reader_or_report(tmp_path, expected):
    with pytest.raises(ValueError, match="expected image rate"):
        report_for(tmp_path, [], expected=expected)
    assert not (tmp_path / "report.json").exists()


def test_real_mcap_checks_images_independently_of_complete_timing(tmp_path):
    rosbag2 = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Image
    from vt_camera_msgs.msg import CameraFrameTiming

    bag = tmp_path / "bag"
    writer = rosbag2.SequentialWriter()
    writer.open(rosbag2.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2.ConverterOptions("", ""))
    types = {"/color": IMAGE_TYPE, "/depth": IMAGE_TYPE, "/timing": quality.CAMERA_TYPE}
    for index, (topic, type_name) in enumerate(types.items(), 1):
        writer.create_topic(rosbag2.TopicMetadata(id=index, name=topic, type=type_name, serialization_format="cdr"))
    for index in range(61):
        ns = SECOND + round(index * SECOND / 30)
        image = Image(height=64, width=64, encoding="mono8", step=64, data=bytes(64 * 64))
        image.header.stamp.sec, image.header.stamp.nanosec = divmod(ns, SECOND)
        writer.write("/color", serialize_message(image), ns)
        if index % 2 == 0:
            writer.write("/depth", serialize_message(image), ns)
        timing = CameraFrameTiming(camera_name="cam", serial_number="123",
                                   group_host_realtime_ns=ns, group_host_monotonic_raw_ns=ns + 100,
                                   group_validity_flags=11)
        timing.header.stamp = image.header.stamp
        writer.write("/timing", serialize_message(timing), ns + 1)
    writer.close()
    before = {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in bag.iterdir()}
    real_deserializer = quality._deserializer
    def no_image_deserialization(type_name):
        assert type_name != IMAGE_TYPE
        return real_deserializer(type_name)
    with patch.object(quality, "_deserializer", side_effect=no_image_deserialization):
        report = quality.generate_capture_quality(bag, tmp_path / "report.json",
                                                   expected_image_rates_hz={"/color": 30., "/depth": 30.})
    assert report["topics"]["/timing"]["source_monotonic_timing"]["positive_interval_rate_hz"] == 30.
    assert report["topics"]["/color"]["rate_assessment"] == "within_tolerance"
    assert report["topics"]["/depth"]["rate_assessment"] == "below_expected"
    assert {w["topic"] for w in report["warnings"] if w["code"] == "image_rate_below_expected"} == {"/depth"}
    assert before == {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in bag.iterdir()}
