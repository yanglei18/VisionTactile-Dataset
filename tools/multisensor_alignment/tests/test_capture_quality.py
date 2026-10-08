from __future__ import annotations

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import yaml

from vt_multisensor_alignment.capture_quality import (
    CAMERA_TYPE, GLOVE_TYPE, TACTILE_SAMPLE_TYPE, TACTILE_STATE_TYPE, TRACKER_TYPE,
    IntervalAccumulator, TopicQualityAccumulator, generate_capture_quality,
)
from vt_multisensor_alignment.config import load_config


ROOT = Path(__file__).resolve().parents[1]
MODULE = "vt_multisensor_alignment.capture_quality"
SECOND = 1_000_000_000


def _header(ns):
    return NS(stamp=NS(sec=ns // SECOND, nanosec=ns % SECOND))


def _block(row=0, group=0):
    return NS(row=row, column_group=group, status_raw=0, adc=[0] * 10)


def _batch(ns, sequence=0, **changes):
    values = dict(
        schema_version=1, header=_header(ns), hand="right", session_id="a",
        stream_epoch=0, sequence=sequence, first_packet_index=sequence,
        host_realtime_ns=ns, host_monotonic_ns=ns + 100,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        connected=True, time_valid=True, reason="", producer_dropped_total=0,
        discarded_bytes_total=0, malformed_packets_total=0, blocks=[_block()],
    )
    values.update(changes)
    return NS(**values)


def _glove(ns, frame_index=0, **changes):
    values = dict(
        schema_version=1, header=_header(ns), session_id="a", frame_index=frame_index,
        host_realtime_ns=ns, host_monotonic_ns=ns + 100, time_valid=True,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        pose_valid=True, producer_dropped_total=0,
    )
    values.update(changes)
    return NS(**values)


def _metadata(bag, topics, duration=SECOND):
    bag.mkdir()
    info = {
        "storage_identifier": "mcap", "duration": {"nanoseconds": duration},
        "starting_time": {"nanoseconds_since_epoch": SECOND},
        "message_count": sum(count for _, count in topics.values()),
        "topics_with_message_count": [
            {"topic_metadata": {"name": topic, "type": type_name}, "message_count": count}
            for topic, (type_name, count) in topics.items()
        ],
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": info}), encoding="utf-8")


class IntervalTests(unittest.TestCase):
    def test_frequency_duplicates_regressions_and_gap_are_distinct(self):
        timing = IntervalAccumulator(gap_threshold_ns=20)
        for value in (100, 110, 110, 90, 120, 200):
            timing.add(value)
        result = timing.document()
        self.assertEqual(result["duplicate_timestamps"], 1)
        self.assertEqual(result["timestamp_regressions"], 1)
        self.assertEqual(result["gaps_over_threshold"], 1)
        self.assertEqual(result["interval_ns"]["maximum"], 80)
        self.assertEqual(result["positive_interval_rate_hz"], 30_000_000)

    def test_segment_boundary_does_not_create_an_interval(self):
        timing = IntervalAccumulator()
        timing.add(100)
        timing.add(200)
        timing.reset_boundary()
        timing.add(1)
        self.assertEqual(timing.document()["interval_ns"]["count"], 1)
        self.assertEqual(timing.regressions, 0)


class CompactQualityTests(unittest.TestCase):
    def test_complete_raw_scan_counts_only_physical_roi_and_zero_is_valid(self):
        quality = TopicQualityAccumulator("/samples", TACTILE_SAMPLE_TYPE)
        blocks = [_block(row, group) for row in range(80) for group in range(8)]
        for index in range(3):
            ns = SECOND + index * 32_000_000
            quality.add(_batch(ns, index, first_packet_index=index * 640, blocks=blocks), ns)
        result = quality.document()
        tactile = result["tactile"]
        self.assertEqual(result["malformed_messages"], 0)
        self.assertEqual(tactile["raw_wire_blocks"], 1920)
        self.assertEqual(tactile["outside_physical_roi_blocks"], 1440)
        self.assertEqual(tactile["usable_physical_blocks"], 480)
        self.assertEqual(tactile["observed_at_least_once_cells"], 400)
        self.assertEqual(tactile["complete_observed_coverage_cycles"], 3)
        self.assertEqual(tactile["coverage_cycle_timing"]["positive_interval_rate_hz"], 31.25)
        self.assertEqual(len(tactile["per_physical_block"]), 160)
        self.assertEqual(tactile["per_physical_block"][0]["observations"], 3)
        self.assertIn("NOT hardware", tactile["cycle_basis"])

    def test_batch_frequency_not_confused_with_physical_block_frequency(self):
        quality = TopicQualityAccumulator("/samples", TACTILE_SAMPLE_TYPE)
        for sequence, row in enumerate((0, 79, 79, 0, 79, 79, 0)):
            ns = SECOND + sequence * 2_000_000
            quality.add(_batch(ns, sequence, blocks=[_block(row)]), ns)
        result = quality.document()
        self.assertEqual(result["source_monotonic_timing"]["positive_interval_rate_hz"], 500)
        block = result["tactile"]["per_physical_block"][0]
        self.assertAlmostEqual(block["positive_interval_rate_hz"], 1000 / 6)
        self.assertEqual(result["tactile"]["observed_at_least_once_cells"], 3)

    def test_sequence_and_cycles_do_not_cross_sessions_epochs_or_disconnects(self):
        for boundary in (dict(session_id="b"), dict(stream_epoch=1),
                         dict(connected=False), dict(time_valid=False)):
            with self.subTest(boundary=boundary):
                quality = TopicQualityAccumulator("/samples", TACTILE_SAMPLE_TYPE)
                first = [_block(row, group) for row in range(10) for group in range(8)]
                second = [_block(row, group) for row in range(10, 20) for group in range(8)]
                quality.add(_batch(SECOND, 100, first_packet_index=0, blocks=first), SECOND)
                quality.add(_batch(SECOND + 10, 0, first_packet_index=0, blocks=second, **boundary), SECOND + 10)
                self.assertEqual(quality.document()["tactile"]["complete_observed_coverage_cycles"], 0)
                if "session_id" in boundary or "stream_epoch" in boundary:
                    self.assertEqual(quality.document()["software_sequence"]["regressions"], 0)

    def test_sequence_highwater_does_not_double_count_repeated_missing_range(self):
        quality = TopicQualityAccumulator("/glove", GLOVE_TYPE)
        for index, sequence in enumerate((0, 3, 1, 3, 4)):
            ns = SECOND + index * 10
            quality.add(_glove(ns, sequence), ns)
        sequence = quality.document()["software_sequence"]
        self.assertEqual(sequence["unobserved_index_values"], 2)
        self.assertEqual(sequence["gap_events"], 1)
        self.assertEqual(sequence["regressions"], 1)
        self.assertEqual(sequence["duplicates"], 1)

    def test_counter_increments_are_not_cumulative_sums_or_initial_loss(self):
        quality = TopicQualityAccumulator("/glove", GLOVE_TYPE)
        for index, count in enumerate((10, 10, 12, 11, 12, 13)):
            ns = SECOND + index * 100
            quality.add(_glove(ns, index, producer_dropped_total=count), ns)
        result = quality.document()["cumulative_counters"]["producer_dropped_total"]
        self.assertEqual(result["observed_increase"], 3)
        self.assertEqual(result["initial_values_sum_not_attributed_to_recording"], 10)
        self.assertEqual(result["counter_regressions"], 1)
        quality.add(_glove(SECOND + 1000, 0, session_id="b", producer_dropped_total=7), SECOND + 1000)
        quality.add(_glove(SECOND + 1100, 1, session_id="b", producer_dropped_total=8), SECOND + 1100)
        result = quality.document()["cumulative_counters"]["producer_dropped_total"]
        self.assertEqual(result["observed_increase"], 4)
        self.assertEqual(result["initial_values_sum_not_attributed_to_recording"], 17)

    def test_counter_does_not_restart_at_tactile_epoch_boundary(self):
        quality = TopicQualityAccumulator("/samples", TACTILE_SAMPLE_TYPE)
        quality.add(_batch(SECOND, producer_dropped_total=4), SECOND)
        quality.add(_batch(SECOND + 10, 1, stream_epoch=1, producer_dropped_total=5), SECOND + 10)
        self.assertEqual(quality.document()["cumulative_counters"]["producer_dropped_total"]["observed_increase"], 1)

    def test_zero_source_stamps_do_not_become_rates_or_latencies(self):
        quality = TopicQualityAccumulator("/glove", GLOVE_TYPE)
        quality.add(_glove(0, host_monotonic_ns=0), SECOND)
        result = quality.document()
        self.assertEqual(result["invalid_time_messages"], 1)
        self.assertIsNone(result["source_monotonic_timing"]["positive_interval_rate_hz"])
        self.assertEqual(result["bag_receive_minus_source_realtime_ns"]["count"], 0)

    def test_clock_step_header_mismatch_and_invalid_pose_reported(self):
        quality = TopicQualityAccumulator("/glove", GLOVE_TYPE)
        quality.add(_glove(SECOND), SECOND)
        quality.add(_glove(SECOND + 30_000_000, 1, host_monotonic_ns=SECOND + 10_000_100), SECOND + 30_000_000)
        quality.add(_glove(SECOND + 40_000_000, 2, header=_header(SECOND), pose_valid=False), SECOND + 40_000_000)
        result = quality.document()
        self.assertEqual(result["clock_step_events"], 1)
        self.assertEqual(result["header_source_mismatches"], 1)
        self.assertEqual(result["invalid_time_messages"], 1)
        self.assertEqual(result["invalid_pose_messages"], 1)

    def test_camera_uses_group_host_clocks_not_header_device_clock(self):
        quality = TopicQualityAccumulator("/timing", CAMERA_TYPE)
        for index in range(3):
            ns = SECOND + index * 10_000_000
            message = NS(camera_name="cam", serial_number="device", header=_header(0),
                         group_host_realtime_ns=ns, group_host_monotonic_raw_ns=ns + 100,
                         group_validity_flags=11)
            quality.add(message, ns)
        self.assertEqual(quality.document()["source_monotonic_timing"]["positive_interval_rate_hz"], 100)
        self.assertEqual(quality.document()["header_source_mismatches"], 0)

    def test_tracker_packet_indices_are_not_claimed_to_be_software_loss(self):
        quality = TopicQualityAccumulator("/tracker", TRACKER_TYPE)
        for index in range(3):
            ns = SECOND + index * 10
            quality.add(NS(header=_header(ns), tracker_id="id", pose_valid=True,
                           host_realtime_ns=ns, host_monotonic_ns=ns + 1,
                           packet_index=index * 10, tracking_status=2), ns)
        self.assertNotIn("software_sequence", quality.document())

    def test_state_coverage_crops_legacy_and_preserves_valid_zero(self):
        for schema, width in ((1, 80), (2, 20)):
            with self.subTest(schema=schema):
                quality = TopicQualityAccumulator("/state", TACTILE_STATE_TYPE)
                mask = [False] * (width * width)
                mask[19 * width + 19] = True
                if width == 80:
                    mask[-1] = True
                message = _batch(SECOND)
                message.schema_version = schema
                message.rows = message.cols = width
                message.valid_mask = mask
                message.block_monotonic_ns = [SECOND] * (width * 8)
                message.snapshot_monotonic_ns = SECOND + 123
                quality.add(message, SECOND)
                result = quality.document()["tactile"]
                self.assertEqual(result["valid_cells_per_snapshot"]["mean"], 1)
                self.assertEqual(result["valid_cell_age_ns"]["mean"], 123)

    def test_malformed_messages_do_not_abort_or_claim_complete_coverage(self):
        quality = TopicQualityAccumulator("/samples", TACTILE_SAMPLE_TYPE)
        quality.add(_batch(SECOND, blocks=[_block(80)]), SECOND)
        quality.add(_batch(SECOND + 1, 1), SECOND + 1)
        result = quality.document()
        self.assertEqual(result["malformed_messages"], 1)
        self.assertEqual(result["decoded_messages"], 1)
        self.assertEqual(result["observed_messages"], 2)
        self.assertEqual(result["tactile"]["observed_at_least_once_cells"], 3)


class CaptureReportTests(unittest.TestCase):
    def test_metadata_only_unsupported_and_image_topics_need_no_ros(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            output = Path(temporary) / "quality.json"
            _metadata(bag, {"/image": ("sensor_msgs/msg/Image", 31), "/other": ("unknown/msg/Custom", 5)})
            with patch(f"{MODULE}._open_reader") as reader:
                report = generate_capture_quality(bag, output)
            reader.assert_not_called()
            self.assertEqual(report["summary"]["metadata_only_topic_count"], 2)
            self.assertEqual(report["topics"]["/image"]["metadata_global_duration_rate_hz"], 31)
            self.assertIn("NOT per-topic", report["topics"]["/image"]["metadata_rate_basis"])
            self.assertEqual(json.loads(output.read_text()), report)

    def test_filtered_reader_skips_images_and_reports_malformed_payloads(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            _metadata(bag, {"/samples": (TACTILE_SAMPLE_TYPE, 2), "/image": ("sensor_msgs/msg/Image", 100)})
            events = [("/samples", b"good", SECOND), ("/samples", b"bad", SECOND + 1)]
            reader = NS(has_next=lambda: bool(events), read_next=lambda: events.pop(0), close=lambda: None)
            def deserialize(payload):
                if payload == b"bad":
                    raise ValueError("invalid CDR")
                return _batch(SECOND)
            with patch(f"{MODULE}._open_reader", return_value=reader), \
                 patch(f"{MODULE}._deserializer", return_value=deserialize), \
                 patch(f"{MODULE}._set_filter") as filtered:
                report = generate_capture_quality(bag, Path(temporary) / "report.json")
            filtered.assert_called_once_with(reader, ["/samples"])
            self.assertEqual(report["topics"]["/samples"]["malformed_messages"], 1)
            self.assertEqual(report["topics"]["/image"]["inspection"], "metadata_only")
            self.assertEqual(report["status"], "warnings")
            self.assertIn("malformed_messages", {item["code"] for item in report["warnings"]})

    def test_missing_message_support_is_explicit_metadata_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            _metadata(bag, {"/glove": (GLOVE_TYPE, 10)})
            with patch(f"{MODULE}._deserializer", side_effect=RuntimeError("source workspace")), \
                 patch(f"{MODULE}._open_reader") as reader:
                report = generate_capture_quality(bag, Path(temporary) / "report.json")
            reader.assert_not_called()
            self.assertEqual(report["topics"]["/glove"]["inspection"], "metadata_only")
            self.assertEqual(report["warnings"][0]["code"], "message_support_unavailable")

    def test_optional_config_only_warns_about_missing_topics_not_extra_topics(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "bag"
            _metadata(bag, {"/unexpected": ("std_msgs/msg/String", 1)})
            config = load_config(ROOT / "config" / "alignment.example.yaml")
            report = generate_capture_quality(bag, Path(temporary) / "report.json", config)
            missing = {item["topic"] for item in report["warnings"] if item["code"] == "expected_topic_missing"}
            self.assertEqual(missing, set(config.required_topics))
            self.assertIn("/unexpected", report["topics"])

    def test_refuses_overwrite_symlink_and_source_directory_without_modifying_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "bag"
            _metadata(bag, {"/image": ("sensor_msgs/msg/Image", 1)})
            output = root / "report.json"
            output.write_text("preserve")
            with self.assertRaises(FileExistsError):
                generate_capture_quality(bag, output)
            self.assertEqual(output.read_text(), "preserve")
            link = root / "dangling.json"
            link.symlink_to(root / "absent.json")
            with self.assertRaises(FileExistsError):
                generate_capture_quality(bag, link)
            with self.assertRaisesRegex(ValueError, "outside"):
                generate_capture_quality(bag, bag / "report.json")
            self.assertEqual(sorted(p.name for p in bag.iterdir()), ["metadata.yaml"])

    def test_atomic_collision_cleans_temporary_and_preserves_existing_destination(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "bag"
            _metadata(bag, {"/image": ("sensor_msgs/msg/Image", 1)})
            output = root / "report.json"
            def collision(source, destination):
                destination.write_text("concurrent report")
                raise FileExistsError("raced destination")
            with patch(f"{MODULE}.os.link", side_effect=collision):
                with self.assertRaises(FileExistsError):
                    generate_capture_quality(bag, output)
            self.assertEqual(output.read_text(), "concurrent report")
            self.assertEqual(sorted(p.name for p in root.iterdir()), ["bag", "report.json"])

    def test_empty_bag_reports_unknown_rate_and_warnings(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "bag"
            _metadata(bag, {"/image": ("sensor_msgs/msg/Image", 0)}, duration=0)
            report = generate_capture_quality(bag, root / "report.json")
            self.assertIsNone(report["topics"]["/image"]["metadata_global_duration_rate_hz"])
            self.assertIn("empty_bag", {item["code"] for item in report["warnings"]})


class RealMcapQualityTests(unittest.TestCase):
    def test_real_mcap_mixed_topics_filtered_report_and_file_safety(self):
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import Image
            from std_msgs.msg import String
            from vt_tactile_msgs.msg import TactileBlock, TactileSampleArray
        except ImportError as error:
            self.skipTest(f"ROS workspace unavailable: {error}")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root / "recording"
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                        rosbag2_py.ConverterOptions("", ""))
            definitions = {"/samples": TACTILE_SAMPLE_TYPE, "/image": "sensor_msgs/msg/Image",
                           "/other": "std_msgs/msg/String"}
            for index, (topic, type_name) in enumerate(definitions.items(), 1):
                writer.create_topic(rosbag2_py.TopicMetadata(
                    id=index, name=topic, type=type_name, serialization_format="cdr"))
            for index in range(3):
                ns = SECOND + index * 32_000_000
                message = TactileSampleArray()
                message.schema_version = 1
                message.header.stamp.sec = ns // SECOND
                message.header.stamp.nanosec = ns % SECOND
                message.hand, message.session_id = "right", "session"
                message.sequence, message.first_packet_index = index, index * 160
                message.host_realtime_ns, message.host_monotonic_ns = ns, ns + 100
                message.clock_domain, message.timing_source = "CLOCK_MONOTONIC", "host_receive"
                message.connected = message.time_valid = True
                message.blocks = [TactileBlock(row=row, column_group=group, adc=[0] * 10)
                                  for row in range(20) for group in range(8)]
                writer.write("/samples", serialize_message(message), ns)
                writer.write("/image", serialize_message(Image(height=1, width=1, step=1, data=[0])), ns + 1)
                writer.write("/other", serialize_message(String(data="untouched")), ns + 2)
            writer.close()
            before = {p.name: p.read_bytes() for p in bag.iterdir() if p.is_file()}
            report = generate_capture_quality(bag, root / "report.json")
            self.assertEqual(report["status"], "ok", report["warnings"])
            self.assertEqual(report["topics"]["/samples"]["tactile"]["observed_at_least_once_cells"], 400)
            self.assertEqual(report["topics"]["/samples"]["tactile"]["coverage_cycle_timing"]["positive_interval_rate_hz"], 31.25)
            self.assertEqual(report["topics"]["/image"]["inspection"], "metadata_only")
            self.assertEqual(before, {p.name: p.read_bytes() for p in bag.iterdir() if p.is_file()})


if __name__ == "__main__":
    unittest.main()
