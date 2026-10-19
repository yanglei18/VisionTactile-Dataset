from dataclasses import replace
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from vt_multisensor_alignment.bag_reader import configured_topic_types, read_unified_bag
from vt_multisensor_alignment.cli import main
from vt_multisensor_alignment.config import AdditionalStreamConfig, load_config
from vt_multisensor_alignment.matcher import select_generic_sample
from vt_multisensor_alignment.model import MessageRef
from vt_multisensor_alignment.tactile_adapter import (
    DEFAULT_TACTILE_TOPIC as TOPIC, TACTILE_SAMPLE_TYPE, TactileContinuity,
    TactileGrid, TactileSampleAdapter,
)
from vt_multisensor_alignment.tactile_export import (
    export_tactile, iter_tactile_events, reconstruct_frames,
)


ROOT = Path(__file__).resolve().parents[1]


def stamp(value):
    return NS(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def block(row=0, group=0, adc=None):
    return NS(row=row, column_group=group, status_raw=255,
              adc=list(range(10)) if adc is None else adc)


def batch(time_ns, sequence=0, **updates):
    fields = dict(
        header=NS(stamp=stamp(time_ns), frame_id="right_tactile"),
        schema_version=1, hand="right", session_id="session-a", stream_epoch=0,
        sequence=sequence, first_packet_index=sequence,
        host_realtime_ns=time_ns, host_monotonic_ns=time_ns + 1000,
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        connected=True, time_valid=True, reason="", producer_dropped_total=0,
        discarded_bytes_total=0, malformed_packets_total=0, blocks=[block()],
    )
    fields.update(updates)
    return NS(**fields)


def decode(continuity, message, index=0):
    return continuity.decode(message, MessageRef(TOPIC, index, 10_000 + index,
                                                message.host_realtime_ns),
                             header_stamp_ns=message.header.stamp.sec * 1_000_000_000
                             + message.header.stamp.nanosec)


def events(*messages):
    continuity = TactileContinuity()
    return [decode(continuity, value, index) for index, value in enumerate(messages)]


class TactileAdapterTests(unittest.TestCase):
    def test_entire_legacy_wire_scan_crops_exactly_without_resampling(self):
        blocks = [block(row, group, [row * 80 + group + 8 * k for k in range(10)])
                  for row in range(80) for group in range(8)]
        event = events(batch(100, blocks=blocks))[0]
        self.assertEqual(len(event.blocks), 640)
        grid = TactileGrid()
        grid.apply(event)
        result = grid.snapshot(100, 0)
        self.assertEqual(result["valid_mask"].sum(), 400)
        np.testing.assert_array_equal(result["adc"],
                                      np.arange(6400).reshape(80, 80)[:20, :20])
        self.assertEqual(result["block_packet_index"][19, 7], 159)

    def test_outside_roi_packets_preserve_wire_continuity_without_refreshing(self):
        source = events(
            batch(100, blocks=[block(20), block(19, 3), block(79)]),
            batch(120, 1, first_packet_index=3, blocks=[block(20), block(79)]),
            batch(140, 2, first_packet_index=5, blocks=[block(19, 7)]),
        )
        self.assertTrue(all(event.usable and not event.barriers for event in source))
        grid = TactileGrid()
        grid.apply(source[0])
        self.assertEqual(grid.block_packet_index[19, 3], 1)
        grid.apply(source[1])
        self.assertEqual(grid.block_realtime_ns[19, 3], 100)
        self.assertEqual(grid.last_contributing_event.sequence, 0)
        self.assertEqual(grid.last_event.sequence, 1)
        self.assertFalse(grid.snapshot(120, 10)["valid_mask"].any())
        self.assertEqual(grid.snapshot(120, 100)["valid_mask"].sum(), 3)
        grid.apply(source[2])
        self.assertEqual(grid.snapshot(140, 100)["valid_mask"].sum(), 5)
        self.assertEqual(grid.block_packet_index[19, 7], 5)

    def test_outside_roi_loss_still_invalidates_physical_cells(self):
        source = events(batch(100), batch(120, 2, first_packet_index=2,
                                         blocks=[block(79)]))
        grid = TactileGrid()
        for event in source:
            grid.apply(event)
        self.assertFalse(grid.snapshot(120, 100)["valid_mask"].any())
        self.assertIsNone(grid.last_contributing_event)
        self.assertIn("sequence_gap", grid.last_barrier["barriers"])

    def test_stride_zero_partial_age_and_source_indices(self):
        grid = TactileGrid()
        grid.apply(events(batch(100, blocks=[block(19, 7)]))[0])
        result = grid.snapshot(110, 10)
        self.assertEqual(result["adc"].shape, (20, 20))
        self.assertEqual(result["block_realtime_ns"].shape, (20, 8))
        self.assertEqual(result["valid_mask"].sum(), 2)
        self.assertTrue(result["valid_mask"][19, 7])
        self.assertEqual(result["adc"][19, 7], 0)
        np.testing.assert_array_equal(result["adc"][19, 7::8], np.arange(2))
        np.testing.assert_array_equal(result["age_ns"][19, 7::8], np.full(2, 10))
        self.assertEqual(result["age_ns"][0, 0], -1)
        self.assertFalse(grid.snapshot(111, 10)["valid_mask"].any())
        self.assertEqual(result["block_message_sequence"][19, 7], 0)
        self.assertEqual(result["block_packet_index"][19, 7], 0)

    def test_previous_accumulates_distinct_blocks_instead_of_10_point_hand(self):
        source = events(batch(100), batch(120, 1, blocks=[block(1, 1)]))
        values = list(reconstruct_frames(source, [(0, 99), (1, 110), (2, 125), (3, 131)], max_age_ns=30))
        self.assertEqual([int(value[0]["valid_mask"].sum()) for value in values], [0, 3, 6, 3])
        self.assertEqual(values[2][0]["age_ns"][0, 0], 25)
        self.assertEqual(values[2][0]["age_ns"][1, 1], 5)

    def test_wire_order_last_update_wins_within_one_batch(self):
        grid = TactileGrid()
        grid.apply(events(batch(100, blocks=[block(), block(adc=[42] * 10)]))[0])
        result = grid.snapshot(100, 0)
        np.testing.assert_array_equal(result["adc"][0, ::8], [42] * 3)
        self.assertEqual(result["block_packet_index"][0, 0], 1)

    def test_discontinuities_bound_gap_and_clear_previous_groups(self):
        for update in (
            dict(session_id="session-b"), dict(stream_epoch=1), dict(sequence=3),
            dict(first_packet_index=3), dict(producer_dropped_total=1),
            dict(discarded_bytes_total=26), dict(malformed_packets_total=1),
        ):
            with self.subTest(update=update):
                second = batch(200, 1, blocks=[block(1, 1)])
                for name, value in update.items():
                    setattr(second, name, value)
                values = list(reconstruct_frames(events(batch(100), second),
                                                [(0, 100), (1, 150), (2, 200)], max_age_ns=1000))
                self.assertEqual([int(x[0]["valid_mask"].sum()) for x in values], [3, 0, 3])
                self.assertIsNotNone(values[1][1]["pending_barrier"])
                self.assertFalse(values[2][0]["valid_mask"][0, 0])
                self.assertEqual(values[2][1]["barrier_count"], 1)

    def test_disconnect_status_does_not_mean_zero_touch_or_carry_old_cache(self):
        source = events(batch(100), batch(150, 1, blocks=[], connected=False, reason="disconnect"),
                        batch(200, 2, first_packet_index=1, blocks=[block(1, 1)]))
        values = list(reconstruct_frames(source, [(0, 151), (1, 200)], max_age_ns=1000))
        self.assertFalse(values[0][0]["valid_mask"].any())
        self.assertEqual(values[1][0]["valid_mask"].sum(), 3)
        self.assertFalse(values[1][0]["valid_mask"][0, 0])

    def test_initial_empty_event_and_new_session_clock_origin(self):
        source = events(
            batch(50, blocks=[], connected=True, time_valid=False, reason="connected"),
            batch(100, 1, first_packet_index=0),
            batch(200, 0, session_id="session-b", first_packet_index=0,
                  host_monotonic_ns=1, blocks=[block(2, 2)]),
        )
        values = list(reconstruct_frames(source, [(0, 75), (1, 100), (2, 175), (3, 200)], max_age_ns=1000))
        self.assertEqual([int(x[0]["valid_mask"].sum()) for x in values], [0, 3, 0, 3])
        self.assertEqual(values[-1][1]["last_source_event"]["session_id"], "session-b")
        self.assertEqual(values[-1][0]["block_monotonic_ns"][2, 2], 1)
        self.assertFalse(values[-1][0]["valid_mask"][0, 0])

    def test_terminal_drop_status_invalidates_holding_previous_data(self):
        source = events(batch(100), batch(150, 3, first_packet_index=2,
                                         blocks=[], producer_dropped_total=1,
                                         time_valid=False, reason="queue_gap"))
        values = list(reconstruct_frames(source, [(0, 125), (1, 150), (2, 175)], max_age_ns=1000))
        self.assertTrue(all(not item[0]["valid_mask"].any() for item in values))
        self.assertIn("producer_dropped_total", values[-1][1]["last_barrier"]["barriers"])

    def test_regressed_batch_sequence_cannot_replace_newer_grid(self):
        source = events(batch(100, 10), batch(150, 9, blocks=[block(2, 2)]))
        self.assertFalse(source[-1].usable)
        arrays, metadata = next(reconstruct_frames(source, [(0, 150)], max_age_ns=1000))
        self.assertFalse(arrays["valid_mask"].any())
        self.assertIn("sequence_gap", metadata["last_barrier"]["barriers"])

    def test_counter_highwater_survives_rejected_events(self):
        source = events(batch(100, 10, stream_epoch=2),
                        batch(150, 9, stream_epoch=1),
                        batch(200, 10, stream_epoch=2),
                        batch(250, 11, stream_epoch=2))
        self.assertEqual([event.usable for event in source], [True, False, False, True])
        self.assertIn("epoch_regression", source[1].barriers)
        self.assertIn("nonincreasing_sequence", source[2].barriers)

    def test_regressed_packet_index_is_not_accepted_as_fresh_data(self):
        source = events(batch(100, 0, first_packet_index=10),
                        batch(150, 1, first_packet_index=5))
        self.assertFalse(source[-1].usable)
        self.assertIn("packet_index_regression", source[-1].barriers)

    def test_duplicate_reversed_and_bad_clocks_are_barriers(self):
        for middle in (
            batch(100, 1), batch(90, 1), batch(150, 1, host_monotonic_ns=1090),
            batch(150, 1, time_valid=False), batch(150, 1, clock_domain="unknown"),
            batch(150, 1, timing_source="publish_time"),
            batch(150, 1, host_monotonic_ns=1_000_000_000),
        ):
            with self.subTest(middle=middle):
                source = events(batch(100), middle)
                self.assertFalse(source[-1].usable)
                self.assertTrue(source[-1].barriers)
                arrays, _ = next(reconstruct_frames(source, [(0, 160)], max_age_ns=1000))
                self.assertFalse(arrays["valid_mask"].any())

    def test_adapter_support_bounds_and_no_dense_payload_retention(self):
        adapter = TactileSampleAdapter("right_tactile")
        for index, value in enumerate((batch(100), batch(200, 3), batch(250, 4))):
            adapter.add(value, MessageRef(TOPIC, index, 1000 + index, value.host_realtime_ns),
                        header_stamp_ns=value.host_realtime_ns)
        samples = adapter.finish()
        self.assertEqual(len(samples), 3)
        self.assertEqual(samples[0].valid_until_ns, 100)
        self.assertEqual(samples[1].valid_from_ns, 200)
        self.assertIsNone(select_generic_sample(samples, timestamp_ns=150, strategy="previous", max_delta_ns=1000))
        self.assertFalse(hasattr(samples[0], "adc"))

    def test_contract_rejects_malformed_blocks_and_header(self):
        for message in (
            batch(100, schema_version=2), batch(100, hand="front"),
            batch(100, sequence=-1), batch(100, time_valid=1),
            batch(100, blocks=[block(80)]), batch(100, blocks=[block(group=8)]),
            batch(100, blocks=[block(adc=[1] * 9)]),
            batch(100, blocks=[block(adc=[65536] * 10)]),
            batch(100, header=NS(stamp=stamp(99))),
        ):
            with self.subTest(message=message), self.assertRaises(ValueError):
                decode(TactileContinuity(), message)


class TactileExportTests(unittest.TestCase):
    def test_metadata_distinguishes_raw_progress_from_physical_progress(self):
        source = events(batch(100), batch(120, 1, blocks=[block(79)]))
        arrays, metadata = next(reconstruct_frames(source, [(0, 125)], max_age_ns=100))
        self.assertEqual(metadata["last_source_event"]["host_realtime_ns"], 120)
        self.assertEqual(metadata["last_contributing_event"]["host_realtime_ns"], 100)
        self.assertEqual(arrays["age_ns"][0, 0], 25)

    def test_streaming_consumes_only_one_lookahead_and_no_future_values(self):
        consumed = []
        def stream():
            continuity = TactileContinuity()
            for i in range(1000):
                consumed.append(i)
                yield decode(continuity, batch(100 + i * 10, i), i)
        reconstructed = reconstruct_frames(stream(), [(0, 105), (1, 115)], max_age_ns=100)
        first, _ = next(reconstructed)
        self.assertEqual(consumed, [0, 1])
        self.assertEqual(first["block_realtime_ns"][0, 0], 100)
        next(reconstructed)
        self.assertEqual(consumed, [0, 1, 2])

    def test_export_npz_manifest_metadata_and_cli(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            aligned = path / "aligned_frames.jsonl"
            aligned.write_text(''.join(json.dumps({"frame_index": i, "reference_time_ns": t}) + '\n'
                                       for i, t in enumerate((110, 150, 205))), encoding="utf-8")
            source = events(batch(100), batch(200, 3, blocks=[block(1, 1)]))
            with patch("vt_multisensor_alignment.tactile_export.iter_tactile_events", return_value=iter(source)):
                status = main(["export-tactile", "--bag", str(path), "--aligned-frames", str(aligned),
                               "--output", str(path / "out"), "--max-age-ms", "0.0001"])
            self.assertEqual(status, 0)
            manifest = json.loads((path / "out/manifest.json").read_text())
            self.assertEqual(manifest["frame_count"], 3)
            self.assertEqual(manifest["schema_version"], 2)
            self.assertEqual((manifest["rows"], manifest["cols"]), (20, 20))
            self.assertEqual((manifest["wire_rows"], manifest["wire_cols"]), (80, 80))
            self.assertEqual(manifest["source_schema_version"], 1)
            self.assertFalse(manifest["hardware_synchronized"])
            metadata = [json.loads(line) for line in (path / "out/source_metadata.jsonl").read_text().splitlines()]
            self.assertIn("sequence_gap", metadata[1]["pending_barrier"]["barriers"])
            with np.load(path / "out/frames/000000002.npz", allow_pickle=False) as arrays:
                self.assertEqual(int(arrays["timestamp_ns"]), 205)
                self.assertEqual(arrays["adc"].shape, (20, 20))
                self.assertEqual(arrays["age_ns"].shape, (20, 20))
                self.assertEqual(arrays["block_realtime_ns"].shape, (20, 8))
                self.assertEqual(arrays["valid_mask"].sum(), 3)
                self.assertEqual(arrays["age_ns"][1, 1], 5)
            with self.assertRaises(FileExistsError):
                export_tactile(bag_path=path, aligned_frames=aligned, output_directory=path / "out")

    def test_export_failure_leaves_no_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            aligned = path / "aligned_frames.jsonl"
            aligned.write_text('{"frame_index":0,"reference_time_ns":100}\ninvalid\n')
            with patch("vt_multisensor_alignment.tactile_export.iter_tactile_events", return_value=iter(events(batch(100)))), \
                 self.assertRaises(ValueError):
                export_tactile(bag_path=path, aligned_frames=aligned, output_directory=path / "out")
            self.assertFalse((path / "out").exists())
            self.assertEqual(list(path.glob(".out.*")), [])

    def test_profile_preserves_required_glove_and_optional_previous_tactile(self):
        original = load_config(ROOT / "config/alignment.right_glove.yaml")
        combined = load_config(ROOT / "config/alignment.right_glove_tactile.yaml")
        self.assertEqual(combined.cameras, original.cameras)
        self.assertEqual(combined.trackers, original.trackers)
        self.assertEqual(combined.additional_streams[0], original.additional_streams[0])
        tactile = combined.additional_streams[1]
        self.assertEqual(tactile.type_name, TACTILE_SAMPLE_TYPE)
        self.assertEqual(tactile.strategy, "previous")
        self.assertFalse(tactile.required)


class TactileBagTests(unittest.TestCase):
    def test_unified_reader_uses_typed_barriers_not_generic_timestamps(self):
        config = load_config(ROOT / "config/alignment.example.yaml")
        config = replace(config, additional_streams=(AdditionalStreamConfig(
            "right_tactile", TOPIC, TACTILE_SAMPLE_TYPE, "field", "host_realtime_ns", "previous", 100, False,
        ),))
        records = []
        for camera in config.cameras:
            header = NS(stamp=stamp(100), frame_id=camera.frame_id)
            for topic in (camera.color_topic, camera.depth_topic, camera.camera_info_topic):
                records.append((topic, NS(header=header), 100))
            records.append((camera.timing_topic, NS(
                header=header, camera_name=camera.name, camera_model=camera.model,
                serial_number=camera.serial, GROUP_VALID_COMMON_STAMP=1,
                GROUP_VALID_IDENTITY=2, GROUP_VALID_CALLBACK_CLOCKS=8,
                group_validity_flags=11, shared_ros_timestamp_ns=100,
                group_host_realtime_ns=100, group_host_monotonic_raw_ns=10,
            ), 100))
        for tracker in config.trackers:
            for time_ns in (90, 110):
                records.append((tracker.sample_topic, NS(
                    header=NS(stamp=stamp(time_ns), frame_id=config.world_frame),
                    role=tracker.role, tracker_id="a" * 64,
                    host_realtime_ns=time_ns, host_monotonic_ns=time_ns + 100,
                    pose_valid=True, tracking_status=2,
                    pose=NS(position=NS(x=0, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)),
                ), time_ns))
        for value in (batch(100), batch(150, 1, connected=False, blocks=[]), batch(200, 2, first_packet_index=1)):
            records.append((TOPIC, value, 1000 + len(records)))
        class Reader:
            def get_all_topics_and_types(self):
                return [NS(name=name, type=kind) for name, kind in configured_topic_types(config).items()]
            def has_next(self):
                return bool(records)
            def read_next(self):
                return records.pop(0)
        with tempfile.TemporaryDirectory() as directory, \
             patch("vt_multisensor_alignment.bag_reader._open_reader", return_value=Reader()), \
             patch("vt_multisensor_alignment.bag_reader._deserializer", return_value=lambda value: value), \
             patch("vt_multisensor_alignment.bag_reader._storage_identifier", return_value="mcap"):
            dataset = read_unified_bag(directory, config)
        self.assertEqual(dataset.message_counts[TOPIC], 3)
        self.assertEqual(dataset.accepted_counts[TOPIC], 2)
        samples = dataset.additional_samples["right_tactile"]
        self.assertIsNone(select_generic_sample(samples, timestamp_ns=175, strategy="previous", max_delta_ns=1000))
        self.assertEqual(samples[-1].reference.sequence, 2)

    def test_real_ros_synthetic_mcap_sparse_roundtrip(self):
        try:
            import rosbag2_py
            from rclpy.serialization import serialize_message
            from vt_tactile_msgs.msg import TactileBlock, TactileSampleArray
        except ImportError as error:
            self.skipTest(f"ROS tactile interfaces are not sourced: {error}")
        with tempfile.TemporaryDirectory() as directory:
            bag = Path(directory) / "synthetic"
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                        rosbag2_py.ConverterOptions("", ""))
            writer.create_topic(rosbag2_py.TopicMetadata(id=1, name=TOPIC, type=TACTILE_SAMPLE_TYPE,
                                                        serialization_format="cdr"))
            for index, time_ns in enumerate((100, 200)):
                source = batch(time_ns, index)
                message = TactileSampleArray()
                for name, value in vars(source).items():
                    if name not in {"header", "blocks"}:
                        setattr(message, name, value)
                message.header.stamp.nanosec = time_ns
                message.header.frame_id = source.header.frame_id
                tactile = TactileBlock()
                tactile.row = index
                tactile.column_group = 7
                tactile.adc = list(range(10))
                message.blocks = [tactile]
                writer.write(TOPIC, serialize_message(message), 1000 + index)
            writer.close()
            result = list(reconstruct_frames(iter_tactile_events(bag), [(0, 205)], max_age_ns=1000))
            self.assertEqual(result[0][0]["valid_mask"].sum(), 4)
            self.assertEqual(result[0][1]["last_source_event"]["message"]["bag_timestamp_ns"], 1001)


if __name__ == "__main__":
    unittest.main()
