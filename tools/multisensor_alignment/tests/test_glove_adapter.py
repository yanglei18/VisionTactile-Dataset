from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import patch

from vt_multisensor_alignment.bag_reader import configured_topic_types, read_unified_bag
from vt_multisensor_alignment.clock_audit import audit_clock_stream
from vt_multisensor_alignment.config import AdditionalStreamConfig, load_config
from vt_multisensor_alignment.glove_adapter import GLOVE_STATE_TYPE, GloveStateAdapter
from vt_multisensor_alignment.matcher import select_generic_sample
from vt_multisensor_alignment.model import MessageRef


ROOT = Path(__file__).resolve().parents[1]
TOPIC = "/gloves/right/state"


def stamp(value: int) -> NS:
    return NS(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def state(time_ns: int, **kwargs) -> NS:
    values = dict(
        header=NS(stamp=stamp(time_ns), frame_id="glove_right_reference"), hand="right",
        host_realtime_ns=time_ns, host_monotonic_ns=time_ns + 1_000,
        session_id="session-a", frame_index=0, calibration_version=1,
        pose_valid=True, time_valid=True, reason="", source_mode="live",
        clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
        reference_node_id=12, source_node_ids=[5, 6, 12],
        source_filter_epochs=[0, 0, 0], units_verified=False,
    )
    values.update(kwargs)
    values.setdefault("source_realtime_ns", [values["host_realtime_ns"]] * len(values["source_node_ids"]))
    values.setdefault("source_monotonic_ns", [values["host_monotonic_ns"]] * len(values["source_node_ids"]))
    return NS(**values)


def add(adapter, message, sequence=0):
    source = message.host_realtime_ns
    adapter.add(
        message, MessageRef(TOPIC, sequence, 100_000 + sequence, source),
        header_stamp_ns=message.header.stamp.sec * 1_000_000_000
        + message.header.stamp.nanosec,
    )


class GloveAdapterTests(unittest.TestCase):
    def test_duplicates_keep_last_result_without_changing_source_time(self):
        adapter = GloveStateAdapter("right_glove")
        add(adapter, state(100), 0)
        add(adapter, state(100, frame_index=1), 1)
        add(adapter, state(200, frame_index=2), 2)
        samples = adapter.finish()
        self.assertEqual([item.timestamp_ns for item in samples], [100, 200])
        self.assertEqual(samples[0].reference.sequence, 1)
        clocks = adapter.clock_observations["glove:right_glove:session-a"]
        self.assertEqual(len(clocks), 2)
        self.assertTrue(audit_clock_stream("glove", tuple(clocks), max_step_ns=0).valid)

    def test_invalid_pose_is_a_barrier_not_just_a_dropped_sample(self):
        for invalid in (
            state(150, pose_valid=False),
            state(150, source_mode="replay"),
            state(0, pose_valid=False, time_valid=False, host_monotonic_ns=0),
            state(150, time_valid=False),
            state(150, clock_domain="CLOCK_MONOTONIC_RAW"),
            state(150, session_id=""),
        ):
            with self.subTest(invalid=invalid):
                adapter = GloveStateAdapter("right_glove")
                add(adapter, state(100), 0)
                add(adapter, invalid, 1)
                add(adapter, state(200), 2)
                samples = adapter.finish()
                self.assertEqual(len(samples), 2)
                for strategy in ("nearest", "previous"):
                    for query in (101, 149, 151, 199):
                        self.assertIsNone(select_generic_sample(
                            samples, timestamp_ns=query, strategy=strategy,
                            max_delta_ns=1000,
                        ))
                    self.assertEqual(select_generic_sample(
                        samples, timestamp_ns=200, strategy=strategy,
                        max_delta_ns=1000,
                    ), 1)

    def test_session_calibration_or_filter_epoch_change_bounds_support(self):
        for changes in (
            {"session_id": "session-b"},
            {"calibration_version": 2},
            {"source_filter_epochs": [0, 1, 0]},
        ):
            with self.subTest(changes=changes):
                adapter = GloveStateAdapter("right_glove")
                add(adapter, state(100), 0)
                add(adapter, state(200, **changes), 1)
                samples = adapter.finish()
                self.assertIsNone(select_generic_sample(
                    samples, timestamp_ns=150, strategy="nearest", max_delta_ns=1000,
                ))

    def test_last_invalid_event_prevents_holding_previous_pose(self):
        adapter = GloveStateAdapter("right_glove")
        add(adapter, state(100))
        add(adapter, state(0, pose_valid=False, time_valid=False, host_monotonic_ns=0))
        self.assertIsNone(select_generic_sample(
            adapter.finish(), timestamp_ns=101, strategy="previous", max_delta_ns=1000,
        ))

    def test_initial_invalid_event_prevents_selecting_future_valid_pose(self):
        adapter = GloveStateAdapter("right_glove")
        add(adapter, state(0, pose_valid=False, time_valid=False, host_monotonic_ns=0))
        add(adapter, state(100))
        self.assertIsNone(select_generic_sample(
            adapter.finish(), timestamp_ns=99, strategy="nearest", max_delta_ns=1000,
        ))

    def test_recalibration_with_same_source_does_not_create_duplicate_time(self):
        adapter = GloveStateAdapter("right_glove")
        add(adapter, state(100), 0)
        add(adapter, state(100, calibration_version=2), 1)
        samples = adapter.finish()
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0].reference.sequence, 1)

    def test_clock_audit_is_separate_by_session_and_detects_real_clock_step(self):
        adapter = GloveStateAdapter("right_glove")
        add(adapter, state(100, host_monotonic_ns=50))
        add(adapter, state(200, host_monotonic_ns=60))
        add(adapter, state(300, host_monotonic_ns=1, session_id="session-b"))
        audits = [audit_clock_stream(key, tuple(values), max_step_ns=5)
                  for key, values in adapter.clock_observations.items()]
        self.assertEqual(len(audits), 2)
        self.assertEqual(audits[0].violations[0].reason, "clock_step")
        self.assertTrue(audits[1].valid)

    def test_same_realtime_with_different_monotonic_is_not_hidden_by_dedup(self):
        adapter = GloveStateAdapter("right_glove")
        add(adapter, state(100, host_monotonic_ns=50))
        add(adapter, state(100, host_monotonic_ns=60))
        clock = tuple(adapter.clock_observations["glove:right_glove:session-a"])
        self.assertFalse(audit_clock_stream("glove", clock, max_step_ns=1000).valid)

    def test_rejects_mismatched_header_and_backward_source(self):
        adapter = GloveStateAdapter("right_glove")
        with self.assertRaisesRegex(ValueError, "timestamp mismatch"):
            add(adapter, state(100, header=NS(stamp=stamp(99))))
        add(adapter, state(100))
        with self.assertRaisesRegex(ValueError, "not increasing"):
            add(adapter, state(99))


class GloveBagReaderTests(unittest.TestCase):
    def test_real_glove_message_cdr_roundtrip_uses_source_clock(self):
        try:
            from rclpy.serialization import deserialize_message, serialize_message
            from vt_glove_msgs.msg import GloveState
        except ImportError as error:
            self.skipTest(f"ROS glove interfaces are not sourced: {error}")
        original = GloveState()
        original.header.stamp.sec = 1
        original.header.stamp.nanosec = 25
        original.host_realtime_ns = 1_000_000_025
        original.host_monotonic_ns = 300_000_000
        original.clock_domain = "CLOCK_MONOTONIC"
        original.timing_source = "host_receive"
        original.reference_node_id = 12
        original.source_mode = "live"
        original.session_id = "cdr-test"
        original.pose_valid = True
        original.time_valid = True
        original.calibration_version = 3
        original.source_node_ids = [5, 6, 12]
        original.source_filter_epochs = [0, 1, 0]
        original.source_realtime_ns = [1_000_000_023, 1_000_000_024, 1_000_000_025]
        original.source_monotonic_ns = [299_999_998, 299_999_999, 300_000_000]
        original.source_sample_indices = [101, 102, 103]
        decoded = deserialize_message(serialize_message(original), GloveState)
        adapter = GloveStateAdapter("right_glove")
        add(adapter, decoded, 7)
        sample = adapter.finish()[0]
        self.assertEqual(sample.timestamp_ns, 1_000_000_025)
        self.assertEqual(sample.reference.sequence, 7)
        clock = adapter.clock_observations["glove:right_glove:cdr-test"][0]
        self.assertEqual(clock.monotonic_ns, 300_000_000)

    def test_reader_retains_valid_unique_glove_results_and_clock_audit(self):
        config = load_config(ROOT / "config" / "alignment.example.yaml")
        config = replace(config, additional_streams=(AdditionalStreamConfig(
            name="right_glove", topic=TOPIC, type_name=GLOVE_STATE_TYPE,
            time_source="field", timestamp_field="host_realtime_ns",
            strategy="nearest", max_delta_ms=20, required=False,
        ),))
        events = []
        for camera in config.cameras:
            header = NS(stamp=stamp(100), frame_id=camera.frame_id)
            for topic in (camera.color_topic, camera.depth_topic, camera.camera_info_topic):
                events.append((topic, NS(header=header), 100))
            events.append((camera.timing_topic, NS(
                header=header, camera_name=camera.name, camera_model=camera.model,
                serial_number=camera.serial, GROUP_VALID_COMMON_STAMP=1,
                GROUP_VALID_IDENTITY=2, GROUP_VALID_CALLBACK_CLOCKS=8,
                group_validity_flags=11, shared_ros_timestamp_ns=100,
                group_host_realtime_ns=100, group_host_monotonic_raw_ns=10,
            ), 100))
        for tracker in config.trackers:
            for time_ns in (90, 110):
                events.append((tracker.sample_topic, NS(
                    header=NS(stamp=stamp(time_ns), frame_id=config.world_frame),
                    role=tracker.role, tracker_id="a" * 64,
                    host_realtime_ns=time_ns, host_monotonic_ns=time_ns + 100,
                    pose_valid=True, tracking_status=2,
                    pose=NS(position=NS(x=0, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)),
                ), time_ns))
        for message in (state(100), state(100), state(0, pose_valid=False, time_valid=False), state(200)):
            events.append((TOPIC, message, 1000 + len(events)))
        class Reader:
            def get_all_topics_and_types(self):
                return [NS(name=name, type=kind) for name, kind in configured_topic_types(config).items()]
            def has_next(self):
                return bool(events)
            def read_next(self):
                return events.pop(0)
        with tempfile.TemporaryDirectory() as directory, \
             patch("vt_multisensor_alignment.bag_reader._open_reader", return_value=Reader()), \
             patch("vt_multisensor_alignment.bag_reader._deserializer", return_value=lambda value: value), \
             patch("vt_multisensor_alignment.bag_reader._storage_identifier", return_value="mcap"):
            dataset = read_unified_bag(directory, config)
        self.assertEqual(dataset.message_counts[TOPIC], 4)
        self.assertEqual(dataset.accepted_counts[TOPIC], 2)
        self.assertEqual(len(dataset.clock_observations["glove:right_glove:session-a"]), 2)
        self.assertIsNone(select_generic_sample(
            dataset.additional_samples["right_glove"], timestamp_ns=150,
            strategy="nearest", max_delta_ns=1000,
        ))


if __name__ == "__main__":
    unittest.main()
