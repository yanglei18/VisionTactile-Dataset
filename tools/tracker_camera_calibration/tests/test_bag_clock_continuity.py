"""Bag-order clock validation with in-memory readers, without ROS or hardware."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from vt_tracker_camera_calib.bag_reader import read_calibration_bag
from vt_tracker_camera_calib.clock import HostClockContinuity, MAX_CLOCK_STEP_NS
from vt_tracker_camera_calib.config import load_config
from vt_tracker_camera_calib.model import BoardObservation, CameraIntrinsics
from vt_tracker_camera_calib.transforms import Transform


ROOT = Path(__file__).resolve().parents[1]


class Reader:
    def __init__(self, records, types):
        self.records = list(records)
        self.types = types

    def has_next(self):
        return bool(self.records)

    def read_next(self):
        topic, message = self.records.pop(0)
        return topic, message, 0

    def get_all_topics_and_types(self):
        return [NS(name=name, type=kind) for name, kind in self.types.items()]


class BagClockContinuityTest(unittest.TestCase):
    def setUp(self):
        self.config = replace(
            load_config(ROOT / "config" / "calibration.example.yaml"),
            fallback_intrinsics=CameraIntrinsics(
                2, 2, np.array([[100., 0., 1.], [0., 100., 1.], [0., 0., 1.]]), np.zeros(5)
            ),
        )
        self.types = {
            self.config.image_topic: "sensor_msgs/msg/Image",
            self.config.timing_topic: "vt_camera_msgs/msg/CameraFrameTiming",
            self.config.tracker_sample_topic: "vt_tracker_msgs/msg/TrackerSample",
        }

    def tracker(self, realtime_ns=1_000_000_000, monotonic_ns=100_000_000, valid=True):
        return NS(
            role=self.config.tracker_role, header=NS(frame_id=self.config.world_frame),
            pose_valid=valid, tracking_status=2, tracker_id="a" * 64,
            host_realtime_ns=realtime_ns, host_monotonic_ns=monotonic_ns,
            pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)),
        )

    def timing(self, stamp=1, realtime_ns=1_005_000_000, monotonic_ns=30_000_000_000):
        return NS(
            camera_name=self.config.camera_name, camera_model=self.config.camera_model,
            serial_number=self.config.camera_serial,
            GROUP_VALID_COMMON_STAMP=1, GROUP_VALID_IDENTITY=2, GROUP_VALID_CALLBACK_CLOCKS=8,
            group_validity_flags=11, shared_ros_timestamp_ns=stamp * 1_000_000_000,
            group_host_realtime_ns=realtime_ns, group_host_monotonic_raw_ns=monotonic_ns,
        )

    def read(self, tracker_messages=None, timing_messages=None):
        if tracker_messages is None:
            tracker_messages = [self.tracker(), self.tracker(1_010_000_000, 110_000_000)]
        if timing_messages is None:
            timing_messages = [self.timing()]
        records = [(self.config.tracker_sample_topic, message) for message in tracker_messages]
        records.extend((self.config.timing_topic, message) for message in timing_messages)
        image = NS(
            header=NS(frame_id=self.config.camera_frame, stamp=NS(sec=1, nanosec=0)),
            width=2, height=2, step=2, encoding="mono8", data=bytes(4),
        )
        readers = [Reader(records, self.types), Reader([(self.config.image_topic, image)], self.types)]

        def detect(_image, **kwargs):
            return BoardObservation(
                kwargs["timestamp_ns"], Transform.identity(), 0.1, 12, kwargs["source_stamp_ns"]
            )

        with tempfile.TemporaryDirectory() as directory, \
                patch("vt_tracker_camera_calib.bag_reader._open_reader", side_effect=readers), \
                patch("vt_tracker_camera_calib.bag_reader._deserializer", return_value=lambda value: value), \
                patch("vt_tracker_camera_calib.bag_reader.detect_board_pose", side_effect=detect):
            return read_calibration_bag(directory, self.config)

    def test_preserves_stream_local_monotonic_domains(self):
        result = self.read()
        self.assertEqual(result.tracker_samples[0].host_monotonic_ns, 100_000_000)
        self.assertEqual(result.board_observations[0].host_monotonic_raw_ns, 30_000_000_000)
        self.assertEqual(result.board_observations[0].timestamp_ns, 1_005_000_000)

    def test_rejects_tracker_realtime_rollback_before_sorting(self):
        with self.assertRaisesRegex(ValueError, "TrackerSample: non-increasing host clock"):
            self.read([self.tracker(), self.tracker(990_000_000, 110_000_000)])

    def test_rejects_tracker_monotonic_reset(self):
        with self.assertRaisesRegex(ValueError, "TrackerSample: non-increasing host clock"):
            self.read([self.tracker(), self.tracker(1_010_000_000, 1_000_000)])

    def test_rejects_forward_realtime_step(self):
        with self.assertRaisesRegex(ValueError, "host clock step exceeds 5 ms"):
            self.read([self.tracker(), self.tracker(1_020_000_000, 110_000_000)])

    def test_invalid_pose_cannot_hide_clock_reset(self):
        with self.assertRaisesRegex(ValueError, "TrackerSample: non-increasing host clock"):
            self.read([
                self.tracker(), self.tracker(990_000_000, 105_000_000, valid=False),
                self.tracker(1_010_000_000, 110_000_000),
            ])

    def test_rejects_camera_realtime_rollback(self):
        with self.assertRaisesRegex(ValueError, "CameraFrameTiming: non-increasing host clock"):
            self.read(timing_messages=[self.timing(), self.timing(2, 1_000_000_000, 30_010_000_000)])

    def test_rejects_camera_forward_clock_step(self):
        with self.assertRaisesRegex(ValueError, "CameraFrameTiming: host clock step exceeds 5 ms"):
            self.read(timing_messages=[self.timing(), self.timing(2, 1_100_000_000, 30_010_000_000)])

    def test_identical_camera_mapping_can_be_repeated(self):
        result = self.read(timing_messages=[self.timing(), self.timing()])
        self.assertEqual(result.timed_image_count, 1)

    def test_duplicate_image_stamp_with_different_monotonic_time_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "conflicting timing observations"):
            self.read(timing_messages=[self.timing(), self.timing(monotonic_ns=30_010_000_000)])

    def test_missing_clock_measurement_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "host clocks must be positive integers"):
            self.read(timing_messages=[self.timing(monotonic_ns=0)])

    def test_clock_gap_is_valid_when_both_domains_advance_together(self):
        clock = HostClockContinuity("test")
        clock.observe(1_000_000_000, 100_000_000)
        clock.observe(11_000_000_000, 10_100_000_000)

    def test_clock_step_threshold_is_inclusive(self):
        clock = HostClockContinuity("test")
        clock.observe(1_000_000_000, 100_000_000)
        clock.observe(1_010_000_000 + MAX_CLOCK_STEP_NS, 110_000_000)
        with self.assertRaisesRegex(ValueError, "host clock step exceeds 5 ms"):
            clock.observe(1_020_000_000 + 2 * MAX_CLOCK_STEP_NS + 1, 120_000_000)


if __name__ == "__main__":
    unittest.main()
