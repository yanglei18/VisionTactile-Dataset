"""Pure message/clock tests and fake ROS lifecycle; never opens a device."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from vt_tracker_camera_calib.charuco import create_board
from vt_tracker_camera_calib.config import load_config
from vt_tracker_camera_calib.gui_live import LiveMonitor, _image_rgb, _load_ros
from vt_tracker_camera_calib.model import BoardObservation
from vt_tracker_camera_calib.transforms import Transform


ROOT = Path(__file__).resolve().parents[1]


class Clock:
    value = 10.0

    def __call__(self):
        return self.value


def image(config, stamp=1, encoding="rgb8", width=2, height=2):
    channels = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(encoding, 3)
    return NS(header=NS(frame_id=config.camera_frame, stamp=NS(sec=stamp, nanosec=0)),
              width=width, height=height, encoding=encoding, step=width * channels,
              data=bytes(range(width * height * channels)))


def info(config, width=2, height=2):
    return NS(header=NS(frame_id=config.camera_frame), width=width, height=height,
              k=[100., 0., width / 2., 0., 100., height / 2., 0., 0., 1.],
              d=[0.] * 5, distortion_model="plumb_bob")


def timing(config, stamp=1, host=100):
    return NS(camera_name=config.camera_name, camera_model=config.camera_model,
              serial_number=config.camera_serial, GROUP_VALID_COMMON_STAMP=1,
              GROUP_VALID_IDENTITY=2, GROUP_VALID_CALLBACK_CLOCKS=8,
              group_validity_flags=11, shared_ros_timestamp_ns=stamp * 1_000_000_000,
              group_host_realtime_ns=host, color_device_timestamp_ns=-999,
              depth_device_timestamp_ns=123)


def tracker(config):
    return NS(role=config.tracker_role, header=NS(frame_id=config.world_frame),
              pose_valid=True, tracking_status=0x12, tracker_id="a" * 64,
              host_realtime_ns=100,
              pose=NS(position=NS(x=.1, y=.2, z=.3), orientation=NS(x=0., y=0., z=0., w=2.)))


def observation(_image, **kwargs):
    return BoardObservation(kwargs["timestamp_ns"], Transform.identity(), .2, 12,
                            kwargs["source_stamp_ns"])


def fake_ros(*, fail_subscription=None):
    state = NS(nodes=[], contexts=[], executors=[])

    class Context:
        def __init__(self):
            self.active = False
            state.contexts.append(self)

        def init(self, args):
            self.active = True

        def ok(self):
            return self.active

        def try_shutdown(self):
            self.active = False

    class Node:
        def __init__(self, name, **kwargs):
            self.name, self.kwargs = name, kwargs
            self.subscriptions, self.destroyed = [], False
            state.nodes.append(self)

        def create_subscription(self, message_type, topic, callback, qos):
            if len(self.subscriptions) == fail_subscription:
                raise RuntimeError("fake subscription init failed")
            self.subscriptions.append((message_type, topic, callback, qos))

        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self, context):
            self.context, self.destroyed = context, False
            self.event = threading.Event()
            state.executors.append(self)

        def add_node(self, node):
            self.node = node

        def spin_once(self, timeout_sec):
            self.event.wait(timeout_sec)
            self.event.clear()

        def wake(self):
            self.event.set()

        def shutdown(self, timeout_sec):
            self.destroyed = True

    return NS(Context=Context, Node=Node, SingleThreadedExecutor=Executor,
              QoSProfile=lambda **kwargs: NS(**kwargs),
              ReliabilityPolicy=NS(BEST_EFFORT="best_effort"),
              HistoryPolicy=NS(KEEP_LAST="keep_last"),
              DurabilityPolicy=NS(VOLATILE="volatile"),
              Image=type("Image", (), {}), CameraInfo=type("CameraInfo", (), {}),
              CameraFrameTiming=type("Timing", (), {}), TrackerSample=type("Tracker", (), {})), state


class LiveMonitorTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config(ROOT / "config" / "calibration.example.yaml")
        self.clock = Clock()
        self.monitor = LiveMonitor(self.config)
        self.monitor._clock = self.clock

    def feed(self, stamp=1):
        self.monitor._running = True
        self.monitor._on_info(info(self.config))
        self.monitor._on_image(image(self.config, stamp))
        self.monitor._on_timing(timing(self.config, stamp))
        self.monitor._on_tracker(tracker(self.config))

    def test_default_snapshot_is_disconnected_and_does_not_import_ros(self):
        with patch("vt_tracker_camera_calib.gui_live._load_ros", side_effect=AssertionError):
            value = LiveMonitor(self.config).snapshot()
        for key in ("image_rgb", "image_age_s", "info_age_s", "timing_age_s",
                    "tracker_age_s", "tracker_position", "tracker_quaternion", "board_rms_px"):
            self.assertIsNone(value[key])
        self.assertFalse(value["board_valid"])
        self.assertFalse(value["tracker_valid"])
        self.assertEqual(value["image_hz"], 0.)
        self.assertEqual(value["error"], "")
        self.assertEqual(value["quality"], "disconnected")

    def test_missing_ros_has_readable_start_error_and_default_snapshot_survives(self):
        real_import = __import__

        def importing(name, *args, **kwargs):
            if name.startswith("rclpy"):
                raise ImportError("deliberately absent ROS")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=importing):
            with self.assertRaisesRegex(RuntimeError, "source /opt/ros/jazzy"):
                self.monitor.start()
        self.assertFalse(self.monitor.snapshot()["running"])
        self.assertIn("ROS", self.monitor.snapshot()["error"])
        self.monitor.stop()

    def test_rgb_conversion_handles_supported_encodings_and_padded_rows(self):
        expected = {"rgb8": [0, 1, 2], "bgr8": [2, 1, 0],
                    "rgba8": [0, 1, 2], "bgra8": [2, 1, 0], "mono8": [0, 0, 0]}
        for encoding, pixel in expected.items():
            with self.subTest(encoding=encoding):
                rgb, stamp = _image_rgb(image(self.config, encoding=encoding), self.config)
                self.assertEqual(rgb[0, 0].tolist(), pixel)
                self.assertEqual(rgb.shape, (2, 2, 3))
                self.assertEqual(stamp, 1_000_000_000)
                self.assertFalse(rgb.flags.writeable)
        message = image(self.config)
        message.step, message.data = 8, bytes(range(16))
        self.assertEqual(_image_rgb(message, self.config)[0][1, 0].tolist(), [8, 9, 10])

    def test_bad_image_invalidates_previous_board_and_image(self):
        for field, value in (("encoding", "16UC1"), ("step", 1), ("data", b"")):
            self.feed()
            message = image(self.config)
            setattr(message, field, value)
            self.monitor._on_image(message)
            self.assertIsNone(self.monitor.snapshot()["image_rgb"])
            self.assertIn("Image", self.monitor.snapshot()["error"])
        message = image(self.config)
        message.header.frame_id = "wrong_camera"
        self.monitor._on_image(message)
        self.assertIn("frame_id", self.monitor.snapshot()["error"])

    def test_snapshot_arrays_are_owned_and_age_rate_are_not_double_counted(self):
        self.feed()
        self.clock.value += .1
        self.monitor._on_image(image(self.config, stamp=2))
        self.monitor._on_tracker(tracker(self.config))
        first = self.monitor.snapshot()
        first["image_rgb"][:] = 255
        first["tracker_position"][0] = 999
        self.clock.value += .2
        second = self.monitor.snapshot()
        self.assertAlmostEqual(second["image_age_s"], .2)
        self.assertAlmostEqual(second["info_age_s"], .3)
        self.assertAlmostEqual(second["tracker_hz"], 10.)
        self.assertEqual(second["image_rgb"][0, 0].tolist(), [0, 1, 2])
        self.assertEqual(second["tracker_position"], [.1, .2, .3])
        self.assertEqual(second["tracker_quaternion"], [0., 0., 0., 1.])
        self.assertAlmostEqual(self.monitor.snapshot()["image_age_s"], .2)
        self.clock.value += 2.
        stale = self.monitor.snapshot()
        self.assertFalse(stale["tracker_valid"])
        self.assertIsNone(stale["tracker_position"])
        self.assertEqual(stale["image_hz"], 0.)

    def test_exact_timing_join_uses_host_mapping_not_device_or_arrival_clock(self):
        self.feed()
        with patch("vt_tracker_camera_calib.gui_live.detect_board_pose", side_effect=observation) as detect:
            self.monitor._detect_latest()
        self.assertEqual(detect.call_args.kwargs["timestamp_ns"], 100)
        self.assertEqual(detect.call_args.kwargs["source_stamp_ns"], 1_000_000_000)
        value = self.monitor.snapshot()
        self.assertTrue(value["board_valid"])
        self.assertTrue(value["timing_matched"])
        self.assertIn("尚未验证采集", value["guidance"])
        self.assertEqual(value["quality"], "preview_only")
        self.clock.value += 1.1
        self.assertFalse(self.monitor.snapshot()["board_valid"])

    def test_mismatched_timing_never_runs_detection_but_keeps_rgb_preview(self):
        self.feed()
        self.monitor._timings.clear()
        self.monitor._on_timing(timing(self.config, stamp=2))
        with patch("vt_tracker_camera_calib.gui_live.detect_board_pose") as detect:
            self.monitor._detect_latest()
        detect.assert_not_called()
        value = self.monitor.snapshot()
        self.assertFalse(value["timing_matched"])
        self.assertFalse(value["board_valid"])
        self.assertIsNotNone(value["image_rgb"])

    def test_delayed_timing_joins_recent_image_not_current_unmatched_image(self):
        self.feed(stamp=1)
        self.clock.value += .04
        self.monitor._on_image(image(self.config, stamp=2))
        with patch("vt_tracker_camera_calib.gui_live.detect_board_pose", side_effect=observation) as detect:
            self.monitor._detect_latest()
            self.assertEqual(detect.call_args.kwargs["source_stamp_ns"], 1_000_000_000)
            self.clock.value += .21
            self.monitor._on_timing(timing(self.config, stamp=2))
            self.monitor._detect_latest()
            self.assertEqual(detect.call_args.kwargs["source_stamp_ns"], 2_000_000_000)

    def test_timing_identity_flags_and_conflicts_cannot_unlock_detection(self):
        for field, value in (("camera_name", "bad"), ("camera_model", "bad"),
                             ("serial_number", "bad"), ("group_validity_flags", 3),
                             ("group_host_realtime_ns", 0)):
            with self.subTest(field=field):
                self.feed()
                message = timing(self.config)
                setattr(message, field, value)
                self.monitor._on_timing(message)
                self.assertFalse(self.monitor.snapshot()["timing_matched"])
                self.assertIn("Timing", self.monitor.snapshot()["error"])
        self.feed()
        self.monitor._on_timing(timing(self.config, host=999))
        self.monitor._on_timing(timing(self.config, host=100))
        self.assertFalse(self.monitor.snapshot()["timing_matched"])
        self.assertIn("冲突", self.monitor.snapshot()["error"])

    def test_join_cache_is_bounded_and_detection_selects_latest_only(self):
        self.monitor._running = True
        self.monitor._on_info(info(self.config))
        for stamp in range(100):
            self.monitor._on_image(image(self.config, stamp=stamp))
            self.monitor._on_timing(timing(self.config, stamp=stamp))
        self.assertEqual(len(self.monitor._images), 8)
        self.assertEqual(len(self.monitor._timings), 64)
        with patch("vt_tracker_camera_calib.gui_live.detect_board_pose", side_effect=observation) as detect:
            self.monitor._detect_latest()
            self.assertEqual(detect.call_count, 1)
            self.assertEqual(detect.call_args.kwargs["source_stamp_ns"], 99_000_000_000)

    def test_detection_rate_limited_and_duplicate_frame_not_redetected(self):
        self.feed()
        with patch("vt_tracker_camera_calib.gui_live.detect_board_pose", side_effect=observation) as detect:
            self.monitor._detect_latest()
            self.clock.value += .05
            self.monitor._on_image(image(self.config, stamp=2))
            self.monitor._on_timing(timing(self.config, stamp=2))
            self.monitor._detect_latest()
            self.assertEqual(detect.call_count, 1)
            self.clock.value += .16
            self.monitor._detect_latest()
            self.assertEqual(detect.call_count, 2)
            self.clock.value += .21
            self.monitor._detect_latest()
            self.assertEqual(detect.call_count, 2)

    def test_camera_info_frame_and_shape_errors_are_visible(self):
        self.feed()
        bad = info(self.config)
        bad.header.frame_id = "another_camera"
        self.monitor._on_info(bad)
        self.assertIsNone(self.monitor._intrinsics)
        self.assertIn("CameraInfo", self.monitor.snapshot()["error"])
        self.monitor._on_info(info(self.config, width=3))
        self.monitor._detect_latest()
        self.assertIn("dimensions", self.monitor.snapshot()["error"])
        self.assertFalse(self.monitor.snapshot()["board_valid"])

    def test_invalid_tracker_pose_flags_and_identity_are_never_shown_as_valid(self):
        self.feed()
        for field, value in (("pose_valid", False), ("tracking_status", 1),
                             ("tracker_id", "not_fingerprint"), ("tracker_id", "b" * 64),
                             ("host_realtime_ns", 0), ("role", "wrong")):
            message = tracker(self.config)
            setattr(message, field, value)
            self.monitor._on_tracker(message)
            self.assertFalse(self.monitor.snapshot()["tracker_valid"])
            self.assertIsNone(self.monitor.snapshot()["tracker_quaternion"])
        message = tracker(self.config)
        message.header.frame_id = "wrong_world"
        self.monitor._on_tracker(message)
        self.assertIn("world", self.monitor.snapshot()["error"])
        for field, value in (("position", NS(x=np.nan, y=0., z=0.)),
                             ("orientation", NS(x=0., y=0., z=0., w=0.))):
            message = tracker(self.config)
            setattr(message.pose, field, value)
            self.monitor._on_tracker(message)
            self.assertFalse(self.monitor.snapshot()["tracker_valid"])

    def test_invalidating_input_during_detection_discards_late_result(self):
        self.feed()

        def delayed(image_value, **kwargs):
            bad = info(self.config)
            bad.header.frame_id = "wrong"
            self.monitor._on_info(bad)
            return observation(image_value, **kwargs)

        with patch("vt_tracker_camera_calib.gui_live.detect_board_pose", side_effect=delayed):
            self.monitor._detect_latest()
        self.assertFalse(self.monitor.snapshot()["board_valid"])
        self.assertEqual(self.monitor.snapshot()["board_corners"], 0)

    def test_real_charuco_detector_on_generated_board(self):
        board = create_board(self.config.board)
        width, height = self.config.board.squares_x * 48, self.config.board.squares_y * 48
        pattern = board.generateImage((width, height), marginSize=0, borderBits=1)
        pixels = np.full((720, 1280), 255, dtype=np.uint8)
        x, y = (1280 - width) // 2, (720 - height) // 2
        pixels[y:y + height, x:x + width] = pattern
        message = image(self.config, encoding="mono8")
        message.width, message.height, message.step, message.data = 1280, 720, 1280, pixels.tobytes()
        camera = info(self.config, width=1280, height=720)
        camera.k = [1600., 0., 640., 0., 1600., 360., 0., 0., 1.]
        self.monitor._running = True
        self.monitor._on_info(camera)
        self.monitor._on_image(message)
        self.monitor._on_timing(timing(self.config))
        self.monitor._detect_latest()
        value = self.monitor.snapshot()
        self.assertTrue(value["board_valid"], value["error"])
        self.assertGreaterEqual(value["board_corners"], 30)
        self.assertLess(value["board_rms_px"], .1)

    def test_ros_lifecycle_is_private_context_four_subscriptions_and_idempotent(self):
        ros, state = fake_ros()
        with patch("vt_tracker_camera_calib.gui_live._load_ros", return_value=ros):
            self.monitor.start()
            self.monitor.start()
            self.assertTrue(self.monitor.snapshot()["running"])
            self.assertEqual(len(state.nodes), 1)
            self.assertEqual({entry[1] for entry in state.nodes[0].subscriptions},
                             {self.config.image_topic, self.config.camera_info_topic,
                              self.config.timing_topic, self.config.tracker_sample_topic})
            self.assertFalse(state.nodes[0].kwargs["start_parameter_services"])
            for subscription in state.nodes[0].subscriptions:
                self.assertEqual(subscription[3].depth, 1)
                self.assertEqual(subscription[3].reliability, "best_effort")
            self.monitor.stop()
            self.monitor.stop()
            self.assertTrue(state.nodes[0].destroyed)
            self.assertTrue(state.executors[0].destroyed)
            self.assertFalse(state.contexts[0].active)
            self.assertFalse(self.monitor._thread.is_alive())
            self.assertFalse(self.monitor._vision_thread.is_alive())
            self.monitor.start()
            self.monitor.stop()
            self.assertEqual(len(state.contexts), 2)
            self.assertIsNot(state.contexts[0], state.contexts[1])

    def test_partial_ros_initialization_failure_cleans_every_resource(self):
        ros, state = fake_ros(fail_subscription=2)
        with patch("vt_tracker_camera_calib.gui_live._load_ros", return_value=ros):
            with self.assertRaisesRegex(RuntimeError, "subscription init failed"):
                self.monitor.start()
        self.assertTrue(state.nodes[0].destroyed)
        self.assertTrue(state.executors[0].destroyed)
        self.assertFalse(state.contexts[0].active)
        self.assertFalse(self.monitor._thread.is_alive())

    def test_slow_vision_does_not_own_or_leak_ros_node_on_stop(self):
        ros, state = fake_ros()
        entered, release = threading.Event(), threading.Event()

        def slow(*args, **kwargs):
            entered.set()
            release.wait(5.)
            return observation(*args, **kwargs)

        with patch("vt_tracker_camera_calib.gui_live._load_ros", return_value=ros), \
                patch("vt_tracker_camera_calib.gui_live.detect_board_pose", side_effect=slow):
            try:
                self.monitor.start()
                self.feed()
                self.assertTrue(entered.wait(1.))
                started = time.monotonic()
                self.monitor.stop()
                self.assertLess(time.monotonic() - started, 2.5)
                self.assertTrue(state.nodes[0].destroyed)
                self.assertFalse(state.contexts[0].active)
                self.assertFalse(self.monitor.snapshot()["board_valid"])
                with self.assertRaisesRegex(RuntimeError, "仍在退出"):
                    self.monitor.start()
            finally:
                release.set()
                self.monitor.stop()
        self.assertFalse(self.monitor._vision_thread.is_alive())
        self.assertEqual(self.monitor.snapshot()["board_corners"], 0)


if __name__ == "__main__":
    unittest.main()
