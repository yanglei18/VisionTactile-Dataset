from pathlib import Path
from types import SimpleNamespace
import unittest

from vt_multisensor_alignment.bag_reader import (
    extract_timestamp_ns,
    join_camera_frames,
    require_topic_types,
    stamp_to_nanoseconds,
)
from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.model import MessageRef, TimingRecord


ROOT = Path(__file__).resolve().parents[1]


def reference(topic: str, sequence: int, stamp_ns: int) -> MessageRef:
    return MessageRef(topic, sequence, stamp_ns + 1_000, stamp_ns)


class BagReaderHelperTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config(ROOT / "config" / "alignment.example.yaml")

    def test_stamp_conversion_rejects_malformed_nsec(self) -> None:
        stamp = SimpleNamespace(sec=2, nanosec=3)
        self.assertEqual(stamp_to_nanoseconds(stamp), 2_000_000_003)
        with self.assertRaisesRegex(ValueError, "supported range"):
            stamp_to_nanoseconds(SimpleNamespace(sec=2, nanosec=1_000_000_000))

    def test_extracts_nested_stamp_and_integer_nanoseconds(self) -> None:
        message = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=4)),
            timing=SimpleNamespace(host_realtime_ns=9_000_000_001),
        )
        self.assertEqual(
            extract_timestamp_ns(message, "header.stamp"), 3_000_000_004
        )
        self.assertEqual(
            extract_timestamp_ns(message, "timing.host_realtime_ns"),
            9_000_000_001,
        )
        with self.assertRaisesRegex(ValueError, "missing timestamp field"):
            extract_timestamp_ns(message, "missing.value")

    def test_topic_gate_checks_required_and_present_optional_types(self) -> None:
        expected = {
            camera.color_topic: "sensor_msgs/msg/Image"
            for camera in self.config.cameras
        }
        with self.assertRaisesRegex(ValueError, "/d405_1/depth/image_rect_raw"):
            require_topic_types(expected, self.config)

    def test_topic_gate_rejects_unconfigured_bag_stream(self) -> None:
        observed = {
            topic: type_name
            for topic, type_name in (
                (camera.color_topic, "sensor_msgs/msg/Image")
                for camera in self.config.cameras
            )
        }
        observed["/unconfigured/state"] = "example_msgs/msg/State"

        with self.assertRaisesRegex(ValueError, "/unconfigured/state"):
            require_topic_types(observed, self.config)

    def test_join_uses_only_complete_exact_stamp_groups(self) -> None:
        color = {
            100: reference("/color", 0, 100),
            200: reference("/color", 1, 200),
        }
        depth = {
            100: reference("/depth", 0, 100),
            300: reference("/depth", 1, 300),
        }
        timing = {
            100: TimingRecord(
                source_timestamp_ns=100,
                host_realtime_ns=1_000,
                host_monotonic_ns=500,
                reference=reference("/timing", 0, 100),
            ),
            200: TimingRecord(
                source_timestamp_ns=200,
                host_realtime_ns=1_100,
                host_monotonic_ns=600,
                reference=reference("/timing", 1, 200),
            ),
        }

        frames = join_camera_frames("d405_1", color, depth, timing)

        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].source_timestamp_ns, 100)
        self.assertEqual(frames[0].host_realtime_ns, 1_000)


if __name__ == "__main__":
    unittest.main()
