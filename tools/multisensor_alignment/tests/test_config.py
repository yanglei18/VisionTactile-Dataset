from pathlib import Path
import tempfile
import unittest

import yaml

from vt_multisensor_alignment.config import load_config


ROOT = Path(__file__).resolve().parents[1]


class ConfigTests(unittest.TestCase):
    def test_right_glove_profile_preserves_core_and_requires_source_timed_state(self) -> None:
        profile_path = ROOT / "config" / "alignment.right_glove.yaml"
        document = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
        original = self._example_document()
        extra = document.pop("additional_streams")
        self.assertEqual(original.pop("additional_streams"), [])
        self.assertEqual(document, original)
        self.assertEqual(extra, [{
            "name": "right_glove", "topic": "/gloves/right/state",
            "type": "vt_glove_msgs/msg/GloveState", "time_source": "field",
            "timestamp_field": "host_realtime_ns", "strategy": "nearest",
            "max_delta_ms": 20.0, "required": True,
        }])
        config = load_config(profile_path)
        self.assertEqual(len(config.required_topics), 16)
        self.assertIn("/gloves/right/state", config.required_topics)

    def test_right_glove_recording_and_alignment_profiles_match_without_raw(self) -> None:
        capture_directory = ROOT.parents[1] / "ros2_ws" / "src" / "vt_realsense_capture" / "config"
        capture = yaml.safe_load(
            (capture_directory / "cameras.with_right_glove.yaml").read_text(encoding="utf-8")
        )
        original_capture = yaml.safe_load(
            (capture_directory / "cameras.yaml").read_text(encoding="utf-8")
        )
        recording_streams = capture["recording"].pop("additional_streams")
        self.assertEqual(original_capture["recording"].pop("additional_streams"), [])
        self.assertEqual(capture, original_capture)
        alignment = load_config(ROOT / "config" / "alignment.right_glove.yaml")
        self.assertEqual(recording_streams, [
            {"topic": item.topic, "type": item.type_name}
            for item in alignment.additional_streams
        ])
        self.assertEqual([item["topic"] for item in recording_streams], ["/gloves/right/state"])

    def test_example_declares_complete_core_contract(self) -> None:
        config = load_config(ROOT / "config" / "alignment.example.yaml")

        self.assertEqual(config.reference_camera, "d405_1")
        self.assertEqual(len(config.cameras), 3)
        self.assertEqual(len(config.trackers), 3)
        self.assertEqual(len(config.required_topics), 15)
        self.assertEqual(config.additional_streams, ())
        self.assertEqual(
            config.thresholds.min_required_stream_coverage_ratio, 0.99
        )
        self.assertEqual(
            {camera.tracker_role for camera in config.cameras},
            {"left_wrist", "right_wrist", "torso"},
        )

    def test_generic_stream_contract_is_loaded(self) -> None:
        document = self._example_document()
        document["additional_streams"] = [
            {
                "name": "left_glove",
                "topic": "/gloves/left/state",
                "type": "glove_msgs/msg/GloveState",
                "time_source": "header_stamp",
                "timestamp_field": "header.stamp",
                "strategy": "nearest",
                "max_delta_ms": 20.0,
                "required": True,
            }
        ]

        config = self._load_document(document)

        stream = config.additional_streams[0]
        self.assertEqual(stream.type_name, "glove_msgs/msg/GloveState")
        self.assertIn("/gloves/left/state", config.required_topics)

    def test_header_time_source_requires_header_stamp_field(self) -> None:
        document = self._example_document()
        document["additional_streams"] = [
            {
                "name": "left_glove",
                "topic": "/gloves/left/state",
                "type": "glove_msgs/msg/GloveState",
                "time_source": "header_stamp",
                "timestamp_field": "device_time_ns",
                "strategy": "nearest",
                "max_delta_ms": 20.0,
                "required": False,
            }
        ]

        with self.assertRaisesRegex(ValueError, "header.stamp"):
            self._load_document(document)

    def test_field_time_source_requires_dotted_ros_field_path(self) -> None:
        document = self._example_document()
        document["additional_streams"] = [
            {
                "name": "left_glove",
                "topic": "/gloves/left/state",
                "type": "glove_msgs/msg/GloveState",
                "time_source": "field",
                "timestamp_field": "samples[0].time",
                "strategy": "previous",
                "max_delta_ms": 20.0,
                "required": False,
            }
        ]

        with self.assertRaisesRegex(ValueError, "field path"):
            self._load_document(document)

    def _example_document(self) -> dict[str, object]:
        return yaml.safe_load(
            (ROOT / "config" / "alignment.example.yaml").read_text(
                encoding="utf-8"
            )
        )

    def _load_document(self, document: dict[str, object]):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "alignment.yaml"
            path.write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )
            return load_config(path)


if __name__ == "__main__":
    unittest.main()
