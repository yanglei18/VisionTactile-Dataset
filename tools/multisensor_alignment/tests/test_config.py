from pathlib import Path
import tempfile
import unittest

import yaml

from vt_multisensor_alignment.config import load_config


ROOT = Path(__file__).resolve().parents[1]


class ConfigTests(unittest.TestCase):
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
