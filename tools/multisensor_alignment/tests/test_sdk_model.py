import unittest

import numpy as np

from vt_multisensor_alignment.errors import DatasetFormatError
from vt_multisensor_alignment.model import Transform
from vt_multisensor_alignment.sdk_model import parse_frame_record


def reference(topic: str, sequence: int, stamp: int) -> dict[str, object]:
    return {
        "topic": topic,
        "sequence": sequence,
        "bag_timestamp_ns": stamp + 50,
        "source_timestamp_ns": stamp,
    }


def transform(x: float = 0.0) -> dict[str, object]:
    return {
        "translation_m": [x, 2.0, 3.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def pose(role: str, tracker_id: str, stamp: int) -> dict[str, object]:
    return {
        "role": role,
        "tracker_id": tracker_id,
        "timestamp_ns": stamp,
        "bracket_gap_ns": 10,
        "before_sequence": 4,
        "after_sequence": 5,
        "world_from_tracker": transform(1.0),
    }


def frame_document(index: int = 4) -> dict[str, object]:
    stamp = 1_000
    return {
        "frame_index": index,
        "reference_camera": "d405_1",
        "reference_time_ns": stamp,
        "cameras": {
            "d405_1": {
                "host_realtime_ns": stamp,
                "source_timestamp_ns": 10_000,
                "delta_ns": 0,
                "color": reference("/d405_1/color/image_raw", 7, 10_000),
                "depth": reference("/d405_1/depth/image_rect_raw", 8, 10_000),
                "timing": reference("/d405_1/frame_timing", 9, 10_000),
                "attached_tracker": pose("left_wrist", "1" * 64, stamp),
                "world_from_camera": transform(4.0),
            },
            "d436": None,
        },
        "trackers": {
            "left_wrist": None,
            "torso": pose("torso", "3" * 64, stamp),
        },
        "additional_streams": {
            "left_glove": {
                "timestamp_ns": stamp - 2,
                "delta_ns": -2,
                "message": reference("/gloves/left/state", 3, stamp - 2),
            },
            "right_glove": None,
        },
        "quality_flags": ["missing_camera:d436"],
    }


class SdkModelTests(unittest.TestCase):
    def test_transform_matrix_is_read_only_and_applies_parent_from_child(self) -> None:
        value = Transform(
            np.array([1.0, 2.0, 3.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
        )

        matrix = value.as_matrix()

        np.testing.assert_array_equal(
            matrix,
            np.array(
                [
                    [1.0, 0.0, 0.0, 1.0],
                    [0.0, 1.0, 0.0, 2.0],
                    [0.0, 0.0, 1.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            ),
        )
        self.assertFalse(matrix.flags.writeable)

    def test_transform_matrix_rotates_before_translating(self) -> None:
        half = np.sqrt(0.5)
        value = Transform(
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, half, half]),
        )

        transformed = value.as_matrix() @ np.array([1.0, 0.0, 0.0, 1.0])

        np.testing.assert_allclose(transformed, [1.0, 1.0, 0.0, 1.0], atol=1e-12)

    def test_parse_frame_record_preserves_nulls_and_typed_values(self) -> None:
        record = parse_frame_record(
            frame_document(),
            expected_index=4,
            camera_names=("d405_1", "d436"),
            tracker_roles=("left_wrist", "torso"),
            additional_stream_names=("left_glove", "right_glove"),
        )

        self.assertEqual(record.frame_index, 4)
        self.assertEqual(record.reference_camera, "d405_1")
        self.assertIsNone(record.cameras["d436"])
        self.assertIsNone(record.trackers["left_wrist"])
        self.assertIsNone(record.additional_streams["right_glove"])
        self.assertEqual(record.quality_flags, ("missing_camera:d436",))
        camera = record.cameras["d405_1"]
        assert camera is not None
        self.assertEqual(camera.color.topic, "/d405_1/color/image_raw")
        self.assertEqual(camera.attached_tracker.role, "left_wrist")
        self.assertEqual(camera.world_from_camera.translation[0], 4.0)
        torso = record.trackers["torso"]
        assert torso is not None
        self.assertEqual(torso.tracker_id, "3" * 64)
        glove = record.additional_streams["left_glove"]
        assert glove is not None
        self.assertEqual(glove.delta_ns, -2)

    def test_parsed_mappings_are_immutable(self) -> None:
        record = parse_frame_record(frame_document(), expected_index=4)

        with self.assertRaises(TypeError):
            record.cameras["d436"] = object()
        with self.assertRaises(TypeError):
            record.trackers["torso"] = None

    def test_parser_rejects_non_contiguous_frame_index(self) -> None:
        with self.assertRaisesRegex(DatasetFormatError, "frame_index"):
            parse_frame_record(frame_document(index=3), expected_index=4)

    def test_parser_rejects_reference_source_stamp_mismatch(self) -> None:
        document = frame_document()
        document["cameras"]["d405_1"]["depth"]["source_timestamp_ns"] = 9_999

        with self.assertRaisesRegex(DatasetFormatError, "source timestamp"):
            parse_frame_record(document, expected_index=4)

    def test_parser_rejects_unexpected_stream_names(self) -> None:
        with self.assertRaisesRegex(DatasetFormatError, "camera names"):
            parse_frame_record(
                frame_document(),
                expected_index=4,
                camera_names=("d405_1",),
            )

    def test_parser_rejects_non_finite_transform(self) -> None:
        document = frame_document()
        document["trackers"]["torso"]["world_from_tracker"]["translation_m"][0] = float("nan")

        with self.assertRaisesRegex(DatasetFormatError, "transform"):
            parse_frame_record(document, expected_index=4)

    def test_parser_rejects_boolean_timestamp(self) -> None:
        document = frame_document()
        document["reference_time_ns"] = True

        with self.assertRaisesRegex(DatasetFormatError, "reference_time_ns"):
            parse_frame_record(document, expected_index=4)


if __name__ == "__main__":
    unittest.main()
