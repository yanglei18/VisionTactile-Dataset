from dataclasses import dataclass
from pathlib import Path
import tempfile
import tomllib
import unittest

import numpy as np

from vt_tracker_camera_calib.bag_reader import (
    decode_image_message,
    stamp_to_nanoseconds,
)
from vt_tracker_camera_calib.config import load_config


ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Stamp:
    sec: int
    nanosec: int


@dataclass
class Image:
    width: int
    height: int
    step: int
    encoding: str
    data: bytes


class ConfigAndBagHelperTest(unittest.TestCase):
    def test_loads_product_example(self) -> None:
        config = load_config(ROOT / "config" / "calibration.example.yaml")
        self.assertEqual(config.camera_serial, "260322278433")
        self.assertEqual(config.board.squares_x, 9)
        self.assertEqual(config.acceptance.min_pairs, 40)

    def test_product_dependency_bounds_match_ros_python_stack(self) -> None:
        document = tomllib.loads((ROOT / "pyproject.toml").read_text())
        dependencies = set(document["project"]["dependencies"])
        self.assertIn("numpy>=1.26,<2", dependencies)
        self.assertIn(
            "opencv-contrib-python-headless>=4.9,<4.12",
            dependencies,
        )

    def test_rejects_wrong_schema(self) -> None:
        source = (ROOT / "config" / "calibration.example.yaml").read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(source.replace("schema_version: 1", "schema_version: 2"))
            with self.assertRaisesRegex(ValueError, "schema_version"):
                load_config(path)

    def test_rejects_nonfinite_numeric_configuration(self) -> None:
        source = (ROOT / "config" / "calibration.example.yaml").read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nonfinite.yaml"
            path.write_text(
                source.replace("square_length_m: 0.040", "square_length_m: .nan"),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "square_length_m"):
                load_config(path)

    def test_decodes_padded_rgb_image_without_copying_padding(self) -> None:
        message = Image(
            width=2,
            height=2,
            step=8,
            encoding="rgb8",
            data=bytes(range(16)),
        )
        image, encoding = decode_image_message(message)
        self.assertEqual(encoding, "rgb8")
        self.assertEqual(image.shape, (2, 2, 3))
        self.assertTrue(np.array_equal(image[1].reshape(-1), np.arange(8, 14)))

    def test_stamp_validation(self) -> None:
        self.assertEqual(stamp_to_nanoseconds(Stamp(2, 3)), 2_000_000_003)
        with self.assertRaises(ValueError):
            stamp_to_nanoseconds(Stamp(1, 1_000_000_000))


if __name__ == "__main__":
    unittest.main()
