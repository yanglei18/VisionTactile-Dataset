from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.extrinsics import load_extrinsics


ROOT = Path(__file__).resolve().parents[1]
TRACKER_IDS = {
    "left_wrist": "1" * 64,
    "right_wrist": "2" * 64,
    "torso": "3" * 64,
}


class ExtrinsicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config(ROOT / "config" / "alignment.example.yaml")

    def test_loads_three_identity_bound_parent_from_child_transforms(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self._write_all(directory)

            values = load_extrinsics(directory, self.config, TRACKER_IDS)

            self.assertEqual(set(values), {"d405_1", "d405_2", "d436"})
            np.testing.assert_allclose(
                values["d405_1"].tracker_from_camera.translation,
                [1.0, 0.0, 0.0],
            )
            self.assertEqual(values["d405_1"].tracker_id, "3" * 64)
            self.assertEqual(len(values["d405_1"].sha256), 64)

    def test_rejects_tracker_identity_that_differs_from_bag(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self._write_all(directory)
            path = directory / "d405_1.yaml"
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            document["tracker"]["tracker_id"] = "f" * 64
            path.write_text(yaml.safe_dump(document), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "tracker_id"):
                load_extrinsics(directory, self.config, TRACKER_IDS)

    def test_rejects_non_valid_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            self._write_all(directory)
            path = directory / "d436.yaml"
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            document["status"] = "REJECTED"
            path.write_text(yaml.safe_dump(document), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "status VALID"):
                load_extrinsics(directory, self.config, TRACKER_IDS)

    def _write_all(self, directory: Path) -> None:
        for index, camera in enumerate(self.config.cameras, start=1):
            tracker = self.config.tracker_by_role[camera.tracker_role]
            document = {
                "schema_version": 1,
                "status": "VALID",
                "camera": {
                    "name": camera.name,
                    "model": camera.model,
                    "serial": camera.serial,
                    "frame_id": camera.frame_id,
                },
                "tracker": {
                    "role": camera.tracker_role,
                    "tracker_id": TRACKER_IDS[camera.tracker_role],
                    "frame_id": tracker.frame_id,
                },
                "transform": {
                    "semantics": "parent_from_child",
                    "parent_frame": tracker.frame_id,
                    "child_frame": camera.frame_id,
                    "translation_m": [float(index), 0.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "quality": {"grade": "PASS"},
                "frames": {"world": self.config.world_frame, "board": "board"},
                "provenance": {"tool": "vt-tracker-camera-calibration"},
            }
            (directory / f"{camera.name}.yaml").write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )


if __name__ == "__main__":
    unittest.main()
