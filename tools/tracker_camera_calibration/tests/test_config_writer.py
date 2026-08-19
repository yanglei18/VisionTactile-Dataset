from pathlib import Path
import stat
import tempfile
import unittest

from vt_tracker_camera_calib.config import load_config
from vt_tracker_camera_calib.config_writer import (
    build_config_document,
    write_config,
)


class ConfigWriterTest(unittest.TestCase):
    def test_writes_valid_identity_bound_d436_config(self) -> None:
        document = build_config_document(
            camera_name="d436",
            tracker_role="right_wrist",
            square_length_mm=39.92,
            marker_length_mm=29.94,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = write_config(Path(directory) / "d436-right.yaml", document)
            config = load_config(path)
            self.assertEqual(config.camera_serial, "408322071716")
            self.assertEqual(config.tracker_role, "right_wrist")
            self.assertEqual(config.tracker_sample_topic, "/vive/right_wrist/sample")
            self.assertAlmostEqual(config.board.square_length_m, 0.03992)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            with self.assertRaises(FileExistsError):
                write_config(path, document)

    def test_rejects_unknown_hardware_and_bad_measurements(self) -> None:
        with self.assertRaisesRegex(ValueError, "camera"):
            build_config_document(
                camera_name="unknown",
                tracker_role="torso",
                square_length_mm=40.0,
                marker_length_mm=30.0,
            )
        with self.assertRaisesRegex(ValueError, "smaller"):
            build_config_document(
                camera_name="d405_1",
                tracker_role="torso",
                square_length_mm=30.0,
                marker_length_mm=30.0,
            )


if __name__ == "__main__":
    unittest.main()
