import tempfile
from pathlib import Path
import unittest

import cv2
import numpy as np

from vt_tracker_camera_calib.charuco import (
    create_board,
    detect_board_pose,
    render_board,
)
from vt_tracker_camera_calib.config import BoardConfig
from vt_tracker_camera_calib.model import CameraIntrinsics


BOARD = BoardConfig(
    squares_x=9,
    squares_y=6,
    square_length_m=0.04,
    marker_length_m=0.03,
    dictionary="DICT_5X5_1000",
    min_corners=12,
    max_reprojection_rms_px=2.0,
)


class CharucoTest(unittest.TestCase):
    def test_default_board_detects_88_corners_at_the_physical_pattern_scale(self) -> None:
        from vt_capture_core.calibration_policy import board_geometry_defaults
        from vt_tracker_camera_calib.config_writer import build_config_document

        defaults = board_geometry_defaults()
        document = build_config_document(
            camera_name="d405_1", tracker_role="torso",
            square_length_mm=defaults["square_length_mm"],
            marker_length_mm=defaults["marker_length_mm"],
        )
        config = BoardConfig(**document["board"])
        board = create_board(config)
        self.assertEqual(len(board.getIds()), 54)
        # 180 x 135 mm pattern at z=300 mm with f=1200 px: 720 x 540 px.
        image = np.full((720, 1280), 255, dtype=np.uint8)
        image[90:630, 280:1000] = board.generateImage((720, 540), marginSize=0)
        intrinsics = CameraIntrinsics(
            1280, 720, np.array([[1200., 0., 640.], [0., 1200., 360.], [0., 0., 1.]]),
            np.zeros(5),
        )
        observation = detect_board_pose(
            image, encoding="mono8", intrinsics=intrinsics, config=config,
            timestamp_ns=100, source_stamp_ns=90,
        )
        self.assertIsNotNone(observation)
        self.assertEqual(observation.corner_count, 88)
        self.assertLess(observation.reprojection_rms_px, .25)
        self.assertAlmostEqual(observation.camera_from_board.translation[2], .3, places=3)

    def test_renders_and_detects_front_parallel_board(self) -> None:
        board = create_board(BOARD)
        if hasattr(board, "generateImage"):
            pattern = board.generateImage(
                (720, 480), marginSize=0, borderBits=1
            )
        else:
            pattern = board.draw((720, 480), marginSize=0, borderBits=1)
        image = np.full((720, 1280), 255, dtype=np.uint8)
        image[120:600, 280:1000] = pattern
        intrinsics = CameraIntrinsics(
            1280,
            720,
            np.array([[1600.0, 0.0, 640.0], [0.0, 1600.0, 360.0], [0.0, 0.0, 1.0]]),
            np.zeros(5),
        )
        observation = detect_board_pose(
            image,
            encoding="mono8",
            intrinsics=intrinsics,
            config=BOARD,
            timestamp_ns=100,
            source_stamp_ns=90,
        )
        self.assertIsNotNone(observation)
        assert observation is not None
        self.assertGreaterEqual(observation.corner_count, 30)
        self.assertLess(observation.reprojection_rms_px, 0.1)
        self.assertAlmostEqual(observation.camera_from_board.translation[2], 0.8, places=3)

    def test_render_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "board.png"
            render_board(BOARD, target, dpi=72)
            self.assertTrue(target.is_file())
            self.assertIsNotNone(cv2.imread(str(target), cv2.IMREAD_GRAYSCALE))
            with self.assertRaises(FileExistsError):
                render_board(BOARD, target, dpi=72)


if __name__ == "__main__":
    unittest.main()
