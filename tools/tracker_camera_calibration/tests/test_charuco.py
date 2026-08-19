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
