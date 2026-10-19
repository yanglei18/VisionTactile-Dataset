import math
import unittest

import numpy as np

from vt_tracker_camera_calib.config import AcceptanceConfig
from vt_tracker_camera_calib.handeye import solve_hand_eye
from vt_tracker_camera_calib.model import CalibrationPair
from vt_tracker_camera_calib.transforms import Transform, rotation_distance_rad


ACCEPTANCE = AcceptanceConfig(
    min_pairs=40,
    holdout_fraction=0.2,
    target_translation_rms_m=0.005,
    target_rotation_rms_deg=0.5,
    maximum_translation_rms_m=0.010,
    maximum_rotation_rms_deg=1.0,
)


def synthetic_pairs(count: int = 50) -> tuple[tuple[CalibrationPair, ...], Transform]:
    generator = np.random.default_rng(20260819)
    tracker_from_camera = Transform.from_rvec_tvec(
        [0.2, -0.1, 0.15], [0.08, -0.03, 0.12]
    )
    world_from_board = Transform.from_rvec_tvec(
        [0.1, 0.2, -0.3], [0.4, -0.2, 1.0]
    )
    pairs = []
    for index in range(count):
        axis = generator.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = generator.uniform(-1.2, 1.2)
        world_from_tracker = Transform.from_rvec_tvec(
            axis * angle,
            generator.uniform([-0.5, -0.4, 0.2], [0.5, 0.4, 1.0]),
        )
        camera_from_board = (
            tracker_from_camera.inverse()
            @ world_from_tracker.inverse()
            @ world_from_board
        )
        pairs.append(
            CalibrationPair(
                timestamp_ns=index * 1_000_000_000,
                world_from_tracker=world_from_tracker,
                camera_from_board=camera_from_board,
                reprojection_rms_px=0.2,
                corner_count=35,
                interpolation_gap_ns=10_000_000,
                local_translation_motion_m=0.0005,
                local_rotation_motion_deg=0.1,
            )
        )
    return tuple(pairs), tracker_from_camera


class HandEyeTest(unittest.TestCase):
    def test_recovers_known_tracker_from_camera_transform(self) -> None:
        pairs, expected = synthetic_pairs()
        solution = solve_hand_eye(pairs, ACCEPTANCE)
        self.assertEqual(solution.quality, "TARGET")
        self.assertLess(
            np.linalg.norm(
                solution.tracker_from_camera.translation - expected.translation
            ),
            1e-8,
        )
        self.assertLess(
            math.degrees(rotation_distance_rad(solution.tracker_from_camera, expected)),
            1e-5,
        )
        self.assertEqual(solution.pair_count, 50)
        self.assertGreaterEqual(sum(score.valid for score in solution.method_scores), 3)

    def test_rejects_degenerate_pose_set(self) -> None:
        pair = CalibrationPair(
            timestamp_ns=0,
            world_from_tracker=Transform.identity(),
            camera_from_board=Transform.identity(),
            reprojection_rms_px=0.1,
            corner_count=30,
            interpolation_gap_ns=1,
            local_translation_motion_m=0.0,
            local_rotation_motion_deg=0.0,
        )
        pairs = tuple(
            CalibrationPair(
                timestamp_ns=index,
                world_from_tracker=pair.world_from_tracker,
                camera_from_board=pair.camera_from_board,
                reprojection_rms_px=pair.reprojection_rms_px,
                corner_count=pair.corner_count,
                interpolation_gap_ns=pair.interpolation_gap_ns,
                local_translation_motion_m=0.0,
                local_rotation_motion_deg=0.0,
            )
            for index in range(40)
        )
        with self.assertRaisesRegex(ValueError, "rotational span"):
            solve_hand_eye(pairs, ACCEPTANCE)


if __name__ == "__main__":
    unittest.main()
