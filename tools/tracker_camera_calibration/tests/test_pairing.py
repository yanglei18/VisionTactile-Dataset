import unittest

import numpy as np

from vt_tracker_camera_calib.config import PairingConfig
from vt_tracker_camera_calib.model import BoardObservation, TimedTransform
from vt_tracker_camera_calib.pairing import pair_static_observations
from vt_tracker_camera_calib.transforms import Transform


CONFIG = PairingConfig(
    max_interpolation_gap_ms=20.0,
    stability_window_ms=50.0,
    max_static_translation_m=0.003,
    max_static_rotation_deg=0.5,
    min_time_separation_s=0.5,
    min_pose_translation_m=0.02,
    min_pose_rotation_deg=5.0,
)


class PairingTest(unittest.TestCase):
    def test_selects_one_static_observation_per_diverse_dwell(self) -> None:
        samples = []
        observations = []
        for dwell in range(4):
            base = dwell * 1_000_000_000
            transform = Transform.from_rvec_tvec(
                [0.0, dwell * 0.1, 0.0], [dwell * 0.05, 0.0, 0.5]
            )
            for offset_ms in range(0, 401, 10):
                samples.append(
                    TimedTransform(base + offset_ms * 1_000_000, transform)
                )
            for offset_ms in (100, 200, 300):
                observations.append(
                    BoardObservation(
                        timestamp_ns=base + offset_ms * 1_000_000,
                        camera_from_board=Transform.identity(),
                        reprojection_rms_px=0.2,
                        corner_count=30,
                        source_stamp_ns=base + offset_ms * 1_000_000,
                    )
                )
        selected = pair_static_observations(observations, samples, CONFIG)
        self.assertEqual(len(selected), 4)
        self.assertTrue(
            np.allclose(selected[-1].world_from_tracker.translation[0], 0.15)
        )

    def test_rejects_motion_inside_stability_window(self) -> None:
        samples = [
            TimedTransform(
                index * 10_000_000,
                Transform.from_rvec_tvec([0.0, 0.0, 0.0], [index * 0.01, 0.0, 0.0]),
            )
            for index in range(30)
        ]
        observation = BoardObservation(
            timestamp_ns=150_000_000,
            camera_from_board=Transform.identity(),
            reprojection_rms_px=0.2,
            corner_count=30,
            source_stamp_ns=150_000_000,
        )
        self.assertEqual(pair_static_observations([observation], samples, CONFIG), ())


if __name__ == "__main__":
    unittest.main()
