from dataclasses import replace
import math
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


def observation_at(timestamp_ns: int) -> BoardObservation:
    return BoardObservation(
        timestamp_ns, Transform.identity(), 0.2, 30, timestamp_ns
    )


def pose(x: float = 0.0, angle_deg: float = 0.0) -> Transform:
    return Transform.from_rvec_tvec(
        [0.0, 0.0, math.radians(angle_deg)], [x, 0.0, 0.0]
    )


def stationary_samples() -> list[TimedTransform]:
    return [TimedTransform(i * 10_000_000, pose()) for i in range(21)]


class PairingTest(unittest.TestCase):
    def test_reports_full_window_diameter(self) -> None:
        samples = stationary_samples()
        samples[8] = TimedTransform(80_000_000, pose(x=-0.001, angle_deg=-0.1))
        samples[12] = TimedTransform(120_000_000, pose(x=0.001, angle_deg=0.1))
        pairs = pair_static_observations([observation_at(100_000_000)], samples, CONFIG)
        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0].local_translation_motion_m, 0.002)
        self.assertAlmostEqual(pairs[0].local_rotation_motion_deg, 0.2, places=7)

    def test_rejects_diameter_even_when_each_displacement_from_center_is_small(self) -> None:
        samples = stationary_samples()
        samples[8] = TimedTransform(80_000_000, pose(x=-0.002))
        samples[12] = TimedTransform(120_000_000, pose(x=0.002))
        self.assertEqual(
            pair_static_observations([observation_at(100_000_000)], samples, CONFIG), ()
        )

    def test_requires_coverage_of_both_window_endpoints(self) -> None:
        for samples in (stationary_samples()[6:], stationary_samples()[:15]):
            with self.subTest(timestamps=[value.timestamp_ns for value in samples]):
                self.assertEqual(
                    pair_static_observations([observation_at(100_000_000)], samples, CONFIG), ()
                )

    def test_interpolates_covered_window_endpoints(self) -> None:
        samples = stationary_samples()
        config = replace(CONFIG, stability_window_ms=45.0)
        pairs = pair_static_observations([observation_at(105_000_000)], samples, config)
        self.assertEqual(len(pairs), 1)
        self.assertEqual(pairs[0].interpolation_gap_ns, 10_000_000)

    def test_rejects_clock_rollback_before_timestamp_sorting(self) -> None:
        samples = [
            replace(value, timestamp_ns=value.timestamp_ns + 1_000_000_000,
                    host_monotonic_ns=value.timestamp_ns + 100_000_000)
            for value in stationary_samples()
        ]
        samples[10] = replace(samples[10], timestamp_ns=1_050_000_000)
        with self.assertRaisesRegex(ValueError, "non-increasing host clock"):
            pair_static_observations([], samples, CONFIG)

    def test_rejects_partial_monotonic_metadata(self) -> None:
        samples = stationary_samples()
        samples[1] = replace(samples[1], host_monotonic_ns=100)
        with self.assertRaisesRegex(ValueError, "incomplete monotonic clock metadata"):
            pair_static_observations([], samples, CONFIG)

    def test_rejects_board_clock_reset_before_sorting(self) -> None:
        observations = [
            replace(observation_at(100_000_000), host_monotonic_raw_ns=200_000_000),
            replace(observation_at(90_000_000), host_monotonic_raw_ns=210_000_000),
        ]
        with self.assertRaisesRegex(ValueError, "non-increasing host clock"):
            pair_static_observations(observations, stationary_samples(), CONFIG)

    def test_rejects_translation_excursion_returning_to_same_endpoint(self) -> None:
        samples = stationary_samples()
        samples[8] = TimedTransform(80_000_000, pose(x=0.1))
        self.assertEqual(
            pair_static_observations([observation_at(100_000_000)], samples, CONFIG),
            (),
        )

    def test_rejects_rotation_excursion_returning_to_same_endpoint(self) -> None:
        samples = stationary_samples()
        samples[12] = TimedTransform(120_000_000, pose(angle_deg=10.0))
        self.assertEqual(
            pair_static_observations([observation_at(100_000_000)], samples, CONFIG),
            (),
        )

    def test_rejects_internal_gap_even_with_exact_center_and_endpoints(self) -> None:
        samples = [
            TimedTransform(timestamp_ns, pose())
            for timestamp_ns in (50_000_000, 100_000_000, 150_000_000)
        ]
        self.assertEqual(
            pair_static_observations([observation_at(100_000_000)], samples, CONFIG),
            (),
        )

    def test_pose_diversity_rejects_return_to_any_previous_pose(self) -> None:
        samples = []
        observations = []
        for index, x in enumerate((0.0, 0.1, 0.0, 0.2)):
            base = index * 1_000_000_000
            samples.extend(
                TimedTransform(base + offset * 10_000_000, pose(x=x))
                for offset in range(21)
            )
            observations.append(observation_at(base + 100_000_000))
        selected = pair_static_observations(observations, samples, CONFIG)
        self.assertEqual(
            [value.timestamp_ns for value in selected],
            [100_000_000, 1_100_000_000, 3_100_000_000],
        )

    def test_duplicate_observation_timestamps_are_rejected(self) -> None:
        observation = observation_at(100_000_000)
        with self.assertRaisesRegex(ValueError, "observation timestamps must be unique"):
            pair_static_observations([observation, observation], stationary_samples(), CONFIG)

    def test_direct_pairing_configuration_cannot_disable_gates_with_nan(self) -> None:
        for value in (math.nan, math.inf, True):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    replace(CONFIG, max_static_translation_m=value)

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
