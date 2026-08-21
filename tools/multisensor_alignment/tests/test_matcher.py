import math
import unittest

import numpy as np

from vt_multisensor_alignment.matcher import (
    interpolate_pose,
    match_nearest_unique,
    select_generic_sample,
)
from vt_multisensor_alignment.model import (
    GenericSample,
    MessageRef,
    TimedPose,
    Transform,
)


def timed_pose(timestamp_ns: int, x: float, quaternion=None) -> TimedPose:
    return TimedPose(
        role="torso",
        tracker_id="a" * 64,
        host_realtime_ns=timestamp_ns,
        host_monotonic_ns=timestamp_ns + 10,
        transform=Transform(
            np.array([x, 0.0, 0.0]),
            np.asarray(quaternion or [0.0, 0.0, 0.0, 1.0]),
        ),
        reference=MessageRef("/vive/torso/sample", timestamp_ns, timestamp_ns, timestamp_ns),
    )


def generic(timestamp_ns: int) -> GenericSample:
    return GenericSample(
        stream_name="left_glove",
        timestamp_ns=timestamp_ns,
        reference=MessageRef("/glove", timestamp_ns, timestamp_ns, timestamp_ns),
    )


class MatcherTests(unittest.TestCase):
    def test_nearest_matching_never_reuses_candidate(self) -> None:
        matches = match_nearest_unique(
            (100, 104, 200),
            (102, 198),
            max_delta_ns=10,
        )

        self.assertEqual(matches, (0, None, 1))

    def test_nearest_matching_rejects_outside_inclusive_limit(self) -> None:
        matches = match_nearest_unique((100,), (111,), max_delta_ns=10)
        self.assertEqual(matches, (None,))

    def test_matching_maximizes_coverage_before_minimizing_residual(self) -> None:
        matches = match_nearest_unique(
            (100, 133),
            (81, 118),
            max_delta_ns=20,
        )

        self.assertEqual(matches, (0, 1))

    def test_pose_interpolation_is_linear_and_slerp(self) -> None:
        half_turn_z = [0.0, 0.0, 1.0, 0.0]
        result = interpolate_pose(
            (timed_pose(100, 0.0), timed_pose(200, 2.0, half_turn_z)),
            timestamp_ns=150,
            max_gap_ns=100,
        )

        self.assertIsNotNone(result)
        np.testing.assert_allclose(result.transform.translation, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(result.transform.quaternion_xyzw[2], math.sqrt(0.5))
        self.assertAlmostEqual(result.transform.quaternion_xyzw[3], math.sqrt(0.5))
        self.assertEqual(result.bracket_gap_ns, 100)
        self.assertEqual((result.before_sequence, result.after_sequence), (100, 200))

    def test_pose_interpolation_never_extrapolates_or_crosses_large_gap(self) -> None:
        samples = (timed_pose(100, 0.0), timed_pose(300, 2.0))
        self.assertIsNone(interpolate_pose(samples, 99, 500))
        self.assertIsNone(interpolate_pose(samples, 301, 500))
        self.assertIsNone(interpolate_pose(samples, 200, 199))

    def test_generic_previous_never_selects_future_sample(self) -> None:
        samples = (generic(100), generic(150), generic(210))

        selected = select_generic_sample(
            samples, timestamp_ns=180, strategy="previous", max_delta_ns=40
        )

        self.assertEqual(selected, 1)
        self.assertIsNone(
            select_generic_sample(
                samples, timestamp_ns=90, strategy="previous", max_delta_ns=40
            )
        )


if __name__ == "__main__":
    unittest.main()
