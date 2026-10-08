import unittest

from vut_validation.metrics import ValidationSession
from vut_validation.model import PoseSample, ValidationThresholds


def pose(index: int, status: int = 2) -> PoseSample:
    return PoseSample(
        tracker_id="23:30:85:74:06:a3",
        host_monotonic_ns=index * 20_000_000,
        host_realtime_ns=1_000_000_000 + index * 20_000_000,
        upstream_timestamp_ms=index * 20,
        position=(index / 100.0, 0.0, 0.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0, 0.0),
        tracking_status=status,
        buttons=0,
    )


class MetricsTests(unittest.TestCase):
    def test_complete_50_hz_stream_passes(self) -> None:
        session = ValidationSession(
            ValidationThresholds(duration_s=1.0)
        )
        for index in range(51):
            session.add(pose(index))

        self.assertTrue(session.finish(expected_trackers=1).passed)

    def test_rotation_only_fails(self) -> None:
        session = ValidationSession(
            ValidationThresholds(duration_s=1.0)
        )
        for index in range(51):
            session.add(pose(index, 3))

        report = session.finish(expected_trackers=1)
        self.assertFalse(report.passed)
        self.assertEqual(report.trackers[0].invalid_tracking_samples, 51)

    def test_missing_trackers_fail(self) -> None:
        session = ValidationSession(
            ValidationThresholds(duration_s=1.0)
        )
        for index in range(51):
            session.add(pose(index))

        self.assertFalse(session.finish(expected_trackers=3).passed)

    def test_non_monotonic_tracker_sample_is_rejected(self) -> None:
        session = ValidationSession(
            ValidationThresholds(duration_s=1.0)
        )
        session.add(pose(2))

        with self.assertRaisesRegex(ValueError, "monotonic"):
            session.add(pose(1))


if __name__ == "__main__":
    unittest.main()
