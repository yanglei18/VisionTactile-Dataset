import math
import unittest

from vut_validation.model import (
    PoseSample,
    ValidationThresholds,
    canonical_tracker_id,
)


class ModelTests(unittest.TestCase):
    def test_slot_bits_are_removed(self) -> None:
        self.assertEqual(
            canonical_tracker_id("23:30:85:74:06:a3"),
            canonical_tracker_id("23:32:85:74:06:a3"),
        )

    def test_zero_mac_is_usb_direct(self) -> None:
        self.assertEqual(
            canonical_tracker_id("00:00:00:00:00:00"),
            "usb-direct",
        )

    def test_defaults_match_contract(self) -> None:
        self.assertEqual(
            ValidationThresholds(),
            ValidationThresholds(300.0, 30.0, 100.0, 2, 0.9, 1.1),
        )

    def test_non_finite_pose_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "finite"):
            PoseSample(
                tracker_id="usb-direct",
                host_monotonic_ns=1,
                host_realtime_ns=2,
                upstream_timestamp_ms=3,
                position=(math.nan, 0.0, 0.0),
                quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
                acceleration=(0.0, 0.0, 0.0),
                angular_velocity=(0.0, 0.0, 0.0, 0.0),
                tracking_status=2,
                buttons=0,
            )


if __name__ == "__main__":
    unittest.main()
