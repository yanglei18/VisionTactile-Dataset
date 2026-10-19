import unittest

from vt_multisensor_alignment.clock_audit import audit_clock_stream
from vt_multisensor_alignment.model import ClockObservation


class ClockAuditTests(unittest.TestCase):
    def test_accepts_realtime_and_monotonic_with_constant_offset(self) -> None:
        report = audit_clock_stream(
            "camera:d405_1",
            (
                ClockObservation(1_000_000_000, 100_000_000),
                ClockObservation(1_010_000_000, 110_000_000),
                ClockObservation(1_020_000_100, 120_000_000),
            ),
            max_step_ns=1_000,
        )

        self.assertTrue(report.valid)
        self.assertEqual(report.maximum_step_error_ns, 100)
        self.assertEqual(report.violations, ())

    def test_rejects_realtime_jump_relative_to_monotonic(self) -> None:
        report = audit_clock_stream(
            "tracker:torso",
            (
                ClockObservation(1_000_000_000, 100_000_000),
                ClockObservation(1_010_000_000, 110_000_000),
                ClockObservation(1_040_000_000, 120_000_000),
            ),
            max_step_ns=5_000_000,
        )

        self.assertFalse(report.valid)
        self.assertEqual(report.maximum_step_error_ns, 20_000_000)
        self.assertEqual(report.violations[0].index, 2)
        self.assertEqual(report.violations[0].reason, "clock_step")

    def test_rejects_non_increasing_clock(self) -> None:
        report = audit_clock_stream(
            "camera:d436",
            (
                ClockObservation(1_000, 100),
                ClockObservation(1_001, 100),
            ),
            max_step_ns=10,
        )

        self.assertFalse(report.valid)
        self.assertEqual(report.violations[0].reason, "non_increasing")


if __name__ == "__main__":
    unittest.main()
