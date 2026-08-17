from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from vut_validation.cli import main
from vut_validation.model import PoseSample


def pose(index: int) -> PoseSample:
    return PoseSample(
        tracker_id="usb-direct",
        host_monotonic_ns=index * 20_000_000,
        host_realtime_ns=1_000_000_000 + index * 20_000_000,
        upstream_timestamp_ms=index * 20,
        position=(index / 100.0, 0.0, 0.0),
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
        angular_velocity=(0.0, 0.0, 0.0, 0.0),
        tracking_status=2,
        buttons=0,
    )


class FakeBackend:
    def start(self, callback) -> None:
        for index in range(51):
            callback(pose(index))

    def stop(self) -> None:
        pass


class CliTests(unittest.TestCase):
    def test_pass_report_is_machine_readable(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory).resolve() / "report.json"

            with redirect_stdout(io.StringIO()):
                code = main(
                    [
                        "run",
                        "--mode",
                        "TRACKER_USB",
                        "--duration",
                        "1",
                        "--expected-trackers",
                        "1",
                        "--output",
                        str(output),
                    ],
                    backend_factory=lambda mode: FakeBackend(),
                    sleep=lambda seconds: None,
                )

            report = json.loads(output.read_text())
            self.assertEqual(code, 0)
            self.assertTrue(report["passed"])
            self.assertEqual(report["observed_trackers"], 1)

    def test_relative_output_fails_setup(self) -> None:
        with redirect_stderr(io.StringIO()):
            code = main(
                [
                    "run",
                    "--mode",
                    "TRACKER_USB",
                    "--output",
                    "report.json",
                ],
                backend_factory=lambda mode: FakeBackend(),
                sleep=lambda seconds: None,
            )

        self.assertEqual(code, 2)

    def test_missing_expected_tracker_returns_validation_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory).resolve() / "report.json"

            with redirect_stdout(io.StringIO()):
                code = main(
                    [
                        "run",
                        "--mode",
                        "DONGLE_USB",
                        "--duration",
                        "1",
                        "--expected-trackers",
                        "3",
                        "--output",
                        str(output),
                    ],
                    backend_factory=lambda mode: FakeBackend(),
                    sleep=lambda seconds: None,
                )

            self.assertEqual(code, 1)
            self.assertFalse(json.loads(output.read_text())["passed"])


if __name__ == "__main__":
    unittest.main()
