from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import tempfile
import unittest

from vt_multisensor_alignment.cli import main


class CliTests(unittest.TestCase):
    def test_version_is_available_without_ros_environment(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            status = main(["--version"])
        self.assertEqual(status, 0)
        self.assertEqual(output.getvalue().strip(), "vt-multisensor-alignment 0.3.0")

    def test_validate_reports_missing_output_as_usage_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = StringIO()
            with redirect_stdout(output):
                status = main(
                    ["validate", "--output", str(Path(temporary) / "missing")]
                )
        self.assertEqual(status, 1)
        self.assertIn("ERROR:", output.getvalue())


if __name__ == "__main__":
    unittest.main()
