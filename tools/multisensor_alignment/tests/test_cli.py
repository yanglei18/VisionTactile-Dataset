from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from vt_multisensor_alignment.cli import main


class CliTests(unittest.TestCase):
    def test_version_is_available_without_ros_environment(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            status = main(["--version"])
        self.assertEqual(status, 0)
        self.assertEqual(output.getvalue().strip(), "vt-multisensor-alignment 0.1.0")

    def test_validate_reports_missing_output_as_usage_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = StringIO()
            with redirect_stdout(output):
                status = main(
                    ["validate", "--output", str(Path(temporary) / "missing")]
                )
        self.assertEqual(status, 1)
        self.assertIn("ERROR:", output.getvalue())

    def test_post_record_report_warnings_do_not_become_a_capture_gate(self):
        report = {"status": "warnings", "summary": {"topic_count": 1},
                  "warnings": [{"code": "missing_stream", "detail": "No camera samples"}]}
        output = StringIO()
        with patch("vt_multisensor_alignment.capture_quality.generate_capture_quality", return_value=report) as generate:
            with redirect_stdout(output):
                status = main(["report-capture", "--bag", "/tmp/input-bag",
                               "--output", "/tmp/post-record-quality.json"])
        self.assertEqual(status, 0)
        generate.assert_called_once_with(bag_path=Path("/tmp/input-bag"),
                                         output_path=Path("/tmp/post-record-quality.json"), config=None)
        result = json.loads(output.getvalue())
        self.assertEqual(result["status"], "warnings")
        self.assertEqual(result["warnings"], report["warnings"])

    def test_post_record_report_loads_optional_config_without_extrinsics(self):
        config = object()
        with patch("vt_multisensor_alignment.cli.load_config", return_value=config) as load:
            with patch("vt_multisensor_alignment.capture_quality.generate_capture_quality", return_value={
                    "status": "ok", "summary": {}, "warnings": []}) as generate:
                with redirect_stdout(StringIO()):
                    status = main(["report-capture", "--bag", "/tmp/input-bag",
                                   "--output", "/tmp/quality.json", "--config", "/tmp/alignment.yaml"])
        self.assertEqual(status, 0)
        load.assert_called_once_with(Path("/tmp/alignment.yaml"))
        self.assertIs(generate.call_args.kwargs["config"], config)

    def test_post_record_io_error_is_reported_without_traceback(self):
        output = StringIO()
        with patch("vt_multisensor_alignment.capture_quality.generate_capture_quality",
                   side_effect=FileExistsError("report already exists")):
            with redirect_stdout(output):
                status = main(["report-capture", "--bag", "/tmp/input-bag", "--output", "/tmp/quality.json"])
        self.assertEqual(status, 1)
        self.assertIn("report already exists", output.getvalue())

    def test_tactile_export_prints_provenance_and_zero_coverage_warning(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary)
            document = {"provenance": {"status": "unverified_missing_manifest"},
                        "quality_summary": {"verdict": "WARNING", "empty_frames": 2}}
            (target / "manifest.json").write_text(json.dumps(document))
            output = StringIO()
            with patch("vt_multisensor_alignment.cli.export_tactile", return_value=target) as export:
                with redirect_stdout(output):
                    status = main(["export-tactile", "--bag", "/tmp/input-bag",
                                   "--aligned-frames", "/tmp/aligned_frames.jsonl",
                                   "--output", str(target), "--require-provenance"])
            self.assertEqual(status, 0)
            self.assertTrue(export.call_args.kwargs["require_provenance"])
            result = json.loads(output.getvalue())
            self.assertEqual(result["provenance"], document["provenance"])
            self.assertEqual(result["quality"]["verdict"], "WARNING")


if __name__ == "__main__":
    unittest.main()
