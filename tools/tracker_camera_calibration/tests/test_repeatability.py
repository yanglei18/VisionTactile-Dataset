import json
import math
from pathlib import Path
import stat
import tempfile
import unittest
from unittest.mock import patch

import yaml

from vt_tracker_camera_calib.repeatability import (
    compare_extrinsics,
    write_repeatability_report,
)
from vt_tracker_camera_calib.transforms import Transform


IDENTITY = {
    "camera": {
        "name": "d405_1",
        "model": "D405",
        "serial": "260322278433",
        "frame_id": "d405_1_color_optical_frame",
    },
    "tracker": {
        "role": "torso",
        "tracker_id": "a" * 64,
        "frame_id": "vive_tracker_torso",
    },
}


def write_extrinsic(
    root: Path,
    run_name: str,
    transform: Transform,
    *,
    tracker_id: str = "a" * 64,
) -> Path:
    directory = root / run_name
    directory.mkdir()
    document = {
        "schema_version": 1,
        "status": "VALID",
        "camera": dict(IDENTITY["camera"]),
        "tracker": {**IDENTITY["tracker"], "tracker_id": tracker_id},
        "transform": {
            "semantics": "parent_from_child",
            "parent_frame": "vive_tracker_torso",
            "child_frame": "d405_1_color_optical_frame",
            "translation_m": transform.translation.tolist(),
            "quaternion_xyzw": transform.quaternion_xyzw.tolist(),
        },
    }
    path = directory / "extrinsics.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


class RepeatabilityTest(unittest.TestCase):
    def test_passes_three_consistent_runs_and_recommends_medoid(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = Transform.from_rvec_tvec(
                [0.1, -0.2, 0.05], [0.08, -0.03, 0.11]
            )
            paths = [
                write_extrinsic(
                    root,
                    "run01",
                    Transform.from_rvec_tvec(
                        [0.1, -0.2, 0.05], [0.078, -0.03, 0.11]
                    ),
                ),
                write_extrinsic(root, "run02", baseline),
                write_extrinsic(
                    root,
                    "run03",
                    Transform.from_rvec_tvec(
                        [0.1, -0.2, 0.052], [0.082, -0.03, 0.11]
                    ),
                ),
            ]
            report = compare_extrinsics(paths)
            self.assertEqual(report.status, "PASS")
            self.assertEqual(report.run_count, 3)
            self.assertEqual(report.recommended_input, "run02/extrinsics.yaml")
            self.assertLessEqual(report.maximum_translation_m, 0.005)
            self.assertLessEqual(report.maximum_rotation_deg, 0.5)

            output = write_repeatability_report(root / "repeatability.json", report)
            document = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(document["status"], "PASS")
            self.assertEqual(len(document["pairwise"]), math.comb(3, 2))
            self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o600)
            with self.assertRaises(FileExistsError):
                write_repeatability_report(output, report)

    def test_fails_when_pairwise_spread_exceeds_threshold(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = [
                write_extrinsic(root, "run01", Transform.identity()),
                write_extrinsic(
                    root,
                    "run02",
                    Transform.from_rvec_tvec([0.0, 0.0, 0.0], [0.003, 0.0, 0.0]),
                ),
                write_extrinsic(
                    root,
                    "run03",
                    Transform.from_rvec_tvec([0.0, 0.0, 0.0], [0.012, 0.0, 0.0]),
                ),
            ]
            report = compare_extrinsics(paths)
            self.assertEqual(report.status, "FAIL")
            self.assertGreater(report.maximum_translation_m, 0.005)

    def test_rejects_mixed_hardware_identity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = [
                write_extrinsic(root, "run01", Transform.identity()),
                write_extrinsic(root, "run02", Transform.identity()),
                write_extrinsic(
                    root,
                    "run03",
                    Transform.identity(),
                    tracker_id="b" * 64,
                ),
            ]
            with self.assertRaisesRegex(ValueError, "same camera"):
                compare_extrinsics(paths)

    def test_requires_three_distinct_valid_runs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = write_extrinsic(root, "run01", Transform.identity())
            with self.assertRaisesRegex(ValueError, "at least three"):
                compare_extrinsics([path, path])

    def test_failed_report_write_does_not_leave_partial_output(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            paths = [
                write_extrinsic(root, run, Transform.identity())
                for run in ("run01", "run02", "run03")
            ]
            report = compare_extrinsics(paths)
            output = root / "repeatability.json"
            with patch(
                "vt_tracker_camera_calib.repeatability.json.dumps",
                side_effect=RuntimeError("synthetic write failure"),
            ):
                with self.assertRaisesRegex(RuntimeError, "synthetic"):
                    write_repeatability_report(output, report)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
