import json
from pathlib import Path
import stat
import tempfile
import unittest

import numpy as np

from vt_multisensor_alignment.aligner import AlignmentResult
from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.export import export_result, validate_output
from vt_multisensor_alignment.extrinsics import ExtrinsicBinding
from vt_multisensor_alignment.model import BagDataset, Transform


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_FILES = {
    "manifest.json",
    "stream_catalog.json",
    "aligned_frames.jsonl",
    "timing_residuals.csv",
    "quality_report.json",
    "diagnostics.svg",
}


class ExportTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config_path = ROOT / "config" / "alignment.example.yaml"
        self.config = load_config(self.config_path)

    def test_export_is_atomic_private_and_self_validating(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            bag = parent / "bag"
            bag.mkdir()
            (bag / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
            target = parent / "aligned"

            export_result(
                output_directory=target,
                config_path=self.config_path,
                config=self.config,
                dataset=self._dataset(bag),
                extrinsics=self._extrinsics(parent),
                result=self._result(),
            )

            self.assertEqual({path.name for path in target.iterdir()}, OUTPUT_FILES)
            self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o700)
            for path in target.iterdir():
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            manifest = json.loads(
                (target / "manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["schema_version"], 1)
            self.assertEqual(manifest["aligned_frame_count"], 1)
            self.assertNotIn(str(parent), json.dumps(manifest))
            validation = validate_output(target)
            self.assertEqual(validation["verdict"], "ACCEPTED")
            self.assertEqual(validation["aligned_frame_count"], 1)

    def test_export_refuses_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            bag = parent / "bag"
            bag.mkdir()
            (bag / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
            target = parent / "aligned"
            target.mkdir()

            with self.assertRaisesRegex(FileExistsError, "refusing to overwrite"):
                export_result(
                    output_directory=target,
                    config_path=self.config_path,
                    config=self.config,
                    dataset=self._dataset(bag),
                    extrinsics=self._extrinsics(parent),
                    result=self._result(),
                )

    def test_validation_detects_changed_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            bag = parent / "bag"
            bag.mkdir()
            (bag / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
            target = parent / "aligned"
            export_result(
                output_directory=target,
                config_path=self.config_path,
                config=self.config,
                dataset=self._dataset(bag),
                extrinsics=self._extrinsics(parent),
                result=self._result(),
            )
            with (target / "aligned_frames.jsonl").open(
                "a", encoding="utf-8"
            ) as stream:
                stream.write("{}\n")

            with self.assertRaisesRegex(ValueError, "integrity mismatch"):
                validate_output(target)

    def _dataset(self, bag: Path) -> BagDataset:
        return BagDataset(
            bag_path=bag.resolve(),
            storage_identifier="mcap",
            topic_types={"/d405_1/color/image_raw": "sensor_msgs/msg/Image"},
            message_counts={"/d405_1/color/image_raw": 1},
            accepted_counts={"/d405_1/color/image_raw": 1},
            camera_frames={name: () for name in self.config.camera_by_name},
            tracker_poses={role: () for role in self.config.tracker_by_role},
            additional_samples={},
            tracker_ids={
                "left_wrist": "1" * 64,
                "right_wrist": "2" * 64,
                "torso": "3" * 64,
            },
            incomplete_camera_groups={name: 0 for name in self.config.camera_by_name},
            clock_observations={},
        )

    def _extrinsics(self, parent: Path) -> dict[str, ExtrinsicBinding]:
        result = {}
        for camera in self.config.cameras:
            tracker = self.config.tracker_by_role[camera.tracker_role]
            result[camera.name] = ExtrinsicBinding(
                camera_name=camera.name,
                camera_model=camera.model,
                camera_serial=camera.serial,
                camera_frame=camera.frame_id,
                tracker_role=camera.tracker_role,
                tracker_id={
                    "left_wrist": "1" * 64,
                    "right_wrist": "2" * 64,
                    "torso": "3" * 64,
                }[camera.tracker_role],
                tracker_frame=tracker.frame_id,
                world_frame=self.config.world_frame,
                tracker_from_camera=Transform(
                    np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0])
                ),
                source_path=(parent / f"{camera.name}.yaml").resolve(),
                sha256="f" * 64,
            )
        return result

    def _result(self) -> AlignmentResult:
        quality = {
            "verdict": "ACCEPTED",
            "reference_frame_count": 1,
            "camera_match_ratio": {
                "d405_1": 1.0,
                "d405_2": 1.0,
                "d436": 1.0,
            },
            "tracker_reference_coverage_ratio": {
                "left_wrist": 1.0,
                "right_wrist": 1.0,
                "torso": 1.0,
            },
            "attached_tracker_coverage_ratio": {
                "d405_1": 1.0,
                "d405_2": 1.0,
                "d436": 1.0,
            },
            "additional_stream_coverage_ratio": {},
            "thresholds": {},
            "clock_audits": [],
            "rejection_reasons": [],
        }
        return AlignmentResult(
            records=(
                {
                    "frame_index": 0,
                    "reference_time_ns": 1_000,
                    "cameras": {},
                    "trackers": {},
                    "additional_streams": {},
                    "quality_flags": [],
                },
            ),
            timing_residuals=(
                {
                    "frame_index": 0,
                    "camera": "d405_1",
                    "camera_delta_ns": 0,
                    "attached_tracker_gap_ns": 10,
                },
            ),
            quality=quality,
            clock_audits=(),
            rejection_reasons=(),
        )


if __name__ == "__main__":
    unittest.main()
