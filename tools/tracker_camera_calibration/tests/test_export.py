import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from vt_tracker_camera_calib.bag_reader import BagObservations
from vt_tracker_camera_calib.config import load_config
from vt_tracker_camera_calib.export import export_result
from vt_tracker_camera_calib.handeye import (
    CalibrationSolution,
    MethodScore,
    Residual,
)
from vt_tracker_camera_calib.model import CalibrationPair, CameraIntrinsics
from vt_tracker_camera_calib.transforms import Transform


ROOT = Path(__file__).resolve().parents[1]


class ExportTest(unittest.TestCase):
    def test_exports_versioned_machine_and_human_readable_artifacts(self) -> None:
        config_path = ROOT / "config" / "calibration.example.yaml"
        config = load_config(config_path)
        transform = Transform.from_rvec_tvec([0.1, 0.2, 0.3], [0.01, -0.02, 0.03])
        pair = CalibrationPair(
            timestamp_ns=1,
            world_from_tracker=Transform.identity(),
            camera_from_board=Transform.identity(),
            reprojection_rms_px=0.4,
            corner_count=30,
            interpolation_gap_ns=1,
            local_translation_motion_m=0.0,
            local_rotation_motion_deg=0.0,
        )
        residual = Residual(0, 0.003, 0.3, 0.4)
        solution = CalibrationSolution(
            tracker_from_camera=transform,
            method="PARK",
            pair_count=40,
            training_count=32,
            holdout_count=8,
            validation_translation_rms_m=0.004,
            validation_rotation_rms_deg=0.4,
            all_translation_rms_m=0.003,
            all_rotation_rms_deg=0.3,
            reprojection_rms_px=0.4,
            quality="TARGET",
            method_scores=(MethodScore("PARK", 0.004, 0.4, 0.8, True),),
            residuals=(residual,),
        )
        observations = BagObservations(
            intrinsics=CameraIntrinsics(
                1280,
                720,
                np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 360.0], [0.0, 0.0, 1.0]]),
                np.zeros(5),
            ),
            tracker_samples=(),
            board_observations=(),
            tracker_id="a" * 64,
            image_count=100,
            timed_image_count=99,
            rejected_image_count=60,
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result"
            exported = export_result(
                output_directory=output,
                config_path=config_path,
                bag_path=Path("/private/session/calibration_01"),
                config=config,
                observations=observations,
                selected_pairs=(pair,),
                solution=solution,
            )
            self.assertEqual(exported, output.resolve())
            self.assertEqual(
                set(path.name for path in output.iterdir()),
                {"extrinsics.yaml", "report.json", "residuals.csv", "diagnostics.svg"},
            )
            external = yaml.safe_load((output / "extrinsics.yaml").read_text())
            self.assertEqual(external["status"], "VALID")
            self.assertEqual(external["transform"]["semantics"], "parent_from_child")
            self.assertEqual(external["provenance"]["source_bag_name"], "calibration_01")
            report = json.loads((output / "report.json").read_text())
            self.assertEqual(report["quality"], "TARGET")
            with self.assertRaises(FileExistsError):
                export_result(
                    output_directory=output,
                    config_path=config_path,
                    bag_path="ignored",
                    config=config,
                    observations=observations,
                    selected_pairs=(pair,),
                    solution=solution,
                )


if __name__ == "__main__":
    unittest.main()
