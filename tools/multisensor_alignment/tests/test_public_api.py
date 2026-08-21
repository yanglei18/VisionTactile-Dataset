from pathlib import Path
import tomllib
import unittest

import vt_multisensor_alignment as sdk
from vt_multisensor_alignment import export


ROOT = Path(__file__).resolve().parents[1]


class PublicApiTests(unittest.TestCase):
    def test_documented_sdk_values_are_exported_from_package_root(self) -> None:
        expected = {
            "AdditionalSample",
            "AlignedDataset",
            "AlignedFrame",
            "CameraInfoData",
            "CameraSample",
            "DatasetClosedError",
            "DatasetError",
            "DatasetFormatError",
            "ImageData",
            "IntegrityError",
            "MessageRef",
            "MissingMessageError",
            "RegionOfInterestData",
            "RejectedDatasetError",
            "SourceBagMismatchError",
            "TrackerPose",
            "Transform",
            "UnsupportedEncodingError",
        }

        self.assertEqual(set(sdk.__all__), expected)
        for name in expected:
            self.assertIsNotNone(getattr(sdk, name))

    def test_package_cli_and_project_versions_are_0_3_0(self) -> None:
        project = tomllib.loads(
            (ROOT / "pyproject.toml").read_text(encoding="utf-8")
        )

        self.assertEqual(sdk.__version__, "0.3.0")
        self.assertEqual(export.TOOL_VERSION, "0.3.0")
        self.assertEqual(project["project"]["version"], "0.3.0")
        self.assertEqual(
            project["project"]["scripts"]["vt-multisensor-view"],
            "vt_multisensor_alignment.viewer_cli:entrypoint",
        )
        self.assertEqual(
            project["project"]["optional-dependencies"]["viewer"],
            ["Pillow>=10,<12"],
        )


if __name__ == "__main__":
    unittest.main()
