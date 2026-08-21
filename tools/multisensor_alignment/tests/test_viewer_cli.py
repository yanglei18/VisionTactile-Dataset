from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import stat
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from PIL import Image

from vt_multisensor_alignment.model import MessageRef, Transform
from vt_multisensor_alignment.sdk_model import AlignedFrame, CameraSample, ImageData
from vt_multisensor_alignment.viewer_cli import main


def aligned_frame(index: int = 0) -> AlignedFrame:
    reference = MessageRef("/camera/image", index, 100 + index, 90 + index)
    color = ImageData(
        np.full((2, 3, 3), [10, 20, 30], dtype=np.uint8),
        "rgb8",
        "camera_optical_frame",
        90 + index,
        reference,
    )
    depth = ImageData(
        np.full((2, 3), 1_000, dtype=np.uint16),
        "16UC1",
        "camera_optical_frame",
        90 + index,
        reference,
    )
    sample = CameraSample(
        camera_name="d405_1",
        host_realtime_ns=1_000 + index * 100,
        source_timestamp_ns=90 + index,
        delta_ns=0,
        color=color,
        depth=depth,
        timing=None,
        timing_reference=reference,
        attached_tracker=None,
        world_from_camera=Transform.identity(),
    )
    return AlignedFrame(
        frame_index=index,
        reference_camera="d405_1",
        reference_time_ns=1_000 + index * 100,
        cameras={"d405_1": sample},
        trackers={"left_wrist": None},
        additional_streams={},
        quality_flags=(),
    )


class FakeDataset:
    camera_names = ("d405_1",)
    reference_times_ns = (1_000, 1_100)

    def __init__(self) -> None:
        self.frame_calls: list[tuple[int, dict[str, object]]] = []
        self.closed = False

    def __len__(self) -> int:
        return 2

    def frame(self, index: int, **kwargs: object) -> AlignedFrame:
        self.frame_calls.append((index, kwargs))
        return aligned_frame(index)

    def __enter__(self) -> "FakeDataset":
        return self

    def __exit__(self, *args: object) -> None:
        self.closed = True


class DatasetOpener:
    def __init__(self) -> None:
        self.dataset = FakeDataset()
        self.calls: list[tuple[Path, Path, dict[str, object]]] = []

    def __call__(
        self, alignment: Path, bag: Path, **kwargs: object
    ) -> FakeDataset:
        self.calls.append((alignment, bag, kwargs))
        return self.dataset


class ViewerCliTests(unittest.TestCase):
    def test_version_does_not_open_dataset(self) -> None:
        opener = DatasetOpener()
        output = StringIO()

        with redirect_stdout(output):
            status = main(["--version"], _dataset_opener=opener)

        self.assertEqual(status, 0)
        self.assertEqual(output.getvalue().strip(), "vt-multisensor-view 0.3.0")
        self.assertEqual(opener.calls, [])

    def test_exports_a_headless_png_without_loading_timing_or_extensions(self) -> None:
        opener = DatasetOpener()
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "snapshot.png"
            output = StringIO()
            with redirect_stdout(output):
                status = main(
                    [
                        "--alignment",
                        "/data/aligned",
                        "--bag",
                        "/data/source_bag",
                        "--start",
                        "0",
                        "--width",
                        "800",
                        "--height",
                        "480",
                        "--export-frame",
                        str(target),
                    ],
                    _dataset_opener=opener,
                )

            self.assertEqual(status, 0)
            self.assertTrue(target.is_file())
            with Image.open(target) as exported:
                self.assertEqual(exported.size, (800, 480))
                self.assertEqual(exported.format, "PNG")
            self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o600)
        self.assertEqual(
            opener.dataset.frame_calls,
            [
                (
                    0,
                    {
                        "include_timing": False,
                        "additional_streams": (),
                    },
                )
            ],
        )
        self.assertTrue(opener.dataset.closed)

    def test_interactive_mode_normalizes_negative_start_and_passes_options(self) -> None:
        opener = DatasetOpener()
        calls: list[tuple[object, int, float, object]] = []

        def runner(dataset, *, start_index, speed, config):
            calls.append((dataset, start_index, speed, config))

        status = main(
            [
                "--alignment",
                "/data/aligned",
                "--bag",
                "/data/source_bag",
                "--start",
                "-1",
                "--speed",
                "2",
                "--allow-rejected",
                "--skip-integrity",
            ],
            _dataset_opener=opener,
            _interactive_runner=runner,
        )

        self.assertEqual(status, 0)
        self.assertEqual(calls[0][1:3], (1, 2.0))
        self.assertEqual(
            opener.calls[0][2],
            {"allow_rejected": True, "verify_integrity": False},
        )
        self.assertTrue(opener.dataset.closed)

    def test_refuses_to_overwrite_snapshot(self) -> None:
        opener = DatasetOpener()
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "snapshot.png"
            target.write_bytes(b"existing")
            output = StringIO()

            with redirect_stdout(output):
                status = main(
                    [
                        "--alignment",
                        "/data/aligned",
                        "--bag",
                        "/data/source_bag",
                        "--export-frame",
                        str(target),
                    ],
                    _dataset_opener=opener,
                )

        self.assertEqual(status, 1)
        self.assertIn("refusing to overwrite", output.getvalue())
        self.assertEqual(opener.dataset.frame_calls, [])

    def test_concurrent_snapshot_creator_is_not_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "snapshot.png"

            class RacingOpener(DatasetOpener):
                def __call__(self, alignment, bag, **kwargs):
                    target.write_bytes(b"concurrent-owner")
                    return super().__call__(alignment, bag, **kwargs)

            opener = RacingOpener()
            output = StringIO()
            with redirect_stdout(output):
                status = main(
                    [
                        "--alignment",
                        "/data/aligned",
                        "--bag",
                        "/data/source_bag",
                        "--export-frame",
                        str(target),
                    ],
                    _dataset_opener=opener,
                )

            self.assertEqual(status, 1)
            self.assertIn("refusing to overwrite", output.getvalue())
            self.assertEqual(target.read_bytes(), b"concurrent-owner")

    def test_missing_pillow_reports_viewer_extra_before_opening_dataset(self) -> None:
        opener = DatasetOpener()
        output = StringIO()

        with mock.patch.dict(sys.modules, {"PIL": None}):
            with redirect_stdout(output):
                status = main(
                    [
                        "--alignment",
                        "/data/aligned",
                        "--bag",
                        "/data/source_bag",
                    ],
                    _dataset_opener=opener,
                    _interactive_runner=lambda *args, **kwargs: None,
                )

        self.assertEqual(status, 1)
        self.assertIn(
            "pip install 'vt-multisensor-alignment[viewer]'",
            output.getvalue(),
        )
        self.assertEqual(opener.calls, [])


if __name__ == "__main__":
    unittest.main()
