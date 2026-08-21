from pathlib import Path
import unittest

import numpy as np

from vt_multisensor_alignment.aligner import align_dataset
from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.extrinsics import ExtrinsicBinding
from vt_multisensor_alignment.model import (
    BagDataset,
    CameraFrame,
    ClockObservation,
    MessageRef,
    TimedPose,
    Transform,
)


ROOT = Path(__file__).resolve().parents[1]
TRACKER_IDS = {
    "left_wrist": "1" * 64,
    "right_wrist": "2" * 64,
    "torso": "3" * 64,
}


def reference(topic: str, sequence: int, source: int) -> MessageRef:
    return MessageRef(topic, sequence, source + 10, source)


def camera_frame(name: str, sequence: int, realtime: int) -> CameraFrame:
    source = 50_000 + sequence
    return CameraFrame(
        camera_name=name,
        source_timestamp_ns=source,
        host_realtime_ns=realtime,
        host_monotonic_ns=realtime - 1_000,
        color=reference(f"/{name}/color/image_raw", sequence, source),
        depth=reference(f"/{name}/depth/image_rect_raw", sequence, source),
        timing=reference(f"/{name}/frame_timing", sequence, source),
    )


def tracker_pose(role: str, sequence: int, realtime: int, x: float) -> TimedPose:
    return TimedPose(
        role=role,
        tracker_id=TRACKER_IDS[role],
        host_realtime_ns=realtime,
        host_monotonic_ns=realtime - 2_000,
        transform=Transform(
            np.array([x, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])
        ),
        reference=reference(f"/vive/{role}/sample", sequence, realtime),
    )


class AlignerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_config(ROOT / "config" / "alignment.example.yaml")

    def test_aligns_all_streams_and_composes_world_from_camera(self) -> None:
        dataset = self._dataset()
        extrinsics = self._extrinsics()

        result = align_dataset(dataset, self.config, extrinsics)

        self.assertTrue(result.accepted)
        self.assertEqual(len(result.records), 2)
        first = result.records[0]
        self.assertEqual(first["reference_time_ns"], 1_000_000_000)
        self.assertEqual(first["cameras"]["d405_2"]["delta_ns"], 1_000_000)
        np.testing.assert_allclose(
            first["cameras"]["d405_1"]["world_from_camera"]["translation_m"],
            [2.0, 0.0, 0.0],
        )
        self.assertEqual(
            set(first["trackers"]), {"left_wrist", "right_wrist", "torso"}
        )
        self.assertEqual(result.quality["verdict"], "ACCEPTED")

    def test_clock_jump_rejects_result_without_discarding_rows(self) -> None:
        dataset = self._dataset(camera_clock_jump=True)

        result = align_dataset(dataset, self.config, self._extrinsics())

        self.assertFalse(result.accepted)
        self.assertEqual(len(result.records), 2)
        self.assertIn("clock_audit_failed:camera:d436", result.rejection_reasons)

    def _dataset(self, *, camera_clock_jump: bool = False) -> BagDataset:
        camera_frames = {}
        for camera in self.config.cameras:
            second_realtime = 1_010_000_000
            values = [
                camera_frame(camera.name, 0, 1_000_000_000),
                camera_frame(camera.name, 1, second_realtime),
            ]
            if camera_clock_jump and camera.name == "d436":
                value = values[1]
                values[1] = CameraFrame(
                    camera_name=value.camera_name,
                    source_timestamp_ns=value.source_timestamp_ns,
                    host_realtime_ns=1_030_000_000,
                    host_monotonic_ns=value.host_monotonic_ns,
                    color=value.color,
                    depth=value.depth,
                    timing=value.timing,
                )
            elif camera.name == "d405_2":
                values = [
                    camera_frame(camera.name, 0, 1_001_000_000),
                    camera_frame(camera.name, 1, 1_011_000_000),
                ]
            camera_frames[camera.name] = tuple(values)
        tracker_poses = {
            role: (
                tracker_pose(role, 0, 999_000_000, 0.0),
                tracker_pose(role, 1, 1_001_000_000, 2.0),
                tracker_pose(role, 2, 1_009_000_000, 4.0),
                tracker_pose(role, 3, 1_011_000_000, 6.0),
            )
            for role in TRACKER_IDS
        }
        clocks = {
            **{
                f"camera:{name}": tuple(
                    ClockObservation(
                        frame.host_realtime_ns, frame.host_monotonic_ns
                    )
                    for frame in frames
                )
                for name, frames in camera_frames.items()
            },
            **{
                f"tracker:{role}": tuple(
                    ClockObservation(
                        pose.host_realtime_ns, pose.host_monotonic_ns
                    )
                    for pose in values
                )
                for role, values in tracker_poses.items()
            },
        }
        return BagDataset(
            bag_path=Path("/tmp/unified-bag"),
            storage_identifier="mcap",
            topic_types={},
            message_counts={},
            accepted_counts={},
            camera_frames=camera_frames,
            tracker_poses=tracker_poses,
            additional_samples={},
            tracker_ids=TRACKER_IDS,
            incomplete_camera_groups={name: 0 for name in camera_frames},
            clock_observations=clocks,
        )

    def _extrinsics(self) -> dict[str, ExtrinsicBinding]:
        result = {}
        for camera in self.config.cameras:
            tracker = self.config.tracker_by_role[camera.tracker_role]
            result[camera.name] = ExtrinsicBinding(
                camera_name=camera.name,
                camera_model=camera.model,
                camera_serial=camera.serial,
                camera_frame=camera.frame_id,
                tracker_role=camera.tracker_role,
                tracker_id=TRACKER_IDS[camera.tracker_role],
                tracker_frame=tracker.frame_id,
                world_frame=self.config.world_frame,
                tracker_from_camera=Transform(
                    np.array([1.0, 0.0, 0.0]),
                    np.array([0.0, 0.0, 0.0, 1.0]),
                ),
                source_path=Path(f"/tmp/{camera.name}.yaml"),
                sha256="f" * 64,
            )
        return result


if __name__ == "__main__":
    unittest.main()
