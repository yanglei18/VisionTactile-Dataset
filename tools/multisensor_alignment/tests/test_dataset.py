import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import numpy as np
import yaml

from vt_multisensor_alignment.dataset import AlignedDataset
from vt_multisensor_alignment.errors import (
    DatasetClosedError,
    DatasetFormatError,
    IntegrityError,
    RejectedDatasetError,
    SourceBagMismatchError,
)
from vt_multisensor_alignment.sdk_model import (
    CameraInfoData,
    RegionOfInterestData,
)


CAMERAS = ("d405_1", "d405_2", "d436")
ROLES = ("left_wrist", "right_wrist", "torso")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def message_ref(topic: str, sequence: int, bag_time: int, source: int) -> dict[str, object]:
    return {
        "topic": topic,
        "sequence": sequence,
        "bag_timestamp_ns": bag_time,
        "source_timestamp_ns": source,
    }


def transform(x: float) -> dict[str, object]:
    return {
        "translation_m": [x, 0.0, 0.0],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
    }


def pose(role: str, frame_time: int, x: float) -> dict[str, object]:
    return {
        "role": role,
        "tracker_id": {"left_wrist": "1", "right_wrist": "2", "torso": "3"}[role] * 64,
        "timestamp_ns": frame_time,
        "bracket_gap_ns": 20,
        "before_sequence": 1,
        "after_sequence": 2,
        "world_from_tracker": transform(x),
    }


def frame_document(index: int, pixel: int = 7) -> dict[str, object]:
    frame_time = 1_000 + index * 100
    source = 10_000 + index * 100
    base = 2_000 + index * 100
    camera = {
        "host_realtime_ns": frame_time,
        "source_timestamp_ns": source,
        "delta_ns": 0,
        "color": message_ref("/d405_1/color", index, base, source),
        "depth": message_ref("/d405_1/depth", index, base + 1, source),
        "timing": message_ref("/d405_1/timing", index, base + 2, source),
        "attached_tracker": pose("left_wrist", frame_time, float(index)),
        "world_from_camera": transform(float(index) + 0.5),
    }
    return {
        "frame_index": index,
        "reference_camera": "d405_1",
        "reference_time_ns": frame_time,
        "cameras": {"d405_1": camera, "d405_2": None, "d436": None},
        "trackers": {
            role: pose(role, frame_time, float(index + role_index))
            for role_index, role in enumerate(ROLES)
        },
        "additional_streams": {
            "left_glove": {
                "timestamp_ns": frame_time - 1,
                "delta_ns": -1,
                "message": message_ref(
                    "/gloves/left/state", index, base + 3, frame_time - 1
                ),
            }
        },
        "quality_flags": [f"pixel:{pixel}"],
    }


def stream_catalog() -> dict[str, object]:
    streams: list[dict[str, object]] = []
    for camera in CAMERAS:
        for suffix, type_name in (
            ("color", "sensor_msgs/msg/Image"),
            ("depth", "sensor_msgs/msg/Image"),
            ("camera_info", "sensor_msgs/msg/CameraInfo"),
            ("timing", "vt_camera_msgs/msg/CameraFrameTiming"),
        ):
            streams.append(
                {
                    "topic": f"/{camera}/{suffix}",
                    "type": type_name,
                    "contract": "core",
                    "required": True,
                    "present": True,
                    "message_count": 2,
                    "accepted_count": 2,
                }
            )
    for role in ROLES:
        streams.append(
            {
                "topic": f"/vive/{role}/sample",
                "type": "vt_tracker_msgs/msg/TrackerSample",
                "contract": "core",
                "required": True,
                "present": True,
                "message_count": 4,
                "accepted_count": 4,
            }
        )
    streams.append(
        {
            "topic": "/gloves/left/state",
            "type": "example_msgs/msg/GloveState",
            "contract": "extension",
            "required": False,
            "present": True,
            "message_count": 2,
            "accepted_count": 2,
            "stream_name": "left_glove",
            "timestamp_field": "header.stamp",
            "selection_strategy": "previous",
            "max_delta_ms": 20.0,
        }
    )
    return {
        "schema_version": 1,
        "bag_contract": "unified-dataset-v1",
        "streams": streams,
        "incomplete_camera_groups": {name: 0 for name in CAMERAS},
    }


def write_fixture(
    root: Path,
    *,
    verdict: str = "ACCEPTED",
    manifest_storage: str = "mcap",
    frames: list[dict[str, object]] | None = None,
) -> tuple[Path, Path]:
    bag = root / "source_bag"
    bag.mkdir()
    metadata = {
        "rosbag2_bagfile_information": {
            "version": 9,
            "storage_identifier": "mcap",
        }
    }
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8"
    )
    output = root / "aligned"
    output.mkdir()
    documents = frames if frames is not None else [frame_document(0), frame_document(1)]
    (output / "stream_catalog.json").write_text(
        json.dumps(stream_catalog()) + "\n", encoding="utf-8"
    )
    (output / "aligned_frames.jsonl").write_text(
        "".join(json.dumps(document) + "\n" for document in documents),
        encoding="utf-8",
    )
    (output / "timing_residuals.csv").write_text(
        "frame_index,camera,camera_delta_ns,attached_tracker_gap_ns\n",
        encoding="utf-8",
    )
    quality = {
        "verdict": verdict,
        "reference_frame_count": len(documents),
        "rejection_reasons": [] if verdict == "ACCEPTED" else ["test_rejection"],
    }
    (output / "quality_report.json").write_text(
        json.dumps(quality) + "\n", encoding="utf-8"
    )
    (output / "diagnostics.svg").write_text("<svg/>\n", encoding="utf-8")
    integrity_names = (
        "aligned_frames.jsonl",
        "diagnostics.svg",
        "quality_report.json",
        "stream_catalog.json",
        "timing_residuals.csv",
    )
    manifest = {
        "schema_version": 1,
        "tool": "vt-multisensor-alignment",
        "tool_version": "0.1.0",
        "bag_contract": "unified-dataset-v1",
        "source_bag": {
            "name": bag.name,
            "storage_identifier": manifest_storage,
            "metadata_sha256": sha256(bag / "metadata.yaml"),
        },
        "configuration": {
            "reference_camera": "d405_1",
            "world_frame": "vive_map",
        },
        "tracker_ids": {
            "left_wrist": "1" * 64,
            "right_wrist": "2" * 64,
            "torso": "3" * 64,
        },
        "extrinsics": {
            name: {
                "camera_name": name,
                "camera_frame": f"{name}_color_optical_frame",
                "tracker_role": ROLES[index],
            }
            for index, name in enumerate(CAMERAS)
        },
        "aligned_frame_count": len(documents),
        "verdict": verdict,
        "files": {
            name: {
                "size_bytes": (output / name).stat().st_size,
                "sha256": sha256(output / name),
            }
            for name in integrity_names
        },
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest) + "\n", encoding="utf-8"
    )
    return output, bag


def ros_stamp(value_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        sec=value_ns // 1_000_000_000,
        nanosec=value_ns % 1_000_000_000,
    )


class FakeResolver:
    def __init__(self) -> None:
        self.resolve_calls: list[tuple[object, ...]] = []
        self.camera_info_calls = 0
        self.closed = False

    def resolve_many(self, references):
        values = tuple(references)
        self.resolve_calls.append(values)
        result = {}
        for reference in values:
            header = SimpleNamespace(
                stamp=ros_stamp(reference.source_timestamp_ns),
                frame_id="d405_1_color_optical_frame",
            )
            if reference.topic.endswith("/color"):
                result[reference] = SimpleNamespace(
                    header=header,
                    width=1,
                    height=1,
                    encoding="rgb8",
                    step=3,
                    data=bytes([7, 8, 9]),
                    is_bigendian=0,
                )
            elif reference.topic.endswith("/depth"):
                result[reference] = SimpleNamespace(
                    header=header,
                    width=1,
                    height=1,
                    encoding="16UC1",
                    step=2,
                    data=bytes([101, 0]),
                    is_bigendian=0,
                )
            elif reference.topic == "/gloves/left/state":
                result[reference] = SimpleNamespace(header=header, value=11)
            else:
                result[reference] = SimpleNamespace(header=header, valid=True)
        return result

    def load_camera_info(self, *, camera_frames, camera_info_topics):
        self.camera_info_calls += 1
        return {
            name: CameraInfoData(
                camera_name=name,
                frame_id=frame,
                width=640,
                height=480,
                distortion_model="plumb_bob",
                d=np.zeros(5),
                k=np.eye(3),
                r=np.eye(3),
                p=np.hstack((np.eye(3), np.zeros((3, 1)))),
                binning_x=0,
                binning_y=0,
                roi=RegionOfInterestData(0, 0, 0, 0, False),
            )
            for name, frame in camera_frames.items()
        }

    def close(self) -> None:
        self.closed = True


class ResolverFactory:
    def __init__(self) -> None:
        self.calls: list[dict[str, object]] = []
        self.instances: list[FakeResolver] = []

    def __call__(self, **kwargs) -> FakeResolver:
        self.calls.append(kwargs)
        instance = FakeResolver()
        self.instances.append(instance)
        return instance


class DatasetTests(unittest.TestCase):
    def test_open_indexes_records_without_opening_rosbag(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()

            with AlignedDataset.open(
                output, bag, _resolver_factory=factory
            ) as dataset:
                self.assertEqual(len(dataset), 2)
                self.assertEqual(dataset.camera_names, CAMERAS)
                self.assertEqual(dataset.tracker_roles, ROLES)
                self.assertEqual(dataset.additional_stream_names, ("left_glove",))
                self.assertEqual(dataset.record(-1).frame_index, 1)
                self.assertEqual(factory.calls, [])
                with self.assertRaises(TypeError):
                    dataset.manifest["verdict"] = "REJECTED"

    def test_open_rejects_tampered_alignment_output(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            with (output / "aligned_frames.jsonl").open("a", encoding="utf-8") as stream:
                stream.write("{}\n")

            with self.assertRaises(IntegrityError):
                AlignedDataset.open(output, bag)

    def test_default_integrity_check_classifies_missing_output_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            (output / "diagnostics.svg").unlink()

            with self.assertRaisesRegex(IntegrityError, "file set"):
                AlignedDataset.open(output, bag)

    def test_integrity_can_be_skipped_without_skipping_structure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            (output / "diagnostics.svg").write_text("<abc/>\n", encoding="utf-8")

            with AlignedDataset.open(
                output, bag, verify_integrity=False
            ) as dataset:
                self.assertEqual(len(dataset), 2)

            frame_path = output / "aligned_frames.jsonl"
            original_size = frame_path.stat().st_size
            frame_path.write_bytes(b"x" * (original_size - 1) + b"\n")
            with self.assertRaises(DatasetFormatError):
                AlignedDataset.open(output, bag, verify_integrity=False)

    def test_skipping_hashes_still_checks_inventory_and_file_sizes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            manifest_path = output / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            manifest["files"].pop("diagnostics.svg")
            manifest_path.write_text(json.dumps(manifest) + "\n", encoding="utf-8")

            with self.assertRaisesRegex(DatasetFormatError, "inventory"):
                AlignedDataset.open(output, bag, verify_integrity=False)

        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            with (output / "diagnostics.svg").open("a", encoding="utf-8") as stream:
                stream.write("larger\n")

            with self.assertRaisesRegex(IntegrityError, "size"):
                AlignedDataset.open(output, bag, verify_integrity=False)

    def test_rejected_verdict_requires_explicit_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary), verdict="REJECTED")

            with self.assertRaises(RejectedDatasetError):
                AlignedDataset.open(output, bag)
            with AlignedDataset.open(output, bag, allow_rejected=True) as dataset:
                self.assertEqual(dataset.quality_report["verdict"], "REJECTED")

    def test_source_bag_name_metadata_and_storage_are_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            output, bag = write_fixture(root)
            wrong_name = root / "wrong_name"
            wrong_name.mkdir()
            (wrong_name / "metadata.yaml").write_bytes((bag / "metadata.yaml").read_bytes())
            with self.assertRaisesRegex(SourceBagMismatchError, "name"):
                AlignedDataset.open(output, wrong_name)

        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            with (bag / "metadata.yaml").open("a", encoding="utf-8") as stream:
                stream.write("changed: true\n")
            with self.assertRaisesRegex(SourceBagMismatchError, "SHA-256"):
                AlignedDataset.open(output, bag)

        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary), manifest_storage="sqlite3")
            with self.assertRaisesRegex(SourceBagMismatchError, "storage"):
                AlignedDataset.open(output, bag)

    def test_open_rejects_non_contiguous_rows_and_bad_cache_size(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(
                Path(temporary), frames=[frame_document(1)]
            )
            with self.assertRaisesRegex(DatasetFormatError, "frame_index"):
                AlignedDataset.open(output, bag)
            with self.assertRaises(ValueError):
                AlignedDataset.open(output, bag, cache_size=-1)

    def test_frame_loads_only_selected_payloads(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            with AlignedDataset.open(
                output, bag, _resolver_factory=factory
            ) as dataset:
                frame = dataset.frame(
                    0,
                    cameras=("d405_1",),
                    image_kinds=("color",),
                    include_timing=False,
                    additional_streams=(),
                )

                self.assertEqual(tuple(frame.cameras), ("d405_1",))
                camera = frame.cameras["d405_1"]
                assert camera is not None
                np.testing.assert_array_equal(camera.color.array, [[[7, 8, 9]]])
                self.assertIsNone(camera.depth)
                self.assertIsNone(camera.timing)
                self.assertEqual(dict(frame.additional_streams), {})
                self.assertEqual(len(factory.instances[0].resolve_calls[0]), 1)

    def test_default_frame_preserves_null_cameras_and_loads_extension(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            with AlignedDataset.open(
                output, bag, _resolver_factory=factory
            ) as dataset:
                frame = dataset.frame(0)

                self.assertIsNone(frame.cameras["d405_2"])
                self.assertIsNone(frame.cameras["d436"])
                first = frame.cameras["d405_1"]
                assert first is not None
                np.testing.assert_array_equal(first.depth.array, [[101]])
                self.assertTrue(first.timing.valid)
                self.assertEqual(frame.additional_streams["left_glove"].message.value, 11)
                self.assertEqual(
                    frame.trackers["torso"].world_from_tracker.as_matrix().shape,
                    (4, 4),
                )

    def test_selection_validation_happens_before_rosbag_io(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            with AlignedDataset.open(
                output, bag, _resolver_factory=factory
            ) as dataset:
                cases = (
                    {"cameras": ("unknown",)},
                    {"cameras": ("d405_1", "d405_1")},
                    {"image_kinds": ("infrared",)},
                    {"additional_streams": ("unknown",)},
                )
                for values in cases:
                    with self.subTest(values=values):
                        with self.assertRaises(ValueError):
                            dataset.frame(0, **values)
                self.assertEqual(factory.calls, [])

    def test_frame_cache_hits_and_evicts_by_complete_selection(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            with AlignedDataset.open(
                output, bag, cache_size=1, _resolver_factory=factory
            ) as dataset:
                first = dataset.frame(0)
                again = dataset.frame(0)
                second = dataset.frame(1)
                reloaded = dataset.frame(0)

                self.assertIs(first, again)
                self.assertIsNot(first, reloaded)
                self.assertEqual(second.frame_index, 1)
                self.assertEqual(len(factory.instances[0].resolve_calls), 3)

    def test_iter_frames_uses_python_slice_bounds_and_positive_step(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            with AlignedDataset.open(
                output, bag, _resolver_factory=factory
            ) as dataset:
                values = list(
                    dataset.iter_frames(
                        start=-2,
                        stop=None,
                        step=1,
                        image_kinds=(),
                        include_timing=False,
                        additional_streams=(),
                    )
                )
                self.assertEqual([value.frame_index for value in values], [0, 1])
                self.assertEqual(factory.calls, [])
                with self.assertRaises(ValueError):
                    list(dataset.iter_frames(step=0))
                with self.assertRaises(ValueError):
                    list(dataset.iter_frames(step=-1))

    def test_iter_frames_normalizes_generator_selections_once(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            with AlignedDataset.open(output, bag) as dataset:
                frames = list(
                    dataset.iter_frames(
                        cameras=(name for name in ("d405_1",)),
                        image_kinds=(),
                        include_timing=False,
                        additional_streams=(),
                    )
                )

                self.assertEqual(
                    [tuple(frame.cameras) for frame in frames],
                    [("d405_1",), ("d405_1",)],
                )

    def test_camera_info_is_lazy_and_cached(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            with AlignedDataset.open(
                output, bag, _resolver_factory=factory
            ) as dataset:
                first = dataset.camera_info
                second = dataset.camera_info

                self.assertIs(first, second)
                self.assertEqual(first["d436"].frame_id, "d436_color_optical_frame")
                self.assertEqual(factory.instances[0].camera_info_calls, 1)

    def test_close_is_idempotent_and_invalidates_access(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output, bag = write_fixture(Path(temporary))
            factory = ResolverFactory()
            dataset = AlignedDataset.open(output, bag, _resolver_factory=factory)
            dataset.frame(0)

            dataset.close()
            dataset.close()

            self.assertTrue(factory.instances[0].closed)
            with self.assertRaises(DatasetClosedError):
                dataset.record(0)
            with self.assertRaises(DatasetClosedError):
                len(dataset)


if __name__ == "__main__":
    unittest.main()
