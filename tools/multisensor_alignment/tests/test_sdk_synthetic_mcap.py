from pathlib import Path
import tempfile
import unittest

import numpy as np
import yaml

from vt_multisensor_alignment import AlignedDataset
from vt_multisensor_alignment.aligner import align_dataset
from vt_multisensor_alignment.bag_reader import configured_topic_types, read_unified_bag
from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.export import export_result
from vt_multisensor_alignment.extrinsics import load_extrinsics


ROOT = Path(__file__).resolve().parents[1]
TRACKER_IDS = {
    "left_wrist": "1" * 64,
    "right_wrist": "2" * 64,
    "torso": "3" * 64,
}


class SdkSyntheticMcapTests(unittest.TestCase):
    def test_sdk_reads_random_and_sequential_payloads_from_real_mcap(self) -> None:
        try:
            import rosbag2_py
            from builtin_interfaces.msg import Time
            from geometry_msgs.msg import PoseStamped
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import CameraInfo, Image
            from vt_camera_msgs.msg import CameraFrameTiming
            from vt_tracker_msgs.msg import TrackerSample
        except ImportError as error:
            self.skipTest(f"ROS 2 workspace is not sourced: {error}")

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config_path = root / "alignment.yaml"
            config_document = yaml.safe_load(
                (ROOT / "config" / "alignment.example.yaml").read_text(
                    encoding="utf-8"
                )
            )
            config_document["additional_streams"] = [
                {
                    "name": "left_glove",
                    "topic": "/gloves/left/state",
                    "type": "geometry_msgs/msg/PoseStamped",
                    "time_source": "header_stamp",
                    "timestamp_field": "header.stamp",
                    "strategy": "nearest",
                    "max_delta_ms": 20.0,
                    "required": True,
                }
            ]
            config_path.write_text(
                yaml.safe_dump(config_document, sort_keys=False),
                encoding="utf-8",
            )
            config = load_config(config_path)
            bag = root / "unified"
            writer = rosbag2_py.SequentialWriter()
            writer.open(
                rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""),
            )
            for identifier, (topic, type_name) in enumerate(
                configured_topic_types(config).items(), start=1
            ):
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        id=identifier,
                        name=topic,
                        type=type_name,
                        serialization_format="cdr",
                    )
                )

            def stamp(value_ns: int) -> Time:
                return Time(
                    sec=value_ns // 1_000_000_000,
                    nanosec=value_ns % 1_000_000_000,
                )

            events: list[tuple[int, str, object]] = []
            reference_times = (1_000_000_000, 1_010_000_000)
            for camera_index, camera in enumerate(config.cameras):
                info = CameraInfo()
                info.header.frame_id = camera.frame_id
                info.width = 1
                info.height = 1
                info.distortion_model = "plumb_bob"
                info.d = [0.0] * 5
                info.k = [1.0, 0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 0.0, 1.0]
                info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                info.p = [
                    1.0,
                    0.0,
                    0.5,
                    0.0,
                    0.0,
                    1.0,
                    0.5,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                ]
                events.append(
                    (900_000_000 + camera_index, camera.camera_info_topic, info)
                )
                for sequence, reference_time in enumerate(reference_times):
                    host_time = reference_time + camera_index * 1_000_000
                    source_time = (
                        10_000_000_000 + camera_index * 100 + sequence
                    )
                    color = Image()
                    color.header.stamp = stamp(source_time)
                    color.header.frame_id = camera.frame_id
                    color.height = 1
                    color.width = 1
                    color.encoding = "rgb8"
                    color.step = 3
                    color.data = [sequence, sequence + 1, sequence + 2]
                    events.append((host_time, camera.color_topic, color))

                    depth = Image()
                    depth.header.stamp = stamp(source_time)
                    depth.header.frame_id = camera.frame_id
                    depth.height = 1
                    depth.width = 1
                    depth.encoding = "16UC1"
                    depth.step = 4
                    depth_value = 100 + sequence
                    depth.data = [depth_value, 0, 0xEE, 0xEE]
                    events.append((host_time + 1, camera.depth_topic, depth))

                    timing = CameraFrameTiming()
                    timing.header.stamp = stamp(source_time)
                    timing.header.frame_id = camera.frame_id
                    timing.camera_name = camera.name
                    timing.camera_model = camera.model
                    timing.serial_number = camera.serial
                    timing.shared_ros_timestamp_ns = source_time
                    timing.group_host_realtime_ns = host_time
                    timing.group_host_monotonic_raw_ns = host_time - 100_000_000
                    timing.group_validity_flags = (
                        timing.GROUP_VALID_COMMON_STAMP
                        | timing.GROUP_VALID_IDENTITY
                        | timing.GROUP_VALID_CALLBACK_CLOCKS
                    )
                    events.append((host_time + 2, camera.timing_topic, timing))

            for tracker_index, tracker in enumerate(config.trackers):
                for sequence, host_time in enumerate(
                    (999_000_000, 1_001_000_000, 1_009_000_000, 1_013_000_000)
                ):
                    sample = TrackerSample()
                    sample.header.stamp = stamp(host_time)
                    sample.header.frame_id = config.world_frame
                    sample.role = tracker.role
                    sample.tracker_id = TRACKER_IDS[tracker.role]
                    sample.host_realtime_ns = host_time
                    sample.host_monotonic_ns = host_time - 200_000_000
                    sample.packet_index = sequence
                    sample.tracking_status = 2
                    sample.pose_valid = True
                    sample.pose.position.x = float(sequence + tracker_index)
                    sample.pose.orientation.w = 1.0
                    events.append(
                        (host_time + tracker_index + 10, tracker.sample_topic, sample)
                    )

            for sequence, host_time in enumerate(reference_times):
                glove = PoseStamped()
                glove.header.stamp = stamp(host_time)
                glove.header.frame_id = config.world_frame
                glove.pose.position.x = float(10 + sequence)
                glove.pose.orientation.w = 1.0
                events.append((host_time + 20, "/gloves/left/state", glove))

            for bag_time, topic, message in sorted(events, key=lambda item: item[0]):
                writer.write(topic, serialize_message(message), bag_time)
            writer.close()

            dataset = read_unified_bag(bag, config)
            extrinsics_directory = root / "extrinsics"
            extrinsics_directory.mkdir()
            for camera in config.cameras:
                tracker = config.tracker_by_role[camera.tracker_role]
                document = {
                    "schema_version": 1,
                    "status": "VALID",
                    "camera": {
                        "name": camera.name,
                        "model": camera.model,
                        "serial": camera.serial,
                        "frame_id": camera.frame_id,
                    },
                    "tracker": {
                        "role": camera.tracker_role,
                        "tracker_id": TRACKER_IDS[camera.tracker_role],
                        "frame_id": tracker.frame_id,
                    },
                    "transform": {
                        "semantics": "parent_from_child",
                        "parent_frame": tracker.frame_id,
                        "child_frame": camera.frame_id,
                        "translation_m": [0.0, 0.0, 0.0],
                        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                    "quality": {"grade": "PASS"},
                    "frames": {"world": config.world_frame, "board": "board"},
                    "provenance": {"tool": "synthetic-sdk-test"},
                }
                (extrinsics_directory / f"{camera.name}.yaml").write_text(
                    yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
                )
            extrinsics = load_extrinsics(
                extrinsics_directory, config, dataset.tracker_ids
            )
            result = align_dataset(dataset, config, extrinsics)
            self.assertTrue(result.accepted, result.rejection_reasons)
            output = export_result(
                output_directory=root / "aligned",
                config_path=config_path,
                config=config,
                dataset=dataset,
                extrinsics=extrinsics,
                result=result,
            )

            with AlignedDataset.open(output, bag) as aligned:
                random_frame = aligned.frame(1)
                sequential_frame = tuple(aligned.iter_frames())[1]

                np.testing.assert_array_equal(
                    random_frame.cameras["d405_1"].depth.array,
                    np.array([[101]], dtype=np.uint16),
                )
                np.testing.assert_array_equal(
                    random_frame.cameras["d405_1"].color.array,
                    [[[1, 2, 3]]],
                )
                self.assertEqual(
                    random_frame.trackers[
                        "torso"
                    ].world_from_tracker.as_matrix().shape,
                    (4, 4),
                )
                self.assertEqual(
                    random_frame.additional_streams[
                        "left_glove"
                    ].message.pose.position.x,
                    11.0,
                )
                self.assertEqual(aligned.camera_info["d436"].width, 1)
                np.testing.assert_array_equal(
                    random_frame.cameras["d405_1"].color.array,
                    sequential_frame.cameras["d405_1"].color.array,
                )


if __name__ == "__main__":
    unittest.main()
