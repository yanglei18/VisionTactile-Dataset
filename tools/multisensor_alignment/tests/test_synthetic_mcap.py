from pathlib import Path
import tempfile
import unittest

import yaml

from vt_multisensor_alignment.aligner import align_dataset
from vt_multisensor_alignment.bag_reader import configured_topic_types, read_unified_bag
from vt_multisensor_alignment.config import load_config
from vt_multisensor_alignment.export import export_result, validate_output
from vt_multisensor_alignment.extrinsics import load_extrinsics


ROOT = Path(__file__).resolve().parents[1]
TRACKER_IDS = {
    "left_wrist": "1" * 64,
    "right_wrist": "2" * 64,
    "torso": "3" * 64,
}


class SyntheticMcapTests(unittest.TestCase):
    def test_real_ros_serialization_round_trip_contains_unified_core(self) -> None:
        try:
            import rosbag2_py
            from builtin_interfaces.msg import Time
            from rclpy.serialization import serialize_message
            from sensor_msgs.msg import CameraInfo, Image
            from vt_camera_msgs.msg import CameraFrameTiming
            from vt_tracker_msgs.msg import TrackerSample
        except ImportError as error:
            self.skipTest(f"ROS 2 workspace is not sourced: {error}")

        config = load_config(ROOT / "config" / "alignment.example.yaml")
        with tempfile.TemporaryDirectory() as temporary:
            bag = Path(temporary) / "unified"
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
            events = []

            def stamp(value_ns: int) -> Time:
                return Time(
                    sec=value_ns // 1_000_000_000,
                    nanosec=value_ns % 1_000_000_000,
                )

            for camera_index, camera in enumerate(config.cameras):
                info = CameraInfo()
                info.header.frame_id = camera.frame_id
                info.width = 2
                info.height = 2
                info.k = [1.0, 0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 0.0, 1.0]
                events.append((900_000_000 + camera_index, camera.camera_info_topic, info))
                offset = camera_index * 1_000_000
                for sequence, reference_time in enumerate(
                    (1_000_000_000, 1_010_000_000)
                ):
                    host_time = reference_time + offset
                    source_time = 10_000_000_000 + camera_index * 100 + sequence
                    for topic in (camera.color_topic, camera.depth_topic):
                        image = Image()
                        image.header.stamp = stamp(source_time)
                        image.header.frame_id = camera.frame_id
                        image.height = 1
                        image.width = 1
                        image.encoding = "mono8"
                        image.step = 1
                        image.data = [sequence]
                        events.append((host_time, topic, image))
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
                    events.append((host_time + 1, camera.timing_topic, timing))
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
                    sample.pose.position.x = float(sequence)
                    sample.pose.orientation.w = 1.0
                    events.append(
                        (host_time + tracker_index + 10, tracker.sample_topic, sample)
                    )
            for bag_time, topic, message in sorted(events, key=lambda item: item[0]):
                writer.write(topic, serialize_message(message), bag_time)
            writer.close()

            dataset = read_unified_bag(bag, config)

            self.assertEqual(len(dataset.topic_types), 15)
            self.assertEqual(
                {name: len(frames) for name, frames in dataset.camera_frames.items()},
                {"d405_1": 2, "d405_2": 2, "d436": 2},
            )
            self.assertEqual(
                {role: len(poses) for role, poses in dataset.tracker_poses.items()},
                {"left_wrist": 4, "right_wrist": 4, "torso": 4},
            )
            self.assertEqual(
                {
                    name: len(values)
                    for name, values in dataset.clock_observations.items()
                },
                {
                    "camera:d405_1": 2,
                    "camera:d405_2": 2,
                    "camera:d436": 2,
                    "tracker:left_wrist": 4,
                    "tracker:right_wrist": 4,
                    "tracker:torso": 4,
                },
            )
            self.assertEqual(dataset.incomplete_camera_groups, {
                "d405_1": 0,
                "d405_2": 0,
                "d436": 0,
            })
            extrinsics_directory = Path(temporary) / "extrinsics"
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
                    "provenance": {"tool": "synthetic-test"},
                }
                (extrinsics_directory / f"{camera.name}.yaml").write_text(
                    yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
                )
            extrinsics = load_extrinsics(
                extrinsics_directory, config, dataset.tracker_ids
            )
            result = align_dataset(dataset, config, extrinsics)
            output = export_result(
                output_directory=Path(temporary) / "aligned",
                config_path=ROOT / "config" / "alignment.example.yaml",
                config=config,
                dataset=dataset,
                extrinsics=extrinsics,
                result=result,
            )

            self.assertTrue(result.accepted, result.rejection_reasons)
            self.assertEqual(validate_output(output)["aligned_frame_count"], 2)


if __name__ == "__main__":
    unittest.main()
