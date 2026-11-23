"""Synthetic six-stream hand data -> actual recorder -> independent MCAP read.

No devices or calibration commands. A private ROS domain and temporary trial
directory isolate this test from running workbench sources. Camera/Tracker
allowlist entries are idle; the verified recording target is the six hand topics.
"""
from collections import Counter, defaultdict
from copy import deepcopy
import os
from pathlib import Path
import time

import pytest


def test_both_hands_raw_pose_and_tactile_survive_one_monitored_recording(tmp_path, monkeypatch):
    rclpy = pytest.importorskip("rclpy")
    rosbag = pytest.importorskip("rosbag2_py")
    from ament_index_python.packages import get_package_prefix
    executable = Path(get_package_prefix("vt_realsense_capture")) / "lib/vt_realsense_capture/monitored_recorder"
    if not executable.is_file():
        pytest.skip("Build monitored_recorder before running DDS integration")
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.serialization import deserialize_message
    from vt_glove_msgs.msg import GloveState, RawImuBatch, RawImuSample
    from vt_tactile_msgs.msg import TactileRow44, TactileRow44Array
    from vt_capture_core.capture_plan import resolve_capture_plan
    from vt_capture_core.recipes import Recipe
    from vt_capture_manager.quality_runner import QualityRunner
    from vt_realsense_capture.controller import RecorderStartSpec
    from vt_realsense_capture.controller_node import ManagedRecorder
    from vt_realsense_capture.recorder import required_topics, write_qos_overrides

    domain = 183 + os.getpid() % 16
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node("synthetic_dual_hand_recording", context=context, enable_rosout=False)
    plan = resolve_capture_plan(Recipe("Six hand streams", cameras=(), trackers=(),
        pose_hands=("left", "right"), tactile_hands=("left", "right"), version=3), admission_policy="formal")
    classes = {"imu/raw": RawImuBatch, "state": GloveState, "tactile/samples": TactileRow44Array}
    topic_classes = {f"/gloves/{hand}/{suffix}": cls for hand in ("left", "right") for suffix, cls in classes.items()}
    publishers = {topic: node.create_publisher(cls, topic, 100) for topic, cls in topic_classes.items()}
    topics = required_topics(("d405_1", "d405_2", "d436"), plan.record_topics())
    directory = tmp_path / "synthetic-dual-hand"
    directory.mkdir()
    qos = write_qos_overrides(directory / "qos.yaml", topics, topic_types=plan.topic_types)
    recorder = ManagedRecorder(Path(__file__).parents[1] / "config/mcap_writer_options.yaml")
    spec = RecorderStartSpec(directory.name, directory, directory / "bag", qos, topics)
    expected = defaultdict(list)
    frames = 24

    def wait_for(predicate):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(.025)
        log = (directory / "recorder.log").read_text(errors="replace")
        pytest.fail(f"Six-stream recorder condition timed out: {log}")

    def header(message, hand, realtime_ns):
        message.header.frame_id = f"glove_{hand}_reference"
        message.header.stamp.sec = realtime_ns // 1_000_000_000
        message.header.stamp.nanosec = realtime_ns % 1_000_000_000

    def publish(hand, index):
        mono, real = time.monotonic_ns(), time.time_ns()
        node_ids = [9, 11] if hand == "left" else [1, 12]
        raw = RawImuBatch(schema_version=1, hand=hand, source_mode="live", session_id=f"{hand}-source",
            config_sha256=("a" if hand == "left" else "b") * 64, batch_sequence=index,
            clock_domain="CLOCK_MONOTONIC", timing_source="host_receive")
        header(raw, hand, real)
        raw.samples = [RawImuSample(node_id=node_id, sample_index=index, filter_epoch=0,
            host_realtime_ns=real, host_monotonic_ns=mono, time_valid=True,
            raw_values=[-32768, index, node_id, 0, 100, 32767], status=255,
            status_known=True, fusion_updated=True) for node_id in node_ids]
        pose = GloveState(schema_version=1, hand=hand, source_mode="live", session_id=raw.session_id,
            frame_index=index, calibration_version=2, config_sha256=raw.config_sha256,
            model_rest_sha256="c" * 64, algorithm="synthetic-recording-test", quality="valid",
            pose_valid=True, time_valid=True, host_realtime_ns=real, host_monotonic_ns=mono,
            reference_node_id=node_ids[-1], solve_completed_monotonic_ns=mono + 1,
            clock_domain="CLOCK_MONOTONIC", timing_source="host_receive",
            source_node_ids=node_ids, source_realtime_ns=[real, real], source_monotonic_ns=[mono, mono],
            source_sample_indices=[index, index], source_filter_epochs=[0, 0],
            mano_pose_rotvec=[index * .001] * 48, root_quat_xyzw=[0., 0., 0., 1.],
            shape_params=[0.] * 10, joint_positions_m=[0.] * 63)
        header(pose, hand, real)
        tactile = TactileRow44Array(schema_version=2, hand=hand, session_id=f"{hand}-tactile",
            stream_epoch=1, sequence=index, first_packet_index=(index - 1) * 20 + 1,
            host_realtime_ns=real, host_monotonic_ns=mono, clock_domain="CLOCK_MONOTONIC",
            timing_source="host_receive", connected=True, time_valid=True,
            blocks=[TactileRow44(row=row, adc=[index * 100 + row * 20 + column for column in range(20)])
                    for row in range(20)])
        header(tactile, hand, real)
        for suffix, message in (("imu/raw", raw), ("state", pose), ("tactile/samples", tactile)):
            topic = f"/gloves/{hand}/{suffix}"
            expected[topic].append(message)
            publishers[topic].publish(message)

    try:
        recorder.start(spec)
        wait_for(lambda: all(publisher.get_subscription_count() for publisher in publishers.values()))
        for index in range(1, frames + 1):
            for hand in ("left", "right"):
                publish(hand, index)
            time.sleep(.01)

        def storage_received_all():
            snapshot = recorder.metrics()
            rows = {row["topic"]: row for row in snapshot.get("topics", [])} if snapshot else {}
            return snapshot if all(rows.get(topic, {}).get("storage_returned_messages") == frames
                                   for topic in publishers) else None

        running = wait_for(storage_received_all)
        assert running["closed"] is False
        stopped = recorder.stop()
        assert stopped.termination_confirmed and stopped.returncode == 0
        metrics = recorder.metrics()
        assert metrics["available"] and metrics["closed"]

        reader = rosbag.SequentialReader()
        reader.open(rosbag.StorageOptions(uri=str(directory / "bag"), storage_id="mcap"),
                    rosbag.ConverterOptions("", ""))
        counts = Counter()
        while reader.has_next():
            topic, payload, _receipt = reader.read_next()
            assert topic in expected
            decoded = deserialize_message(payload, topic_classes[topic])
            # ROS message equality checks every field, including int16 extremes,
            # source clocks, node/pose indices, identities and tactile ADC rows.
            assert decoded == expected[topic][counts[topic]]
            counts[topic] += 1
        reader.close()
        assert counts == {topic: frames for topic in publishers}
        rows = {row["topic"]: row for row in metrics["topics"]}
        for topic, count in counts.items():
            assert count == rows[topic]["writer_received_messages"] == rows[topic]["storage_returned_messages"]

        hand_expectations = {}
        for hand in ("left", "right"):
            edge = dict(source_mode="live", session_id=f"{hand}-source",
                        config_sha256=("a" if hand == "left" else "b") * 64, profile_version=2, rest_sha256="c" * 64)
            hand_expectations[hand] = dict(start=dict(edge), finish=dict(edge))
        report = QualityRunner(timeout_s=20).run(directory / "bag", directory / "quality.json",
            expected_topics=list(plan.required_topics()), require_recorder_counts=True,
            hand_expectations=hand_expectations)
        assert report["recorder_reconciliation"]["status"] == "verified"
        assert report["runner_provenance"]["optional_topics"] == []
        assert report["status"] == "ok", report["warnings"]
        for hand in ("left", "right"):
            raw = report["topics"][f"/gloves/{hand}/imu/raw"]
            assert raw["inspection"] == "raw_imu_samples" and raw["recorded_samples"] == frames * 2
            assert raw["malformed_messages"] == raw["unobserved_batches"] == 0
            assert all(node["samples"] == frames and node["unobserved_sample_indices"] == 0
                       for node in raw["nodes"].values())
            pose = report["topics"][f"/gloves/{hand}/state"]
            assert pose["invalid_pose_messages"] == pose["invalid_time_messages"] == 0
            tactile = report["topics"][f"/gloves/{hand}/tactile/samples"]
            assert tactile["tactile"]["observed_at_least_once_cells"] == 400
        wrong_profile = deepcopy(hand_expectations)
        for edge in wrong_profile["left"].values():
            edge["profile_version"] = 99
        mismatch = QualityRunner(timeout_s=20).run(directory / "bag", directory / "quality-wrong-profile.json",
            expected_topics=list(plan.required_topics()), require_recorder_counts=True,
            hand_expectations=wrong_profile)
        assert mismatch["recorder_reconciliation"]["status"] == "verified"
        assert mismatch["status"] == "warnings"
        assert [(warning["code"], warning.get("topic")) for warning in mismatch["warnings"]] == [
            ("recorded_hand_identity_mismatch", "/gloves/left/state")]
    finally:
        recorder.stop()
        node.destroy_node()
        context.shutdown()
