"""Small synthetic DDS -> normal ManagedRecorder -> MCAP integration, no devices."""
import json
import os
from pathlib import Path
import signal
import time

import pytest


@pytest.mark.parametrize("clean_stop", [True, False])
def test_managed_recorder_storage_counts_and_close_evidence(tmp_path, monkeypatch, clean_stop):
    rclpy = pytest.importorskip("rclpy")
    rosbag = pytest.importorskip("rosbag2_py")
    from ament_index_python.packages import get_package_prefix
    if not (Path(get_package_prefix("vt_realsense_capture")) /
            "lib/vt_realsense_capture/monitored_recorder").is_file():
        pytest.skip("Build monitored_recorder before running DDS integration")
    from rclpy.context import Context
    from rclpy.node import Node
    from std_msgs.msg import String
    from vt_realsense_capture.controller import RecorderStartSpec
    from vt_realsense_capture.controller_node import ManagedRecorder
    from vt_realsense_capture.recorder import required_topics, write_qos_overrides

    domain = 210 + os.getpid() % 20
    monkeypatch.setenv("ROS_DOMAIN_ID", str(domain))
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node("writer_metrics_test", context=context, enable_rosout=False)
    publisher = node.create_publisher(String, "/test/observed", 100)
    topics = (*required_topics(("d405_1", "d405_2", "d436")), "/test/observed")
    qos = write_qos_overrides(tmp_path / "qos.yaml", topics,
                             topic_types={"/test/observed": "std_msgs/msg/String"})
    recorder = ManagedRecorder(Path(__file__).parents[1] / "config/mcap_writer_options.yaml")
    spec = RecorderStartSpec("test-session", tmp_path, tmp_path / "bag", qos, topics)

    def wait_for(predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            time.sleep(.05)
        log = (tmp_path / "recorder.log").read_text(errors="replace")
        pytest.fail(f"Recorder did not reach expected observation: {log}")

    try:
        recorder.start(spec)
        wait_for(lambda: publisher.get_subscription_count())
        for i in range(40):
            publisher.publish(String(data=f"sample-{i}"))
            time.sleep(.01)
        def observed():
            data = recorder.metrics()
            if data and data.get("available"):
                topic = next(t for t in data["topics"] if t["topic"] == "/test/observed")
                if topic["storage_returned_messages"] == 40:
                    return data
        running = wait_for(observed)
        assert running["closed"] is False
        if not clean_stop:
            # Only the owned synthetic recorder's process group is killed.
            os.killpg(recorder._process.pid, signal.SIGKILL)
        stopped = recorder.stop()
        snapshot = json.loads((tmp_path / "recorder-metrics.json").read_text())
        assert snapshot["closed"] is clean_stop
        assert stopped.termination_confirmed is True
        if not clean_stop:
            assert stopped.returncode != 0
            return
        assert stopped.returncode == 0
        assert snapshot["session_id"] == "test-session"
        assert snapshot["pid"] > 0
        final = recorder.metrics()
        assert final["available"] and final["closed"]
        reader = rosbag.SequentialReader()
        reader.open(rosbag.StorageOptions(uri=str(tmp_path / "bag"), storage_id="mcap"),
                    rosbag.ConverterOptions("", ""))
        count = size = 0
        while reader.has_next():
            topic, data, _ = reader.read_next()
            assert topic == "/test/observed"
            count += 1
            size += len(data)
        row = next(t for t in final["topics"] if t["topic"] == "/test/observed")
        assert count == row["writer_received_messages"] == row["storage_returned_messages"] == 40
        assert size == row["writer_received_bytes"] == row["storage_returned_bytes"]
    finally:
        recorder.stop()
        node.destroy_node()
        context.shutdown()
