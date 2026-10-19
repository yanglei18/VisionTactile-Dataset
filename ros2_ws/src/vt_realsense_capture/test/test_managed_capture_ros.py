"""Real ROS service transport, isolated DDS domain; only fake Recorder is used."""

from pathlib import Path
import threading
import time

import pytest


@pytest.fixture(autouse=True)
def ample_disk_space(monkeypatch):
    from vt_realsense_capture.storage import FileStorage
    monkeypatch.setattr(FileStorage, "disk_free_bytes", lambda self, path: 1024**4)


def test_managed_services_snapshot_retry_bound_stop_and_legacy_share_worker(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    try:
        from vt_camera_msgs.msg import CaptureCommand
        from vt_camera_msgs.srv import GetCaptureSnapshot, ManagedCaptureCommand
    except ImportError:
        pytest.skip("Build the new vt_camera_msgs managed services first")
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from vt_realsense_capture.config import load_config
    from vt_realsense_capture.controller import RecorderHealth, RecorderStopFact
    from vt_realsense_capture.controller_node import build_controller_node

    calls = []
    class FakeRecorder:
        def start(self, spec):
            assert (spec.session_dir / "controller_session.json").exists()
            calls.append(("start", threading.get_ident()))
        def health(self):
            return RecorderHealth.ALIVE
        def stop(self):
            calls.append(("stop", threading.get_ident()))
            return RecorderStopFact(0, "fake", True, False)

    context = Context()
    rclpy.init(context=context, domain_id=191)
    config = load_config(Path(__file__).parents[1] / "config/cameras.yaml")
    node = build_controller_node(context=context, config=config, output_root=tmp_path,
        recorder_factory=FakeRecorder, session_id_factory=lambda: "ros-managed-test")
    client = Node("managed_capture_test_client", context=context, enable_rosout=False)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    executor.add_node(client)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()

    def call(service, request):
        assert service.wait_for_service(timeout_sec=3)
        future = service.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        assert done.wait(5)
        return future.result()

    try:
        snapshot = client.create_client(GetCaptureSnapshot, "/capture/get_snapshot")
        managed = client.create_client(ManagedCaptureCommand, "/capture/managed_command")
        initial = call(snapshot, GetCaptureSnapshot.Request()).context
        request = ManagedCaptureCommand.Request(request_id="start", command=1,
            expected_controller_instance_id=initial.controller_instance_id,
            expected_session_id=initial.session_id, expected_state_revision=initial.state_revision,
            session_label="DDS trial", planned_duration_sec=30)
        first = call(managed, request)
        assert first.accepted and first.context.state == 3 and first.context.start_request_id == "start"
        duplicate = call(managed, request)
        assert duplicate == first
        wrong = ManagedCaptureCommand.Request(request_id="wrong-stop", command=2,
            expected_controller_instance_id=initial.controller_instance_id,
            expected_session_id="wrong-session", expected_state_revision=first.context.state_revision)
        assert call(managed, wrong).code == "session_mismatch"
        # Compatibility topic STOP is processed by the very same worker.
        publisher = client.create_publisher(CaptureCommand, "/capture/command", 10)
        deadline = time.monotonic() + 3
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(.01)
        publisher.publish(CaptureCommand(request_id="legacy-stop", command=CaptureCommand.STOP))
        complete = first.context
        deadline = time.monotonic() + 3
        while complete.state != 5 and time.monotonic() < deadline:
            complete = call(snapshot, GetCaptureSnapshot.Request()).context
            time.sleep(.01)
        assert complete.state == 5 and complete.start_request_id == "start"
        assert complete.session_label == "DDS trial" and complete.output_path.endswith("/bag")
        assert [kind for kind, _ in calls] == ["start", "stop"]
        assert len({identity for _, identity in calls}) == 1
    finally:
        executor.shutdown(timeout_sec=3)
        worker.join(3)
        node.close()
        client.destroy_node()
        node.destroy_node()
        context.shutdown()


def test_socket_v2_to_manager_to_real_controller_complete_ledger_and_manifest(tmp_path, monkeypatch):
    """Full control path: Unix socket -> durable manager -> DDS -> real controller.

    Only Recorder is fake. This deliberately creates no bag or device node.
    Manager imports are optional for the standalone capture package test suite.
    """
    import itertools
    import json
    import socket
    import sqlite3

    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("vt_capture_manager.management")
    try:
        from vt_camera_msgs.srv import ManagedCaptureCommand  # noqa: F401
    except ImportError:
        pytest.skip("Build the new vt_camera_msgs managed services first")
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from vt_capture_core import MAX_MESSAGE_BYTES, decode_message, encode_message
    from vt_capture_manager.config import ManagerConfig
    from vt_capture_manager.management import ManagementService
    from vt_capture_manager.model import SnapshotModel
    from vt_capture_manager.recorder_adapter import ManagedRosAdapter
    from vt_capture_manager.server import SnapshotServer
    from vt_realsense_capture.config import load_config
    from vt_realsense_capture.controller import RecorderHealth, RecorderStopFact
    from vt_realsense_capture.controller_node import build_controller_node

    def forbidden_real_recorder(*_args, **_kwargs):
        pytest.fail("This end-to-end test must never start a real Recorder process")

    monkeypatch.setattr("vt_realsense_capture.controller_node.ManagedRecorder.start", forbidden_real_recorder)
    calls = []
    management = None

    class FakeRecorder:
        def start(self, spec):
            # Both independent durable intents must precede the process boundary.
            assert management.store.operation("e2e-start")["phase"] == "DISPATCHED"
            intent = json.loads((spec.session_dir / "controller_session.json").read_text())
            assert intent["lifecycle"] == "start_intent" and intent["start_request_id"] == "e2e-start"
            calls.append(("start", threading.get_ident()))

        def health(self):
            return RecorderHealth.ALIVE

        def stop(self):
            assert management.store.operation("e2e-stop")["phase"] == "DISPATCHED"
            calls.append(("stop", threading.get_ident()))
            return RecorderStopFact(0, "fake Recorder terminated", True, False)

    output_root = tmp_path / "captures"
    output_root.mkdir()
    context = Context()
    rclpy.init(context=context, domain_id=192)
    config = load_config(Path(__file__).parents[1] / "config/cameras.yaml")
    controller = build_controller_node(context=context, config=config, output_root=output_root,
        recorder_factory=FakeRecorder, session_id_factory=lambda: "full-stack-fake-session")
    manager_node = Node("full_stack_manager_adapter_test", context=context, enable_rosout=False,
                        start_parameter_services=False)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(controller)
    executor.add_node(manager_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    # This transport/ledger test deliberately has no bag; quality subprocesses
    # are covered separately with closed metadata fixtures.
    model = SnapshotModel(ManagerConfig((), disk_path=str(output_root), quality_enabled=False))
    management = ManagementService(model, ManagedRosAdapter(manager_node), enabled=True,
        state_dir=tmp_path / "manager-state", timeout_s=2., poll_interval_s=.02).start()
    server = SnapshotServer(model, tmp_path / "ipc/manager.sock", management=management).start()
    request_ids = itertools.count()

    def wire(identifier, command, **params):
        request = dict(version=2, request_id=identifier, command=command, params=params)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(3)
            client.connect(str(server.path))
            client.sendall(encode_message(request))
            data = bytearray()
            while not data.endswith(b"\n"):
                chunk = client.recv(65536)
                assert chunk, "manager socket closed before a complete reply"
                data.extend(chunk)
                assert len(data) <= MAX_MESSAGE_BYTES
        response = decode_message(bytes(data))
        assert response["version"] == 2 and response["request_id"] == identifier
        return response

    def until_snapshot(predicate):
        deadline = time.monotonic() + 8
        last = None
        while time.monotonic() < deadline:
            last = wire(f"snapshot-{next(request_ids)}", "snapshot")["snapshot"]
            if predicate(last):
                return last
            time.sleep(.01)
        pytest.fail(f"End-to-end state did not converge: {last}")

    try:
        initial = until_snapshot(lambda value: value["capabilities"]["record"])
        assert initial["management"]["controller"]["state"] == 0
        batch_response = wire("e2e-batch", "batch.create", label="Full-stack fake batch")
        assert batch_response["ok"] and batch_response["operation"]["code"] == "batch_created"
        batch_id = batch_response["snapshot"]["management"]["batches"][0]["id"]
        current = batch_response["snapshot"]["management"]["controller"]
        start_params = dict(batch_id=batch_id, label="Full-stack fake trial", duration_s=30,
                            expected_controller_instance_id=current["controller_instance_id"],
                            expected_state_revision=current["state_revision"])
        start_response = wire("e2e-start", "capture.start", **start_params)
        assert start_response["ok"]
        trial_id = start_response["operation"]["trial_id"]
        active = until_snapshot(lambda value: any(
            trial["id"] == trial_id and trial["state"] == "RECORDING"
            for trial in value["management"]["trials"]))
        assert active["recording"]["state"] == "RECORDING"
        assert active["management"]["controller"]["start_request_id"] == "e2e-start"
        assert active["management"]["controller"]["planned_duration_sec"] == 30
        assert wire("e2e-start", "capture.start", **start_params)["ok"]
        assert len(calls) == 1  # Network-level replay did not spawn another Recorder.
        stop_response = wire("e2e-stop", "capture.stop", trial_id=trial_id,
            expected_controller_instance_id=current["controller_instance_id"],
            expected_session_id=active["management"]["controller"]["session_id"])
        assert stop_response["ok"]
        complete = until_snapshot(lambda value: any(
            trial["id"] == trial_id and trial["state"] == "COMPLETE"
            for trial in value["management"]["trials"]))
        assert complete["recording"]["state"] == "COMPLETE"
        trial = next(row for row in complete["management"]["trials"] if row["id"] == trial_id)
        assert trial["quality"] == "NOT_ASSESSED"
        expected_path = str(output_root / "full-stack-fake-session/bag")
        assert trial["output_path"] == expected_path
        with sqlite3.connect(management.store.directory / "capture.sqlite3") as database:
            row = database.execute("SELECT state,session_id,start_request_id,output_path,quality FROM trials WHERE id=?", (trial_id,)).fetchone()
            assert row == ("COMPLETE", "full-stack-fake-session", "e2e-start", expected_path, "NOT_ASSESSED")
            operations = database.execute("SELECT request_id,state,phase FROM operations ORDER BY request_id").fetchall()
            assert operations == [("e2e-batch", "accepted", "TERMINAL"),
                                  ("e2e-start", "accepted", "TERMINAL"),
                                  ("e2e-stop", "accepted", "TERMINAL")]
        manifest_path = management.store.manifests / (trial_id + ".json")
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            manifest = json.loads(manifest_path.read_text())
            if manifest["state"] == "COMPLETE":
                break
            time.sleep(.01)
        assert manifest["state"] == "COMPLETE" and manifest["start_request_id"] == "e2e-start"
        assert manifest["output_path"] == expected_path and manifest["quality"] == "NOT_ASSESSED"
        controller_journal = json.loads((output_root / "full-stack-fake-session/controller_session.json").read_text())
        assert controller_journal["lifecycle"] == "complete"
        assert controller_journal["start_request_id"] == "e2e-start"
        assert not Path(expected_path).exists(), "Fake Recorder must not create any bag"
        assert [kind for kind, _ in calls] == ["start", "stop"]
        assert len({thread for _, thread in calls}) == 1
    finally:
        server.close()
        management.close()
        executor.shutdown(timeout_sec=3)
        ros_thread.join(3)
        controller.close()
        manager_node.destroy_node()
        controller.destroy_node()
        context.shutdown()
