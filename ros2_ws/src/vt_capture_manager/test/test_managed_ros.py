"""Real managed-service DDS transport with entirely synthetic recorder services."""
import os
import time
import uuid

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("VT_CAPTURE_MANAGER_ROS_TEST") != "1",
                              reason="opt-in synthetic ROS test requires an isolated ROS domain")


def test_async_adapter_service_intent_and_reconcile_without_recorder(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    try:
        from vt_camera_msgs.srv import GetCaptureSnapshot, ManagedCaptureCommand
    except ImportError:
        pytest.skip("managed interfaces not built")
    from vt_capture_manager.config import ManagerConfig
    from vt_capture_manager.model import SnapshotModel
    from vt_capture_manager.management import ManagementService
    from vt_capture_manager.recorder_adapter import ManagedRosAdapter, CONTEXT_FIELDS

    context = Context()
    rclpy.init(context=context)
    client_node = Node("manager_synthetic_client", context=context, enable_rosout=False, start_parameter_services=False)
    server_node = Node("manager_fake_recorder_services", context=context, enable_rosout=False, start_parameter_services=False)
    prefix = "/managed_fake_"+uuid.uuid4().hex
    current = dict(controller_instance_id="fake-instance", state_revision=1, state=0, session_id="", start_request_id="",
                   session_label="", output_path="", recorder_alive=False, elapsed_sec=0., planned_duration_sec=0,
                   topics=["/synthetic/only"], max_duration_sec=300, detail="synthetic idle")
    calls = []
    service = None

    def fill(response):
        for field in CONTEXT_FIELDS:
            setattr(response.context, field, current[field])
        return response

    def snapshot_callback(request, response):
        return fill(response)

    def command_callback(request, response):
        assert service.store.operation(request.request_id)["phase"] == "DISPATCHED"
        calls.append(request.command)
        current["state_revision"] += 1
        if request.command == 1:
            current.update(state=3, session_id="synthetic-owned", start_request_id=request.request_id,
                           session_label=request.session_label, output_path="/does/not/create/a/bag",
                           recorder_alive=True, planned_duration_sec=request.planned_duration_sec, detail="synthetic recording")
        else:
            current.update(state=5, recorder_alive=False, elapsed_sec=2., detail="synthetic complete")
        response.accepted, response.code, response.detail = True, "accepted", "synthetic service only"
        return fill(response)

    server_node.create_service(GetCaptureSnapshot, prefix+"/snapshot", snapshot_callback)
    server_node.create_service(ManagedCaptureCommand, prefix+"/command", command_callback)
    adapter = ManagedRosAdapter(client_node, snapshot_service=prefix+"/snapshot", command_service=prefix+"/command")
    service = ManagementService(SnapshotModel(ManagerConfig((), quality_enabled=False)), adapter, enabled=True, state_dir=tmp_path/"state",
                                timeout_s=1., poll_interval_s=.02).start()
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    executor.add_node(server_node)

    def until(predicate):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail("synthetic managed service did not converge")

    def request(identifier, command, **params):
        return dict(version=2, request_id=identifier, command=command, params=params)

    try:
        until(lambda: service.snapshot(2)["capabilities"]["record"])
        result = service.dispatch(request("batch", "batch.create", label="fake ROS batch"))
        batch = result["snapshot"]["management"]["batches"][0]["id"]
        result = service.dispatch(request("start", "capture.start", batch_id=batch, label="fake ROS trial", duration_s=10,
                                           expected_controller_instance_id="fake-instance", expected_state_revision=1))
        trial_id = result["operation"]["trial_id"]
        assert result["operation"]["state"] == "accepted" and not calls
        until(lambda: service.store.trial(trial_id)["state"] == "RECORDING")
        result = service.dispatch(request("stop", "capture.stop", trial_id=trial_id,
                                           expected_controller_instance_id="fake-instance", expected_session_id="synthetic-owned"))
        assert result["ok"]
        until(lambda: service.store.trial(trial_id)["state"] == "COMPLETE")
        assert calls == [1, 2]
        assert service.store.trial(trial_id)["output_path"] == "/does/not/create/a/bag"
    finally:
        service.close()
        assert calls in ([], [1], [1, 2])  # Closing never creates an extra STOP.
        executor.remove_node(client_node)
        executor.remove_node(server_node)
        client_node.destroy_node()
        server_node.destroy_node()
        executor.shutdown()
        context.shutdown()
