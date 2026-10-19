"""ROS service adapter, with no ROS import until explicitly constructed."""
from concurrent.futures import Future


CONTEXT_FIELDS = ("controller_instance_id", "state_revision", "state", "session_id", "start_request_id",
                  "session_label", "output_path", "recorder_alive", "elapsed_sec", "planned_duration_sec",
                  "topics", "max_duration_sec", "detail")


def context_dict(message):
    value = {name: getattr(message, name) for name in CONTEXT_FIELDS}
    value["topics"] = list(value["topics"])
    return value


class ManagedRosAdapter:
    """Uses the manager node's executor; never spins inside another callback."""
    def __init__(self, node, *, snapshot_service="/capture/get_snapshot", command_service="/capture/managed_command"):
        from rclpy.qos import qos_profile_services_default
        from vt_camera_msgs.srv import GetCaptureSnapshot, ManagedCaptureCommand
        self._snapshot_type = GetCaptureSnapshot
        self._command_type = ManagedCaptureCommand
        self._snapshot_client = node.create_client(GetCaptureSnapshot, snapshot_service, qos_profile=qos_profile_services_default)
        self._command_client = node.create_client(ManagedCaptureCommand, command_service, qos_profile=qos_profile_services_default)

    @staticmethod
    def _call(client, request, convert):
        result = Future()
        if not client.service_is_ready():
            result.set_exception(ConnectionError("Managed recorder service is not ready"))
            return result
        try:
            pending = client.call_async(request)
        except Exception as exc:
            result.set_exception(exc)
            return result

        def complete(source):
            if result.done():
                return
            try:
                value = convert(source.result())
                if not result.done():
                    result.set_result(value)
            except Exception as exc:
                if not result.done():
                    result.set_exception(exc)

        def cancel_local(future):
            if future.cancelled():
                # Only abandon our waiter. The controller may still execute;
                # never interpret local cancellation as cancellation of START.
                client.remove_pending_request(pending)

        result.add_done_callback(cancel_local)
        pending.add_done_callback(complete)
        return result

    def get_snapshot(self):
        return self._call(self._snapshot_client, self._snapshot_type.Request(), lambda response: context_dict(response.context))

    def command(self, values):
        request = self._command_type.Request()
        for name, value in values.items():
            setattr(request, name, value)
        return self._call(self._command_client, request, lambda response: dict(
            accepted=response.accepted, code=response.code, detail=response.detail, context=context_dict(response.context)))
