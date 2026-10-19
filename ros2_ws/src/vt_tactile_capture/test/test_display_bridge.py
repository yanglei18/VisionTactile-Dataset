import ast
from copy import deepcopy
import importlib.util
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from vt_tactile_msgs.msg import TactileState

from vt_tactile_capture.display_bridge import TactileDisplayBridge
from vt_tactile_capture.display_transport import LocalTactileReceiver


class FakeSender:
    def __init__(self, path):
        self.path = path
        self.messages = []
        self.sent = self.dropped = self.rejected = 0
        self.closed = False

    def send(self, message):
        self.messages.append(message)
        self.sent += 1
        return True

    def close(self):
        self.closed = True


@pytest.fixture
def context():
    value = Context()
    rclpy.init(context=value, domain_id=193, args=[])
    yield value
    if value.ok():
        value.shutdown()


def test_bridge_is_best_effort_depth_one_read_only(context):
    node = TactileDisplayBridge(context=context, sender_factory=FakeSender)
    try:
        assert node.subscription.topic_name == "/gloves/right/tactile/state"
        assert node.subscription.qos_profile.depth == 1
        assert node.subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert node.subscription.qos_profile.durability == DurabilityPolicy.VOLATILE
        assert not hasattr(node, "source")
        assert not hasattr(node, "sample_publisher")
        message = SimpleNamespace(hand="right", clock_domain="CLOCK_MONOTONIC", timing_source="host_receive")
        node.on_state(message)
        assert node.sender.messages == [message]
        assert node.received == 1
        # Only Node infrastructure publishers exist; no posture/tactile publication.
        assert all(pub.topic_name in ("/rosout", "/parameter_events") for pub in node.publishers)
    finally:
        sender = node.sender
        node.destroy_node()
    assert sender.closed


@pytest.mark.parametrize("changes", [
    {"hand": "left"}, {"clock_domain": "CLOCK_MONOTONIC_RAW"}, {"timing_source": "replay"},
])
def test_wrong_hand_or_clock_is_not_forwarded(context, changes):
    node = TactileDisplayBridge(context=context, sender_factory=FakeSender)
    try:
        values = dict(hand="right", clock_domain="CLOCK_MONOTONIC", timing_source="host_receive")
        values.update(changes)
        node.on_state(SimpleNamespace(**values))
        assert not node.sender.messages
        assert node.rejected == 1
    finally:
        node.destroy_node()


@pytest.mark.parametrize("name,value", [("use_sim_time", True), ("hand", "bad"), ("topic", "relative")])
def test_invalid_startup_rejected_before_sender(context, name, value):
    calls = []

    def factory(path):
        calls.append(path)
        return FakeSender(path)

    with pytest.raises(ValueError):
        TactileDisplayBridge(context=context, sender_factory=factory,
                             parameter_overrides=[Parameter(name, value=value)])
    assert not calls


def test_live_parameters_are_startup_only(context):
    node = TactileDisplayBridge(context=context, sender_factory=FakeSender)
    try:
        for name, value in (("use_sim_time", True), ("hand", "left"), ("topic", "/other"), ("socket_path", "/other.sock")):
            assert not node.set_parameters([Parameter(name, value=value)])[0].successful
        assert node.set_parameters([Parameter("hand", value="right")])[0].successful
    finally:
        node.destroy_node()


@pytest.mark.parametrize("schema,side", [(1, 80), (2, 20)])
def test_actual_synthetic_ros_subscription_to_local_ipc(context, schema, side):
    topic = "/synthetic_tactile_display_" + uuid.uuid4().hex
    with tempfile.TemporaryDirectory(prefix="vtt-ros-") as directory:
        path = str(Path(directory) / "right.sock")
        receiver = LocalTactileReceiver(path)
        node = TactileDisplayBridge(context=context, parameter_overrides=[
            Parameter("socket_path", value=path), Parameter("topic", value=topic),
        ])
        publisher_node = Node("synthetic_tactile_display_source", context=context)
        publisher = publisher_node.create_publisher(TactileState, topic, QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.add_node(publisher_node)
        message = TactileState(schema_version=schema, rows=side, cols=side, stream_epoch=1, hand="right", session_id="synthetic",
            source_sequence=42, host_realtime_ns=1_700_000_000_123_456_789, host_monotonic_ns=123_456_789_123,
            snapshot_monotonic_ns=123_456_799_123, max_age_ns=100_000_000, connected=True, time_valid=True,
            clock_domain="CLOCK_MONOTONIC", timing_source="host_receive", unit="adc_counts", reason="synthetic",
            adc=[12000] * (side * side), valid_mask=[True] * (side * side), updated_mask=[True] * (side * side),
            block_monotonic_ns=[123_456_789_123] * (side * 8),
            block_realtime_ns=[1_700_000_000_123_456_789] * (side * 8))
        message.adc[0] = 0
        message.adc[2] = 65535
        message.valid_mask[1] = False
        message.updated_mask[3] = False
        message.header.frame_id = "right_tactile"
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(message.host_realtime_ns, 10**9)
        before = deepcopy(message)
        try:
            decoded = None
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                publisher.publish(message)
                executor.spin_once(timeout_sec=.02)
                decoded = receiver.receive_latest()
                if decoded is not None:
                    break
            assert decoded is not None
            assert decoded.source_sequence == 42
            assert (decoded.schema_version, decoded.rows, decoded.cols) == (2, 20, 20)
            for field in ("adc", "valid_mask", "updated_mask"):
                values = getattr(message, field)
                expected = [values[row * side + col] for row in range(20) for col in range(20)]
                assert getattr(decoded, field).tolist() == expected
            assert decoded.block_monotonic_ns.tolist() == list(message.block_monotonic_ns)[:160]
            assert decoded.block_realtime_ns.tolist() == list(message.block_realtime_ns)[:160]
            assert decoded.host_monotonic_ns == message.host_monotonic_ns
            assert decoded.host_realtime_ns == message.host_realtime_ns
            assert decoded.snapshot_monotonic_ns == message.snapshot_monotonic_ns
            assert decoded.header.frame_id == message.header.frame_id
            assert message == before
            assert node.sender.sent >= 1 and node.sender.rejected == 0
            assert all(pub.topic_name in ("/rosout", "/parameter_events") for pub in node.publishers)
        finally:
            executor.shutdown()
            node.destroy_node()
            publisher_node.destroy_node()
            receiver.close()


def test_launch_only_starts_bridge_and_uses_string_parameters(monkeypatch):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument

    root = Path(__file__).parents[1]
    spec = importlib.util.spec_from_file_location("display_bridge_launch", root / "launch/tactile_display_bridge.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "Node", lambda **kwargs: kwargs)
    description = module.generate_launch_description()
    declarations = {entity.name: entity for entity in description.entities if isinstance(entity, DeclareLaunchArgument)}
    assert set(declarations) == {"topic", "socket_path", "hand", "namespace"}
    nodes = [entity for entity in description.entities if isinstance(entity, dict)]
    assert len(nodes) == 1 and nodes[0]["executable"] == "vt_tactile_display_bridge"
    context = LaunchContext()
    context.launch_configurations.update(topic="/gloves/right/tactile/state", socket_path="/tmp/private dir/right.sock", hand="right")
    params = nodes[0]["parameters"][0]
    assert params["socket_path"].evaluate(context) == "/tmp/private dir/right.sock"
    assert params["use_sim_time"] is False
    entrypoints = (root / "setup.py").read_text()
    assert "vt_tactile_display_bridge = vt_tactile_capture.display_bridge:main" in entrypoints


def test_bridge_module_has_no_hardware_or_gui_imports():
    source = Path(__file__).parents[1] / "vt_tactile_capture/display_bridge.py"
    imports = [node.module or "" for node in ast.walk(ast.parse(source.read_text())) if isinstance(node, ast.ImportFrom)]
    assert not any(any(part in item for part in ("serial", "viewer", "torch", "PyQt")) for item in imports)


def test_two_hand_bridges_derive_separate_topics_sockets_and_filter_sources(context):
    nodes = [TactileDisplayBridge(context=context, sender_factory=FakeSender,
                namespace=f"/gloves/{hand}/tactile", parameter_overrides=[Parameter("hand", value=hand)])
             for hand in ("left", "right")]
    try:
        for node, hand in zip(nodes, ("left", "right")):
            assert node.subscription.topic_name == f"/gloves/{hand}/tactile/state"
            assert node.sender.path.endswith(f"/{hand}.sock")
            for source_hand in ("left", "right"):
                node.on_state(SimpleNamespace(hand=source_hand, clock_domain="CLOCK_MONOTONIC", timing_source="host_receive"))
            assert node.rejected == 1
            assert [message.hand for message in node.sender.messages] == [hand]
        assert nodes[0].get_fully_qualified_name() != nodes[1].get_fully_qualified_name()
    finally:
        for node in nodes:
            node.destroy_node()
