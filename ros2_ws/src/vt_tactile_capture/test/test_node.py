from dataclasses import replace
import os

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.context import Context
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time

from vt_tactile_capture.node import TactileNode
from vt_tactile_capture.protocol import Block
from vt_tactile_capture.replay import TactileReplayNode
from vt_tactile_capture.ros_messages import heatmap_message, sample_message
from vt_tactile_capture.serial_source import Batch
from vt_tactile_capture.viewer import ViewerModel, create_viewer_node
from vt_tactile_msgs.msg import TactileState


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeSource:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.batches = []
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self, timeout=1.0):
        self.stopped = True
        return True

    def drain(self, limit=128):
        result, self.batches = self.batches[:limit], self.batches[limit:]
        return result


@pytest.fixture
def context():
    value = Context()
    rclpy.init(context=value, domain_id=int(os.environ.get("VT_TACTILE_UNIT_TEST_DOMAIN_ID", "196")), args=[])
    yield value
    value.shutdown()


def batch(**kwargs):
    default = dict(session_id="synthetic", stream_epoch=1, sequence=1,
                   host_realtime_ns=10_000_000_000, host_monotonic_ns=8_000_000_000,
                   connected=True, time_valid=True, reason="ok",
                   blocks=(Block(row=0, column_group=0, status_raw=17, adc=tuple(range(10))),
                           Block(row=2, column_group=7, status_raw=255, adc=(23000,) * 10)))
    default.update(kwargs)
    return Batch(**default)


@pytest.fixture
def node(context):
    value = TactileNode(context=context, source_factory=FakeSource,
                        monotonic_ns=lambda: 8_010_000_000,
                        namespace="/gloves/right/tactile",
                        parameter_overrides=[Parameter("port", value="/fake/not_opened")])
    yield value
    value.destroy_node()


def test_qos_topics_and_worker_parameters(node):
    assert node.sample_publisher.topic_name == "/gloves/right/tactile/samples"
    assert node.state_publisher.topic_name == "/gloves/right/tactile/state"
    assert node.sample_publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    assert node.sample_publisher.qos_profile.durability == DurabilityPolicy.VOLATILE
    assert node.sample_publisher.qos_profile.depth == 128
    assert node.state_publisher.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert node.source.started
    assert node.source.kwargs["no_data_timeout"] == 2.0
    assert node.source.kwargs["queue_capacity"] == 512
    assert node.source.kwargs["read_batch_bytes"] == 1024
    assert node.source.kwargs["read_timeout"] == 0.002
    assert node.config["max_cell_age_ms"] == 100.0
    assert node.heatmap_publisher is None


def test_read_batch_overrides_reach_worker_and_qos(context):
    overrides = {"port": "/fake/not_opened", "queue_capacity": 256,
                 "sample_qos_depth": 64, "read_batch_bytes": 2048,
                 "read_timeout_sec": 0.004}
    value = TactileNode(context=context, source_factory=FakeSource,
                        parameter_overrides=[Parameter(name, value=setting)
                                             for name, setting in overrides.items()])
    try:
        assert value.source.started
        assert value.source.kwargs["queue_capacity"] == 256
        assert value.source.kwargs["read_batch_bytes"] == 2048
        assert value.source.kwargs["read_timeout"] == 0.004
        assert value.sample_publisher.qos_profile.depth == 64
    finally:
        value.destroy_node()


def test_two_capture_instances_keep_workers_topics_sessions_and_shutdown_separate(context):
    nodes = []
    try:
        for hand, adc in (("left", 111), ("right", 222)):
            node = TactileNode(context=context, source_factory=FakeSource,
                namespace=f"/gloves/{hand}/tactile", monotonic_ns=lambda: 8_010_000_000,
                parameter_overrides=[Parameter("hand", value=hand), Parameter("port", value=f"/fake/{hand}")])
            nodes.append(node)
            assert node.source.kwargs["hand"] == hand and node.source.kwargs["port"] == f"/fake/{hand}"
            assert node.frame_id == f"{hand}_tactile"
            assert node.sample_publisher.topic_name == f"/gloves/{hand}/tactile/samples"
            node.source.batches.append(batch(hand=hand, session_id=hand + "-session",
                blocks=(Block(row=0, column_group=0, status_raw=0, adc=(adc,) * 10),)))
            node.drain_samples()
            node.publish_state()
            assert node._last_snapshot.hand == hand
            assert node._last_snapshot.adc[0] == adc
        assert nodes[0].get_fully_qualified_name() != nodes[1].get_fully_qualified_name()
        nodes[0].destroy_node()
        assert nodes[0].source.stopped and not nodes[1].source.stopped
        nodes[1].publish_state()
        assert nodes[1]._last_snapshot.adc[0] == 222
    finally:
        for node in nodes:
            node.destroy_node()


@pytest.mark.parametrize("size", [1, 4096])
def test_read_batch_size_accepts_inclusive_bounds(context, size):
    value = TactileNode(context=context, source_factory=FakeSource,
                        parameter_overrides=[Parameter("port", value="/fake/not_opened"),
                                             Parameter("read_batch_bytes", value=size)])
    try:
        assert value.source.kwargs["read_batch_bytes"] == size
    finally:
        value.destroy_node()


@pytest.mark.parametrize("name, setting", [
    ("read_batch_bytes", 0), ("read_batch_bytes", -1), ("read_batch_bytes", 4097),
    ("read_batch_bytes", True), ("read_batch_bytes", 1024.0),
    ("read_timeout_sec", 0.0), ("read_timeout_sec", -0.001),
    ("read_timeout_sec", float("inf")), ("read_timeout_sec", float("-inf")),
    ("read_timeout_sec", float("nan")),
])
def test_invalid_read_parameters_rejected_before_worker(context, name, setting):
    constructed = []

    def forbidden(**kwargs):
        constructed.append(kwargs)
        raise AssertionError("Invalid parameters must never construct a worker")

    with pytest.raises((ValueError, TypeError, InvalidParameterTypeException)):
        TactileNode(context=context, source_factory=forbidden,
                    parameter_overrides=[Parameter("port", value="/fake/not_opened"),
                                         Parameter(name, value=setting)])
    assert not constructed


@pytest.mark.parametrize("name, replacement", [
    ("read_batch_bytes", 2048), ("read_timeout_sec", 0.004),
    ("queue_capacity", 1024), ("sample_qos_depth", 256),
])
def test_read_and_buffer_parameters_are_startup_only(node, name, replacement):
    original = node.get_parameter(name).value
    source_kwargs = dict(node.source.kwargs)
    result = node.set_parameters([Parameter(name, value=replacement)])
    assert not result[0].successful
    assert "startup-only" in result[0].reason
    assert node.get_parameter(name).value == original
    assert node.source.kwargs == source_kwargs
    unchanged = node.set_parameters([Parameter(name, value=original)])
    assert unchanged[0].successful


def test_drain_preserves_every_block_and_acquisition_clocks(node):
    node.sample_publisher = Recorder()
    node.state_publisher = Recorder()
    first = batch()
    second = batch(sequence=2, first_packet_index=2, host_realtime_ns=10_005_000_000,
                   host_monotonic_ns=8_005_000_000)
    node.source.batches = [first, second]
    node.drain_samples()
    assert len(node.sample_publisher.messages) == 2
    assert len(node.sample_publisher.messages[0].blocks) == 2
    assert list(node.sample_publisher.messages[0].blocks[0].adc) == list(range(10))
    assert node.sample_publisher.messages[0].blocks[1].status_raw == 255
    assert node.sample_publisher.messages[1].header.stamp.sec == 10
    assert node.sample_publisher.messages[1].header.stamp.nanosec == 5_000_000
    node.publish_state()
    node.publish_state()
    state1, state2 = node.state_publisher.messages
    assert state1.host_realtime_ns == state2.host_realtime_ns == 10_005_000_000
    assert state1.header == state2.header
    assert state1.schema_version == 2 and (state1.rows, state1.cols) == (20, 20)
    assert sum(state1.valid_mask) == 5
    assert sum(state1.updated_mask) == 5
    assert not any(state2.updated_mask)
    assert state1.adc[0] == 0 and state1.valid_mask[0]


def test_expiry_and_disconnection_never_create_new_measurement_time(node):
    node.state_publisher = Recorder()
    first = batch()
    node.source.batches = [first]
    node.drain_samples()
    node._monotonic_ns = lambda: 8_200_000_000
    node.publish_state()
    state = node.state_publisher.messages[-1]
    assert not any(state.valid_mask)
    assert state.reason == "stale"
    assert state.host_realtime_ns == 10_000_000_000
    node.source.batches = [replace(first, sequence=2, first_packet_index=2, blocks=(),
                                  connected=False, reason="serial_disconnected",
                                  host_realtime_ns=10_250_000_000, host_monotonic_ns=8_250_000_000)]
    node.drain_samples()
    node.publish_state()
    assert node.state_publisher.messages[-1].host_realtime_ns == 10_000_000_000


def test_no_port_never_constructs_worker(context):
    def forbidden(**kwargs):
        raise AssertionError("No port must never create a serial owner")
    node = TactileNode(context=context, source_factory=forbidden)
    try:
        node.drain_samples()
        node.publish_state()
        assert node.source is None
        assert node._last_snapshot.reason == "port_not_configured"
        assert not node._last_snapshot.connected
    finally:
        node.destroy_node()


def test_hand_derives_frame_id_in_live_and_replay(context):
    for node_class in (TactileNode, TactileReplayNode):
        node = node_class(context=context, parameter_overrides=[Parameter("hand", value="left")])
        try:
            assert node.frame_id == "left_tactile"
        finally:
            node.destroy_node()


def test_live_simulated_time_is_rejected_at_start_and_runtime(context, node):
    with pytest.raises(ValueError, match="use_sim_time"):
        TactileNode(context=context, parameter_overrides=[Parameter("use_sim_time", value=True)])
    result = node.set_parameters([Parameter("use_sim_time", value=True)])
    assert not result[0].successful
    assert not node.get_parameter("use_sim_time").value


def test_optional_rgb_preview_preserves_header_and_invalid_pixels(node):
    node.source.batches = [batch()]
    node.drain_samples()
    node.state_publisher = Recorder()
    node.publish_state()
    state = node.state_publisher.messages[-1]
    image = heatmap_message(state)
    assert image.encoding == "rgb8"
    assert image.height == image.width == 20
    assert image.step == 60
    assert len(image.data) == 20 * 20 * 3
    assert list(image.data[3:6]) == [0, 0, 0]
    assert image.header == state.header


def test_legacy_rgb_preview_crops_by_coordinates_and_preserves_source():
    legacy = TactileState(schema_version=1, rows=80, cols=80,
                         adc=[20000] * 6400, valid_mask=[True] * 6400,
                         updated_mask=[False] * 6400,
                         block_monotonic_ns=[100] * 640, block_realtime_ns=[1000] * 640)
    legacy.header.stamp.sec = 12
    legacy.header.frame_id = "right_tactile"
    legacy.adc[80] = 22000  # (1, 0), retained at compact index 20.
    legacy.adc[20] = 24000  # (0, 20), must not enter the compact grid.
    legacy.valid_mask[81] = False
    compact = TactileState(schema_version=2, rows=20, cols=20,
                          adc=[20000] * 400, valid_mask=[True] * 400)
    compact.header = legacy.header
    compact.adc[20] = 22000
    compact.valid_mask[21] = False
    image = heatmap_message(legacy)
    assert image == heatmap_message(compact)
    assert image.width == image.height == 20 and image.step == 60
    assert len(image.data) == 1200 and list(image.data[63:66]) == [0, 0, 0]
    assert (legacy.schema_version, legacy.rows, legacy.cols, len(legacy.adc)) == (1, 80, 80, 6400)
    assert legacy.adc[80] == 22000 and legacy.adc[20] == 24000


@pytest.mark.parametrize("field", ["adc", "valid_mask"])
def test_rgb_preview_rejects_incomplete_compact_arrays(field):
    state = TactileState(schema_version=2, rows=20, cols=20,
                         adc=[0] * 400, valid_mask=[True] * 400)
    setattr(state, field, list(getattr(state, field))[:-1])
    with pytest.raises(ValueError, match="400"):
        heatmap_message(state)


def test_diagnostics_are_low_rate_and_stable_for_unchanged_input(node):
    node.source.batches = [batch()]
    node.drain_samples()
    node.publish_state()
    node.diagnostic_publisher = Recorder()
    node.publish_diagnostics()
    node.publish_diagnostics()
    first, second = node.diagnostic_publisher.messages
    assert first.status == second.status
    assert first.status[0].level == DiagnosticStatus.OK
    assert node.diagnostic_timer.timer_period_ns == 1_000_000_000


def test_diagnostics_measure_host_publish_rates_and_source_age(node):
    node.diagnostic_publisher = Recorder()
    node.source.batches = [batch(), batch(sequence=2, first_packet_index=2,
                                          host_realtime_ns=10_005_000_000,
                                          host_monotonic_ns=8_005_000_000)]
    node.drain_samples()
    node.publish_state()
    node._monotonic_ns = lambda: 9_010_000_000
    node.publish_diagnostics()
    values = {item.key: item.value for item in node.diagnostic_publisher.messages[-1].status[0].values}
    assert float(values["batch_rate_hz"]) == 2.0
    assert float(values["packet_rate_hz"]) == 4.0
    assert values["valid_cell_count"] == "5"
    assert values["cell_count"] == "400"
    assert values["rows"] == values["cols"] == "20"
    assert values["source_age_ns"] == "1005000000"
    assert "not_hardware_scan_rate" in values["rate_scope"]
    node._monotonic_ns = lambda: 10_010_000_000
    node.publish_state()
    node.publish_diagnostics()
    values = {item.key: item.value for item in node.diagnostic_publisher.messages[-1].status[0].values}
    assert float(values["batch_rate_hz"]) == float(values["packet_rate_hz"]) == 0.0
    assert values["valid_cell_count"] == "0"


def test_viewer_best_effort_is_compatible_without_serial(context):
    viewer = create_viewer_node(ViewerModel(), context=context, namespace="/gloves/right/tactile")
    try:
        assert viewer.state_subscription.topic_name == "/gloves/right/tactile/state"
        assert viewer.state_subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert not hasattr(viewer, "source")
    finally:
        viewer.destroy_node()


def test_hardware_free_replay_uses_recorded_monotonic_axis(context):
    replay = TactileReplayNode(context=context,
                               parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        assert not hasattr(replay, "source")
        assert replay.sample_subscription.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        replay.state_publisher = Recorder()
        replay.get_clock().set_ros_time_override(Time(nanoseconds=10_020_000_000))
        message = sample_message(batch(), "right_tactile")
        replay.on_samples(message)
        replay.publish_state()
        state = replay.state_publisher.messages[-1]
        assert state.host_realtime_ns == 10_000_000_000
        assert state.snapshot_monotonic_ns == 8_020_000_000
        assert sum(state.valid_mask) == 5
        replay.get_clock().set_ros_time_override(Time(nanoseconds=10_200_000_000))
        replay.publish_state()
        assert not any(replay.state_publisher.messages[-1].valid_mask)
        replay.get_clock().set_ros_time_override(Time(nanoseconds=10_000_000_000))
        replay.on_samples(message)
        replay.publish_state()
        assert sum(replay.state_publisher.messages[-1].valid_mask) == 5
        message.header.stamp.nanosec = 1
        replay.on_samples(message)
        assert replay.rejected_samples == 1
        assert replay._last_batch is None
        assert not any(replay.assembler.snapshot(8_000_000_000, 100_000_000).valid_mask)
    finally:
        replay.destroy_node()
