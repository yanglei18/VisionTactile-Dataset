from __future__ import annotations

import os
import stat
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)

from vt_vive_tracker.health import INVALID_DATA
from vt_vive_tracker.identity import tracker_id
from vt_vive_tracker.model import NativePose, normalize_pose
from vt_vive_tracker.node import (
    ViveTrackerNode,
    make_runtime_config,
)
import vt_vive_tracker.node as node_module


PACKAGE_ROOT = Path(__file__).parents[1]
RUNTIME_MODULES = (
    PACKAGE_ROOT / "vt_vive_tracker" / "node.py",
    PACKAGE_ROOT / "vt_vive_tracker" / "pose_source.py",
)
FORBIDDEN = (
    "send_feature_report",
    "get_feature_report",
    "UltimateTrackerAPI",
    "execute_feature_writes",
    "PAIR_DEVICE",
)
ROLES = ("left_wrist", "right_wrist", "torso")
ADDRESSES = (
    bytes.fromhex("230142b782d3"),
    bytes.fromhex("310253c893e4"),
    bytes.fromhex("410364d9a4f5"),
)


class FakeSource:
    instances = []

    def __init__(self, bundle_path, role_map, **kwargs):
        self.bundle_path = bundle_path
        self.role_map = role_map
        self.kwargs = kwargs
        self.stop_count = 0
        self.__class__.instances.append(self)

    def start(self, on_sample, on_invalid, on_error):
        self.on_sample = on_sample
        self.on_invalid = on_invalid
        self.on_error = on_error

    def stop(self):
        self.stop_count += 1


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def write_private(path, text):
    path.write_text(text, encoding="utf-8")
    path.chmod(0o600)
    return path


def role_map_text():
    return (
        "schema_version: 1\n"
        "roles:\n"
        + "".join(
            f"  {role}: {tracker_id(address)}\n"
            for role, address in zip(ROLES, ADDRESSES)
        )
    )


def parameter_overrides(tmp_path, **extra):
    bundle = write_private(tmp_path / "bundle.json", "{}\n")
    roles = write_private(tmp_path / "roles.yaml", role_map_text())
    values = {
        "bundle_path": str(bundle),
        "role_map_path": str(roles),
        **extra,
    }
    return [
        Parameter(name, value=value)
        for name, value in values.items()
    ]


@pytest.fixture
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def node(tmp_path, monkeypatch, ros_context):
    FakeSource.instances.clear()
    runtime = tmp_path / "runtime"
    runtime.mkdir(mode=0o700)
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(runtime))
    value = ViveTrackerNode(
        parameter_overrides=parameter_overrides(tmp_path),
        source_factory=FakeSource,
    )
    yield value
    value.destroy_node()


def make_sample(role="left_wrist", status=2, realtime_ns=1_234_567_890):
    index = ROLES.index(role)
    native = NativePose(
        address=ADDRESSES[index],
        packet_index=42,
        tracker_index=index,
        buttons=5,
        position=(1.0, 2.0, 3.0),
        quaternion_wzyx=(1.0, 0.0, 0.0, 0.0),
        acceleration=(4.0, 5.0, 6.0),
        angular_velocity_native=(7.0, 8.0, 9.0, 99.0),
        tracking_status=status,
    )
    return normalize_pose(
        native,
        role=role,
        tracker_id=tracker_id(ADDRESSES[index]),
        host_monotonic_ns=123,
        host_realtime_ns=realtime_ns,
    )


def test_runtime_modules_contain_no_device_write_or_legacy_api_tokens():
    for path in RUNTIME_MODULES:
        text = path.read_text(encoding="utf-8")
        for token in FORBIDDEN:
            assert token not in text, f"{path.name} contains {token}"


def test_exact_parameter_defaults_and_entry_point_are_declared():
    source = (PACKAGE_ROOT / "vt_vive_tracker" / "node.py").read_text()
    assert 'declare_parameter("frame_id", "vive_map")' in source
    assert 'declare_parameter("status_rate_hz", 10.0)' in source
    assert 'declare_parameter("read_timeout_ms", 100)' in source
    assert 'declare_parameter("disconnect_timeout_ms", 1000)' in source
    assert 'declare_parameter("queue_capacity", 4096)' in source
    setup = (PACKAGE_ROOT / "setup.py").read_text()
    assert (
        "vt_vive_tracker_node = vt_vive_tracker.node:main" in setup
    )


def test_runtime_config_requires_paths_and_rejects_sim_time():
    with pytest.raises(ValueError, match="bundle_path"):
        make_runtime_config(
            bundle_path="",
            role_map_path="/roles.yaml",
            frame_id="vive_map",
            status_rate_hz=10.0,
            read_timeout_ms=100,
            disconnect_timeout_ms=1000,
            queue_capacity=4096,
            use_sim_time=False,
        )
    with pytest.raises(ValueError, match="use_sim_time"):
        make_runtime_config(
            bundle_path="/bundle.json",
            role_map_path="/roles.yaml",
            frame_id="vive_map",
            status_rate_hz=10.0,
            read_timeout_ms=100,
            disconnect_timeout_ms=1000,
            queue_capacity=4096,
            use_sim_time=True,
        )


def test_creates_nine_exact_topics_with_explicit_qos(node):
    assert set(node.publisher_groups) == set(ROLES)
    observed_topics = set()
    for role, group in node.publisher_groups.items():
        observed_topics.update(
            {
                group.sample.topic_name,
                group.pose.topic_name,
                group.status.topic_name,
            }
        )
        for publisher in (group.sample, group.pose):
            qos = publisher.qos_profile
            assert qos.history == HistoryPolicy.KEEP_LAST
            assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
            assert qos.durability == DurabilityPolicy.VOLATILE
        status_qos = group.status.qos_profile
        assert status_qos.reliability == ReliabilityPolicy.RELIABLE
        assert status_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert status_qos.depth == 1
    assert observed_topics == {
        f"/vive/{role}/{kind}"
        for role in ROLES
        for kind in ("sample", "pose", "status")
    }


def replace_publishers(node):
    recorders = {}
    for role, group in node.publisher_groups.items():
        recorders[role] = {
            "sample": Recorder(),
            "pose": Recorder(),
            "status": Recorder(),
        }
        group.sample = recorders[role]["sample"]
        group.pose = recorders[role]["pose"]
        group.status = recorders[role]["status"]
    return recorders


def test_sample_stamp_pose_header_and_role_routing_are_exact(node):
    recorders = replace_publishers(node)
    source = FakeSource.instances[-1]
    source.on_sample(make_sample("right_wrist"))

    node.drain_once()

    sample_message = recorders["right_wrist"]["sample"].messages[0]
    pose_message = recorders["right_wrist"]["pose"].messages[0]
    assert sample_message.header.stamp.sec == 1
    assert sample_message.header.stamp.nanosec == 234_567_890
    assert sample_message.header == pose_message.header
    assert sample_message.header.frame_id == "vive_map"
    assert sample_message.role == "right_wrist"
    assert sample_message.tracker_id == tracker_id(ADDRESSES[1])
    assert recorders["left_wrist"]["sample"].messages == []
    assert recorders["torso"]["sample"].messages == []


def test_nontracking_sample_has_sample_but_no_pose(node):
    recorders = replace_publishers(node)
    FakeSource.instances[-1].on_sample(
        make_sample("left_wrist", status=3)
    )

    node.drain_once()

    assert len(recorders["left_wrist"]["sample"].messages) == 1
    assert recorders["left_wrist"]["pose"].messages == []


def test_invalid_report_immediately_publishes_status_only(node):
    recorders = replace_publishers(node)
    source = FakeSource.instances[-1]
    source.on_invalid(
        "torso",
        tracker_id(ADDRESSES[2]),
        node._monotonic_ns(),
    )

    node.drain_once()

    assert recorders["torso"]["sample"].messages == []
    assert recorders["torso"]["pose"].messages == []
    assert len(recorders["torso"]["status"].messages) == 1
    assert recorders["torso"]["status"].messages[0].state == INVALID_DATA


def test_periodic_status_publishes_all_three_roles(node):
    recorders = replace_publishers(node)

    node.publish_status_once()

    assert all(
        len(recorders[role]["status"].messages) == 1 for role in ROLES
    )


def test_destroy_stops_source_once_and_releases_owner_only_lock(
    node, tmp_path
):
    source = FakeSource.instances[-1]
    lock_path = node.lock_path
    assert stat.S_IMODE(lock_path.stat().st_mode) == 0o600

    node.destroy_node()
    node.destroy_node()

    assert source.stop_count == 1


def test_main_handles_keyboard_interrupt_and_shuts_down(monkeypatch):
    events = []

    class FakeNode:
        def destroy_node(self):
            events.append("destroy")

    monkeypatch.setattr(
        node_module, "ViveTrackerNode", lambda: events.append("node") or FakeNode()
    )
    monkeypatch.setattr(
        node_module.rclpy, "init", lambda args=None: events.append("init")
    )
    monkeypatch.setattr(
        node_module.rclpy,
        "spin",
        lambda node: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(node_module.rclpy, "ok", lambda: True)
    monkeypatch.setattr(
        node_module.rclpy, "shutdown", lambda: events.append("shutdown")
    )

    node_module.main([])

    assert events == ["init", "node", "destroy", "shutdown"]
