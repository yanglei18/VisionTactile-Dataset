from __future__ import annotations

import ast
import math
import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from vt_tracker_msgs.msg import TrackerStatus

from vt_vive_tracker.visualization_model import FIXED_ROLES, VisualHealth
from vt_vive_tracker_gui.ros_node import TrackerGuiNode
from vt_vive_tracker_gui.snapshot_store import LatestSnapshotStore


PACKAGE_ROOT = Path(__file__).parents[1]


class MutableClock:
    def __init__(self, value: int = 1_000_000_000) -> None:
        self.value = value

    def __call__(self) -> int:
        return self.value


@pytest.fixture
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def mutable_clock():
    return MutableClock()


@pytest.fixture
def store():
    return LatestSnapshotStore()


@pytest.fixture
def node(ros_context, mutable_clock, store):
    value = TrackerGuiNode(store, monotonic_ns=mutable_clock)
    yield value
    value.destroy_node()


def make_pose(frame_id: str = "vive_map", x: float = 1.0) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x = x
    message.pose.position.y = 2.0
    message.pose.position.z = 3.0
    message.pose.orientation.w = 1.0
    return message


def make_status(role: str = "left_wrist", state: int = 2) -> TrackerStatus:
    message = TrackerStatus()
    message.role = role
    message.state = state
    message.tracker_id = "a" * 64
    message.tracking_status = 7
    message.valid_sample_count = 123
    message.invalid_report_count = 4
    message.dropped_queue_count = 1
    return message


def pose_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def status_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def test_node_creates_only_six_read_only_subscriptions(node):
    assert {sub.topic_name for sub in node.tracker_subscriptions} == {
        f"/vive/{role}/{kind}"
        for role in FIXED_ROLES
        for kind in ("pose", "status")
    }
    assert len(node.tracker_subscriptions) == 6
    assert "/rosout" not in {
        publisher.topic_name for publisher in node.publishers
    }
    parameter_service_suffixes = {
        "describe_parameters",
        "get_parameter_types",
        "get_parameters",
        "list_parameters",
        "set_parameters",
        "set_parameters_atomically",
    }
    assert all(
        service.service_name.rsplit("/", 1)[-1]
        not in parameter_service_suffixes
        for service in node.services
    )
    assert list(node.clients) == []

    source = (
        PACKAGE_ROOT / "vt_vive_tracker_gui" / "ros_node.py"
    ).read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "create_service" not in source
    assert "create_client" not in source


def test_subscriptions_match_tracker_publisher_qos(node):
    pose_subscriptions = [
        sub
        for sub in node.tracker_subscriptions
        if sub.topic_name.endswith("/pose")
    ]
    status_subscriptions = [
        sub
        for sub in node.tracker_subscriptions
        if sub.topic_name.endswith("/status")
    ]

    assert all(
        sub.qos_profile.history == HistoryPolicy.KEEP_LAST
        and sub.qos_profile.depth == 10
        and sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        and sub.qos_profile.durability == DurabilityPolicy.VOLATILE
        for sub in pose_subscriptions
    )
    assert all(
        sub.qos_profile.history == HistoryPolicy.KEEP_LAST
        and sub.qos_profile.depth == 1
        and sub.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        and sub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
        for sub in status_subscriptions
    )
    assert node.snapshot_timer.timer_period_ns == 50_000_000


def test_valid_pose_immediately_overwrites_latest_snapshot(node, store):
    node.on_status("left_wrist", make_status("left_wrist"))
    before = store.latest().version

    node.on_pose("left_wrist", make_pose(x=4.0))

    value = store.latest()
    assert value.version == before + 1
    assert value.roles[0].pose.position == (4.0, 2.0, 3.0)
    assert value.roles[0].status.tracker_id == "a" * 64


def test_valid_status_immediately_forwards_all_fields(node, store):
    message = make_status("right_wrist", state=3)

    node.on_status("right_wrist", message)

    value = store.latest()
    assert value.version == 1
    assert value.roles[1].status.state == 3
    assert value.roles[1].status.tracker_id == "a" * 64
    assert value.roles[1].status.tracking_status == 7
    assert value.roles[1].status.valid_sample_count == 123
    assert value.roles[1].status.invalid_report_count == 4
    assert value.roles[1].status.dropped_queue_count == 1


def test_timer_advances_fresh_pose_to_offline_without_new_messages(
    mutable_clock, node, store
):
    node.on_status("torso", make_status("torso"))
    node.on_pose("torso", make_pose())
    assert store.latest().roles[2].health is VisualHealth.FRESH

    mutable_clock.value += 1_000_000_001
    node.publish_snapshot()

    assert store.latest().roles[2].health is VisualHealth.OFFLINE


def test_wrong_frame_and_non_finite_pose_are_rejected(node, store):
    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    invalid = make_pose()
    invalid.pose.orientation.z = math.nan
    node.on_pose("right_wrist", invalid)

    node.publish_snapshot()
    value = store.latest()
    assert value.roles[0].pose is None
    assert value.roles[1].pose is None


def test_status_role_mismatch_is_rejected(node, store):
    node.on_status("torso", make_status("left_wrist"))

    node.publish_snapshot()

    assert store.latest().roles[2].status is None


def test_warnings_are_independently_throttled_and_reach_store(
    mutable_clock, node, store, capfd
):
    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    first = store.latest_diagnostic()
    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    assert store.latest_diagnostic().version == first.version

    node.on_pose("right_wrist", make_pose(frame_id="camera_link"))
    second = store.latest_diagnostic()
    assert second.version == first.version + 1

    invalid = make_pose()
    invalid.pose.position.x = math.inf
    node.on_pose("left_wrist", invalid)
    third = store.latest_diagnostic()
    assert third.version == second.version + 1

    mutable_clock.value += 4_999_999_999
    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    assert store.latest_diagnostic().version == third.version

    mutable_clock.value += 1
    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    fourth = store.latest_diagnostic()
    assert fourth.version == third.version + 1
    assert fourth.monotonic_ns == 6_000_000_000
    assert fourth.text == "unexpected pose frame"

    logged = capfd.readouterr().err
    assert fourth.text in logged


def test_mismatched_status_writes_throttled_diagnostic(node, store):
    node.on_status("torso", make_status("left_wrist"))

    event = store.latest_diagnostic()
    assert event.version == 1
    assert event.text == "mismatched status role"
    assert event.monotonic_ns == 1_000_000_000


def test_ros_node_imports_no_hardware_or_process_modules():
    path = PACKAGE_ROOT / "vt_vive_tracker_gui" / "ros_node.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported_roots = set()
    for item in ast.walk(tree):
        if isinstance(item, ast.Import):
            imported_roots.update(
                alias.name.split(".", 1)[0] for alias in item.names
            )
        elif isinstance(item, ast.ImportFrom) and item.module:
            imported_roots.add(item.module.split(".", 1)[0])

    assert imported_roots.isdisjoint({"pyvut", "hid", "hidapi", "subprocess"})


def test_real_ros_publishers_deliver_all_roles_and_survive_gui_shutdown(
    ros_context,
):
    store = LatestSnapshotStore()
    gui_node = TrackerGuiNode(store)
    publisher_node = Node(
        "test_tracker_publisher",
        enable_rosout=False,
        start_parameter_services=False,
    )
    pose_publishers = {
        role: publisher_node.create_publisher(
            PoseStamped, f"/vive/{role}/pose", pose_qos()
        )
        for role in FIXED_ROLES
    }
    status_publishers = {
        role: publisher_node.create_publisher(
            TrackerStatus, f"/vive/{role}/status", status_qos()
        )
        for role in FIXED_ROLES
    }
    executor = SingleThreadedExecutor()
    executor.add_node(gui_node)
    executor.add_node(publisher_node)
    gui_destroyed = False

    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            for index, role in enumerate(FIXED_ROLES, start=1):
                status_publishers[role].publish(make_status(role))
                pose_publishers[role].publish(make_pose(x=float(index)))
            executor.spin_once(timeout_sec=0.05)
            latest = store.latest()
            if latest is not None and all(
                role.pose is not None and role.status is not None
                for role in latest.roles
            ):
                break

        latest = store.latest()
        assert latest is not None
        assert [role.pose.position[0] for role in latest.roles] == [1.0, 2.0, 3.0]
        assert [role.status.tracker_id for role in latest.roles] == [
            "a" * 64,
            "a" * 64,
            "a" * 64,
        ]

        executor.remove_node(gui_node)
        gui_node.destroy_node()
        gui_destroyed = True
        pose_publishers["torso"].publish(make_pose(x=9.0))
        assert publisher_node.get_name() == "test_tracker_publisher"
        assert {
            publisher.topic_name
            for publisher in publisher_node.publishers
            if publisher.topic_name.startswith("/vive/")
        } == {
            f"/vive/{role}/{kind}"
            for role in FIXED_ROLES
            for kind in ("pose", "status")
        }
    finally:
        executor.remove_node(gui_node)
        executor.remove_node(publisher_node)
        if not gui_destroyed:
            gui_node.destroy_node()
        publisher_node.destroy_node()
        executor.shutdown()
