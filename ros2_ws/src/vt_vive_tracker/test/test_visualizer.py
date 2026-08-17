from __future__ import annotations

import ast
from pathlib import Path

import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from vt_tracker_msgs.msg import TrackerStatus

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    PoseValue,
    RoleSnapshot,
    StatusValue,
    VisualHealth,
)
from vt_vive_tracker.visualizer import (
    TrackerVisualizerNode,
    markers_for_snapshot,
)


PACKAGE_ROOT = Path(__file__).parents[1]


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def node(ros_context):
    value = TrackerVisualizerNode(monotonic_ns=lambda: 1_000_000_000)
    yield value
    value.destroy_node()


def make_pose(frame_id="vive_map", x=1.0):
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.pose.position.x = x
    message.pose.position.y = 2.0
    message.pose.position.z = 3.0
    message.pose.orientation.w = 1.0
    return message


def make_status(role="left_wrist", state=2):
    message = TrackerStatus()
    message.role = role
    message.state = state
    message.tracker_id = "b" * 64
    message.tracking_status = 7
    message.valid_sample_count = 123
    message.invalid_report_count = 4
    message.dropped_queue_count = 1
    return message


def marker_by_namespace(markers, namespace):
    return next(item for item in markers if item.ns == namespace)


def test_node_creates_six_fixed_subscriptions_with_matching_qos(node):
    assert {sub.topic_name for sub in node.tracker_subscriptions} == {
        f"/vive/{role}/{kind}"
        for role in FIXED_ROLES
        for kind in ("pose", "status")
    }
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
        sub.qos_profile.reliability
        == ReliabilityPolicy.BEST_EFFORT
        and sub.qos_profile.durability == DurabilityPolicy.VOLATILE
        for sub in pose_subscriptions
    )
    assert all(
        sub.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        and sub.qos_profile.durability
        == DurabilityPolicy.TRANSIENT_LOCAL
        for sub in status_subscriptions
    )
    assert node.marker_publisher.topic_name == (
        "/vive/visualization/markers"
    )
    assert (
        node.marker_publisher.qos_profile.reliability
        == ReliabilityPolicy.RELIABLE
    )


def test_no_pose_snapshot_emits_red_sphere_and_offline_text_only():
    snapshot = RoleSnapshot(
        "left_wrist",
        VisualHealth.OFFLINE,
        None,
        (),
        0.0,
        None,
    )

    markers = markers_for_snapshot(snapshot, Time())

    assert [(item.ns, item.type) for item in markers] == [
        ("left_wrist/health", Marker.SPHERE),
        ("left_wrist/label", Marker.TEXT_VIEW_FACING),
    ]
    assert all(item.header.frame_id == "vive_map" for item in markers)
    assert all(
        item.color.r == 1.0 and item.color.g == 0.0
        for item in markers
    )
    assert markers[0].pose.position.x == -0.30
    assert "left_wrist OFFLINE" in markers[-1].text
    assert "valid=? invalid=? dropped=?" in markers[-1].text


def test_fresh_snapshot_emits_pose_trail_counters_and_green_health():
    value = PoseValue(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    status = StatusValue(2, 123, 4, 0)
    snapshot = RoleSnapshot(
        "right_wrist",
        VisualHealth.FRESH,
        value,
        (value,),
        119.5,
        status,
    )

    markers = markers_for_snapshot(snapshot, Time())

    assert {item.type for item in markers} == {
        Marker.ARROW,
        Marker.LINE_STRIP,
        Marker.TEXT_VIEW_FACING,
        Marker.SPHERE,
    }
    assert len({item.id for item in markers}) == 4
    assert all(item.header.frame_id == "vive_map" for item in markers)
    arrow = marker_by_namespace(markers, "right_wrist/arrow")
    assert arrow.pose.position.x == 1.0
    assert arrow.pose.orientation.w == 1.0
    label = marker_by_namespace(markers, "right_wrist/label")
    assert "119.5 Hz" in label.text
    assert "valid=123 invalid=4 dropped=0" in label.text
    health = marker_by_namespace(markers, "right_wrist/health")
    assert (health.color.r, health.color.g, health.color.b) == (
        0.0,
        1.0,
        0.0,
    )


@pytest.mark.parametrize(
    ("health", "expected_rgb"),
    (
        (VisualHealth.DELAYED, (1.0, 0.75, 0.0)),
        (VisualHealth.OFFLINE, (1.0, 0.0, 0.0)),
    ),
)
def test_delayed_and_offline_poses_are_never_green(
    health, expected_rgb
):
    value = PoseValue(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    snapshot = RoleSnapshot(
        "torso", health, value, (value,), 0.0, None
    )

    markers = markers_for_snapshot(snapshot, Time())

    health_marker = marker_by_namespace(markers, "torso/health")
    assert (
        health_marker.color.r,
        health_marker.color.g,
        health_marker.color.b,
    ) == expected_rgb
    arrow = marker_by_namespace(markers, "torso/arrow")
    assert arrow.color.a == 0.35


def test_callbacks_reject_wrong_frame_and_mismatched_status_role(node):
    node.on_pose("torso", make_pose(frame_id="camera_link"))
    node.on_status("torso", make_status(role="left_wrist"))

    torso = node.model.snapshots(1_000_000_000)[2]

    assert torso.pose is None
    assert torso.status is None


def test_warning_throttle_is_independent_per_role(node, capfd):
    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    node.on_pose("right_wrist", make_pose(frame_id="camera_link"))
    first = capfd.readouterr().err

    node.on_pose("left_wrist", make_pose(frame_id="camera_link"))
    second = capfd.readouterr().err

    assert "role=left_wrist" in first
    assert "role=right_wrist" in first
    assert "role=left_wrist" not in second


def test_callbacks_route_pose_and_status_to_only_the_fixed_role(node):
    node.on_status("right_wrist", make_status(role="right_wrist"))
    node.on_pose("right_wrist", make_pose(x=4.0))

    snapshots = node.model.snapshots(1_000_000_000)

    assert snapshots[0].pose is None
    assert snapshots[1].pose.position == (4.0, 2.0, 3.0)
    assert snapshots[1].status.valid_sample_count == 123
    assert snapshots[1].status.tracker_id == "b" * 64
    assert snapshots[1].status.tracking_status == 7
    assert snapshots[2].pose is None


def test_one_publish_contains_namespaces_for_all_three_roles(node):
    recorder = Recorder()
    node.marker_publisher = recorder

    node.publish_markers()

    assert len(recorder.messages) == 1
    message = recorder.messages[0]
    assert type(message) is MarkerArray
    assert {item.ns.split("/", 1)[0] for item in message.markers} == set(
        FIXED_ROLES
    )


def test_visualizer_imports_only_ros_and_local_runtime_modules():
    path = PACKAGE_ROOT / "vt_vive_tracker" / "visualizer.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported_roots = {
        alias.name.split(".", 1)[0]
        for item in ast.walk(tree)
        if isinstance(item, (ast.Import, ast.ImportFrom))
        for alias in item.names
    }

    assert imported_roots.isdisjoint({"pyvut", "hid", "hidapi"})


def test_setup_exports_visualizer_entry_point():
    setup = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    assert (
        "vt_vive_tracker_visualizer = vt_vive_tracker.visualizer:main"
        in setup
    )
