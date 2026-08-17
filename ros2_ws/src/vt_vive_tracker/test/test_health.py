from dataclasses import replace

from vt_vive_tracker.health import (
    CONNECTED_NO_TRACKING,
    DISCONNECTED,
    INVALID_DATA,
    TRACKING,
    BoundedSampleQueue,
    TrackerHealthBook,
)
from vt_vive_tracker.identity import tracker_id
from vt_vive_tracker.model import NativePose, normalize_pose
from vt_vive_tracker.roles import RoleMap


ADDRESSES = (
    bytes.fromhex("230142b782d3"),
    bytes.fromhex("310253c893e4"),
    bytes.fromhex("410364d9a4f5"),
)
ROLES = ("left_wrist", "right_wrist", "torso")


def role_map():
    by_role = {
        role: tracker_id(address)
        for role, address in zip(ROLES, ADDRESSES)
    }
    return RoleMap(
        by_role,
        {value: role for role, value in by_role.items()},
    )


def sample(
    role="left_wrist",
    *,
    status=2,
    monotonic_ns=100,
    packet_index=1,
):
    mapping = role_map()
    native = NativePose(
        address=ADDRESSES[ROLES.index(role)],
        packet_index=packet_index,
        tracker_index=0,
        buttons=0,
        position=(1.0, 2.0, 3.0),
        quaternion_wzyx=(1.0, 0.0, 0.0, 0.0),
        acceleration=(0.0, 0.0, 0.0),
        angular_velocity_native=(0.0, 0.0, 0.0, 0.0),
        tracking_status=status,
    )
    return normalize_pose(
        native,
        role=role,
        tracker_id=mapping.tracker_id_for_role(role),
        host_monotonic_ns=monotonic_ns,
        host_realtime_ns=monotonic_ns + 10,
    )


def snapshots(book, now_ns=100):
    return {value.role: value for value in book.snapshot(now_ns)}


def test_initial_state_is_disconnected_for_all_roles():
    values = snapshots(TrackerHealthBook(role_map()), now_ns=0)

    assert set(values) == set(ROLES)
    assert all(value.state == DISCONNECTED for value in values.values())
    assert all(value.valid_sample_count == 0 for value in values.values())


def test_finite_status_three_is_connected_without_tracking():
    book = TrackerHealthBook(role_map())
    assert book.observe_sample(sample(status=3)) is True

    value = snapshots(book)["left_wrist"]
    assert value.state == CONNECTED_NO_TRACKING
    assert value.tracking_status == 3
    assert value.valid_sample_count == 1
    assert value.last_valid_pose_monotonic_ns == 0


def test_valid_status_two_is_tracking_and_updates_pose_timestamp():
    book = TrackerHealthBook(role_map())
    book.observe_sample(sample(monotonic_ns=123))

    value = snapshots(book, now_ns=123)["left_wrist"]
    assert value.state == TRACKING
    assert value.valid_sample_count == 1
    assert value.last_report_monotonic_ns == 123
    assert value.last_valid_pose_monotonic_ns == 123


def test_isolated_invalid_after_recent_pose_counts_without_downgrade():
    mapping = role_map()
    book = TrackerHealthBook(mapping)
    book.observe_sample(sample("left_wrist", monotonic_ns=100))
    book.observe_sample(sample("right_wrist", monotonic_ns=100))

    assert book.observe_invalid(
        "left_wrist",
        mapping.tracker_id_for_role("left_wrist"),
        110,
    ) is False

    values = snapshots(book, now_ns=110)
    assert values["left_wrist"].state == TRACKING
    assert values["left_wrist"].invalid_report_count == 1
    assert values["right_wrist"].state == TRACKING
    assert values["right_wrist"].invalid_report_count == 0


def test_invalid_without_a_recent_valid_pose_sets_invalid_data():
    mapping = role_map()
    book = TrackerHealthBook(
        mapping, disconnect_timeout_ns=1_000_000_000
    )
    book.observe_sample(sample("left_wrist", monotonic_ns=100))

    assert book.observe_invalid(
        "left_wrist",
        mapping.tracker_id_for_role("left_wrist"),
        1_000_000_100,
    ) is True

    value = snapshots(book, now_ns=1_000_000_100)["left_wrist"]
    assert value.state == INVALID_DATA
    assert value.invalid_report_count == 1


def test_one_second_without_report_transitions_to_disconnected():
    book = TrackerHealthBook(role_map(), disconnect_timeout_ns=1_000_000_000)
    book.observe_sample(sample(monotonic_ns=50))

    assert snapshots(book, now_ns=1_000_000_049)["left_wrist"].state == TRACKING
    assert (
        snapshots(book, now_ns=1_000_000_050)["left_wrist"].state
        == DISCONNECTED
    )


def test_counters_are_per_role_and_never_reset_on_disconnect():
    mapping = role_map()
    book = TrackerHealthBook(mapping, disconnect_timeout_ns=10)
    book.observe_sample(sample("left_wrist", monotonic_ns=1))
    book.observe_sample(sample("left_wrist", monotonic_ns=2))
    book.observe_invalid(
        "left_wrist",
        mapping.tracker_id_for_role("left_wrist"),
        3,
    )
    book.record_drop("left_wrist")

    values = snapshots(book, now_ns=13)
    left = values["left_wrist"]
    assert left.state == DISCONNECTED
    assert left.valid_sample_count == 2
    assert left.invalid_report_count == 1
    assert left.dropped_queue_count == 1
    assert values["right_wrist"].valid_sample_count == 0


def test_state_transition_requests_one_immediate_publish():
    book = TrackerHealthBook(role_map())
    assert book.consume_status_publish_request() is False

    book.observe_sample(sample())
    assert book.consume_status_publish_request() is True
    assert book.consume_status_publish_request() is False

    book.observe_sample(sample(monotonic_ns=101, packet_index=2))
    assert book.consume_status_publish_request() is False

    snapshots(book, now_ns=1_000_000_101)
    assert book.consume_status_publish_request() is True


def test_default_queue_preserves_all_4096_samples_in_order():
    book = TrackerHealthBook(role_map())
    queue = BoundedSampleQueue(book)
    original = [
        replace(
            sample(monotonic_ns=index + 1),
            packet_index=index % 0x10000,
        )
        for index in range(4096)
    ]

    for value in original:
        queue.put(value)

    assert queue.drain() == tuple(original)
    assert snapshots(book, now_ns=5000)["left_wrist"].dropped_queue_count == 0


def test_queue_overflow_drops_oldest_and_counts_only_dropped_role():
    book = TrackerHealthBook(role_map())
    queue = BoundedSampleQueue(book, capacity=2)
    first = sample("left_wrist", packet_index=1)
    second = sample("right_wrist", packet_index=2)
    newest = sample("torso", packet_index=3)

    queue.put(first)
    queue.put(second)
    queue.put(newest)

    assert queue.drain() == (second, newest)
    values = snapshots(book)
    assert values["left_wrist"].dropped_queue_count == 1
    assert values["right_wrist"].dropped_queue_count == 0
    assert values["torso"].dropped_queue_count == 0
