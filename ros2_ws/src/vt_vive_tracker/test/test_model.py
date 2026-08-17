from dataclasses import FrozenInstanceError
import math

import pytest

from vt_vive_tracker.model import (
    NativePose,
    normalize_pose,
    stamp_from_realtime_ns,
)


def native(**overrides):
    fields = {
        "address": bytes.fromhex("230142b782d3"),
        "packet_index": 7,
        "tracker_index": 1,
        "buttons": 0,
        "position": (1.0, 2.0, 3.0),
        "quaternion_wzyx": (1.0, 0.0, 0.0, 0.0),
        "acceleration": (4.0, 5.0, 6.0),
        "angular_velocity_native": (7.0, 8.0, 9.0, 99.0),
        "tracking_status": 2,
    }
    if "quaternion" in overrides:
        overrides["quaternion_wzyx"] = overrides.pop("quaternion")
    if "angular_velocity" in overrides:
        overrides["angular_velocity_native"] = overrides.pop(
            "angular_velocity"
        )
    fields.update(overrides)
    return NativePose(**fields)


def normalized(value=None, **overrides):
    return normalize_pose(
        native() if value is None else value,
        role=overrides.pop("role", "left_wrist"),
        tracker_id=overrides.pop("tracker_id", "a" * 64),
        host_monotonic_ns=overrides.pop("host_monotonic_ns", 10),
        host_realtime_ns=overrides.pop("host_realtime_ns", 20),
        **overrides,
    )


def test_firmware_quaternion_is_converted_to_ros_xyzw():
    sample = normalized(
        native(quaternion=(0.5, -0.25, 0.125, -0.75))
    )
    assert sample.quaternion_xyzw == (-0.75, 0.125, -0.25, 0.5)


def test_status_two_finite_unit_quaternion_is_valid():
    assert normalized(
        native(quaternion=(1.0, 0.0, 0.0, 0.0), tracking_status=2)
    ).pose_valid is True


@pytest.mark.parametrize("status", [0, 1, 3, 4])
def test_non_full_tracking_status_is_not_valid(status):
    assert normalized(native(tracking_status=status)).pose_valid is False


def test_firmware_flagged_full_tracking_status_is_valid():
    assert normalized(native(tracking_status=0x12)).pose_valid is True


def test_unknown_fourth_angular_value_may_be_nonfinite():
    value = normalized(
        native(
            angular_velocity=(1.0, 2.0, 3.0, float("nan")),
            tracking_status=0x12,
        )
    )

    assert value.pose_valid is True
    assert value.angular_velocity_xyz == (1.0, 2.0, 3.0)


@pytest.mark.parametrize("norm", [0.899, 1.101])
def test_quaternion_norm_outside_tolerance_is_not_valid(norm):
    assert normalized(
        native(quaternion=(norm, 0.0, 0.0, 0.0))
    ).pose_valid is False


@pytest.mark.parametrize("norm", [0.90, 1.10])
def test_quaternion_norm_tolerance_boundaries_are_valid(norm):
    assert normalized(
        native(quaternion=(norm, 0.0, 0.0, 0.0))
    ).pose_valid is True


def test_realtime_nanoseconds_split_exactly():
    assert stamp_from_realtime_ns(1_234_567_890) == (1, 234_567_890)


def test_unknown_fourth_angular_value_is_not_published_as_an_axis():
    sample = normalized(
        native(angular_velocity=(1.0, 2.0, 3.0, 99.0))
    )
    assert sample.angular_velocity_xyz == (1.0, 2.0, 3.0)


def test_normalized_sample_is_frozen_and_preserves_audit_fields():
    sample = normalized()
    assert sample.packet_index == 7
    assert sample.tracking_status == 2
    assert sample.raw_buttons == 0
    assert sample.host_monotonic_ns == 10
    assert sample.host_realtime_ns == 20
    with pytest.raises(FrozenInstanceError):
        sample.pose_valid = False


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("packet_index", False),
        ("tracker_index", False),
        ("buttons", False),
        ("tracking_status", False),
    ],
)
def test_native_integer_fields_reject_booleans(field, value):
    with pytest.raises(TypeError, match=field):
        native(**{field: value})


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("packet_index", -1),
        ("packet_index", 0x10000),
        ("tracker_index", -1),
        ("tracker_index", 0x100),
        ("buttons", -1),
        ("buttons", 0x100),
        ("tracking_status", -1),
        ("tracking_status", 0x100),
    ],
)
def test_native_unsigned_fields_are_range_checked(field, value):
    with pytest.raises(ValueError, match=field):
        native(**{field: value})


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("position", (1.0, 2.0)),
        ("quaternion_wzyx", (1.0, 0.0, 0.0)),
        ("acceleration", (1.0, 2.0, 3.0, 4.0)),
        ("angular_velocity_native", (1.0, 2.0, 3.0)),
    ],
)
def test_native_vectors_require_exact_lengths(field, value):
    with pytest.raises(ValueError, match="exactly"):
        native(**{field: value})


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("position", (math.nan, 0.0, 0.0)),
        ("quaternion_wzyx", (math.inf, 0.0, 0.0, 0.0)),
        ("acceleration", (0.0, -math.inf, 0.0)),
        ("angular_velocity_native", (0.0, 0.0, math.nan, 0.0)),
    ],
)
def test_native_required_components_must_be_finite(field, value):
    with pytest.raises(ValueError, match="finite"):
        native(**{field: value})


@pytest.mark.parametrize("value", [False, -1])
def test_timestamp_conversion_rejects_boolean_or_negative(value):
    expected = TypeError if value is False else ValueError
    with pytest.raises(expected):
        stamp_from_realtime_ns(value)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("role", "head"),
        ("tracker_id", "A" * 64),
        ("host_monotonic_ns", False),
        ("host_realtime_ns", -1),
    ],
)
def test_normalize_rejects_invalid_metadata(field, value):
    with pytest.raises((TypeError, ValueError, KeyError)):
        normalized(**{field: value})
