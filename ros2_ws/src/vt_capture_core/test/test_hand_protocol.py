"""Version 7 adds bounded hand control without changing legacy snapshots."""
from copy import deepcopy

import pytest

from vt_capture_core import PlanControlSnapshot, ProtocolError, decode_message, encode_message
from vt_capture_core.calibration_protocol import empty_calibration


def params(**changes):
    return dict(hand="right", expected_manager_instance_id="manager", expected_service_instance_id="service",
        expected_session_id=None, expected_profile_version=None, name="capture", args={"step": "neutral"}) | changes


def command(**changes):
    return dict(version=7, request_id="one", command="hand.command", params=params()) | changes


def row(**changes):
    return dict(hand="right", group_id="right_glove", control_socket="/private/right.sock",
        tactile_socket="/private/tactile.sock", resources="/assets", port="/serial", glove_python="/env/python", tactile_port="/tactile",
        service_instance_id="service", service_state="ready", connected=False, raw_fresh=False,
        calibration_ready=False, pose_valid=False, session_id="", profile_version=0,
        busy=False, age_s=.01, detail="No calibration yet") | changes


def response(version=7):
    snapshot = PlanControlSnapshot("live", "manager").to_dict()
    snapshot["capabilities"]["calibration"] = False
    snapshot["calibration"] = empty_calibration()
    if version >= 7:
        snapshot["capabilities"]["hand_control"] = True
        snapshot["hand_sessions"] = {"right": row()}
    return dict(version=version, request_id="one", ok=True, snapshot=snapshot, operation=None)


def test_v7_hand_command_keeps_null_uncalibrated_binding():
    value = command()
    assert decode_message(encode_message(value)) == value
    assert value["params"]["expected_profile_version"] is None


@pytest.mark.parametrize("version", range(1, 7))
def test_legacy_versions_do_not_accept_hand_commands(version):
    with pytest.raises(ProtocolError):
        encode_message(command(version=version))


@pytest.mark.parametrize("changes", [
    dict(name="connect", args={"port": "/serial"}), dict(name="disconnect", args={}),
    dict(name="capture_shape_contact", args={"category": 2, "contact_confirmed": False}),
    dict(name="capture_shape_contact", args={"category": 6, "contact_confirmed": True}),
    dict(name="capture", args={"step": "automatic"}), dict(hand="both"),
    dict(expected_manager_instance_id=""), dict(expected_service_instance_id=""),
    dict(expected_profile_version=True), dict(expected_profile_version=-1),
    dict(name="set_functional_enabled", args={"enabled": 1}),
])
def test_v7_rejects_unbound_or_device_ownership_commands(changes):
    with pytest.raises(ProtocolError):
        encode_message(command(params=params(**changes)))


@pytest.mark.parametrize("name,args", [
    ("capture_functional_hand", {}), ("capture_functional", {"finger": "thumb"}),
    ("capture_shape_contact", {"category": 2, "contact_confirmed": True}),
    ("preview_shape", {"mode": "candidate"}), ("retry_calibration_save_to", {"directory": "/profiles"}),
])
def test_v7_supported_operation_arguments_round_trip(name, args):
    value = command(params=params(name=name, args=args, expected_session_id="session", expected_profile_version=2))
    assert decode_message(encode_message(value)) == value


def test_v7_ready_service_without_calibration_is_valid_snapshot():
    value = response()
    assert decode_message(encode_message(value)) == value
    for field in ("hand_sessions", "capabilities"):
        invalid = deepcopy(value)
        invalid["snapshot"].pop(field)
        with pytest.raises(ProtocolError):
            encode_message(invalid)


@pytest.mark.parametrize("changes", [dict(hand="left"), dict(raw_fresh=1), dict(profile_version=-1),
    dict(age_s=float("nan")), dict(service_state="recording"), dict(pose_valid="true")])
def test_v7_rejects_inconsistent_hand_snapshot(changes):
    value = response()
    value["snapshot"]["hand_sessions"]["right"].update(changes)
    with pytest.raises(ProtocolError):
        encode_message(value)


def test_v6_snapshot_remains_valid_without_v7_hand_fields():
    value = response(6)
    assert decode_message(encode_message(value)) == value
    with pytest.raises(ProtocolError):
        encode_message(dict(response(), version=6))
