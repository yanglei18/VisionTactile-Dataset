"""Guided calibration is explicit v6; legacy capture contracts remain intact."""
from copy import deepcopy

import pytest

from vt_capture_core import PlanControlSnapshot, ProtocolError, decode_message, encode_message


def start_params():
    return dict(expected_manager_instance_id="manager", mode="guided", camera_name="d405_1",
                tracker_role="torso", config_path="", bag_paths=[], board=dict(
                    square_length_mm=40., marker_length_mm=30., squares_x=9, squares_y=6,
                    dictionary="DICT_5X5_1000"))


def request(command="calibration.start", params=None, version=6):
    return dict(version=version, request_id="request", command=command,
                params=start_params() if params is None else params)


def test_calibration_start_roundtrip_and_legacy_rejection():
    value = request()
    assert decode_message(encode_message(value)) == value
    for version in range(1, 6):
        with pytest.raises(ProtocolError):
            encode_message(dict(value, version=version))


@pytest.mark.parametrize("changes", [
    dict(board={}), dict(mode="automatic-shell"), dict(camera_name="../camera"),
    dict(tracker_role="other"), dict(bag_paths=["relative/bag"]),
    dict(config_path="relative/config.yaml"), dict(bag_paths=["/bag"]),
])
def test_start_rejects_invalid_contract(changes):
    with pytest.raises(ProtocolError):
        encode_message(request(params=start_params() | changes))


@pytest.mark.parametrize("count", [1, 3])
def test_offline_accepts_explicit_bag_count(count):
    params = start_params() | dict(mode="offline", bag_paths=[f"/bag/{i}" for i in range(count)])
    assert decode_message(encode_message(request(params=params)))["params"] == params


def test_v6_response_requires_calibration_branch_and_capability():
    from vt_capture_core.calibration_protocol import empty_calibration
    value = PlanControlSnapshot("live", "manager").to_dict()
    value["capabilities"]["calibration"] = False
    value["calibration"] = empty_calibration()
    response = dict(version=6, request_id="request", ok=True, snapshot=value, operation=None)
    assert decode_message(encode_message(response)) == response
    invalid = deepcopy(response)
    invalid["snapshot"].pop("calibration")
    with pytest.raises(ProtocolError):
        encode_message(invalid)
    with pytest.raises(ProtocolError):
        encode_message(dict(response, version=5))


def test_cancel_requires_session_and_manager_identity():
    value = request("calibration.cancel", dict(session_id="session", expected_manager_instance_id="manager"))
    assert decode_message(encode_message(value)) == value
    value["params"].pop("session_id")
    with pytest.raises(ProtocolError):
        encode_message(value)


def test_v6_does_not_downgrade_capture_start():
    from test_plan_protocol import planned_start
    value = request("capture.start", planned_start())
    assert decode_message(encode_message(value)) == value
    value["params"].pop("recipe")
    with pytest.raises(ProtocolError):
        encode_message(value)


@pytest.mark.parametrize("field,value", [("cameras", [{}]), ("trackers", [{}])])
def test_catalog_requires_usable_device_identity(field, value):
    from vt_capture_core.calibration_protocol import empty_calibration, validate_calibration_snapshot
    with pytest.raises(ProtocolError):
        validate_calibration_snapshot(empty_calibration() | {field: value})


def test_session_result_is_a_structured_reference():
    from vt_capture_core.calibration_protocol import empty_calibration, validate_calibration_snapshot
    value = empty_calibration() | dict(session=dict(session_id="session", state="READY", result="arbitrary"))
    with pytest.raises(ProtocolError):
        validate_calibration_snapshot(value)


def test_restore_requires_both_history_and_current_revision():
    params = dict(expected_manager_instance_id="manager", camera_name="d436",
                  revision="a"*32, expected_active_revision="b"*32)
    value = request("calibration.restore", params)
    assert decode_message(encode_message(value)) == value
    for key in ("revision", "expected_active_revision"):
        missing = deepcopy(value)
        missing["params"].pop(key)
        with pytest.raises(ProtocolError):
            encode_message(missing)


def test_v6_typed_models_use_the_same_wire_contract():
    from vt_capture_core import CalibrationControlRequest, CalibrationControlSnapshot, CalibrationControlResponse
    outgoing = CalibrationControlRequest("request", "calibration.start", start_params())
    assert decode_message(encode_message(outgoing)) == request()
    snapshot = CalibrationControlSnapshot("live", "manager")
    response = CalibrationControlResponse("request", snapshot)
    assert decode_message(encode_message(response))["snapshot"]["calibration"]["session"] is None
