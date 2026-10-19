import json

import pytest

from vt_capture_core import (
    MAX_MESSAGE_BYTES, PROTOCOL_VERSION, Capabilities, CaptureSnapshot, DeviceSnapshot,
    EventSnapshot, ProtocolError, RecordingSnapshot, Request, ResourceSnapshot,
    Response, decode_message, encode_message,
)


@pytest.fixture
def response():
    return Response(
        request_id="request-1",
        snapshot=CaptureSnapshot(
            mode="live", instance_id="manager-instance", sequence=7,
            recording=RecordingSnapshot(state="IDLE", detail="就绪"),
            devices=[DeviceSnapshot(
                id="glove:left", label="左手", kind="glove", side="left",
                topic="/gloves/left/state", status="online", receive_hz=60.0,
                valid_hz=58.0, age_s=0.01, preview={"demo": False, "phase": 0.5},
            )],
            events=[EventSnapshot("startup", "info", "Ready")],
            resources=ResourceSnapshot(123456789), capabilities=Capabilities(),
        ),
    )


@pytest.mark.parametrize("command", ["snapshot", "ping"])
def test_request_roundtrip(command):
    request = Request("request-1", command)
    assert PROTOCOL_VERSION == 1
    wire = encode_message(request)
    assert type(wire) is bytes
    assert wire.endswith(b"\n") and wire.count(b"\n") == 1
    for frame in (wire, wire[:-1], wire[:-1] + b"\r\n", wire.decode()):
        assert decode_message(frame) == request.to_dict()


def test_response_roundtrip_and_unknown_vs_zero(response):
    assert decode_message(encode_message(response)) == response.to_dict()
    unknown = DeviceSnapshot("camera:d405_1", "D405", "camera").to_dict()
    assert unknown["receive_hz"] == 0
    assert unknown["valid_hz"] is None
    assert unknown["age_s"] is None
    assert ResourceSnapshot().to_dict()["disk_free_bytes"] is None
    assert ResourceSnapshot(0).to_dict()["disk_free_bytes"] == 0


def test_error_response_still_has_snapshot(response):
    failure = Response("invalid-request", response.snapshot, ok=False, error="unknown command")
    result = decode_message(encode_message(failure))
    assert result["ok"] is False
    assert result["error"] == "unknown command"
    assert result["snapshot"] == response.snapshot.to_dict()


def test_to_dict_is_detached(response):
    result = response.to_dict()
    result["snapshot"]["devices"][0]["preview"]["phase"] = 100
    assert response.snapshot.devices[0].preview["phase"] == 0.5


@pytest.mark.parametrize("message", [
    None, [], "string", {},
    {"version": 1, "request_id": "x", "command": "start"},
    {"version": 1, "request_id": "x", "command": "stop"},
    {"version": 7, "request_id": "x", "command": "ping"},
    {"version": True, "request_id": "x", "command": "ping"},
    {"version": 1.0, "request_id": "x", "command": "ping"},
    {"version": 1, "request_id": "", "command": "ping"},
    {"version": 1, "request_id": "  ", "command": "ping"},
    {"version": 1, "request_id": 5, "command": "ping"},
    {"version": 1, "request_id": "x", "command": "ping", "extra": 1},
    {"version": 1, "request_id": "x", "ok": False, "error": "missing snapshot"},
])
def test_invalid_message_rejected_on_encode_and_decode(message):
    with pytest.raises(ProtocolError):
        encode_message(message)
    with pytest.raises(ProtocolError):
        decode_message(json.dumps(message))


@pytest.mark.parametrize("path,value", [
    (("ok",), 1),
    (("snapshot", "mode"), "pretend-live"),
    (("snapshot", "instance_id"), ""),
    (("snapshot", "sequence"), True),
    (("snapshot", "sequence"), -1),
    (("snapshot", "recording", "state"), "ERROR"),
    (("snapshot", "recording", "elapsed_s"), -0.01),
    (("snapshot", "recording", "duration_s"), float("inf")),
    (("snapshot", "recording", "duration_s"), 10 ** 500),
    (("snapshot", "devices", 0, "kind"), "unknown-kind"),
    (("snapshot", "devices", 0, "status"), "connected"),
    (("snapshot", "devices", 0, "receive_hz"), True),
    (("snapshot", "devices", 0, "receive_hz"), float("nan")),
    (("snapshot", "devices", 0, "valid_hz"), -1),
    (("snapshot", "devices", 0, "age_s"), float("-inf")),
    (("snapshot", "devices", 0, "preview"), []),
    (("snapshot", "devices", 0, "preview"), {"phase": float("nan")}),
    (("snapshot", "resources", "disk_free_bytes"), True),
    (("snapshot", "resources", "disk_free_bytes"), 2.5),
    (("snapshot", "resources", "disk_free_bytes"), -1),
    (("snapshot", "capabilities", "record"), "false"),
    (("snapshot", "events", 0, "level"), "debug"),
    (("snapshot", "events", 0, "id"), ""),
])
def test_nested_schema_rejected(response, path, value):
    message = response.to_dict()
    target = message
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ProtocolError):
        encode_message(message)
    with pytest.raises(ProtocolError):
        decode_message(json.dumps(message))


@pytest.mark.parametrize("field", ["devices", "events"])
def test_duplicate_identity_rejected(response, field):
    message = response.to_dict()
    message["snapshot"][field] *= 2
    with pytest.raises(ProtocolError, match="duplicate id"):
        encode_message(message)


def test_extra_and_missing_nested_fields_rejected(response):
    message = response.to_dict()
    del message["snapshot"]["resources"]["disk_free_bytes"]
    with pytest.raises(ProtocolError):
        encode_message(message)
    message = response.to_dict()
    message["snapshot"]["recording"]["free_bytes"] = 0
    with pytest.raises(ProtocolError):
        encode_message(message)


@pytest.mark.parametrize("payload", [
    b"", b"\xff", b"\xef\xbb\xbf{}", b"{}\n{}\n", b"{}\n\n", b"{}\r", b"{\n}",
    b'{"version":1,"version":1,"request_id":"x","command":"ping"}',
    b'{"version":1,"request_id":"x","command":"ping","extra":NaN}',
    b'{"version":1,"request_id":"x","command":"ping","extra":Infinity}',
    b'{"version":1,"request_id":"x","command":"ping","extra":1e999}',
    b'{"version":1,"request_id":"\\ud800","command":"ping"}',
    "\ud800", None, bytearray(b"{}"),
])
def test_malformed_wire_rejected(payload):
    with pytest.raises(ProtocolError):
        decode_message(payload)


def test_duplicate_nested_key_rejected(response):
    raw = encode_message(response).replace(b'"demo":false', b'"demo":false,"demo":true')
    with pytest.raises(ProtocolError, match="duplicate JSON key"):
        decode_message(raw)


def test_escaped_newline_is_still_one_frame(response):
    message = response.to_dict()
    message["snapshot"]["recording"]["detail"] = "one\ntwo\rthree"
    assert decode_message(encode_message(message)) == message


def test_size_boundary_counts_utf8_and_lf():
    request = Request("x").to_dict()
    baseline = len(encode_message(request))
    request["request_id"] = "x" * (MAX_MESSAGE_BYTES - baseline + 1)
    wire = encode_message(request)
    assert len(wire) == MAX_MESSAGE_BYTES
    assert decode_message(wire) == request
    request["request_id"] += "x"
    with pytest.raises(ProtocolError, match="exceeds"):
        encode_message(request)
    with pytest.raises(ProtocolError, match="exceeds"):
        decode_message(wire + b" ")
    request["request_id"] = "左" * (MAX_MESSAGE_BYTES // 2)
    with pytest.raises(ProtocolError, match="exceeds"):
        encode_message(request)


def test_preview_limits_and_non_json_values(response):
    for preview in ({"bad": object()}, {"bad": (1, 2)}, {1: "not a string key"}):
        message = response.to_dict()
        message["snapshot"]["devices"][0]["preview"] = preview
        with pytest.raises(ProtocolError):
            encode_message(message)
    nested = {}
    for _ in range(40):
        nested = {"nested": nested}
    message = response.to_dict()
    message["snapshot"]["devices"][0]["preview"] = nested
    with pytest.raises(ProtocolError, match="depth"):
        encode_message(message)
    with pytest.raises(ProtocolError, match="depth"):
        decode_message(json.dumps(message))
    recursive = {}
    recursive["self"] = recursive
    message["snapshot"]["devices"][0]["preview"] = recursive
    with pytest.raises(ProtocolError):
        encode_message(message)


def test_default_models_validate():
    assert CaptureSnapshot("demo", "demo-session").to_dict()["capabilities"] == {
        "record": False, "device_control": False,
    }
    with pytest.raises(ProtocolError):
        DeviceSnapshot("left", "Left", "glove", age_s=float("nan")).to_dict()
