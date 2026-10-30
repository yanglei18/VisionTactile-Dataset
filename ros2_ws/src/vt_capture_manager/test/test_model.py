from types import SimpleNamespace as NS
import sys

import pytest

from vt_capture_core import encode_message
from vt_capture_manager.config import DeviceConfig, ManagerConfig, load_config
from vt_capture_manager.model import SnapshotModel


class Clock:
    ns = 10_000_000_000
    def __call__(self):
        return self.ns


def message(kind, clock, **changes):
    header = NS(frame_id="glove_right_reference", stamp=NS(sec=clock.ns//10**9, nanosec=clock.ns%10**9))
    if kind == "glove":
        fields = dict(header=header, schema_version=1, hand="right", host_realtime_ns=clock.ns,
            pose_valid=True, time_valid=True, joint_positions_m=[.01]*63, mano_pose_rotvec=[0.]*48,
            shape_params=[0.]*10, root_quat_xyzw=[0., 0., 0., 1.], quality="valid", reason="",
            calibration_version=1, session_id="one", config_sha256="a", model_rest_sha256="b", frame_index=1,
            source_mode="live")
    elif kind == "tracker":
        header.frame_id = "vive_map"
        fields = dict(header=header, role="right_wrist", tracker_id="known", host_realtime_ns=clock.ns,
            tracking_status=2, pose_valid=True, packet_index=1,
            pose=NS(position=NS(x=1., y=2., z=3.), orientation=NS(x=0., y=0., z=0., w=1.)))
    elif kind == "tactile":
        fields = dict(header=header, schema_version=2, hand="right", rows=20, cols=20, adc=[0]*400,
            valid_mask=[True]*400, block_realtime_ns=[clock.ns]*160, host_realtime_ns=clock.ns,
            max_age_ns=500_000_000, connected=True, time_valid=True, reason="ok", session_id="one",
            stream_epoch=1, source_sequence=1)
    else:
        fields = dict(header=header, width=2, height=2, step=6, data=bytes(12), encoding="rgb8", is_bigendian=0)
    fields.update(changes)
    return NS(**fields)


def make(kind="glove"):
    clock = Clock()
    identity = "right_wrist" if kind == "tracker" else "one"
    config = ManagerConfig((DeviceConfig(identity, identity, kind, "right", "/test/state"),))
    model = SnapshotModel(config, monotonic_ns=clock, realtime_ns=clock,
                          disk_usage=lambda _: NS(free=100), instance_id="test")
    return model, clock, identity


def test_tracker_preview_uses_explicit_source_frame_not_hardcoded_pyvut():
    clock = Clock()
    config = ManagerConfig((DeviceConfig("right_wrist", "Right", "tracker", "right", "/vive/right_wrist/sample"),),
                           process_groups=({"preset": "trackers", "parameters": {"frame_id": "vive_hub_map"}},))
    model = SnapshotModel(config, monotonic_ns=clock, realtime_ns=clock, disk_usage=lambda _: NS(free=100))
    msg = message("tracker", clock)
    model.observe("right_wrist", msg)
    assert not model.snapshot()["devices"][0]["preview"]
    msg.header.frame_id = "vive_hub_map"
    model.observe("right_wrist", msg)
    assert model.snapshot()["devices"][0]["preview"]["valid"]


def test_readonly_tracker_uses_explicit_scene_world_frame(tmp_path):
    path=tmp_path/'scene.json'
    path.write_text('{"schema_version":1,"world_frame":"vive_hub_map"}')
    clock=Clock()
    config=ManagerConfig((DeviceConfig('right_wrist','Right','tracker','right','/vive/right_wrist/sample'),),
                         scene_config_path=str(path))
    model=SnapshotModel(config,monotonic_ns=clock,realtime_ns=clock,disk_usage=lambda _:NS(free=100))
    msg=message('tracker',clock);msg.header.frame_id='vive_hub_map'
    model.observe('right_wrist',msg)
    assert model.snapshot()['devices'][0]['preview']['valid']
    from dataclasses import replace
    incompatible=replace(config,process_groups=({'preset':'trackers','parameters':{'frame_id':'vive_map'}},))
    with pytest.raises(ValueError,match='world frame'): SnapshotModel(incompatible)


def test_glove_preview_preserves_per_frame_sources_and_clears_unknown_metadata():
    model, clock, identity = make()
    sources = dict(reference_node_id=11,
        segment_observed_mask=[i not in (2, 5, 8, 11, 13) for i in range(16)],
        joint_observed_mask=[i in (0, 1, 4, 7, 10, 15) for i in range(16)],
        inferred_mask=[i not in (0, 1, 4, 7, 10, 15) for i in range(16)])
    msg = message("glove", clock, **sources)
    model.observe(identity, msg)
    preview = model.snapshot()["devices"][0]["preview"]
    assert preview["valid"] and preview["glove_sources"] == sources
    msg.segment_observed_mask[0] = False
    assert preview["glove_sources"]["segment_observed_mask"][0]
    clock.ns += 1_000_000
    model.observe(identity, message("glove", clock))
    assert "glove_sources" not in model.snapshot()["devices"][0]["preview"]


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_unknown_fresh_stale_offline_and_protocol(kind):
    model, clock, identity = make(kind)
    assert model.snapshot()["devices"][0]["status"] == "unknown"
    model.observe(identity, message(kind, clock))
    value = model.snapshot()
    assert value["devices"][0]["status"] == "online"
    assert value["capabilities"] == dict(record=False, device_control=False)
    encode_message(dict(version=1, request_id="test", ok=True, snapshot=value))
    clock.ns += 2_000_000_000
    value = model.snapshot()["devices"][0]
    assert value["status"] == "stale"
    assert value["receive_hz"] == 0.
    if kind in {"glove", "tracker"}:
        assert value["preview"]["valid"] is False
    if kind == "tactile":
        assert not any(value["preview"]["valid_mask"])
    clock.ns += 4_000_000_000
    assert model.snapshot()["devices"][0]["status"] == "offline"


@pytest.mark.parametrize("kind", ["glove", "tracker"])
def test_invalid_pose_is_not_counted_and_duplicate_not_counted(kind):
    model, clock, identity = make(kind)
    msg = message(kind, clock)
    model.observe(identity, msg)
    model.observe(identity, msg)
    model.observe(identity, message(kind, clock, pose_valid=False))
    result = model.snapshot()["devices"][0]
    assert result["receive_hz"] == 3.
    assert result["valid_hz"] == 1.
    assert result["preview"]["valid"] is False


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_process_readiness_uses_latest_sample_not_rate_window(kind):
    model, clock, identity = make(kind)
    assert not model.topic_health()["/test/state"]
    model.observe(identity, message(kind, clock))
    assert model.topic_health()["/test/state"]
    if kind == "camera":
        assert model.snapshot()["devices"][0]["valid_hz"] is None
    if kind in {"glove", "tracker"}:
        model.observe(identity, message(kind, clock, pose_valid=False))
        assert model.snapshot()["devices"][0]["valid_hz"] > 0
        assert not model.topic_health()["/test/state"]
    clock.ns += 2_000_000_000
    assert not model.topic_health()["/test/state"]


def test_process_controller_topic_health_does_not_depend_on_recording_phase():
    model, clock, _ = make()
    for state in (0, 1, 2, 3, 4, 5, 6):
        model.observe_capture(NS(header=NS(stamp=NS(sec=10, nanosec=0)), state=state,
                                 session_id="x", detail="", recorder_alive=state == 3, disk_free_bytes=100))
        assert model.topic_health()[model.config.capture_status_topic]
    clock.ns += 2_000_000_000
    assert not model.topic_health()[model.config.capture_status_topic]


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove"])
def test_recording_activity_tracks_arrival_independently_of_sample_quality(kind):
    model, clock, identity = make(kind)
    assert model.topic_activity() == {"/test/state": False}
    model.observe(identity, message(kind, clock))
    assert model.topic_activity() == {"/test/state": True}
    # Even malformed data is recordable evidence. Preview validity must not
    # silently remove a receiving stream from recording quality expectations.
    model.observe(identity, NS())
    assert not model.topic_health()["/test/state"]
    assert model.topic_activity()["/test/state"]
    clock.ns += 2_000_000_000
    assert not model.topic_activity()["/test/state"]


def test_tactile_activity_requires_raw_arrival_even_while_state_heartbeats_continue():
    model, clock, identity = make("tactile")
    assert model.sample_activity_topics() == ("/test/samples",)
    assert model.topic_activity() == {"/test/samples": False}
    model.observe(identity, message("tactile", clock))
    assert model.snapshot()["devices"][0]["status"] == "online"
    assert model.topic_activity() == {"/test/samples": False}
    # Raw arrival remains recordable without parsing or requiring valid cells.
    model.observe_topic_activity("/test/samples")
    model.observe(identity, NS())
    assert not model.topic_health()["/test/state"]
    assert model.topic_activity() == {"/test/samples": True}
    clock.ns += 2_000_000_000
    model.observe(identity, message("tactile", clock, connected=False, time_valid=False))
    assert model.snapshot()["devices"][0]["status"] == "offline"
    assert model.topic_activity() == {"/test/samples": False}


def test_raw_activity_observers_are_bounded_by_explicit_tactile_configuration():
    devices = (DeviceConfig("right", "right", "tactile", "right", "/right/tactile/state"),
               DeviceConfig("same", "same", "tactile", "right", "/right/tactile/state"),
               DeviceConfig("custom", "custom", "tactile", "left", "/custom_tactile"),
               DeviceConfig("camera", "camera", "camera", "", "/camera/state"))
    model = SnapshotModel(ManagerConfig(devices))
    assert model.sample_activity_topics() == ("/right/tactile/samples",)
    with pytest.raises(ValueError, match="not configured"):
        model.observe_topic_activity("/unconfigured/samples")
    assert model.topic_activity() == {"/right/tactile/samples": False, "/camera/state": False}


@pytest.mark.parametrize("changes", [dict(hand="left"), dict(joint_positions_m=[0.]*62),
    dict(joint_positions_m=[float("nan")]*63), dict(mano_pose_rotvec=[]), dict(schema_version=2)])
def test_malformed_glove_rejected(changes):
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, **changes))
    value = model.snapshot()
    assert value["devices"][0]["valid_hz"] == 0
    assert value["devices"][0]["preview"] == {}
    assert value["events"][0]["level"] == "warning"


def test_valid_glove_can_have_no_joints_but_real_mano_without_fake_neutral():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, joint_positions_m=[]))
    result = model.snapshot()["devices"][0]
    assert result["valid_hz"] == 1.
    assert result["preview"]["joints"] == []
    assert result["preview"]["valid"] is True
    assert result["preview"]["mano"]["pose"] == [0.] * 48
    assert result["preview"]["quality"] == "valid"


def test_legacy_glove_without_source_mode_preserves_skeleton_only_contract():
    model, clock, identity = make()
    msg = message("glove", clock, joint_positions_m=[])
    del msg.source_mode
    model.observe(identity, msg)
    assert model.snapshot()["devices"][0]["preview"] == dict(joints=[], valid=False, quality="valid", reason="")


def test_mano_preview_preserves_source_pose_shape_and_identity_without_second_root_rotation():
    model, clock, identity = make()
    pose = [index / 30. - .7 for index in range(48)]
    shape = [index / 7. - .4 for index in range(10)]
    msg = message("glove", clock, mano_pose_rotvec=pose, shape_params=shape,
                  root_quat_xyzw=[0., 0., 1., 0.], session_id="original-session", frame_index=123,
                  calibration_version=9, config_sha256="c" * 64, model_rest_sha256="d" * 64)
    model.observe(identity, msg)
    snapshot = model.snapshot()
    preview = snapshot["devices"][0]["preview"]
    assert preview["mano"] == dict(pose=pose, shape=shape, hand="right", session_id="original-session",
        frame_index=123, calibration_version=9, config_sha256="c" * 64, model_rest_sha256="d" * 64,
        source_realtime_ns=clock.ns, source_mode="live")
    assert preview["joints"] == [[.01] * 3] * 21
    assert "root_quat_xyzw" not in preview["mano"] and "replay" not in preview
    encode_message(dict(version=1, request_id="mano", ok=True, snapshot=snapshot))
    pose[0] = 999.
    shape[0] = 999.
    preview["mano"]["pose"][1] = 999.
    cached = model.snapshot()["devices"][0]["preview"]["mano"]
    assert cached["pose"][:2] == [-.7, 1 / 30. - .7]
    assert cached["shape"][0] == -.4


def test_left_mano_is_forwarded_as_left_without_mirroring_or_root_transform():
    clock = Clock()
    config = ManagerConfig((DeviceConfig("left_glove", "left", "glove", "left", "/test/left/state"),))
    model = SnapshotModel(config, monotonic_ns=clock, realtime_ns=clock, disk_usage=lambda _: NS(free=100))
    msg = message("glove", clock, hand="left", mano_pose_rotvec=[.125] * 48)
    msg.header.frame_id = "glove_left_reference"
    model.observe("left_glove", msg)
    preview = model.snapshot()["devices"][0]["preview"]
    assert preview["valid"] and preview["mano"]["hand"] == "left"
    assert preview["mano"]["pose"] == [.125] * 48


@pytest.mark.parametrize("changes", [dict(pose_valid=False), dict(time_valid=False),
    dict(host_realtime_ns=0), dict(host_realtime_ns=1), dict(host_realtime_ns=99_000_000_000)])
def test_invalid_or_unusable_time_never_exports_current_mano(changes):
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, quality="unavailable", reason="input_not_current", **changes))
    preview = model.snapshot()["devices"][0]["preview"]
    assert not preview["valid"] and "mano" not in preview
    assert preview["quality"] == "unavailable" and preview["reason"] == "input_not_current"


def test_future_within_legacy_clock_tolerance_still_has_no_current_mano():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, host_realtime_ns=clock.ns + 1))
    assert "mano" not in model.snapshot()["devices"][0]["preview"]


def test_mano_is_removed_when_source_ages_without_replacing_it_by_a_neutral_pose():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, mano_pose_rotvec=[.2] * 48))
    assert "mano" in model.snapshot()["devices"][0]["preview"]
    clock.ns += 2_000_000_000
    preview = model.snapshot()["devices"][0]["preview"]
    assert not preview["valid"] and "mano" not in preview
    assert len(preview["joints"]) == 21  # Legacy geometry may be explicitly held.


def test_wall_clock_adjustment_cannot_make_a_cached_future_mano_current():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock))
    assert "mano" in model.snapshot()["devices"][0]["preview"]
    model._wall = lambda: clock.ns - 100_000_000
    assert "mano" not in model.snapshot()["devices"][0]["preview"]


@pytest.mark.parametrize("valid", [True, False])
def test_replay_is_explicit_on_both_mano_and_legacy_preview(valid):
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, source_mode="replay", pose_valid=valid))
    preview = model.snapshot()["devices"][0]["preview"]
    assert preview["replay"] is True and preview["valid"] is valid
    assert ("mano" in preview) is valid
    if valid:
        assert preview["mano"]["source_mode"] == "replay"


@pytest.mark.parametrize("source_mode", ["", "unknown", "demo"])
def test_unknown_source_mode_does_not_fabricate_live_mano(source_mode):
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, source_mode=source_mode))
    preview = model.snapshot()["devices"][0]["preview"]
    assert "mano" not in preview and "replay" not in preview


@pytest.mark.parametrize("changes", [dict(mano_pose_rotvec=[float("nan")] * 48),
    dict(shape_params=[float("inf")] * 10), dict(shape_params=[0.] * 9),
    dict(session_id=""), dict(session_id="x" * 257), dict(config_sha256="x" * 129),
    dict(model_rest_sha256=""), dict(frame_index=-1), dict(frame_index=True),
    dict(calibration_version=2**32)])
def test_malformed_mano_preview_never_survives(changes):
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, **changes))
    result = model.snapshot()["devices"][0]
    assert result["preview"] == {} and result["valid_hz"] == 0.


def test_session_and_calibration_change_replace_all_mano_provenance_together():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock))
    clock.ns += 10_000_000
    model.observe(identity, message("glove", clock, session_id="two", calibration_version=2,
                  frame_index=4, config_sha256="c", model_rest_sha256="d", mano_pose_rotvec=[.3] * 48))
    preview = model.snapshot()["devices"][0]["preview"]["mano"]
    assert preview["session_id"] == "two" and preview["calibration_version"] == 2
    assert preview["config_sha256"] == "c" and preview["model_rest_sha256"] == "d"
    assert preview["frame_index"] == 4 and preview["pose"] == [.3] * 48
    assert preview["source_realtime_ns"] == clock.ns


@pytest.mark.parametrize("stamp", [0, 1, 99_000_000_000])
def test_stale_or_missing_source_not_fresh_from_arrival(stamp):
    model, clock, identity = make()
    model.observe(identity, message("glove", clock, host_realtime_ns=stamp))
    result = model.snapshot()["devices"][0]
    assert result["status"] == "stale"
    assert result["valid_hz"] == 0
    assert result["preview"]["valid"] is False


def test_tactile_uses_stride8_and_ages_each_cell():
    model, clock, identity = make("tactile")
    msg = message("tactile", clock)
    msg.block_realtime_ns[0] = clock.ns - 600_000_000
    model.observe(identity, msg)
    masks = model.snapshot()["devices"][0]["preview"]["valid_mask"]
    assert not masks[0] and not masks[8] and not masks[16] and masks[1]
    clock.ns += 600_000_000
    result = model.snapshot()["devices"][0]
    assert result["status"] == "online"  # Recent topic, cells have stricter TTL.
    assert not any(result["preview"]["valid_mask"])


def test_legacy_tactile_crop_not_reshape_and_zero_is_valid():
    model, clock, identity = make("tactile")
    values = list(range(6400))
    model.observe(identity, message("tactile", clock, schema_version=1, rows=80, cols=80,
        adc=values, valid_mask=[True]*6400, block_realtime_ns=[clock.ns]*640))
    result = model.snapshot()["devices"][0]["preview"]
    assert result["values"][20] == 80.
    assert result["values"][0] == 0. and result["valid_mask"][0]


def test_capture_status_is_read_only_and_ages_out():
    model, clock, _ = make()
    assert model.snapshot()["recording"]["state"] == "UNAVAILABLE"
    header = NS(stamp=NS(sec=10, nanosec=0))
    model.observe_capture(NS(header=header, state=3, session_id="external", detail="running",
                             recorder_alive=True, disk_free_bytes=55))
    result = model.snapshot()
    assert result["recording"]["state"] == "RECORDING"
    assert result["recording"]["elapsed_s"] == 0  # Not fabricated.
    assert not result["capabilities"]["record"]
    clock.ns += 6_000_000_000
    assert model.snapshot()["recording"]["state"] == "UNAVAILABLE"


def test_config_defaults_and_overrides(tmp_path):
    config = load_config()
    assert len(config.devices) == 13
    assert any(d.topic == "/d436/depth/image_rect_raw" for d in config.devices)
    assert all(d.image_reliability == "reliable" for d in config.devices if d.kind == "camera")
    path = tmp_path / "manager.json"
    path.write_text('{"devices": [], "capture_status_topic": "/custom/status"}')
    assert load_config(path).capture_status_topic == "/custom/status"
    path.write_text('{"devices": [], "unknown": true}')
    with pytest.raises(ValueError):
        load_config(path)


def test_explicit_image_subscription_preserves_sensor_data_compatibility(tmp_path):
    import json
    device = dict(id="camera", label="Camera", kind="camera", side="", topic="/custom/image")
    path = tmp_path / "manager.json"
    path.write_text(json.dumps(dict(devices=[device])))
    assert load_config(path).devices[0].image_reliability == "best_effort"
    device["image_reliability"] = "reliable"
    path.write_text(json.dumps(dict(devices=[device])))
    assert load_config(path).devices[0].image_reliability == "reliable"
    device["image_reliability"] = "auto"
    path.write_text(json.dumps(dict(devices=[device])))
    with pytest.raises(ValueError, match="image_reliability"):
        load_config(path)
    device.update(kind="tracker", image_reliability="reliable")
    path.write_text(json.dumps(dict(devices=[device])))
    with pytest.raises(ValueError, match="only applies to camera"):
        load_config(path)


def test_shipped_example_matches_defaults():
    from pathlib import Path
    path = Path(__file__).resolve().parents[1] / "config/workstation.example.yaml"
    assert load_config(path) == load_config()


def test_out_of_order_source_does_not_replace_newer_pose():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock))
    clock.ns += 20_000_000
    model.observe(identity, message("glove", clock, host_realtime_ns=clock.ns-30_000_000,
                                    joint_positions_m=[2.]*63, frame_index=0))
    result = model.snapshot()["devices"][0]
    assert result["preview"]["joints"][0] == [.01]*3
    assert result["valid_hz"] == 1.


def test_newest_events_first_and_bounded():
    model, _, _ = make()
    for index in range(40):
        model.event("info", str(index))
    values = model.snapshot()["events"]
    assert len(values) == 32
    assert values[0]["message"] == "39"


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
@pytest.mark.parametrize("previous_valid", [False, True])
def test_future_source_does_not_poison_ordering_or_valid_rate(kind, previous_valid):
    model, clock, identity = make(kind)
    if previous_valid:
        model.observe(identity, message(kind, clock))
        clock.ns += 10_000_000
    future = message(kind, clock)
    if kind == "camera":
        future.header.stamp.sec += 3600
    else:
        future.host_realtime_ns += 3_600_000_000_000
    model.observe(identity, future)
    value = model.snapshot()["devices"][0]
    assert value["status"] == "stale"
    if kind != "camera":
        assert value["valid_hz"] == float(previous_valid)
    clock.ns += 10_000_000
    model.observe(identity, message(kind, clock))
    value = model.snapshot()["devices"][0]
    assert value["status"] == "online"
    if kind != "camera":
        assert value["valid_hz"] == float(1+previous_valid)
    if kind in {"tracker", "glove"}:
        assert value["preview"]["valid"]
    if kind == "tactile":
        assert all(value["preview"]["valid_mask"])


def test_future_frame_does_not_bypass_previous_valid_watermark():
    model, clock, identity = make()
    model.observe(identity, message("glove", clock))
    model.observe(identity, message("glove", clock, host_realtime_ns=clock.ns+3_600_000_000_000))
    model.observe(identity, message("glove", clock, host_realtime_ns=clock.ns-1_000_000))
    assert model.snapshot()["devices"][0]["valid_hz"] == 1.
    clock.ns += 1_000_000
    model.observe(identity, message("glove", clock))
    assert model.snapshot()["devices"][0]["valid_hz"] == 2.


def test_future_frame_does_not_reset_duplicate_suppression():
    model, clock, identity = make()
    valid = message("glove", clock)
    model.observe(identity, valid)
    model.observe(identity, message("glove", clock, host_realtime_ns=clock.ns+3_600_000_000_000))
    model.observe(identity, valid)
    result = model.snapshot()["devices"][0]
    assert result["status"] == "online"
    assert result["valid_hz"] == 1.


@pytest.mark.parametrize("previous_valid", [False, True])
def test_capture_recovers_after_future_status(previous_valid):
    model, clock, _ = make()
    def capture(seconds, state):
        return NS(header=NS(stamp=NS(sec=seconds, nanosec=0)), state=state, session_id="external",
                  detail="", recorder_alive=True, disk_free_bytes=55)
    if previous_valid:
        model.observe_capture(capture(10, 0))
    model.observe_capture(capture(3610, 3))
    assert model.snapshot()["recording"]["state"] == ("IDLE" if previous_valid else "UNAVAILABLE")
    model.observe_capture(capture(10, 3))
    assert model.snapshot()["recording"]["state"] == "RECORDING"


@pytest.mark.parametrize("previous_valid", [False, True])
def test_camera_preview_recovers_after_future_result(previous_valid):
    model, clock, identity = make("camera")
    model.observe(identity, message("camera", clock))
    first = dict(width=1, height=1, format="jpeg", data_b64="first")
    recovered = dict(width=1, height=1, format="jpeg", data_b64="recovered")
    if previous_valid:
        model.set_camera_preview(identity, clock.ns, first)
    model.set_camera_preview(identity, clock.ns+3_600_000_000_000, first)
    clock.ns += 1_000_000
    model.set_camera_preview(identity, clock.ns, recovered)
    assert model.snapshot()["devices"][0]["preview"] == recovered


@pytest.mark.parametrize("ahead_ns", [200_000_000, 500_000_000])
def test_camera_preview_uses_the_same_future_tolerance_as_online_status(ahead_ns):
    model, clock, identity = make("camera")
    source = Clock()
    source.ns = clock.ns + ahead_ns
    preview = dict(width=2, height=2, format="jpeg", data_b64="current-preview")
    model.observe(identity, message("camera", source))
    model.set_camera_preview(identity, source.ns, preview)
    value = model.snapshot()["devices"][0]
    assert value["status"] == "online"
    assert value["age_s"] == 0.
    assert value["preview"] == preview


def test_camera_preview_beyond_future_tolerance_does_not_replace_current_frame():
    model, clock, identity = make("camera")
    current = dict(width=2, height=2, format="jpeg", data_b64="current-preview")
    rejected = dict(width=2, height=2, format="jpeg", data_b64="future-preview")
    model.observe(identity, message("camera", clock))
    model.set_camera_preview(identity, clock.ns, current)
    model.set_camera_preview(identity, clock.ns + 500_000_001, rejected)
    assert model.snapshot()["devices"][0]["preview"] == current


@pytest.mark.parametrize("receiving", [False, True])
def test_camera_preview_still_expires_with_future_tolerant_display(receiving):
    model, clock, identity = make("camera")
    preview = dict(width=2, height=2, format="jpeg", data_b64="old-preview")
    model.observe(identity, message("camera", clock))
    model.set_camera_preview(identity, clock.ns, preview)
    clock.ns += int(model.config.stale_after_s * 1e9) + 1
    if receiving:
        model.observe(identity, message("camera", clock))
    value = model.snapshot()["devices"][0]
    assert value["status"] == ("online" if receiving else "stale")
    assert value["preview"] == {}


@pytest.mark.parametrize("kind", ["camera", "tracker", "glove", "tactile"])
def test_provenance_is_message_observation_not_hardware_discovery(kind):
    model, clock, identity = make(kind)
    before = model.provenance()[0]
    assert before["identity"] is None and before["identity_status"] == "unknown"
    assert before["observed_source_realtime_ns"] is None and not before["fresh"]
    model.observe(identity, message(kind, clock))
    observed = model.provenance()[0]
    assert observed["fresh"] and observed["observed_source_realtime_ns"] == clock.ns
    if kind == "camera":
        assert observed["identity"] is None  # Image never supplied a physical serial.
    elif kind == "tracker":
        assert observed["identity"]["tracker_id"] == "known"
    elif kind == "glove":
        assert observed["identity"]["calibration_version"] == 1
        assert observed["identity"]["config_sha256"] == "a"
    else:
        assert observed["identity"]["stream_epoch"] == 1
    clock.ns += 2_000_000_000
    assert not model.provenance()[0]["fresh"]


def test_import_does_not_load_ros():
    import subprocess
    result = subprocess.run([sys.executable, "-c", "import vt_capture_manager.node; import sys; assert 'rclpy' not in sys.modules"], capture_output=True)
    assert result.returncode == 0, result.stderr
