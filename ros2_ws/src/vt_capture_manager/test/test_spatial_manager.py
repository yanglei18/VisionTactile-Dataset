"""Joint scenes retain source coordinates and cannot control hardware."""
from types import SimpleNamespace as NS
import json
import socket
import time

import pytest

from vt_capture_manager.spatial.config import load_scene_config
from vt_capture_manager.spatial.scene_store import SceneStore
from vt_capture_manager.spatial.scene_server import SceneServer


class Clock:
    ns = 10_000_000_000
    def __call__(self):
        return self.ns


def sample(clock, role="left_wrist", position=(1., 2., 3.), **kw):
    values = dict(role=role, tracker_id="a" * 64, host_realtime_ns=clock.ns,
                  host_monotonic_ns=clock.ns, tracking_status=2, pose_valid=True,
                  header=NS(frame_id="vive_map"), packet_index=1,
                  pose=NS(position=NS(**dict(zip("xyz", position))),
                          orientation=NS(x=0., y=0., z=0., w=1.)))
    values.update(kw)
    return NS(**values)


def make():
    clock = Clock()
    store = SceneStore("manager-test", load_scene_config(),
                       realtime_ns=clock, monotonic_ns=clock)
    return store, clock


def test_scene_config_rejects_bad_bounds_and_resolves_paths(tmp_path):
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(dict(schema_version=1, scene_hz=100000)))
    with pytest.raises(ValueError):
        load_scene_config(path)
    path.write_text(json.dumps(dict(schema_version=1, alignment_config_path="missing.yaml")))
    with pytest.raises((ValueError, FileNotFoundError)):
        load_scene_config(path)
    path.write_text(json.dumps(dict(schema_version=1, enabled=True, world_frame="vive_map")))
    assert load_scene_config(path).enabled


def test_preview_notice_survives_missing_data_without_changing_geometry(tmp_path):
    notice = '左腕·试用候选（最初第3轮；三轮一致性未通过）'
    path = tmp_path / 'scene.yaml'
    path.write_text(json.dumps(dict(schema_version=1, preview_notice=notice)))
    config = load_scene_config(path)
    store = SceneStore('notice-test', config)
    assert store.description()['preview_notice'] == notice
    snapshot = store.snapshot()
    assert snapshot['preview_notice'] == notice
    assert snapshot['cameras'] == []
    assert all(item['state'] == 'waiting' and not item['valid'] for item in snapshot['trackers'])
    assert not config.cloud['enabled']
    assert load_scene_config().preview_notice == ''


def test_scene_cloud_source_is_explicit_and_legacy_default_is_preserved(tmp_path):
    path=tmp_path/'scene.json'
    path.write_text(json.dumps({'cloud':{'enabled':True}}))
    assert load_scene_config(path).cloud['source']=='rgbd'
    path.write_text(json.dumps({'cloud':{'enabled':True,'source':'realsense','max_points_per_camera':2097152}}))
    assert load_scene_config(path).cloud['source']=='realsense'
    for cloud in ({'source':'guess'},{'source':'realsense','topics':[]},{'topics':{'cam':'not a topic'}}):
        path.write_text(json.dumps({'cloud':cloud}))
        with pytest.raises(ValueError):load_scene_config(path)


@pytest.mark.parametrize('notice', [None, False, 17, [], {}, 'x' * 1025])
def test_preview_notice_rejects_invalid_or_unbounded_values(tmp_path, notice):
    path = tmp_path / 'scene.yaml'
    path.write_text(json.dumps(dict(preview_notice=notice)))
    with pytest.raises(ValueError, match='preview_notice'):
        load_scene_config(path)


def test_common_positions_and_disconnection_are_independent():
    store, clock = make()
    store.observe_tracker("left_wrist", sample(clock))
    store.observe_tracker("right_wrist", sample(clock, "right_wrist", (5., 2., 3.)))
    first = store.snapshot()
    roles = {item["role"]: item for item in first["trackers"]}
    assert roles["left_wrist"]["position_m"] == [1., 2., 3.]
    assert roles["right_wrist"]["position_m"] == [5., 2., 3.]
    assert roles["torso"]["state"] == "waiting"
    clock.ns += 300_000_000
    store.observe_tracker("right_wrist", sample(clock, "right_wrist", (6., 2., 3.)))
    roles = {item["role"]: item for item in store.snapshot()["trackers"]}
    assert roles["left_wrist"]["state"] == "delayed"
    assert roles["right_wrist"]["state"] == "fresh"


def test_duplicates_never_extend_freshness_and_invalid_pose_holds_history():
    store, clock = make()
    msg = sample(clock)
    store.observe_tracker("left_wrist", msg)
    clock.ns += 1_100_000_000
    store.observe_tracker("left_wrist", msg)
    current = store.snapshot()["trackers"][0]
    assert current["state"] == "offline" and not current["valid"]
    store.observe_tracker("left_wrist", sample(clock, pose_valid=False))
    current = store.snapshot()["trackers"][0]
    assert current["state"] == "invalid"
    assert current["position_m"] == [1., 2., 3.]


def test_identity_change_clears_trajectory_and_frame_mismatch_is_rejected():
    store, clock = make()
    store.observe_tracker("left_wrist", sample(clock))
    clock.ns += 10_000_000
    store.observe_tracker("left_wrist", sample(clock, position=(2., 2., 3.)))
    before = store.snapshot()
    clock.ns += 10_000_000
    store.observe_tracker("left_wrist", sample(clock, position=(8., 0., 0.), tracker_id="b" * 64))
    result = store.snapshot()
    assert result["coordinate_epoch"] != before["coordinate_epoch"]
    assert sum(map(len, result["trackers"][0]["trail_segments"])) == 1
    clock.ns += 10_000_000
    store.observe_tracker("left_wrist", sample(clock, header=NS(frame_id="wrong")))
    assert not store.snapshot()["trackers"][0]["valid"]


def test_snapshot_is_detached_and_has_no_fictitious_shared_time():
    store, clock = make()
    store.observe_tracker("left_wrist", sample(clock))
    a = store.snapshot()
    a["trackers"][0]["position_m"][0] = 1000
    b = store.snapshot()
    assert b["trackers"][0]["position_m"][0] == 1.
    assert b["scene_time_ns"] is None and b["time_mode"] == "monitor"
    assert b["map_identity"] is None


def test_server_latest_and_control_command_rejection(tmp_path):
    from vt_capture_core.spatial.scene_protocol import encode_scene_message, decode_scene_message
    store, clock = make()
    store.observe_tracker("left_wrist", sample(clock))
    path = tmp_path / "scene.sock"
    server = SceneServer(store, path).start()
    def exchange(command, params):
        request = dict(protocol="vt.scene", version=1, request_id="test", command=command, params=params)
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(1.)
            connection.connect(str(path))
            connection.sendall(encode_scene_message(request))
            payload = b""
            while b"\n" not in payload:
                payload += connection.recv(65536)
            return decode_scene_message(payload)
    try:
        hello = exchange("hello", {})
        assert hello["hello"]["manager_instance_id"] == "manager-test"
        latest = exchange("latest", dict(after_sequence=0, viewer_id="one", interest=["tracker"]))
        assert latest["ok"] and latest["scene"]["trackers"][0]["valid"]
        with pytest.raises(ValueError):
            exchange("device.start", {})
    finally:
        server.close()
    assert not path.exists()


def test_default_manager_and_workstation_accept_optional_scene_path(tmp_path):
    from vt_capture_manager.config import load_config
    from vt_capture_core.workstation import validate_configuration
    path = tmp_path / "manager.json"
    value = dict(scene_config_path=str(tmp_path / "scene.yaml"))
    path.write_text(json.dumps(value))
    assert load_config(path).scene_config_path == value["scene_config_path"]
    assert validate_configuration(value)["scene_config_path"] == value["scene_config_path"]


def test_newly_received_historical_measurement_is_not_live():
    store, clock = make()
    old = sample(clock)
    clock.ns += 5_000_000_000
    store.observe_tracker('left_wrist', old)
    state = store.snapshot()['trackers'][0]
    assert state['state'] == 'offline' and not state['valid']
    assert state['age_s'] >= 5.


def test_retired_tracker_packet_cannot_replace_current_identity():
    store, clock = make()
    old = sample(clock)
    store.observe_tracker('left_wrist', old)
    clock.ns += 10_000_000
    store.observe_tracker('left_wrist', sample(clock, tracker_id='new-device'))
    epoch = store.coordinate_epoch
    assert not store.observe_tracker('left_wrist', old)
    current = store.snapshot()['trackers'][0]
    assert current['tracker_id'] == 'new-device' and current['valid']
    assert store.coordinate_epoch == epoch


def test_retired_clock_segment_cannot_restore_previous_time_epoch():
    store, clock = make()
    store.observe_tracker('left_wrist', sample(clock))
    clock.ns += 20_000_000
    old = sample(clock)
    store.observe_tracker('left_wrist', old)
    # Source realtime jumped backward while its monotonic clock advanced.
    clock.ns += 20_000_000
    store.observe_tracker('left_wrist', sample(clock, host_realtime_ns=clock.ns-100_000_000))
    before = store.snapshot()
    assert not store.observe_tracker('left_wrist', old)
    after = store.snapshot()
    assert after['time_epoch'] == before['time_epoch']
    assert after['trackers'][0]['source_realtime_ns'] == before['trackers'][0]['source_realtime_ns']


@pytest.mark.parametrize('patch', [{'schema_version': True}, {'schema_version': 1.0},
                                  {'alignment_config_path': False}, {'camera_device_info_services': []}])
def test_scene_configuration_rejects_ambiguous_types(tmp_path, patch):
    path = tmp_path / 'scene.yaml'
    path.write_text(json.dumps({'schema_version': 1, **patch}))
    with pytest.raises(ValueError):
        load_scene_config(path)


def test_scene_configuration_duplicate_keys_rejected(tmp_path):
    path = tmp_path / 'scene.yaml'
    path.write_text('schema_version: 1\nenabled: true\nenabled: false\n')
    with pytest.raises(ValueError):
        load_scene_config(path)


def test_producer_failure_never_returns_old_scene_as_live_and_can_recover(tmp_path):
    from vt_capture_core.spatial.scene_protocol import encode_scene_message, decode_scene_message
    store, _ = make()
    server = SceneServer(store, tmp_path/'scene.sock')
    request = encode_scene_message(dict(protocol='vt.scene', version=1, request_id='r', command='latest', params={}))
    original = store.snapshot
    def broken():
        raise RuntimeError('geometry revision update failed')
    store.snapshot = broken
    server._refresh_cache()
    result = decode_scene_message(server._reply(request))
    assert not result['ok'] and 'scene' not in result
    assert result['error']['code'] == 'scene_unavailable'
    store.snapshot = original
    server._refresh_cache()
    assert decode_scene_message(server._reply(request))['ok']


def test_cloud_scene_uses_frame_time_pose_and_rejects_old_generation():
    from vt_capture_core.spatial.transforms import RigidTransform
    store, clock = make()
    store.observe_tracker('left_wrist', sample(clock, position=(1., 0., 0.)))
    frame_time = clock.ns+10_000_000
    clock.ns += 20_000_000
    store.observe_tracker('left_wrist', sample(clock, position=(3., 0., 0.)))
    class Registry:
        def camera_entities(self, trackers, now_ns, *, require_live_tracker=True):
            return [dict(id='cam', state='fresh', registration_state='ready', geometry_revision=1,
                         geometry_sha256='a'*64, position_m=[3., 0., 0.], quaternion_xyzw=[0., 0., 0., 1.])]
    header = dict(camera_id='cam', manager_instance_id=store.manager_instance_id, scene_epoch=store.scene_epoch,
                  time_epoch=0, coordinate_epoch=0, sync_epoch=0, geometry_revision=1, geometry_sha256='a'*64,
                  frame_key=dict(scene_time_ns=frame_time, reference_camera='cam'), source_realtime_ns=frame_time,
                  world_from_color=RigidTransform((2.1, 0., 0.), (0., 0., 0., 1.)).as_dict())
    class Supervisor:
        def latest_headers(self):
            return {'cam': header}
    store.camera_registry, store.cloud_supervisor = Registry(), Supervisor()
    store.set_interest('viewer', ['tracker', 'camera', 'cloud'])
    scene = store.snapshot()
    assert scene['time_mode'] == 'aligned' and scene['scene_time_ns'] == frame_time
    assert scene['trackers'][0]['position_m'] == [2., 0., 0.]
    assert scene['cameras'][0]['position_m'] == [2.1, 0., 0.]
    assert scene['cameras'][0]['frame_key'] == header['frame_key']
    header['coordinate_epoch'] = 99
    assert store.snapshot()['time_mode'] == 'monitor'


def test_scene_freezes_effective_three_frame_configuration_for_replay(tmp_path):
    source = {'schema_version': 1, 'reference_camera': 'cam', 'cameras': [dict(name='cam', model='D405', serial='123',
              frame_id='cam_color_optical_frame', tracker_role='torso', color_topic='/cam/color/image_raw',
              depth_topic='/cam/depth/image_rect_raw', camera_info_topic='/cam/color/camera_info', timing_topic='/cam/frame_timing')],
              'trackers': [dict(role='torso', frame_id='vive_tracker_torso', sample_topic='/vive/torso/sample')], 'additional_streams': [], 'alignment': {}, 'frames': {'world': 'vive_map'}}
    alignment = tmp_path/'alignment.json'
    alignment.write_text(json.dumps(source))
    path = tmp_path/'scene.json'
    path.write_text(json.dumps(dict(schema_version=1, alignment_config_path='alignment.json')))
    config = load_scene_config(path)
    assert config.alignment_document['schema_version'] == 2
    frozen = config.alignment_document['cameras'][0]
    assert frozen['color_frame_id'] == 'cam_color_optical_frame'
    assert frozen['depth_frame_id'] == 'cam_depth_optical_frame'
    assert frozen['timing_frame_id'] == 'cam_frame_group'
    assert 'frame_id' not in frozen
    assert json.loads(alignment.read_text()) == source
