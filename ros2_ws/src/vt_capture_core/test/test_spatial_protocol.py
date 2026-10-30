import json

import pytest

from vt_capture_core.spatial.scene_protocol import (
    MAX_SCENE_BYTES, SceneProtocolError, decode_scene_message, encode_scene_message,
)


def request(command='latest', params=None):
    return dict(protocol='vt.scene', version=1, request_id='one', command=command,
                params=params if params is not None else {'after_sequence': 0, 'viewer_id': 'ui', 'interest': ['tracker', 'camera']})


def scene():
    return dict(manager_instance_id='manager', scene_epoch='s1', sequence=1,
                mode='live', world_frame='vive_map', time_epoch='t1', coordinate_epoch='c1',
                sync_epoch='y1', time_mode='monitor', scene_time_ns=None,
                generated_realtime_ns=100, config_revision='config', geometry_revision='geometry',
                active_reference_camera=None, map_identity=None,
                display_from_world={'translation_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
                trackers=[], cameras=[], diagnostics=[])


def test_read_only_requests_and_response_roundtrip():
    for message in [request(), request('hello', {}), request('describe', {}),
                    dict(protocol='vt.scene', version=1, request_id='one', ok=True, error=None, scene=scene()),
                    dict(protocol='vt.scene', version=1, request_id='one', ok=True, error=None, no_change=True),
                    dict(protocol='vt.scene', version=1, request_id='one', ok=False, error={'code': 'bad_request', 'message': 'invalid'})]:
        wire = encode_scene_message(message)
        assert wire.endswith(b'\n') and wire.count(b'\n') == 1
        assert decode_scene_message(wire) == message


@pytest.mark.parametrize('notice', ['', '左腕·试用候选（三轮一致性未通过）', 'x' * 1024])
def test_optional_preview_notice_roundtrip(notice):
    response = dict(protocol='vt.scene', version=1, request_id='notice', ok=True, error=None,
                    scene=dict(scene(), preview_notice=notice))
    assert decode_scene_message(encode_scene_message(response)) == response


@pytest.mark.parametrize('notice', [None, False, 17, [], {}, 'x' * 1025])
def test_preview_notice_is_bounded_plain_text(notice):
    response = dict(protocol='vt.scene', version=1, request_id='notice', ok=True, error=None,
                    scene=dict(scene(), preview_notice=notice))
    with pytest.raises(SceneProtocolError, match='preview_notice'):
        encode_scene_message(response)


@pytest.mark.parametrize('message', [
    request('device.start'), request('capture.start'), request('latest', {'after_sequence': -1}),
    request('latest', {'after_sequence': True}), request('latest', {'interest': ['hardware']}),
    dict(request(), version=True), dict(request(), extra='unknown'),
    dict(protocol='vt.scene', version=1, request_id='one', ok=True, error=None),
    dict(protocol='vt.scene', version=1, request_id='one', ok=False, error=None, scene=scene()),
])
def test_malformed_or_control_messages_rejected(message):
    with pytest.raises(SceneProtocolError):
        encode_scene_message(message)
    with pytest.raises(SceneProtocolError):
        decode_scene_message(json.dumps(message))


@pytest.mark.parametrize('wire', [
    b'{"protocol":"vt.scene","protocol":"vt.scene"}', b'{}\n{}\n', b'\xff',
    b'{"bad":NaN}', b'['*1000 + b']'*1000, b'x'*(MAX_SCENE_BYTES+1),
])
def test_frame_limits_duplicate_keys_nonfinite_and_recursion(wire):
    with pytest.raises(SceneProtocolError):
        decode_scene_message(wire)


def test_entity_bounds_and_finite_pose_checked():
    value = scene()
    value['trackers'] = [dict(id='a', position_m=[1, 2, float('nan')])]
    response = dict(protocol='vt.scene', version=1, request_id='r', ok=True, error=None, scene=value)
    with pytest.raises(SceneProtocolError):
        encode_scene_message(response)
    value['trackers'] = [dict(id='a', position_m=[1, 2, 3], quaternion_xyzw=[0, 0, 0, 0])]
    with pytest.raises(SceneProtocolError):
        encode_scene_message(response)
    value['trackers'] = [dict(id='a', state='invented')]
    with pytest.raises(SceneProtocolError):
        encode_scene_message(response)


@pytest.mark.parametrize('kind', ['trackers', 'cameras'])
@pytest.mark.parametrize('entity', [
    {}, {'label': 'waiting'}, {'id': None}, {'id': ''}, {'id': '   '},
    {'id': 17}, {'id': 'x'*257}, {'id': 'device', 'label': None},
    {'id': 'device', 'label': ''}, {'id': 'device', 'label': 17},
])
def test_entity_requires_renderable_stable_id_and_optional_nonempty_label(kind, entity):
    value = scene()
    value[kind] = [entity]
    response = dict(protocol='vt.scene', version=1, request_id='entity', ok=True, error=None, scene=value)
    with pytest.raises(SceneProtocolError):
        encode_scene_message(response)
    with pytest.raises(SceneProtocolError):
        decode_scene_message(json.dumps(response))


@pytest.mark.parametrize('kind', ['trackers', 'cameras'])
def test_waiting_entity_allows_unknown_observed_identity_and_absent_label(kind):
    value = scene()
    value[kind] = [dict(id='waiting-device', role=None, tracker_id=None, camera_id=None,
                        state='waiting', position_m=None, quaternion_xyzw=None)]
    response = dict(protocol='vt.scene', version=1, request_id='waiting', ok=True, error=None, scene=value)
    assert decode_scene_message(encode_scene_message(response)) == response
