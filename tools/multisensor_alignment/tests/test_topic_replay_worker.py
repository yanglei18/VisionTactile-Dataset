"""Partial bags are independent of the spatial calibration and live clocks."""
from types import SimpleNamespace as NS

import pytest
import yaml

from vt_multisensor_alignment.topic_replay_worker import TopicReplaySource, _bounded_fields, _device


def make_source(tmp_path, types, messages, *, decode=None, seek=True):
    (tmp_path / 'metadata.yaml').write_text(yaml.safe_dump(dict(rosbag2_bagfile_information=dict(
        relative_file_paths=[], topics_with_message_count=[dict(topic_metadata=dict(name=name, type=kind),
        message_count=sum(topic == name for topic, _, _ in messages)) for name, kind in types.items()]))))
    readers = []

    class Reader:
        def __init__(self):
            self.index = 0
            self.closed = False
            readers.append(self)

        def get_all_topics_and_types(self):
            return [NS(name=name, type=kind) for name, kind in types.items()]

        def has_next(self):
            return self.index < len(messages)

        def read_next(self):
            result = messages[self.index]
            self.index += 1
            return result

        def close(self):
            self.closed = True

    if seek:
        def reader_seek(self, stamp):
            self.index = next((i for i, (_, _, value) in enumerate(messages) if value >= stamp), len(messages))
        Reader.seek = reader_seek
    source = TopicReplaySource(tmp_path, reader_factory=lambda _: Reader(),
                               deserializer_factory=decode or (lambda _: lambda payload: NS(value=payload.decode())))
    return source, readers


def test_empty_tracker_and_unknown_topic_remain_playable(tmp_path):
    source, readers = make_source(tmp_path, {'/tracker': 'vt_tracker_msgs/msg/TrackerSample', '/other': 'custom/msg/Foo'},
                                  [('/other', b'first', 100), ('/other', b'last', 200)])
    try:
        assert source.metadata['start_ns'] == 100
        assert source.metadata['end_ns'] == 200
        assert source.metadata['topics'][0]['count'] == 0
        assert source.metadata['warnings']
        assert source.frame('/tracker', 150)['device'] is None
        assert source.frame('/other', 150)['details']['fields']['value'] == 'first'
        assert source.frame('/other', 99)['sample_ns'] is None
    finally:
        source.close()
    assert all(reader.closed for reader in readers)


@pytest.mark.parametrize('seek', [False, True])
def test_backward_seek_and_topic_switch_read_exact_recorded_message(tmp_path, seek):
    source, _ = make_source(tmp_path, {'/a': 'Test', '/b': 'Test'},
        [('/a', b'a0', 10), ('/b', b'b0', 12), ('/a', b'a1', 20), ('/a', b'a2', 20)], seek=seek)
    try:
        assert source.frame('/a', 20)['details']['fields']['value'] == 'a2'
        assert source.frame('/b', 20)['details']['fields']['value'] == 'b0'
        assert source.frame('/a', 10)['details']['fields']['value'] == 'a0'
        assert source.frame('/a', 15)['sample_age_ns'] == 5
    finally:
        source.close()


def test_missing_message_package_keeps_raw_topic_summary(tmp_path):
    def missing(_):
        raise ImportError('custom messages not installed')
    source, _ = make_source(tmp_path, {'/custom': 'custom/msg/Foo'}, [('/custom', b'raw', 10)], decode=missing)
    try:
        frame = source.frame('/custom', 10)
        assert frame['details']['representation'] == 'serialized'
        assert frame['details']['serialized_bytes'] == 3
        assert frame['details']['prefix_hex'] == b'raw'.hex()
        assert frame['warnings']
    finally:
        source.close()


def test_bad_message_does_not_block_next_sample_or_other_topic(tmp_path):
    def decode(_):
        def deserialize(payload):
            if payload == b'bad':
                raise ValueError('invalid CDR')
            return NS(value=payload.decode())
        return deserialize
    source, _ = make_source(tmp_path, {'/a': 'Test', '/b': 'Test'},
                            [('/a', b'bad', 10), ('/b', b'ok', 20), ('/a', b'next', 30)], decode=decode)
    try:
        assert source.frame('/a', 10)['warnings']
        assert source.frame('/b', 20)['details']['fields']['value'] == 'ok'
        assert source.frame('/a', 30)['details']['fields']['value'] == 'next'
    finally:
        source.close()


def test_image_only_bag_produces_replay_preview(tmp_path, monkeypatch):
    import sys
    converter = NS(jpeg_preview=lambda message, **_: dict(width=1, height=1, format='jpeg', data_b64='jpeg'))
    monkeypatch.setitem(sys.modules, 'vt_capture_manager.image_preview', converter)
    source, _ = make_source(tmp_path, {'/rgb': 'sensor_msgs/msg/Image'}, [('/rgb', b'pixels', 10)],
                            decode=lambda _: lambda payload: NS(encoding='rgb8', width=1, height=1, data=payload))
    try:
        frame = source.frame('/rgb', 10)
        assert frame['device']['status'] == 'ready'
        assert frame['device']['preview']['mode'] == 'replay'
        assert frame['device']['preview']['data_b64'] == 'jpeg'
    finally:
        source.close()


def test_tracker_invalid_pose_keeps_fields_but_no_stale_graphics():
    warnings = []
    message = NS(pose_valid=False)
    assert _device('/tracker', 'vt_tracker_msgs/msg/TrackerSample', message, 1, 1, warnings) is None
    assert warnings


def test_tracker_accepts_recorded_coordinate_frame_without_calibration():
    message = NS(pose_valid=True, pose=NS(position=NS(x=1, y=2, z=3), orientation=NS(x=0, y=0, z=0, w=1)),
                 header=NS(frame_id='old_recorded_frame', stamp=NS(sec=1, nanosec=0)))
    device = _device('/tracker', 'vt_tracker_msgs/msg/TrackerSample', message, 1, 1, [])
    assert device['preview']['position'] == [1, 2, 3]
    assert device['preview']['mode'] == 'replay'
    assert device['frame_id'] == 'old_recorded_frame'
    assert device['preview']['frame_id'] == 'old_recorded_frame'
    assert device['source_realtime_ns'] == 1_000_000_000
    assert device['preview']['source_realtime_ns'] == 1_000_000_000


def test_glove_replay_uses_recorded_joints_without_wall_clock():
    message = NS(pose_valid=True, time_valid=True, hand='right', joint_positions_m=list(range(63)),
                 header=NS(stamp=NS(sec=1, nanosec=0)))
    device = _device('/glove', 'vt_glove_msgs/msg/GloveState', message, 100, 100, [])
    assert len(device['preview']['joints']) == 21
    assert device['preview']['mode'] == 'replay'


def test_tactile_state_preserves_recorded_mask_without_wall_clock():
    mask = [False] * 400
    mask[9] = True
    message = NS(hand='right', schema_version=2, rows=20, cols=20, adc=list(range(400)), valid_mask=mask,
                 connected=True, time_valid=True)
    device = _device('/tactile', 'vt_tactile_msgs/msg/TactileState', message, 1, 10**18, [])
    assert device['preview']['valid_mask'] == mask
    assert device['preview']['values'][9] == 9


def test_field_summary_bounds_buffers_and_marks_truncation():
    details = _bounded_fields(NS(data=bytes(1_000_000), array=list(range(10_000))))
    assert details['truncated']
    assert details['fields']['data']['bytes'] == 1_000_000
    assert len(details['fields']['array']['first']) == 64
    assert len(str(details)) < 2000


def test_missing_metadata_reports_actual_problem(tmp_path):
    with pytest.raises(ValueError, match='metadata.yaml'):
        TopicReplaySource(tmp_path)


@pytest.mark.parametrize('location', ['session', 'metadata'])
def test_source_resolves_archive_session_and_metadata_paths(tmp_path, location):
    bag = tmp_path / 'bag'
    bag.mkdir()
    original, _ = make_source(bag, {'/a': 'Test'}, [('/a', b'a0', 10)])
    original.close()
    selected = tmp_path if location == 'session' else bag / 'metadata.yaml'
    source = TopicReplaySource(selected, reader_factory=original._factory,
                               deserializer_factory=original._decode_factory)
    try:
        assert source.path == bag
        assert source.frame('/a', 10)['details']['fields']['value'] == 'a0'
    finally:
        source.close()


def test_cached_frames_are_invalidated_if_bag_changes(tmp_path):
    source, _ = make_source(tmp_path, {'/a': 'Test'}, [('/a', b'a0', 10)])
    try:
        assert source.frame('/a', 10)['sample_ns'] == 10
        metadata = tmp_path / 'metadata.yaml'
        metadata.write_text(metadata.read_text() + '\n# changed\n')
        with pytest.raises(ValueError, match='发生变化'):
            source.frame('/a', 10)
    finally:
        source.close()


def test_out_of_order_topic_does_not_block_other_topics(tmp_path):
    source, _ = make_source(tmp_path, {'/a': 'Test', '/b': 'Test'},
        [('/a', b'a2', 30), ('/a', b'a0', 10), ('/b', b'b0', 12), ('/a', b'a1', 20)])
    try:
        assert source.frame('/a', 15)['details']['fields']['value'] == 'a0'
        assert source.frame('/b', 12)['details']['fields']['value'] == 'b0'
        assert source.frame('/a', 35)['details']['fields']['value'] == 'a2'
        assert any('乱序' in warning for warning in source.metadata['warnings'])
    finally:
        source.close()


def test_invalid_timestamp_does_not_shift_following_message_reference(tmp_path):
    source, _ = make_source(tmp_path, {'/a': 'Test'},
        [('/a', b'invalid', -1), ('/a', b'correct', 10)], seek=False)
    try:
        assert source.frame('/a', 10)['details']['fields']['value'] == 'correct'
        assert source.metadata['topics'][0]['count'] == 1
    finally:
        source.close()


def test_incomplete_or_bad_geometry_only_adds_warning(tmp_path):
    geometry = tmp_path.with_name(tmp_path.name + '.spatial')
    geometry.mkdir()
    (geometry / 'manifest.json').write_text('{invalid json')
    source, _ = make_source(tmp_path, {'/a': 'Test'}, [('/a', b'ok', 10)])
    try:
        assert source.frame('/a', 10)['sample_ns'] == 10
        assert any('空间几何清单无法读取' in note for note in source.metadata['warnings'])
    finally:
        source.close()


def test_invalid_tracker_frame_retains_recorded_identity_and_source_time(tmp_path):
    message = NS(pose_valid=False, role='left_wrist', tracker_id='recorded-device',
                 host_realtime_ns=1_000_000_001,
                 header=NS(frame_id='recorded_frame', stamp=NS(sec=1, nanosec=0)))
    source, _ = make_source(tmp_path, {'/tracker': 'vt_tracker_msgs/msg/TrackerSample'},
                            [('/tracker', b'payload', 100)], decode=lambda _: lambda payload: message)
    try:
        frame = source.frame('/tracker', 100)
        assert frame['device'] is None
        assert frame['frame_id'] == 'recorded_frame'
        assert frame['source_realtime_ns'] == 1_000_000_000
        assert frame['host_realtime_ns'] == 1_000_000_001
        assert frame['tracker_id'] == 'recorded-device'
        assert frame['role'] == 'left_wrist'
    finally:
        source.close()
