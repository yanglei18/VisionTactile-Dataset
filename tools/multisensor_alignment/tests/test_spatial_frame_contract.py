from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from vt_multisensor_alignment.config import load_config, load_config_document
from vt_multisensor_alignment import bag_reader


def config_file(tmp_path, version=2):
    example = Path(__file__).resolve().parents[1]/'config'/'alignment.example.yaml'
    document = yaml.safe_load(example.read_text())
    if version == 2:
        document['schema_version'] = 2
        for camera in document['cameras']:
            camera['color_frame_id'] = camera.pop('frame_id')
            camera['depth_frame_id'] = camera['name']+'_depth_optical_frame'
            camera['timing_frame_id'] = camera['name']+'_frame_group'
            camera['depth_camera_info_topic'] = '/'+camera['name']+'/depth/camera_info'
    path = tmp_path/'config.yaml'
    path.write_text(yaml.safe_dump(document))
    return path


def test_schema_two_separates_real_color_depth_and_timing_frames(tmp_path):
    config = load_config(config_file(tmp_path))
    camera = config.cameras[0]
    assert config.schema_version == 2
    assert camera.frame_id == camera.color_frame_id
    assert len({camera.color_frame_id, camera.depth_frame_id, camera.timing_frame_id}) == 3
    assert camera.depth_camera_info_topic not in config.required_topics
    assert bag_reader.configured_topic_types(config)[camera.depth_camera_info_topic] == 'sensor_msgs/msg/CameraInfo'


def test_schema_one_retains_legacy_single_frame_semantics(tmp_path):
    config = load_config(config_file(tmp_path, 1))
    camera = config.cameras[0]
    assert camera.frame_id == camera.color_frame_id == camera.depth_frame_id == camera.timing_frame_id
    assert camera.depth_camera_info_topic is None
    assert len(config.required_topics) == 15


def test_recorded_alignment_document_loads_without_reopening_live_configuration(tmp_path):
    path = config_file(tmp_path)
    recorded = yaml.safe_load(path.read_text())
    assert load_config_document(recorded) == load_config(path)


def fake_bag(config):
    messages = []
    def header(frame, t=1000):
        return NS(frame_id=frame, stamp=NS(sec=0, nanosec=t))
    for camera in config.cameras:
        for topic, frame in ((camera.color_topic, camera.color_frame_id), (camera.depth_topic, camera.depth_frame_id),
                             (camera.camera_info_topic, camera.color_frame_id), (camera.depth_camera_info_topic, camera.depth_frame_id)):
            if topic:
                messages.append((topic, NS(header=header(frame)), 1001))
        timing = NS(header=header(camera.timing_frame_id), camera_name=camera.name, camera_model=camera.model,
                    serial_number=camera.serial, shared_ros_timestamp_ns=1000, group_host_realtime_ns=1000,
                    group_host_monotonic_raw_ns=1000, GROUP_VALID_COMMON_STAMP=1, GROUP_VALID_IDENTITY=2,
                    GROUP_VALID_CALLBACK_CLOCKS=4, group_validity_flags=7)
        messages.append((camera.timing_topic, timing, 1001))
    for tracker in config.trackers:
        for t in (900, 1100):
            message = NS(header=header(config.world_frame, t), role=tracker.role, tracker_id='1'*64,
                         host_realtime_ns=t, host_monotonic_ns=t, pose_valid=True, tracking_status=2,
                         pose=NS(position=NS(x=0, y=0, z=0), orientation=NS(x=0, y=0, z=0, w=1)))
            messages.append((tracker.sample_topic, message, t+1))
    return messages


@pytest.mark.parametrize('wrong_stream', [None, 'depth', 'timing', 'depth_info'])
def test_indexer_checks_each_real_frame_without_rewriting_headers(tmp_path, monkeypatch, wrong_stream):
    config = load_config(config_file(tmp_path))
    messages = fake_bag(config)
    if wrong_stream:
        camera = config.cameras[0]
        topic = {'depth': camera.depth_topic, 'timing': camera.timing_topic, 'depth_info': camera.depth_camera_info_topic}[wrong_stream]
        next(msg for name, msg, _ in messages if name == topic).header.frame_id = camera.color_frame_id
    queue = list(messages)
    reader = NS(has_next=lambda: bool(queue), read_next=lambda: queue.pop(0))
    monkeypatch.setattr(bag_reader, '_open_reader', lambda path: reader)
    monkeypatch.setattr(bag_reader, '_message_types', lambda reader: bag_reader.configured_topic_types(config))
    monkeypatch.setattr(bag_reader, '_storage_identifier', lambda path: 'mcap')
    monkeypatch.setattr(bag_reader, '_deserializer', lambda name: lambda payload: payload)
    if wrong_stream:
        with pytest.raises(ValueError, match='frame mismatch'):
            bag_reader.read_unified_bag(tmp_path, config)
    else:
        result = bag_reader.read_unified_bag(tmp_path, config)
        assert all(len(frames) == 1 for frames in result.camera_frames.values())


def test_indexer_preserves_invalid_tracker_boundaries_for_spatial_replay(tmp_path, monkeypatch):
    config = load_config(config_file(tmp_path))
    events = fake_bag(config)
    topic, invalid, stamp = next(item for item in events if item[0] == config.trackers[0].sample_topic)
    from copy import deepcopy
    invalid = deepcopy(invalid)
    invalid.pose_valid = False
    invalid.host_realtime_ns = invalid.host_monotonic_ns = 1000
    invalid.header.stamp.nanosec = 1000
    events.append((topic, invalid, 1001))
    events.sort(key=lambda item: item[2])
    reader = NS(has_next=lambda: bool(events), read_next=lambda: events.pop(0))
    monkeypatch.setattr(bag_reader, '_open_reader', lambda path: reader)
    monkeypatch.setattr(bag_reader, '_message_types', lambda reader: bag_reader.configured_topic_types(config))
    monkeypatch.setattr(bag_reader, '_storage_identifier', lambda path: 'mcap')
    monkeypatch.setattr(bag_reader, '_deserializer', lambda name: lambda payload: payload)
    result = bag_reader.read_unified_bag(tmp_path, config)
    assert result.invalid_tracker_times[config.trackers[0].role] == (1000,)


def test_replay_indexer_can_explicitly_allow_absent_camera_topics(tmp_path, monkeypatch):
    config = load_config(config_file(tmp_path))
    reference = config.camera_by_name[config.reference_camera]
    absent = {reference.color_topic, reference.depth_topic, reference.camera_info_topic, reference.timing_topic, reference.depth_camera_info_topic}
    events = [entry for entry in fake_bag(config) if entry[0] not in absent]
    reader = NS(has_next=lambda: bool(events), read_next=lambda: events.pop(0))
    observed = {key: value for key, value in bag_reader.configured_topic_types(config).items() if key not in absent}
    monkeypatch.setattr(bag_reader, '_open_reader', lambda path: reader)
    monkeypatch.setattr(bag_reader, '_message_types', lambda reader: observed)
    monkeypatch.setattr(bag_reader, '_storage_identifier', lambda path: 'mcap')
    monkeypatch.setattr(bag_reader, '_deserializer', lambda name: lambda payload: payload)
    result = bag_reader.read_unified_bag(tmp_path, config, allow_partial_cameras=True)
    assert result.camera_frames[reference.name] == ()
    assert sum(bool(frames) for frames in result.camera_frames.values()) == 2


def test_legacy_indexer_retains_factory_calibration_counts_without_sdk_dependency(tmp_path, monkeypatch):
    config = load_config(config_file(tmp_path, 1))
    events = fake_bag(config)
    observed = bag_reader.configured_topic_types(config)
    for camera in config.cameras:
        for suffix, type_name in (("depth/camera_info", "sensor_msgs/msg/CameraInfo"),
                                  ("extrinsics/depth_to_color", "realsense2_camera_msgs/msg/Extrinsics")):
            topic = f"/{camera.name}/{suffix}"
            observed[topic] = type_name
            events.append((topic, b"opaque auxiliary payload", 1001))
    reader = NS(has_next=lambda: bool(events), read_next=lambda: events.pop(0))
    monkeypatch.setattr(bag_reader, '_open_reader', lambda path: reader)
    monkeypatch.setattr(bag_reader, '_message_types', lambda reader: observed)
    monkeypatch.setattr(bag_reader, '_storage_identifier', lambda path: 'mcap')

    def deserializer(type_name):
        assert type_name != "realsense2_camera_msgs/msg/Extrinsics"
        return lambda payload: payload

    monkeypatch.setattr(bag_reader, '_deserializer', deserializer)
    result = bag_reader.read_unified_bag(tmp_path, config)
    assert result.topic_types == observed
    for camera in config.cameras:
        assert len(result.camera_frames[camera.name]) == 1
        for suffix in ("depth/camera_info", "extrinsics/depth_to_color"):
            topic = f"/{camera.name}/{suffix}"
            assert result.message_counts[topic] == 1
            assert result.accepted_counts[topic] == 0


def test_schema_two_allows_explicit_shared_tracker_and_legacy_keeps_one_to_one(tmp_path):
    current = yaml.safe_load(config_file(tmp_path, 2).read_text())
    current['cameras'][0]['tracker_role'] = current['cameras'][2]['tracker_role']
    config = load_config_document(current)
    assert config.cameras[0].tracker_role == config.cameras[2].tracker_role
    old = yaml.safe_load(config_file(tmp_path, 1).read_text())
    old['cameras'][0]['tracker_role'] = old['cameras'][2]['tracker_role']
    with pytest.raises(ValueError, match='one-to-one'):
        load_config_document(old)


@pytest.mark.parametrize('invalid', ['unknown_camera_role', 'duplicate_tracker_role'])
def test_shared_tracker_contract_still_rejects_invalid_role_bindings(tmp_path, invalid):
    document = yaml.safe_load(config_file(tmp_path, 2).read_text())
    if invalid == 'unknown_camera_role':
        document['cameras'][0]['tracker_role'] = 'unconfigured'
    else:
        document['trackers'][0]['role'] = document['trackers'][1]['role']
    with pytest.raises(ValueError, match='unsupported|duplicate'):
        load_config_document(document)
