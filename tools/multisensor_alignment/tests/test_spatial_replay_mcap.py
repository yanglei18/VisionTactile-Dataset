"""Optional real ROS serialization and subprocess replay, no sensor required."""
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

rosbag2_py = pytest.importorskip('rosbag2_py')
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CameraInfo
from vt_camera_msgs.msg import CameraFrameTiming
from vt_tracker_msgs.msg import TrackerSample

from vt_capture_core.spatial.cloud_protocol import unpack_cloud_frame
from vt_capture_manager.spatial.camera_geometry import CameraGeometryRegistry
from vt_multisensor_alignment.config import load_config_document
from vt_multisensor_alignment.bag_reader import configured_topic_types
from vt_multisensor_alignment.image_reader import BagImageReader
from vt_multisensor_alignment.spatial_replay_worker import BagReplaySource


def make_bag(directory, *, shared_tracker=False):
    document = yaml.safe_load((Path(__file__).parents[1]/'config'/'alignment.example.yaml').read_text())
    document['schema_version'] = 2
    for camera in document['cameras']:
        camera['color_frame_id'] = camera.pop('frame_id')
        camera['depth_frame_id'] = camera['name']+'_depth_optical_frame'
        camera['timing_frame_id'] = camera['name']+'_frame_group'
        camera['depth_camera_info_topic'] = '/'+camera['name']+'/depth/camera_info'
    if shared_tracker:
        document['cameras'][0]['tracker_role'] = document['cameras'][2]['tracker_role']
    config = load_config_document(document)
    configs = [dict(camera, id=camera['name']) for camera in document['cameras']]
    registry = CameraGeometryRegistry(configs)
    bag = directory/'bag'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    for number, (topic, kind) in enumerate(configured_topic_types(config).items(), 1):
        writer.create_topic(rosbag2_py.TopicMetadata(id=number, name=topic, type=kind, serialization_format='cdr'))
    events = []
    ids = {role: str(i+1)*64 for i, role in enumerate(('left_wrist', 'right_wrist', 'torso'))}
    def stamp(message, ns, frame):
        message.header.stamp.sec = ns//1_000_000_000
        message.header.stamp.nanosec = ns%1_000_000_000
        message.header.frame_id = frame
    for number, camera in enumerate(config.cameras):
        registry.observe_device_info(camera.name, type('Info', (), {'serial_number': camera.serial, 'device_name': camera.model})(), observed_ns=950_000_000)
        registry.set_depth_units(camera.name, .001, 'realsense_ros_z16_mm')
        registry.observe_depth_to_color(camera.name, {'translation_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]})
        tracker_frame = config.tracker_by_role[camera.tracker_role].frame_id
        registry.set_extrinsic(camera.name, dict(schema_version=1, status='VALID',
            camera=dict(name=camera.name, model=camera.model, serial=camera.serial, frame_id=camera.color_frame_id),
            tracker=dict(role=camera.tracker_role, tracker_id=ids[camera.tracker_role], frame_id=tracker_frame),
            transform=dict(semantics='parent_from_child', parent_frame=tracker_frame, child_frame=camera.color_frame_id,
                           translation_m=[0., 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]), frames={'world': 'vive_map'}))
        for stream, topic in [('color', camera.camera_info_topic), ('depth', camera.depth_camera_info_topic)]:
            info = CameraInfo()
            stamp(info, 900_000_000, getattr(camera, stream+'_frame_id'))
            info.width = info.height = 2
            info.k = [2., 0., .5, 0., 2., .5, 0., 0., 1.]
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [2., 0., .5, 0., 0., 2., .5, 0., 0., 0., 1., 0.]
            info.distortion_model = 'plumb_bob'
            info.d = [0.]*5
            registry.observe_camera_info(camera.name, info, stream)
            events.append((900_000_000, topic, info))
        for index in range(2):
            source = 10_000_000_000+number*100+index
            host = 1_000_000_000+index*20_000_000+number*1_000_000
            for stream, topic in [('color', camera.color_topic), ('depth', camera.depth_topic)]:
                image = Image()
                stamp(image, source, getattr(camera, stream+'_frame_id'))
                image.width = image.height = 2
                image.encoding = 'rgb8' if stream == 'color' else '16UC1'
                image.step = 6 if stream == 'color' else 4
                image.data = bytes([50+number, 100, 150]*4) if stream == 'color' else bytes([232, 3]*4)
                events.append((host, topic, image))
            timing = CameraFrameTiming()
            stamp(timing, source, camera.timing_frame_id)
            timing.camera_name, timing.camera_model, timing.serial_number = camera.name, camera.model, camera.serial
            timing.shared_ros_timestamp_ns = source
            timing.group_host_realtime_ns = host
            timing.group_host_monotonic_raw_ns = host
            timing.group_validity_flags = (timing.GROUP_VALID_COMMON_STAMP|timing.GROUP_VALID_IDENTITY|timing.GROUP_VALID_CALLBACK_CLOCKS)
            events.append((host+1, camera.timing_topic, timing))
    for number, tracker in enumerate(config.trackers):
        for index in range(5):
            host = 990_000_000+index*10_000_000
            sample = TrackerSample()
            stamp(sample, host, 'vive_map')
            sample.role, sample.tracker_id = tracker.role, ids[tracker.role]
            sample.host_realtime_ns = sample.host_monotonic_ns = host
            sample.pose_valid, sample.tracking_status, sample.packet_index = True, 2, index
            sample.pose.position.x = number+index*.1
            sample.pose.orientation.w = 1.
            events.append((host, tracker.sample_topic, sample))
    for timestamp, topic, message in sorted(events, key=lambda entry: entry[0]):
        writer.write(topic, serialize_message(message), timestamp)
    writer.close()
    sidecar = bag.with_name(bag.name+'.spatial')
    sidecar.mkdir()
    revisions = []
    for camera in config.cameras:
        geometry = registry.geometry(camera.name)
        data = json.dumps(geometry, sort_keys=True).encode()
        digest = hashlib.sha256(data).hexdigest()
        filename = 'geometry-'+digest+'.json'
        (sidecar/filename).write_bytes(data)
        revisions.append(dict(camera_id=camera.name, geometry_revision=geometry['geometry_revision'],
            geometry_sha256=geometry['geometry_sha256'], artifact=filename, artifact_sha256=digest,
            observed_realtime_ns=950_000_000, time_epoch=0, coordinate_epoch=0, scene_epoch='recorded'))
    manifest = dict(schema_version=1, kind='vt.spatial.geometry', bag_path=str(bag), session_id='synthetic',
        world_frame='vive_map', complete=True, geometry_status='available', config_revision='synthetic',
        alignment_config=document, camera_config=configs, reference_camera=config.reference_camera,
        revisions=revisions, epochs=[], updated_realtime_ns=2_000_000_000)
    (sidecar/'manifest.json').write_text(json.dumps(manifest))
    return bag


def test_real_schema2_mcap_image_and_cloud_replay(tmp_path):
    bag = make_bag(tmp_path)
    source = BagReplaySource(bag)
    assert source.frame_count == 2
    scene = source.frame(0)
    assert len(source.last_clouds) == 3, scene['diagnostics']
    for encoded in source.last_clouds:
        frame = unpack_cloud_frame(base64.b64decode(encoded))
        assert frame.header['point_count'] == 1
        assert frame.header['time_epoch'] == scene['time_epoch']
    selected = source.dataset.camera_frames['d405_1'][0]
    with BagImageReader(bag) as images:
        assert images.read(selected.depth).pixels().tolist() == [[1000, 1000], [1000, 1000]]
    source.close()
    process = subprocess.Popen([sys.executable, '-m', 'vt_multisensor_alignment.spatial_replay_worker', str(bag)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output, error = process.communicate(b'{"command":"frame","index":1,"request_id":7}\n', timeout=20.)
    assert process.returncode == 0, error.decode()
    metadata, response = [json.loads(line) for line in output.splitlines()]
    assert metadata['metadata']['frame_count'] == 2
    assert response['ok'] and len(response['clouds']) == 3
    assert response['request_id'] == 7 and response['index'] == 1
    print('SPATIAL_REPLAY_MCAP='+str(bag))


def test_real_mcap_replay_uses_shared_tracker_binding_from_recorded_contract(tmp_path):
    source = BagReplaySource(make_bag(tmp_path, shared_tracker=True))
    try:
        scene = source.frame(0)
        camera = next(value for value in scene['cameras'] if value['id'] == 'd405_1')
        assert camera['tracker_role'] == 'right_wrist'
        assert camera['position_m'][0] == pytest.approx(1.1)
        assert len(source.last_clouds) == 3
    finally:
        source.close()
