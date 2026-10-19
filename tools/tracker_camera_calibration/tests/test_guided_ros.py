"""Opt-in synthetic ROS integration; no physical sensor or production domain.

VT_GUIDED_ROS_TEST=1 ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
Run after sourcing Jazzy and the built message workspace.
"""
import json
import os
import subprocess
import sys
from pathlib import Path
import time

import cv2
import numpy as np
import pytest

from vt_tracker_camera_calib.charuco import create_board
from vt_tracker_camera_calib.config import load_config
from vt_tracker_camera_calib.config_writer import build_config_document, write_config
from vt_tracker_camera_calib.guided_live import GuidedMonitor
from vt_tracker_camera_calib.guided_worker import Request, Worker
from vt_tracker_camera_calib.gui_workflow import ManagedJob, record_command
from vt_tracker_camera_calib.transforms import Transform

pytestmark = pytest.mark.skipif(os.environ.get('VT_GUIDED_ROS_TEST') != '1',
                                reason='explicit isolated synthetic ROS opt-in required')


def messages(config, number=0, stamp=10_000_000_000):
    from sensor_msgs.msg import Image, CameraInfo
    from vt_camera_msgs.msg import CameraFrameTiming
    from vt_tracker_msgs.msg import TrackerSample
    rng = np.random.default_rng(20261015 + number)
    transform = Transform.from_rvec_tvec(rng.uniform(-.4, .4, 3),
                                         [-.18 + rng.uniform(-.04, .04), -.12 + rng.uniform(-.03, .03), rng.uniform(.62, .9)])
    expected = Transform.from_rvec_tvec([.2, -.1, .15], [.08, -.03, .12])
    world_board = Transform.from_rvec_tvec([.1, .2, -.3], [.4, -.2, 1.])
    tracker_pose = world_board @ transform.inverse() @ expected.inverse()
    matrix = np.array([[900., 0., 400.], [0., 900., 300.], [0., 0., 1.]])
    board = create_board(config.board).generateImage((900, 600))
    object_corners = np.array([[0.,0.,0.],[.36,0.,0.],[.36,.24,0.],[0.,.24,0.]])
    rvec, _ = cv2.Rodrigues(transform.rotation)
    projected, _ = cv2.projectPoints(object_corners, rvec, transform.translation, matrix, np.zeros(5))
    homography = cv2.getPerspectiveTransform(np.array([[0,0],[899,0],[899,599],[0,599]], dtype=np.float32),
                                             projected.reshape(-1,2).astype(np.float32))
    pixels = cv2.warpPerspective(board, homography, (800, 600), borderValue=255)
    image = Image(); image.header.frame_id = config.camera_frame
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    image.height, image.width, image.step, image.encoding = 600, 800, 800, 'mono8'
    image.data = pixels.tobytes()
    info = CameraInfo(); info.header = image.header; info.height = 600; info.width = 800
    info.k = matrix.reshape(-1).tolist(); info.d = [0.] * 5; info.distortion_model = 'plumb_bob'
    timing = CameraFrameTiming(); timing.header = image.header
    timing.camera_name, timing.camera_model, timing.serial_number = config.camera_name, config.camera_model, config.camera_serial
    timing.shared_ros_timestamp_ns = stamp; timing.group_host_realtime_ns = stamp
    timing.group_host_monotonic_raw_ns = stamp + 1_000_000_000
    timing.group_validity_flags = (timing.GROUP_VALID_COMMON_STAMP | timing.GROUP_VALID_IDENTITY | timing.GROUP_VALID_CALLBACK_CLOCKS)
    tracker = []
    for delta in range(-150_000_000, 151_000_000, 25_000_000):
        sample = TrackerSample(); sample.header.frame_id = config.world_frame
        sample.header.stamp.sec, sample.header.stamp.nanosec = divmod(stamp + delta, 1_000_000_000)
        sample.role = config.tracker_role; sample.tracker_id = 'a' * 64
        sample.pose_valid = True; sample.tracking_status = 2
        sample.host_realtime_ns = stamp + delta; sample.host_monotonic_ns = stamp + delta + 2_000_000_000
        sample.pose.position.x, sample.pose.position.y, sample.pose.position.z = tracker_pose.translation.tolist()
        sample.pose.orientation.x, sample.pose.orientation.y, sample.pose.orientation.z, sample.pose.orientation.w = tracker_pose.quaternion_xyzw.tolist()
        tracker.append(sample)
    return image, info, timing, tracker


def setup_config(tmp_path):
    document = build_config_document(camera_name='d436', tracker_role='torso',
                                     square_length_mm=40, marker_length_mm=30)
    # This synthetic scene projects an explicit legacy 0.36 x 0.24 m board.
    # A product default change must not silently rescale its known geometry.
    document['board'].update(squares_x=9, squares_y=6, dictionary='DICT_5X5_1000')
    path = write_config(tmp_path / 'config.yaml', document)
    return path, load_config(path)


def test_real_ros_preview_accepts_static_pose_and_recorder_closes(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from vt_camera_msgs.msg import CameraFrameTiming
    from vt_tracker_msgs.msg import TrackerSample
    config_path, config = setup_config(tmp_path)
    context = Context(); context.init(args=[])
    node = Node('guided_test_source', context=context)
    publishers = [node.create_publisher(kind, topic, 100) for kind, topic in (
        (Image, config.image_topic), (CameraInfo, config.camera_info_topic),
        (CameraFrameTiming, config.timing_topic), (TrackerSample, config.tracker_sample_topic))]
    monitor = GuidedMonitor(config)
    recorder = ManagedJob()
    try:
        monitor.start()
        recorder.start(record_command(config, tmp_path / 'recorded'), tmp_path / 'recorder.log')
        deadline = time.monotonic() + 15
        while (any(p.get_subscription_count() < 2 for p in publishers)
               and time.monotonic() < deadline):
            time.sleep(.05)
        assert all(p.get_subscription_count() >= 2 for p in publishers)
        for number in range(3):
            image, info, timing, samples = messages(config, number, (10 + number) * 1_000_000_000)
            for sample in samples:
                publishers[3].publish(sample)
            publishers[1].publish(info); publishers[2].publish(timing); publishers[0].publish(image)
            time.sleep(.25)
            monitor.detect_latest()
        snapshot = monitor.snapshot()
        assert not snapshot['detail'], snapshot
        assert snapshot['accepted'] >= 2
        assert snapshot['preview']['source_stamp_ns'] == 12_000_000_000
    finally:
        recorder.stop()
        deadline = time.monotonic() + 15
        while recorder.running and time.monotonic() < deadline:
            time.sleep(.05)
        monitor.stop()
        node.destroy_node(); context.try_shutdown()
    assert not recorder.running
    assert (tmp_path / 'recorded/metadata.yaml').exists()


@pytest.mark.parametrize("run_count", [1, 3])
def test_real_mcap_offline_worker_detects_solves_and_reserves_validation(tmp_path, run_count):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    config_path, config = setup_config(tmp_path)
    bags = []
    for run_index in range(run_count):
        bag = tmp_path / f'synthetic-bag-{run_index}'
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        for index, (topic, type_name) in enumerate(((config.image_topic, 'sensor_msgs/msg/Image'),
            (config.camera_info_topic, 'sensor_msgs/msg/CameraInfo'),
            (config.timing_topic, 'vt_camera_msgs/msg/CameraFrameTiming'),
            (config.tracker_sample_topic, 'vt_tracker_msgs/msg/TrackerSample'))):
            writer.create_topic(rosbag2_py.TopicMetadata(id=index, name=topic, type=type_name, serialization_format='cdr'))
        for number in range(65):
            stamp = (100 + run_index * 100 + number) * 1_000_000_000
            image, info, timing, samples = messages(config, number + run_index * 100, stamp)
            for sample in samples:
                writer.write(config.tracker_sample_topic, serialize_message(sample), sample.host_realtime_ns)
            writer.write(config.camera_info_topic, serialize_message(info), stamp)
            writer.write(config.timing_topic, serialize_message(timing), stamp)
            writer.write(config.image_topic, serialize_message(image), stamp)
        del writer
        bags.append(bag)
    request = Request.from_document(dict(schema_version=1, session_id='synthetic', mode='offline',
        config_path=str(config_path), output_root=str(tmp_path), bag_paths=[str(path) for path in bags]))
    if run_count == 3:
        request_path = tmp_path / 'request.json'
        request_path.write_text(json.dumps(dict(schema_version=1, session_id=request.session_id,
            mode=request.mode, config_path=str(request.config_path), output_root=str(request.output_root),
            bag_paths=[str(path) for path in request.bag_paths])))
        completed = subprocess.run([sys.executable, '-m', 'vt_tracker_camera_calib.guided_worker',
            '--request', str(request_path), '--status', str(tmp_path / 'status.json'),
            '--cancel', str(tmp_path / 'cancel')], capture_output=True, text=True, timeout=60)
        assert completed.returncode == 0, completed.stderr + (tmp_path / 'status.json').read_text()
    else:
        worker = Worker(request, tmp_path / 'status.json', tmp_path / 'cancel')
        assert worker.run() == 0, (tmp_path / 'status.json').read_text()
    status = json.loads((tmp_path / 'status.json').read_text())
    assert status['hint_code'] == ('CANDIDATE' if run_count == 1 else 'READY')
    manifest = json.loads(Path(status['result']['manifest_path']).read_text())
    assert manifest['activation_eligible'] is (run_count == 3)
    validation = manifest['rounds'][0]['validation']
    assert validation['status'] == 'PASS'
    assert validation['training_count'] >= 40
    assert validation['validation_count'] >= 10
    assert validation['translation_rms_m'] < .005
    assert validation['rotation_rms_deg'] < .5


def test_real_guided_worker_cancel_closes_only_owned_recorder(tmp_path):
    from rclpy.context import Context
    from rclpy.node import Node
    from sensor_msgs.msg import Image, CameraInfo
    from vt_camera_msgs.msg import CameraFrameTiming
    from vt_tracker_msgs.msg import TrackerSample
    config_path, config = setup_config(tmp_path)
    request_path = tmp_path / 'request.json'
    request_path.write_text(json.dumps(dict(schema_version=1, session_id='cancel-test', mode='guided',
        config_path=str(config_path), output_root=str(tmp_path), bag_paths=[])))
    context = Context(); context.init(args=[])
    node = Node('guided_cancel_test_camera', context=context)
    publishers = [node.create_publisher(kind, topic, 100) for kind, topic in (
        (Image, config.image_topic), (CameraInfo, config.camera_info_topic),
        (CameraFrameTiming, config.timing_topic), (TrackerSample, config.tracker_sample_topic))]
    process = subprocess.Popen([sys.executable, '-m', 'vt_tracker_camera_calib.guided_worker',
        '--request', str(request_path), '--status', str(tmp_path / 'status.json'),
        '--cancel', str(tmp_path / 'cancel')], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and (not (tmp_path / 'recorder-lease.json').exists()
                or any(p.get_subscription_count() < 2 for p in publishers)):
            assert process.poll() is None, process.stderr.read().decode()
            time.sleep(.05)
        assert (tmp_path / 'recorder-lease.json').exists()
        for number in range(3):
            image, info, timing, samples = messages(config, number, (10 + number) * 1_000_000_000)
            for sample in samples:
                publishers[3].publish(sample)
            publishers[1].publish(info); publishers[2].publish(timing); publishers[0].publish(image)
            time.sleep(.25)
        (tmp_path / 'cancel').touch()
        _, error = process.communicate(timeout=15)
        assert process.returncode == 130, error.decode()
        assert json.loads((tmp_path / 'status.json').read_text())['state'] == 'CANCELLED'
        assert not (tmp_path / 'recorder-lease.json').exists()
        assert (tmp_path / 'round-1/bag/metadata.yaml').is_file()
        # The independent synthetic camera/Tracker source remains operational.
        assert context.ok()
        publishers[0].publish(image)
    finally:
        if process.poll() is None:
            (tmp_path / 'cancel').touch()
            process.wait(timeout=15)
        node.destroy_node(); context.try_shutdown()
