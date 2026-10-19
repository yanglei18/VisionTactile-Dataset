"""Synthetic ROS only. Run with an isolated ROS_DOMAIN_ID and local discovery."""
import os
import socket
import time
import uuid

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("VT_CAPTURE_MANAGER_ROS_TEST") != "1",
                              reason="opt-in synthetic ROS test requires an isolated ROS domain")


def test_tactile_recording_activity_observes_raw_samples_separately_from_state():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from vt_tactile_msgs.msg import TactileSampleArray, TactileState
    from vt_capture_manager.config import DeviceConfig, ManagerConfig
    from vt_capture_manager.model import SnapshotModel
    from vt_capture_manager.node import build_node

    prefix = "/manager_tactile_activity_" + uuid.uuid4().hex
    config = ManagerConfig((DeviceConfig("touch", "touch", "tactile", "right", prefix + "/state"),))
    monotonic = [10_000_000_000]
    model = SnapshotModel(config, monotonic_ns=lambda: monotonic[0])
    context = Context()
    rclpy.init(context=context)
    manager = build_node(config, context=context, model=model)
    source = Node("synthetic_tactile_activity", context=context,
                  enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(manager)
    executor.add_node(source)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    state = source.create_publisher(TactileState, prefix + "/state", qos)
    samples = source.create_publisher(TactileSampleArray, prefix + "/samples", qos)

    def until(predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail("synthetic tactile activity was not observed")

    try:
        sample_subscription = next(item for item in manager.subscriptions
                                   if item.topic_name == prefix + "/samples")
        assert sample_subscription.raw
        assert sample_subscription.qos_profile.depth == 1
        assert sample_subscription.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        until(lambda: state.get_subscription_count() == samples.get_subscription_count() == 1)
        state.publish(TactileState())  # An invalid heartbeat cannot stand in for raw data.
        until(lambda: model.snapshot()["devices"][0]["receive_hz"] >= 1)
        assert model.topic_activity() == {prefix + "/samples": False}
        samples.publish(TactileSampleArray())  # Invalid payload is still recordable evidence.
        until(lambda: model.topic_activity()[prefix + "/samples"])
        assert not model.topic_health()[prefix + "/state"]
        monotonic[0] += 2_000_000_000
        state.publish(TactileState())
        until(lambda: model.snapshot()["devices"][0]["age_s"] == 0)
        assert model.topic_activity() == {prefix + "/samples": False}
    finally:
        executor.remove_node(manager)
        executor.remove_node(source)
        manager.destroy_node()
        source.destroy_node()
        executor.shutdown()
        context.shutdown()


@pytest.mark.parametrize("image_reliability", ["best_effort", "reliable"])
def test_synthetic_ros_to_unix_snapshot(tmp_path, image_reliability):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from vt_camera_msgs.msg import CaptureStatus
    from vt_glove_msgs.msg import GloveState
    from vt_tactile_msgs.msg import TactileState
    from vt_tracker_msgs.msg import TrackerSample
    from vt_capture_core import encode_message, decode_message
    from vt_capture_manager.config import DeviceConfig, ManagerConfig
    from vt_capture_manager.node import build_node
    from vt_capture_manager.server import SnapshotServer

    prefix = "/manager_synthetic_" + uuid.uuid4().hex
    kinds = ("camera", "tracker", "glove", "tactile")
    ids = ("camera", "right_wrist", "glove", "tactile")
    config = ManagerConfig(tuple(DeviceConfig(identity, identity, kind, "right", prefix+"/"+kind,
                                 image_reliability=image_reliability if kind == "camera" else "best_effort")
                                 for identity, kind in zip(ids, kinds)), capture_status_topic=prefix+"/status")
    context = Context()
    rclpy.init(context=context)
    manager = build_node(config, context=context)
    publisher_node = Node("synthetic_manager_sources", context=context, enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(manager)
    executor.add_node(publisher_node)
    types = (Image, TrackerSample, GloveState, TactileState)
    publishers = []
    for kind, cls in zip(kinds, types):
        reliable = kind == "glove" or (kind == "camera" and image_reliability == "reliable")
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        publishers.append(publisher_node.create_publisher(cls, prefix+"/"+kind, qos))
    status_publisher = publisher_node.create_publisher(CaptureStatus, prefix+"/status", QoSProfile(depth=5,
        reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
    server = SnapshotServer(manager.model, tmp_path/"private"/"manager.sock").start()
    try:
        deadline = time.monotonic()+8
        while any(p.get_subscription_count() != 1 for p in publishers+[status_publisher]) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert all(p.get_subscription_count() == 1 for p in publishers+[status_publisher])
        realtime = time.time_ns()
        mono = time.monotonic_ns()

        def stamp(message):
            message.header.stamp.sec = realtime//1_000_000_000
            message.header.stamp.nanosec = realtime%1_000_000_000
            return message

        image = stamp(Image(width=8, height=8, step=24, encoding="rgb8", data=bytes([255, 0, 0])*64))
        tracker = stamp(TrackerSample(role="right_wrist", tracker_id="synthetic", host_realtime_ns=realtime,
                                      host_monotonic_ns=mono, pose_valid=True, tracking_status=2))
        tracker.header.frame_id = "vive_map"
        tracker.pose.position.x = 1.
        tracker.pose.orientation.w = 1.
        glove = stamp(GloveState(schema_version=1, hand="right", session_id="synthetic", pose_valid=True,
                                time_valid=True, host_realtime_ns=realtime, host_monotonic_ns=mono,
                                mano_pose_rotvec=[0.]*48, shape_params=[0.]*10, root_quat_xyzw=[0.,0.,0.,1.],
                                joint_positions_m=[.01]*63, quality="valid", frame_index=1))
        glove.header.frame_id = "glove_right_reference"
        tactile = stamp(TactileState(schema_version=2, hand="right", session_id="synthetic", rows=20, cols=20,
            adc=[0]*400, valid_mask=[True]*400, block_realtime_ns=[realtime]*160, connected=True, time_valid=True,
            host_realtime_ns=realtime, host_monotonic_ns=mono, max_age_ns=1_000_000_000, stream_epoch=1, source_sequence=1))
        capture = stamp(CaptureStatus(state=CaptureStatus.IDLE, recorder_alive=False, session_id="", detail="synthetic idle"))
        for publisher, message in zip(publishers, (image, tracker, glove, tactile)):
            publisher.publish(message)
        status_publisher.publish(capture)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            snapshot = manager.model.snapshot()
            if (all(d["receive_hz"] >= 1 for d in snapshot["devices"]) and
                    snapshot["devices"][0]["preview"] and snapshot["recording"]["state"] == "IDLE"):
                break
        assert all(d["status"] == "online" for d in snapshot["devices"])
        assert snapshot["devices"][0]["preview"]["format"] == "jpeg"
        assert snapshot["devices"][1]["preview"]["position"] == [1., 0., 0.]
        assert len(snapshot["devices"][2]["preview"]["joints"]) == 21
        assert all(snapshot["devices"][3]["preview"]["valid_mask"])
        assert snapshot["recording"]["state"] == "IDLE"
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(2)
            client.connect(str(server.path))
            client.sendall(encode_message(dict(version=1, request_id="e2e", command="snapshot")))
            data = bytearray()
            while not data.endswith(b"\n"):
                data.extend(client.recv(65536))
        result = decode_message(bytes(data))
        assert result["ok"] and result["snapshot"]["capabilities"] == dict(record=False, device_control=False)
        # Manager has no device/recorder publishers or control services.
        assert all(p.topic_name == "/parameter_events" for p in manager.publishers)
        assert all(service.srv_name.endswith("/get_type_description") for service in manager.services)
    finally:
        server.close()
        executor.remove_node(manager)
        executor.remove_node(publisher_node)
        manager.destroy_node()
        publisher_node.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_slow_image_callback_does_not_block_other_images_or_recorder(monkeypatch):
    """Exercise the production executor with a stalled preview handoff, no devices."""
    rclpy = pytest.importorskip("rclpy")
    from threading import Event
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from vt_camera_msgs.msg import CaptureStatus
    from vt_camera_msgs.srv import GetCaptureSnapshot
    from vt_capture_manager.config import DeviceConfig, ManagerConfig
    from vt_capture_manager.node import build_node, build_executor
    from vt_capture_manager.recorder_adapter import ManagedRosAdapter

    prefix = "/manager_concurrent_" + uuid.uuid4().hex
    config = ManagerConfig(tuple(DeviceConfig(name, name, "camera", "", prefix+"/"+name,
                                             image_reliability="reliable") for name in ("slow", "fast")),
                           capture_status_topic=prefix+"/status")
    context = Context()
    rclpy.init(context=context)
    manager = build_node(config, context=context)
    source = Node("manager_concurrent_sources", context=context, enable_rosout=False, start_parameter_services=False)
    executor = build_executor(manager)
    executor.add_node(source)
    qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    slow = source.create_publisher(Image, prefix+"/slow", qos)
    fast = source.create_publisher(Image, prefix+"/fast", qos)
    status = source.create_publisher(CaptureStatus, prefix+"/status", qos)

    def snapshot_callback(_request, response):
        response.context.controller_instance_id = "synthetic-concurrent-controller"
        return response

    source.create_service(GetCaptureSnapshot, prefix+"/snapshot", snapshot_callback)
    adapter = ManagedRosAdapter(manager, snapshot_service=prefix+"/snapshot", command_service=prefix+"/command")
    blocked, release = Event(), Event()
    submit = manager._preview_worker.submit

    def slow_submit(identity, message):
        if identity == "slow":
            blocked.set()
            release.wait(10.)
        else:
            submit(identity, message)

    monkeypatch.setattr(manager._preview_worker, "submit", slow_submit)

    def until(predicate, seconds=5.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        pytest.fail("independent ROS callback did not complete")

    def image():
        message = Image(width=8, height=8, step=24, encoding="rgb8", data=bytes([255, 0, 0])*64)
        message.header.stamp = source.get_clock().now().to_msg()
        return message

    try:
        until(lambda: all(p.get_subscription_count() == 1 for p in (slow, fast, status))
              and adapter._snapshot_client.service_is_ready(), seconds=8.)
        slow.publish(image())
        until(blocked.is_set)
        fast.publish(image())
        capture = CaptureStatus(state=CaptureStatus.IDLE, recorder_alive=False, detail="synthetic idle")
        capture.header.stamp = source.get_clock().now().to_msg()
        status.publish(capture)
        pending = adapter.get_snapshot()
        until(lambda: pending.done() and manager.model.snapshot()["devices"][1]["preview"]
              and manager.model.snapshot()["recording"]["state"] == "IDLE")
        assert not release.is_set()
        assert pending.result()["controller_instance_id"] == "synthetic-concurrent-controller"
    finally:
        release.set()
        executor.shutdown()
        manager.destroy_node()
        source.destroy_node()
        context.shutdown()


def test_three_trackers_share_source_world_in_scene_socket(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from vt_tracker_msgs.msg import TrackerSample
    from vt_capture_manager.config import DeviceConfig, ManagerConfig
    from vt_capture_manager.node import build_node
    from vt_capture_manager.spatial.scene_server import SceneServer
    from vt_capture_core.spatial.scene_protocol import encode_scene_message, decode_scene_message
    scene_config = tmp_path/'scene.yaml'
    scene_config.write_text('schema_version: 1\nenabled: true\n')
    roles = ('left_wrist', 'right_wrist', 'torso')
    prefix = '/scene_synthetic_' + uuid.uuid4().hex
    config = ManagerConfig(tuple(DeviceConfig(role, role, 'tracker', '', prefix+'/'+role) for role in roles),
                           scene_config_path=str(scene_config))
    context = Context()
    rclpy.init(context=context)
    manager = build_node(config, context=context)
    publisher = Node('synthetic_scene_sources', context=context, enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(manager)
    executor.add_node(publisher)
    publishers = [publisher.create_publisher(TrackerSample, prefix+'/'+role, qos_profile_sensor_data) for role in roles]
    server = SceneServer(manager.scene_store, tmp_path/'private'/'manager-scene.sock').start()
    try:
        deadline = time.monotonic()+8
        while any(p.get_subscription_count() != 1 for p in publishers) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert all(p.get_subscription_count() == 1 for p in publishers)
        realtime, mono = time.time_ns(), time.monotonic_ns()
        for index, (role, pub) in enumerate(zip(roles, publishers)):
            msg = TrackerSample(role=role, tracker_id='synthetic-'+role, host_realtime_ns=realtime,
                                host_monotonic_ns=mono, pose_valid=True, tracking_status=2)
            msg.header.frame_id = 'vive_map'
            msg.pose.position.x = float(index+1)
            msg.pose.orientation.w = 1.
            pub.publish(msg)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
            scene = manager.scene_store.snapshot()
            if all(t['valid'] for t in scene['trackers']):
                break
        assert [t['position_m'][0] for t in scene['trackers']] == [1., 2., 3.]
        time.sleep(.06)  # Allow one scene producer interval, never a ROS wait.
        with socket.socket(socket.AF_UNIX) as client:
            client.settimeout(1.)
            client.connect(str(server.path))
            client.sendall(encode_scene_message(dict(protocol='vt.scene', version=1, request_id='ros-scene',
                          command='latest', params=dict(viewer_id='test', interest=['tracker']))))
            data = bytearray()
            while not data.endswith(b'\n'):
                part = client.recv(65536)
                assert part
                data.extend(part)
        scene = decode_scene_message(bytes(data))['scene']
        assert scene['manager_instance_id'] == manager.model.instance_id
        assert scene['scene_time_ns'] is None
        assert [t['position_m'][0] for t in scene['trackers']] == [1., 2., 3.]
        assert all(p.topic_name == '/parameter_events' for p in manager.publishers)
        assert all(s.srv_name.endswith('/get_type_description') for s in manager.services)
    finally:
        server.close()
        executor.remove_node(manager)
        executor.remove_node(publisher)
        manager.destroy_node()
        publisher.destroy_node()
        executor.shutdown()
        context.shutdown()


@pytest.mark.parametrize('cloud_enabled', [False, True])
def test_camera_pose_requires_read_only_matching_device_info(tmp_path, cloud_enabled):
    rclpy = pytest.importorskip('rclpy')
    import yaml
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from realsense2_camera_msgs.srv import DeviceInfo
    from realsense2_camera_msgs.msg import Extrinsics
    from vt_tracker_msgs.msg import TrackerSample
    from vt_camera_msgs.msg import CameraFrameTiming
    from vt_capture_manager.config import DeviceConfig, ManagerConfig
    from vt_capture_manager.node import build_node
    name = 'synthetic_camera_'+uuid.uuid4().hex[:8]
    camera = dict(name=name, model='D405', serial='SYNTHETIC-123', tracker_role='torso',
                  frame_id=name+'_color_optical_frame', depth_frame_id=name+'_depth_optical_frame',
                  color_topic='/'+name+'/color/image_raw', depth_topic='/'+name+'/depth/image_rect_raw',
                  camera_info_topic='/'+name+'/color/camera_info', timing_topic='/'+name+'/frame_timing')
    (tmp_path/'alignment.yaml').write_text(yaml.safe_dump(dict(schema_version=2, cameras=[camera], trackers=[dict(role='torso', frame_id='vive_tracker_torso', sample_topic='/'+name+'/tracker')], frames={'world':'vive_map'})))
    transform = dict(semantics='parent_from_child', parent_frame='vive_tracker_torso', child_frame=camera['frame_id'],
                     translation_m=[.1, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.])
    (tmp_path/(name+'.yaml')).write_text(yaml.safe_dump(dict(schema_version=1, status='VALID', transform=transform,
        camera=dict(name=name, model='D405', serial='SYNTHETIC-123', frame_id=camera['frame_id']),
        tracker=dict(role='torso', tracker_id='a'*64, frame_id='vive_tracker_torso'), frames=dict(world='vive_map'))))
    scene_path = tmp_path/'scene.yaml'
    scene_path.write_text(yaml.safe_dump(dict(schema_version=1, alignment_config_path='alignment.yaml', extrinsics_directory='.',
        camera_device_info_services={name:'/'+name+'/device_info'},
        depth_units={name:dict(meters_per_unit=.001, source='realsense_ros_z16_mm')},
        cloud=dict(enabled=cloud_enabled, stride=4, hz=10))))
    config = ManagerConfig((DeviceConfig('torso', 'torso', 'tracker', '', '/'+name+'/tracker'),), scene_config_path=str(scene_path))
    context = Context()
    rclpy.init(context=context)
    manager = build_node(config, context=context)
    source = Node(name, context=context, enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(manager)
    executor.add_node(source)
    publishers = [source.create_publisher(Image, camera[key], qos_profile_sensor_data) for key in ('color_topic', 'depth_topic')]
    color_info = source.create_publisher(CameraInfo, camera['camera_info_topic'], qos_profile_sensor_data)
    depth_info = source.create_publisher(CameraInfo, '/'+name+'/depth/camera_info', qos_profile_sensor_data)
    tracker_pub = source.create_publisher(TrackerSample, '/'+name+'/tracker', qos_profile_sensor_data)
    extrinsics = source.create_publisher(Extrinsics, '/'+name+'/extrinsics/depth_to_color',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def identify(request, response):
        response.serial_number = 'SYNTHETIC-123'
        response.device_name = 'Intel RealSense D405'
        response.firmware_version = 'synthetic'
        return response
    service = source.create_service(DeviceInfo, '/'+name+'/device_info', identify)
    timing_pub = source.create_publisher(CameraFrameTiming, camera['timing_topic'], qos_profile_sensor_data)
    supervisor = None
    if cloud_enabled:
        from vt_capture_manager.spatial.cloud_supervisor import CloudSupervisor
        supervisor = CloudSupervisor(manager.scene_store, tmp_path/'private-cloud'/'manager-cloud.sock').start()
        manager.scene_store.cloud_supervisor = supervisor
    frame_index = 0
    try:
        deadline = time.monotonic()+12
        camera_entity = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.002)
            manager.scene_store.set_interest('integration-test', ['tracker', 'camera', 'cloud'] if cloud_enabled else ['camera'])
            for publisher, frame in ((color_info, camera['frame_id']), (depth_info, camera['depth_frame_id'])):
                info = CameraInfo(width=100, height=80, distortion_model='plumb_bob', d=[0.]*5,
                    k=[100.,0.,50.,0.,100.,40.,0.,0.,1.], r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                    p=[100.,0.,50.,0.,0.,100.,40.,0.,0.,0.,1.,0.])
                info.header.frame_id = frame
                publisher.publish(info)
            msg = TrackerSample(role='torso', tracker_id='a'*64, host_realtime_ns=time.time_ns(),
                host_monotonic_ns=time.monotonic_ns(), pose_valid=True, tracking_status=2)
            msg.header.frame_id = 'vive_map'
            msg.pose.position.x = 1.
            msg.pose.orientation.w = 1.
            tracker_pub.publish(msg)
            extrinsics.publish(Extrinsics(rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.], translation=[.02,0.,0.]))
            frame_index += 1
            if frame_index % 4 == 0:
                stamp = time.time_ns()
                for publisher, encoding, frame, pixels, step in (
                        (publishers[0], 'rgb8', camera['frame_id'], bytes([30, 100, 220])*8000, 300),
                        (publishers[1], '16UC1', camera['depth_frame_id'], b'\xe8\x03'*8000, 200)):
                    image = Image(width=100, height=80, encoding=encoding, data=pixels, step=step)
                    image.header.frame_id = frame
                    image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                    publisher.publish(image)
                timing = CameraFrameTiming(camera_name=name, camera_model='D405', serial_number='SYNTHETIC-123',
                    shared_ros_timestamp_ns=stamp, group_host_realtime_ns=stamp,
                    group_host_monotonic_raw_ns=time.monotonic_ns(), group_validity_flags=31,
                    color_frame_number=frame_index, depth_frame_number=frame_index)
                timing.header.frame_id = name+'_frame_group'
                timing.header.stamp.sec, timing.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                timing_pub.publish(timing)
            camera_entity = manager.scene_store.snapshot()['cameras'][0]
            if camera_entity['position_m'] is not None and (not cloud_enabled or supervisor.latest_headers()):
                break
        assert camera_entity['position_m'] == pytest.approx([1.1, 0., 0.]), camera_entity
        assert len(camera_entity['frustum_vertices']) == 8
        assert manager.scene_store.camera_registry.geometry(name)['depth_to_color']['translation_m'] == [.02, 0., 0.]
        if supervisor is not None:
            from vt_capture_core.spatial.cloud_protocol import PREFIX_BYTES, inspect_cloud_prefix, unpack_cloud_frame
            assert supervisor.latest_headers(), supervisor.status()
            descriptor = supervisor.description()
            assert descriptor and descriptor['worker_pid'] != os.getpid()
            with socket.socket(socket.AF_UNIX) as client:
                client.settimeout(2.)
                client.connect(descriptor['socket_path'])
                def receive(size):
                    data = b''
                    while len(data) < size:
                        part = client.recv(size-len(data))
                        assert part
                        data += part
                    return data
                prefix = receive(PREFIX_BYTES)
                h, payload = inspect_cloud_prefix(prefix)
                cloud = unpack_cloud_frame(prefix+receive(h+payload))
            assert cloud.header['point_count'] == 500
            import struct
            assert struct.unpack('<3f', cloud.xyz_bytes[:12]) == pytest.approx((-.48, -.4, 1.))
            assert cloud.rgba_bytes[:3] == bytes([30, 100, 220])
            assert cloud.header['world_from_color']['translation_m'] == pytest.approx([1.1, 0., 0.])
            assert cloud.header['manager_instance_id'] == manager.model.instance_id
            assert cloud.header['worker_instance_id'] == descriptor['worker_instance_id']
            scene = manager.scene_store.snapshot()
            assert scene['time_mode'] == 'aligned'
            assert scene['cameras'][0]['frame_key']['reference_camera'] == name
            assert len(scene['cameras'][0]['frame_key']) >= 3
        assert all(p.topic_name == '/parameter_events' for p in manager.publishers)
        assert all(s.srv_name.endswith('/get_type_description') for s in manager.services)
    finally:
        if supervisor is not None:
            supervisor.close()
        executor.remove_node(manager)
        executor.remove_node(source)
        manager.destroy_node()
        source.destroy_node()
        executor.shutdown()
        context.shutdown()
