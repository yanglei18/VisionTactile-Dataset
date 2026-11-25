"""Camera-relative wrists and joints are placed in the shared map at color capture time."""
from math import sqrt
from types import SimpleNamespace as NS

import pytest

from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_manager.spatial.config import SceneConfig
from vt_capture_manager.spatial.scene_store import SceneStore
from vt_capture_manager.spatial.scene_server import SceneServer
from vt_capture_core.spatial.scene_protocol import encode_scene_message, decode_scene_message

from test_spatial_camera_geometry import camera_config, camera_info, prepared
from test_spatial_manager import Clock, sample

MS = 1_000_000


def configured():
    clock = Clock()
    store = SceneStore('manager', SceneConfig(cameras=(camera_config(),)), realtime_ns=clock, monotonic_ns=clock)
    registry = prepared()
    registry.observe_device_info('cam', NS(serial_number='123', device_name='D405'), observed_ns=clock.ns)
    store.camera_registry = registry
    return store, clock


def place_tracker(store, clock, time, x, rotation=(0, 0, 0, 1)):
    clock.ns = time
    msg = sample(clock, role='torso', position=(x, 0., 0.), tracker_id='LHR-123')
    msg.pose.orientation = NS(**dict(zip('xyzw', rotation)))
    assert store.observe_tracker('torso', msg)


def optical(store, stamp, index, wrist, tips, hand='left', instance='sensor'):
    sec, ns = divmod(stamp, 10**9)
    return NS(schema_version=1, hand=hand, cube_instance_id=instance, cube_source_image_index=index,
              header=NS(frame_id='cam_color_optical_frame', stamp=NS(sec=sec, nanosec=ns)),
              camera_name='cam', camera_serial='123', color_host_realtime_ns=stamp,
              color_frame_number=index, intrinsics_sha256=store.camera_registry.optical_registration('cam').intrinsics,
              status='tracked', wrist_valid=True, joints_valid=True, layout_sha256='e'*64, mount_sha256='f'*64,
              wrist=NS(position=NS(**dict(zip('xyz', wrist.translation_m))),
                       orientation=NS(**dict(zip('xyzw', wrist.quaternion_xyzw)))),
              joint_positions_m=[v for point in tips for v in point], glove_model_rest_sha256='a'*64,
              glove_session_id='session', glove_config_sha256='b'*64, glove_calibration_version=1)


def hand(store, side='left'):
    return next(h for h in store.snapshot(include_hands=True)['hands'] if h['hand'] == side)


def test_head_motion_cancels_for_wrist_and_asymmetric_fingers_at_source_time():
    store, clock = configured()
    start = clock.ns
    wrist_world = RigidTransform((.2,.1,.6), (0.,0.,0.,1.))
    joints = [wrist_world.apply((i*.002, (i%5)*.01, -(i%3)*.001)) for i in range(21)]
    calibration = store.camera_registry.optical_registration('cam').tracker_from_color
    place_tracker(store, clock, start, 0.)
    for index, x in [(1,0.),(2,.1)]:
        if index == 1:
            target = start
            world_from_tracker = RigidTransform.identity()
        else:
            # Make the newest camera pose quite different from the correct, interpolated one.
            place_tracker(store, clock, start+40*MS, 2*x, (0.,0.,sqrt(.5),sqrt(.5)))
            target = start+20*MS
            world_from_tracker = RigidTransform.identity().interpolate(
                RigidTransform((2*x,0,0), (0.,0.,sqrt(.5),sqrt(.5))), .5)
        camera_from_world = world_from_tracker.compose(calibration).inverse()
        msg = optical(store, target, index, camera_from_world.compose(wrist_world),
                      [camera_from_world.apply(j) for j in joints])
        store.observe_hand('left', msg)
        result = hand(store)
        assert result['valid'] and result['source_realtime_ns'] == target
        assert result['position_m'] == pytest.approx(wrist_world.translation_m)
        assert result['quaternion_xyzw'] == pytest.approx(wrist_world.quaternion_xyzw)
        for actual, expected in zip(result['joint_positions_m'], joints, strict=True):
            assert actual == pytest.approx(expected)


def test_clock_timestamp_visibility_geometry_and_per_side_restart():
    store, clock = configured()
    now = clock.ns
    place_tracker(store, clock, now, .3)
    wrist = RigidTransform((.1, 0., .4), (0,0,0,1))
    def msg(side, i, **changes):
        m = optical(store, now, i, wrist, [wrist.translation_m]*21, side)
        m.__dict__.update(changes)
        return m
    store.observe_hand('left', msg('left', 1))
    store.observe_hand('right', msg('right', 1, joints_valid=False, joint_positions_m=[], status='glove_gap'))
    assert hand(store, 'right')['valid'] and not hand(store, 'right')['joint_positions_m']
    store.observe_hand('left', msg('left', 2, wrist_valid=False, status='cube_lost'))
    store.observe_hand('left', msg('left', 1))
    assert not hand(store)['valid'] and not hand(store)['joint_positions_m']
    assert hand(store, 'right')['valid']
    store.observe_hand('left', msg('left', 3, color_host_realtime_ns=0))
    assert not hand(store)['valid']
    store.observe_hand('left', msg('left', 4, camera_serial='wrong'))
    assert not hand(store)['valid']
    store.observe_hand('left', msg('left', 1, cube_instance_id='new'))
    assert hand(store)['valid']
    store.observe_hand('left', msg('left', 9, wrist_valid=False, status='cube_lost'))
    assert hand(store)['valid']
    clock.ns = now+10*MS
    store.observe_hand('left', msg('left', 2, color_host_realtime_ns=clock.ns, cube_instance_id='new'))
    assert 'Tracker' in hand(store)['reason'] and not hand(store)['valid']
    place_tracker(store, clock, now+20*MS, .3)
    assert hand(store)['valid']
    clock.ns += 251*MS
    assert hand(store)['state'] == 'offline' and hand(store)['joint_positions_m'] == []
    store.camera_registry.observe_camera_info('cam', camera_info(fx=105.))
    store.observe_hand('right', msg('right', 2, color_host_realtime_ns=clock.ns, intrinsics_sha256='c'*64))
    assert not hand(store, 'right')['valid']


def test_synchronized_cloud_requires_same_color_frame_and_never_revives_lost_hand():
    store, clock = configured()
    now = clock.ns
    place_tracker(store, clock, now, 0.)
    wrist = RigidTransform((.1,0.,.4), (0,0,0,1))
    for index in range(1,4):
        place_tracker(store, clock, now+index*MS, 0.)
        store.observe_hand('left', optical(store, clock.ns, index, wrist, [wrist.translation_m]*21))
    config = store.camera_registry.geometry('cam')
    frame_key = dict(scene_time_ns=now+MS, shared_ros_timestamp_ns=now+MS, color_frame_number=1, reference_camera='cam')
    cloud = dict(manager_instance_id='manager', scene_epoch=store.scene_epoch, coordinate_epoch=0, time_epoch=0,
                 sync_epoch=0, geometry_revision=config['geometry_revision'], geometry_sha256=config['geometry_sha256'],
                 source_realtime_ns=now+MS, frame_key=frame_key, world_from_color=RigidTransform.identity().as_dict())
    store.cloud_supervisor = NS(latest_headers=lambda: {'cam':cloud})
    store.set_interest('test',['cloud','hand'])
    assert hand(store)['source_realtime_ns'] == now+MS and hand(store)['valid']
    frame_key['color_frame_number'] = 999
    assert not hand(store)['valid'] and hand(store)['joint_positions_m'] == []
    frame_key['color_frame_number'] = 1
    lost = optical(store,now+4*MS,4,wrist,[])
    lost.wrist_valid, lost.joints_valid, lost.status = False,False,'cube_lost'
    store.observe_hand('left', lost)
    assert not hand(store)['valid']


def test_hands_wire_is_negotiated_and_legacy_response_has_no_new_fields(tmp_path):
    store, _ = configured()
    server = SceneServer(store, tmp_path/'unused.sock')
    def request(cmd, params):
        return decode_scene_message(server._reply(encode_scene_message(
            dict(protocol='vt.scene', version=1, request_id='r', command=cmd, params=params))))
    assert 'hand' in request('hello', {})['hello']['capabilities']
    assert 'hands' not in request('latest', {'interest':['camera']})['scene']
    result = request('latest', {'interest':['hand']})['scene']['hands']
    assert [h['hand'] for h in result] == ['left', 'right']
    assert all(not h['valid'] for h in result)
