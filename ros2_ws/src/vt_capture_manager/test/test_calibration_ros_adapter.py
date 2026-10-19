"""Pure identity and publication conversion tests."""
from types import SimpleNamespace as NS
import pytest

from vt_capture_manager.calibration.ros_adapter import CalibrationIdentity, calibration_message
from vt_capture_manager.spatial.camera_geometry import CameraGeometryRegistry


def camera():
    return dict(id='cam', serial='123', model='D405', tracker_role='torso', color_frame_id='cam_color_optical_frame')


def document():
    return dict(schema_version=1, status='VALID', camera=dict(name='cam', serial='123', model='D405', frame_id='cam_color_optical_frame'),
                tracker=dict(role='torso', tracker_id='a' * 64, frame_id='vive_tracker_torso'),
                frames=dict(world='vive_map'), transform=dict(semantics='parent_from_child', parent_frame='vive_tracker_torso',
                child_frame='cam_color_optical_frame', translation_m=[.1, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]),
                quality=dict(grade='TARGET'))


def sample():
    return NS(role='torso', tracker_id='a' * 64, header=NS(frame_id='vive_map'), host_realtime_ns=1000000000,
              host_monotonic_ns=1000000000, pose_valid=True, tracking_status=2,
              pose=NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.)))


def test_identity_requires_fresh_physical_camera_and_matching_live_tracker():
    registry = CameraGeometryRegistry([camera()])
    identity = CalibrationIdentity(registry, [dict(role='torso', frame_id='vive_tracker_torso')], clock=lambda: 1000000000)
    assert identity.identity(document()) == (False, 'camera_identity_unverified')
    registry.observe_device_info('cam', NS(serial_number='123', device_name='D405'), observed_ns=1000000000)
    assert identity.identity(document()) == (False, 'tracker_unavailable')
    identity.observe_tracker('torso', sample())
    assert identity.expected_tracker_id('torso') == 'a' * 64
    assert identity.identity(document()) == (True, '')
    wrong = sample()
    wrong.tracker_id = 'b' * 64
    identity.observe_tracker('torso', wrong)
    assert identity.identity(document()) == (False, 'tracker_identity_mismatch')


def test_stale_or_future_tracker_identity_is_never_used():
    now = [1000000000]
    registry = CameraGeometryRegistry([camera()])
    identity = CalibrationIdentity(registry, [dict(role='torso', frame_id='vive_tracker_torso')], clock=lambda: now[0])
    identity.observe_tracker('torso', sample())
    now[0] += 2000000000
    assert identity.expected_tracker_id('torso') == ''
    future = sample()
    future.host_realtime_ns = now[0] + 1000000000
    identity.observe_tracker('torso', future)
    assert identity.expected_tracker_id('torso') == ''


def test_invalid_candidate_does_not_remove_last_good_spatial_geometry():
    registry = CameraGeometryRegistry([camera()])
    registry.set_extrinsic('cam', document())
    before = registry.geometry('cam')
    invalid = document()
    invalid['camera']['serial'] = 'other'
    with pytest.raises(ValueError):
        registry.set_extrinsic('cam', invalid)
    assert registry.geometry('cam')['extrinsic'] == before['extrinsic']
    assert registry.geometry('cam')['geometry_revision'] == before['geometry_revision']


class Message:
    def __init__(self):
        self.header = NS(frame_id='', stamp=NS(sec=0, nanosec=0))
        self.tracker_from_camera = NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=0.))


def test_typed_publication_carries_version_identity_validity_and_direction():
    message = calibration_message(Message, document(), dict(revision='r1', sha256='c' * 64, activated_ns=100),
                                  identity_valid=False, reason='tracker_unavailable', now_ns=1234567890)
    assert message.header.frame_id == 'vive_tracker_torso'
    assert message.camera_frame_id == 'cam_color_optical_frame'
    assert message.revision == 'r1' and message.valid is False
    assert message.valid_until_ns == 0
    assert message.tracker_from_camera.translation.x == .1
    assert message.tracker_from_camera.rotation.w == 1.


def test_rebinding_tracker_is_atomic_and_keeps_camera_identity():
    registry = CameraGeometryRegistry([camera()])
    registry.set_extrinsic('cam', document())
    candidate = document()
    candidate['tracker']['role'] = 'left_wrist'
    candidate['tracker']['frame_id'] = 'vive_tracker_left_wrist'
    candidate['transform']['parent_frame'] = 'vive_tracker_left_wrist'
    assert registry.replace_calibration_binding('cam', candidate)
    current = registry.geometry('cam')
    assert current['config']['tracker_role'] == 'left_wrist'
    assert current['config']['serial'] == '123'
    candidate['camera']['serial'] = 'other'
    with pytest.raises(ValueError):
        registry.replace_calibration_binding('cam', candidate)
    assert registry.geometry('cam') == current


def test_tracker_identity_accepts_valid_tracking_mode_with_high_flags():
    identity = CalibrationIdentity(CameraGeometryRegistry([camera()]),
        [dict(role='torso', frame_id='vive_tracker_torso')], clock=lambda: 1000000000)
    flagged = sample()
    flagged.tracking_status = 0x12
    identity.observe_tracker('torso', flagged)
    assert identity.expected_tracker_id('torso') == 'a' * 64


def test_tracker_source_graph_cache_is_bounded_and_invalidates_within_lease():
    from vt_capture_manager.calibration.ros_adapter import TrackerSourceGraph
    now, queries, endpoints = [0.], [], [[object()]]
    def query(topic):
        queries.append(topic)
        return endpoints[0]
    graph = TrackerSourceGraph(query, clock=lambda: now[0], refresh_s=.25)
    for _ in range(100):
        assert graph.valid('/vive/torso/sample')
    assert len(queries) == 1
    endpoints[0] = [object(), object()]
    now[0] = .25
    assert not graph.valid('/vive/torso/sample')
    assert len(queries) == 2
