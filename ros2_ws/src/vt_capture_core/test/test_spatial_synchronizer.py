from vt_capture_core.spatial.synchronizer import CameraFrameSample, FrameSynchronizer
from vt_capture_core.spatial.timeline import PoseBuffer
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_core.spatial.types import PoseSample

MS = 1_000_000


def frame(camera, source_ms, receive_ms=None, epoch='t1', geometry='g1'):
    return CameraFrameSample(camera, f'{camera}:{source_ms}', source_ms*MS,
                             (source_ms if receive_ms is None else receive_ms)*MS,
                             epoch, 'c1', geometry)


def test_each_camera_uses_own_frame_time_and_nearest_within_inclusive_tolerance():
    sync = FrameSynchronizer(['a', 'b', 'c'])
    for item in [frame('a', 100), frame('b', 79), frame('b', 120), frame('c', 95)]:
        sync.append(item)
    result = sync.poll(120*MS)
    assert result.complete and result.scene_time_ns == 100*MS
    assert result.reference_camera == 'a'
    assert result.residuals_ns == {'a': 0, 'b': 20*MS, 'c': -5*MS}
    assert sync.poll(130*MS) is None


def test_live_preview_can_skip_backlog_without_changing_matching_tolerance():
    sync = FrameSynchronizer(['a', 'b'], prefer_latest=True)
    for t in (100, 133, 166):
        sync.append(frame('a', t))
        sync.append(frame('b', t+5))
    result = sync.poll(180*MS)
    assert result.complete and result.scene_time_ns == 166*MS
    assert result.residuals_ns['b'] == 5*MS
    assert sync.poll(181*MS) is None


def test_latest_preview_preserves_pending_wait_and_does_not_use_out_of_tolerance_frame():
    sync = FrameSynchronizer(['a', 'b'], prefer_latest=True)
    sync.append(frame('a', 100))
    sync.append(frame('b', 121))
    assert sync.poll(150*MS) is None
    sync.append(frame('a', 166))
    result = sync.poll(200*MS)
    assert result.scene_time_ns == 100*MS
    assert result.missing == ('b',)


def test_latest_preview_option_does_not_change_replay_order():
    sync = FrameSynchronizer(['a', 'b'], mode='replay', prefer_latest=True)
    for t in (100, 133, 166):
        sync.append(frame('a', t))
        sync.append(frame('b', t))
    assert sync.poll(180*MS).scene_time_ns == 100*MS


def test_waits_bounded_time_then_publishes_partial_without_reusing_frames():
    sync = FrameSynchronizer(['a', 'b'])
    sync.append(frame('a', 100))
    sync.append(frame('b', 121))
    assert sync.poll(199*MS) is None
    result = sync.poll(200*MS)
    assert result.missing == ('b',) and not result.complete
    assert [item.camera_id for item in result.frames] == ['a']
    sync.append(frame('a', 140, receive_ms=201))
    result = sync.poll(201*MS)
    assert result.complete and result.residuals_ns['b'] == -19*MS
    sync.append(frame('a', 145, receive_ms=202))
    assert sync.poll(202*MS) is None


def test_reference_failover_after_250ms_and_scene_time_never_moves_back():
    sync = FrameSynchronizer(['a', 'b'])
    sync.append(frame('a', 100))
    sync.append(frame('b', 100))
    assert sync.poll(100*MS).scene_time_ns == 100*MS
    sync.append(frame('b', 300))
    assert sync.poll(349*MS) is None
    result = sync.poll(400*MS)
    assert result.reference_camera == 'b' and result.scene_time_ns == 300*MS
    assert result.sync_epoch == 1 and result.missing == ('a',)
    assert not sync.append(frame('b', 100, receive_ms=401)).accepted


def test_preferred_reference_requires_continuous_second_before_restoring():
    sync = FrameSynchronizer(['a', 'b'])
    for item in [frame('a', 100), frame('b', 100)]:
        sync.append(item)
    sync.poll(100*MS)
    sync.append(frame('b', 400))
    sync.poll(500*MS)
    assert sync.active_reference_camera == 'b'
    for t in range(600, 1600, 100):
        sync.append(frame('a', t))
        sync.append(frame('b', t))
        sync.poll(t*MS)
        assert sync.active_reference_camera == 'b'
    sync.append(frame('a', 1600))
    sync.append(frame('b', 1600))
    result = sync.poll(1600*MS)
    assert result.reference_camera == 'a' and result.sync_epoch == 2


def test_duplicate_and_out_of_order_frames_do_not_keep_dead_reference_fresh():
    sync = FrameSynchronizer(['a', 'b'])
    sync.append(frame('a', 100))
    sync.append(frame('b', 100))
    sync.poll(100*MS)
    assert not sync.append(frame('a', 100, receive_ms=400)).accepted
    sync.append(frame('b', 400))
    result = sync.poll(500*MS)
    assert result.reference_camera == 'b'


def test_generation_mismatch_rejected_until_explicit_manager_reset():
    sync = FrameSynchronizer(['a', 'b'])
    sync.append(frame('a', 100))
    assert not sync.append(frame('b', 100, epoch='t2')).accepted
    sync.reset(time_epoch='t2', coordinate_epoch='c1', geometry_revision='g1')
    assert not sync.append(frame('a', 100)).accepted
    sync.append(frame('a', 50, epoch='t2'))
    sync.append(frame('b', 50, epoch='t2'))
    assert sync.poll(50*MS).scene_time_ns == 50*MS


def test_replay_keeps_reference_and_explicit_timeline_can_emit_partial():
    sync = FrameSynchronizer(['a', 'b'], mode='replay')
    sync.append(frame('b', 100))
    assert sync.poll(1000*MS) is None
    assert sync.active_reference_camera == 'a'
    result = sync.poll(1000*MS, scene_time_ns=100*MS)
    assert result.missing == ('a',) and result.reference_camera == 'a'


def test_pose_wait_then_interpolate_each_camera_at_its_time_without_extrapolation():
    poses = PoseBuffer()
    def pose(t):
        return PoseSample('role', 'id', 'map', t*MS, t*MS, 't1', 'c1',
                          RigidTransform((t/1000, 0, 0), (0, 0, 0, 1)), True)
    poses.append(pose(100))
    sync = FrameSynchronizer(['a', 'b'], tracker_buffers={'a': poses, 'b': poses})
    sync.append(frame('a', 110))
    sync.append(frame('b', 120))
    assert sync.poll(120*MS) is None
    poses.append(pose(150))
    result = sync.poll(150*MS)
    assert result.complete
    assert result.poses['a'].pose.translation_m[0] == .11
    assert result.poses['b'].pose.translation_m[0] == .12
    sync.append(frame('a', 180))
    sync.append(frame('b', 190))
    result = sync.poll(280*MS)
    assert result.unavailable_poses == ('a', 'b') and not result.complete
    assert result.poses['a'].reason == 'no_after'


def test_image_queue_bound_and_pose_gap_are_enforced():
    poses = PoseBuffer()
    for t in (100, 151):
        poses.append(PoseSample('role', 'id', 'map', t*MS, t*MS, 't1', 'c1', RigidTransform.identity(), True))
    sync = FrameSynchronizer(['a'], tracker_buffers={'a': poses}, max_frames_per_camera=3)
    for t in (110, 111, 112, 113):
        sync.append(frame('a', t))
    result = sync.poll(300*MS)
    assert result.scene_time_ns == 111*MS
    assert result.poses['a'].reason == 'gap_exceeded'


def test_all_stale_cameras_stop_outputs_and_recovery_discards_old_pending():
    sync = FrameSynchronizer(['a', 'b'])
    sync.append(frame('a', 100))
    assert sync.poll(150*MS) is None
    assert sync.poll(500*MS) is None
    sync.append(frame('a', 600))
    sync.append(frame('b', 600))
    result = sync.poll(600*MS)
    assert result.complete and result.scene_time_ns == 600*MS


def test_preferred_recovery_gap_restarts_continuity_timer():
    sync = FrameSynchronizer(['a', 'b'])
    for item in [frame('a', 100), frame('b', 100)]:
        sync.append(item)
    sync.poll(100*MS)
    sync.append(frame('b', 400))
    sync.poll(500*MS)
    for t in range(600, 2000, 100):
        sync.append(frame('b', t))
        if t not in (800, 900):
            sync.append(frame('a', t))
        sync.poll(t*MS)
        assert sync.active_reference_camera == 'b'
    for camera in ('a', 'b'):
        sync.append(frame(camera, 2000))
    assert sync.poll(2000*MS).reference_camera == 'a'
