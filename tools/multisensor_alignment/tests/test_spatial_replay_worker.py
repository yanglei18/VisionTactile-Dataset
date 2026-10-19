from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from vt_multisensor_alignment.model import MessageRef, TimedPose, Transform
from vt_multisensor_alignment import spatial_replay_worker as worker


def dataset(tmp_path):
    def pose(t):
        return TimedPose('left_wrist', '1'*64, t, t, Transform([t/1e9, 0, 0], [0, 0, 0, 1]), MessageRef('/vive/left/sample', t, t, t))
    config = NS(world_frame='vive_map', cameras=(), reference_camera=None,
                thresholds=NS(max_camera_delta_ms=20., max_tracker_gap_ms=50.))
    data = NS(tracker_poses={'left_wrist': (pose(100_000_000), pose(120_000_000), pose(140_000_000))},
              invalid_tracker_times={}, camera_frames={})
    return config, data


def source(tmp_path, monkeypatch):
    config, data = dataset(tmp_path)
    monkeypatch.setattr(worker, '_read_replay_dataset', lambda *a, **kw: (config, data))
    monkeypatch.setattr(worker, '_assert_closed_bag', lambda *a: None)
    return worker.BagReplaySource(tmp_path), data


def test_legacy_bag_has_real_tracker_frames_but_no_fabricated_cameras_or_clouds(tmp_path, monkeypatch):
    replay, _ = source(tmp_path, monkeypatch)
    assert replay.frame_count == 3
    result = replay.frame(1)
    assert result['mode'] == 'replay'
    assert result['trackers'][0]['position_m'] == [.12, 0., 0.]
    assert result['cameras'] == [] and replay.last_clouds == []
    assert 'geometry' in replay.note.lower()


def test_invalid_tracker_event_blocks_interpolation_in_recorded_pose_stream(tmp_path, monkeypatch):
    replay, data = source(tmp_path, monkeypatch)
    data.invalid_tracker_times['left_wrist'] = (110_000_000,)
    selection = replay._pose('left_wrist', 115_000_000)
    assert selection.pose is None and selection.reason == 'invalid_pose'
    assert replay._pose('left_wrist', 120_000_000).pose is not None


def test_invalid_indices_and_cancellation_never_publish_previous_scene(tmp_path, monkeypatch):
    replay, _ = source(tmp_path, monkeypatch)
    replay.frame(0)
    with pytest.raises(IndexError):
        replay.frame(-1)
    replay.cancelled = lambda: True
    with pytest.raises(InterruptedError):
        replay.frame(1)


def test_active_sidecar_rejected_even_with_ros_metadata_present(tmp_path):
    import json
    (tmp_path/'metadata.yaml').write_text('rosbag2_bagfile_information: {}')
    materials = NS(manifest={'complete': False})
    with pytest.raises(ValueError, match='closed'):
        worker._assert_closed_bag(tmp_path, materials)


def test_missing_reference_uses_partial_timeline_without_switching_or_distant_reuse(tmp_path, monkeypatch):
    config, data = dataset(tmp_path)
    config.reference_camera = 'reference'
    config.cameras = (NS(name='reference', serial='a', tracker_role='left_wrist'),
                      NS(name='other', serial='b', tracker_role='left_wrist'))
    data.camera_frames = {'reference': (), 'other': (NS(host_realtime_ns=100_000_000), NS(host_realtime_ns=200_000_000))}
    monkeypatch.setattr(worker, '_read_replay_dataset', lambda *a, **kw: (config, data))
    monkeypatch.setattr(worker, '_assert_closed_bag', lambda *a: None)
    source = worker.BagReplaySource(tmp_path)
    assert source.frame_count == 2
    scene = source.frame(0)
    assert scene['active_reference_camera'] == 'reference'
    assert 'reference_missing' in scene['diagnostics']
    assert source._nearest_camera('other', 150_000_000) is None


def test_recorded_display_transform_is_restored_without_rezeroing_raw_poses(tmp_path, monkeypatch):
    transform = {'translation_m': [3., 4., 5.], 'quaternion_xyzw': [0., 0., 1., 0.]}
    monkeypatch.setattr(worker, 'GeometryMaterials', lambda path: NS(manifest={'display_from_world': transform}, note='recorded geometry'))
    replay, _ = source(tmp_path, monkeypatch)
    scene = replay.frame(0)
    assert scene['display_from_world'] == transform
    assert scene['trackers'][0]['position_m'] == [.1, 0., 0.]


def test_invalid_recorded_display_transform_is_rejected(tmp_path, monkeypatch):
    malformed = {'translation_m': [0., 0., 0.], 'quaternion_xyzw': [0., 0., 0., 0.]}
    monkeypatch.setattr(worker, 'GeometryMaterials', lambda path: NS(manifest={'display_from_world': malformed}, note='recorded geometry'))
    with pytest.raises(ValueError, match='quaternion'):
        replay, _ = source(tmp_path, monkeypatch)
        replay.frame(0)


def test_replay_prefers_recorded_undistorted_color_frustum(tmp_path, monkeypatch):
    vertices = [[-.1, -.1, .1], [.1, -.1, .1], [.1, .1, .1], [-.1, .1, .1],
                [-.4, -.4, .5], [.4, -.4, .5], [.4, .4, .5], [-.4, .4, .5]]
    config, data = dataset(tmp_path)
    config.reference_camera = 'camera'
    config.cameras = (NS(name='camera', serial='serial', tracker_role='left_wrist'),)
    data.camera_frames = {'camera': (NS(host_realtime_ns=100_000_000, source_timestamp_ns=17,
                                        color=NS(bag_timestamp_ns=100_000_001)),)}
    geometry = dict(extrinsic=dict(tracker={'tracker_id': '1'*64},
                                   transform=dict(translation_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1])),
                    geometry_revision=1, geometry_sha256='0'*64, color_frustum=vertices,
                    color_intrinsics=dict(width=10, height=10, fx=10, fy=10, cx=5, cy=5,
                                          distortion_model='plumb_bob', d=[.3, 0, 0, 0, 0]))
    def select(*args, require_depth=True, **kw):
        return NS(status='unavailable' if require_depth else 'ready', reason='missing_depth' if require_depth else '',
                  geometry=geometry, time_epoch=0, coordinate_epoch=0)
    materials = NS(manifest={}, note='recorded geometry', select=select)
    monkeypatch.setattr(worker, 'GeometryMaterials', lambda path: materials)
    monkeypatch.setattr(worker, '_read_replay_dataset', lambda *a, **kw: (config, data))
    monkeypatch.setattr(worker, '_assert_closed_bag', lambda *a: None)
    scene = worker.BagReplaySource(tmp_path).frame(0)
    assert scene['cameras'][0]['frustum_vertices'] == vertices
