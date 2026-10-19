from dataclasses import replace

import pytest

from vt_capture_core.spatial.timeline import PoseBuffer
from vt_capture_core.spatial.transforms import RigidTransform
from vt_capture_core.spatial.types import PoseSample


def sample(time, *, epoch='one', valid=True, tracker_id='device'):
    return PoseSample('left_wrist', tracker_id, 'vive_map', time, time + 100,
                      epoch, 'map-1', RigidTransform((time / 1e9, 0, 0), (0, 0, 0, 1)) if valid else None, valid)


def test_sorted_insert_duplicate_does_not_refresh_and_interpolation():
    buffer = PoseBuffer()
    assert buffer.append(sample(100)).accepted
    buffer.append(sample(300))
    buffer.append(sample(200))
    duplicate = buffer.append(replace(sample(300), received_monotonic_ns=99999))
    assert not duplicate.accepted and duplicate.reason == 'duplicate'
    assert buffer.latest.received_monotonic_ns == 400
    selection = buffer.select(250)
    assert selection.status == 'interpolated'
    assert (selection.before_ns, selection.after_ns, selection.gap_ns) == (200, 300, 100)
    assert selection.pose.translation_m[0] == pytest.approx(250e-9)
    assert buffer.select(200).status == 'exact'


def test_no_extrapolation_and_gap_boundary():
    buffer = PoseBuffer()
    assert buffer.select(0).reason == 'empty'
    buffer.append(sample(100_000_000))
    buffer.append(sample(150_000_000))
    assert buffer.select(125_000_000).status == 'interpolated'
    assert buffer.select(125_000_000, max_gap_ns=49_999_999).reason == 'gap_exceeded'
    assert buffer.select(0).reason == 'no_before'
    assert buffer.select(200_000_000).reason == 'no_after'


def test_invalid_sample_breaks_interpolation():
    buffer = PoseBuffer()
    for point in (sample(100), sample(200, valid=False), sample(300)):
        buffer.append(point)
    for timestamp in (150, 200, 250):
        assert buffer.select(timestamp).reason == 'invalid_pose'


def test_epoch_change_clears_and_retired_epoch_cannot_return():
    buffer = PoseBuffer()
    buffer.append(sample(100))
    result = buffer.append(sample(200, epoch='two'))
    assert result.accepted and result.reset
    assert len(buffer) == 1
    assert not buffer.append(sample(300)).accepted
    assert buffer.select(150).pose is None
    buffer.reset(time_epoch='three', coordinate_epoch='map-1')
    assert not buffer.append(sample(400, epoch='two')).accepted
    assert buffer.append(sample(400, epoch='three')).accepted


def test_count_and_time_limits_both_apply():
    buffer = PoseBuffer(max_samples=3, max_age_ns=100)
    for t in (100, 110, 120, 130):
        buffer.append(sample(t))
    assert len(buffer) == 3 and buffer.select(100).reason == 'no_before'
    buffer.append(sample(1000))
    assert len(buffer) == 1
    assert not buffer.append(sample(200)).accepted


def test_identity_change_resets_instead_of_interpolating():
    buffer = PoseBuffer()
    buffer.append(sample(100))
    assert buffer.append(sample(200, tracker_id='replacement')).reset
    assert buffer.select(150).status == 'unavailable'
