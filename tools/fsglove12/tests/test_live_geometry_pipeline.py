from copy import deepcopy

import pytest

from test_live_runtime_high_rate import setup_direct, feed


class Preview:
    def __init__(self):
        self.requests = []
        self.result = None

    def submit(self, request):
        self.requests[:] = [deepcopy(request)]

    def poll(self):
        result, self.result = self.result, None
        return result

    def snapshot(self):
        return {'capacity': 1}

    def finish(self):
        self.result = {**self.requests[0], 'geometry': {'vertices_m': [[1, 2, 3]],
            'joints_m': [[4, 5, 6]] * 21}, 'error': ''}


class Geometry:
    hand = 'right'

    def __init__(self):
        self.preview = Preview()
        self.measurements = []

    def measure_mano(self, pose, shape):
        self.measurements.append((deepcopy(pose), list(shape)))
        return {'joints_m': [[len(self.measurements), 0, 0]] * 21}

    def create_preview_renderer(self):
        return self.preview

    def render_mano(self, *_):
        raise AssertionError('Complete surface must not execute on processing thread')


def prepared(tmp_path):
    controller, session, backend, clock = setup_direct(tmp_path)
    geometry = controller._geometry_backend = Geometry()
    original = backend.solve
    def solve(*args, **kwargs):
        result = original(*args, **kwargs)
        result['valid_mask'] = [True] * 16
        result['joint_available_mask'] = [True] * 16
        return result
    backend.solve = solve
    return controller, session, geometry, clock


def test_all_measurements_keep_same_frame_joints_while_display_is_busy(tmp_path):
    controller, session, geometry, clock = prepared(tmp_path)
    for _ in range(20):
        feed(session, clock, 12, 3)
        controller._tick(session)
        clock.advance(1 / 120)
    assert len(geometry.measurements) == session.available_poses == 20
    assert controller._snapshot['geometry']['joints_m'][0][0] == 20
    assert controller._snapshot['display_geometry'] is None
    assert len(geometry.preview.requests) == 1
    geometry.preview.finish()
    controller._poll_geometry_preview(session)
    assert controller._snapshot['surface_preview']['measurement_pose']['frame_index'] == 19
    assert controller._snapshot['pose']['frame_index'] == 19


@pytest.mark.parametrize('change', ['profile', 'clear', 'shape', 'epoch', 'stop', 'late'])
def test_old_surface_never_crosses_context_boundary(tmp_path, change):
    controller, session, geometry, clock = prepared(tmp_path)
    feed(session, clock, 12)
    controller._tick(session)
    geometry.preview.finish()
    if change == 'profile':
        controller._profile = {'profile_version': 2}
    elif change == 'clear':
        controller._clear_generation += 1
    elif change == 'shape':
        controller._shape_preview_revision += 1
    elif change == 'epoch':
        controller._snapshot['orientation']['nodes']['12']['epoch'] += 1
    elif change == 'stop':
        session.stop.set()
    else:
        clock.advance(.3)
    controller._poll_geometry_preview(session)
    assert controller._snapshot['display_geometry'] is None


def test_candidate_shape_changes_only_async_preview(tmp_path):
    controller, session, geometry, clock = prepared(tmp_path)
    controller._shape_preview = 'candidate'
    controller._shape_candidate = {'shape_profile': {'shape_params': [2.] * 10}}
    feed(session, clock, 12)
    controller._tick(session)
    assert geometry.measurements[0][1] == [0.] * 10
    assert geometry.preview.requests[0]['pose']['shape_params'] == [2.] * 10
    assert geometry.preview.requests[0]['measurement_pose']['shape_params'] == [0.] * 10


def test_display_process_exit_does_not_reset_profile_or_interrupt_measurements(tmp_path):
    controller, session, geometry, clock = prepared(tmp_path)
    profile, fusion = controller._profile, session.fusion
    feed(session, clock, 12)
    controller._tick(session)
    geometry.preview.snapshot = lambda: {'capacity': 1, 'alive': False}
    controller._poll_geometry_preview(session)
    for _ in range(3):
        clock.advance(1 / 120)
        feed(session, clock, 12)
        controller._tick(session)
    assert controller._profile is profile and session.fusion is fusion
    assert len(geometry.measurements) == session.available_poses == 4
    assert '显示进程已结束' in controller._snapshot['display_geometry_error']
    assert not session.stop.is_set()
