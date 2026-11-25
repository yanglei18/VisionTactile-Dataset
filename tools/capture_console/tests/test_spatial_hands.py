"""Bilateral optical hands on the existing world-space canvas."""
from copy import deepcopy
from math import sqrt

from vt_capture_console.backend.scene_demo import DemoSceneBackend, demo_scene
from vt_capture_console.ui.widgets.spatial_page import SpatialPage
from vt_capture_console.ui.widgets.spatial_scene import hand_markers, scene_lines, scene_surfaces
from vt_capture_console.viewmodels.spatial import SpatialViewModel, display_point, entity_color


def hand(side):
    sign = 1 if side == 'left' else -1
    wrist = [-.11*sign, 0., .06]
    points = [wrist]
    # Asymmetric labelled fingers, already in world. A nonidentity wrist catches double rotation.
    for x, y in ((.051,.10), (.022,.17), (0.,.19), (-.02,.17), (-.04,.14)):
        points.extend([[wrist[0] + sign*x*(.55+.45*j/4), wrist[1]+y*(.3+.7*j/4), wrist[2]]
                       for j in (1, 2, 3, 4)])
    return dict(id=side, hand=side, label=side, camera_id='d436', source_realtime_ns=100,
                state='fresh', valid=True, reason='', position_m=wrist,
                quaternion_xyzw=[0., sqrt(.5), 0., sqrt(.5)], joint_positions_m=points)


def scene():
    return dict(demo_scene(0), trackers=[], cameras=[], hands=[hand('left'), hand('right')],
                display_from_world=dict(translation_m=[.2,.3,-.4], quaternion_xyzw=[1.,0.,0.,0.]))


def test_world_landmarks_share_the_camera_display_transform_once_and_hide_on_loss():
    model = SpatialViewModel()
    model.layers = {'hand'}
    frame = scene()
    model.update(frame)
    assert {e['id'] for e in model.entities()} == {'hand/left', 'hand/right'}
    lines = scene_lines(model)
    assert len(lines) == 46
    left, right = frame['hands']
    tip_segment = tuple(display_point(frame, p) for p in left['joint_positions_m'][7:9])
    assert (*tip_segment, entity_color(left), 2.4) in lines
    assert entity_color(left) != entity_color(right)
    assert len(hand_markers(model)) == 42
    assert not scene_surfaces(model)

    left.update(valid=False, state='invalid')  # A bad old recording may still contain XYZ after loss.
    right['joint_positions_m'] = []
    model.update(frame)
    assert len(model.entities()) == 1 and len(hand_markers(model)) == 1
    assert not scene_lines(model)
    model.layers.add('axes')
    assert len(scene_lines(model)) == 9
    model.set_preset('tracker')
    assert not model.entities()


def test_source_loss_clears_telemetry_and_a_subsequent_freeze_cannot_revive_hands():
    model = SpatialViewModel()
    model.update(scene())
    model.select('hand/right')
    model.source_available = False
    assert model.selected['position_m'] is None
    assert not model.entities()
    model.set_frozen(True)
    model.source_available = True
    assert model.selected['position_m'] is None
    assert not model.entities()


def test_actual_spatial_page_refreshes_moving_fingers_and_wrist_only_without_refitting(qapp, monkeypatch):
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    page.resize(1000, 650)
    frame = scene()
    try:
        page.show()
        page.apply_frame(frame, dict(state='ready', message=''))
        qapp.processEvents()
        assert page.scene.hand_status.isVisible()
        assert '21 点手形' in page.scene.hand_status.text()
        fits, updates = [], []
        old = page.scene.viewport.set_model
        def update(model):
            updates.append(model)
            old(model)
        monkeypatch.setattr(page.scene.viewport, 'fit', lambda: fits.append(1))
        monkeypatch.setattr(page.scene.viewport, 'set_model', update)
        frame['hands'][0]['joint_positions_m'][8][1] -= .02
        page.apply_frame(frame, dict(state='ready', message=''))
        assert updates and not fits
        frame['hands'][0]['joint_positions_m'] = []
        page.apply_frame(frame, dict(state='ready', message=''))
        assert '仅腕部' in page.scene.hand_status.text()
        assert any('仅腕部' in page.devices.item(i).text() for i in range(page.devices.count()))
        page.select('hand/left')
        assert 'd436' in page.detail.text()
        model2 = SpatialViewModel()
        model2.update(page.model.frame)  # Detached view can consume a frozen frame.
        assert len(model2.entities()) == 2
        legacy = deepcopy(frame)
        del legacy['hands']
        page.apply_frame(legacy, dict(state='ready', message=''))
        assert page.scene.hand_status.isHidden() and not hand_markers(page.model)
        assert not fits
    finally:
        page.close()
        backend.close()
