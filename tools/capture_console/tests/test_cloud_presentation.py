from types import SimpleNamespace

import pytest

from vt_capture_console.backend.cloud_presentation import CloudPresentation


def group(sequence=1, names=('head', 'left', 'right')):
    frame = dict(mode='live', manager_instance_id='manager', scene_epoch='scene',
                 time_epoch=1, coordinate_epoch=1, sync_epoch=1, world_frame='world',
                 generated_realtime_ns=1_100_000_000, trackers=[],
                 cameras=[dict(id=n, geometry_revision=1, geometry_sha256='a'*64,
                               registration_state='ready', age_s=.1, state='fresh',
                               frame_key=sequence) for n in ('head', 'left', 'right')])
    clouds = {n: dict(header=dict(cloud_sequence=sequence, source_realtime_ns=1_000_000_000, point_count=1,
                                 frame_key=sequence)) for n in names}
    return frame, clouds


def test_partial_delivery_keeps_whole_group_not_mixed_poses():
    clock = SimpleNamespace(now=10.)
    p = CloudPresentation(lambda: clock.now)
    original, clouds = group()
    p.select(original, clouds)
    clock.now += .1
    new, partial = group(2, ('head',))
    shown, shown_clouds = p.select(new, partial)
    assert p.held
    assert set(shown_clouds) == {'head', 'left', 'right'}
    assert {c['frame_key'] for c in shown['cameras']} == {1}
    assert original['cameras'][0]['age_s'] == .1
    assert shown['cameras'][0]['age_s'] == pytest.approx(.2)
    complete_frame, complete_clouds = group(2)
    shown, shown_clouds = p.select(complete_frame, complete_clouds)
    assert not p.held
    assert shown is complete_frame
    assert shown_clouds is complete_clouds


def test_repeated_groups_never_renew_expiry():
    clock = SimpleNamespace(now=10.)
    p = CloudPresentation(lambda: clock.now)
    frame, clouds = group()
    p.select(frame, clouds)
    for i in range(1, 5):
        clock.now = 10.+i*.1
        p.select(frame, clouds)
    clock.now = 10.51
    new, partial = group(2, ('head',))
    assert p.select(new, partial) == (new, partial)
    assert not p.held


@pytest.mark.parametrize('field,value', [('scene_epoch','other'), ('time_epoch',2),
    ('coordinate_epoch',2), ('sync_epoch',2), ('manager_instance_id','other'),
    ('world_frame','other'), ('mode','replay')])
def test_generation_change_does_not_hold(field, value):
    p = CloudPresentation()
    p.select(*group())
    frame, clouds = group(2, ('head',))
    frame[field] = value
    assert p.select(frame, clouds) == (frame, clouds)
    assert not p.held


def test_geometry_identity_loss_and_disabled_clear():
    p = CloudPresentation()
    p.select(*group())
    frame, clouds = group(2, ('head',))
    frame['cameras'][1]['registration_state'] = 'identity_unverified'
    assert p.select(frame, clouds) == (frame, clouds)
    p.select(*group())
    assert p.select(None, {}, enabled=False) == (None, {})
    assert not p.clouds


def test_source_age_bounds_hold_and_arrays_are_not_copied():
    p = CloudPresentation()
    frame, clouds = group()
    p.select(frame, clouds)
    next_frame, empty = group(2, ())
    shown, held = p.select(next_frame, empty)
    assert held['head'] is clouds['head']
    next_frame['generated_realtime_ns'] = 2_100_000_000
    assert p.select(next_frame, empty) == (next_frame, empty)
    assert not p.held


def test_page_holds_clouds_and_their_scene_then_expires(qapp, monkeypatch):
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    current = SimpleNamespace(clouds=group()[1])
    backend = SimpleNamespace(mode='live', set_interest=lambda *a: None,
        cloud_backend=SimpleNamespace(latest=lambda f: current.clouds, status=lambda: 'ready'))
    page = SpatialPage(backend)
    clock = SimpleNamespace(now=10.)
    page._cloud_presentation = CloudPresentation(lambda: clock.now)
    page.model.layers.add('cloud')
    drawn = []
    monkeypatch.setattr(page.scene, 'set_clouds', lambda clouds,*a: drawn.append(dict(clouds)))
    try:
        frame, current.clouds = group()
        page.apply_frame(frame, {'state':'ready'})
        assert len(drawn[-1]) == 3
        clock.now = 10.1
        frame, current.clouds = group(2, ('head',))
        page.apply_frame(frame, {'state':'ready'})
        assert len(drawn[-1]) == 3
        assert {c['frame_key'] for c in page.model.frame['cameras']} == {1}
        assert '暂留' in page.status.text()
        clock.now = 10.6
        page.apply_frame(frame, {'state':'ready'})
        assert set(drawn[-1]) == {'head'}
        assert {c['frame_key'] for c in page.model.frame['cameras']} == {2}
        page.apply_frame(None, {'state':'offline'})
        assert drawn[-1] == {}
    finally:
        page.close()


def test_page_labels_driver_points_without_fake_rgbd_distance_limits(qapp,monkeypatch):
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    frame,clouds=group(names=('head',))
    clouds['head']['header']['origin']=dict(source='realsense',topic='/d436/depth/color/points',
        native_frame_id='d436_depth_optical_frame',input_points=10,invalid_points=9)
    backend=SimpleNamespace(mode='live',set_interest=lambda *a:None,
        cloud_backend=SimpleNamespace(latest=lambda f:clouds,status=lambda:'ready'))
    page=SpatialPage(backend)
    monkeypatch.setattr(page.scene,'set_clouds',lambda *a:None)
    try:
        page.model.layers.add('cloud');page.apply_frame(frame,{'state':'ready'})
        text=page.cloud_limits.text()
        assert 'RealSense' in text and '/d436/depth/color/points' in text
        assert '后台深度 0.1' not in text and '无效 9' in text
    finally:page.close()
