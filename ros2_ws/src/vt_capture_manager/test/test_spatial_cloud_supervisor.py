from types import SimpleNamespace as NS

import pytest

from vt_capture_manager.spatial import cloud_supervisor as module


def header(camera, sequence=1, **changes):
    result=dict(camera_id=camera, frame_key=f'group-{sequence}', source_realtime_ns=100,
        manager_instance_id='manager', worker_instance_id='worker', scene_epoch='scene',
        time_epoch=1, coordinate_epoch=2, sync_epoch=3, geometry_revision=4,
        geometry_sha256='a'*64, point_count=1, cloud_sequence=sequence,
        world_from_color=dict(translation_m=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.]))
    result.update(changes)
    return result


@pytest.fixture
def supervisor(monkeypatch,tmp_path):
    clock=NS(now=10.)
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock.now))
    store=NS(manager_instance_id='manager',config=NS(cameras=[{'id':c} for c in ('head','left','right')]))
    value=module.CloudSupervisor(store,tmp_path/'cloud.sock')
    value._worker_id='worker';value._ready=True;value._process=NS(poll=lambda:None)
    return value,clock


def test_partial_successor_keeps_exact_complete_group_without_renewing_lease(supervisor):
    value,clock=supervisor
    value._event(dict(event='frames',headers=[header(c) for c in ('head','left','right')]))
    original=dict(value._headers)
    for sequence in range(2,9):
        clock.now=10.+(sequence-1)/15.
        value._event(dict(event='frames',headers=[header('head',sequence),header('left',sequence)]))
        assert value._headers==original
        assert set(value.latest_headers())=={'head','left','right'}
        assert {h['frame_key'] for h in value.latest_headers().values()}=={'group-1'}
    clock.now=10.5
    assert value.latest_headers()=={}
    value._event(dict(event='frames',headers=[header('head',9)]))
    assert set(value.latest_headers())=={'head'}


def test_complete_successor_replaces_all_three_together(supervisor):
    value,clock=supervisor
    for sequence in (1,2):
        clock.now+=.1
        value._event(dict(event='frames',headers=[header(c,sequence) for c in ('head','left','right')]))
        assert {h['frame_key'] for h in value.latest_headers().values()}=={f'group-{sequence}'}


@pytest.mark.parametrize('changes',[{'scene_epoch':'new'}, {'time_epoch':2},
    {'coordinate_epoch':3},{'sync_epoch':4},{'geometry_revision':5},{'geometry_sha256':'b'*64}])
def test_partial_new_generation_never_reuses_old_complete_group(supervisor,changes):
    value,clock=supervisor
    value._event(dict(event='frames',headers=[header(c) for c in ('head','left','right')]))
    clock.now+=.1
    value._event(dict(event='frames',headers=[header('head',2,**changes)]))
    assert set(value.latest_headers())=={'head'}
    assert value.latest_headers()['head']['frame_key']=='group-2'


def test_no_complete_group_does_not_mix_partial_groups(supervisor):
    value,clock=supervisor
    value._event(dict(event='frames',headers=[header('head')]))
    clock.now+=.1
    value._event(dict(event='frames',headers=[header('left',2),header('right',2)]))
    assert set(value.latest_headers())=={'left','right'}
    assert {h['frame_key'] for h in value.latest_headers().values()}=={'group-2'}
