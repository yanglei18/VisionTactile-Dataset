from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from vt_capture_console.backend.cloud_presentation import CloudPresentation


def scene(sequences=(1,1,1), times=(1_000_000_000,1_080_000_000,1_160_000_000)):
    names=('head','left','right'); roles=('torso','left_wrist','right_wrist')
    frame=dict(mode='live',manager_instance_id='manager',scene_epoch='scene',world_frame='world',
               time_epoch=0,coordinate_epoch=0,sync_epoch=0,time_mode='independent',scene_time_ns=None,
               generated_realtime_ns=1_200_000_000,trackers=[],cameras=[],diagnostics=[])
    clouds={}
    for name,role,seq,source in zip(names,roles,sequences,times):
        key=dict(scene_time_ns=source,reference_camera=name,sequence=seq)
        pose=dict(translation_m=[float(seq),0.,0.],quaternion_xyzw=[0.,0.,0.,1.])
        bound=dict(role=role,tracker_id=role,world_from_tracker=pose,before_ns=source,after_ns=source,gap_ns=0)
        frame['trackers'].append(dict(id=role,role=role,tracker_id=role,source_realtime_ns=source,
            position_m=pose['translation_m'],quaternion_xyzw=pose['quaternion_xyzw'],state='fresh',valid=True,age_s=.1))
        frame['cameras'].append(dict(id=name,tracker_role=role,registration_state='ready',geometry_revision=1,
            geometry_sha256='a'*64,frame_key=key,position_m=[float(seq)+.1,0,0],quaternion_xyzw=[0,0,0,1],
            source_realtime_ns=source,state='fresh',age_s=.1))
        clouds[name]=dict(header=dict(worker_instance_id='worker',cloud_sequence=seq,frame_key=key,
            matching_mode='independent',source_realtime_ns=source,point_count=1,tracker_pose=bound,
            world_from_color=dict(translation_m=[float(seq)+.1,0,0],quaternion_xyzw=[0,0,0,1])))
    return frame,clouds


def test_fast_camera_advances_while_slow_camera_keeps_its_old_pose_and_cloud():
    clock=NS(now=10.); p=CloudPresentation(lambda:clock.now)
    first,clouds=scene(); p.select(first,clouds)
    clock.now+=.1
    new,updates=scene((2,2,2)); updates={k:v for k,v in updates.items() if k!='left'}
    original=deepcopy(new)
    shown,visible=p.select(new,updates)
    assert {k:v['header']['cloud_sequence'] for k,v in visible.items()}=={'head':2,'left':1,'right':2}
    assert visible['left'] is clouds['left']
    assert shown['cameras'][1]['frame_key']==first['cameras'][1]['frame_key']
    assert shown['trackers'][1]['position_m']==[1.,0.,0.]
    assert shown['trackers'][0]['position_m']==[2.,0.,0.]
    assert new==original
    assert p.held


def test_per_camera_expiry_is_not_refreshed_by_other_cameras_or_repeated_scene():
    clock=NS(now=10.); p=CloudPresentation(lambda:clock.now)
    first,clouds=scene(); p.select(first,clouds)
    for i in range(1,7):
        clock.now=10.+i*.6
        new,updates=scene((i+1,1,i+1))
        # Include the same old left frame on every refresh: must not renew it.
        shown,visible=p.select(new,updates)
    assert set(visible)=={'head','right'}
    assert visible['head']['header']['cloud_sequence']==7
    new,updates=scene((8,2,8))
    assert set(p.select(new,updates)[1])=={'head','left','right'}


@pytest.mark.parametrize('key,value',[('geometry_revision',2),('geometry_sha256','b'*64),
    ('registration_state','identity_unverified'),('tracker_role','replacement')])
def test_per_camera_geometry_change_only_clears_affected_channel(key,value):
    p=CloudPresentation(); p.select(*scene())
    new,clouds=scene((2,2,2)); new['cameras'][1][key]=value
    del clouds['left']
    shown,visible=p.select(new,clouds)
    assert set(visible)=={'head','right'}
    assert visible['head']['header']['cloud_sequence']==2


@pytest.mark.parametrize('key,value',[('scene_epoch','new'),('time_epoch',1),('coordinate_epoch',1),
    ('sync_epoch',1),('manager_instance_id','new'),('world_frame','other')])
def test_global_epoch_change_clears_all_old_pairs(key,value):
    p=CloudPresentation(); p.select(*scene())
    new,clouds=scene((2,2,2)); new[key]=value
    assert p.select(new,{'head':clouds['head']})[1]=={'head':clouds['head']}
    assert not p.held


def test_stale_source_and_offline_never_hold_or_reanimate_pairs():
    p=CloudPresentation(); p.select(*scene())
    new,clouds=scene(); new['generated_realtime_ns']=4_200_000_000
    assert p.select(new,clouds)[1]=={}
    assert p.select(None,{},enabled=False)==(None,{})
    assert not p._camera_cache


def test_short_gap_holds_frozen_pair_with_real_age_not_latest_tracker():
    clock=NS(now=10.); p=CloudPresentation(lambda:clock.now)
    original,clouds=scene(); p.select(original,clouds)
    new,_=scene((99,99,99)); new['generated_realtime_ns']=2_400_000_000
    clock.now=11.2
    shown,visible=p.select(new,{})
    assert len(visible)==3 and p.stale_cameras=={'head','left','right'}
    assert shown['trackers'][0]['position_m']==[1.,0.,0.]
    assert shown['cameras'][0]['state']=='stale'
    assert shown['cameras'][0]['age_s']==pytest.approx(1.4)
    clock.now=13.2
    assert not p.select(new,{})[1]  # Stalled scene clock cannot renew lifetime.


def test_expired_new_packet_is_never_admitted_for_display_hold():
    p=CloudPresentation(); frame,clouds=scene()
    frame['generated_realtime_ns']=2_400_000_000
    assert not p.select(frame,clouds)[1]


def test_metadata_ahead_of_binary_uses_packet_camera_and_tracker_poses():
    p=CloudPresentation(); old,packets=scene()
    ahead,_=scene((10,10,10))
    shown,visible=p.select(ahead,packets)
    assert len(visible)==3
    assert shown['cameras'][0]['frame_key']==old['cameras'][0]['frame_key']
    assert shown['cameras'][0]['position_m']==[1.1,0,0]
    assert shown['trackers'][0]['position_m']==[1.,0,0]
    assert ahead['cameras'][0]['position_m']==[10.1,0,0]


def test_display_policy_switch_does_not_mix_strict_group_and_independent_pairs():
    p=CloudPresentation(); p.select(*scene())
    new,clouds=scene((2,2,2)); new['time_mode']='aligned'; new['scene_time_ns']=1_000_000_000
    shown,visible=p.select(new,{'head':clouds['head']})
    assert set(visible)=={'head'} and not p.held
    new,clouds=scene((3,3,3))
    shown,visible=p.select(new,{'left':clouds['left']})
    assert set(visible)=={'left'} and not p.held


def test_page_reports_independent_matching_skew_and_per_camera_age(qapp,monkeypatch):
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    frame,clouds=scene()
    current=NS(clouds=clouds)
    backend=NS(mode='live',set_interest=lambda *a:None,
        cloud_backend=NS(latest=lambda f:current.clouds,status=lambda:'ready'))
    page=SpatialPage(backend); clock=NS(now=10.)
    page._cloud_presentation=CloudPresentation(lambda:clock.now)
    drawn=[]
    monkeypatch.setattr(page.scene,'set_clouds',lambda clouds,*a:drawn.append(dict(clouds)))
    try:
        page.model.layers.add('cloud');page.apply_frame(frame,{'state':'ready'})
        assert '逐路独立匹配' in page.status.text() and '非同时采样' in page.status.text()
        assert '160 ms' in page.status.text()
        assert '源帧年龄 200 ms' in page.cloud_limits.text()
        clock.now+=.1; frame,current.clouds=scene((2,2,2)); del current.clouds['left']
        page.apply_frame(frame,{'state':'ready'})
        assert drawn[-1]['head']['header']['cloud_sequence']==2
        assert drawn[-1]['left']['header']['cloud_sequence']==1
        assert '部分通道暂留' in page.status.text()
    finally: page.close()
