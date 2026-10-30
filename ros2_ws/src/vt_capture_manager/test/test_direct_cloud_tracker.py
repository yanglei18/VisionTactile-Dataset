"""Live independent clouds receive ROS poses without Manager snapshot delay."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest

from test_independent_clouds import independent, send, headers, ROLES


def message(role='torso', tracker='head-tracker', source=990_000_000, mono=None, valid=True):
    return NS(role=role,tracker_id=tracker,host_realtime_ns=source,
              host_monotonic_ns=source if mono is None else mono,pose_valid=valid,
              header=NS(frame_id='vive_map'),
              pose=NS(position=NS(x=source/1e9,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))


def direct():
    w,c,s,e,control=independent()
    control.update(tracker_inputs={r:'/vive/'+r+'/sample' for r in ROLES},world_frame='vive_map',poses={})
    w.configure(control)
    return w,c,s,e,control


def test_direct_source_progresses_without_manager_pose_snapshots():
    w,c,s,e,control=direct()
    assert w.observe_tracker('torso',message())
    assert w.observe_tracker('torso',message(source=1_010_000_000))
    # Repeated Manager control with NO poses must not clear the direct history.
    w.configure(deepcopy(control))
    send(w,'head');w.tick()
    h=headers(s)['head']
    assert h['tracker_pose']['before_ns']==990_000_000
    assert h['tracker_pose']['after_ns']==1_010_000_000
    assert h['tracker_pose']['world_from_tracker']['translation_m'][0]==pytest.approx(1.)


@pytest.mark.parametrize('bad',['identity','role','frame','invalid','nan'])
def test_bad_direct_sample_never_supplies_valid_bracket(bad):
    w,c,s,e,control=direct()
    w.observe_tracker('torso',message())
    m=message(source=1_010_000_000)
    if bad=='identity':m.tracker_id='other-device'
    if bad=='role':m.role='left_wrist'
    if bad=='frame':m.header.frame_id='other-world'
    if bad=='invalid':m.pose_valid=False
    if bad=='nan':m.pose.position.x=float('nan')
    w.observe_tracker('torso',m)
    send(w,'head');c.now+=600_000_000;w.tick()
    assert not s.frames


def test_direct_source_clock_reset_stops_until_manager_epoch_ack():
    w,c,s,e,control=direct()
    w.observe_tracker('torso',message())
    assert not w.observe_tracker('torso',message(source=1_010_000_000,mono=1_030_000_000))
    assert e[-1]['event']=='clock_reset'
    w.configure(deepcopy(control))
    assert not w.control['interest']
    control['time_epoch']+=1
    w.configure(control)
    assert w.control['interest'] and not len(w._direct_store.buffers['torso'])


def test_direct_generation_change_cannot_reuse_previous_poses():
    w,c,s,e,control=direct()
    w.observe_tracker('torso',message())
    w.observe_tracker('torso',message(source=1_010_000_000))
    control['coordinate_epoch']+=1;w.configure(control)
    send(w,'head');c.now+=600_000_000;w.tick()
    assert not s.frames


def test_direct_buffer_is_bounded_and_invalid_pose_is_not_skipped():
    w,c,s,e,control=direct()
    for i in range(600):
        source=1_000_000_000+i*1_000_000;c.now=source
        w.observe_tracker('torso',message(source=source,valid=i!=598))
    buffer=w._direct_store.buffers['torso']
    assert len(buffer)<=512
    assert buffer.select(1_598_500_000).reason=='invalid_pose'
