"""Known synthetic rotations exercise real QMT, not hardware accuracy claims."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from vt_fsglove12.heading_correction import CausalHeadingCorrector, HINGES
from vt_fsglove12.upstream_functional import KIND


W = R.from_euler("x", np.pi / 2)
PAIRS = {"index": ("3", "4", 1, 2), "middle": ("1", "2", 4, 5),
         "ring": ("7", "8", 10, 11), "little": ("9", "10", 7, 8),
         "thumb": ("5", "6", 14, 15)}


def fixture_model():
    base = R.from_euler("xyz", [[i*.019, -.17+i*.004, i*.043] for i in range(16)])
    # Distinct initial yaw gauges and arbitrary mounting rotations for EVERY IMU.
    heading = R.from_euler("z", np.linspace(-2.1, 1.8, 16)[:, None])
    k = W.inv() * heading
    local = R.from_euler("xyz", [[.2+i*.031, -.4+i*.037, .3-i*.025] for i in range(16)])
    rest = R.from_euler("xyz", [[.04, .09, -.12]]*16)
    parents = np.array([0,0,1,2,0,4,5,0,7,8,0,10,11,0,13,14])
    cal = SimpleNamespace(_base_quat=base.as_quat(), _global_installation_error=(k*base).as_rotvec(),
                          _local_installation_error=local.as_rotvec(),
                          _delta_rot_fa_rotvec=rest.as_rotvec(), _joint_fa=parents)
    binding = dict(session_id="synthetic", upstream_state_sha256="core", config_sha256="config", rest_sha256="rest",
                   nodes_epochs={str(n): 0 for n in range(1,13)},
                   dependency_nodes_by_slot={str(c): [pn,cn] for pn,cn,p,c in PAIRS.values()})
    axis = np.array([.31,.72,.18]); axis /= np.linalg.norm(axis)
    a0 = R.from_euler("xyz", [.11,-.06,.08])
    profile = dict(session_id="synthetic", functional_enabled=True, functional_fingers={})
    for finger,(pn,cn,p,c) in PAIRS.items():
        profile["functional_fingers"][finger] = dict(kind=KIND, finger=finger, binding=deepcopy(binding), slot_refs={str(c):
            dict(mode="hinge_projection", dependency_nodes=[pn,cn], neutral_local_xyzw=a0.as_quat().tolist(), axis_local=axis.tolist())})
    return cal, binding, profile, k, local, rest, a0, axis


def make_frame(model, time, drift_deg=0., *, vertical=False, global_motion=True):
    cal,binding,profile,k,local,rest,a0,axis = model
    index=int(round(time*30))+1
    nodes={str(n):dict(quat_xyzw=[0.,0.,0.,1.], age_s=0., epoch=0, sample_index=index,
                      raw_values=[11,-12,13,14,15,8197], gyro_rad_s=[.1,.2,.3]) for n in range(1,13)}
    truth={}
    for finger,(pn,cn,p,c) in PAIRS.items():
        world=(R.align_vectors([0,0,1],(rest[c]*a0).apply(axis))[0] if vertical else
               R.from_euler("xyz", [.2+.05*np.sin(time), .17+.03*np.cos(time), .22+.13*time]
                            if global_motion else [.2,.17,.22]))
        parent=W.inv()*world
        child=parent*rest[c]*a0*R.from_rotvec(axis*(.6+.5*np.sin(3*time)))
        qp=k[p].inv()*parent*local[p]
        qc=k[c].inv()*child*local[c]
        nodes[pn]["quat_xyzw"]=qp.as_quat().tolist()
        nodes[cn]["quat_xyzw"]=(R.from_euler("z",drift_deg,degrees=True)*qc).as_quat().tolist()
        truth[cn]=qc
    return dict(timestamp_s=time,session_id="synthetic",nodes=nodes), truth


def error_deg(a,b):
    return float(np.rad2deg((R.from_quat(a).inv()*b).magnitude()))


def run(model, corrector, end=10., drift=lambda t: 20., **kwargs):
    cal,binding,profile,*_=model
    last=None
    for time in np.arange(round(end*30)+1)/30:
        frame,truth=make_frame(model,time,drift(time),**kwargs)
        original=deepcopy(frame)
        output,diagnostic=corrector.apply(frame,profile,cal,binding)
        assert frame == original
        for pn,cn,p,c in PAIRS.values():
            assert output["nodes"][pn] == frame["nodes"][pn]
            assert output["nodes"][cn]["raw_values"] == frame["nodes"][cn]["raw_values"]
            assert output["nodes"][cn]["gyro_rad_s"] == frame["nodes"][cn]["gyro_rad_s"]
        last=frame,output,diagnostic,truth
    return last


def test_default_disabled_and_copy_independence():
    model=fixture_model();cal,binding,profile,*_=model
    frame,_=make_frame(model,0.,30.)
    output,d=CausalHeadingCorrector().apply(frame,profile,cal,binding)
    assert output == frame and not d["enabled"] and not d["applied_slots"]
    output["nodes"]["4"]["raw_values"][0]=99
    assert frame["nodes"]["4"]["raw_values"][0]==11


@pytest.mark.parametrize("global_motion",[False,True])
def test_clean_hinges_and_distinct_initial_yaws_are_unchanged(global_motion):
    model=fixture_model()
    frame,out,d,truth=run(model,CausalHeadingCorrector({"enabled":True}),end=4.,drift=lambda t:0.,global_motion=global_motion)
    assert out==frame and d["applied_slots"]==[]
    for detail in d["pairs"].values():
        assert detail["status"]=="tracking_relative_heading"
        assert abs(detail["delta_rad"]) < 1e-12


@pytest.mark.parametrize("drift",[20.,-25.])
def test_real_qmt_corrects_injected_child_yaw_without_changing_parent_or_calibration(drift):
    model=fixture_model(); before=deepcopy(model[2]); cal_before=deepcopy(model[0])
    frame,out,d,truth=run(model,CausalHeadingCorrector({"enabled":True}),end=12.,drift=lambda t:drift)
    assert d["applied_slots"] == sorted(HINGES.values())
    for _,cn,_,c in PAIRS.values():
        assert error_deg(out["nodes"][cn]["quat_xyzw"],truth[cn]) < .03
        assert np.rad2deg(d["pairs"][str(c)]["delta_rad"]) == pytest.approx(-drift,abs=.03)
    assert model[2]==before
    for key,val in vars(cal_before).items(): np.testing.assert_array_equal(getattr(model[0],key),val)


def test_causal_slow_drift_and_rate_limit_do_not_smooth_finger_flexion():
    model=fixture_model();c=CausalHeadingCorrector({"enabled":True})
    _,out,d,truth=run(model,c,end=12.,drift=lambda t:t)
    # Finite causal window and correction-only smoothing produce explicit lag,
    # but reduce the injected 12-degree error; this is not a zero-lag claim.
    for _,cn,_,slot in PAIRS.values():
        assert error_deg(out["nodes"][cn]["quat_xyzw"],truth[cn]) < 3.
        assert d["pairs"][str(slot)]["window_span_s"] <= 3.+1e-9
    assert d["uses_future_samples"] is False and d["correction_only_smoothing"] is True


def test_vertical_hinge_is_unobservable_and_passes_through():
    model=fixture_model()
    frame,out,d,_=run(model,CausalHeadingCorrector({"enabled":True}),end=4.,vertical=True)
    assert out==frame and not d["applied_slots"]
    assert all(v["status"]=="currently_unobservable_passthrough" for v in d["pairs"].values())


def test_missing_epoch_profile_or_backward_time_clears_history_and_never_stops():
    model=fixture_model();cal,binding,profile,*_=model;c=CausalHeadingCorrector({"enabled":True})
    frame,_,d,_=run(model,c,end=4.)
    missing=deepcopy(frame);missing["timestamp_s"]+=1/30;del missing["nodes"]["4"]
    out,d=c.apply(missing,profile,cal,binding)
    assert "4" not in out["nodes"] and d["pairs"]["2"]["reason"]=="missing_parent_or_child"
    new,_=make_frame(model,4.+2/30,20.);out,d=c.apply(new,profile,cal,binding)
    assert d["pairs"]["2"]["window_samples"]==1 and out["nodes"]["4"]==new["nodes"]["4"]
    new["nodes"]["4"]["epoch"]=1
    out,d=c.apply(new,profile,cal,binding)
    assert d["pairs"]["2"]["reason"]=="filter_epoch_changed"
    changed=deepcopy(profile);changed["functional_fingers"]["thumb"]["slot_refs"]["15"]["axis_local"]=[1,0,0]
    fresh,_=make_frame(model,4.1,20.);out,d=c.apply(fresh,changed,cal,binding)
    assert d["reset_reason"]=="profile_or_binding_changed" and not d["applied_slots"]
    earlier,_=make_frame(model,1.,20.);out,d=c.apply(earlier,changed,cal,binding)
    assert d["reset_reason"]=="clock_reversed_or_observation_gap" and not d["applied_slots"]


def test_binding_skew_and_missing_functional_profile_are_passthrough():
    model=fixture_model();cal,binding,profile,*_=model
    frame,_=make_frame(model,1.,20.)
    frame["nodes"]["4"]["age_s"]=.05
    c=CausalHeadingCorrector({"enabled":True});out,d=c.apply(frame,profile,cal,binding)
    assert d["pairs"]["2"]["reason"]=="parent_child_source_time_skew"
    assert out==frame
    profile["functional_fingers"]["index"]["binding"]["config_sha256"]="other"
    out,d=c.apply(frame,profile,cal,binding)
    assert d["pairs"]["2"]["reason"]=="functional_binding_mismatch"
    profile["functional_enabled"]=False
    out,d=c.apply(frame,profile,cal,binding)
    assert out==frame and d["reason"]=="functional_profile_disabled"


def test_repeated_source_samples_do_not_fill_window_or_claim_observation():
    model=fixture_model();cal,binding,profile,*_=model;c=CausalHeadingCorrector({"enabled":True})
    for i in range(100):
        frame,_=make_frame(model,i/30,20.)
        for n in frame["nodes"].values():n["sample_index"]=1
        output,d=c.apply(frame,profile,cal,binding)
        assert output==frame
    assert all(v["window_samples"] <= 1 for v in d["pairs"].values())


def test_initial_correction_is_zero_and_later_changes_are_rate_bounded():
    model=fixture_model();cal,binding,profile,*_=model;c=CausalHeadingCorrector({"enabled":True})
    previous=0.
    for i in range(150):
        frame,_=make_frame(model,i/30,20.)
        out,d=c.apply(frame,profile,cal,binding)
        delta=d["pairs"]["2"]["delta_rad"]
        assert abs(delta-previous) <= np.deg2rad(5/30)+1e-10
        if i<60: assert out==frame
        previous=delta


@pytest.mark.parametrize("config",[{"enabled":1},{"window_s":9},{"rating_min":2},{"unknown":1}])
def test_invalid_options_rejected(config):
    with pytest.raises(ValueError):CausalHeadingCorrector(config)
