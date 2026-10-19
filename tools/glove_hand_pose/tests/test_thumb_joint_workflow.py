"""Real PoseEngine A/B workflow using synthetic capture rows, never hardware."""
from copy import deepcopy

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.diagnostic_recording import DiagnosticRecorder
from vt_glove_pose.diagnostics import diagnose_session
from vt_glove_pose.engine import PoseEngine
from vt_glove_pose.math3d import conj, from_axis_angle, mul, rotate
from vt_glove_pose.records import Sample
from vt_glove_pose.thumb_flexion import ThumbFlexionEstimator

from test_thumb_calibration import make_capture, rx


I=np.array([1.,0.,0.,0.])


@pytest.fixture(scope="module")
def synthetic():
    return make_capture(common_flex=.65,neutral_flex=.2)


def ready_engine(capture, *, legacy=False):
    engine=PoseEngine(load_config(),algorithm_mode="legacy_quaternion_v1" if legacy else "thumb_paired_gyro_gravity_v1")
    for node in range(1,13):
        engine.process(Sample(node,(0,0,0,0,0,8197),10.,1000.))
    engine.neutral_ready=True
    engine.mount_confirmed=True
    engine.mounts={node:I.copy() for node in range(1,13)}
    engine.zero={node:I.copy() for node in range(1,13)}
    engine.axis_sign_versions={node:2 for node in range(1,13)}
    engine.axes_quality={node:{"previous":node} for node in range(1,13)}
    engine.neutral_acc={node:np.array([0.,0.,1.]) for node in range(1,13)}
    ids=thumb_nodes(engine)
    for node,key in zip(ids,("neutral_prox_acc_g","neutral_dist_acc_g")):
        engine.neutral_acc[node]=capture[key].copy()
        engine.bias[node]=np.array([.12,-.08,.04])
    return engine


def thumb_nodes(engine):
    return tuple(engine.config["fingers"]["thumb"][key] for key in ("proximal","distal"))


def saved_state(engine):
    return deepcopy((engine.mounts,engine.axes_quality,engine.axis_sign_versions,engine.zero))


def assert_same_saved(engine,previous):
    for current,old in ((engine.mounts,previous[0]),(engine.zero,previous[3])):
        assert current.keys()==old.keys()
        for node in old:
            np.testing.assert_array_equal(current[node],old[node])
    assert engine.axes_quality==previous[1]
    assert engine.axis_sign_versions==previous[2]


def prepare_capture(engine,target,data):
    engine.begin_axis(target,duration=8)
    active=engine.active
    prefix="common" if target=="thumb_common" else "flex"
    rate=engine.config["sample_rate_hz"]
    for node,label in zip(thumb_nodes(engine),("prox","dist")):
        gyros=data[f"{prefix}_{label}_gyro_dps"]
        accels=data[f"{prefix}_{label}_acc_g"]
        active["rows"][node]=[(active["start"]+i/rate,gyro+engine.bias[node],acc.copy(),I.copy())
                               for i,(gyro,acc) in enumerate(zip(gyros,accels))]
    return active


def finish_capture(engine):
    end=engine.active["end"]
    engine._finish_calibration()
    # Direct capture injection avoids thousands of irrelevant VQF updates.
    # Advance the already available state timestamps like a successful read.
    engine.last_time=end
    for state in engine.states.values():
        state["time"]=end
        state["times"].append(end)


def stage_a(engine,capture):
    prepare_capture(engine,"thumb_common",capture)
    finish_capture(engine)
    assert engine.thumb_common_capture is not None,engine.message


def stage_b(engine,capture):
    prepare_capture(engine,"thumb_hinge",capture)
    finish_capture(engine)


def test_a_success_caches_capture_without_replacing_any_existing_axis(synthetic):
    capture,_=synthetic
    engine=ready_engine(capture)
    previous=saved_state(engine)
    stage_a(engine,capture)
    assert "A 已通过" in engine.message
    assert engine.active is None
    assert_same_saved(engine,previous)
    assert set(engine.thumb_common_capture)=={"prox_gyro","dist_gyro","prox_acc","dist_acc"}
    # Caller capture has bias added; the cached numerical API receives it removed.
    expected=capture["common_prox_gyro_dps"][48:936]
    np.testing.assert_allclose(engine.thumb_common_capture["prox_gyro"],expected,atol=1e-12)


@pytest.mark.parametrize("installation",[0,2])
def test_b_success_commits_both_shared_axes_and_preserves_other_fingers(installation):
    capture,truth=make_capture(installation=installation)
    engine=ready_engine(capture)
    previous=saved_state(engine)
    stage_a(engine,capture)
    stage_b(engine,capture)
    assert "联合轴已保存" in engine.message,engine.message
    assert engine.thumb_common_capture is None
    assert engine.thumb_estimator is None
    assert engine.neutral_ready
    for node,key in zip(thumb_nodes(engine),("axis_prox","axis_dist")):
        np.testing.assert_allclose(rotate(engine.mounts[node],truth[key]),[-1,0,0],atol=1e-12)
        assert engine.axes_quality[node]["joint_method"]=="paired_common_rotation_hinge_v1"
        assert engine.axes_quality[node]["neutral_axis_gravity_difference"]==pytest.approx(0,abs=1e-12)
        assert engine.axis_sign_versions[node]==2
    for node in set(range(1,13))-set(thumb_nodes(engine)):
        np.testing.assert_array_equal(engine.mounts[node],previous[0][node])
        assert engine.axes_quality[node]==previous[1][node]
    for node in range(1,13):
        np.testing.assert_array_equal(engine.zero[node],previous[3][node])


@pytest.mark.parametrize("problem",["proximal_moves","saturation","unpaired","timestamp_skew","missing_row_gap"])
def test_b_failure_retains_both_old_axes_and_accepted_a_for_retry(synthetic,problem):
    capture,_=deepcopy(synthetic)
    engine=ready_engine(capture)
    stage_a(engine,capture)
    previous=saved_state(engine)
    accepted_a=deepcopy(engine.thumb_common_capture)
    if problem=="proximal_moves": capture["flex_prox_gyro_dps"][:,0]=3
    active=prepare_capture(engine,"thumb_hinge",capture)
    prox,dist=thumb_nodes(engine)
    if problem=="saturation":
        stamp,gyro,acc,q=active["rows"][dist][10]
        gyro[0]=32764*engine.config["gyro_dps_per_lsb"]
    if problem=="unpaired": active["rows"][dist].pop(100)
    if problem=="timestamp_skew":
        active["rows"][dist]=[(t+.03,g,a,q) for t,g,a,q in active["rows"][dist]]
    if problem=="missing_row_gap":
        for node in (prox,dist): del active["rows"][node][100:180]
    finish_capture(engine)
    assert "未通过" in engine.message
    assert_same_saved(engine,previous)
    assert engine.thumb_common_capture==accepted_a
    assert engine.active is None and engine.neutral_ready


def test_b_before_a_is_rejected_without_starting_capture(synthetic):
    capture,_=synthetic
    engine=ready_engine(capture)
    previous=saved_state(engine)
    with pytest.raises(ValueError,match="联合标定 A"):
        engine.begin_axis("thumb_hinge")
    assert engine.active is None
    assert_same_saved(engine,previous)


def test_a_rejects_nonrigid_capture_without_changing_saved_axes(synthetic):
    capture,_=deepcopy(synthetic)
    engine=ready_engine(capture)
    previous=saved_state(engine)
    capture["common_dist_gyro_dps"]*=1.5
    prepare_capture(engine,"thumb_common",capture)
    finish_capture(engine)
    assert "未通过" in engine.message
    assert engine.thumb_common_capture is None
    assert_same_saved(engine,previous)


def test_engine_does_not_trim_away_first_flexion_before_point_four_seconds(synthetic):
    capture,truth=deepcopy(synthetic)
    engine=ready_engine(capture)
    stage_a(engine,capture)
    speed=np.zeros(960)
    speed[:42]=60
    speed[42:84]=-60
    for start in range(120,840,240):
        speed[start:start+120]=30
        speed[start+120:start+240]=-30
    theta=.2+np.deg2rad(np.cumsum(speed)/120)
    gravity=truth["mp"]@capture["neutral_prox_acc_g"]
    capture["flex_dist_gyro_dps"]=speed[:,None]*truth["axis_dist"]
    capture["flex_dist_acc_g"]=np.asarray([rx(angle)@gravity for angle in theta])@truth["md"]
    stage_b(engine,capture)
    assert "联合轴已保存" in engine.message,engine.message
    for node,key in zip(thumb_nodes(engine),("axis_prox","axis_dist")):
        np.testing.assert_allclose(rotate(engine.mounts[node],truth[key]),[-1,0,0],atol=1e-12)


def test_successful_new_neutral_clears_pending_a_but_preserves_mounts(synthetic):
    capture,_=synthetic
    engine=ready_engine(capture)
    stage_a(engine,capture)
    old_mounts=deepcopy(engine.mounts)
    engine.begin_neutral(duration=5,confirm_mount=True)
    for node in range(1,13):
        engine.active["rows"][node]=[(engine.active["start"]+i/120,engine.bias[node].copy(),
                                      engine.neutral_acc[node].copy(),I.copy()) for i in range(600)]
    finish_capture(engine)
    assert "静止归零完成" in engine.message
    assert engine.neutral_ready and engine.thumb_common_capture is None
    for node in range(1,13):
        np.testing.assert_array_equal(engine.mounts[node],old_mounts[node])
    with pytest.raises(ValueError,match="联合标定 A"):
        engine.begin_axis("thumb_hinge")


def test_load_clears_pending_a_and_requires_fresh_neutral(synthetic,tmp_path):
    capture,_=synthetic
    engine=ready_engine(capture)
    stage_a(engine,capture)
    path=tmp_path/"old-axes.json"
    engine.save_calibration(path)
    engine.load_calibration(path)
    assert engine.thumb_common_capture is None
    assert not engine.neutral_ready
    assert engine.zero=={}
    assert engine.thumb_estimator is None


@pytest.mark.parametrize("time_change",[.6,-.1])
def test_data_gap_or_reorder_clears_pending_a_without_deleting_axes(synthetic,time_change):
    capture,_=synthetic
    engine=ready_engine(capture)
    stage_a(engine,capture)
    old=deepcopy(engine.mounts)
    engine.process(Sample(1,(0,0,0,0,0,8197),engine.last_time+time_change,1000.))
    assert engine.thumb_common_capture is None
    assert not engine.neutral_ready
    for node in old:
        np.testing.assert_array_equal(engine.mounts[node],old[node])


def test_cancelled_b_can_retry_using_valid_a(synthetic):
    capture,_=synthetic
    engine=ready_engine(capture)
    stage_a(engine,capture)
    accepted=deepcopy(engine.thumb_common_capture)
    engine.begin_axis("thumb_hinge")
    engine.cancel_calibration()
    assert engine.thumb_common_capture==accepted
    stage_b(engine,capture)
    assert "联合轴已保存" in engine.message


def test_snapshot_uses_new_thumb_estimator_and_keeps_other_fingers_enabled(synthetic):
    capture,_=synthetic
    engine=ready_engine(capture)
    stage_a(engine,capture)
    stage_b(engine,capture)
    prox,dist=thumb_nodes(engine)
    engine.thumb_estimator=ThumbFlexionEstimator(engine.config["sample_rate_hz"])
    engine.thumb_estimator.reset(rotate(engine.mounts[prox],engine.neutral_acc[prox]),
                                 rotate(engine.mounts[dist],engine.neutral_acc[dist]))
    # Deliberately disagreeing old quaternion estimate: new estimator stays at
    # the explicitly established neutral instead of falling back to old IP.
    engine.states[prox]["quat"]=I.copy()
    engine.states[dist]["quat"]=mul(conj(engine.mounts[dist]),
        mul(from_axis_angle([-1,0,0],.7),engine.mounts[dist]))
    snapshot=engine.get_snapshot(now=engine.last_time)
    thumb=snapshot["model"]["fingers"]["thumb"]
    assert thumb["ip_deg"]==pytest.approx(0,abs=1e-12)
    assert thumb["raw_angles_deg"]["ip_deg"]==pytest.approx(0,abs=1e-12)
    assert thumb["ip_quaternion_deg"]==pytest.approx(np.degrees(.7),abs=1e-10)
    assert thumb["ip_estimator"]["quality"]=="tracking"
    assert thumb["ip_confident"]
    for name,finger in snapshot["model"]["fingers"].items():
        if name!="thumb":
            assert finger["valid"] and finger["geometry_live"]
            assert finger["pip_valid"] and "ip_estimator" not in finger


def test_legacy_offline_engine_does_not_accept_new_calibration_commands(synthetic):
    capture,_=synthetic
    engine=ready_engine(capture,legacy=True)
    for name in ("thumb_common","thumb_hinge"):
        with pytest.raises(ValueError,match="未知"):
            engine.begin_axis(name)


@pytest.mark.parametrize("calibration_method", ["strict_v1", "robust_v2"])
def test_naturally_processed_joint_commands_reproduce_in_diagnostic_session(synthetic,tmp_path,calibration_method):
    """Replay A's pending state and B's commit from raw packets, not injected rows."""
    capture,truth=synthetic
    engine=ready_engine(capture)
    initial=engine.export_diagnostic_checkpoint()
    assert PoseEngine.from_offline_checkpoint(initial).export_diagnostic_checkpoint()==initial
    previous=saved_state(engine)
    path=tmp_path/"natural-thumb-joint-session.jsonl"
    recorder=DiagnosticRecorder(path)
    assert recorder.start(initial)
    rate=engine.config["sample_rate_hz"]
    gyro_scale=engine.config["gyro_dps_per_lsb"]
    acc_scale=engine.config["accel_g_per_lsb"]
    thumb=dict(zip(thumb_nodes(engine),("prox","dist")))
    total_records=0
    try:
        for target,prefix in (("thumb_common","common"),("thumb_hinge","flex")):
            args = dict(target=target,duration=8)
            if calibration_method != "strict_v1":
                args["calibration_method"] = calibration_method
            engine.begin_axis(**args)
            assert recorder.offer("command",name="axis",args=args)
            start=engine.last_time
            batch=[]
            for tick in range(960):
                stamp=start+(tick+1)/rate
                for node in range(1,13):
                    if node in thumb:
                        label=thumb[node]
                        gyro=capture[f"{prefix}_{label}_gyro_dps"][tick]+engine.bias[node]
                        acc=capture[f"{prefix}_{label}_acc_g"][tick]
                    else:
                        gyro=engine.bias[node]
                        acc=engine.neutral_acc[node]
                    raw=np.rint(np.r_[gyro/gyro_scale,acc/acc_scale]).astype(int)
                    assert np.all(np.abs(raw)<32700)
                    sample=Sample(node,tuple(int(value) for value in raw),stamp,1000.+stamp)
                    engine.process(sample)
                    batch.append(dict(node_id=node,values=list(sample.values),
                                      host_monotonic_s=stamp,host_wall_s=sample.host_wall_s,status=None))
                # Bounded USB-like batches retain all sample order while giving
                # the writer enough work per event without real-time sleeping.
                if (tick+1)%60==0:
                    assert recorder.offer("batch",samples=batch,performance={}),recorder.status()
                    total_records+=len(batch)
                    batch=[]
            assert engine.active is None
            assert engine.neutral_ready and engine.gap_count==engine.invalid_count==0
            assert all(state["count"]==1+total_records//12 and len(state["times"])==min(512,state["count"])
                       for state in engine.states.values())
            if target=="thumb_common":
                assert "A 已通过" in engine.message,engine.message
                assert engine.thumb_common_capture is not None
                assert_same_saved(engine,previous)
                checkpoint = engine.export_diagnostic_checkpoint()
                assert PoseEngine.from_offline_checkpoint(checkpoint).export_diagnostic_checkpoint() == checkpoint
                assert recorder.offer("checkpoint", checkpoint=checkpoint)
            else:
                assert "联合轴已保存" in engine.message,engine.message
                assert engine.thumb_common_capture is None
                for node,key in zip(thumb_nodes(engine),("axis_prox","axis_dist")):
                    np.testing.assert_allclose(rotate(engine.mounts[node],truth[key]),[-1,0,0],atol=2e-4)
                    expected = "paired_common_rotation_hinge_v1" if calibration_method == "strict_v1" else "paired_common_rotation_hinge_robust_v2"
                    assert engine.axes_quality[node]["joint_method"] == expected
            snapshot=engine.get_snapshot(now=engine.last_time)
            assert recorder.offer("snapshot",now_s=engine.last_time,snapshot=snapshot,performance={})
        assert snapshot["model"]["fingers"]["thumb"]["ip_estimator"] is not None
    finally:
        recorder.stop("test_complete")
        assert recorder.wait_closed(.5),recorder.status()
    assert recorder.status()["state"]=="complete",recorder.status()
    report=diagnose_session(path)
    assert report["session_complete"],report
    assert report["reconstruction"]["status"]=="match",report["reconstruction"]
    assert report["reconstruction"]["algorithm_mode"]=="thumb_paired_gyro_gravity_v1"
    assert report["reconstruction"]["compared_snapshots"]==2
    assert report["counts"]["commands"]==2
    assert report["total_records"]==total_records==2*960*12
