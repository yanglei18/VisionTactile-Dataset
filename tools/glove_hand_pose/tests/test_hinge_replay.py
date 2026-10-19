"""Offline natural calibration -> existing observer, with synthetic angle truth."""
import copy
import csv
import json

import numpy as np
import pytest

from vt_glove_pose.config import load_config
from vt_glove_pose.hinge_replay import calibrate_hinge_pair, replay_hinge_pair, compare_hinge_pair
from vt_glove_pose.math3d import from_axis_angle, to_matrix
from vt_glove_pose.stream import CSV_FIELDS


RATE, GYRO_SCALE, ACC_SCALE = 120., .00875, .000122
MP = to_matrix(from_axis_angle([.3, -.4, .8], .9))
MD = to_matrix(from_axis_angle([.8, .2, -.3], -1.1))
GRAVITY = np.array([.25, .2, np.sqrt(1-.25**2-.2**2)])
BP, BD = np.array([.6, -.3, .2]), np.array([-.2, .4, .1])


def capture(gp, gd, ap, ad, name):
    n = len(gp)
    raw_p = np.rint(gp/GYRO_SCALE).astype(np.int16)
    raw_d = np.rint(gd/GYRO_SCALE).astype(np.int16)
    return dict(prox_gyro_raw=raw_p, dist_gyro_raw=raw_d,
                prox_gyro_dps=raw_p.astype(float)*GYRO_SCALE,
                dist_gyro_dps=raw_d.astype(float)*GYRO_SCALE,
                prox_acc_raw=np.rint(ap/ACC_SCALE).astype(np.int16),
                dist_acc_raw=np.rint(ad/ACC_SCALE).astype(np.int16),
                near_limit_mask=np.zeros((n, 2), bool),
                host_monotonic_s=np.repeat((10.+np.arange(n)/RATE)[:, None], 2, axis=1),
                metadata=dict(source=name, proximal=5, distal=6,
                              sample_rate_hz=RATE, gyro_dps_per_lsb=GYRO_SCALE,
                              dropped_pair_candidates=0))


def neutral_capture():
    n=360
    return capture(np.tile(BP, (n,1)), np.tile(BD, (n,1)),
                   np.tile(GRAVITY @ MP, (n,1)), np.tile(GRAVITY @ MD, (n,1)), "neutral")


def motion_capture(phase=0., n=1200):
    """Analytic body rates of Rz(yaw) Ry(pitch) Rx(roll), plus hinge Rx(theta)."""
    t=np.arange(n)/RATE
    yaw=.4*np.sin(.7*t); pitch=.45*np.sin(1.1*t); roll=.3*np.sin(1.4*t)
    yaw_rate=.28*np.cos(.7*t); pitch_rate=.495*np.cos(1.1*t); roll_rate=.42*np.cos(1.4*t)
    theta=.55+.5*np.sin(1.8*t+phase)
    theta_rate=.9*np.cos(1.8*t+phase)
    pg=[]; dg=[]; pa=[]; da=[]
    for i in range(n):
        rx=to_matrix(from_axis_angle([1,0,0], roll[i]))
        ry=to_matrix(from_axis_angle([0,1,0], pitch[i]))
        rz=to_matrix(from_axis_angle([0,0,1], yaw[i]))
        h=to_matrix(from_axis_angle([1,0,0], theta[i]))
        common=rx.T @ ry.T @ [0,0,yaw_rate[i]] + rx.T @ [0,pitch_rate[i],0] + [roll_rate[i],0,0]
        distal=h.T @ common + [theta_rate[i],0,0]
        pg.append(np.rad2deg(common) @ MP + BP)
        dg.append(np.rad2deg(distal) @ MD + BD)
        gravity=(rz @ ry @ rx).T @ GRAVITY
        pa.append(gravity @ MP); da.append((h.T @ gravity) @ MD)
    return capture(np.array(pg), np.array(dg), np.array(pa), np.array(da), f"motion_{phase}"), np.rad2deg(theta)


@pytest.fixture(scope="module")
def calibrated():
    neutral=neutral_capture()
    motion,_=motion_capture()
    profile=calibrate_hinge_pair(neutral,motion,accel_g_per_lsb=ACC_SCALE)
    assert profile["profile_status"] == "offline_candidate_ready", profile
    return profile


def test_natural_calibration_runs_without_legacy_ab_and_preserves_reference(calibrated):
    assert calibrated["axis_fit"]["candidate_status"] == "candidate_supported"
    assert calibrated["frames"]["status"] == "frames_ready"
    assert not calibrated["usable_as_live_calibration"]
    assert not calibrated["anatomical_flexion_sign_verified"]
    np.testing.assert_allclose(calibrated["neutral"]["prox_bias_dps"], BP, atol=GYRO_SCALE)
    json.dumps(calibrated, allow_nan=False)


def test_independent_movement_replay_has_nonzero_initial_pose_and_tracks_return(calibrated):
    test,truth=motion_capture(phase=.6)
    before=test["prox_gyro_dps"].copy()
    report=replay_hinge_pair(calibrated,test)
    assert report["replay_status"] == "completed"
    sign=np.sign(np.dot(calibrated["axis_fit"]["axis_prox"], MP.T @ [1,0,0]))
    error=np.array([s["angle_deg"] for s in report["series"]])-sign*truth
    assert abs(error[0]) < .1
    assert np.percentile(abs(error),95) < .6
    assert abs(error[-1]) < .6
    assert abs(report["series"][0]["angle_deg"]) > 20
    assert report["summary"]["quality_counts"]["tracking"] > .95*len(truth)
    assert not report["summary"]["history_loss"]
    assert not report["independent_angle_truth_available"]  # CLI inputs themselves have no labels.
    np.testing.assert_array_equal(before,test["prox_gyro_dps"])
    json.dumps(report,allow_nan=False)


def test_explicit_candidate_and_comparison_share_input_not_states(calibrated):
    data,truth=motion_capture(phase=.4,n=600)
    original=data["prox_gyro_dps"].copy()
    comparison=compare_hinge_pair(calibrated,data)
    assert not comparison["independent_angle_truth_available"]
    assert not comparison["live_calibration_changed"]
    assert comparison["disagreement"]["samples"]==600
    sign=np.sign(np.dot(calibrated["axis_fit"]["axis_prox"], MP.T @ [1,0,0]))
    for name in ("baseline","constrained"):
        result=comparison[name]
        assert result["replay_status"]=="completed"
        estimate=np.array([s["angle_deg"] for s in result["series"]])
        assert np.percentile(abs(estimate-sign*truth),95)<.8
    assert comparison["constrained"]["summary"]["final_relative_bias_dps"] is not None
    np.testing.assert_array_equal(original,data["prox_gyro_dps"])
    repeated=replay_hinge_pair(calibrated,data,observer="constrained")
    assert repeated==comparison["constrained"]


def test_unknown_observer_cannot_silently_fall_back(calibrated):
    with pytest.raises(ValueError,match="observer"):
        replay_hinge_pair(calibrated,neutral_capture(),observer="typo")


def test_quiet_is_not_straight_no_automatic_zero(calibrated):
    motion,truth=motion_capture(phase=.2,n=1)
    n=480
    still=capture(np.tile(BP,(n,1)),np.tile(BD,(n,1)),
                  np.tile(motion["prox_acc_raw"][0]*ACC_SCALE,(n,1)),
                  np.tile(motion["dist_acc_raw"][0]*ACC_SCALE,(n,1)),"bent_hold")
    report=replay_hinge_pair(calibrated,still)
    angles=np.array([s["angle_deg"] for s in report["series"]])
    assert abs(angles[-1]) > 30
    assert np.ptp(angles) < .1


@pytest.mark.parametrize("problem",["near_limit","host_gap","discarded_pair"])
def test_input_loss_is_not_silently_recovered(calibrated,problem):
    test,_=motion_capture(phase=.2,n=360)
    if problem=="near_limit":
        test["near_limit_mask"][100,1]=True
    elif problem=="host_gap":
        test["host_monotonic_s"][100:]+=1
    else:
        test["metadata"]["dropped_pair_candidates"]=2
    result=replay_hinge_pair(calibrated,test)
    assert result["summary"]["history_loss"]
    assert result["series"][-1]["uncertain"]


def test_first_sample_limit_marks_history_loss_without_integrating(calibrated):
    test,_=motion_capture(n=2)
    test["near_limit_mask"][0,0]=True
    result=replay_hinge_pair(calibrated,test)
    assert result["series"][0]["history_loss"]
    assert result["series"][0]["raw_near_limit"]


@pytest.mark.parametrize("case",["short","moving","clipped"])
def test_rejects_unsuitable_neutral_without_inventing_profile(case):
    neutral=neutral_capture(); motion,_=motion_capture(n=300)
    if case=="short":
        for key,value in neutral.items():
            if isinstance(value,np.ndarray): neutral[key]=value[:100]
    elif case=="moving":
        neutral["prox_gyro_dps"][:,0]+=5*np.sin(np.arange(360))
    else:
        neutral["near_limit_mask"][4,0]=True
    profile=calibrate_hinge_pair(neutral,motion,accel_g_per_lsb=ACC_SCALE)
    assert profile["profile_status"] in ("insufficient_neutral","neutral_invalid")
    assert profile["axis_fit"] is None
    with pytest.raises(ValueError,match="ready offline"):
        replay_hinge_pair(profile,motion)


def test_static_motion_is_not_free_motion_calibration():
    profile=calibrate_hinge_pair(neutral_capture(),neutral_capture(),accel_g_per_lsb=ACC_SCALE)
    assert profile["profile_status"]=="axis_insufficient_excitation"


def test_mismatched_nodes_or_units_never_silently_replay(calibrated):
    motion,_=motion_capture(n=300)
    motion["metadata"]["proximal"]=3
    with pytest.raises(ValueError,match="differ"):
        replay_hinge_pair(calibrated,motion)
    motion["metadata"]["proximal"]=5
    motion["metadata"]["source_configuration"]=load_config()
    motion["metadata"]["source_configuration"]["gyro_dps_per_lsb"]*=2
    with pytest.raises(ValueError,match="recorded"):
        replay_hinge_pair(calibrated,motion)


def test_corrupt_frame_does_not_enter_observer(calibrated):
    damaged=copy.deepcopy(calibrated)
    damaged["frames"]["prox_sensor_to_functional"][0][0]+=1
    with pytest.raises(ValueError,match="proper rotations"):
        replay_hinge_pair(damaged,neutral_capture())


@pytest.mark.parametrize("change", ["missing_nodes", "bad_nodes", "bad_scale"])
def test_malformed_profile_settings_give_actionable_error(calibrated, change):
    damaged = copy.deepcopy(calibrated)
    if change == "missing_nodes":
        del damaged["settings"]["node_ids"]
    elif change == "bad_nodes":
        damaged["settings"]["node_ids"] = [5]
    else:
        damaged["settings"]["gyro_dps_per_lsb"] = float("nan")
    with pytest.raises(ValueError):
        replay_hinge_pair(damaged, neutral_capture())


def write_capture(path,data):
    with path.open("w",newline="") as stream:
        writer=csv.writer(stream); writer.writerow(CSV_FIELDS)
        for i,(tp,td) in enumerate(data["host_monotonic_s"]):
            for node,label,t in ((5,"prox",tp),(6,"dist",td)):
                writer.writerow([t,1000+t,node,*data[label+"_gyro_raw"][i],*data[label+"_acc_raw"][i],0])


def test_cli_calibrate_and_replay_files_end_to_end(tmp_path):
    from vt_glove_pose.cli import main
    neutral=tmp_path/"neutral.csv"; motion=tmp_path/"motion.csv"; test=tmp_path/"test.csv"
    write_capture(neutral,neutral_capture())
    write_capture(motion,motion_capture()[0])
    write_capture(test,motion_capture(phase=.3,n=360)[0])
    profile=tmp_path/"offline_profile.json"; result=tmp_path/"angles.json"
    assert main(["calibrate-hinge",str(neutral),str(motion),"--output",str(profile)])==0
    assert json.loads(profile.read_text())["profile_status"]=="offline_candidate_ready"
    assert main(["replay-hinge",str(test),"--profile",str(profile),"--output",str(result)])==0
    report=json.loads(result.read_text())
    assert report["summary"]["samples"]==360
    assert report["offline_only"] and not report["live_calibration_changed"]
    before=result.read_bytes()
    assert main(["replay-hinge",str(test),"--profile",str(profile),"--output",str(result)])==2
    assert result.read_bytes()==before
    comparison=tmp_path/"compare.json"
    assert main(["compare-hinge",str(test),"--profile",str(profile),"--output",str(comparison)])==0
    saved=json.loads(comparison.read_text())
    assert saved["disagreement"]["samples"]==360
    assert not saved["independent_angle_truth_available"]
