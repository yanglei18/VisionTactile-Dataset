"""Synthetic paired rigid-body/isolated-IP calibration; no devices or files."""
from copy import deepcopy

import numpy as np
import pytest

from vt_glove_pose.math3d import from_axis_angle, to_matrix, rotate
from vt_glove_pose.thumb_calibration import (
    ThumbCalibrationError, calibrate_thumb_pair, fit_thumb_common_rotation,
)


def rx(theta):
    c,s=np.cos(theta),np.sin(theta)
    return np.array([[1.,0,0],[0,c,-s],[0,s,c]])


def make_capture(*, common_flex=.6, neutral_flex=.2, installation=0):
    rate=120.0
    t=np.arange(960)/rate
    mp=to_matrix(from_axis_angle([1,2,-.5], .7+.13*installation))
    md=to_matrix(from_axis_angle([-1,.3,2], -1.1+.17*installation))
    held=rx(-common_flex)
    gp=np.column_stack((23*np.sin(1.2*t),31*np.cos(.85*t),17*np.sin(1.4*t+.4)))
    gravity=np.array([.2,.3,.93]); gravity/=np.linalg.norm(gravity)
    world=np.eye(3)
    ap=[]
    for angular in gp:
        ap.append(world.T@gravity)
        speed=np.linalg.norm(angular)
        if speed:
            world=world@to_matrix(from_axis_angle(angular,np.deg2rad(speed)/rate))
    ap=np.asarray(ap)
    theta=neutral_flex+.35*(1-np.cos(2*np.pi*t/4))
    speed=np.degrees(.35*(2*np.pi/4)*np.sin(2*np.pi*t/4))
    distal_g=np.column_stack((-speed,np.zeros(len(t)),np.zeros(len(t))))
    distal_a=np.asarray([rx(angle)@gravity for angle in theta])
    capture=dict(
        common_prox_gyro_dps=gp@mp,
        common_dist_gyro_dps=gp@held@md,
        common_prox_acc_g=ap@mp,
        common_dist_acc_g=ap@held@md,
        flex_prox_gyro_dps=np.zeros((len(t),3)),
        flex_dist_gyro_dps=distal_g@md,
        flex_prox_acc_g=np.tile(gravity@mp,(len(t),1)),
        flex_dist_acc_g=distal_a@md,
        neutral_prox_acc_g=mp.T@gravity,
        neutral_dist_acc_g=md.T@rx(neutral_flex)@gravity,
    )
    truth=dict(rotation=mp.T@held@md,axis_prox=mp.T@np.array([-1.,0,0]),
               axis_dist=md.T@np.array([-1.,0,0]),theta=theta-neutral_flex,
               mp=mp,md=md)
    return capture,truth


@pytest.fixture(scope="module")
def baseline():
    return make_capture()


def common_kwargs(capture):
    return dict(prox_gyro_dps=capture["common_prox_gyro_dps"],
                dist_gyro_dps=capture["common_dist_gyro_dps"],
                prox_acc_g=capture["common_prox_acc_g"],
                dist_acc_g=capture["common_dist_acc_g"])


@pytest.mark.parametrize("installation",[0,1,2,3])
def test_one_physical_axis_maps_to_both_differently_installed_sensors(installation):
    capture,truth=make_capture(installation=installation)
    result=calibrate_thumb_pair(**capture)
    np.testing.assert_allclose(result["dist_to_prox_rotation"],truth["rotation"],atol=1e-12)
    np.testing.assert_allclose(result["shared_axis_prox"],truth["axis_prox"],atol=1e-12)
    np.testing.assert_allclose(result["shared_axis_dist"],truth["axis_dist"],atol=1e-12)
    np.testing.assert_allclose(np.asarray(result["dist_to_prox_rotation"])@result["shared_axis_dist"],
                               result["shared_axis_prox"],atol=1e-12)
    for name,axis in (("proximal",truth["axis_prox"]),("distal",truth["axis_dist"])):
        np.testing.assert_allclose(rotate(result["mounts"][name],axis),[-1,0,0],atol=1e-12)
    quality=result["quality"]
    assert quality["distal_linearity"]==pytest.approx(1)
    assert quality["neutral_axis_gravity_difference"]==pytest.approx(0,abs=1e-12)
    assert quality["hinge_gravity_difference_p95"]<1e-12
    assert not quality["anatomical_accuracy_verified"]
    assert not quality["hardware_time_synchronization_verified"]


@pytest.mark.parametrize("held_flex",[-.7,0,.3,1.0])
def test_common_stage_may_hold_any_fixed_ip_angle_not_assumed_straight(held_flex):
    capture,truth=make_capture(common_flex=held_flex,neutral_flex=.35)
    result=calibrate_thumb_pair(**capture)
    np.testing.assert_allclose(result["shared_axis_prox"],truth["axis_prox"],atol=1e-12)
    p=capture["flex_prox_acc_g"]@to_matrix(result["mounts"]["proximal"]).T
    d=capture["flex_dist_acc_g"]@to_matrix(result["mounts"]["distal"]).T
    measured=np.arctan2(-d[:,1],d[:,2])-np.arctan2(-p[:,1],p[:,2])
    np.testing.assert_allclose(measured,truth["theta"],atol=1e-12)
    assert "zero" not in result and "neutral_ready" not in result


def test_moderate_measurement_noise_has_small_shared_axis_error(baseline):
    capture,truth=deepcopy(baseline)
    rng=np.random.default_rng(19)
    for name,value in capture.items():
        if "gyro" in name:
            value+=rng.normal(0,.08,value.shape)
        elif "neutral" not in name:
            value+=rng.normal(0,.001,value.shape)
    result=calibrate_thumb_pair(**capture)
    np.testing.assert_allclose(result["dist_to_prox_rotation"],truth["rotation"],atol=.002)
    np.testing.assert_allclose(result["shared_axis_prox"],truth["axis_prox"],atol=.002)
    assert result["quality"]["common_motion"]["gyro_residual_rms_dps"]<.3


def test_pure_function_does_not_mutate_inputs_or_old_mounts_on_failure(baseline):
    capture,_=deepcopy(baseline)
    original=deepcopy(capture)
    old={5:[1,0,0,0],6:[1,0,0,0]}
    result=calibrate_thumb_pair(**capture)
    result["mounts"]["proximal"][0]=999
    for key in capture:
        np.testing.assert_array_equal(capture[key],original[key])
    capture["flex_prox_gyro_dps"][:,0]=10
    with pytest.raises(ThumbCalibrationError):
        calibrate_thumb_pair(**capture)
    assert old=={5:[1,0,0,0],6:[1,0,0,0]}
    np.testing.assert_array_equal(capture["flex_prox_gyro_dps"][:,0],10)


def test_stage_a_rejects_single_axis_excitation(baseline):
    capture,truth=deepcopy(baseline)
    p=np.tile([15.,0,0],(960,1))
    capture["common_prox_gyro_dps"]=p
    capture["common_dist_gyro_dps"]=p@truth["rotation"]
    with pytest.raises(ThumbCalibrationError,match="non-collinear"):
        calibrate_thumb_pair(**capture)


def test_stage_a_rejects_quiet_capture(baseline):
    capture,_=deepcopy(baseline)
    capture["common_prox_gyro_dps"][:]=0
    capture["common_dist_gyro_dps"][:]=0
    with pytest.raises(ThumbCalibrationError,match="insufficient common"):
        calibrate_thumb_pair(**capture)


def test_stage_a_rejects_amplitude_scale_mismatch(baseline):
    capture,_=deepcopy(baseline)
    capture["common_dist_gyro_dps"]*=1.3
    with pytest.raises(ThumbCalibrationError,match="amplitude"):
        calibrate_thumb_pair(**capture)


def test_stage_a_rejects_thumb_shape_changing_during_common_rotation(baseline):
    capture,_=deepcopy(baseline)
    perturb=to_matrix(from_axis_angle([.5,1,2],.6))
    capture["common_dist_gyro_dps"][480:]=capture["common_dist_gyro_dps"][480:]@perturb
    capture["common_dist_acc_g"][480:]=capture["common_dist_acc_g"][480:]@perturb
    with pytest.raises(ThumbCalibrationError,match="constant relative rotation"):
        calibrate_thumb_pair(**capture)


def test_stage_a_rejects_gravity_direction_disagreement(baseline):
    capture,_=deepcopy(baseline)
    capture["common_dist_acc_g"]*= -1
    with pytest.raises(ThumbCalibrationError,match="gravity directions"):
        calibrate_thumb_pair(**capture)


@pytest.mark.parametrize("name",["common_prox_gyro_dps","common_dist_gyro_dps","flex_prox_gyro_dps","flex_dist_gyro_dps"])
def test_saturation_rejected_in_either_stage_or_sensor(baseline,name):
    capture,_=deepcopy(baseline)
    capture[name][0,0]=286
    with pytest.raises(ThumbCalibrationError,match="near-limit"):
        calibrate_thumb_pair(**capture)


@pytest.mark.parametrize("stage",["common","flex"])
def test_raw_saturation_mask_prevents_bias_subtraction_hiding_saturation(baseline,stage):
    capture,_=baseline
    mask=np.zeros((960,2),dtype=bool)
    mask[25,1]=True
    with pytest.raises(ThumbCalibrationError,match="saturated raw"):
        calibrate_thumb_pair(**capture,**{stage+"_saturated":mask})


def test_stage_b_rejects_proximal_movement(baseline):
    capture,_=deepcopy(baseline)
    capture["flex_prox_gyro_dps"][:,0]=3
    with pytest.raises(ThumbCalibrationError,match="proximal segment moved"):
        calibrate_thumb_pair(**capture)


def test_stage_b_rejects_proximal_gravity_movement_even_if_gyro_claims_zero(baseline):
    capture,_=deepcopy(baseline)
    capture["flex_prox_acc_g"][480:]=capture["flex_prox_acc_g"][480:]@rx(.5)
    with pytest.raises(ThumbCalibrationError,match="proximal gravity direction moved"):
        calibrate_thumb_pair(**capture)


def test_stage_b_rejects_multiaxis_distal_or_cmc_motion(baseline):
    capture,_=deepcopy(baseline)
    capture["flex_dist_gyro_dps"][480:]=capture["flex_dist_gyro_dps"][480:]@rx(1.3)
    with pytest.raises(ThumbCalibrationError,match="single hinge"):
        calibrate_thumb_pair(**capture)


def test_stage_b_requires_both_flexion_and_extension(baseline):
    capture,truth=deepcopy(baseline)
    capture["flex_dist_gyro_dps"][:]=truth["axis_dist"]*20
    with pytest.raises(ThumbCalibrationError,match="flexion and extension"):
        calibrate_thumb_pair(**capture)


def test_first_short_meaningful_bout_cannot_be_replaced_by_later_sign(baseline):
    capture,truth=deepcopy(baseline)
    capture["flex_dist_gyro_dps"][:3]=truth["axis_dist"]*60
    capture["flex_dist_gyro_dps"][3:6]=0
    with pytest.raises(ThumbCalibrationError,match="first flexion is too short"):
        calibrate_thumb_pair(**capture)


def test_initial_flexion_finishing_before_point_four_seconds_keeps_correct_sign(baseline):
    capture,truth=deepcopy(baseline)
    # The complete first flexion is in 0..0.35s. A historical 0.4s trim would
    # incorrectly see extension first and flip both calibrated axes.
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
    result=calibrate_thumb_pair(**capture)
    np.testing.assert_allclose(result["shared_axis_dist"],truth["axis_dist"],atol=1e-12)
    np.testing.assert_allclose(result["shared_axis_prox"],truth["axis_prox"],atol=1e-12)
    np.testing.assert_allclose(rotate(result["mounts"]["distal"],truth["axis_dist"]),[-1,0,0],atol=1e-12)


def test_first_instruction_resolves_axis_sign_not_pca_arbitrary_sign(baseline):
    capture,_=deepcopy(baseline)
    # Change the complete sensor basis by a proper half turn, not anatomical sign.
    half=rx(np.pi)
    for key in ("common_dist_gyro_dps","common_dist_acc_g","flex_dist_gyro_dps","flex_dist_acc_g"):
        capture[key]=capture[key]@half
    capture["neutral_dist_acc_g"]=half.T@capture["neutral_dist_acc_g"]
    result=calibrate_thumb_pair(**capture)
    moving=np.flatnonzero(np.linalg.norm(capture["flex_dist_gyro_dps"],axis=1)>5)[0]
    first=rotate(result["mounts"]["distal"],capture["flex_dist_gyro_dps"][moving])
    assert first[0]<-5
    np.testing.assert_allclose(first[1:],0,atol=1e-12)


def test_neutral_gravity_component_mismatch_is_not_forced_to_match(baseline):
    capture,truth=deepcopy(baseline)
    capture["neutral_prox_acc_g"]=truth["mp"].T@np.array([.6,0,.8])
    with pytest.raises(ThumbCalibrationError,match="neutral gravity components disagree"):
        calibrate_thumb_pair(**capture)


def test_nonhinge_acceleration_projection_is_rejected_in_b(baseline):
    capture,truth=deepcopy(baseline)
    transverse=to_matrix(from_axis_angle([0,1,0],.6))
    capture["flex_dist_acc_g"]=(capture["flex_dist_acc_g"]@truth["md"].T@transverse)@truth["md"]
    with pytest.raises(ThumbCalibrationError,match="shared-axis gravity components disagree"):
        calibrate_thumb_pair(**capture)


def test_axis_parallel_to_gravity_is_unobservable(baseline):
    capture,truth=deepcopy(baseline)
    up=np.array([.99,0,np.sqrt(1-.99**2)])
    capture["neutral_prox_acc_g"]=truth["mp"].T@up
    capture["neutral_dist_acc_g"]=truth["md"].T@up
    with pytest.raises(ThumbCalibrationError,match="unobservable"):
        calibrate_thumb_pair(**capture)


@pytest.mark.parametrize("bad",[float("nan"),float("inf"),-float("inf")])
def test_nonfinite_capture_is_rejected(baseline,bad):
    capture,_=deepcopy(baseline)
    capture["common_prox_acc_g"][20,2]=bad
    with pytest.raises(ThumbCalibrationError,match="finite"):
        calibrate_thumb_pair(**capture)


@pytest.mark.parametrize("change",["short","shape","unpaired","zero_acc","dynamic_acc"])
def test_malformed_or_unreliable_common_capture_is_rejected(baseline,change):
    capture,_=deepcopy(baseline)
    args=common_kwargs(capture)
    if change=="short": args={k:v[:20] for k,v in args.items()}
    if change=="shape": args["prox_acc_g"]=np.zeros((960,2))
    if change=="unpaired": args["dist_gyro_dps"]=args["dist_gyro_dps"][:-1]
    if change=="zero_acc": args["dist_acc_g"][:]=0
    if change=="dynamic_acc": args["dist_acc_g"]*=1.5
    with pytest.raises(ThumbCalibrationError):
        fit_thumb_common_rotation(**args)


@pytest.mark.parametrize("kwargs",[
    {"sample_rate_hz":True},{"sample_rate_hz":0},{"sample_rate_hz":float("nan")},
    {"gyro_saturation_dps":False},{"gyro_saturation_dps":-1},
    {"common_saturated":[1]*960},{"flex_saturated":np.zeros(2,dtype=bool)},
])
def test_invalid_options_are_rejected(baseline,kwargs):
    capture,_=baseline
    with pytest.raises(ThumbCalibrationError):
        calibrate_thumb_pair(**capture,**kwargs)
