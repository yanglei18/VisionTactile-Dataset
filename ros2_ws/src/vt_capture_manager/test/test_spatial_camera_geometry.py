from types import SimpleNamespace as NS

import pytest

from vt_capture_manager.spatial.camera_geometry import CameraGeometryRegistry


def camera_config():
    return dict(id="cam", label="Camera", serial="123", tracker_role="torso",
                color_frame_id="cam_color_optical_frame", depth_frame_id="cam_depth_optical_frame")


def camera_info(frame="cam_color_optical_frame", fx=100.):
    return NS(header=NS(frame_id=frame), width=100, height=80,
              k=[fx, 0., 50., 0., fx, 40., 0., 0., 1.], d=[0.]*5,
              r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
              p=[fx, 0., 50., 0., 0., fx, 40., 0., 0., 0., 1., 0.],
              distortion_model="plumb_bob", binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False))


def extrinsic():
    return dict(schema_version=1, status="VALID", camera=dict(name="cam", serial="123", frame_id="cam_color_optical_frame"),
                tracker=dict(role="torso", tracker_id="LHR-123", frame_id="vive_tracker_torso"),
                transform=dict(semantics="parent_from_child", parent_frame="vive_tracker_torso",
                               child_frame="cam_color_optical_frame", translation_m=[.1, 0., 0.], quaternion_xyzw=[0., 0., 0., 1.]),
                frames=dict(world="vive_map"), quality=dict(grade="ACCEPTED"))


def tracker():
    return dict(id="torso", tracker_id="LHR-123", position_m=[1., 2., 3.],
                quaternion_xyzw=[0., 0., 0., 1.], state="fresh", source_realtime_ns=1_000_000_000)


def prepared():
    registry = CameraGeometryRegistry([camera_config()])
    registry.observe_camera_info("cam", camera_info())
    registry.set_extrinsic("cam", extrinsic())
    return registry


def test_device_info_is_required_and_timing_identity_cannot_substitute():
    registry = prepared()
    assert registry.camera_entities([tracker()], now_ns=1_000_000_000)[0]["registration_state"] == "identity_unverified"
    registry.observe_device_info("cam", NS(serial_number="123", device_name="D405", firmware_version="test"), observed_ns=1_000_000_000)
    entity = registry.camera_entities([tracker()], now_ns=1_000_000_000)[0]
    assert entity["state"] == "fresh"
    assert entity["position_m"] == pytest.approx([1.1, 2., 3.])
    assert len(entity["frustum_vertices"]) == 8
    assert entity["intrinsics"]["fx"] == 100.
    assert registry.camera_entities([tracker()], now_ns=7_000_000_000)[0]["registration_state"] == "identity_unverified"


def test_serial_and_tracker_mismatch_hide_camera_pose():
    registry = prepared()
    registry.observe_device_info("cam", NS(serial_number="999"), observed_ns=1_000_000_000)
    entity = registry.camera_entities([tracker()], now_ns=1_000_000_000)[0]
    assert entity["registration_state"] == "identity_mismatch" and entity["position_m"] is None
    registry.observe_device_info("cam", NS(serial_number="123"), observed_ns=1_000_000_000)
    bad = dict(tracker(), tracker_id="LHR-OTHER")
    assert registry.camera_entities([bad], now_ns=1_000_000_000)[0]["registration_state"] == "tracker_mismatch"


def test_intrinsic_change_revises_geometry_but_duplicate_does_not():
    registry = prepared()
    before = registry.geometry("cam")["geometry_revision"]
    assert registry.observe_camera_info("cam", camera_info()) is False
    assert registry.geometry("cam")["geometry_revision"] == before
    registry.observe_camera_info("cam", camera_info(fx=120.))
    assert registry.geometry("cam")["geometry_revision"] > before
    assert len(registry.geometry("cam")["geometry_sha256"]) == 64


def test_rejected_calibration_and_frame_mismatch_fail_closed():
    registry = prepared()
    document = extrinsic()
    document["status"] = "REJECTED"
    with pytest.raises(ValueError, match="VALID"):
        registry.set_extrinsic("cam", document)
    with pytest.raises(ValueError, match="frame"):
        registry.observe_camera_info("cam", camera_info("other"))


def test_invalid_pose_and_missing_geometry_remain_explicit():
    registry = CameraGeometryRegistry([camera_config()])
    registry.observe_device_info("cam", NS(serial_number="123"), observed_ns=1_000_000_000)
    assert registry.camera_entities([tracker()], now_ns=1_000_000_000)[0]["registration_state"] == "missing_calibration"
    registry = prepared()
    registry.observe_device_info("cam", NS(serial_number="123"), observed_ns=1_000_000_000)
    assert registry.camera_entities([dict(tracker(), state="stale")], now_ns=1_000_000_000)[0]["registration_state"] == "tracker_unavailable"


def test_new_invalid_intrinsics_and_depth_geometry_remove_previous_geometry():
    registry=prepared()
    revision=registry.geometry('cam')['geometry_revision']
    with pytest.raises(ValueError): registry.observe_camera_info('cam',camera_info(fx=0.))
    assert registry.geometry('cam')['color_intrinsics'] is None
    assert registry.geometry('cam')['geometry_revision']>revision
    registry.observe_depth_to_color('cam',dict(translation_m=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.]))
    with pytest.raises(ValueError): registry.observe_depth_to_color('cam',dict(translation_m=[float('nan'),0.,0.],quaternion_xyzw=[0.,0.,0.,1.]))
    assert registry.geometry('cam')['depth_to_color'] is None


def test_camera_info_accepts_ros_numpy_scalar_arrays_and_rejects_boolean():
    import numpy as np
    registry=CameraGeometryRegistry([camera_config()])
    info=camera_info(); info.k=np.asarray(info.k); info.r=np.asarray(info.r); info.p=np.asarray(info.p)
    assert registry.observe_camera_info('cam',info)
    info.k=[True,0.,50.,0.,100.,40.,0.,0.,1.]
    with pytest.raises(ValueError): registry.observe_camera_info('cam',info)


def test_identity_revision_changes_on_reconnect_but_not_polling_same_device():
    registry=prepared()
    registry.observe_device_info('cam',NS(serial_number='123'),observed_ns=1)
    revision=registry.geometry('cam')['identity_revision']
    registry.observe_device_info('cam',NS(serial_number='123'),observed_ns=2)
    assert registry.geometry('cam')['identity_revision']==revision
    registry.invalidate_identity('cam','graph changed')
    assert registry.geometry('cam')['identity_revision']>revision

def test_raw_color_frustum_uses_undistorted_corner_rays():
    pytest.importorskip('cv2')
    registry=prepared()
    info=camera_info(); info.d=[.2,0.,0.,0.,0.]
    registry.observe_camera_info('cam',info)
    registry.observe_device_info('cam',NS(serial_number='123'),observed_ns=1_000_000_000)
    entity=registry.camera_entities([tracker()],now_ns=1_000_000_000)[0]
    assert entity['state']=='fresh'
    assert len(entity['frustum_vertices'])==8
    assert abs(entity['frustum_vertices'][0][0])<.025  # Positive radial distortion shrinks the inverse corner ray.
    assert entity['frustum_vertices'][0][2]==.05

def test_external_tracker_frame_binding_rejects_self_consistent_foreign_parent():
    registry=CameraGeometryRegistry([dict(camera_config(),tracker_frame_id='vive_tracker_torso')])
    document=extrinsic(); document['tracker']['frame_id']='foreign_parent'; document['transform']['parent_frame']='foreign_parent'
    with pytest.raises(ValueError,match='tracker.frame_id'): registry.set_extrinsic('cam',document)
    assert registry.geometry('cam')['extrinsic'] is None
