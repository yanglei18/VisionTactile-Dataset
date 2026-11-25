from dataclasses import replace
from math import cos, radians, sin, sqrt
import json
from types import SimpleNamespace as N

import pytest

from vt_capture_manager.optical.hand_fusion import FusionJoin, ShapeBuffer, color_time, fuse, read_mounts


CAMERA = "d436"
SIDE = "right"
LAYOUT = "a" * 64
TIME = 2_000_000_000
HOST = 10_000_000_000
MS = 1_000_000


def header(frame, t):
    sec, nanosec = divmod(t, 1_000_000_000)
    return N(frame_id=frame, stamp=N(sec=sec, nanosec=nanosec))


def mount_file(tmp_path):
    item = dict(layout_sha256=LAYOUT, cube_from_model_wrist=dict(
        translation_m=[.02, 0., .01], quaternion_xyzw=[0., 0., 0., 1.]))
    path = tmp_path / "mount.json"
    path.write_text(json.dumps(dict(schema_version=1, hands={SIDE: item})))
    return path


def cube(t=TIME, index=1):
    # Quarter turn about Z plus camera-frame translation.
    return N(schema_version=1, hand=SIDE, camera_name=CAMERA,
        header=header(CAMERA+"_color_optical_frame", t), valid=True, status="tracked",
        layout_sha256=LAYOUT, intrinsics_sha256="c"*64, instance_id="instance", source_image_index=index,
        pose=N(position=N(x=.2, y=.1, z=.6), orientation=N(x=0., y=0., z=sqrt(.5), w=sqrt(.5))))


def timing(t=TIME, host=HOST):
    return N(camera_name=CAMERA, serial_number="physical", header=header(CAMERA+"_frame_group", t),
             shared_ros_timestamp_ns=t, color_host_realtime_ns=host, color_frame_number=42,
             group_validity_flags=27, color_validity_flags=65, GROUP_VALID_COMMON_STAMP=1,
             GROUP_VALID_IDENTITY=2, GROUP_VALID_CALLBACK_CLOCKS=8, GROUP_VALID_UNIQUE=16,
             VALID_HOST_REALTIME=64, VALID_FRAME_NUMBER=1)


def glove(host=HOST, frame=20):
    # Shape has already been rotated 90 degrees about X by the old IMU root.
    # The local point (0,.08,0) is now observed as (0,0,.08).
    return N(schema_version=1, hand=SIDE, source_mode="live", header=header("glove_right_reference", host),
        pose_valid=True, time_valid=True, session_id="right-1", calibration_version=2,
        config_sha256="d"*64, model_rest_sha256="e"*64, frame_index=frame, host_realtime_ns=host,
        host_monotonic_ns=host+500, timing_source="host_receive", clock_domain="CLOCK_MONOTONIC",
        root_quat_xyzw=[sqrt(.5), 0., 0., sqrt(.5)], joint_positions_m=[0.,0.,0.] + [0., 0., .08]*20)


def test_known_root_removed_before_optical_pose_and_nearest_true_shape_retains_identity(tmp_path):
    mounts = read_mounts(mount_file(tmp_path))
    q = FusionJoin(CAMERA, mounts)
    q.add_cube(SIDE, cube(), 10*MS)
    # Timing and hand samples arrive after image, in different orders.
    q.shapes[SIDE].add(glove(HOST+4*MS, 8), SIDE)
    q.add_timing(timing(), 17*MS)
    q.shapes[SIDE].add(glove(HOST-8*MS, 7), SIDE)
    assert q.drain(30*MS) == []
    outcome, = q.drain(75*MS)
    assert outcome.status == "tracked"
    assert outcome.wrist.translation_m == pytest.approx([.2, .12, .61])
    assert outcome.joints[:3] == pytest.approx(outcome.wrist.translation_m)
    assert outcome.joints[-3:] == pytest.approx([.12, .12, .61])
    assert outcome.shape.frame == 8 and outcome.shape.realtime == HOST+4*MS
    assert (outcome.before, outcome.after) == (HOST-8*MS, HOST+4*MS)
    assert not q.drain(100*MS)


@pytest.mark.parametrize("hand,thumb_side", [("left", 1), ("right", -1)])
@pytest.mark.parametrize("pitch", [135, -135], ids=["fingers-away", "fingers-toward-lens"])
def test_camera_view_preserves_hand_side_and_finger_depth(tmp_path, hand, thumb_side, pitch):
    # Synthetic wrist model: +Y points along open fingers, +Z is dorsal.
    # The two thumb tips lie toward the centre of the image at the starting pose.
    local = [(0., 0., 0.)]
    for offset, length in [(0.045, .060), (.020, .092), (0., .100), (-.018, .086), (-.034, .065)]:
        for step in range(1, 5):
            local.append((thumb_side * offset, length * step / 4, 0.))
    # Flex the index fingertip independently of the hand's overall forward axis.
    x, y, z = local[7]
    local[8] = (x, y + .004, z - .012)
    shape = glove()
    shape.hand = hand
    shape.session_id = f"{hand}-1"
    shape.header = header(f"glove_{hand}_reference", HOST)
    # The input glove root has already rotated these points +90 degrees about X.
    shape.joint_positions_m = [v for x, y, z in local for v in (x, -z, y)]
    history = ShapeBuffer()
    history.add(shape, hand)

    # The mount turns the wrist 180 degrees about Z relative to the cube.
    path = tmp_path / "mount.json"
    path.write_text(json.dumps({"schema_version": 1, "hands": {hand: {
        "layout_sha256": LAYOUT, "cube_from_model_wrist": {
            "translation_m": [0., 0., .01], "quaternion_xyzw": [0., 0., 1., 0.]}}}}))
    theta = radians(pitch)
    obs = cube()
    obs.hand = hand
    x_center = -.12 * thumb_side
    obs.pose.position = N(x=x_center, y=0., z=.55)
    # Camera-from-cube = X pitch followed in local coordinates by a half turn about Z.
    obs.pose.orientation = N(x=0., y=-sin(theta/2), z=cos(theta/2), w=0.)
    out = fuse(obs, hand, CAMERA, read_mounts(path), color_time(timing(), CAMERA), history)
    assert out.status == "tracked"
    wrist = (x_center, -.01 * sin(theta), .55 + .01 * cos(theta))
    # Camera optical +Z is away from the lens. Check every labelled joint, including
    # the thumb side and distal segments, against an independent camera-frame reference.
    expected = [(wrist[0] + x, wrist[1] + y*cos(theta) - z*sin(theta),
                 wrist[2] + y*sin(theta) + z*cos(theta)) for x, y, z in local]
    assert out.joints == pytest.approx([v for point in expected for v in point])
    assert out.wrist.apply((0., 0., 1.)) == pytest.approx(
        (wrist[0], wrist[1]-sin(theta), wrist[2]+cos(theta)))


def test_missing_mount_only_affects_its_side_unavailable_timing_never_becomes_ros_time(tmp_path):
    mounts = read_mounts(mount_file(tmp_path))
    obs = cube()
    valid_time = color_time(timing(), CAMERA)
    h = ShapeBuffer()
    for bad, code in [({}, "mount_missing"), ({"left":mounts[SIDE]}, "mount_missing"),
                      ({SIDE: replace(mounts[SIDE], layout_sha256="f"*64)}, "layout_mismatch")]:
        r = fuse(obs, SIDE, CAMERA, bad, valid_time, h)
        assert r.status == code and r.wrist is None and r.joints == ()
    q = FusionJoin(CAMERA, mounts)
    q.add_timing(timing(t=TIME+1), 1)
    q.shapes[SIDE].add(glove(TIME), SIDE)
    q.add_cube(SIDE, obs, 1)
    r, = q.drain(100*MS)
    assert r.status == "camera_timing_unavailable" and r.wrist is None and not r.joints
    obs.valid, obs.status = False, "lost"
    r = fuse(obs, SIDE, CAMERA, mounts, valid_time, h)
    assert r.status == "cube_lost" and r.wrist is None and not r.joints


def test_mount_reload_keeps_binding_and_rejects_another_glove_model(tmp_path):
    mounts = read_mounts(mount_file(tmp_path))
    join = FusionJoin(CAMERA, {})
    join.add_timing(timing(), 2*MS)
    join.shapes[SIDE].add(glove(), SIDE)
    join.add_cube(SIDE, cube(), 3*MS)
    join.replace_mounts(mounts)
    result, = join.drain(70*MS)
    assert result.status == "mount_changed"
    join.add_cube(SIDE, cube(index=2), 71*MS)
    result, = join.drain(140*MS)
    assert result.status == "tracked" and result.joints
    join.replace_mounts({SIDE: replace(mounts[SIDE], model_rest_sha256="0"*64)})
    join.add_cube(SIDE, cube(index=3), 145*MS)
    result, = join.drain(211*MS)
    assert result.status == "mount_model_mismatch" and result.wrist is None and not result.joints


@pytest.mark.parametrize("key,val,expected", [
    ("session_id", "next", "glove_source_changed"),
    ("config_sha256", "2"*64, "glove_source_changed"),
    ("model_rest_sha256", "4"*64, "glove_source_changed"),
    ("calibration_version", 3, "glove_source_changed"),
    ("frame_index", 4, "glove_clock_discontinuity"),
    ("host_monotonic_ns", 400, "glove_clock_discontinuity"),
    ("clock_domain", "CLOCK_MONOTONIC_RAW", "invalid_glove"),
    ("source_mode", "replay", "invalid_glove"),
    ("pose_valid", False, "invalid_glove"),
    ("joint_positions_m", [1., 2., 3.]*21, "invalid_glove"),
])
def test_incompatible_or_unhealthy_glove_cannot_supply_joint_measurements(tmp_path, key, val, expected):
    h = ShapeBuffer()
    first, last = glove(HOST-5*MS, 5), glove(HOST+5*MS, 6)
    setattr(last, key, val)
    h.add(first, SIDE)
    h.add(last, SIDE)
    r = fuse(cube(), SIDE, CAMERA, read_mounts(mount_file(tmp_path)), color_time(timing(), CAMERA), h)
    assert r.status == expected and r.wrist is not None and r.shape is None and not r.joints


def test_no_shape_extrapolation_or_interpolation_across_gap_and_conflicting_duplicate():
    h = ShapeBuffer()
    h.add(glove(HOST+80*MS, 2), SIDE)
    h.add(glove(HOST, 1), SIDE)
    assert h.nearest_in_bracket(HOST+20*MS)[0] == "glove_gap"
    assert h.nearest_in_bracket(HOST-1)[0] == "glove_not_bracketed"
    assert h.nearest_in_bracket(HOST+80*MS)[0] == "tracked"
    h.add(glove(HOST, 9), SIDE)
    assert h.nearest_in_bracket(HOST)[0] == "invalid_glove"


def test_duplicate_timing_different_serial_and_missing_validity_flags():
    for field, bad in (("serial_number", ""), ("camera_name", "other"), ("shared_ros_timestamp_ns", TIME+1),
                       ("color_host_realtime_ns", 0), ("color_validity_flags", 1), ("group_validity_flags", 1)):
        m = timing()
        setattr(m, field, bad)
        assert color_time(m, CAMERA) is None
    assert color_time(timing(), CAMERA, "unexpected") is None
    join = FusionJoin(CAMERA, {})
    join.add_timing(timing(), 100)
    m = timing()
    m.serial_number = "second"
    join.add_timing(m, 200)
    assert join.timings[TIME][0] is None


def test_pending_queue_is_bounded_and_late_packets_cannot_revive_a_retired_cube():
    q = FusionJoin(CAMERA, {})
    overflows = []
    for n in range(q.MAX_PENDING + 2):
        overflows += q.add_cube(SIDE, cube(TIME+n, n+1), n)
    assert len(q.pending[SIDE]) == q.MAX_PENDING
    assert [r.status for r in overflows] == 2*["fusion_queue_overflow"]
    rep = cube(TIME+20)
    rep.instance_id = "next"
    displaced = q.add_cube(SIDE, rep, 20)
    assert len(displaced) == q.MAX_PENDING and all(r.status == "cube_source_changed" for r in displaced)
    assert not q.add_cube(SIDE, cube(TIME+21, 100), 21)
    assert [r.cube.instance_id for r in q.drain(100*MS)] == ["next"]


def test_mount_semantics_reject_unknown_or_malformed_measurements(tmp_path):
    path = mount_file(tmp_path)
    doc = json.loads(path.read_text())
    four = {"layout_sha256": LAYOUT, "cube_from_model_wrist":{"translation_m":[0,0,0],"quaternion_xyzw":[0,0,0,0]}}
    for bad in [dict(schema_version=True, hands=doc["hands"]),dict(schema_version=1,hands={"unassigned":doc["hands"][SIDE]}),
                dict(schema_version=1,hands={SIDE:{"layout_sha256":"bad"}}),dict(schema_version=1,hands={SIDE:four})]:
        path.write_text(json.dumps(bad))
        with pytest.raises(ValueError):
            read_mounts(path)


def test_slightly_out_of_order_camera_timings_ok_counter_reset_invalidates_pending(tmp_path):
    q = FusionJoin(CAMERA, read_mounts(mount_file(tmp_path)))
    m = timing(TIME+33*MS, HOST+33*MS)
    m.color_frame_number = 43
    q.add_timing(m, 10*MS)
    q.add_timing(timing(), 15*MS)
    assert len(q.timings) == 2
    q.shapes[SIDE].add(glove(), SIDE, 10*MS)
    q.add_cube(SIDE, cube(), 5*MS)
    nxt = timing(TIME+66*MS, HOST+66*MS)
    nxt.color_frame_number = 1
    q.add_timing(nxt, 40*MS)
    r, = q.drain(80*MS)
    assert r.status == "camera_clock_discontinuity" and r.wrist is None
    assert list(q.timings) == [TIME+66*MS] and not q.shapes[SIDE].times


def test_stale_buffered_glove_cannot_become_current_after_clock_reuse(tmp_path):
    q = FusionJoin(CAMERA, read_mounts(mount_file(tmp_path)))
    q.shapes[SIDE].add(glove(), SIDE, 1*MS)
    q.add_cube(SIDE, cube(), 900*MS)
    q.add_timing(timing(), 900*MS)
    r, = q.drain(990*MS)
    assert r.status == "glove_arrival_expired" and r.wrist is not None and not r.joints
