"""Recorded factory calibration is inspected independently of world/Tracker poses."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from vt_multisensor_alignment import capture_quality as quality


INFO_TYPE = "sensor_msgs/msg/CameraInfo"
EXTRINSICS_TYPE = "realsense2_camera_msgs/msg/Extrinsics"
IDENTITY = [1., 0., 0., 0., 1., 0., 0., 0., 1.]


def camera_info(**changes):
    values = dict(header=NS(frame_id="camera_color_optical_frame"), width=1280, height=720,
                  distortion_model="plumb_bob", d=[0.] * 5,
                  k=[600., 0., 640., 0., 600., 360., 0., 0., 1.], r=IDENTITY[:],
                  p=[600., 0., 640., 0., 0., 600., 360., 0., 0., 0., 1., 0.],
                  binning_x=0, binning_y=0,
                  roi=NS(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
    values.update(changes)
    return NS(**values)


def accumulator(type_name=INFO_TYPE):
    from vt_multisensor_alignment.camera_calibration import CalibrationQualityAccumulator
    return CalibrationQualityAccumulator(type_name)


def test_intrinsics_preserve_first_valid_sample_without_source_header_noise():
    subject = accumulator()
    first = camera_info()
    subject.add(first, 1)
    later = deepcopy(first)
    later.header.stamp = NS(sec=123, nanosec=1)
    subject.add(later, 2)
    later.k[0] = 601.
    subject.add(later, 3)
    result = subject.document()
    assert result["inspection"] == "calibration_messages"
    assert result["decoded_messages"] == result["valid_calibration_messages"] == 3
    assert result["invalid_calibration_messages"] == 0
    assert result["calibration_change_events"] == 1
    assert result["first_valid_calibration"]["k"][0] == 600.
    assert result["first_valid_calibration"]["frame_id"] == "camera_color_optical_frame"
    assert result["first_valid_calibration"]["projection_supported"] is True


@pytest.mark.parametrize("changes", [
    {"width": 0}, {"height": -1}, {"header": NS(frame_id="")},
    {"k": [0.] * 9}, {"p": [0.] * 12}, {"r": [float("nan")] * 9},
    {"d": [float("inf")] * 5}, {"d": [0.] * 4}, {"k": [1.] * 8},
])
def test_invalid_intrinsics_do_not_claim_usable_calibration(changes):
    subject = accumulator()
    subject.add(camera_info(**changes), 1)
    result = subject.document()
    assert result["invalid_calibration_messages"] == 1
    assert result["valid_calibration_messages"] == 0
    assert result["first_valid_calibration"] is None
    assert result["first_invalid_reason"]


@pytest.mark.parametrize("field, row", [
    ("k", [0., 0., 0.]), ("k", [0., .1, 1.]),
    ("p", [0., 0., 0., 0.]), ("p", [0., 0., 1., .1]),
])
def test_intrinsics_reject_nonstandard_homogeneous_rows_with_positive_focal_lengths(field, row):
    message = camera_info()
    getattr(message, field)[-len(row):] = row
    subject = accumulator()
    subject.add(message, 1)
    result = subject.document()
    assert result["invalid_calibration_messages"] == 1
    assert result["valid_calibration_messages"] == 0
    assert result["first_valid_calibration"] is None
    assert "homogeneous row" in result["first_invalid_reason"]


def test_intrinsics_accept_roundoff_in_homogeneous_rows():
    message = camera_info()
    message.k[-3:] = [1e-10, -1e-10, 1. + 1e-10]
    message.p[-4:] = [1e-10, -1e-10, 1. - 1e-10, 1e-10]
    subject = accumulator()
    subject.add(message, 1)
    assert subject.document()["valid_calibration_messages"] == 1


def test_unknown_distortion_is_preserved_and_explicitly_unsupported():
    subject = accumulator()
    subject.add(camera_info(distortion_model="vendor_model", d=[.1, .2]), 1)
    result = subject.document()
    assert result["unsupported_distortion_messages"] == 1
    assert result["first_valid_calibration"]["distortion_model"] == "vendor_model"
    assert result["first_valid_calibration"]["d"] == [.1, .2]
    assert result["first_valid_calibration"]["projection_supported"] is False


def test_factory_extrinsics_accept_identity_zero_baseline_and_preserve_column_major():
    subject = accumulator(EXTRINSICS_TYPE)
    subject.add(NS(rotation=IDENTITY, translation=[0.] * 3), 1)
    # +90 degrees about Z, in SDK / ROS Extrinsics column-major layout.
    rotation = [0., 1., 0., -1., 0., 0., 0., 0., 1.]
    subject.add(NS(rotation=rotation, translation=[.01, 0., 0.]), 2)
    result = subject.document()
    assert result["valid_calibration_messages"] == 2
    assert result["calibration_change_events"] == 1
    assert result["first_valid_calibration"]["rotation_layout"] == "column_major"
    assert result["first_valid_calibration"]["translation_m"] == [0.] * 3


@pytest.mark.parametrize("rotation, translation", [
    ([-1., 0., 0., 0., 1., 0., 0., 0., 1.], [0.] * 3),
    ([2., 0., 0., 0., 1., 0., 0., 0., 1.], [0.] * 3),
    (IDENTITY, [float("inf"), 0., 0.]), (IDENTITY[:8], [0.] * 3),
])
def test_factory_extrinsics_reject_reflections_nonrotations_and_nonfinite(rotation, translation):
    subject = accumulator(EXTRINSICS_TYPE)
    subject.add(NS(rotation=rotation, translation=translation), 1)
    assert subject.document()["invalid_calibration_messages"] == 1


def test_real_mcap_inspects_calibration_payloads_and_keeps_optional_absence_as_coverage(tmp_path):
    rosbag2 = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo
    from realsense2_camera_msgs.msg import Extrinsics

    bag = tmp_path / "bag"
    writer = rosbag2.SequentialWriter()
    writer.open(rosbag2.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2.ConverterOptions("", ""))
    types = {"/color_info": INFO_TYPE, "/depth_info": INFO_TYPE, "/extrinsics": EXTRINSICS_TYPE,
             "/absent_optional_info": INFO_TYPE, "/unknown": "std_msgs/msg/String"}
    for index, (topic, type_name) in enumerate(types.items(), 1):
        writer.create_topic(rosbag2.TopicMetadata(id=index, name=topic, type=type_name, serialization_format="cdr"))
    values = camera_info()
    info = CameraInfo(width=values.width, height=values.height, distortion_model=values.distortion_model,
                      d=values.d, k=values.k, r=values.r, p=values.p)
    info.header.frame_id = values.header.frame_id
    for index in range(2):
        info.header.stamp.sec = index + 1
        writer.write("/color_info", serialize_message(info), (index + 1) * 1_000_000_000)
    # Positive focal lengths do not make a nonstandard homogeneous row valid.
    info.k = [*values.k[:8], 0.]
    writer.write("/depth_info", serialize_message(info), 1_000_000_000)
    writer.write("/extrinsics", serialize_message(Extrinsics(rotation=IDENTITY, translation=[0.] * 3)), 1_000_000_000)
    writer.write("/extrinsics", b"malformed", 1_000_000_001)
    writer.close()
    before = {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in bag.iterdir()}
    report = quality.generate_capture_quality(
        bag, tmp_path / "report.json", optional_topics=("/absent_optional_info", "/unknown"))
    assert report["topics"]["/color_info"]["valid_calibration_messages"] == 2
    assert report["topics"]["/color_info"]["calibration_change_events"] == 0
    assert report["topics"]["/depth_info"]["invalid_calibration_messages"] == 1
    assert report["topics"]["/extrinsics"]["malformed_messages"] == 1
    assert report["topics"]["/extrinsics"]["first_valid_calibration"]["translation_m"] == [0.] * 3
    assert report["topics"]["/absent_optional_info"]["inspection"] == "metadata_only"
    assert report["summary"]["inspected_topic_count"] == 3
    assert {(w["code"], w.get("topic")) for w in report["warnings"]} == {
        ("invalid_camera_calibration", "/depth_info"), ("malformed_messages", "/extrinsics")}
    assert before == {p.name: (p.stat().st_size, p.stat().st_mtime_ns) for p in bag.iterdir()}
