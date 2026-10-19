"""Quality subprocess tests: fake processes and metadata-only bags; no recorder."""

import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from vt_capture_manager import quality_runner as module
from vt_capture_manager.quality_runner import QualityCancelled, QualityRunError, QualityRunner


@pytest.fixture
def bag(tmp_path):
    directory = tmp_path / "closed bag ; inert shell text"
    directory.mkdir()
    # This is not an MCAP writer/Recorder; only the supported metadata-only path
    # is exercised. Sensor payloads are explicitly not claimed as inspected.
    metadata = dict(rosbag2_bagfile_information=dict(
        storage_identifier="mcap", duration=dict(nanoseconds=1_000_000_000),
        starting_time=dict(nanoseconds_since_epoch=1_000_000_000), message_count=0,
        topics_with_message_count=[dict(topic_metadata=dict(name="/camera/image", type="unknown/msg/Custom"), message_count=0)],
        relative_file_paths=[],
    ))
    (directory / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    return directory


def fake_report(argv):
    def value(flag):
        return argv[argv.index(flag) + 1]
    expected = json.loads(value("--expected-topics"))
    optional = json.loads(value("--optional-topics")) if "--optional-topics" in argv else []
    advisories = json.loads(value("--advisories"))
    image_rates = json.loads(value("--expected-image-rates-hz"))
    warnings = [dict(code="expected_topic_missing", detail="fake missing observation", topic=topic) for topic in expected]
    warnings.extend(advisories)
    return dict(schema_version=1, report_kind="post_record_capture_quality", generated_at_utc="2026-10-08T00:00:00+00:00",
        status="warnings" if warnings else "ok",
        bag=dict(path=value("--bag"), duration_ns=0, start_realtime_ns=0, metadata_message_count=0),
        summary=dict(topic_count=0, inspected_topic_count=0, metadata_only_topic_count=0, warning_count=len(warnings)),
        topics={}, warnings=warnings, limitations=["test observations"],
        topic_coverage=dict(optional_topics=optional, absent_optional_topics=optional), runner_provenance=dict(
            invocation_id=value("--invocation-id"), input_fingerprint=value("--input-fingerprint"),
            expected_topics=expected, optional_topics=optional, advisories=advisories,
            expected_image_rates_hz=image_rates))


class FakeProcess:
    def __init__(self, *, returncode=0, logs=b"", kill_required=False):
        self.returncode = returncode
        self.terminated = self.killed = self.waited = False
        self.kill_required = kill_required
        read_fd, write_fd = os.pipe()
        self.stdout = os.fdopen(read_fd, "rb", buffering=0)
        if logs:
            os.write(write_fd, logs)
        os.close(write_fd)

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        if not self.kill_required:
            self.returncode = -15

    def kill(self):
        self.killed = True
        self.returncode = -9

    def wait(self, timeout):
        self.waited = True
        if self.returncode is None:
            raise subprocess.TimeoutExpired("owned-quality-child", timeout)
        return self.returncode


def install_fake(monkeypatch, *, mutate=None, returncode=0, logs=b"", process=None):
    calls = []
    def spawn(argv, **kwargs):
        assert type(argv) is list and argv[1:3] == ["-m", "vt_capture_manager.quality_runner"]
        assert kwargs["shell"] is False and kwargs["start_new_session"] is True
        assert kwargs["stdin"] == subprocess.DEVNULL
        report = fake_report(argv)
        if mutate is not None:
            mutate(report, argv)
        target = Path(argv[argv.index("--output") + 1])
        target.write_text(json.dumps(report))
        child = process or FakeProcess(returncode=returncode, logs=logs)
        calls.append((argv, kwargs, child))
        return child
    monkeypatch.setattr(module.subprocess, "Popen", spawn)
    return calls


def test_fixed_argv_no_shell_identity_and_new_outside_report(bag, tmp_path, monkeypatch):
    calls = install_fake(monkeypatch)
    output = tmp_path / "reports/result.json"
    result = QualityRunner().run(bag, output, expected_topics=["/left/state", "/right/state"])
    assert result == json.loads(output.read_text())
    assert result["status"] == "warnings"
    assert str(bag) in calls[0][0]  # Shell metacharacters are one inert argv entry.
    assert not calls[0][2].terminated
    assert not list(output.parent.glob(".quality-*"))
    assert sorted(path.name for path in bag.iterdir()) == ["metadata.yaml"]


def test_expected_image_rates_are_bound_to_child_and_published_provenance(bag, tmp_path, monkeypatch):
    calls = install_fake(monkeypatch)
    expected = {"/camera/image": 30.}
    result = QualityRunner().run(bag, tmp_path / "rates.json", expected_topics=["/camera/image"],
                                 expected_image_rates_hz=expected)
    argv = calls[0][0]
    assert json.loads(argv[argv.index("--expected-image-rates-hz") + 1]) == expected
    assert result["runner_provenance"]["expected_image_rates_hz"] == expected


def test_optional_topics_and_image_rates_are_bound_to_child_and_provenance(bag, tmp_path, monkeypatch):
    calls = install_fake(monkeypatch)
    result = QualityRunner().run(bag, tmp_path / "optional.json", expected_topics=[],
                                 optional_topics=("/tracker", "/camera/image"),
                                 expected_image_rates_hz={"/camera/image": 30.})
    expected = ["/camera/image", "/tracker"]
    argv = calls[0][0]
    assert json.loads(argv[argv.index("--optional-topics") + 1]) == expected
    assert result["runner_provenance"]["optional_topics"] == expected
    assert result["topic_coverage"]["absent_optional_topics"] == expected
    assert result["status"] == "ok"


@pytest.mark.parametrize("optional", ["/tracker", {"/tracker"}, ["relative"], [True],
                                      ["/tracker", "/tracker"], ["/required"],
                                      [f"/topic_{i}" for i in range(257)]])
def test_invalid_or_overlapping_optional_topics_do_not_launch(bag, tmp_path, monkeypatch, optional):
    calls = install_fake(monkeypatch)
    with pytest.raises(QualityRunError, match="optional_topics"):
        QualityRunner().run(bag, tmp_path / "optional.json", expected_topics=["/required"],
                            optional_topics=optional)
    assert not calls


@pytest.mark.parametrize("change", ["missing_provenance", "changed_provenance", "missing_coverage", "wrong_coverage"])
def test_optional_report_contract_mismatch_is_not_published(bag, tmp_path, monkeypatch, change):
    def mutate(report, argv):
        if change == "missing_provenance":
            report["runner_provenance"].pop("optional_topics")
        elif change == "changed_provenance":
            report["runner_provenance"]["optional_topics"] = []
        elif change == "missing_coverage":
            report.pop("topic_coverage")
        else:
            report["topic_coverage"]["absent_optional_topics"] = []
    install_fake(monkeypatch, mutate=mutate)
    output = tmp_path / "mismatched-optional.json"
    with pytest.raises(QualityRunError, match="optional|coverage"):
        QualityRunner().run(bag, output, expected_topics=[], optional_topics=["/tracker"])
    assert not output.exists()


def test_real_child_distinguishes_optional_absence_from_required_missing(bag, tmp_path):
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    info = metadata["rosbag2_bagfile_information"]
    info["message_count"] = 1
    info["topics_with_message_count"][0]["topic_metadata"]["type"] = "sensor_msgs/msg/Image"
    info["topics_with_message_count"].append(dict(
        topic_metadata=dict(name="/actual", type="unknown/msg/Custom"), message_count=1))
    metadata_path.write_text(yaml.safe_dump(metadata))
    report = QualityRunner(timeout_s=10).run(
        bag, tmp_path / "optional-child.json", expected_topics=["/actual"],
        optional_topics=["/tracker", "/camera/image"], expected_image_rates_hz={"/camera/image": 30.})
    assert report["status"] == "ok"
    assert report["warnings"] == []
    assert report["topics"]["/camera/image"]["rate_assessment"] == "insufficient_samples"
    assert report["topic_coverage"]["absent_optional_topics"] == ["/camera/image", "/tracker"]


@pytest.mark.parametrize("rates", [[], {"relative": 30}, {"/camera/image": True},
    {"/camera/image": 0}, {"/camera/image": -1}, {"/camera/image": float("inf")},
    {"/camera/image": float("nan")}, {"/camera/image": "30"}, {"/not_expected": 30},
    {"/camera/image": 10001}])
def test_invalid_image_rate_contract_never_launches(bag, tmp_path, monkeypatch, rates):
    calls = install_fake(monkeypatch)
    with pytest.raises(QualityRunError, match="expected_image_rates_hz"):
        QualityRunner().run(bag, tmp_path / "rates.json", expected_topics=["/camera/image"],
                            expected_image_rates_hz=rates)
    assert not calls


@pytest.mark.parametrize("change", ["missing", "changed"])
def test_image_rate_provenance_mismatch_is_not_published(bag, tmp_path, monkeypatch, change):
    def mutate(report, argv):
        if change == "missing":
            report["runner_provenance"].pop("expected_image_rates_hz")
        else:
            report["runner_provenance"]["expected_image_rates_hz"] = {"/camera/image": 15.}
    install_fake(monkeypatch, mutate=mutate)
    output = tmp_path / "rates.json"
    with pytest.raises(QualityRunError, match="identity mismatch"):
        QualityRunner().run(bag, output, expected_topics=["/camera/image"],
                            expected_image_rates_hz={"/camera/image": 30.})
    assert not output.exists()


@pytest.mark.parametrize("field", ["provenance", "topic"])
def test_boolean_rate_cannot_impersonate_one_hz_contract(bag, tmp_path, monkeypatch, field):
    def mutate(report, argv):
        report["topics"] = {"/camera/image": dict(type="sensor_msgs/msg/Image", recorded_message_count=2,
            inspection="image_headers", expected_rate_hz=True if field == "topic" else 1.,
            rate_assessment="within_tolerance")}
        report["summary"].update(topic_count=1, inspected_topic_count=1, metadata_only_topic_count=0, warning_count=0)
        report.update(warnings=[], status="ok")
        if field == "provenance":
            report["runner_provenance"]["expected_image_rates_hz"] = {"/camera/image": True}
    install_fake(monkeypatch, mutate=mutate)
    with pytest.raises(QualityRunError):
        QualityRunner().run(bag, tmp_path / "boolean-rate.json", expected_topics=["/camera/image"],
                            expected_image_rates_hz={"/camera/image": 1.})


@pytest.mark.parametrize("rates", [{}, {"/camera/image": 30.}])
def test_image_headers_count_as_inspected_with_explicit_rate_basis(bag, tmp_path, monkeypatch, rates):
    def mutate(report, argv):
        report["topics"] = {"/camera/image": dict(type="sensor_msgs/msg/Image", recorded_message_count=31,
            inspection="image_headers", expected_rate_hz=rates.get("/camera/image"),
            rate_assessment="within_tolerance" if rates else "not_configured")}
        report["summary"].update(topic_count=1, inspected_topic_count=1, metadata_only_topic_count=0,
                                 warning_count=0)
        report.update(warnings=[], status="ok")
    install_fake(monkeypatch, mutate=mutate)
    result = QualityRunner().run(bag, tmp_path / "headers.json", expected_topics=["/camera/image"],
                                 expected_image_rates_hz=rates)
    assert result["summary"]["inspected_topic_count"] == 1


def _calibration_report_entry():
    return dict(type="realsense2_camera_msgs/msg/Extrinsics", recorded_message_count=1,
                inspection="calibration_messages", calibration_kind="extrinsics",
                observed_messages=1, decoded_messages=1, malformed_messages=0,
                valid_calibration_messages=1, invalid_calibration_messages=0,
                unsupported_distortion_messages=0, calibration_change_events=0,
                first_decode_error=None, first_invalid_reason=None,
                first_valid_calibration=dict(rotation=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                                             rotation_layout="column_major", translation_m=[0., 0., 0.]))


def test_calibration_payloads_count_as_inspected(bag, tmp_path, monkeypatch):
    def mutate(report, argv):
        report["topics"] = {"/calibration": _calibration_report_entry()}
        report["summary"].update(topic_count=1, inspected_topic_count=1, metadata_only_topic_count=0, warning_count=0)
        report.update(warnings=[], status="ok")
    install_fake(monkeypatch, mutate=mutate)
    result = QualityRunner().run(bag, tmp_path / "calibration.json", expected_topics=["/calibration"])
    assert result["summary"]["inspected_topic_count"] == 1


@pytest.mark.parametrize("change", ["wrong_type", "wrong_kind", "metadata_bypass", "bad_count",
                                      "missing_sample", "missing_invalid_warning", "missing_change_warning"])
def test_calibration_report_cannot_hide_invalid_or_uninspected_parameters(bag, tmp_path, monkeypatch, change):
    def mutate(report, argv):
        entry = _calibration_report_entry()
        report["topics"] = {"/calibration": entry}
        report["summary"].update(topic_count=1, inspected_topic_count=1, metadata_only_topic_count=0, warning_count=0)
        report.update(warnings=[], status="ok")
        if change == "wrong_type":
            entry["type"] = "unknown/msg/Custom"
        elif change == "wrong_kind":
            entry["calibration_kind"] = "intrinsics"
        elif change == "metadata_bypass":
            entry["inspection"] = "metadata_only"
            report["summary"].update(inspected_topic_count=0, metadata_only_topic_count=1)
        elif change == "bad_count":
            entry["invalid_calibration_messages"] = True
        elif change == "missing_sample":
            entry["first_valid_calibration"] = None
        elif change == "missing_invalid_warning":
            entry.update(valid_calibration_messages=0, invalid_calibration_messages=1,
                         first_valid_calibration=None, first_invalid_reason="invalid rotation")
        else:
            entry.update(recorded_message_count=2, observed_messages=2, decoded_messages=2,
                         valid_calibration_messages=2, calibration_change_events=1)
    install_fake(monkeypatch, mutate=mutate)
    output = tmp_path / "invalid-calibration.json"
    with pytest.raises(QualityRunError, match="calibration|Calibration"):
        QualityRunner().run(bag, output, expected_topics=["/calibration"])
    assert not output.exists()


@pytest.mark.parametrize("contract, has_images, missing", [
    ("complete", True, {"/cam/depth/camera_info", "/cam/extrinsics/depth_to_color"}),
    ("complete", False, set()), ("legacy", True, set()),
])
def test_optional_calibration_becomes_needed_only_for_a_recorded_camera_bundle(contract, has_images, missing):
    candidate = ["/cam/color/image_raw", "/cam/depth/image_rect_raw", "/cam/color/camera_info"]
    if contract == "complete":
        candidate += ["/cam/depth/camera_info", "/cam/extrinsics/depth_to_color"]
    entries = {"/cam/color/image_raw": dict(type="sensor_msgs/msg/Image", recorded_message_count=int(has_images)),
               "/cam/color/camera_info": dict(type="sensor_msgs/msg/CameraInfo", recorded_message_count=1)}
    warnings = module._camera_calibration_observations(entries, [], candidate)
    assert {item["topic"] for item in warnings if item["code"] == "camera_calibration_missing"} == missing


def test_recorded_late_camera_cannot_hide_missing_calibration_warning(bag, tmp_path, monkeypatch):
    optional = ["/cam/color/image_raw", "/cam/depth/image_rect_raw", "/cam/color/camera_info",
                "/cam/depth/camera_info", "/cam/extrinsics/depth_to_color"]
    def mutate(report, argv):
        report["topics"] = {"/cam/color/image_raw": dict(type="sensor_msgs/msg/Image", recorded_message_count=1,
            inspection="image_headers", expected_rate_hz=None, rate_assessment="not_configured")}
        report["summary"].update(topic_count=1, inspected_topic_count=1, metadata_only_topic_count=0)
        report["topic_coverage"]["absent_optional_topics"] = [
            topic for topic in report["topic_coverage"]["optional_topics"] if topic != "/cam/color/image_raw"]
    install_fake(monkeypatch, mutate=mutate)
    with pytest.raises(QualityRunError, match="calibration coverage"):
        QualityRunner().run(bag, tmp_path / "hidden-calibration.json", expected_topics=[], optional_topics=optional)


def test_real_child_validates_late_camera_calibration_content_and_missing_optional_bundle(tmp_path):
    rosbag2 = pytest.importorskip("rosbag2_py")
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image

    directory = tmp_path / "real-camera"
    writer = rosbag2.SequentialWriter()
    writer.open(rosbag2.StorageOptions(uri=str(directory), storage_id="mcap"), rosbag2.ConverterOptions("", ""))
    for index, (topic, type_name) in enumerate([
            ("/cam/color/image_raw", "sensor_msgs/msg/Image"),
            ("/cam/color/camera_info", "sensor_msgs/msg/CameraInfo")], 1):
        writer.create_topic(rosbag2.TopicMetadata(id=index, name=topic, type=type_name, serialization_format="cdr"))
    image = Image(width=1, height=1, encoding="mono8", step=1, data=[42])
    image.header.stamp.sec = 1
    writer.write("/cam/color/image_raw", serialize_message(image), 1_000_000_000)
    # Default CameraInfo is uncalibrated, despite having a nonzero topic count.
    writer.write("/cam/color/camera_info", serialize_message(CameraInfo()), 1_000_000_000)
    writer.close()
    optional = ["/cam/color/image_raw", "/cam/depth/image_rect_raw", "/cam/color/camera_info",
                "/cam/depth/camera_info", "/cam/extrinsics/depth_to_color"]
    result = QualityRunner(timeout_s=10).run(directory, tmp_path / "quality.json", expected_topics=[],
                                           optional_topics=optional)
    assert result["topics"]["/cam/color/camera_info"]["invalid_calibration_messages"] == 1
    assert {(item["code"], item.get("topic")) for item in result["warnings"]} == {
        ("invalid_camera_calibration", "/cam/color/camera_info"),
        ("camera_calibration_missing", "/cam/depth/camera_info"),
        ("camera_calibration_missing", "/cam/extrinsics/depth_to_color")}


@pytest.mark.parametrize("change", ["legacy_metadata", "wrong_rate", "unconfigured"])
def test_image_report_cannot_claim_older_or_different_rate_assessment(bag, tmp_path, monkeypatch, change):
    def mutate(report, argv):
        entry = dict(type="sensor_msgs/msg/Image", recorded_message_count=31,
                     inspection="image_headers", expected_rate_hz=30., rate_assessment="within_tolerance")
        if change == "legacy_metadata":
            entry = dict(type="sensor_msgs/msg/Image", recorded_message_count=31, inspection="metadata_only")
        elif change == "wrong_rate":
            entry["expected_rate_hz"] = 15.
        else:
            entry["rate_assessment"] = "not_configured"
        report["topics"] = {"/camera/image": entry}
        report["summary"].update(topic_count=1, inspected_topic_count=int(change != "legacy_metadata"),
                                 metadata_only_topic_count=int(change == "legacy_metadata"), warning_count=0)
        report.update(warnings=[], status="ok")
    install_fake(monkeypatch, mutate=mutate)
    output = tmp_path / "old-report.json"
    with pytest.raises(QualityRunError):
        QualityRunner().run(bag, output, expected_topics=["/camera/image"],
                            expected_image_rates_hz={"/camera/image": 30.})
    assert not output.exists()


@pytest.mark.parametrize("assessment", ["below_expected", "insufficient_samples", "wrong_type"])
def test_image_rate_observation_without_required_warning_is_rejected(bag, tmp_path, monkeypatch, assessment):
    def mutate(report, argv):
        wrong_type = assessment == "wrong_type"
        report["topics"] = {"/camera/image": dict(
            type="unknown/msg/Custom" if wrong_type else "sensor_msgs/msg/Image", recorded_message_count=1,
            inspection="metadata_only" if wrong_type else "image_headers",
            expected_rate_hz=30., rate_assessment=assessment)}
        report["summary"].update(topic_count=1, inspected_topic_count=int(not wrong_type),
                                 metadata_only_topic_count=int(wrong_type), warning_count=0)
        report.update(warnings=[], status="ok")
    install_fake(monkeypatch, mutate=mutate)
    output = tmp_path / "omitted-warning.json"
    with pytest.raises(QualityRunError, match="omitted"):
        QualityRunner().run(bag, output, expected_topics=["/camera/image"],
                            expected_image_rates_hz={"/camera/image": 30.})
    assert not output.exists()


@pytest.mark.parametrize("rates", [None, {"/camera/image": 30.}])
def test_real_child_preserves_empty_image_header_and_rate_assessment(bag, tmp_path, rates):
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    metadata["rosbag2_bagfile_information"]["topics_with_message_count"][0]["topic_metadata"]["type"] = "sensor_msgs/msg/Image"
    metadata_path.write_text(yaml.safe_dump(metadata))
    report = QualityRunner(timeout_s=10).run(bag, tmp_path / "empty-image.json", expected_topics=["/camera/image"],
                                            expected_image_rates_hz=rates)
    entry = report["topics"]["/camera/image"]
    assert entry["inspection"] == "image_headers"
    assert entry["observed_messages"] == 0
    assert entry["expected_rate_hz"] == (30. if rates else None)
    assert entry["rate_assessment"] == ("insufficient_samples" if rates else "not_configured")
    assert report["summary"]["inspected_topic_count"] == 1
    assert report["runner_provenance"]["expected_image_rates_hz"] == (rates or {})
    assert any(w["code"] == "image_timing_insufficient" for w in report["warnings"]) == bool(rates)


def test_existing_and_inside_outputs_rejected_without_process(bag, tmp_path, monkeypatch):
    calls = install_fake(monkeypatch)
    output = tmp_path / "existing.json"
    output.write_text("preserve")
    for target in (output, bag / "must-not-write.json"):
        with pytest.raises(QualityRunError):
            QualityRunner().run(bag, target, expected_topics=[])
    assert output.read_text() == "preserve" and not calls
    assert not (bag / "must-not-write.json").exists()


@pytest.mark.parametrize("topics", [["/same", "/same"], ["not-absolute"], ["/x;touch_bad"], "not-list", [True]])
def test_invalid_expected_topics_never_launch(bag, tmp_path, monkeypatch, topics):
    calls = install_fake(monkeypatch)
    with pytest.raises(QualityRunError):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=topics)
    assert not calls


@pytest.mark.parametrize("mutation", [
    lambda report: report.update(schema_version=2),
    lambda report: report.update(status="PASS"),
    lambda report: report["bag"].update(path="/different/bag"),
    lambda report: report["runner_provenance"].update(invocation_id="different"),
    lambda report: report["runner_provenance"].update(input_fingerprint="different"),
    lambda report: report["runner_provenance"].update(expected_topics=[]),
    lambda report: report["summary"].update(warning_count=0),
    lambda report: report["bag"].update(duration_ns=float("nan")),
    lambda report: report.update(warnings=[], status="ok", summary=dict(topic_count=0, inspected_topic_count=0, metadata_only_topic_count=0, warning_count=0)),
])
def test_invalid_report_not_published(bag, tmp_path, monkeypatch, mutation):
    install_fake(monkeypatch, mutate=lambda report, _: mutation(report))
    output = tmp_path / "report.json"
    with pytest.raises(QualityRunError):
        QualityRunner().run(bag, output, expected_topics=["/required"])
    assert not output.exists() and not list(tmp_path.glob(".quality-*"))


def test_cancel_before_launch_has_no_side_effects(bag, tmp_path, monkeypatch):
    calls = install_fake(monkeypatch)
    with pytest.raises(QualityCancelled):
        QualityRunner().run(bag, tmp_path / "reports/result.json", expected_topics=[], cancel=lambda: True)
    assert not calls and not (tmp_path / "reports").exists()


def test_cancel_running_child_only_terminates_owned_process(bag, tmp_path, monkeypatch):
    child = FakeProcess(returncode=None)
    calls = install_fake(monkeypatch, process=child)
    with pytest.raises(QualityCancelled):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[], cancel=lambda: bool(calls))
    assert child.terminated and child.waited and not child.killed
    assert not (tmp_path / "report.json").exists()


def test_timeout_escalates_only_owned_child(bag, tmp_path, monkeypatch):
    child = FakeProcess(returncode=None, kill_required=True)
    install_fake(monkeypatch, process=child)
    with pytest.raises(QualityRunError, match="timed out"):
        QualityRunner(timeout_s=.02, poll_interval_s=.005, terminate_grace_s=.01).run(
            bag, tmp_path / "report.json", expected_topics=[])
    assert child.terminated and child.killed and child.waited
    assert not (tmp_path / "report.json").exists()


def test_child_failure_reports_bounded_error(bag, tmp_path, monkeypatch):
    install_fake(monkeypatch, returncode=2, logs=b"missing ROS message support")
    with pytest.raises(QualityRunError, match="missing ROS message support"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])
    assert not (tmp_path / "report.json").exists()


def test_metadata_or_data_stat_change_during_work_discards_report(bag, tmp_path, monkeypatch):
    payload = bag / "data.mcap"
    payload.write_bytes(b"not-a-real-bag")
    def changed(report, argv):
        payload.write_bytes(b"modified-file-not-a-real-bag")
    install_fake(monkeypatch, mutate=changed)
    with pytest.raises(QualityRunError, match="input changed"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])
    assert not (tmp_path / "report.json").exists()


def test_metadata_path_replacement_detected_even_with_same_content(bag, tmp_path, monkeypatch):
    def changed(report, argv):
        target = bag / "metadata.yaml"
        data = target.read_bytes()
        target.unlink()
        target.write_bytes(data)
    install_fake(monkeypatch, mutate=changed)
    with pytest.raises(QualityRunError, match="input changed"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])


def test_configuration_identity_change_discards_report(bag, tmp_path, monkeypatch):
    config = tmp_path / "alignment.yaml"
    config.write_text("old configuration")
    install_fake(monkeypatch, mutate=lambda *_: config.write_text("new configuration"))
    with pytest.raises(QualityRunError, match="input changed"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[], alignment_config=config)


def test_collision_does_not_replace_another_report(bag, tmp_path, monkeypatch):
    output = tmp_path / "report.json"
    install_fake(monkeypatch, mutate=lambda *_: output.write_text("concurrent report"))
    with pytest.raises(QualityRunError):
        QualityRunner().run(bag, output, expected_topics=[])
    assert output.read_text() == "concurrent report"


def test_symlink_inputs_outputs_and_missing_metadata_files_rejected(bag, tmp_path, monkeypatch):
    calls = install_fake(monkeypatch)
    alias = tmp_path / "bag-link"
    alias.symlink_to(bag, target_is_directory=True)
    with pytest.raises(QualityRunError, match="Symlink"):
        QualityRunner().run(alias, tmp_path / "report.json", expected_topics=[])
    link = tmp_path / "output-link"
    link.symlink_to(bag, target_is_directory=True)
    with pytest.raises(QualityRunError, match="Symlink"):
        QualityRunner().run(bag, link / "report.json", expected_topics=[])
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())
    metadata["rosbag2_bagfile_information"]["relative_file_paths"] = ["missing.mcap"]
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata))
    with pytest.raises(QualityRunError, match="absent"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])
    assert not calls


def test_report_size_and_metadata_size_bounds(bag, tmp_path, monkeypatch):
    install_fake(monkeypatch)
    monkeypatch.setattr(module, "MAX_REPORT_BYTES", 32)
    with pytest.raises(QualityRunError, match="regular file no larger"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])
    monkeypatch.setattr(module, "MAX_METADATA_BYTES", 32)
    with pytest.raises(QualityRunError, match="regular file no larger"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])


def test_real_child_reuses_metadata_only_report_and_expected_topic_warnings(bag, tmp_path):
    original = (bag / "metadata.yaml").read_bytes()
    output = tmp_path / "quality/report.json"
    result = QualityRunner(timeout_s=10).run(bag, output,
        expected_topics=["/camera/image", "/gloves/left/state"])
    assert result == json.loads(output.read_text())
    assert result["report_kind"] == "post_record_capture_quality" and result["status"] == "warnings"
    assert result["summary"]["inspected_topic_count"] == 0
    assert result["topics"]["/camera/image"]["inspection"] == "metadata_only"
    assert {warning["topic"] for warning in result["warnings"] if warning["code"] == "expected_topic_missing"} == {
        "/camera/image", "/gloves/left/state"}
    provenance = result["runner_provenance"]
    assert "metadata.yaml" in provenance["topic_statistics_source"]
    assert "not hashed" in provenance["fingerprint_basis"]
    assert provenance["metadata_sha256"]
    assert provenance["expected_image_rates_hz"] == {}
    assert provenance["expected_image_rates_source"] == "not configured"
    assert (bag / "metadata.yaml").read_bytes() == original
    assert sorted(path.name for path in bag.iterdir()) == ["metadata.yaml"]


def test_real_child_with_alignment_config_keeps_configured_type_observations(bag, tmp_path):
    root = Path(__file__).resolve().parents[4]
    config = root / "tools/multisensor_alignment/config/alignment.example.yaml"
    if not config.exists():
        pytest.skip("source-tree alignment example not installed")
    result = QualityRunner(timeout_s=10).run(bag, tmp_path / "quality.json", expected_topics=["/additional"], alignment_config=config)
    missing = {warning.get("topic") for warning in result["warnings"] if warning["code"] == "expected_topic_missing"}
    assert "/additional" in missing and "/d405_1/frame_timing" in missing
    assert result["runner_provenance"]["alignment_config"]["path"] == str(config)


def test_real_child_advisory_is_published_with_coherent_status_and_count(bag, tmp_path):
    # The existing report generator can inspect this fixture's metadata without
    # loading payloads. Its ordinary report is ok; the advisory must change it.
    metadata_path = bag / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    metadata["rosbag2_bagfile_information"]["message_count"] = 1
    metadata["rosbag2_bagfile_information"]["topics_with_message_count"][0]["message_count"] = 1
    metadata_path.write_text(yaml.safe_dump(metadata))
    runner = QualityRunner(timeout_s=10)
    baseline = runner.run(bag, tmp_path / "baseline.json", expected_topics=["/camera/image"], advisories=None)
    assert baseline["status"] == "ok" and baseline["warnings"] == []
    advisory = dict(code="alignment_config_unavailable",
        detail="Configured alignment file was unavailable at START; checked actual controller topics only.")
    output = tmp_path / "advised.json"
    report = runner.run(bag, output, expected_topics=["/camera/image"], advisories=[advisory, advisory])
    assert report == json.loads(output.read_text())
    assert report["status"] == "warnings" and report["summary"]["warning_count"] == 1
    assert report["warnings"] == [advisory]
    assert report["runner_provenance"]["advisories"] == [advisory]
    assert report["runner_provenance"]["alignment_config"] is None


@pytest.mark.parametrize("advisories", [
    "not-array", [dict(code="bad-name", detail="reason")],
    [dict(code="valid", detail="")], [dict(code="valid", detail=True)],
    [dict(code="valid", detail="x" * 2049)], [dict(code="valid", detail="reason", extra=True)],
    [dict(code="valid", detail="reason")] * 17,
    [dict(code=f"reason_{index}", detail="界" * 2048) for index in range(16)],
])
def test_invalid_or_excessive_advisories_do_not_launch(bag, tmp_path, monkeypatch, advisories):
    calls = install_fake(monkeypatch)
    with pytest.raises(QualityRunError, match="advisories"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[], advisories=advisories)
    assert not calls and not (tmp_path / "report.json").exists()


@pytest.mark.parametrize("omit", ["warnings", "provenance"])
def test_parent_rejects_child_omitting_requested_advisory(bag, tmp_path, monkeypatch, omit):
    advisory = dict(code="alignment_config_unavailable", detail="Configured file was unavailable at START.")
    def mutate(report, argv):
        if omit == "warnings":
            report["warnings"] = []
            report["summary"]["warning_count"] = 0
            report["status"] = "ok"
        else:
            report["runner_provenance"]["advisories"] = []
    install_fake(monkeypatch, mutate=mutate)
    with pytest.raises(QualityRunError):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[], advisories=[advisory])
    assert not (tmp_path / "report.json").exists()


def test_fingerprint_does_not_read_payload_files(bag, monkeypatch):
    data = bag / "large-recording.mcap"
    data.write_bytes(b"stat only; not a real bag")
    original = module._read_bounded
    read_paths = []
    def observe(path, maximum):
        read_paths.append(path)
        assert path != data, "large payload files must never be hashed by fingerprinting"
        return original(path, maximum)
    monkeypatch.setattr(module, "_read_bounded", observe)
    first = module._fingerprint(bag)
    assert first["file_count"] == 2 and read_paths == [bag / "metadata.yaml"]


def test_actual_owned_child_is_reaped_on_cancellation(bag, tmp_path, monkeypatch):
    original = module.subprocess.Popen
    children = []
    def observe(*args, **kwargs):
        child = original(*args, **kwargs)
        children.append(child)
        return child
    monkeypatch.setattr(module.subprocess, "Popen", observe)
    with pytest.raises(QualityCancelled):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[], cancel=lambda: bool(children))
    assert len(children) == 1 and children[0].poll() is not None
    assert not (tmp_path / "report.json").exists()


def test_selector_setup_failure_also_reaps_owned_child(bag, tmp_path, monkeypatch):
    child = FakeProcess(returncode=None)
    install_fake(monkeypatch, process=child)
    def fail():
        raise OSError("selector allocation failed")
    monkeypatch.setattr(module.selectors, "DefaultSelector", fail)
    with pytest.raises(QualityRunError, match="selector allocation failed"):
        QualityRunner().run(bag, tmp_path / "report.json", expected_topics=[])
    assert child.terminated and child.waited


@pytest.mark.parametrize("kwargs", [{"timeout_s": 0}, {"timeout_s": float("nan")},
                                    {"poll_interval_s": 2}, {"terminate_grace_s": 11}])
def test_invalid_runtime_bounds(kwargs):
    with pytest.raises(ValueError):
        QualityRunner(**kwargs)
