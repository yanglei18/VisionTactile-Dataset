from copy import deepcopy

import pytest

from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.ui.main_window import MainWindow
from vt_capture_console.ui.widgets.trial_details import TrialDetailsDialog
from test_managed_workflow import ManagedBackend


def detail(trial_id="t1", *, state="COMPLETED", verdict="warnings"):
    return {
        "trial_id": trial_id, "metadata": {"start": {"task_name": "抓取", "operator_name": "A01"}, "finish": {"state": "COMPLETE"}},
        "quality": {"state": state, "verdict": verdict, "detail": "已检查完成" if state == "COMPLETED" else "依赖不可用",
                    "attempt": 1, "report_path": "/test-only/report.json", "report": {
                        "summary": {"topic_count": 1, "inspected_topic_count": 0, "warning_count": 1},
                        "topics": {"/test/image": {"recorded_message_count": 60, "metadata_global_duration_rate_hz": 30.}},
                        "warnings": [{"code": "empty_topic", "topic": "/test/depth", "detail": "缺流 <b>不是 HTML</b>"}],
                        "limitations": ["图像仅统计 metadata，未解析像素内容。"]}},
        "artifacts": {"start_metadata_path": "/test-only/start.json", "finish_metadata_path": "/test-only/finish.json"},
    }


class DetailsBackend(ManagedBackend):
    supports_details = True

    def __init__(self):
        super().__init__()
        self.latest_operation = dict(request_id="old", state="accepted", code="started", trial_id="t1", detail="")
        self.confirm_recording()
        self.value["management"]["trials"][0].update(state="COMPLETE", quality="WARNINGS")
        self.value["management"]["controller"].update(state=5, recorder_alive=False)
        self.latest_details = None
        self.details_status = {"state": "idle", "trial_id": "", "request_id": "", "error": ""}
        self.details_requests = []
        self.result = detail()

    def request_details(self, trial_id):
        self.details_requests.append(trial_id)
        self.latest_details = deepcopy(self.result)
        self.details_status = {"state": "ready", "trial_id": trial_id, "request_id": "details-1", "error": ""}
        return "details-1"


def test_details_are_on_demand_and_never_capture_commands(qapp):
    backend = DetailsBackend()
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        for _ in range(4):
            window.refresh()
        assert backend.details_requests == []
        window.trial_table.selectRow(0)
        window.inspect_trial_button.click()
        dialog = window.details_dialog
        assert dialog.isVisible()
        assert backend.details_requests == ["t1"]
        assert "存在告警" in dialog.quality_summary.text()
        assert dialog.topics.item(0, 2).text() == "30.00"
        assert dialog.topics.item(0, 3).text() == "消息数 / 数据包总时长"
        assert "<b>不是 HTML</b>" in dialog.warnings.toPlainText()
        assert not dialog.retry_button.isEnabled()  # Data warnings are not execution failures.
        dialog.close()
        window.refresh()
        assert backend.commands == []
    finally:
        window.close()


def test_retry_targets_selected_failed_job_and_does_not_change_capture_state(qapp):
    backend = DetailsBackend()
    backend.result = detail(state="ERROR", verdict="")
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        window.trial_table.selectRow(0)
        window.inspect_trial_button.click()
        window.details_dialog.retry_button.click()
        assert backend.commands[-1][1:] == ("quality.retry", {"trial_id": "t1"})
        assert backend.value["management"]["trials"][0]["state"] == "COMPLETE"
        assert window.pending_request
        backend.latest_operation.update(state="accepted", code="quality_queued", trial_id="t1")
        window.refresh()
        assert window.pending_request is None
    finally:
        window.close()


def test_stale_details_disable_retry_but_keep_history(qapp):
    backend = DetailsBackend()
    backend.result = detail(state="ERROR", verdict="")
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        window.trial_table.selectRow(0)
        window.inspect_trial_button.click()
        backend.supports_details = False
        backend.details_status["state"] = "stale"
        window.refresh()
        assert "缓存详情" in window.details_dialog.connection.text()
        assert not window.details_dialog.retry_button.isEnabled()
        assert "抓取" in window.details_dialog.metadata.toPlainText()
        assert backend.commands == []
    finally:
        window.close()


def test_selected_trial_stays_bound_when_new_row_is_inserted(qapp):
    backend = DetailsBackend()
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        window.trial_table.selectRow(0)
        old = backend.value["management"]["trials"][0]
        backend.value["management"]["trials"].insert(0, {**old, "id": "new-trial", "label": "另一试次"})
        window.refresh()
        assert window.trial_table.currentRow() == 1
        window.inspect_trial_button.click()
        assert backend.details_requests == ["t1"]
        assert window.details_dialog.trial_id == "t1"
    finally:
        window.close()


def test_optional_metadata_v3_and_demo_frozen_in_memory(qapp, tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    backend = DemoBackend()
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        window.navigation.button(4).click()
        window.task_name.setText("抓取试验")
        window.operator_name.setText("OP01")
        window.start_button.click()
        identifier = backend.trials[0]["id"]
        window.task_name.setText("后续任务")
        window.stop_button.click()
        window.navigation.button(1).click()
        window.trial_table.selectRow(0)
        window.inspect_trial_button.click()
        assert backend.latest_details["metadata"]["start"]["task_name"] == "抓取试验"
        assert backend.latest_details["quality"]["report"]["report_kind"] == "synthetic_demo"
        assert "演示详情" in window.details_dialog.connection.text()
        assert not window.details_dialog.open_report_button.isEnabled()
        assert backend.latest_details["trial_id"] == identifier
        assert list(tmp_path.iterdir()) == []
    finally:
        window.close()


def test_legacy_backend_keeps_controls_and_does_not_silently_drop_metadata(qapp):
    backend = ManagedBackend()
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        window.navigation.button(4).click()
        assert not window.task_name.isEnabled()
        window.task_name.setText("切换后台之前输入的任务")
        window.refresh()
        assert window.task_name.isEnabled()  # Downgrade must still let the operator clear retained input.
        window.start_button.click()
        assert not backend.commands
        assert "不支持元信息" in window.operation_notice.text()
        window.task_name.clear()
        window.start_button.click()
        assert backend.commands[0][1] == "capture.start"
        assert "task_name" not in backend.commands[0][2]
    finally:
        window.close()


def test_missing_local_path_is_reported_without_launching_program(qapp, monkeypatch):
    dialog = TrialDetailsDialog()
    monkeypatch.setattr("vt_capture_console.ui.widgets.trial_details.QDesktopServices.openUrl",
                        lambda *_: (_ for _ in ()).throw(AssertionError("must not launch")))
    try:
        dialog._paths = {"report": "/test-only/does-not-exist/report.json"}
        dialog._open("report")
        assert "无法打开本机路径" in dialog.connection.text()
    finally:
        dialog.close()


@pytest.mark.parametrize("report", [
    {"summary": "truncated", "warnings": None, "topics": []},
    {"summary": {}, "warnings": [None, "partial"], "topics": {"/bad": None}, "limitations": 4},
    {"preview_truncated": True, "summary": "Preview omitted"},
])
def test_bounded_generic_report_preview_does_not_crash_ui(qapp, report):
    backend = DetailsBackend()
    backend.result["quality"]["report"] = report
    window = MainWindow(backend)
    window.navigation.button(1).click()
    try:
        window.trial_table.selectRow(0)
        window.inspect_trial_button.click()
        assert window.details_dialog.topics.rowCount() == 0
        assert "—" in window.details_dialog.quality_summary.text()
        if report.get("preview_truncated"):
            assert "仅展示报告摘要" in window.details_dialog.warnings.toPlainText()
    finally:
        window.close()


@pytest.mark.parametrize("language,phrases", [
    ("zh_CN", ["图像源时间平均帧率 20.000 Hz", "最低 29.700 Hz", "期望 30 Hz",
               "有效图像时间戳不足两个", "图像源时间间隔 2 次超过 50000000 ns",
               "图像源时间戳重复 3 次", "图像源时间戳回退 4 次"]),
    ("en", ["Image source-time average rate 20.000 Hz", "minimum 29.700 Hz", "expected 30 Hz",
            "Fewer than two valid image timestamps", "2 image source-time intervals exceed 50000000 ns",
            "3 duplicate image source timestamps", "4 regressing image source timestamps"]),
])
def test_image_quality_warnings_localize_evidence_without_mutating_report(qapp, language, phrases):
    from vt_capture_console import i18n

    before_language = i18n.language()
    i18n.set_language(language, persist=False)
    backend = DetailsBackend()
    details = detail()
    warnings = [
        dict(code="image_rate_below_expected", observed_rate_hz=20., minimum_rate_hz=29.7, expected_rate_hz=30.),
        dict(code="image_timing_insufficient"),
        dict(code="image_source_gaps", count=2, gap_threshold_ns=50_000_000),
        dict(code="image_source_duplicates", count=3),
        dict(code="image_source_regressions", count=4),
    ]
    for entry in warnings:
        entry.update(topic="/相机/opaque_topic", detail="Untranslated original observation")
    warnings.extend([
        dict(code="image_rate_below_expected", observed_rate_hz="bad", detail="Keep malformed evidence"),
        dict(code="external_code", detail="Keep unknown text / 原始诊断"),
    ])
    details["quality"]["report"]["warnings"] = warnings
    original = deepcopy(details)
    dialog = TrialDetailsDialog()
    try:
        dialog.display(backend.value["management"]["trials"][0], details, {"state": "ready"}, supported=True)
        text = dialog.warnings.toPlainText()
        assert all(phrase in text for phrase in phrases)
        assert "/相机/opaque_topic" in text
        assert "Keep malformed evidence" in text
        assert "Keep unknown text / 原始诊断" in text
        assert details == original
    finally:
        dialog.close()
        i18n.set_language(before_language, persist=False)


@pytest.mark.parametrize("language,phrase", [
    ("zh_CN", "未录入可选数据主题：3 路"),
    ("en", "Optional topics not recorded: 3"),
])
def test_optional_absence_is_information_without_quality_warnings(qapp, language, phrase):
    from vt_capture_console import i18n

    before_language = i18n.language()
    i18n.set_language(language, persist=False)
    backend = DetailsBackend()
    details = detail(verdict="ok")
    report = details["quality"]["report"]
    absent = [f"/vive/{role}/sample" for role in ("left_wrist", "right_wrist", "torso")]
    report.update(topic_coverage={"optional_topics": absent, "absent_optional_topics": absent},
                  warnings=[], limitations=[])
    report["summary"]["warning_count"] = 0
    original = deepcopy(details)
    dialog = TrialDetailsDialog()
    try:
        trial = backend.value["management"]["trials"][0]
        dialog.display(trial, details, {"state": "ready"}, supported=True)
        assert phrase in dialog.quality_summary.text()
        assert dialog.quality_summary.toolTip().splitlines() == absent
        assert all(topic not in dialog.warnings.toPlainText() for topic in absent)
        assert "存在告警" not in dialog.quality_summary.text()
        assert "Warnings found" not in dialog.quality_summary.text()
        assert details == original
        # A legacy report has no availability contract and must clear the hint.
        del report["topic_coverage"]
        dialog.display(trial, details, {"state": "ready"}, supported=True)
        assert phrase not in dialog.quality_summary.text()
        assert not dialog.quality_summary.toolTip()
    finally:
        dialog.close()
        i18n.set_language(before_language, persist=False)


@pytest.mark.parametrize("language,phrases,basis", [
    ("zh_CN", ["相机标定参数无效：2 条", "畸变模型暂不支持", "标定参数发生变化：1 次", "图像已有数据，但未录入对应的相机标定参数"], "标定参数"),
    ("en", ["Invalid camera calibration: 2 messages", "Unsupported distortion model", "Calibration changed: 1 time", "Images were recorded without the corresponding camera calibration"], "Calibration parameters"),
])
def test_camera_calibration_evidence_is_concise_and_has_no_frame_rate(qapp, language, phrases, basis):
    from vt_capture_console import i18n

    before_language = i18n.language()
    i18n.set_language(language, persist=False)
    backend = DetailsBackend()
    details = detail()
    report = details["quality"]["report"]
    report["warnings"] = [
        dict(code="invalid_camera_calibration", count=2),
        dict(code="unsupported_camera_distortion", count=1),
        dict(code="camera_calibration_changed", count=1),
        dict(code="camera_calibration_missing"),
        dict(code="invalid_camera_calibration", count="bad", detail="Keep raw evidence"),
    ]
    report["topics"] = {"/d436/extrinsics/depth_to_color": dict(
        inspection="calibration_messages", recorded_message_count=1,
        metadata_global_duration_rate_hz=.033)}
    original = deepcopy(details)
    dialog = TrialDetailsDialog()
    try:
        dialog.display(backend.value["management"]["trials"][0], details, {"state": "ready"}, supported=True)
        assert all(phrase in dialog.warnings.toPlainText() for phrase in phrases)
        assert "Keep raw evidence" in dialog.warnings.toPlainText()
        assert dialog.topics.item(0, 1).text() == "1"
        assert dialog.topics.item(0, 2).text() == "—"
        assert dialog.topics.item(0, 3).text() == basis
        assert details == original
    finally:
        dialog.close()
        i18n.set_language(before_language, persist=False)
