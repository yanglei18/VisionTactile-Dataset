"""On-demand trial inspection; no ROS, file parsing, or capture ownership."""
from __future__ import annotations

from ...i18n import tr

import json
import math
from pathlib import Path

from PySide6.QtCore import Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from ...i18n.widgets import (
    QAbstractItemView, QDialog, QHBoxLayout, QHeaderView, QLabel,
    QPlainTextEdit, QPushButton, QTabWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from ...viewmodels.recording import TRIAL_LABELS


QUALITY_LABELS = {
    "NOT_ASSESSED": tr('未执行'), "NOT_CHECKED": tr('未执行'), "not_checked": tr('未执行'),
    "QUEUED": tr('等待检查'), "RUNNING": tr('检查中'), "COMPLETED": tr('检查完成'),
    "OK": tr('已检查项无告警'), "WARNINGS": tr('存在告警'), "ERROR": tr('检查执行失败'),
    "INTERRUPTED": tr('检查中断'), "": tr('未执行'), None: tr('未执行'),
}


def _quality_warning_detail(entry):
    """Translate known structured evidence without rewriting archived reports."""
    code = entry.get("code")
    if code == "image_rate_below_expected":
        values = [entry.get(key) for key in ("observed_rate_hz", "minimum_rate_hz", "expected_rate_hz")]
        if all(type(value) in (int, float) and 0 <= value < 2**63 and math.isfinite(value) for value in values):
            return tr('图像源时间平均帧率 {observed:.3f} Hz；最低 {minimum:.3f} Hz，期望 {expected:g} Hz。',
                      observed=values[0], minimum=values[1], expected=values[2])
    elif code == "image_timing_insufficient":
        return tr('有效图像时间戳不足两个，或源时间跨度不足 1 秒，无法评估期望帧率。')
    elif code == "camera_calibration_missing":
        return tr('图像已有数据，但未录入对应的相机标定参数。')
    elif code in {"invalid_camera_calibration", "unsupported_camera_distortion", "camera_calibration_changed"}:
        count = entry.get("count")
        if type(count) is int and count >= 0:
            if code == "invalid_camera_calibration":
                return tr('相机标定参数无效：{count} 条；原因见完整报告。', count=count)
            if code == "unsupported_camera_distortion":
                return tr('畸变模型暂不支持：{count} 条；原始参数已保留。', count=count)
            return tr('标定参数发生变化：{count} 次；请查看完整报告。', count=count)
    elif code in {"image_source_gaps", "image_source_duplicates", "image_source_regressions"}:
        count = entry.get("count")
        if type(count) is int and count >= 0:
            if code == "image_source_gaps":
                threshold = entry.get("gap_threshold_ns")
                if type(threshold) is int and threshold > 0:
                    return tr('图像源时间间隔 {count} 次超过 {threshold} ns；请查看完整报告。',
                              count=count, threshold=threshold)
            elif code == "image_source_duplicates":
                return tr('图像源时间戳重复 {count} 次。', count=count)
            else:
                return tr('图像源时间戳回退 {count} 次。', count=count)
    return entry.get("detail", "")


def plain_label(text="", name="", wrap=True):
    widget = QLabel(text)
    widget.setObjectName(name)
    widget.setTextFormat(Qt.TextFormat.PlainText)
    widget.setWordWrap(wrap)
    widget.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    return widget


def readonly_text():
    widget = QPlainTextEdit()
    widget.setReadOnly(True)
    widget.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
    return widget


def replace_text(widget, text):
    # Preserve scroll position/selection while the background refreshes.
    if widget.toPlainText() != text:
        position = widget.verticalScrollBar().value()
        widget.setPlainText(text)
        widget.verticalScrollBar().setValue(position)


class TrialDetailsDialog(QDialog):
    retry_requested = Signal(str)
    refresh_requested = Signal(str)
    replay_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle(tr('试次详情 · 元信息与录后质检'))
        self.setModal(False)
        self.resize(900, 660)
        self.setMinimumSize(680, 470)
        self.trial_id = None
        self._paths = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 20, 22, 18)
        layout.setSpacing(12)
        self.heading = plain_label(tr('选择试次'), "sectionTitle")
        self.connection = plain_label(tr('详情按需加载，不影响录制。'), "muted")
        layout.addWidget(self.heading)
        layout.addWidget(self.connection)
        self.tabs = QTabWidget()
        self.overview = readonly_text()
        self.metadata = readonly_text()
        self.tabs.addTab(self.overview, tr('概览'))
        self.tabs.addTab(self.metadata, tr('元信息快照'))
        quality_page = QWidget()
        quality_layout = QVBoxLayout(quality_page)
        self.quality_summary = plain_label(tr('尚未执行质量检查。'))
        quality_layout.addWidget(self.quality_summary)
        self.topics = QTableWidget(0, 4)
        self.topics.setHorizontalHeaderLabels([tr('数据主题'), tr('消息数'), tr('统计 Hz'), tr('频率依据')])
        self.topics.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for index in (1, 2):
            self.topics.horizontalHeader().setSectionResizeMode(index, QHeaderView.ResizeMode.ResizeToContents)
        self.topics.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeMode.Stretch)
        self.topics.verticalHeader().hide()
        self.topics.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.topics.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.topics.setMinimumHeight(100)
        quality_layout.addWidget(self.topics, 1)
        self.warnings = readonly_text()
        quality_layout.addWidget(self.warnings, 1)
        self.tabs.addTab(quality_page, tr('质量报告'))
        layout.addWidget(self.tabs, 1)
        layout.addWidget(plain_label(tr('录制结束不等于质量合格；无告警仅涵盖本报告已检查的项目。'), "muted"))
        actions = QHBoxLayout()
        self.refresh_button = QPushButton(tr('刷新详情'))
        self.retry_button = QPushButton(tr('重试检查'))
        self.open_bag_button = QPushButton(tr('打开数据目录'))
        self.open_report_button = QPushButton(tr('打开报告'))
        self.replay_button = QPushButton(tr('在档案中回放'))
        self.replay_button.clicked.connect(lambda: self.replay_requested.emit(self.trial_id or ""))
        self.refresh_button.clicked.connect(lambda: self.refresh_requested.emit(self.trial_id or ""))
        self.retry_button.clicked.connect(lambda: self.retry_requested.emit(self.trial_id or ""))
        self.open_bag_button.clicked.connect(lambda: self._open("bag"))
        self.open_report_button.clicked.connect(lambda: self._open("report"))
        for button in (self.refresh_button, self.retry_button, self.open_bag_button, self.open_report_button, self.replay_button):
            button.setEnabled(False)
            actions.addWidget(button)
        actions.addStretch()
        close = QPushButton(tr('关闭'))
        close.clicked.connect(self.close)
        actions.addWidget(close)
        layout.addLayout(actions)

    def bind(self, trial):
        identity = trial["id"]
        if identity != self.trial_id:
            self.trial_id = identity
            self._paths = {}
            self.metadata.clear()
            self.warnings.clear()
            self.topics.setRowCount(0)
            self.tabs.setCurrentIndex(0)
        self.heading.setText(trial.get("label") or identity)

    def display(self, trial, details, status, *, supported=False, pending=False, demo=False):
        self.bind(trial)
        connection_state = status.get("state", "unsupported")
        matching = details is not None and details.get("trial_id") == self.trial_id
        fresh = supported and matching and connection_state == "ready"
        quality = details["quality"] if matching else {}
        metadata = details["metadata"] if matching else {}
        self.connection.setText({
            "idle": tr('等待加载详情。'), "loading": tr('正在加载详情…'),
            "ready": tr('演示详情 · 合成数据，不生成数据包或报告文件。') if demo else tr('详情已更新 · 元信息为对应试次的冻结记录。'),
            "stale": tr('后台未连接：显示缓存详情，不能据此确认当前检查状态。'),
            "unsupported": tr('当前后台不提供 v3 试次详情；原录制功能不受影响。'),
            "error": tr('详情加载失败：') + status.get("error", tr('请刷新重试')),
        }.get(connection_state, tr('详情状态未知')))
        quality_state = quality.get("state", trial.get("quality", "NOT_ASSESSED"))
        verdict = quality.get("verdict", "")
        quality_text = QUALITY_LABELS.get(quality_state, quality_state)
        if quality_state == "COMPLETED":
            quality_text += " · " + {"ok": tr('已检查项无告警'), "warnings": tr('存在告警')}.get(verdict, tr('结果未知'))
        report = quality.get("report") or {}
        summary = report.get("summary")
        summary = summary if isinstance(summary, dict) else {}
        summary_text = tr(
            '{v0}  ·  第 {v1} 次检查\n{v2}\n数据主题 {v3}  ·  已解析 {v4}  ·  告警 {v5}',
            v0=quality_text, v1=quality.get('attempt', 0), v2=quality.get('detail', ''),
            v3=summary.get('topic_count', '—'), v4=summary.get('inspected_topic_count', '—'),
            v5=summary.get('warning_count', '—'))
        coverage = report.get("topic_coverage")
        absent = coverage.get("absent_optional_topics", []) if isinstance(coverage, dict) else []
        absent = absent if isinstance(absent, list) and all(isinstance(topic, str) for topic in absent) else []
        if absent:
            summary_text += tr('\n未录入可选数据主题：{count} 路（悬停查看）。', count=len(absent))
        self.quality_summary.setText(summary_text)
        self.quality_summary.setToolTip("\n".join(absent))
        overview = [tr('试次：{v0}', v0=self.trial_id), tr('批次：{v0}', v0=trial['batch_id']),
                    tr('录制状态：{v0}', v0=TRIAL_LABELS.get(trial['state'], trial['state'])),
                    tr('会话：{v0}', v0=trial.get('session_id') or tr('尚未确认')),
                    tr('实际 / 计划时长：{v0:.1f} / {v1:.0f} 秒', v0=trial['elapsed_s'], v1=trial['duration_s']),
                    tr('bag：{v0}', v0=trial.get('output_path') or (tr('仅演示，无数据包') if demo else tr('尚未确认'))),
                    tr('质量检查：{v0}', v0=quality_text), tr('说明：{v0}', v0=trial.get('detail', ''))]
        if matching:
            artifacts = details["artifacts"]
            overview += ["", tr('开始快照：') + (artifacts.get("start_metadata_path") or tr('未提供')),
                         tr('结束快照：') + (artifacts.get("finish_metadata_path") or tr('未提供')),
                         tr('质量报告：') + (quality.get("report_path") or tr('尚未生成'))]
        replace_text(self.overview, tr("\n").join(overview))
        replace_text(self.metadata, json.dumps(metadata, ensure_ascii=False, indent=2) if matching
                     else tr('未获得元信息。旧试次不会被补写为已有完整采集快照。'))
        warning_entries = report.get("warnings")
        warning_entries = warning_entries if isinstance(warning_entries, list) else []
        warnings = [tr('[{v0}] {v1}\n{v2}', v0=entry.get('code', 'warning'), v1=entry.get('topic', ''), v2=_quality_warning_detail(entry))
                    for entry in warning_entries if isinstance(entry, dict)]
        limitations = report.get("limitations")
        limitations = limitations if isinstance(limitations, list) else []
        text = tr("\n\n").join(warnings) or (tr('已检查项未发现告警。') if verdict == "ok" else tr('尚无报告内容。'))
        if limitations:
            text += tr('\n\n检查范围与限制：\n') + "\n".join(str(item) for item in limitations)
        if report.get("preview_truncated"):
            text += tr('\n\n界面仅展示报告摘要；请打开完整报告查看其余内容。')
        replace_text(self.warnings, text)
        topics = report.get("topics")
        rows = [(name, entry) for name, entry in topics.items() if isinstance(entry, dict)] if isinstance(topics, dict) else []
        self.topics.setRowCount(len(rows))
        for row, (topic, entry) in enumerate(rows):
            # Do not relabel a bag-span average as an actual source rate.
            calibration = entry.get("inspection") == "calibration_messages"
            rate = None if calibration else entry.get("metadata_global_duration_rate_hz")
            values = (topic, entry.get("recorded_message_count", "—"),
                      tr('{v0:.2f}', v0=rate) if type(rate) in (float, int) else "—",
                      tr('标定参数') if calibration else
                      (tr('消息数 / 数据包总时长') if rate is not None else tr('见原始报告')))
            for column, value in enumerate(values):
                item = self.topics.item(row, column)
                if item is None:
                    item = QTableWidgetItem()
                    self.topics.setItem(row, column, item)
                if item.text() != str(value):
                    item.setText(str(value))
                item.setToolTip(str(value))
        self._paths = {"bag": trial.get("output_path", ""), "report": quality.get("report_path", "")}
        self.refresh_button.setEnabled(supported and connection_state != "loading")
        self.retry_button.setEnabled(fresh and not pending and quality_state in {"ERROR", "INTERRUPTED"})
        self.open_bag_button.setEnabled(bool(self._paths["bag"]) and not demo)
        self.open_report_button.setEnabled(bool(self._paths["report"]) and quality_state == "COMPLETED" and not demo)
        from .topic_replay import trial_replay_allowed
        self.replay_button.setEnabled(trial_replay_allowed(trial, demo=demo))

    def _open(self, kind):
        # Explicit user action only. No file is executed or parsed by the GUI.
        path = Path(self._paths.get(kind, ""))
        valid = path.is_absolute() and (path.is_dir() if kind == "bag" else path.is_file())
        if not valid or not QDesktopServices.openUrl(QUrl.fromLocalFile(str(path))):
            self.connection.setText(tr('无法打开本机路径：文件可能已移动，或未配置默认查看程序。'))
