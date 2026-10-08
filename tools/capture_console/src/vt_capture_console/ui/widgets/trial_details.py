"""On-demand trial inspection; no ROS, file parsing, or capture ownership."""
from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices
from PySide6.QtWidgets import (
    QAbstractItemView, QDialog, QHBoxLayout, QHeaderView, QLabel,
    QPlainTextEdit, QPushButton, QTabWidget, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from ...viewmodels.recording import TRIAL_LABELS


QUALITY_LABELS = {
    "NOT_ASSESSED": "未执行", "NOT_CHECKED": "未执行", "not_checked": "未执行",
    "QUEUED": "等待检查", "RUNNING": "检查中", "COMPLETED": "检查完成",
    "OK": "已检查项无告警", "WARNINGS": "存在告警", "ERROR": "检查执行失败",
    "INTERRUPTED": "检查中断", "": "未执行", None: "未执行",
}


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

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("试次详情 · 元信息与录后质检")
        self.setModal(False)
        self.resize(900, 660)
        self.setMinimumSize(680, 470)
        self.trial_id = None
        self._paths = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 20, 22, 18)
        layout.setSpacing(12)
        self.heading = plain_label("选择试次", "sectionTitle")
        self.connection = plain_label("详情按需加载，不影响录制。", "muted")
        layout.addWidget(self.heading)
        layout.addWidget(self.connection)
        self.tabs = QTabWidget()
        self.overview = readonly_text()
        self.metadata = readonly_text()
        self.tabs.addTab(self.overview, "概览")
        self.tabs.addTab(self.metadata, "元信息快照")
        quality_page = QWidget()
        quality_layout = QVBoxLayout(quality_page)
        self.quality_summary = plain_label("尚未执行质量检查。")
        quality_layout.addWidget(self.quality_summary)
        self.topics = QTableWidget(0, 4)
        self.topics.setHorizontalHeaderLabels(["Topic", "消息数", "统计 Hz", "频率依据"])
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
        self.tabs.addTab(quality_page, "质量报告")
        layout.addWidget(self.tabs, 1)
        layout.addWidget(plain_label("录制结束不等于质量合格；无告警仅涵盖本报告已检查的项目。", "muted"))
        actions = QHBoxLayout()
        self.refresh_button = QPushButton("刷新详情")
        self.retry_button = QPushButton("重试检查")
        self.open_bag_button = QPushButton("打开数据目录")
        self.open_report_button = QPushButton("打开报告")
        self.refresh_button.clicked.connect(lambda: self.refresh_requested.emit(self.trial_id or ""))
        self.retry_button.clicked.connect(lambda: self.retry_requested.emit(self.trial_id or ""))
        self.open_bag_button.clicked.connect(lambda: self._open("bag"))
        self.open_report_button.clicked.connect(lambda: self._open("report"))
        for button in (self.refresh_button, self.retry_button, self.open_bag_button, self.open_report_button):
            button.setEnabled(False)
            actions.addWidget(button)
        actions.addStretch()
        close = QPushButton("关闭")
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
            "idle": "等待加载详情。", "loading": "正在加载详情…",
            "ready": "演示详情 · 合成数据，不生成 bag 或报告文件。" if demo else "详情已更新 · 元信息为对应试次的冻结记录。",
            "stale": "后台未连接：显示缓存详情，不能据此确认当前检查状态。",
            "unsupported": "当前后台不提供 v3 试次详情；原录制功能不受影响。",
            "error": "详情加载失败：" + status.get("error", "请刷新重试"),
        }.get(connection_state, "详情状态未知"))
        quality_state = quality.get("state", trial.get("quality", "NOT_ASSESSED"))
        verdict = quality.get("verdict", "")
        quality_text = QUALITY_LABELS.get(quality_state, quality_state)
        if quality_state == "COMPLETED":
            quality_text += " · " + {"ok": "已检查项无告警", "warnings": "存在告警"}.get(verdict, "结果未知")
        report = quality.get("report") or {}
        summary = report.get("summary")
        summary = summary if isinstance(summary, dict) else {}
        self.quality_summary.setText(
            f"{quality_text}  ·  第 {quality.get('attempt', 0)} 次检查\n"
            f"{quality.get('detail', '')}\n"
            f"Topic {summary.get('topic_count', '—')}  ·  已解析 {summary.get('inspected_topic_count', '—')}  ·  告警 {summary.get('warning_count', '—')}")
        overview = [f"试次：{self.trial_id}", f"批次：{trial['batch_id']}",
                    f"录制状态：{TRIAL_LABELS.get(trial['state'], trial['state'])}",
                    f"会话：{trial.get('session_id') or '尚未确认'}",
                    f"实际 / 计划时长：{trial['elapsed_s']:.1f} / {trial['duration_s']:.0f} 秒",
                    f"bag：{trial.get('output_path') or ('仅演示，无 bag' if demo else '尚未确认')}",
                    f"质量检查：{quality_text}", f"说明：{trial.get('detail', '')}"]
        if matching:
            artifacts = details["artifacts"]
            overview += ["", "开始快照：" + (artifacts.get("start_metadata_path") or "未提供"),
                         "结束快照：" + (artifacts.get("finish_metadata_path") or "未提供"),
                         "质量报告：" + (quality.get("report_path") or "尚未生成")]
        replace_text(self.overview, "\n".join(overview))
        replace_text(self.metadata, json.dumps(metadata, ensure_ascii=False, indent=2) if matching
                     else "未获得元信息。旧试次不会被补写为已有完整采集快照。")
        warning_entries = report.get("warnings")
        warning_entries = warning_entries if isinstance(warning_entries, list) else []
        warnings = [f"[{entry.get('code', 'warning')}] {entry.get('topic', '')}\n{entry.get('detail', '')}"
                    for entry in warning_entries if isinstance(entry, dict)]
        limitations = report.get("limitations")
        limitations = limitations if isinstance(limitations, list) else []
        text = "\n\n".join(warnings) or ("已检查项未发现告警。" if verdict == "ok" else "尚无报告内容。")
        if limitations:
            text += "\n\n检查范围与限制：\n" + "\n".join(str(item) for item in limitations)
        if report.get("preview_truncated"):
            text += "\n\n界面仅展示报告摘要；请打开完整报告查看其余内容。"
        replace_text(self.warnings, text)
        topics = report.get("topics")
        rows = [(name, entry) for name, entry in topics.items() if isinstance(entry, dict)] if isinstance(topics, dict) else []
        self.topics.setRowCount(len(rows))
        for row, (topic, entry) in enumerate(rows):
            # Do not relabel a bag-span average as an actual source rate.
            rate = entry.get("metadata_global_duration_rate_hz")
            values = (topic, entry.get("recorded_message_count", "—"),
                      f"{rate:.2f}" if type(rate) in (float, int) else "—",
                      "metadata / bag 总时长" if rate is not None else "见原始报告")
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

    def _open(self, kind):
        # Explicit user action only. No file is executed or parsed by the GUI.
        path = Path(self._paths.get(kind, ""))
        valid = path.is_absolute() and (path.is_dir() if kind == "bag" else path.is_file())
        if not valid or not QDesktopServices.openUrl(QUrl.fromLocalFile(str(path))):
            self.connection.setText("无法打开本机路径：文件可能已移动，或未配置默认查看程序。")
