"""Archive-owned topic playback, independent of recording commands and quality."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import QSplitter, QSlider
from ...i18n import tr
from ...i18n.messages import message as diagnostic
from ...i18n.widgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox,
                            QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
                            QStackedWidget, QTabWidget)
from .trial_details import plain_label, readonly_text, replace_text
from .preview import PreviewWidget


def trial_replay_allowed(trial, *, demo=False):
    return bool(not demo and trial and trial.get("state") in {"COMPLETE", "FAILED"}
                and trial.get("output_path"))


class TopicReplayPanel(QWidget):
    browse_requested = Signal()

    def __init__(self, parent=None, *, backend=None, path_blocker=None):
        super().__init__(parent)
        self.setObjectName("archiveReplayPanel")
        self.backend = backend
        self.path_blocker = path_blocker or (lambda _: False)
        self._closed = False
        self._inventory_signature = None
        self._frame_signature = None
        self._notice = ""
        self.preview = None
        self._previews = {}
        self._snapshot_signature = None
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)
        context = QHBoxLayout()
        self.back_button = QPushButton(tr('返回试次列表'))
        self.back_button.clicked.connect(self.browse_requested)
        context.addWidget(self.back_button)
        self.context = plain_label(tr('历史回放'), "sectionTitle")
        context.addWidget(self.context, 1)
        root.addLayout(context)
        self.status = plain_label(tr('选择试次后，在档案中回放已有话题。'), "muted")
        self.status.setMaximumHeight(56)
        root.addWidget(self.status)
        self.views = QTabWidget()
        self.views.setDocumentMode(True)
        from ...backend.archive_scene import ArchiveSceneBackend
        from .spatial_page import SpatialPage
        self.scene_backend = ArchiveSceneBackend()
        self.spatial = SpatialPage(self.scene_backend, archive_embedded=True)
        self.spatial.set_motion_enabled(False)
        self.views.addTab(self.spatial, tr('空间'))
        split = QSplitter(Qt.Orientation.Horizontal)
        self.topics = QTableWidget(0, 3)
        self.topics.setHorizontalHeaderLabels([tr('数据主题'), tr('消息数'), tr('类型 / 状态')])
        self.topics.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.topics.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        self.topics.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.topics.verticalHeader().hide()
        self.topics.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.topics.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.topics.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.topics.itemSelectionChanged.connect(self._select_topic)
        self.topics.setMinimumWidth(240)
        split.addWidget(self.topics)
        self.tabs = QTabWidget()
        self.preview_stack = QStackedWidget()
        self.empty = plain_label(tr('所选时刻尚无可显示的样本。'))
        self.empty.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.preview_stack.addWidget(self.empty)
        self.details = readonly_text()
        self.warnings = readonly_text()
        self.tabs.addTab(self.preview_stack, tr('预览'))
        self.tabs.addTab(self.details, tr('消息字段'))
        split.addWidget(self.tabs)
        split.setStretchFactor(0, 2)
        split.setStretchFactor(1, 3)
        self.views.addTab(split, tr('话题详情'))
        self.views.addTab(self.warnings, tr('缺失与限制'))
        root.addWidget(self.views, 1)
        self.sample_time = plain_label("", "muted")
        root.addWidget(self.sample_time)
        controls = QHBoxLayout()
        self.play_button = QPushButton(tr('播放'))
        self.play_button.setEnabled(False)
        self.play_button.clicked.connect(self._toggle_play)
        controls.addWidget(self.play_button)
        self.timeline = QSlider(Qt.Orientation.Horizontal)
        self.timeline.setRange(0, 10000)
        self.timeline.setAccessibleName(tr('回放时间轴'))
        self.timeline.sliderReleased.connect(self._seek)
        self.timeline.actionTriggered.connect(self._slider_action)
        self.timeline.setEnabled(False)
        controls.addWidget(self.timeline, 1)
        self.time_label = plain_label("00:00 / 00:00")
        controls.addWidget(self.time_label)
        self.play_rate = QComboBox()
        for rate in (.25, .5, 1., 2., 4.):
            self.play_rate.addItem(f"{rate:g}×", rate)
        self.play_rate.setCurrentIndex(2)
        self.play_rate.currentIndexChanged.connect(self._rate_changed)
        controls.addWidget(self.play_rate)
        root.addLayout(controls)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.refresh)

    def _get_backend(self):
        if self.backend is None:
            from ...backend.topic_replay import TopicReplayBackend
            self.backend = TopicReplayBackend()
        return self.backend

    def load(self, path, *, label=""):
        self.clear()
        if self.path_blocker(path):
            self._notice = tr('此试次仍在录制或状态未确认，结束后再回放。')
            self.status.setText(self._notice)
            return
        self._get_backend().load(path)
        name = Path(path).parent.name if Path(path).name == "bag" else Path(path).name
        self.context.setText(tr('{v0}', v0=label or name))
        self.context.setToolTip(str(path))
        self.views.setCurrentWidget(self.spatial)
        self._notice = ""
        self.timer.start()
        self.refresh()

    def clear(self, message=""):
        if self.backend is not None:
            self.backend.clear()
        self._notice = message
        self._inventory_signature = None
        self._snapshot_signature = None
        self.topics.blockSignals(True)
        self.topics.setRowCount(0)
        self.topics.blockSignals(False)
        self._clear_sample()
        self.scene_backend.clear()
        self.spatial.refresh()
        self.warnings.clear()
        self.status.setText(message or tr('选择试次后，在档案中回放已有话题。'))
        self.play_button.setEnabled(False)
        self.play_button.setText(tr('播放'))
        self.timeline.setEnabled(False)
        self.timeline.setValue(0)
        self.time_label.setText("00:00 / 00:00")
        self.timer.stop()

    def _clear_sample(self):
        self._frame_signature = None
        self.preview_stack.setCurrentWidget(self.empty)
        for preview in self._previews.values():
            preview.clear_frame()
            preview.set_device({"id": "empty", "kind": preview.kind, "status": "offline", "preview": {}})
        self.preview = None
        self.details.clear()
        self.sample_time.setText("")

    def _select_topic(self):
        row = self.topics.currentRow()
        if self.backend is not None and row >= 0:
            self._clear_sample()
            self.backend.select_topic(self.topics.item(row, 0).text())
            self.refresh()

    def _toggle_play(self):
        if self.backend is None:
            return
        if self.backend.timeline()["playing"]:
            self.backend.pause()
        else:
            self.backend.play()
        self.refresh()

    def _seek(self):
        if self.backend is None:
            return
        state = self.backend.timeline()
        stamp = state["start_ns"] + (state["end_ns"] - state["start_ns"]) * self.timeline.value() // 10000
        self._clear_sample()
        self.scene_backend.clear()
        self.spatial.refresh()
        self.backend.seek(stamp)

    def _slider_action(self, action):
        # Keyboard/page clicks do not emit sliderReleased; defer until Qt has
        # applied the new value. Dragging is handled on release only.
        if not self.timeline.isSliderDown():
            QTimer.singleShot(0, self._seek)

    def _rate_changed(self):
        if self.backend is not None:
            self.backend.set_rate(self.play_rate.currentData())

    @staticmethod
    def _duration(nanoseconds):
        seconds = max(0, nanoseconds / 1e9)
        return f"{int(seconds)//60:02}:{seconds%60:04.1f}"

    def refresh(self):
        if self.backend is None or self._closed or self._notice:
            return
        metadata = self.backend.metadata()
        state = self.backend.timeline()
        status = self.backend.status()
        entries = metadata.get("topics", [])
        signature = tuple((t["name"], t["type"], t["count"], t["kind"]) for t in entries)
        if signature != self._inventory_signature:
            self._inventory_signature = signature
            self.topics.blockSignals(True)
            self.topics.setRowCount(len(entries))
            for row, topic in enumerate(entries):
                values = (topic["name"], str(topic["count"]),
                          tr('无数据') if not topic["count"] else topic["type"])
                for column, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    item.setToolTip(topic["type"])
                    self.topics.setItem(row, column, item)
            self.topics.blockSignals(False)
        for row, entry in enumerate(entries):
            if entry["name"] == state["topic"] and self.topics.currentRow() != row:
                self.topics.blockSignals(True)
                self.topics.selectRow(row)
                self.topics.blockSignals(False)
        frame = self.backend.latest()
        snapshot = self.backend.snapshot() if hasattr(self.backend, "snapshot") else dict(
            time_ns=state["time_ns"], frames={frame["topic"]: frame} if frame else {}, generation=0)
        snapshot_signature = (snapshot.get("generation"), snapshot.get("seek_revision"),
                              snapshot.get("sequence"), snapshot.get("time_ns"), bool(snapshot.get("frames")))
        if snapshot_signature != self._snapshot_signature:
            self._snapshot_signature = snapshot_signature
            self.scene_backend.update(snapshot, metadata)
            self.spatial.refresh()
        usable = bool(entries) and status["state"] in {"ready", "loading"}
        self.play_button.setEnabled(usable)
        self.timeline.setEnabled(usable)
        self.play_button.setText(tr('暂停') if state["playing"] else tr('播放'))
        missing = metadata.get("warnings", [])
        message = diagnostic(status.get("message", ""))
        self.status.setText(tr('{v0} · 提示 {v1} 项', v0=message, v1=len(missing)))
        self.status.setToolTip(state.get("path", ""))
        warnings = list(missing)
        warnings.extend(snapshot.get("warnings", []))
        for topic, sample_frame in snapshot.get("frames", {}).items():
            warnings.extend(f"{topic}: {diagnostic(item)}" for item in sample_frame.get("warnings", []))
        if frame and frame.get("topic") == state["topic"] and frame["topic"] not in snapshot.get("frames", {}):
            warnings.extend(frame.get("warnings", []))
        warnings = list(dict.fromkeys(warnings))
        self.status.setText(tr('{v0} · 提示 {v1} 项', v0=message, v1=len(warnings)))
        self.views.setTabText(self.views.indexOf(self.warnings), tr('缺失与限制（{v0}）', v0=len(warnings)))
        replace_text(self.warnings, "\n\n".join(str(diagnostic(item)) for item in warnings)
                     or tr('未发现缺失话题提示；质量结果仍以原报告为准。'))
        duration = max(0, state["end_ns"] - state["start_ns"])
        # Decoding all streams is atomic but may lag the requested playhead.
        # The displayed timeline and both views describe the published snapshot.
        display_ns = snapshot["time_ns"] if snapshot.get("frames") else state["time_ns"]
        offset = max(0, display_ns - state["start_ns"])
        if not self.timeline.isSliderDown():
            self.timeline.setValue(min(10000, offset * 10000 // duration) if duration else 0)
        self.time_label.setText(f"{self._duration(offset)} / {self._duration(duration)}")
        if status["state"] != "ready" or not frame or frame.get("topic") != state["topic"]:
            if status["state"] in {"error", "waiting"} or not frame:
                self._clear_sample()
            return
        frame_signature = (frame["topic"], frame.get("sample_ns"), repr(frame.get("warnings", [])))
        if frame_signature != self._frame_signature:
            device = frame.get("device")
            has_preview = isinstance(device, dict) and device.get("kind") in {"camera", "tracker", "glove", "tactile"}
            if not has_preview or (self.preview is not None and self.preview.kind != device["kind"]):
                self._clear_sample()
            self._frame_signature = frame_signature
            encoded = json.dumps(frame.get("details", {}), ensure_ascii=False, indent=2)
            replace_text(self.details, encoded[:65536] + (tr('\n…内容过长，已截断。') if len(encoded) > 65536 else ""))
            if has_preview:
                device = deepcopy(device)
                device["status"] = "ready"  # availability of this historical sample, never LIVE
                preview = device.setdefault("preview", {})
                preview.pop("demo", None)
                preview.update(mode="replay", replay=True)
                kind = device["kind"]
                if kind not in self._previews:
                    self._previews[kind] = PreviewWidget(kind)
                    self._previews[kind].set_motion_enabled(False)
                    self.preview_stack.addWidget(self._previews[kind])
                self.preview = self._previews[kind]
                self.preview.set_device(device)
                self.preview_stack.setCurrentWidget(self.preview)
            else:
                self.empty.setText(tr('此话题的内容显示在“消息字段”中。') if frame.get("sample_ns") is not None
                                   else tr('所选时刻尚无可显示的样本。'))
        sample = frame.get("sample_ns")
        if sample is not None:
            self.sample_time.setText(tr('历史样本时间 {v0} · 距播放位置 {v1:.3f} 秒',
                                        v0=sample, v1=max(0, display_ns-sample)/1e9))
        else:
            self.sample_time.setText(tr('此话题在所选时刻没有样本。'))

    def hideEvent(self, event):
        self.timer.stop()
        if self.backend is not None:
            self.backend.pause()
        super().hideEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        if self.backend is not None and not self._closed and self.backend.timeline().get("path"):
            self.timer.start()
            self.refresh()

    def closeEvent(self, event):
        if not self._closed:
            self._closed = True
            self.timer.stop()
            if self.backend is not None:
                self.backend.close()
            self.spatial.close()
            self.scene_backend.clear()
        super().closeEvent(event)
